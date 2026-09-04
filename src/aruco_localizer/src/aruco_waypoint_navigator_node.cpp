#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>

using namespace std::chrono_literals;

struct MarkerPoint {
    int id;
    double x;
    double y;
};

class ArucoWaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Trigger = std_srvs::srv::Trigger;

    ArucoWaypointNavigator()
    : Node("aruco_waypoint_navigator"),
      route_active_(false),
      nav_goal_active_(false),
      localization_fresh_(false),
      imu_valid_(false),
      last_marker_id_(-1),
      step_index_(0),
      snapshot_counter_(0),
      stable_rotation_cycles_(0),
      pending_patrol_leg_(Direction::NONE)
    {
        declareParameters();
        loadMarkers();
        initSessionDirectory();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/navigate_to_pose");
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_visual_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/aruco_corridor_route", 10);

        auto sensor_qos = rclcpp::SensorDataQoS();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, sensor_qos,
            std::bind(&ArucoWaypointNavigator::imuCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", sensor_qos,
            std::bind(&ArucoWaypointNavigator::imageCallback, this, std::placeholders::_1));

        localization_fresh_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aruco/localization_fresh", 10,
            std::bind(&ArucoWaypointNavigator::localizationFreshCallback,
                this, std::placeholders::_1));

        marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/aruco/last_marker_id", 10,
            std::bind(&ArucoWaypointNavigator::markerIdCallback,
                this, std::placeholders::_1));

        nav_to_0_service_ = this->create_service<Trigger>(
            "/navigate_to_marker_0",
            std::bind(&ArucoWaypointNavigator::handleNavTo0,
                this, std::placeholders::_1, std::placeholders::_2));

        nav_to_5_service_ = this->create_service<Trigger>(
            "/navigate_to_marker_5",
            std::bind(&ArucoWaypointNavigator::handleNavTo5,
                this, std::placeholders::_1, std::placeholders::_2));

        start_patrol_service_ = this->create_service<Trigger>(
            "/start_marker_patrol",
            std::bind(&ArucoWaypointNavigator::handleStartPatrol,
                this, std::placeholders::_1, std::placeholders::_2));

        stop_patrol_service_ = this->create_service<Trigger>(
            "/stop_marker_patrol",
            std::bind(&ArucoWaypointNavigator::handleStop,
                this, std::placeholders::_1, std::placeholders::_2));

        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&ArucoWaypointNavigator::controlLoop, this));
        visual_timer_ = this->create_wall_timer(
            1s, std::bind(&ArucoWaypointNavigator::publishRouteVisual, this));

        logBlackboxEvent("NODE_INIT", "Aruco Corridor Navigator with Blackbox Diagnostic Logging Initialized.");
        RCLCPP_INFO(this->get_logger(),
            "Aruco Corridor Navigator Initialized. Blackbox Logger Active in /home/user/turtlebot3_ws/nav_debug/");
    }

private:
    enum class StepType { NAVIGATE, ROTATE, WAIT_FOR_MARKER };
    enum class Direction { NONE, TO_ZERO, TO_FIVE };

    struct RouteStep {
        StepType type;
        geometry_msgs::msg::PoseStamped pose;
        double rotation_delta{0.0};
        int marker_id{-1};
        std::string description;
    };

    static double normalizeAngle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void declareParameters() {
        this->declare_parameter<std::string>("marker_yaml_path",
            "/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map_markers.yaml");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");
        this->declare_parameter<double>("max_rotation_speed", 0.40);
        this->declare_parameter<double>("min_rotation_speed", 0.12);
        this->declare_parameter<double>("rotation_kp", 1.4);
        this->declare_parameter<double>("rotation_tolerance", 0.05); // 약 2.8도
        this->declare_parameter<double>("rotation_timeout", 10.0);
        this->declare_parameter<double>("marker_wait_timeout", 3.0);

        marker_yaml_path_ = this->get_parameter("marker_yaml_path").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        max_rotation_speed_ = this->get_parameter("max_rotation_speed").as_double();
        min_rotation_speed_ = this->get_parameter("min_rotation_speed").as_double();
        rotation_kp_ = this->get_parameter("rotation_kp").as_double();
        rotation_tolerance_ = this->get_parameter("rotation_tolerance").as_double();
        rotation_timeout_ = this->get_parameter("rotation_timeout").as_double();
        marker_wait_timeout_ = this->get_parameter("marker_wait_timeout").as_double();
    }

    bool loadMarkers() {
        try {
            const YAML::Node config = YAML::LoadFile(marker_yaml_path_);
            for (const auto& marker : config["markers"]) {
                const int id = marker["id"].as<int>();
                markers_[id] = {id, marker["x"].as<double>(), marker["y"].as<double>()};
            }
            return markers_.count(0) && markers_.count(1) && markers_.count(2) &&
                   markers_.count(3) && markers_.count(4) && markers_.count(5);
        } catch (const std::exception& exception) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load marker YAML: %s", exception.what());
            return false;
        }
    }

    void initSessionDirectory() {
        auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm_buf;
        localtime_r(&now_t, &tm_buf);
        char buf[64];
        std::strftime(buf, sizeof(buf), "run_%Y%m%d_%H%M%S", &tm_buf);
        session_name_ = std::string(buf);
        session_dir_ = "/home/user/turtlebot3_ws/nav_debug/" + session_name_;
        try {
            std::filesystem::create_directories(session_dir_);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to create session dir %s: %s", session_dir_.c_str(), e.what());
        }
    }

    // 블랙박스 진단 로그 및 카메라 스냅샷 저장
    void logBlackboxEvent(const std::string& event_tag, const std::string& details) {
        snapshot_counter_++;
        double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
        getRobotPoseWithYaw(cur_x, cur_y, cur_yaw);

        auto now = this->now();
        std::stringstream ss;
        ss << "[" << std::fixed << std::setprecision(3) << now.seconds() << "] "
           << "[" << event_tag << "] "
           << "Pose(map): (" << cur_x << ", " << cur_y << ", " << (cur_yaw * 180.0 / M_PI) << "°) | "
           << "IMU_Yaw: " << (current_imu_yaw_ * 180.0 / M_PI) << "° | "
           << "ArucoLock: " << (localization_fresh_ ? "YES" : "NO") << " (ID: " << last_marker_id_ << ") | "
           << details;

        std::string log_line = ss.str();
        RCLCPP_INFO(this->get_logger(), "%s", log_line.c_str());

        // 세션 디렉터리 및 최신 모니터링 로그 파일 저장
        std::vector<std::string> log_paths = {
            session_dir_ + "/navigation.log",
            "/home/user/turtlebot3_ws/nav_debug/navigation.log",
            "/home/user/.gemini/antigravity-ide/brain/ea338eee-0179-45cc-ac2b-5af60250a7da/scratch/nav_log.txt"
        };
        for (const auto& path : log_paths) {
            if (path.empty()) continue;
            std::ofstream ofs(path, std::ios::app);
            if (ofs.is_open()) {
                ofs << log_line << std::endl;
            }
        }

        // 이미지 스냅샷 저장
        if (!latest_image_.empty()) {
            std::stringstream fname;
            fname << "snap_" << std::setw(3) << std::setfill('0') << snapshot_counter_ << "_" << event_tag << ".jpg";
            std::string img_path_session = session_dir_ + "/" + fname.str();
            std::string img_path_global = "/home/user/turtlebot3_ws/nav_debug/" + fname.str();
            std::string img_path2 = "/home/user/.gemini/antigravity-ide/brain/ea338eee-0179-45cc-ac2b-5af60250a7da/scratch/" + fname.str();
            if (!session_dir_.empty()) {
                cv::imwrite(img_path_session, latest_image_);
            }
            cv::imwrite(img_path_global, latest_image_);
            cv::imwrite(img_path2, latest_image_);
        }
    }

    geometry_msgs::msg::PoseStamped markerPose(int marker_id, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.header.stamp = this->now();
        const auto& marker = markers_.at(marker_id);
        pose.pose.position.x = marker.x;
        pose.pose.position.y = marker.y;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(quaternion);
        return pose;
    }

    void addNavigationStep(int marker_id, double yaw, const std::string& description) {
        RouteStep step;
        step.type = StepType::NAVIGATE;
        step.pose = markerPose(marker_id, yaw);
        step.description = description;
        route_steps_.push_back(step);
    }

    void addRotationStep(double delta, const std::string& description) {
        RouteStep step;
        step.type = StepType::ROTATE;
        step.rotation_delta = delta;
        step.description = description;
        route_steps_.push_back(step);
    }

    void addMarkerStep(int marker_id, const std::string& description) {
        RouteStep step;
        step.type = StepType::WAIT_FOR_MARKER;
        step.marker_id = marker_id;
        step.description = description;
        route_steps_.push_back(step);
    }

    void buildRoute(Direction direction) {
        route_steps_.clear();
        step_index_ = 0;
        if (direction == Direction::TO_ZERO) {
            // 1. 상단 복도: 5번 ➔ 4번 마커 정중앙 도달
            addNavigationStep(4, M_PI, "Drive straight to Marker 4 (top hallway)");
            // 2. 4번 마커에서 즉시 90도 좌회전 (IMU 피벗)
            addRotationStep(M_PI_2, "In-place turn 90 deg at Marker 4 (facing Marker 3)");
            // 3. 3번 마커 확인 (회전 완료 후 시야 확보)
            addMarkerStep(3, "Verify Marker 3 in view");
            // 4. 중앙 세로 복도: 4번 ➔ 1번 마커 정중앙 도달 (3번, 2번 통과)
            addNavigationStep(1, -M_PI_2, "Drive straight to Marker 1 (vertical hallway via 3, 2)");
            // 5. 1번 마커에서 즉시 90도 좌회전 (IMU 피벗)
            addRotationStep(M_PI_2, "In-place turn 90 deg at Marker 1 (facing Marker 0)");
            // 6. 0번 마커 확인
            addMarkerStep(0, "Verify Marker 0 in view");
            // 7. 하단 복도: 1번 ➔ 0번 마커 도착
            addNavigationStep(0, 0.0, "Drive straight to Marker 0 (bottom hallway)");
        } else {
            addNavigationStep(1, M_PI, "Drive straight to Marker 1 (bottom hallway)");
            addRotationStep(-M_PI_2, "In-place turn 90 deg at Marker 1 (facing Marker 2)");
            addMarkerStep(2, "Verify Marker 2 in view");
            addNavigationStep(4, M_PI_2, "Drive straight to Marker 4 (vertical hallway via 2, 3)");
            addRotationStep(-M_PI_2, "In-place turn 90 deg at Marker 4 (facing Marker 5)");
            addMarkerStep(5, "Verify Marker 5 in view");
            addNavigationStep(5, 0.0, "Drive straight to Marker 5 (top hallway)");
        }
    }

    bool beginRoute(Direction direction, Direction patrol_return, std::string& message) {
        if (route_active_) {
            message = "Another navigation task is already active.";
            return false;
        }

        double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
        bool has_pose = getRobotPoseWithYaw(cur_x, cur_y, cur_yaw);
        if (!has_pose) {
            message = "Cannot start: robot pose in map is not yet initialized (waiting for map->odom TF).";
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
            return false;
        }

        if (!imu_valid_) {
            message = "Cannot start: IMU orientation is unavailable.";
            return false;
        }

        if (!nav_action_client_->wait_for_action_server(3s)) {
            message = "Cannot start: Nav2 navigate_to_pose action is unavailable.";
            return false;
        }

        pending_patrol_leg_ = patrol_return;
        route_active_ = true;
        buildRoute(direction);

        logBlackboxEvent("ROUTE_START", direction == Direction::TO_ZERO ?
            "Starting route to Marker 0 (via 4, 3, 2, 1)" :
            "Starting route to Marker 5 (via 1, 2, 3, 4)");

        executeCurrentStep();
        message = direction == Direction::TO_ZERO ?
            "Started marker-verified route to marker 0." :
            "Started marker-verified route to marker 5.";
        return true;
    }

    void executeCurrentStep() {
        if (!route_active_) return;
        if (step_index_ >= route_steps_.size()) {
            if (pending_patrol_leg_ != Direction::NONE) {
                const Direction next_direction = pending_patrol_leg_;
                pending_patrol_leg_ = Direction::NONE;
                logBlackboxEvent("SHUTTLE_LOOP", "Reversing patrol shuttle direction.");
                buildRoute(next_direction);
                executeCurrentStep();
                return;
            }
            route_active_ = false;
            publishZeroVelocity();
            logBlackboxEvent("ROUTE_COMPLETE", "Corridor navigation completed successfully!");
            return;
        }

        RouteStep& step = route_steps_[step_index_];
        step_start_time_ = this->now();
        nav_trial_count_ = 0;

        std::stringstream ss;
        ss << "Starting Step " << (step_index_ + 1) << "/" << route_steps_.size() << ": " << step.description;
        logBlackboxEvent("STEP_START", ss.str());

        if (step.type == StepType::NAVIGATE) {
            sendNavigationGoal(step.pose);
        } else if (step.type == StepType::ROTATE) {
            rotation_target_yaw_ = normalizeAngle(current_imu_yaw_ + step.rotation_delta);
            stable_rotation_cycles_ = 0;
            std::stringstream rot_ss;
            rot_ss << "Target IMU Yaw: " << (rotation_target_yaw_ * 180.0 / M_PI) << "° (Delta: "
                   << (step.rotation_delta * 180.0 / M_PI) << "°)";
            logBlackboxEvent("ROTATE_INIT", rot_ss.str());
        }
    }

    void sendNavigationGoal(geometry_msgs::msg::PoseStamped pose) {
        pose.header.stamp = this->now();
        NavigateToPose::Goal goal;
        goal.pose = pose;

        const uint64_t goal_id = ++current_nav_goal_id_;
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback =
            [this, goal_id](const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
                this->goalResponseCallback(goal_handle, goal_id);
            };
        options.result_callback =
            [this, goal_id](const GoalHandleNavigateToPose::WrappedResult& result) {
                this->resultCallback(result, goal_id);
            };

        nav_goal_active_ = true;
        nav_action_client_->async_send_goal(goal, options);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle, uint64_t goal_id) {
        if (!route_active_ || goal_id != current_nav_goal_id_) {
            if (goal_handle) {
                nav_action_client_->async_cancel_goal(goal_handle);
            }
            return;
        }

        if (!goal_handle) {
            nav_goal_active_ = false;
            logBlackboxEvent("NAV_REJECTED", "Nav2 server rejected navigation goal (ID: " + std::to_string(goal_id) + ").");
            abortRoute("Nav2 rejected navigation goal.");
            return;
        }
        active_goal_handle_ = goal_handle;
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result, uint64_t goal_id) {
        // 이미 지나간 스텝이거나 취소된 이전 골의 지연 응답은 무시
        if (goal_id != current_nav_goal_id_) {
            RCLCPP_DEBUG(this->get_logger(), "Ignored stale nav result for goal %lu (current: %lu)",
                goal_id, current_nav_goal_id_);
            return;
        }

        nav_goal_active_ = false;
        active_goal_handle_.reset();
        if (!route_active_) return;

        // 정상 취소된 경우 재시도하지 않고 조용히 종료
        if (result.code == rclcpp_action::ResultCode::CANCELED) {
            logBlackboxEvent("NAV_CANCELED", "Navigation goal was canceled (ID: " + std::to_string(goal_id) + ").");
            return;
        }

        // 현재 스텝이 NAVIGATE가 아니면 새 골을 전송하지 않음
        if (step_index_ >= route_steps_.size() || route_steps_[step_index_].type != StepType::NAVIGATE) {
            logBlackboxEvent("NAV_IGNORED", "Result received for goal " + std::to_string(goal_id) + " but current step is not NAVIGATE.");
            return;
        }

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
            getRobotPoseWithYaw(cur_x, cur_y, cur_yaw);
            double dist = std::hypot(cur_x - route_steps_[step_index_].pose.pose.position.x,
                                     cur_y - route_steps_[step_index_].pose.pose.position.y);
            if (dist < 0.35) {
                logBlackboxEvent("NAV_RECOVERY_ARRIVED",
                    "Nav2 finished near waypoint (dist=" + std::to_string(dist) + "m). Proceeding.");
                step_index_++;
                executeCurrentStep();
                return;
            }
            if (nav_trial_count_ < 2) {
                nav_trial_count_++;
                logBlackboxEvent("NAV_RETRY",
                    "Retrying Nav2 goal (trial " + std::to_string(nav_trial_count_) + "/2) after 500ms cooldown...");
                // 30ms 만에 카운트를 모두 소진하고 즉시 폭파되는 것을 방지하기 위해 500ms 딜레이 후 재시도
                retry_timer_ = this->create_wall_timer(
                    500ms, [this]() {
                        retry_timer_->cancel();
                        if (route_active_ && step_index_ < route_steps_.size() &&
                            route_steps_[step_index_].type == StepType::NAVIGATE) {
                            sendNavigationGoal(route_steps_[step_index_].pose);
                        }
                    });
                return;
            }
            logBlackboxEvent("NAV_FAILED", "Nav2 straight navigation failed after retries.");
            abortRoute("Nav2 failed straight navigation.");
            return;
        }

        logBlackboxEvent("NAV_ARRIVED", "Arrived at target waypoint. Moving to next step.");
        step_index_++;
        executeCurrentStep();
    }

    void controlLoop() {
        if (!route_active_ || step_index_ >= route_steps_.size()) return;
        const RouteStep& step = route_steps_[step_index_];

        if (step.type == StepType::NAVIGATE) {
            double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
            getRobotPoseWithYaw(cur_x, cur_y, cur_yaw);
            double dist = std::hypot(cur_x - step.pose.pose.position.x,
                                     cur_y - step.pose.pose.position.y);
            double elapsed = (this->now() - step_start_time_).seconds();

            // 목표 근접 (25cm 이내) 또는 2.0초 경과 후 35cm 이내 도달 시 지체 없이 다음 단계(회전 등)로 전환
            if (dist <= 0.25 || (elapsed > 2.0 && dist <= 0.35)) {
                logBlackboxEvent("NAV_PROXIMITY_ARRIVED",
                    "Arrived near waypoint (dist=" + std::to_string(dist) + "m, elapsed=" + std::to_string(elapsed) + "s). Advancing to next step.");
                // 이전 골의 비동기 취소 및 콜백 무효화
                current_nav_goal_id_++;
                if (active_goal_handle_) {
                    nav_action_client_->async_cancel_goal(active_goal_handle_);
                    active_goal_handle_.reset();
                }
                nav_goal_active_ = false;
                publishZeroVelocity();
                step_index_++;
                executeCurrentStep();
                return;
            }
            return;
        }

        if (step.type == StepType::WAIT_FOR_MARKER) {
            const bool received_after_step = last_marker_time_ >= step_start_time_;
            if ((localization_fresh_ && received_after_step && last_marker_id_ == step.marker_id) ||
                (last_marker_id_ == step.marker_id && (this->now() - last_marker_time_).seconds() < 2.0)) {
                logBlackboxEvent("MARKER_LOCKED", "Successfully verified ArUco marker " + std::to_string(step.marker_id));
                step_index_++;
                executeCurrentStep();
                return;
            }
            if ((this->now() - step_start_time_).seconds() > marker_wait_timeout_) {
                logBlackboxEvent("MARKER_TIMEOUT", "Marker " + std::to_string(step.marker_id) + " wait timeout, proceeding anyway.");
                step_index_++;
                executeCurrentStep();
            }
            return;
        }

        if (step.type == StepType::ROTATE) {
            if (!imu_valid_) {
                abortRoute("IMU data was lost during rotation.");
                return;
            }
            if ((this->now() - step_start_time_).seconds() > rotation_timeout_) {
                logBlackboxEvent("ROTATE_TIMEOUT", "IMU rotation timed out, proceeding.");
                step_index_++;
                executeCurrentStep();
                return;
            }

            const double yaw_error = normalizeAngle(rotation_target_yaw_ - current_imu_yaw_);
            if (std::abs(yaw_error) <= rotation_tolerance_) {
                publishZeroVelocity();
                stable_rotation_cycles_++;
                if (stable_rotation_cycles_ >= 3) {
                    logBlackboxEvent("ROTATE_DONE", "In-place 90 deg rotation completed stably.");
                    step_index_++;
                    executeCurrentStep();
                }
                return;
            }

            stable_rotation_cycles_ = 0;
            const double speed = std::clamp(rotation_kp_ * std::abs(yaw_error),
                min_rotation_speed_, max_rotation_speed_);
            geometry_msgs::msg::Twist command;
            command.angular.z = std::copysign(speed, yaw_error);
            cmd_vel_pub_->publish(command);
        }
    }

    void abortRoute(const std::string& reason) {
        if (!route_active_) return;
        logBlackboxEvent("ROUTE_ABORTED", reason);
        route_active_ = false;
        pending_patrol_leg_ = Direction::NONE;
        current_nav_goal_id_++; // 모든 대기 중인 네비게이션 골 콜백 무효화
        if (retry_timer_) {
            retry_timer_->cancel();
        }
        publishZeroVelocity();
        if (active_goal_handle_) {
            nav_action_client_->async_cancel_goal(active_goal_handle_);
            active_goal_handle_.reset();
        } else if (nav_goal_active_) {
            nav_action_client_->async_cancel_all_goals();
        }
        nav_goal_active_ = false;
    }

    void publishZeroVelocity() {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
        const auto& orientation = message->orientation;
        const double norm = std::sqrt(orientation.x * orientation.x +
            orientation.y * orientation.y + orientation.z * orientation.z +
            orientation.w * orientation.w);
        if (norm < 0.5) return;
        current_imu_yaw_ = tf2::getYaw(orientation);
        imu_valid_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr message) {
        try {
            latest_image_ = cv_bridge::toCvCopy(message, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            // 무시
        }
    }

    void localizationFreshCallback(const std_msgs::msg::Bool::SharedPtr message) {
        localization_fresh_ = message->data;
    }

    void markerIdCallback(const std_msgs::msg::Int32::SharedPtr message) {
        last_marker_id_ = message->data;
        last_marker_time_ = this->now();
    }

    bool getRobotPoseWithYaw(double& x, double& y, double& yaw) {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            yaw = tf2::getYaw(transform.transform.rotation);
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    void handleNavTo0(const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        response->success = beginRoute(Direction::TO_ZERO, Direction::NONE, response->message);
    }

    void handleNavTo5(const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        response->success = beginRoute(Direction::TO_FIVE, Direction::NONE, response->message);
    }

    void handleStartPatrol(const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (!getRobotPoseWithYaw(x, y, yaw)) {
            response->success = false;
            response->message = "Cannot start patrol without a current map pose.";
            return;
        }
        (void)x;
        const Direction first = y > 1.0 ? Direction::TO_ZERO : Direction::TO_FIVE;
        const Direction second = first == Direction::TO_ZERO ?
            Direction::TO_FIVE : Direction::TO_ZERO;
        response->success = beginRoute(first, second, response->message);
    }

    void handleStop(const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        if (!route_active_) {
            response->success = false;
            response->message = "No active navigation task.";
            return;
        }
        abortRoute("Stop service requested.");
        response->success = true;
        response->message = "Navigation stop requested.";
    }

    void publishRouteVisual() {
        if (markers_.size() < 6) return;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = map_frame_;
        line.header.stamp = this->now();
        line.ns = "aruco_corridor_route";
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.04;
        line.color.r = 0.1F;
        line.color.g = 0.9F;
        line.color.b = 0.3F;
        line.color.a = 0.9F;
        for (const int marker_id : {5, 4, 3, 2, 1, 0}) {
            geometry_msgs::msg::Point point;
            point.x = markers_.at(marker_id).x;
            point.y = markers_.at(marker_id).y;
            line.points.push_back(point);
        }
        path_visual_pub_->publish(line);
    }

    std::string marker_yaml_path_;
    std::string map_frame_;
    std::string base_frame_;
    std::string imu_topic_;
    double max_rotation_speed_;
    double min_rotation_speed_;
    double rotation_kp_;
    double rotation_tolerance_;
    double rotation_timeout_;
    double marker_wait_timeout_;
    std::unordered_map<int, MarkerPoint> markers_;
    std::vector<RouteStep> route_steps_;
    bool route_active_;
    bool nav_goal_active_;
    bool localization_fresh_;
    bool imu_valid_;
    int last_marker_id_;
    size_t step_index_;
    int snapshot_counter_;
    int stable_rotation_cycles_;
    int nav_trial_count_{0};
    uint64_t current_nav_goal_id_{0};
    std::string session_name_;
    std::string session_dir_;
    double current_imu_yaw_{0.0};
    double rotation_target_yaw_{0.0};
    Direction pending_patrol_leg_;
    rclcpp::Time last_marker_time_;
    rclcpp::Time step_start_time_;
    cv::Mat latest_image_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_visual_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_fresh_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr marker_id_sub_;
    rclcpp::Service<Trigger>::SharedPtr nav_to_0_service_;
    rclcpp::Service<Trigger>::SharedPtr nav_to_5_service_;
    rclcpp::Service<Trigger>::SharedPtr start_patrol_service_;
    rclcpp::Service<Trigger>::SharedPtr stop_patrol_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr visual_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoWaypointNavigator>());
    rclcpp::shutdown();
    return 0;
}
