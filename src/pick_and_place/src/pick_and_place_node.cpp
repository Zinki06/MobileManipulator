#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cleanup_interfaces/srv/evaluate_grasp.hpp>
#include <cleanup_interfaces/srv/place_object.hpp>
#include <cleanup_interfaces/srv/execute_pick.hpp>
#include <cleanup_interfaces/msg/gripper_hardware_state.hpp>
#include "pick_and_place/grasp_kinematics.hpp"
#include "pick_and_place/candidate_grasp.hpp"
#include "pick_and_place/place_kinematics.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <algorithm>
#include <limits>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

class PickAndPlaceActionNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand;
    using Trigger = std_srvs::srv::Trigger;
    using EvaluateGrasp = cleanup_interfaces::srv::EvaluateGrasp;
    using PlaceObject = cleanup_interfaces::srv::PlaceObject;
    using ExecutePick = cleanup_interfaces::srv::ExecutePick;
    using PickResult = ExecutePick::Response;

    PickAndPlaceActionNode()
    : Node("pick_and_place_action_node"),
      has_target_(false),
      target_received_time_(0, 0, RCL_ROS_TIME)
    {
        this->declare_parameter<double>("target_max_age", 6.0);
        this->declare_parameter<std::string>("target_topic", "/object_centroid");
        require_hardware_state_ = declare_parameter<bool>("require_gripper_hardware_state", false);
        hardware_sub_ = create_subscription<cleanup_interfaces::msg::GripperHardwareState>(
            "/manipulation/gripper_hardware_state", rclcpp::QoS(1).transient_local(),
            [this](cleanup_interfaces::msg::GripperHardwareState::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                hardware_state_ = *msg;
            });
        reached_joint_tolerance_ = declare_parameter<double>("reached_joint_tolerance", 0.0);
        if (!std::isfinite(reached_joint_tolerance_) || reached_joint_tolerance_ < 0.0 ||
            reached_joint_tolerance_ > 0.015) {
            throw std::invalid_argument("Reached joint tolerance must be within 0..0.015 rad");
        }
        insertion_depth_ = this->declare_parameter<double>("grasp_insertion_depth", 0.012);
        floor_height_ = this->declare_parameter<double>("floor_height", 0.0);
        use_candidate_grasp_ = declare_parameter<bool>("use_candidate_grasp", false);
        forward_offset_ = declare_parameter<double>("grasp_forward_offset", 0.0);
        min_body_depth_ = declare_parameter<double>("grasp_min_body_depth", 0.006);
        approach_error_margin_ = declare_parameter<double>("approach_error_margin", 0.02);
        approach_goal_tolerance_ = declare_parameter<double>("approach_goal_tolerance", 0.02);
        if (!std::isfinite(min_body_depth_) || min_body_depth_ < 0.004 ||
            min_body_depth_ > 0.020 || !std::isfinite(approach_error_margin_) ||
            approach_error_margin_ < 0.0 || approach_error_margin_ > 0.05 ||
            !std::isfinite(approach_goal_tolerance_) || approach_goal_tolerance_ < 0.0 ||
            approach_goal_tolerance_ > 0.05) {
            throw std::invalid_argument("Invalid grasp depth or approach error margin");
        }
        if (!std::isfinite(forward_offset_) || std::abs(forward_offset_) > 0.030) {
            throw std::invalid_argument("Grasp forward offset must be within 30mm");
        }
        RCLCPP_INFO(get_logger(),
            "GRASP_CONFIG strategy=%s forward_offset=%.3f min_body_depth=%.3f approach_error_margin=%.3f",
            use_candidate_grasp_ ? "candidate_body" : "legacy", forward_offset_,
            min_body_depth_, approach_error_margin_);
        if (!std::isfinite(insertion_depth_) || insertion_depth_ < 0.0 ||
            insertion_depth_ > 0.025 || !std::isfinite(floor_height_)) {
            throw std::invalid_argument("Invalid floor or grasp insertion depth");
        }
        target_max_age_ = this->get_parameter("target_max_age").as_double();
        if (!std::isfinite(target_max_age_) || target_max_age_ <= 0.0 || target_max_age_ > 10.0) {
            throw std::invalid_argument("target_max_age must be within 0..10 seconds");
        }
        const std::string target_topic =
            this->get_parameter("target_topic").as_string();
        carrying_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/cleanup/carrying", rclcpp::QoS(1).transient_local());
        carrying_timer_ = create_wall_timer(200ms, [this]() {publishCarrying();});
        phase_pub_ = create_publisher<std_msgs::msg::String>("/pick/events", 20);
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                    for (size_t j = 0; j < 4; ++j) {
                        if (msg->name[i] == "joint" + std::to_string(j + 1) &&
                            std::isfinite(msg->position[i])) {
                            std::lock_guard<std::mutex> lock(joint_mutex_);
                            arm_positions_[j] = msg->position[i];
                            arm_stamps_[j] = rclcpp::Time(msg->header.stamp).seconds();
                            arm_velocities_[j] = i < msg->velocity.size() ? msg->velocity[i] :
                                std::numeric_limits<double>::infinity();
                        }
                    }
                    if (msg->name[i] != "gripper_left_joint" ||
                        !std::isfinite(msg->position[i])) {continue;}
                    std::lock_guard<std::mutex> lock(joint_mutex_);
                    gripper_position_ = msg->position[i];
                    gripper_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
                }
                joint_feedback_.notify_all();
            });

        action_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // 1. Action Clients 생성
        arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory", action_callback_group_);
        gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/gripper_controller/gripper_cmd", action_callback_group_);

        // 2. TF 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 물체 맵 좌표 구독
        target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            target_topic, 10,
            std::bind(&PickAndPlaceActionNode::targetCallback, this, std::placeholders::_1));

        // 4. 파지 시퀀스 실행 및 안전 주차(Park) 서비스
        execute_service_ = this->create_service<Trigger>(
            "/execute_pick_and_place",
            std::bind(&PickAndPlaceActionNode::handleExecuteRequest, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            action_callback_group_);

        pick_service_ = create_service<ExecutePick>(
            "/cleanup/execute_pick",
            [this](const std::shared_ptr<ExecutePick::Request>,
                   std::shared_ptr<PickResult> response) {executePick(*response);},
            rmw_qos_profile_services_default, action_callback_group_);
        prepare_gripper_service_ = create_service<Trigger>(
            "/cleanup/prepare_gripper",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> response) {
                std::unique_lock<std::mutex> lock(operation_mutex_, std::try_to_lock);
                if (!lock.owns_lock() || release_requested_ || action_fault_ || carrying_) {
                    response->message = "Preparation refused: manipulation active, faulted, or carrying";
                    return;
                }
                phase("GRIPPER_PREFLIGHT", "Verify opening before navigation");
                response->success = sendGripperGoal(0.019);
                response->message = response->success ? "Gripper opening verified" :
                    "GRIPPER_FAULT: " + gripper_failure_;
            }, rmw_qos_profile_services_default, action_callback_group_);

        park_service_ = this->create_service<Trigger>(
            "/park_arm",
            std::bind(&PickAndPlaceActionNode::handleParkRequest, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            action_callback_group_);

        open_gripper_service_ = this->create_service<Trigger>(
            "/open_gripper",
            std::bind(&PickAndPlaceActionNode::handleOpenGripperRequest, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            action_callback_group_);

        stop_manipulation_service_ = create_service<Trigger>(
            "/cleanup/stop_manipulation",
            std::bind(&PickAndPlaceActionNode::handleStopManipulation, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, action_callback_group_);

        close_gripper_service_ = this->create_service<Trigger>(
            "/close_gripper",
            std::bind(&PickAndPlaceActionNode::handleCloseGripperRequest, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            action_callback_group_);

        place_service_ = this->create_service<PlaceObject>(
            "/cleanup/place_object",
            std::bind(&PickAndPlaceActionNode::handlePlaceRequest, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, action_callback_group_);

        evaluate_grasp_service_ = this->create_service<EvaluateGrasp>(
            "/cleanup/evaluate_grasp",
            std::bind(&PickAndPlaceActionNode::evaluateGrasp, this,
                      std::placeholders::_1, std::placeholders::_2));
        observe_service_ = this->create_service<Trigger>(
            "/observe_floor", [this](const std::shared_ptr<Trigger::Request>,
                std::shared_ptr<Trigger::Response> response) {
                std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
                if (!operation_lock.owns_lock() || release_requested_ || action_fault_) {
                    response->message = "Manipulation busy or finishing";
                    return;
                }
                bool expected = false;
                if (!sequence_in_progress_.compare_exchange_strong(expected, true)) {
                    response->message = "Pick sequence active";
                    return;
                }
                auto pose = initialPoseJoints();
                pose[3] += 0.20;  // Temporary camera look-down; initial/park pose is unchanged.
                try {
                    response->success = sendArmGoal(pose, 1.5);
                    response->message = "Temporary floor observation pose";
                } catch (const std::exception & error) {
                    response->message = error.what();
                }
                sequence_in_progress_ = false;
            }, rmw_qos_profile_services_default, action_callback_group_);

        // 5. 런처 실행 시 로봇팔 초기 자세 자동 이동 비동기 스레드 시작
        if (declare_parameter<bool>("startup_park", true)) {
            init_thread_ = std::thread(&PickAndPlaceActionNode::initializePoseOnStartup, this);
        }

        RCLCPP_INFO(this->get_logger(), "Pick & Place Action Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "Services ready: /execute_pick_and_place, /park_arm, /open_gripper, /close_gripper");
    }

    ~PickAndPlaceActionNode() override {
        if (init_thread_.joinable()) {
            init_thread_.join();
        }
    }

private:
    std::vector<double> freshArmPositions() {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        std::vector<double> start(4);
        for (size_t j = 0; j < 4; ++j) {
            const double age = now().seconds() - arm_stamps_[j];
            if (age < 0.0 || age > 0.25 || !std::isfinite(arm_positions_[j])) {
                throw std::runtime_error("Fresh arm feedback required for candidate planning");
            }
            start[j] = arm_positions_[j];
        }
        return start;
    }

    bool planAutomaticGrasp(double x, double y, double z, double floor,
                             pick_and_place::GraspPlan & plan,
                             pick_and_place::CandidateDiagnostics * diagnostics = nullptr) {
        return use_candidate_grasp_ ? pick_and_place::planCandidateExecution(
            x + forward_offset_, y, z, floor, freshArmPositions(), plan, diagnostics,
            min_body_depth_) :
            pick_and_place::planGrasp(x + forward_offset_, y, z, floor, insertion_depth_, plan);
    }

    void evaluateGrasp(const std::shared_ptr<EvaluateGrasp::Request> request,
                       std::shared_ptr<EvaluateGrasp::Response> response) {
        if (request->require_candidate && !use_candidate_grasp_) {
            response->message = "Candidate grasp required but executor is configured for legacy";
            return;
        }
        const auto& target = request->target;
        if (target.header.frame_id != "map" || !std::isfinite(target.point.x) ||
            !std::isfinite(target.point.y) || !std::isfinite(target.point.z)) {
            response->message = "Expected a finite map-frame target";
            return;
        }
        try {
            tf2::Transform map_base, base_arm;
            tf2::fromMsg(tf_buffer_->lookupTransform("map", "base_link",
                tf2::TimePointZero, tf2::durationFromSec(0.2)).transform, map_base);
            tf2::fromMsg(tf_buffer_->lookupTransform("base_link", "link1",
                tf2::TimePointZero, tf2::durationFromSec(0.2)).transform, base_arm);
            const tf2::Vector3 point(target.point.x, target.point.y, target.point.z);
            const auto local = (map_base * base_arm).inverse() * point;
            response->success = true;
            const auto floor = (map_base * base_arm).inverse() *
                tf2::Vector3(point.x(), point.y(), floor_height_);
            pick_and_place::GraspPlan plan;
            pick_and_place::CandidateDiagnostics current_counts, approach_counts;
            response->reachable = planAutomaticGrasp(
                local.x(), local.y(), local.z(), floor.z(), plan, &current_counts);
            response->message = "link1 target x=" + std::to_string(local.x()) +
                ", y=" + std::to_string(local.y()) + ", z=" + std::to_string(local.z());
            response->message += "; executor_strategy=" + std::string(
                use_candidate_grasp_ ? "candidate_body" : "legacy") +
                "; forward_offset=" + std::to_string(forward_offset_) +
                "; surface_height=" + std::to_string(local.z() - floor.z()) +
                "; min_body_depth=" + std::to_string(min_body_depth_);
            if (response->reachable) {
                response->message += "; grasp pitch=" + std::to_string(plan.pitch_degrees) +
                    "; insertion=" + std::to_string(plan.insertion_depth);
                return;
            }
            if (use_candidate_grasp_) {
                response->message += "; current_rejections={" +
                    pick_and_place::candidateRejectionSummary(current_counts) + "}";
            }
            const double yaw = std::atan2(point.y() - map_base.getOrigin().y(),
                                          point.x() - map_base.getOrigin().x());
            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, yaw);
            // Retain clearance in front of the base; Nav2 still collision-checks
            // every proposed pose/path. The arm offset comes from actual TF.
            for (double standoff = 0.30; standoff >= 0.179; standoff -= 0.02) {
                tf2::Transform proposed(orientation, tf2::Vector3(
                    point.x() - standoff * std::cos(yaw),
                    point.y() - standoff * std::sin(yaw), map_base.getOrigin().z()));
                // When the actual grasp is unreachable, do not retry a goal Nav2
                // would immediately call reached. Select the next closer pose.
                if ((proposed.getOrigin() - map_base.getOrigin()).length() <=
                        approach_goal_tolerance_ + 1e-6 &&
                    map_base.getRotation().angleShortestPath(orientation) < 0.01) {continue;}
                const auto relative = (proposed * base_arm).inverse() * point;
                // Optional conservative margin. Arrival is always reobserved and
                // checked against actual geometry before any grasp is executed.
                const auto proposed_floor = (proposed * base_arm).inverse() *
                    tf2::Vector3(point.x(), point.y(), floor_height_);
                if (!planAutomaticGrasp(relative.x(), relative.y(), relative.z(),
                        proposed_floor.z(), plan, &approach_counts) ||
                    (approach_error_margin_ > 0.0 &&
                    !planAutomaticGrasp(relative.x() + approach_error_margin_,
                        relative.y(), relative.z(), proposed_floor.z(), plan,
                        &approach_counts))) {continue;}
                response->approach_available = true;
                response->approach_pose.header.frame_id = "map";
                response->approach_pose.header.stamp = this->now();
                response->approach_pose.pose.position.x = proposed.getOrigin().x();
                response->approach_pose.pose.position.y = proposed.getOrigin().y();
                response->approach_pose.pose.orientation = tf2::toMsg(orientation);
                response->message += "; feasible base standoff=" + std::to_string(standoff) +
                    "; approach_error_margin=" + std::to_string(approach_error_margin_);
                return;
            }
            response->message += "; no safe grasp within approach standoff limits";
            if (use_candidate_grasp_) {
                response->message += "; closest_approach_rejections={" +
                    pick_and_place::candidateRejectionSummary(approach_counts) + "}";
            }
        } catch (const std::exception& exception) {
            response->success = false;
            response->message = exception.what();
        }
    }

    static std::vector<double> initialPoseJoints() {
        return {0.0, -0.523, -0.523, 1.5707};
    }

    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (msg->header.frame_id.empty() || !std::isfinite(msg->point.x) ||
            !std::isfinite(msg->point.y) || !std::isfinite(msg->point.z)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /object_centroid without frame_id.");
            return;
        }
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_map_target_ = *msg;
        target_received_time_ = this->now();
        has_target_ = true;
    }

    void handleExecuteRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        PickResult result;
        executePick(result);
        response->success = result.success;
        response->message = result.message;
    }

    void executePick(PickResult & response) {
        response.result_code = PickResult::PRECONDITION_FAILED;
        response.holding_state = PickResult::HOLD_UNKNOWN;
        response.stage = "PRECHECK";
        std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
        if (!operation_lock.owns_lock() || release_requested_ || action_fault_ || carrying_) {
            response.message = "Manipulation busy, faulted, or already carrying";
            return;
        }
        bool expected = false;
        if (!sequence_in_progress_.compare_exchange_strong(expected, true)) {
            response.message = "A pick-and-place sequence is already in progress.";
            return;
        }

        try {
            response.success = executePickSequence(response);
            if (response.success) {response.result_code = PickResult::OK;}
        } catch (const std::exception & ex) {
            response.success = false;
            response.message = std::string("Unexpected pick-and-place error: ") + ex.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response.message.c_str());
        }
        if (!response.success && (release_requested_ || action_fault_)) {
            response.result_code = PickResult::INTERRUPTED;
        }
        response.holding_state = holding_state_;
        phase("PICK_RESULT", "stage=" + response.stage +
            "; code=" + std::to_string(response.result_code) +
            "; holding_state=" + std::to_string(response.holding_state) +
            "; close_started=" + std::to_string(response.close_started) +
            "; " + response.message);
        sequence_in_progress_ = false;
    }

    bool executePickSequence(PickResult & outcome) {
        auto & message = outcome.message;
        geometry_msgs::msg::PointStamped target_in_map;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (!has_target_) {
                message = "No pick target has been received.";
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                return false;
            }
            const double target_age = (this->now() - target_received_time_).seconds();
            const double observation_age =
                (this->now() - rclcpp::Time(latest_map_target_.header.stamp)).seconds();
            if (target_age < 0.0 || target_age > target_max_age_ || observation_age < 0.0 ||
                observation_age > target_max_age_) {
                has_target_ = false;
                message = "TARGET_EXPIRED: observation_age=" + std::to_string(observation_age) +
                    "s; receipt_age=" + std::to_string(target_age) +
                    "s; limit=" + std::to_string(target_max_age_) + "s";
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                return false;
            }
            target_in_map = latest_map_target_;
            phase("TARGET_ACCEPTED", "observation_age=" + std::to_string(observation_age) +
                "s; limit=" + std::to_string(target_max_age_) + "s");
        }

        if (!arm_action_client_->wait_for_action_server(2s)) {
            message = "Arm action server is unavailable.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }
        if (release_requested_) {return false;}
        if (!gripper_action_client_->wait_for_action_server(2s)) {
            message = "Gripper action server is unavailable.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // 1. 최신 TF(Time 0)를 조회하여 map/base -> link1 상대 좌표 변환
        geometry_msgs::msg::PointStamped target_in_arm;
        try {
            target_in_map.header.stamp = rclcpp::Time(0);
            target_in_arm = tf_buffer_->transform(target_in_map, "link1", tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Transform to link1 failed: %s", ex.what());
            message = std::string("TF transform to link1 failed: ") + ex.what();
            return false;
        }

        double rel_x = target_in_arm.point.x;
        double rel_y = target_in_arm.point.y;
        double rel_z = target_in_arm.point.z;

        RCLCPP_INFO(this->get_logger(),
            "============================================================");
        RCLCPP_INFO(this->get_logger(),
            "[TARGET (link1)] x = %.3f m, y = %.3f m, z = %.3f m", rel_x, rel_y, rel_z);

        auto floor_target = target_in_map;
        floor_target.point.z = floor_height_;
        const auto floor = tf_buffer_->transform(floor_target, "link1", tf2::durationFromSec(0.2));
        pick_and_place::GraspPlan plan;
        if (!planAutomaticGrasp(rel_x, rel_y, rel_z, floor.point.z, plan)) {
            message = "No safe pitch/insertion/lift plan; base must reapproach.";
            return false;
        }
        const auto & grasp_joints = plan.insertion.back();
        const auto & pre_grasp_joints = plan.pregrasp;
        phase("PLAN", "strategy=" + std::string(use_candidate_grasp_ ? "candidate_body" : "legacy") +
            "; forward_offset=" + std::to_string(forward_offset_) + "; pitch=" + std::to_string(plan.pitch_degrees) +
            "; insertion=" + std::to_string(plan.insertion_depth) +
            "; surface_to_body=" + std::to_string(plan.surface_to_body_depth) +
            "; target_link1=" + std::to_string(rel_x + forward_offset_) + "," +
            std::to_string(rel_y) + "," + std::to_string(rel_z));

        RCLCPP_INFO(this->get_logger(),
            "[IK Grasp Result] J1(Yaw): %.1f°, J2(Shoulder): %.1f°, J3(Elbow): %.1f°, J4(Wrist): %.1f°",
            grasp_joints[0] * 180.0 / M_PI,
            grasp_joints[1] * 180.0 / M_PI,
            grasp_joints[2] * 180.0 / M_PI,
            grasp_joints[3] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
            "============================================================");

        // === Step 1: 그리퍼 열기 (Open Gripper) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 1/7] Opening Gripper...");
        phase("OPEN");
        outcome.stage = "OPEN";
        outcome.result_code = PickResult::GRIPPER_FAILED;
        if (!sendGripperGoal(0.019)) {
            message = "Failed to open gripper: " + gripper_failure_;
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 2: 물체 상공 접근 (Pre-Grasp Pose) ===
        // Joint 1(Yaw)이 물체 방향으로 정렬되며 물체 상공으로 위치
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 2/7] Moving to Pre-Grasp Pose (Above target, Yaw: %.1f°)...",
            pre_grasp_joints[0] * 180.0 / M_PI);
        phase("PREGRASP");
        outcome.stage = "PREGRASP";
        outcome.result_code = PickResult::ARM_FAILED;
        if (!sendArmGoal(pre_grasp_joints, 2.5)) {
            message = "Failed to reach pre-grasp pose.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 3: 물체 위치로 수직 하강 (Grasp Pose) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 3/7] Descending to Surface Pose...");
        phase("DESCEND");
        outcome.stage = "DESCEND";
        if (!sendArmPath(plan.descent, use_candidate_grasp_ ? 4.0 : 1.8)) {
            message = "Failed to reach target grasp pose.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        if (!use_candidate_grasp_) {
        phase("INSERT", "depth=" + std::to_string(plan.insertion_depth));
        RCLCPP_INFO(this->get_logger(), "===> [Step 4/7] Inserting before closing...");
        if (!sendArmPath(plan.insertion, 1.0)) {
            message = "Insertion did not complete; gripper will not close.";
            return false;
        }
        }
        // === Step 4: 물체 파지 (Close Gripper) ===
        phase("CLOSE");
        outcome.stage = "CLOSE";
        outcome.result_code = PickResult::GRIPPER_FAILED;
        outcome.close_started = true;
        RCLCPP_INFO(this->get_logger(), "===> [Step 5/7] Closing Gripper (Grasping object)...");
        if (!sendGripperGoal(-0.010, 15.0)) {
            message = "Failed to close gripper: " + gripper_failure_;
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }
        // Controller success only acknowledges motion. Empty fingers reaching
        // their closed stop must never be transported as a collected object.
        if (!waitForGripper(true, -0.010)) {
            // Missing, moving, or stale feedback is not proof of an empty grasp.
            const bool empty = waitForGripper(false, -0.010);
            if (!empty) {
                message = "Grasp state unknown: " + gripper_failure_;
                return false;
            }
            holding_state_ = PickResult::HOLD_EMPTY;
            const bool lifted = sendArmPath(plan.lift, use_candidate_grasp_ ? 4.0 : 1.8);
            const bool parked = lifted && sendArmGoal(initialPoseJoints(), 2.5);
            outcome.result_code = parked ? PickResult::EMPTY_GRASP : PickResult::ARM_FAILED;
            message = parked ? "EMPTY_GRASP: no stable finger obstruction after closing" :
                "Grasp unverified and arm recovery failed; operator inspection required";
            phase("GRASP_UNVERIFIED", message);
            return false;
        }

        // === Step 5: 물체 수직 인양 (Lift Object) ===
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 6/7] Lifting Object (+%.1f cm)...", plan.lift_height * 100.0);
        phase("LIFT");
        outcome.stage = "LIFT";
        outcome.result_code = PickResult::ARM_FAILED;
        if (!sendArmPath(plan.lift, use_candidate_grasp_ ? 4.0 : 1.8)) {
            message = "Lift failed; holding state unknown.";
            return false;
        }
        const bool held_after_lift = waitForGripper(true, -0.010);

        // === Step 6: 안전 홈 포즈 복귀 (Return to Home) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 7/7] Returning to Home Pose...");
        const std::vector<double> home_joints = initialPoseJoints();
        phase("RETURN_HOME");
        outcome.stage = "RETURN_HOME";
        if (!sendArmGoal(home_joints, 2.5)) {
            message = "Object grasped, but the arm failed to return home.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        if (!held_after_lift || !waitForGripper(true, -0.010)) {
            const bool empty = waitForGripper(false, -0.010);
            holding_state_ = empty ? PickResult::HOLD_EMPTY : PickResult::HOLD_UNKNOWN;
            outcome.result_code = empty ? PickResult::EMPTY_GRASP : PickResult::GRIPPER_FAILED;
            message = empty ? "EMPTY_GRASP: finger obstruction lost during lift/return" :
                "Grasp state unknown after lift/return: " + gripper_failure_;
            phase("GRASP_UNVERIFIED", message);
            return false;
        }
        carrying_ = true;
        publishCarrying();
        phase("GRASP_HOLD_CONFIRMED", "Stable non-empty finger opening after lift and return");

        phase("SEQUENCE_COMPLETE", "Physical grasp still requires visual verification");
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            has_target_ = false;
        }
        message = "Pick-and-place sequence completed successfully.";
        return true;
    }

    // FollowJointTrajectory 액션 전송 및 완료 대기
    bool sendArmGoal(const std::vector<double>& joints, double duration_sec) {
        return sendArmPath({joints}, duration_sec);
    }

    void handlePlaceRequest(const std::shared_ptr<PlaceObject::Request> request,
                            std::shared_ptr<PlaceObject::Response> response) {
        std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
        if (!operation_lock.owns_lock() || release_requested_ || action_fault_) {
            response->message = "Manipulation busy or finishing";
            return;
        }
        bool expected = false;
        if (!sequence_in_progress_.compare_exchange_strong(expected, true)) {
            response->message = "Arm sequence already active";
            return;
        }
        try {
            const auto & target = request->floor_target;
            const double age = (now() - rclcpp::Time(target.header.stamp)).seconds();
            if (target.header.frame_id.empty() || age < 0.0 || age > 2.0 ||
                !std::isfinite(target.point.x) || !std::isfinite(target.point.y) ||
                !std::isfinite(target.point.z)) {
                throw std::runtime_error("Placement target invalid or stale");
            }
            auto latest = target;
            latest.header.stamp = rclcpp::Time(0);
            const auto local = tf_buffer_->transform(latest, "link1", tf2::durationFromSec(0.2));
            std::vector<double> start(4);
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                for (size_t j = 0; j < 4; ++j) {
                    const double joint_age = now().seconds() - arm_stamps_[j];
                    if (joint_age < 0.0 || joint_age > 0.25) {
                        throw std::runtime_error("Fresh arm feedback required for placement");
                    }
                    start[j] = arm_positions_[j];
                }
            }
            pick_and_place::PlacePlan plan;
            if (!pick_and_place::planPlace(local.point.x, local.point.y, local.point.z,
                                          request->release_height, start, plan)) {
                throw std::runtime_error("Collection point has no safe lowering/retreat path");
            }
            if (!waitForGripper(true, -0.010)) {
                throw std::runtime_error("Holding feedback missing before placement");
            }
            phase("PLACE_APPROACH", "pitch=" + std::to_string(plan.pitch_degrees));
            if (!sendArmGoal(plan.above, 3.0)) {
                throw std::runtime_error("Placement approach failed; gripper remains closed");
            }
            phase("PLACE_LOWER");
            if (!sendArmPath(plan.lower, 3.0) || !waitForGripper(true, -0.010)) {
                throw std::runtime_error("Placement lowering/hold failed; no release commanded");
            }
            if (!openForPlacement()) {
                throw std::runtime_error("Placement opening not confirmed");
            }
            response->released = true;
            phase("PLACE_RETREAT");
            if (!sendArmPath(plan.retreat, 2.5)) {
                throw std::runtime_error("Object released but arm retreat failed");
            }
            response->success = true;
            response->message = "Lowered at collection point, opened, and retracted";
            phase("PLACE_COMPLETE");
        } catch (const std::exception & error) {
            response->message = error.what();
            phase("PLACE_FAILED", response->message);
        }
        sequence_in_progress_ = false;
    }

    void publishCarrying() {
        std_msgs::msg::Bool msg;
        msg.data = carrying_.load();
        if (msg.data) {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            const double age = now().seconds() - gripper_stamp_;
            msg.data = hardwareFeedbackValid() && age >= 0.0 && age < 0.25 &&
                gripper_position_ > -0.0085 && gripper_position_ < 0.0175;
        }
        carrying_pub_->publish(msg);
    }

    void phase(const std::string & name, const std::string & detail = "") {
        std_msgs::msg::String event;
        event.data = name + "|" + detail;
        phase_pub_->publish(event);
        RCLCPP_INFO(get_logger(), "[PICK_PHASE] %s", event.data.c_str());
    }

    bool sendArmPath(const std::vector<std::vector<double>> & path, double duration_sec) {
        if (release_requested_ || action_fault_ || path.empty()) {return false;}
        if (!arm_action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Arm Action Server unavailable!");
            return false;
        }

        std::vector<double> current(4);
        bool stationary = true;
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            for (size_t j = 0; j < 4; ++j) {
                const double age = now().seconds() - arm_stamps_[j];
                if (age < 0.0 || age > 0.25) {
                    RCLCPP_ERROR(get_logger(), "Fresh arm feedback required for floor check");
                    return false;
                }
                current[j] = arm_positions_[j];
                stationary = stationary && std::isfinite(arm_velocities_[j]) &&
                    std::abs(arm_velocities_[j]) < 0.02;
            }
        }
        try {
            geometry_msgs::msg::PointStamped floor;
            floor.header.frame_id = "map";
            floor.point.z = floor_height_;
            const auto local = tf_buffer_->transform(floor, "link1", tf2::durationFromSec(0.2));
            if (!pick_and_place::fingerPathClear(current, path, local.point.z)) {
                RCLCPP_ERROR(get_logger(), "Arm path rejected: finger mesh floor clearance <6mm");
                return false;
            }
        } catch (const tf2::TransformException & error) {
            RCLCPP_ERROR(get_logger(), "Arm floor check TF unavailable: %s", error.what());
            return false;
        }
        if (stationary && path.size() == 1 && reached_joint_tolerance_ > 0.0) {
            double error = 0.0;
            for (size_t j = 0; j < current.size(); ++j) {
                error = std::max(error, std::abs(path.front()[j] - current[j]));
            }
            if (error <= reached_joint_tolerance_) {
                phase("ARM_ALREADY_REACHED", "Fresh encoders and floor clearance verified");
                return true;
            }
        }
        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4"};

        for (size_t i = 0; i < path.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = path[i];
            point.time_from_start = rclcpp::Duration::from_seconds(
                duration_sec * (i + 1) / path.size());
            goal.trajectory.points.push_back(point);
        }

        return runAction<FollowJointTrajectory>(arm_action_client_, goal, duration_sec + 3.0);
    }

    template<typename ActionT>
    bool runAction(const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
                   const typename ActionT::Goal & goal, double seconds,
                   bool opening = false) {
        last_action_error_.clear();
        if (action_fault_ || (release_requested_ && !opening)) {
            last_action_error_ = "Action fault or cancellation requested";
            return false;
        }
        auto abandoned = std::make_shared<std::atomic_bool>(false);
        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        options.goal_response_callback = [client, abandoned](auto handle) {
            // Even an acceptance arriving after our caller returned must be canceled.
            if (*abandoned && handle) {client->async_cancel_goal(handle);}
        };
        decltype(client->async_send_goal(goal, options)) accepted;
        {
            std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
            if (action_fault_ || (release_requested_ && !opening)) {
                last_action_error_ = "Action canceled before dispatch";
                return false;
            }
            accepted = client->async_send_goal(goal, options);
        }
        pending_cleanup_ = [client, accepted, abandoned]() mutable {
            *abandoned = true;
            if (accepted.wait_for(3s) != std::future_status::ready) {return false;}
            const auto handle = accepted.get();
            if (!handle) {return true;}
            auto result = client->async_get_result(handle);
            if (result.wait_for(0s) != std::future_status::ready) {
                client->async_cancel_goal(handle);
            }
            return result.wait_for(3s) == std::future_status::ready &&
                result.get().code != rclcpp_action::ResultCode::UNKNOWN;
        };
        if (accepted.wait_for(3s) != std::future_status::ready) {
            *abandoned = true;
            action_fault_ = true;
            phase("ACTION_FAULT", "Acceptance unknown; restart after inspection");
            last_action_error_ = "Goal acceptance timed out; execution unknown";
            // Cover the race where the callback completed just before abandonment.
            if (accepted.wait_for(0s) == std::future_status::ready) {cancelPending();}
            return false;
        }
        const auto handle = accepted.get();
        if (!handle) {
            pending_cleanup_ = {};
            last_action_error_ = "Controller rejected goal";
            return false;
        }
        auto result = client->async_get_result(handle);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while ((opening || !release_requested_) && std::chrono::steady_clock::now() < deadline &&
               result.wait_for(20ms) != std::future_status::ready) {}
        if ((!opening && release_requested_) || result.wait_for(0s) != std::future_status::ready) {
            last_action_error_ = release_requested_ ? "Action interrupted" : "Action result timed out";
            cancelPending();
            return false;
        }
        const auto wrapped = result.get();
        pending_cleanup_ = {};
        const bool valid = wrapped.result && actionResultValid(*wrapped.result);
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !valid) {
            last_action_error_ = "Controller result status=" +
                std::to_string(static_cast<int>(wrapped.code)) + "; invalid or unsuccessful result";
            return false;
        }
        return true;
    }

    static bool actionResultValid(const FollowJointTrajectory::Result & result) {
        return result.error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    }

    bool actionResultValid(const GripperCommand::Result & result) {
        phase("GRIPPER_RESULT", "position=" + std::to_string(result.position) +
            "; effort=" + std::to_string(result.effort) +
            "; stalled=" + std::to_string(result.stalled) +
            "; reached_goal=" + std::to_string(result.reached_goal));
        return std::isfinite(result.position) && (result.reached_goal || result.stalled);
    }

    bool cancelPending() {
        if (!pending_cleanup_) {return true;}
        bool stopped = false;
        try {stopped = pending_cleanup_();} catch (const std::exception &) {}
        if (stopped) {pending_cleanup_ = {};}
        else {
            action_fault_ = true;
            phase("ACTION_FAULT", "Cancellation unconfirmed; restart after inspection");
        }
        return stopped;
    }

    void handleStopManipulation(const std::shared_ptr<Trigger::Request>,
                                std::shared_ptr<Trigger::Response> response) {
        std::lock_guard<std::mutex> release_lock(release_mutex_);
        {
            std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
            release_requested_ = true;
        }
        std::unique_lock<std::mutex> operation_lock(operation_mutex_);
        response->success = cancelPending() && !action_fault_;
        response->message = response->success ? "Manipulation stopped; fingers unchanged" :
            "Manipulation fault; restart after inspection";
        release_requested_ = false;
    }

    bool waitForGripper(bool holding, double target, bool explicit_open = false) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        double stable_since = -1.0, last_stamp = -1.0, last_position = 0.0;
        int samples = 0;
        while (rclcpp::ok() && (!release_requested_ || explicit_open) &&
            std::chrono::steady_clock::now() < deadline) {
            double position, stamp;
            bool hardware_ok;
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                position = gripper_position_;
                stamp = gripper_stamp_;
                hardware_ok = hardwareFeedbackValid();
            }
            const double now = this->now().seconds();
            const bool fresh = hardware_ok && stamp > 0.0 && now >= stamp && now - stamp < 0.25;
            const bool matches = holding ? (position > -0.0085 && position < 0.0175) :
                std::abs(position - target) <= 0.0015;
            if (!fresh || !matches) {stable_since = -1.0; samples = 0;}
            else if (stamp != last_stamp) {
                if (stable_since < 0.0 || std::abs(position - last_position) > 0.0005) {
                    stable_since = stamp;
                    samples = 0;
                }
                ++samples;
                if (samples >= 4 && stamp - stable_since >= 0.4) {
                    if (holding) {holding_state_ = PickResult::HOLD_CONFIRMED;}
                    phase("GRIPPER_FEEDBACK", "position=" + std::to_string(position) +
                        "; holding=" + std::to_string(holding));
                    return true;
                }
            }
            last_stamp = stamp;
            last_position = position;
            std::this_thread::sleep_for(20ms);
        }
        std::lock_guard<std::mutex> lock(joint_mutex_);
        gripper_failure_ = std::string(holding ?
            "Holding unconfirmed: empty, moving, or stale fingers" : "Target width not reached with fresh encoders") +
            "; target=" + std::to_string(target) +
            "; position=" + std::to_string(gripper_position_) +
            "; feedback_age=" + std::to_string(now().seconds() - gripper_stamp_) +
            "; hardware=" + hardware_state_.fault +
            "; canceled=" + std::to_string(release_requested_.load());
        phase("GRIPPER_FEEDBACK_REJECTED", gripper_failure_);
        return false;
    }

    // Caller holds joint_mutex_. Older standalone/fake controllers can opt out explicitly.
    bool hardwareFeedbackValid() const {
        if (!require_hardware_state_) {return true;}
        const double age = now().seconds() - rclcpp::Time(hardware_state_.header.stamp).seconds();
        return age >= 0.0 && age < 0.5 && hardware_state_.read_ok && hardware_state_.command_ok &&
            hardware_state_.ros_connected && hardware_state_.manipulator_connected &&
            hardware_state_.all_joints_torque_enabled && hardware_state_.fault.empty();
    }

    // GripperCommand 액션 전송 및 완료 대기
    bool sendGripperGoal(double position, double max_effort = 10.0, bool explicit_open = false,
                         bool * feedback_failure = nullptr, double action_timeout = 3.0) {
        if (feedback_failure) {*feedback_failure = false;}
        gripper_failure_.clear();
        if (release_requested_ && !explicit_open) {
            gripper_failure_ = "Opening/closing interrupted";
            return false;
        }
        if (!gripper_action_client_->wait_for_action_server(2s)) {
            gripper_failure_ = "Gripper action server unavailable";
            RCLCPP_ERROR(this->get_logger(), "Gripper Action Server unavailable!");
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            const double age = now().seconds() - gripper_stamp_;
            if (!hardwareFeedbackValid()) {
                gripper_failure_ = "Hardware state missing, stale, torque disabled, or transport fault: " +
                    hardware_state_.fault;
                phase("GRIPPER_HARDWARE_FAILED", gripper_failure_);
                return false;
            }
            if (gripper_stamp_ <= 0.0 || age < 0.0 || age >= 0.25) {
                gripper_failure_ = "Fresh gripper feedback required before command; age=" +
                    std::to_string(age);
                phase("GRIPPER_FEEDBACK_REJECTED", gripper_failure_);
                return false;
            }
        }

        GripperCommand::Goal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        // A failed closing/opening operation cannot prove continued possession.
        if (position < 0.0 || holding_state_ != PickResult::HOLD_EMPTY) {
            holding_state_ = PickResult::HOLD_UNKNOWN;
        }

        phase("GRIPPER_COMMAND", "target=" + std::to_string(position) +
            "; max_effort=" + std::to_string(max_effort) +
            "; timeout=" + std::to_string(action_timeout) +
            "; explicit_open=" + std::to_string(explicit_open));

        if (!runAction<GripperCommand>(gripper_action_client_, goal, action_timeout, explicit_open)) {
            gripper_failure_ = last_action_error_;
            phase("GRIPPER_ACTION_FAILED", gripper_failure_);
            return false;
        }
        const bool confirmed = position < 0.0 || waitForGripper(false, position, explicit_open);
        if (feedback_failure) {*feedback_failure = !confirmed;}
        if (confirmed && position > 0.0) {
            holding_state_ = PickResult::HOLD_EMPTY;
            carrying_ = false;
            publishCarrying();
        }
        return confirmed;
    }

    bool openForPlacement() {
        phase("PLACE_OPEN");
        for (int attempt = 1; attempt <= 3; ++attempt) {
            if (!rclcpp::ok() || release_requested_ || action_fault_) {return false;}
            phase("PLACE_OPEN_ATTEMPT", "attempt=" + std::to_string(attempt) +
                "/3; target=0.019; max_effort=10.0");
            bool feedback_failure = false;
            if (sendGripperGoal(0.019, 10.0, false, &feedback_failure)) {return true;}
            if (!feedback_failure || release_requested_ || action_fault_) {
                phase("PLACE_OPEN_ABORTED", "Action failed or opening canceled; no retry");
                return false;
            }
            if (attempt == 3) {break;}
            phase("PLACE_OPEN_RETRY_WAIT", "Opening unconfirmed; keeping arm pose; delay=0.3s");
            const auto deadline = std::chrono::steady_clock::now() + 300ms;
            while (std::chrono::steady_clock::now() < deadline) {
                if (!rclcpp::ok() || release_requested_ || action_fault_) {return false;}
                std::this_thread::sleep_for(20ms);
            }
        }
        phase("PLACE_OPEN_EXHAUSTED", "Opening unconfirmed after 3 attempts; no retreat");
        return false;
    }

    void initializePoseOnStartup() {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm action server to initialize arm pose...");
        if (!arm_action_client_->wait_for_action_server(30s)) {
            RCLCPP_WARN(this->get_logger(), "Arm action server not available within 30s for initial pose.");
            return;
        }

        // Wake on fresh feedback instead of assuming readiness after a fixed sleep.
        {
            std::unique_lock<std::mutex> lock(joint_mutex_);
            if (!joint_feedback_.wait_for(lock, 2s, [this]() {
                    const auto time = now().seconds();
                    return std::all_of(arm_stamps_.begin(), arm_stamps_.end(),
                        [time](double stamp) {return time >= stamp && time - stamp < 0.25;});
                })) {return;}
        }

        std::unique_lock<std::mutex> operation_lock(operation_mutex_);
        const std::vector<double> init_joints = initialPoseJoints();
        RCLCPP_INFO(
            this->get_logger(),
            "Arm action server ready! Moving arm to initial pose [0.0, -0.523, -0.523, 1.5707] rad...");
        if (sendArmGoal(init_joints, 2.5)) {
            RCLCPP_INFO(this->get_logger(), "Arm successfully positioned at initial pose.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to move arm to initial pose on startup.");
        }
    }

    void handleParkRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
        if (!operation_lock.owns_lock() || release_requested_ || action_fault_) {
            response->message = "Manipulation busy or finishing";
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Moving robot arm to the initial safe pose...");
        const std::vector<double> init_joints = initialPoseJoints();
        if (sendArmGoal(init_joints, 3.0)) {
            response->success = true;
            response->message = "Arm moved to the initial safe pose.";
            RCLCPP_INFO(this->get_logger(), "Arm is in the initial safe pose.");
        } else {
            response->success = false;
            response->message = "Failed to move arm to the initial safe pose.";
        }
    }

    void handleOpenGripperRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        std::lock_guard<std::mutex> release_lock(release_mutex_);
        {
            std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
            release_requested_ = true;
        }
        struct ResetRelease {
            std::atomic_bool & value;
            ~ResetRelease() {value = false;}
        } reset_release{release_requested_};
        std::unique_lock<std::mutex> operation_lock(operation_mutex_);
        // Opening is permitted only after the owned action has actually terminated.
        const bool canceled = cancelPending() && !action_fault_;
        if (canceled && sendGripperGoal(0.019, 10.0, true)) {
            response->success = true;
            response->message = "Gripper opened.";
        } else {
            response->success = false;
            response->message = "Failed to cancel manipulation or open gripper.";
        }
    }

    void handleCloseGripperRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
        if (!operation_lock.owns_lock() || release_requested_ || action_fault_) {
            response->message = "Manipulation busy or finishing";
            return;
        }
        if (sendGripperGoal(-0.010, 15.0)) {
            response->success = true;
            response->message = "Gripper closed.";
        } else {
            response->success = false;
            response->message = "Failed to close gripper.";
        }
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::mutex joint_mutex_;
    std::condition_variable joint_feedback_;
    double reached_joint_tolerance_{0.0};
    std::array<double, 4> arm_positions_{};
    std::array<double, 4> arm_velocities_{};
    std::array<double, 4> arm_stamps_{};
    double gripper_position_{0.0}, gripper_stamp_{0.0};
    bool use_candidate_grasp_{false};
    double forward_offset_{0.0};
    double min_body_depth_{0.006};
    double approach_error_margin_{0.02};
    double approach_goal_tolerance_{0.02};
    rclcpp::Service<Trigger>::SharedPtr execute_service_;
    rclcpp::Service<Trigger>::SharedPtr park_service_;
    rclcpp::Service<Trigger>::SharedPtr open_gripper_service_;
    rclcpp::Service<ExecutePick>::SharedPtr pick_service_;
    rclcpp::Service<Trigger>::SharedPtr prepare_gripper_service_;
    uint8_t holding_state_{PickResult::HOLD_UNKNOWN};  // Protected by operation_mutex_.
    std::string gripper_failure_, last_action_error_;
    bool require_hardware_state_{false};
    cleanup_interfaces::msg::GripperHardwareState hardware_state_;
    rclcpp::Subscription<cleanup_interfaces::msg::GripperHardwareState>::SharedPtr hardware_sub_;
    rclcpp::Service<Trigger>::SharedPtr close_gripper_service_;
    rclcpp::Service<PlaceObject>::SharedPtr place_service_;
    rclcpp::Service<Trigger>::SharedPtr observe_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
    rclcpp::Service<EvaluateGrasp>::SharedPtr evaluate_grasp_service_;
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PointStamped latest_map_target_;
    bool has_target_;
    double target_max_age_{6.0};
    double insertion_depth_{0.012}, floor_height_{0.0};
    rclcpp::Time target_received_time_;
    std::mutex target_mutex_;
    std::atomic_bool sequence_in_progress_{false};
    std::atomic_bool carrying_{false}, release_requested_{false}, action_fault_{false};
    std::mutex dispatch_mutex_;
    std::function<bool()> pending_cleanup_;  // Protected by operation_mutex_.
    rclcpp::Service<Trigger>::SharedPtr stop_manipulation_service_;
    std::mutex operation_mutex_, release_mutex_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carrying_pub_;
    rclcpp::TimerBase::SharedPtr carrying_timer_;
    std::thread init_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceActionNode>();

    // Action 응답 및 콜백을 블로킹 없이 처리하기 위한 멀티스레드 실행기
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
