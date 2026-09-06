#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cleanup_interfaces/srv/evaluate_grasp.hpp>
#include <cleanup_interfaces/srv/place_object.hpp>
#include "pick_and_place/grasp_kinematics.hpp"
#include "pick_and_place/candidate_grasp.hpp"
#include "pick_and_place/place_kinematics.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class PickAndPlaceActionNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand;
    using Trigger = std_srvs::srv::Trigger;
    using EvaluateGrasp = cleanup_interfaces::srv::EvaluateGrasp;
    using PlaceObject = cleanup_interfaces::srv::PlaceObject;

    PickAndPlaceActionNode()
    : Node("pick_and_place_action_node"),
      has_target_(false),
      target_received_time_(0, 0, RCL_ROS_TIME)
    {
        this->declare_parameter<double>("target_max_age", 2.0);
        this->declare_parameter<std::string>("target_topic", "/object_centroid");
        insertion_depth_ = this->declare_parameter<double>("grasp_insertion_depth", 0.012);
        floor_height_ = this->declare_parameter<double>("floor_height", 0.0);
        use_candidate_grasp_ = declare_parameter<bool>("use_candidate_grasp", false);
        forward_offset_ = declare_parameter<double>("grasp_forward_offset", 0.0);
        if (!std::isfinite(forward_offset_) || std::abs(forward_offset_) > 0.030) {
            throw std::invalid_argument("Grasp forward offset must be within 30mm");
        }
        RCLCPP_INFO(get_logger(), "GRASP_CONFIG strategy=%s forward_offset=%.3f",
            use_candidate_grasp_ ? "candidate_body" : "legacy", forward_offset_);
        if (!std::isfinite(insertion_depth_) || insertion_depth_ < 0.0 ||
            insertion_depth_ > 0.025 || !std::isfinite(floor_height_)) {
            throw std::invalid_argument("Invalid floor or grasp insertion depth");
        }
        target_max_age_ = this->get_parameter("target_max_age").as_double();
        const std::string target_topic =
            this->get_parameter("target_topic").as_string();
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
                        }
                    }
                    if (msg->name[i] != "gripper_left_joint" ||
                        !std::isfinite(msg->position[i])) {continue;}
                    std::lock_guard<std::mutex> lock(joint_mutex_);
                    gripper_position_ = msg->position[i];
                    gripper_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
                }
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
        init_thread_ = std::thread(&PickAndPlaceActionNode::initializePoseOnStartup, this);

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
                             pick_and_place::GraspPlan & plan) {
        return use_candidate_grasp_ ? pick_and_place::planCandidateExecution(
            x + forward_offset_, y, z, floor, freshArmPositions(), plan) :
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
            response->reachable = planAutomaticGrasp(
                local.x(), local.y(), local.z(), floor.z(), plan);
            response->message = "link1 target x=" + std::to_string(local.x()) +
                ", y=" + std::to_string(local.y()) + ", z=" + std::to_string(local.z());
            response->message += "; strategy=" + std::string(
                use_candidate_grasp_ ? "candidate_body" : "legacy") +
                "; forward_offset=" + std::to_string(forward_offset_);
            if (response->reachable) {
                response->message += "; grasp pitch=" + std::to_string(plan.pitch_degrees) +
                    "; insertion=" + std::to_string(plan.insertion_depth);
                return;
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
                const auto relative = (proposed * base_arm).inverse() * point;
                // A 2cm arrival error must still permit both grasp and lift.
                const auto proposed_floor = (proposed * base_arm).inverse() *
                    tf2::Vector3(point.x(), point.y(), floor_height_);
                if (!planAutomaticGrasp(relative.x(), relative.y(), relative.z(),
                        proposed_floor.z(), plan) ||
                    !planAutomaticGrasp(relative.x() + 0.02, relative.y(), relative.z(),
                        proposed_floor.z(), plan)) {continue;}
                response->approach_available = true;
                response->approach_pose.header.frame_id = "map";
                response->approach_pose.header.stamp = this->now();
                response->approach_pose.pose.position.x = proposed.getOrigin().x();
                response->approach_pose.pose.position.y = proposed.getOrigin().y();
                response->approach_pose.pose.orientation = tf2::toMsg(orientation);
                response->message += "; feasible base standoff=" + std::to_string(standoff);
                return;
            }
            response->message += "; no IK-feasible pose within approach standoff limits";
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

        bool expected = false;
        if (!sequence_in_progress_.compare_exchange_strong(expected, true)) {
            response->success = false;
            response->message = "A pick-and-place sequence is already in progress.";
            return;
        }

        std::string message;
        try {
            response->success = executePickSequence(message);
            response->message = message;
        } catch (const std::exception & ex) {
            response->success = false;
            response->message = std::string("Unexpected pick-and-place error: ") + ex.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }

        sequence_in_progress_ = false;
    }

    bool executePickSequence(std::string & message) {
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
            if (target_age > target_max_age_ || observation_age < 0.0 ||
                observation_age > target_max_age_) {
                has_target_ = false;
                message = "The latest pick target is stale.";
                RCLCPP_WARN(
                    this->get_logger(), "%s Age: %.2fs", message.c_str(), target_age);
                return false;
            }
            target_in_map = latest_map_target_;
        }

        if (!arm_action_client_->wait_for_action_server(2s)) {
            message = "Arm action server is unavailable.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }
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
        if (!sendGripperGoal(0.019)) {
            message = "Failed to open gripper.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 2: 물체 상공 접근 (Pre-Grasp Pose) ===
        // Joint 1(Yaw)이 물체 방향으로 정렬되며 물체 상공으로 위치
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 2/7] Moving to Pre-Grasp Pose (Above target, Yaw: %.1f°)...",
            pre_grasp_joints[0] * 180.0 / M_PI);
        phase("PREGRASP");
        if (!sendArmGoal(pre_grasp_joints, 2.5)) {
            message = "Failed to reach pre-grasp pose.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 3: 물체 위치로 수직 하강 (Grasp Pose) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 3/7] Descending to Surface Pose...");
        phase("DESCEND");
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
        RCLCPP_INFO(this->get_logger(), "===> [Step 5/7] Closing Gripper (Grasping object)...");
        if (!sendGripperGoal(-0.010, 15.0)) {
            message = "Failed to close gripper.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }
        // Controller success only acknowledges motion. Empty fingers reaching
        // their closed stop must never be transported as a collected object.
        if (!waitForGripper(true, -0.010)) {
            const bool lifted = sendArmPath(plan.lift, use_candidate_grasp_ ? 4.0 : 1.8);
            const bool parked = lifted && sendArmGoal(initialPoseJoints(), 2.5);
            message = parked ? "EMPTY_GRASP: no stable finger obstruction after closing" :
                "Grasp unverified and arm recovery failed; operator inspection required";
            phase("GRASP_UNVERIFIED", message);
            return false;
        }

        // === Step 5: 물체 수직 인양 (Lift Object) ===
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 6/7] Lifting Object (+%.1f cm)...", plan.lift_height * 100.0);
        phase("LIFT");
        if (!sendArmPath(plan.lift, use_candidate_grasp_ ? 4.0 : 1.8)) {
            message = "Lift failed; holding state unknown.";
            return false;
        }
        const bool held_after_lift = waitForGripper(true, -0.010);

        // === Step 6: 안전 홈 포즈 복귀 (Return to Home) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 7/7] Returning to Home Pose...");
        const std::vector<double> home_joints = initialPoseJoints();
        phase("RETURN_HOME");
        if (!sendArmGoal(home_joints, 2.5)) {
            message = "Object grasped, but the arm failed to return home.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        if (!held_after_lift || !waitForGripper(true, -0.010)) {
            message = "EMPTY_GRASP: finger obstruction lost during lift/return";
            phase("GRASP_UNVERIFIED", message);
            return false;
        }
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
            phase("PLACE_OPEN");
            if (!sendGripperGoal(0.019) || !waitForGripper(false, 0.019)) {
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

    void phase(const std::string & name, const std::string & detail = "") {
        std_msgs::msg::String event;
        event.data = name + "|" + detail;
        phase_pub_->publish(event);
        RCLCPP_INFO(get_logger(), "[PICK_PHASE] %s", event.data.c_str());
    }

    bool sendArmPath(const std::vector<std::vector<double>> & path, double duration_sec) {
        if (path.empty()) {return false;}
        if (!arm_action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Arm Action Server unavailable!");
            return false;
        }

        std::vector<double> current(4);
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            for (size_t j = 0; j < 4; ++j) {
                const double age = now().seconds() - arm_stamps_[j];
                if (age < 0.0 || age > 0.25) {
                    RCLCPP_ERROR(get_logger(), "Fresh arm feedback required for floor check");
                    return false;
                }
                current[j] = arm_positions_[j];
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
        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4"};

        for (size_t i = 0; i < path.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = path[i];
            point.time_from_start = rclcpp::Duration::from_seconds(
                duration_sec * (i + 1) / path.size());
            goal.trajectory.points.push_back(point);
        }

        auto goal_handle_future = arm_action_client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(3s) != std::future_status::ready) return false;

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = arm_action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::duration<double>(duration_sec + 3.0)) !=
            std::future_status::ready)
        {
            arm_action_client_->async_cancel_goal(goal_handle);
            return false;
        }

        const auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    }

    bool waitForGripper(bool holding, double target) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        double stable_since = -1.0, last_stamp = -1.0, last_position = 0.0;
        int samples = 0;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            double position, stamp;
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                position = gripper_position_;
                stamp = gripper_stamp_;
            }
            const double now = this->now().seconds();
            const bool fresh = stamp > 0.0 && now >= stamp && now - stamp < 0.25;
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
                    phase("GRIPPER_FEEDBACK", "position=" + std::to_string(position) +
                        "; holding=" + std::to_string(holding));
                    return true;
                }
            }
            last_stamp = stamp;
            last_position = position;
            std::this_thread::sleep_for(20ms);
        }
        phase("GRIPPER_FEEDBACK_REJECTED", holding ?
            "Empty, moving, or stale fingers" : "Open width not reached with fresh encoders");
        return false;
    }

    // GripperCommand 액션 전송 및 완료 대기
    bool sendGripperGoal(double position, double max_effort = 10.0) {
        if (!gripper_action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Gripper Action Server unavailable!");
            return false;
        }

        GripperCommand::Goal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;

        auto goal_handle_future = gripper_action_client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(3s) != std::future_status::ready) return false;

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = gripper_action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(3s) != std::future_status::ready) {
            gripper_action_client_->async_cancel_goal(goal_handle);
            return false;
        }
        const auto result = result_future.get();
        if (result.result) {
            phase("GRIPPER_RESULT", "position=" + std::to_string(result.result->position) +
                "; effort=" + std::to_string(result.result->effort) +
                "; stalled=" + std::to_string(result.result->stalled) +
                "; reached_goal=" + std::to_string(result.result->reached_goal));
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {return false;}
        return position < 0.0 || waitForGripper(false, position);
    }

    void initializePoseOnStartup() {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm action server to initialize arm pose...");
        if (!arm_action_client_->wait_for_action_server(30s)) {
            RCLCPP_WARN(this->get_logger(), "Arm action server not available within 30s for initial pose.");
            return;
        }

        // 안정적인 컨트롤러 구동을 위해 0.5초 대기
        rclcpp::sleep_for(500ms);

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
        if (sendGripperGoal(0.019)) {
            response->success = true;
            response->message = "Gripper opened.";
        } else {
            response->success = false;
            response->message = "Failed to open gripper.";
        }
    }

    void handleCloseGripperRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
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
    std::array<double, 4> arm_positions_{};
    std::array<double, 4> arm_stamps_{};
    double gripper_position_{0.0}, gripper_stamp_{0.0};
    bool use_candidate_grasp_{false};
    double forward_offset_{0.0};
    rclcpp::Service<Trigger>::SharedPtr execute_service_;
    rclcpp::Service<Trigger>::SharedPtr park_service_;
    rclcpp::Service<Trigger>::SharedPtr open_gripper_service_;
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
    double target_max_age_{2.0};
    double insertion_depth_{0.012}, floor_height_{0.0};
    rclcpp::Time target_received_time_;
    std::mutex target_mutex_;
    std::atomic_bool sequence_in_progress_{false};
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
