#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

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

    PickAndPlaceActionNode() : Node("pick_and_place_action_node"), has_target_(false) {
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
            "/object_centroid", 10,
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
    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_map_target_ = *msg;
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
                message = "No /object_centroid target has been received.";
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
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

        // 2. Grasp 및 Pre-grasp / Lift 역기구학 계산
        // (1) Grasp 목표 관절각 계산
        std::vector<double> grasp_joints(4, 0.0);
        if (!solve4DofIK(rel_x, rel_y, rel_z, grasp_joints, -60.0)) {
            RCLCPP_ERROR(this->get_logger(), "Grasp target is out of manipulator workspace!");
            message = "Grasp target is outside the manipulator workspace.";
            return false;
        }

        // (2) Pre-grasp / Lift (물체 상공 +6cm) 목표 관절각 계산
        double lift_z = rel_z + 0.06;
        std::vector<double> pre_grasp_joints(4, 0.0);
        if (!solve4DofIK(rel_x, rel_y, lift_z, pre_grasp_joints, -60.0)) {
            lift_z = rel_z + 0.04;
            if (!solve4DofIK(rel_x, rel_y, lift_z, pre_grasp_joints, -60.0)) {
                pre_grasp_joints = grasp_joints;
                pre_grasp_joints[1] = -0.523;
                pre_grasp_joints[2] = -0.523;
                pre_grasp_joints[3] = 1.5707;
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "[IK Grasp Result] J1(Yaw): %.1f°, J2(Shoulder): %.1f°, J3(Elbow): %.1f°, J4(Wrist): %.1f°",
            grasp_joints[0] * 180.0 / M_PI,
            grasp_joints[1] * 180.0 / M_PI,
            grasp_joints[2] * 180.0 / M_PI,
            grasp_joints[3] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
            "============================================================");

        // === Step 1: 그리퍼 열기 (Open Gripper) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 1/6] Opening Gripper...");
        if (!sendGripperGoal(0.019)) {
            message = "Failed to open gripper.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 2: 물체 상공 접근 (Pre-Grasp Pose) ===
        // Joint 1(Yaw)이 물체 방향으로 정렬되며 물체 상공으로 위치
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 2/6] Moving to Pre-Grasp Pose (Above target, Yaw: %.1f°)...",
            pre_grasp_joints[0] * 180.0 / M_PI);
        if (!sendArmGoal(pre_grasp_joints, 2.5)) {
            message = "Failed to reach pre-grasp pose.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 3: 물체 위치로 수직 하강 (Grasp Pose) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 3/6] Descending to Target Grasp Pose...");
        if (!sendArmGoal(grasp_joints, 1.8)) {
            message = "Failed to reach target grasp pose.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 4: 물체 파지 (Close Gripper) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 4/6] Closing Gripper (Grasping object)...");
        if (!sendGripperGoal(-0.010, 15.0)) {
            message = "Failed to close gripper.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 5: 물체 수직 인양 (Lift Object) ===
        RCLCPP_INFO(this->get_logger(),
            "===> [Step 5/6] Lifting Object (+%.1f cm)...", (lift_z - rel_z) * 100.0);
        if (!sendArmGoal(pre_grasp_joints, 1.8)) {
            RCLCPP_WARN(this->get_logger(), "Object grasped, but failed to lift cleanly.");
        }

        // === Step 6: 안전 홈 포즈 복귀 (Return to Home) ===
        RCLCPP_INFO(this->get_logger(), "===> [Step 6/6] Returning to Home Pose...");
        std::vector<double> home_joints = {0.0, -0.523, -0.523, 1.5707};
        if (!sendArmGoal(home_joints, 2.5)) {
            message = "Object grasped, but the arm failed to return home.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "===> [SUCCESS] Pick & Place Sequence Completed Successfully!\n");
        message = "Pick-and-place sequence completed successfully.";
        return true;
    }

    // FollowJointTrajectory 액션 전송 및 완료 대기
    bool sendArmGoal(const std::vector<double>& joints, double duration_sec) {
        if (!arm_action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Arm Action Server unavailable!");
            return false;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joints;
        point.time_from_start.sec = static_cast<int32_t>(duration_sec);
        point.time_from_start.nanosec = static_cast<uint32_t>((duration_sec - static_cast<int32_t>(duration_sec)) * 1e9);

        goal.trajectory.points.push_back(point);

        auto goal_handle_future = arm_action_client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(3s) != std::future_status::ready) return false;

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = arm_action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::duration<double>(duration_sec + 3.0)) !=
            std::future_status::ready)
        {
            return false;
        }

        return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
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
        if (result_future.wait_for(3s) != std::future_status::ready) return false;

        return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    // OpenMANIPULATOR-X 4-DOF 정밀 해석적 기구학 (Analytical IK)
    // 입력: link1 기준 물체 좌표 (x, y, z)
    // 출력: [joint1, joint2, joint3, joint4] rad
    bool solve4DofIK(
        double x, double y, double z,
        std::vector<double>& out_joints,
        double preferred_pitch = -60.0)
    {
        // OpenMANIPULATOR-X 링크 기하 구조 치수 (단위: m)
        const double X_OFFSET = 0.012;   // Joint 1 회전축 X 오프셋
        const double Z_OFFSET = 0.0765;  // Joint 2 회전축 Z 높이
        const double L2 = 0.13025;       // Joint 2 to Joint 3 링크 길이
        const double ALPHA2 = 0.1853;    // Link 3 굽힘 각도 (atan2(0.024, 0.128))
        const double OFFSET2 = M_PI / 2.0 - ALPHA2; // 1.3855 rad (79.38°)
        const double L3 = 0.124;         // Joint 3 to Joint 4 링크 길이
        const double L4 = 0.126;         // Joint 4 to Gripper Tip 링크 길이

        // 1. Joint 1 (Base Yaw) 계산
        double dx = x - X_OFFSET;
        double dy = y;
        out_joints[0] = std::atan2(dy, dx);

        // 2. 평면 2D 좌표계 변환 (Joint 1/2 회전 중심 기준)
        double r = std::sqrt(dx * dx + dy * dy);
        double z_rel = z - Z_OFFSET;

        // 3. 다양한 접근 피치 각도 탐색 (preferred_pitch 우선)
        std::vector<double> pitch_candidates;
        pitch_candidates.push_back(preferred_pitch);
        for (double p = -85.0; p <= 15.0; p += 2.5) {
            if (std::abs(p - preferred_pitch) > 1e-3) {
                pitch_candidates.push_back(p);
            }
        }

        for (double pitch_deg : pitch_candidates) {
            double phi = pitch_deg * M_PI / 180.0; // 수평 기준 End-Effector 절대 피치각 (하향은 음수)

            // 손목 관절(Joint 4) 위치 역산
            double rw = r - L4 * std::cos(phi);
            double zw = z_rel - L4 * std::sin(phi);

            double D_sq = rw * rw + zw * zw;
            double cos_d = (D_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
            if (cos_d < -1.0 || cos_d > 1.0) continue; // 작업 영역 밖

            // Elbow-Up 형상 선택
            double delta_theta = std::acos(cos_d);

            double gamma = std::atan2(zw, rw);
            double delta = std::atan2(L3 * std::sin(delta_theta), L2 + L3 * std::cos(delta_theta));
            double theta2_abs = gamma + delta;
            double theta3_abs = theta2_abs - delta_theta;

            // URDF 관절 각도 변환
            double q2 = OFFSET2 - theta2_abs;
            double q3 = -theta3_abs - q2;
            double q4 = -phi + theta3_abs;

            // OpenMANIPULATOR-X URDF 물리적 관절 리미트 검사
            // Joint 2: -1.79 ~ 1.57 rad (-102° ~ 90°)
            // Joint 3: -0.94 ~ 1.38 rad (-54° ~ 79°)
            // Joint 4: -1.79 ~ 2.04 rad (-102° ~ 117°)
            if (q2 >= -1.75 && q2 <= 1.55 &&
                q3 >= -0.92 && q3 <= 1.35 &&
                q4 >= -1.75 && q4 <= 2.00)
            {
                out_joints[1] = q2;
                out_joints[2] = q3;
                out_joints[3] = q4;
                return true;
            }
        }

        return false;
    }

    void initializePoseOnStartup() {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm action server to initialize arm pose...");
        if (!arm_action_client_->wait_for_action_server(30s)) {
            RCLCPP_WARN(this->get_logger(), "Arm action server not available within 30s for initial pose.");
            return;
        }

        // 안정적인 컨트롤러 구동을 위해 0.5초 대기
        rclcpp::sleep_for(500ms);

        std::vector<double> init_joints = {0.0, -0.523, -0.523, 1.5707};
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
        RCLCPP_INFO(this->get_logger(), "Parking robot arm to resting position...");
        std::vector<double> park_joints = {0.0, -1.57, 1.37, 0.26};
        if (sendArmGoal(park_joints, 3.0)) {
            response->success = true;
            response->message = "Arm safely parked in resting pose.";
            RCLCPP_INFO(this->get_logger(), "Arm safely parked.");
        } else {
            response->success = false;
            response->message = "Failed to park arm.";
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
    rclcpp::Service<Trigger>::SharedPtr execute_service_;
    rclcpp::Service<Trigger>::SharedPtr park_service_;
    rclcpp::Service<Trigger>::SharedPtr open_gripper_service_;
    rclcpp::Service<Trigger>::SharedPtr close_gripper_service_;
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PointStamped latest_map_target_;
    bool has_target_;
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
