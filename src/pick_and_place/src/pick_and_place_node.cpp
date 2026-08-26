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

        // 4. 파지 시퀀스 실행 서비스
        execute_service_ = this->create_service<Trigger>(
            "/execute_pick_and_place",
            std::bind(
                &PickAndPlaceActionNode::handleExecuteRequest,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rmw_qos_profile_services_default,
            action_callback_group_);

        // 5. 런처 실행 시 로봇팔 초기 자세 자동 이동 비동기 스레드 시작
        init_thread_ = std::thread(&PickAndPlaceActionNode::initializePoseOnStartup, this);

        RCLCPP_INFO(this->get_logger(), "Pick & Place Action Node Initialized.");
        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for /object_centroid. Call /execute_pick_and_place to start.");
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

        // 1. 최신 TF(Time 0)를 조회하여 map -> link1 상대 좌표 변환
        geometry_msgs::msg::PointStamped target_in_arm;
        try {
            target_in_map.header.stamp = rclcpp::Time(0);
            target_in_arm = tf_buffer_->transform(target_in_map, "link1", tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Transform from map to link1 failed: %s", ex.what());
            message = std::string("TF transform from map to link1 failed: ") + ex.what();
            return false;
        }

        double rel_x = target_in_arm.point.x;
        double rel_y = target_in_arm.point.y;
        double rel_z = target_in_arm.point.z;

        RCLCPP_INFO(this->get_logger(), 
            "[RELATIVE TARGET (link1)] x = %.3f m, y = %.3f m, z = %.3f m", rel_x, rel_y, rel_z);

        // 2. 4-DOF 역기구학 계산
        std::vector<double> joint_angles(4, 0.0);
        if (!solve4DofIK(rel_x, rel_y, rel_z, joint_angles)) {
            RCLCPP_ERROR(this->get_logger(), "Target is out of reachable workspace!");
            message = "Target is outside the manipulator workspace.";
            return false;
        }

        // === Step 1: 그리퍼 열기 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 1. Opening Gripper (Action)...");
        if (!sendGripperGoal(0.019)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gripper!");
            message = "Failed to open the gripper.";
            return false;
        }

        // === Step 2: 로봇팔 목표 좌표 이동 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 2. Moving Arm to Target [%.2f, %.2f, %.2f, %.2f] rad...",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);
        if (!sendArmGoal(joint_angles, 3.0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to target!");
            message = "Failed to move the arm to the target.";
            return false;
        }

        // === Step 3: 그리퍼 닫기 (물체 파지) Action ===
        RCLCPP_INFO(this->get_logger(), "===> 3. Closing Gripper (Action Grasp)...");
        if (!sendGripperGoal(-0.010)) {
            message = "Failed to close the gripper.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        // === Step 4: 홈 포즈 복귀 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 4. Returning to Home Pose...");
        std::vector<double> home_joints = {0.0, -0.523, -0.523, 1.5707};
        if (!sendArmGoal(home_joints, 2.5)) {
            message = "Object grasped, but the arm failed to return home.";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "===> Pick & Place Sequence Completed Successfully!\n");
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

    // OpenMANIPULATOR-X 4-DOF 해석적 기구학 (Analytical IK)
    // OpenMANIPULATOR-X 4-DOF 해석적 기구학 (Analytical IK)
    bool solve4DofIK(double x, double y, double z, std::vector<double>& out_joints) {
        // OpenMANIPULATOR-X 링크 길이 (단위: m)
        const double L1 = 0.077;  // Base to Joint2 (Z축 오프셋)
        const double L2 = 0.130;  // Joint2 to Joint3
        const double L3 = 0.124;  // Joint3 to Joint4
        const double L4 = 0.126;  // Joint4 to Gripper Tip

        // 1. Joint 1 (Base Yaw)
        out_joints[0] = std::atan2(y, x);

        // 2. 평면 2D 좌표계 변환
        double r = std::sqrt(x * x + y * y);
        double z_rel = z - L1; // Joint2 중심 기준 상대 높이

        // 3. 다양한 접근 피치 각도 탐색 (바닥 물체는 -80도 ~ 0도 하향 접근)
        for (double pitch_deg = -80.0; pitch_deg <= 10.0; pitch_deg += 2.0) {
            double phi = pitch_deg * M_PI / 180.0; // 수평 기준 End-Effector 절대 피치각

            // 손목 관절(Joint 4) 목표 위치 역산
            double rw = r - L4 * std::cos(phi);
            double zw = z_rel - L4 * std::sin(phi);

            double D = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
            if (D < -1.0 || D > 1.0) continue; // 도달 불가

            // Elbow-Up 해 선택 (팔꿈치가 위로 솟고 손끝이 바닥으로 내려가는 형상)
            double q3 = -std::acos(D); 

            // Joint 2 계산 (URDF 기준: 수직 위쪽이 0 rad이므로 pi/2 - planar_angle 적용)
            double alpha = std::atan2(zw, rw);
            double beta = std::atan2(L3 * std::sin(q3), L2 + L3 * std::cos(q3));
            double theta2_planar = alpha - beta;
            double q2 = (M_PI / 2.0) - theta2_planar;

            // Joint 4 계산 (전체 피치각 phi 유지)
            double q4 = phi - theta2_planar - q3;

            // OpenMANIPULATOR-X 실제 관절 가동 범위(Limit) 체크
            // Joint 2: -1.8 ~ 1.57 rad (-103도 ~ 90도)
            // Joint 3: -1.57 ~ 1.53 rad (-90도 ~ 87도)
            // Joint 4: -1.8 ~ 2.0 rad (-103도 ~ 114도)
            if (q2 >= -1.8 && q2 <= 1.57 &&
                q3 >= -1.57 && q3 <= 1.53 &&
                q4 >= -1.8 && q4 <= 2.0) {
                
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

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Service<Trigger>::SharedPtr execute_service_;
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
