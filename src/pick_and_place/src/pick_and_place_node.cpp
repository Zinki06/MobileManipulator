#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>

using namespace std::chrono_literals;

class PickAndPlaceActionNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand;

    PickAndPlaceActionNode() : Node("pick_and_place_action_node"), has_target_(false) {
        // 1. Action Clients 생성
        arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");
        gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/gripper_controller/gripper_cmd");

        // 2. TF 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 물체 맵 좌표 구독
        target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/object_centroid", 10,
            std::bind(&PickAndPlaceActionNode::targetCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pick & Place Action Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "Waiting for Action Servers & /object_centroid...");

        // 4. 터미널 입력 스레드 시작
        input_thread_ = std::thread(&PickAndPlaceActionNode::terminalLoop, this);
    }

    ~PickAndPlaceActionNode() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_map_target_ = *msg;
        has_target_ = true;
    }

    void terminalLoop() {
        // 액션 서버 연결 대기
        if (!arm_action_client_->wait_for_action_server(5s)) {
            RCLCPP_WARN(this->get_logger(), "Arm Action Server not ready yet.");
        }
        if (!gripper_action_client_->wait_for_action_server(5s)) {
            RCLCPP_WARN(this->get_logger(), "Gripper Action Server not ready yet.");
        }

        while (rclcpp::ok()) {
            std::cout << "\n======================================================\n";
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                if (has_target_) {
                    std::cout << "[GLOBAL TARGET in map] X: " << latest_map_target_.point.x
                              << " m | Y: " << latest_map_target_.point.y
                              << " m | Z: " << latest_map_target_.point.z << " m\n";
                    std::cout << ">>> Press [ENTER] to execute Pick & Place Action Sequence: ";
                } else {
                    std::cout << "[WAITING] No /object_centroid received yet...\n";
                    std::this_thread::sleep_for(1s);
                    continue;
                }
            }

            std::string input;
            std::getline(std::cin, input);

            if (!rclcpp::ok()) break;

            executePickSequence();
        }
    }

    void executePickSequence() {
        geometry_msgs::msg::PointStamped target_in_map;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (!has_target_) {
                RCLCPP_WARN(this->get_logger(), "No target point available!");
                return;
            }
            target_in_map = latest_map_target_;
        }

        // 1. 최신 TF(Time 0)를 조회하여 map -> link1 상대 좌표 변환
        geometry_msgs::msg::PointStamped target_in_arm;
        try {
            target_in_map.header.stamp = rclcpp::Time(0);
            target_in_arm = tf_buffer_->transform(target_in_map, "link1", tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Transform from map to link1 failed: %s", ex.what());
            return;
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
            return;
        }

        // === Step 1: 그리퍼 열기 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 1. Opening Gripper (Action)...");
        if (!sendGripperGoal(0.019)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gripper!");
            return;
        }

        // === Step 2: 로봇팔 목표 좌표 이동 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 2. Moving Arm to Target [%.2f, %.2f, %.2f, %.2f] rad...",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);
        if (!sendArmGoal(joint_angles, 3.0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to target!");
            return;
        }

        // === Step 3: 그리퍼 닫기 (물체 파지) Action ===
        RCLCPP_INFO(this->get_logger(), "===> 3. Closing Gripper (Action Grasp)...");
        if (!sendGripperGoal(-0.010)) {
            RCLCPP_WARN(this->get_logger(), "Grasp completed with stall or resistance (normal for grasping).");
        }

        // === Step 4: 홈 포즈 복귀 Action ===
        RCLCPP_INFO(this->get_logger(), "===> 4. Returning to Home Pose...");
        std::vector<double> home_joints = {0.0, -0.523, -0.523, 1.5707};
        sendArmGoal(home_joints, 2.5);

        RCLCPP_INFO(this->get_logger(), "===> Pick & Place Sequence Completed Successfully!\n");
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
        return (result_future.wait_for(std::chrono::duration<double>(duration_sec + 3.0)) == std::future_status::ready);
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
        return (result_future.wait_for(3s) == std::future_status::ready);
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

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PointStamped latest_map_target_;
    bool has_target_;
    std::mutex target_mutex_;
    std::thread input_thread_;
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