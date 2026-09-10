// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#ifndef MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
#define MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cleanup_interfaces/msg/gripper_hardware_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "manipulation_hardware/opencr.hpp"
#include "manipulation_hardware/visibility_control.h"

namespace robotis
{
namespace manipulation_hardware
{
class OpenCRSystemHardware
  : public hardware_interface::SystemInterface
{
public:
  OpenCRSystemHardware() = default;
  explicit OpenCRSystemHardware(std::unique_ptr<OpenCR> transport)
  : opencr_(std::move(transport)) {}
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenCRSystemHardware)

  MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish_gripper_state(bool force = false);
  hardware_interface::return_type fail_io(const std::string & message);
  rclcpp::Node::SharedPtr telemetry_node_;
  rclcpp::Publisher<cleanup_interfaces::msg::GripperHardwareState>::SharedPtr telemetry_pub_;
  bool read_ok_{false}, command_ok_{false}, goal_refresh_ok_{false};
  bool active_{false};
  int16_t gripper_current_{80};
  uint32_t board_millis_{0};
  std::chrono::steady_clock::time_point board_updated_{}, telemetry_sent_{}, goals_refreshed_{};
  std::string fault_;
  double last_gripper_requested_{std::numeric_limits<double>::quiet_NaN()};
  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;

  std::array<int32_t, 4> joints_acceleration_;
  std::array<int32_t, 4> joints_velocity_;

  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  std::unique_ptr<OpenCR> opencr_;

  std::vector<double> dxl_wheel_commands_;
  std::vector<double> dxl_joint_commands_;
  std::vector<double> dxl_gripper_commands_;

  std::vector<double> dxl_positions_;
  std::vector<double> dxl_velocities_;

  std::vector<double> opencr_sensor_states_;
};
}  // namespace manipulation_hardware
}  // namespace robotis
#endif  // MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
