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

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation_hardware/opencr_system.hpp"

namespace robotis
{
namespace manipulation_hardware
{
auto logger = rclcpp::get_logger("manipulation_hardware");
hardware_interface::CallbackReturn OpenCRSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // This plugin owns the shared OpenCR device: two wheels, four arm joints, two fingers.
  const std::vector<std::string> names = {"wheel_left_joint", "wheel_right_joint",
    "joint1", "joint2", "joint3", "joint4", "gripper_left_joint", "gripper_right_joint"};
  if (info_.joints.size() != names.size() || info_.sensors.size() != 2 ||
    info_.sensors[0].state_interfaces.size() != 10 || info_.sensors[1].state_interfaces.size() != 4)
  {
    RCLCPP_ERROR(logger, "Unexpected OpenCR hardware layout");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    if (info_.joints[i].name != names[i]) {
      RCLCPP_ERROR(logger, "Unexpected joint at index %zu: %s", i, info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const auto current = info_.hardware_parameters.find("gripper_goal_current_raw");
  const int requested_current = current == info_.hardware_parameters.end() ? 80 : std::stoi(current->second);
  if (requested_current <= 0 || requested_current > 80) {
    RCLCPP_ERROR(logger, "gripper_goal_current_raw must be in 1..80; never infer current from action effort");
    return hardware_interface::CallbackReturn::ERROR;
  }
  gripper_current_ = static_cast<int16_t>(requested_current);
  telemetry_node_ = std::make_shared<rclcpp::Node>("manipulation_hardware_telemetry",
    rclcpp::NodeOptions().use_global_arguments(false));
  telemetry_pub_ = telemetry_node_->create_publisher<cleanup_interfaces::msg::GripperHardwareState>(
    "/manipulation/gripper_hardware_state", rclcpp::QoS(1).transient_local());
  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  joints_acceleration_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);

  joints_velocity_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

  if (!opencr_) {opencr_ = std::make_unique<OpenCR>(id_);}
  if (opencr_->open_port(usb_port_)) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);
  if (model_number != 0x5000) {
    RCLCPP_ERROR(logger, "Expected the TurtleBot3 OpenCR proxy (model 0x5000)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->is_connect_manipulator()) {
    RCLCPP_INFO(logger, "Connected manipulator");
  } else {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->is_connect_wheels()) {
    RCLCPP_INFO(logger, "Connected wheels");
  } else {
    RCLCPP_FATAL(logger, "Not connected wheels");
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_wheel_commands_.resize(2, 0.0);

  dxl_joint_commands_.resize(4, 0.0);
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.05;
  dxl_joint_commands_[2] = 1.05;
  dxl_joint_commands_[3] = 0.0;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  opencr_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size() +
    info_.sensors[1].state_interfaces.size(),
    0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenCRSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  for (uint8_t i = 0, k = 0; i < info_.sensors.size(); i++) {
    for (uint8_t j = 0; j < info_.sensors[i].state_interfaces.size(); j++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[i].name,
          info_.sensors[i].state_interfaces[j].name,
          &opencr_sensor_states_[k++])
      );
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenCRSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[1]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[2].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[3].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[4].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[5].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[3]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[6].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[7].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenCRSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  fault_.clear();
  // Firmware calibrates inside its USB request handler (up to five seconds).
  // During that window a missing reply is not yet a permanent connection fault.
  active_ = false;
  const auto activation_started = std::chrono::steady_clock::now();
  const auto activation_deadline = activation_started + std::chrono::seconds(8);
  RCLCPP_INFO(logger, "Waiting up to 8s for OpenCR IMU calibration and heartbeat acknowledgement");
  opencr_->imu_recalibration();
  int consecutive_ready = 0;
  bool heartbeat_ok = false;
  bool calibration_done = false;
  bool ros_connected = false;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < activation_deadline) {
    heartbeat_ok = opencr_->send_heartbeat(heartbeat_++);
    read_ok_ = opencr_->read_all();
    calibration_done = read_ok_ && !opencr_->get_data<uint8_t>(
      opencr_control_table.imu_re_calibration.address, 1);
    ros_connected = read_ok_ && opencr_->get_data<uint8_t>(
      opencr_control_table.connect_ROS2.address, 1);
    if (heartbeat_ok && calibration_done && ros_connected) {
      if (++consecutive_ready >= 3) {break;}
    } else {
      consecutive_ready = 0;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  if (!rclcpp::ok() || consecutive_ready < 3) {
    fail_io("OpenCR activation readiness failed within 8s: heartbeat_write=" +
      std::to_string(heartbeat_ok) + "; read=" + std::to_string(read_ok_) +
      "; calibration_done=" + std::to_string(calibration_done) +
      "; ros_connected=" + std::to_string(ros_connected));
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger, "OpenCR calibration cleared; communication ready after %.2fs",
    std::chrono::duration<double>(std::chrono::steady_clock::now() - activation_started).count());
  // Preserve measured positions; activation never imposes an implicit closed or home pose.
  const auto joints = opencr_->get_joint_positions();
  std::copy(joints.begin(), joints.end(), dxl_joint_commands_.begin());
  dxl_gripper_commands_[0] = dxl_gripper_commands_[1] = opencr_->get_gripper_position();
  std::copy(joints.begin(), joints.end(), dxl_positions_.begin() + 2);
  dxl_positions_[6] = dxl_positions_[7] = dxl_gripper_commands_[0];
  command_ok_ = opencr_->set_joint_profile_acceleration(joints_acceleration_) &&
    opencr_->set_joint_profile_velocity(joints_velocity_) &&
    opencr_->set_gripper_profile_acceleration(gripper_acceleration_) &&
    opencr_->set_gripper_profile_velocity(gripper_velocity_) &&
    opencr_->set_gripper_current(gripper_current_) &&
    opencr_->set_joint_positions(dxl_joint_commands_) &&
    opencr_->set_gripper_position(dxl_gripper_commands_[0]);
  if (!command_ok_) {
    fail_io("OpenCR rejected configuration or initial position commit");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!opencr_->joints_torque(opencr::ON) || !opencr_->wheels_torque(opencr::ON)) {
    fail_io("OpenCR torque request failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  board_millis_ = opencr_->get_data<uint32_t>(opencr_control_table.millis.address, 4);
  board_updated_ = std::chrono::steady_clock::now();
  active_ = true;
  RCLCPP_INFO(logger, "Measured-position activation; gripper current=%d raw (action effort is not measured force)",
    gripper_current_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenCRSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Keep the arm supported on a controller fault; do not open or drop a held object.
  active_ = false;
  const bool stopped = opencr_ && opencr_->set_wheel_velocities({0.0, 0.0});
  return stopped ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn OpenCRSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  if (!opencr_) {return hardware_interface::CallbackReturn::SUCCESS;}
  const bool wheels_stopped = opencr_->set_wheel_velocities({0.0, 0.0});
  const bool joints_off = opencr_->joints_torque(opencr::OFF);
  const bool wheels_off = opencr_->wheels_torque(opencr::OFF);
  // No hidden home/zero-gripper motion and no sleep in the transport destructor.
  return wheels_stopped && joints_off && wheels_off ? hardware_interface::CallbackReturn::SUCCESS :
    hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type OpenCRSystemHardware::fail_io(const std::string & message)
{
  fault_ = message;
  command_ok_ = false;
  if (!read_ok_) {
    std::fill(dxl_positions_.begin(), dxl_positions_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(dxl_velocities_.begin(), dxl_velocities_.end(), std::numeric_limits<double>::quiet_NaN());
  }
  RCLCPP_ERROR(logger, "%s", message.c_str());
  // Stop only the base on I/O faults. Torque-off would drop the arm or carried object.
  if (opencr_) {opencr_->set_wheel_velocities({0.0, 0.0});}
  publish_gripper_state(true);
  return hardware_interface::return_type::ERROR;
}

void OpenCRSystemHardware::publish_gripper_state(bool force)
{
  const auto steady = std::chrono::steady_clock::now();
  if (!telemetry_pub_ || (!force && steady - telemetry_sent_ < std::chrono::milliseconds(100))) {return;}
  telemetry_sent_ = steady;
  cleanup_interfaces::msg::GripperHardwareState msg;
  msg.header.stamp = telemetry_node_->now();
  msg.header.frame_id = "gripper_left_link";
  msg.read_ok = read_ok_;
  msg.command_ok = command_ok_;
  msg.goal_refresh_ok = goal_refresh_ok_;
  msg.requested_position = dxl_gripper_commands_.empty() ?
    std::numeric_limits<double>::quiet_NaN() : dxl_gripper_commands_[0];
  msg.fault = fault_;
  msg.position = msg.velocity = msg.goal_position_readback = std::numeric_limits<double>::quiet_NaN();
  if (read_ok_) {
    msg.board_millis = board_millis_;
    msg.ros_connected = opencr_->get_data<uint8_t>(opencr_control_table.connect_ROS2.address, 1);
    msg.manipulator_connected = opencr_->get_data<uint8_t>(opencr_control_table.connect_manipulator.address, 1);
    msg.all_joints_torque_enabled = opencr_->get_data<uint8_t>(opencr_control_table.torque_joints.address, 1);
    msg.position = opencr_->get_gripper_position();
    msg.velocity = opencr_->get_gripper_velocity();
    msg.present_current_raw = opencr_->get_data<int16_t>(opencr_control_table.present_current_gripper.address, 2);
    msg.goal_current_raw = opencr_->get_data<int16_t>(opencr_control_table.goal_current_gripper.address, 2);
    if (goal_refresh_ok_) {msg.goal_position_readback = opencr_->get_gripper_goal_position();}
  }
  telemetry_pub_->publish(msg);
}

hardware_interface::return_type OpenCRSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to read wheels and manipulator states");

  const auto steady = std::chrono::steady_clock::now();
  if (!goal_refresh_ok_ || steady - goals_refreshed_ >= std::chrono::milliseconds(500)) {
    goal_refresh_ok_ = opencr_->refresh_goals();
    goals_refreshed_ = steady;
    if (!goal_refresh_ok_) {read_ok_ = false; return fail_io("OpenCR goal refresh failed");}
  }
  read_ok_ = opencr_->read_all();
  if (!read_ok_) {return fail_io("OpenCR feedback read failed; cached joint state invalidated");}
  const uint32_t ticks = opencr_->get_data<uint32_t>(opencr_control_table.millis.address, 4);
  if (active_ && static_cast<uint32_t>(ticks - board_millis_) > 0x7fffffffU) {
    read_ok_ = false;
    return fail_io("OpenCR clock reset; hardware configuration must be re-established");
  }
  if (ticks != board_millis_) {board_millis_ = ticks; board_updated_ = steady;}
  if (active_ && steady - board_updated_ > std::chrono::milliseconds(250)) {
    read_ok_ = false;
    return fail_io("OpenCR clock stopped advancing; feedback invalidated");
  }
  if (active_ && (!opencr_->get_data<uint8_t>(opencr_control_table.connect_ROS2.address, 1) ||
    !opencr_->get_data<uint8_t>(opencr_control_table.connect_manipulator.address, 1)))
  {
    return fail_io("OpenCR ROS/manipulator disconnected; goal commits unavailable");
  }

  const auto wheels = opencr_->get_wheel_positions();
  const auto wheel_velocities = opencr_->get_wheel_velocities();
  dxl_positions_[0] = wheels[opencr::wheels::LEFT];
  dxl_velocities_[0] = wheel_velocities[opencr::wheels::LEFT];

  dxl_positions_[1] = wheels[opencr::wheels::RIGHT];
  dxl_velocities_[1] = wheel_velocities[opencr::wheels::RIGHT];

  dxl_positions_[2] = opencr_->get_joint_positions()[opencr::joints::JOINT1];
  dxl_velocities_[2] = opencr_->get_joint_velocities()[opencr::joints::JOINT1];

  dxl_positions_[3] = opencr_->get_joint_positions()[opencr::joints::JOINT2];
  dxl_velocities_[3] = opencr_->get_joint_velocities()[opencr::joints::JOINT2];

  dxl_positions_[4] = opencr_->get_joint_positions()[opencr::joints::JOINT3];
  dxl_velocities_[4] = opencr_->get_joint_velocities()[opencr::joints::JOINT3];

  dxl_positions_[5] = opencr_->get_joint_positions()[opencr::joints::JOINT4];
  dxl_velocities_[5] = opencr_->get_joint_velocities()[opencr::joints::JOINT4];

  dxl_positions_[6] = opencr_->get_gripper_position();
  dxl_velocities_[6] = opencr_->get_gripper_velocity();

  dxl_positions_[7] = opencr_->get_gripper_position();
  dxl_velocities_[7] = opencr_->get_gripper_velocity();

  opencr_sensor_states_[0] = opencr_->get_imu().orientation.x;
  opencr_sensor_states_[1] = opencr_->get_imu().orientation.y;
  opencr_sensor_states_[2] = opencr_->get_imu().orientation.z;
  opencr_sensor_states_[3] = opencr_->get_imu().orientation.w;

  opencr_sensor_states_[4] = opencr_->get_imu().angular_velocity.x;
  opencr_sensor_states_[5] = opencr_->get_imu().angular_velocity.y;
  opencr_sensor_states_[6] = opencr_->get_imu().angular_velocity.z;

  opencr_sensor_states_[7] = opencr_->get_imu().linear_acceleration.x;
  opencr_sensor_states_[8] = opencr_->get_imu().linear_acceleration.y;
  opencr_sensor_states_[9] = opencr_->get_imu().linear_acceleration.z;

  opencr_sensor_states_[10] = opencr_->get_battery().voltage;
  opencr_sensor_states_[11] = opencr_->get_battery().percentage;
  opencr_sensor_states_[12] = opencr_->get_battery().design_capacity;
  opencr_sensor_states_[13] = opencr_->get_battery().present;

  publish_gripper_state();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenCRSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to write wheels and manipulator commands");
  if (!active_) {return hardware_interface::return_type::OK;}
  const bool finite = std::all_of(dxl_joint_commands_.begin(), dxl_joint_commands_.end(),
    [](double value) {return std::isfinite(value);}) &&
    std::all_of(dxl_wheel_commands_.begin(), dxl_wheel_commands_.end(),
    [](double value) {return std::isfinite(value);});
  if (!finite) {return fail_io("Nonfinite arm or wheel command");}
  command_ok_ = opencr_->send_heartbeat(heartbeat_++) &&
    opencr_->set_wheel_velocities(dxl_wheel_commands_) &&
    opencr_->set_joint_positions(dxl_joint_commands_) &&
    opencr_->set_gripper_position(dxl_gripper_commands_[0]);
  if (!command_ok_) {return fail_io("OpenCR command staging/commit failed");}
  if (dxl_gripper_commands_[0] != last_gripper_requested_) {
    // Until the next explicit refresh, register 216 contains only our proxy write.
    goal_refresh_ok_ = false;
    last_gripper_requested_ = dxl_gripper_commands_[0];
  }
  return hardware_interface::return_type::OK;
}
}  // namespace manipulation_hardware
}  // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::manipulation_hardware::OpenCRSystemHardware,
  hardware_interface::SystemInterface)
