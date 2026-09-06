#ifndef ARUCO_LOCALIZER__CARRY_STATE_HPP_
#define ARUCO_LOCALIZER__CARRY_STATE_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace aruco_localizer
{
// Only the arm executor publishes this lease, after lift/park and encoder verification.
class CarryState
{
public:
  explicit CarryState(rclcpp::Node & node)
  {
    subscription_ = node.create_subscription<std_msgs::msg::Bool>(
      "/cleanup/carrying", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        holding_ = msg->data;
        received_ = std::chrono::steady_clock::now();
      });
  }
  bool active() const
  {
    return holding_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_).count() < 0.75;
  }
private:
  bool holding_{false};
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};
}  // namespace aruco_localizer
#endif  // ARUCO_LOCALIZER__CARRY_STATE_HPP_
