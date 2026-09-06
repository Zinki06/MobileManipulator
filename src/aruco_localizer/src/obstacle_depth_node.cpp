#include "aruco_localizer/depth_projection.hpp"
#include "aruco_localizer/carry_state.hpp"
#include <array>
#include <algorithm>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ObstacleDepth : public rclcpp::Node
{
public:
  ObstacleDepth() : Node("obstacle_depth"), carry_(*this)
  {
    const double rate = declare_parameter<double>("publish_rate", 10.0);
    const int stride = declare_parameter<int>("stride", 8);
    if (!std::isfinite(rate) || rate < 5.0 || rate > 30.0 || stride < 1 || stride > 8) {
      throw std::invalid_argument("Invalid obstacle depth rate/stride");
    }
    mask_min_ = declare_parameter<std::vector<double>>("carry_mask_min", {0.06, -0.09, 0.16});
    mask_max_ = declare_parameter<std::vector<double>>("carry_mask_max", {0.18, 0.09, 0.35});
    clear_all_while_carrying_ = declare_parameter<bool>("carry_clear_all_obstacles", false);
    if (mask_min_.size() != 3 || mask_max_.size() != 3) {
      throw std::invalid_argument("Carry mask requires three base_link coordinates");
    }
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(mask_min_[i]) || !std::isfinite(mask_max_[i]) ||
        mask_min_[i] >= mask_max_[i] || std::abs(mask_min_[i]) > 0.4 ||
        std::abs(mask_max_[i]) > 0.4) {throw std::invalid_argument("Invalid carry mask");}
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        const std::array<double, 4> parked{0.0, -0.523, -0.523, 1.5707};
        std::array<bool, 4> found{};
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          for (size_t j = 0; j < 4; ++j) {
            if (msg->name[i] == "joint" + std::to_string(j + 1)) {
              found[j] = std::isfinite(msg->position[i]) &&
                std::abs(msg->position[i] - parked[j]) < 0.08 &&
                i < msg->velocity.size() && std::isfinite(msg->velocity[i]) &&
                std::abs(msg->velocity[i]) < 0.05;
            }
          }
        }
        parked_ = std::all_of(found.begin(), found.end(), [](bool value) {return value;});
        joint_stamp_ = rclcpp::Time(msg->header.stamp);
      });
    interval_ = 1.0 / rate;
    stride_ = static_cast<unsigned>(stride);
    output_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cleanup/obstacle_points", 1);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    camera_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    depth_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {convert(*msg);});
  }

private:
  void convert(const sensor_msgs::msg::Image & msg)
  {
    const auto started = std::chrono::steady_clock::now();
    if (!info_ || std::chrono::duration<double>(started - last_).count() < interval_) {return;}
    const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
    if (age < -0.1 || age > 0.25) {return;}
    last_ = started;
    try {
      if ((msg.encoding != "16UC1" && msg.encoding != "32FC1") ||
        !info_->width || !info_->height) {throw std::invalid_argument("Depth encoding/intrinsics");}
      const double sx = static_cast<double>(msg.width) / info_->width;
      const double sy = static_cast<double>(msg.height) / info_->height;
      aruco_localizer::projectDepth(msg.data.data(), msg.data.size(), msg.width, msg.height,
        msg.step, msg.encoding == "32FC1", msg.is_bigendian, stride_,
        info_->k[0] * sx, info_->k[4] * sy, info_->k[2] * sx, info_->k[5] * sy, points_);
      // No usable surface must expire the downstream watchdog, not claim free space.
      if (points_.size() < 60) {return;}
      const double joint_age = (now() - joint_stamp_).seconds();
      if (carry_.active() && parked_ && joint_age >= 0.0 && joint_age < 0.25) {
        if (clear_all_while_carrying_) {
          points_.clear();
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[CARRY_MUTED] Carrying object; all depth obstacle points cleared for motion safety");
        } else {
          // One stamped TF per frame. Keep output coordinates/interfaces unchanged.
          const auto transform = buffer_->lookupTransform(
            "base_link", msg.header.frame_id, msg.header.stamp);
          tf2::Transform to_base;
          tf2::fromMsg(transform.transform, to_base);
          size_t kept = 0;
          const size_t before = points_.size() / 3;
          for (size_t i = 0; i < points_.size(); i += 3) {
            const auto point = to_base * tf2::Vector3(points_[i], points_[i + 1], points_[i + 2]);
            bool inside = true;
            for (size_t j = 0; j < 3; ++j) {
              inside = inside && point[j] >= mask_min_[j] && point[j] <= mask_max_[j];
            }
            if (!inside) {
              for (size_t j = 0; j < 3; ++j) {points_[kept++] = points_[i + j];}
            }
          }
          points_.resize(kept);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[CARRY_SELF_FILTER] removed=%zu retained=%zu; parked arm verified",
            before - kept / 3, kept / 3);
          // A valid frame containing only known self geometry may produce an empty cloud.
          // Invalid/zero raw depth still expires above; external obstacles are retained.
        }
      }
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = msg.header;
      cloud.height = 1;
      cloud.width = points_.size() / 3;
      cloud.fields.resize(3);
      for (size_t i = 0; i < 3; ++i) {
        cloud.fields[i].name = std::string(1, "xyz"[i]);
        cloud.fields[i].offset = i * 4;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
      }
      const uint16_t endian = 1;
      cloud.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian) == 0;
      cloud.point_step = 12;
      cloud.row_step = cloud.width * 12;
      cloud.is_dense = true;
      cloud.data.resize(points_.size() * sizeof(float));
      std::memcpy(cloud.data.data(), points_.data(), cloud.data.size());
      output_->publish(cloud);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[DEPTH_PERF] callback_ms=%.3f input_age_ms=%.1f points=%u",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count(),
        age * 1000, cloud.width);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Invalid obstacle depth: %s", error.what());
    }
  }
  aruco_localizer::CarryState carry_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::vector<double> mask_min_, mask_max_;
  bool clear_all_while_carrying_{false};
  bool parked_{false};
  rclcpp::Time joint_stamp_{0, 0, RCL_ROS_TIME};
  unsigned stride_{8};
  double interval_{0.1};
  std::chrono::steady_clock::time_point last_{};
  std::vector<float> points_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDepth>());
  rclcpp::shutdown();
}
