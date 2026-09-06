#include "aruco_localizer/msg/marker_observation.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace aruco_localizer
{

class NavigationDebugRecorder : public rclcpp::Node
{
public:
  NavigationDebugRecorder()
  : Node("navigation_debug_recorder")
  {
    this->declare_parameter<std::string>("output_directory", "/tmp/aruco_navigation_debug");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>(
      "image_topic", "/camera/camera/color/image_raw");
    this->declare_parameter<double>("telemetry_period", 2.0);

    output_directory_ = this->get_parameter("output_directory").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    const std::string image_topic = this->get_parameter("image_topic").as_string();
    const double telemetry_period =
      this->get_parameter("telemetry_period").as_double();
    if (telemetry_period <= 0.0) {
      throw std::invalid_argument("telemetry_period must be positive");
    }

    initializeSessionDirectory();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    event_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/route_navigation/events", 50,
      std::bind(
        &NavigationDebugRecorder::eventCallback, this, std::placeholders::_1));
    route_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/route_navigation/status", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        route_status_ = message->data;
      });
    localization_fresh_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/aruco/localization_fresh", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        localization_fresh_ = message->data;
      });
    marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/aruco/last_marker_id", 10,
      [this](const std_msgs::msg::Int32::SharedPtr message) {
        last_marker_id_ = message->data;
      });
    marker_observation_sub_ =
      this->create_subscription<aruco_localizer::msg::MarkerObservation>(
      "/aruco/marker_observation", rclcpp::SensorDataQoS(),
      [this](const aruco_localizer::msg::MarkerObservation::SharedPtr message) {
        raw_marker_id_ = message->marker_id;
        raw_marker_bearing_ = message->bearing;
        raw_marker_received_at_ = this->now();
      });
    localization_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/aruco/localization_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        localization_mode_ = message->data;
      });
    correction_age_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/aruco/correction_age", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        correction_age_ = message->data;
      });
    correction_distance_sub_ =
      this->create_subscription<std_msgs::msg::Float64>(
      "/aruco/distance_since_correction", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        distance_since_correction_ = message->data;
      });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odom_x_ = message->pose.pose.position.x;
        odom_y_ = message->pose.pose.position.y;
        odom_yaw_ = tf2::getYaw(message->pose.pose.orientation);
        odom_angular_speed_ = message->twist.twist.angular.z;
        has_odom_ = true;
      });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        command_linear_speed_ = message->linear.x;
        command_angular_speed_ = message->angular.z;
        has_command_ = true;
      });
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(
        &NavigationDebugRecorder::imageCallback, this, std::placeholders::_1));
    telemetry_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(telemetry_period)),
      std::bind(&NavigationDebugRecorder::recordTelemetry, this));

    RCLCPP_INFO(
      this->get_logger(), "Navigation debug session: %s", session_directory_.c_str());
  }

private:
  void initializeSessionDirectory()
  {
    const auto time = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    std::tm local_time;
    localtime_r(&time, &local_time);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "run_%Y%m%d_%H%M%S", &local_time);

    session_directory_ =
      (std::filesystem::path(output_directory_) /
      (std::string(timestamp) + "_" + std::to_string(getpid()))).string();
    std::filesystem::create_directories(session_directory_);

    latest_log_path_ =
      (std::filesystem::path(output_directory_) / "navigation.log").string();
    std::ofstream latest_log(latest_log_path_, std::ios::trunc);
    if (!latest_log.is_open()) {
      throw std::runtime_error("Cannot open latest navigation log: " + latest_log_path_);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    try {
      latest_image_ = cv_bridge::toCvCopy(message, "bgr8")->image;
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to convert debug image: %s", exception.what());
    }
  }

  void eventCallback(const std_msgs::msg::String::SharedPtr message)
  {
    const auto delimiter = message->data.find('|');
    const std::string tag = message->data.substr(0, delimiter);
    const std::string details = delimiter == std::string::npos ?
      std::string() : message->data.substr(delimiter + 1U);

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    const bool pose_available = getRobotPose(x, y, yaw);

    std::ostringstream line;
    line << "[" << std::fixed << std::setprecision(3) << this->now().seconds() <<
      "] [" << tag << "] Pose(map): ";
    if (pose_available) {
      line << "(" << x << ", " << y << ", " << yaw * 180.0 / M_PI << "deg)";
    } else {
      line << "unavailable";
    }
    line << " | ArUcoRecent: " << (localization_fresh_ ? "YES" : "NO") <<
      " (ID: " << last_marker_id_ << ") | RawMarker: ";
    if (raw_marker_received_at_.nanoseconds() != 0 &&
      (this->now() - raw_marker_received_at_).seconds() <= 1.0)
    {
      line << "ID=" << raw_marker_id_ << " bearing=" <<
        raw_marker_bearing_ * 180.0 / M_PI << "deg";
    } else {
      line << "none";
    }
    line << " | " << details;

    appendToLogs(line.str());

    if (!latest_image_.empty()) {
      ++snapshot_counter_;
      std::ostringstream filename;
      filename << "snap_" << std::setw(3) << std::setfill('0') << snapshot_counter_ <<
        "_" << sanitizeTag(tag) << ".jpg";
      cv::imwrite(
        (std::filesystem::path(session_directory_) / filename.str()).string(),
        latest_image_);
    }
  }

  void recordTelemetry()
  {
    if (route_status_.empty() || route_status_.rfind("state=IDLE", 0U) == 0U) {
      return;
    }

    double map_x = 0.0;
    double map_y = 0.0;
    double map_yaw = 0.0;
    const bool map_pose_available = getRobotPose(map_x, map_y, map_yaw);

    std::ostringstream line;
    line << "[" << std::fixed << std::setprecision(3) << this->now().seconds() <<
      "] [TELEMETRY] Pose(map): ";
    if (map_pose_available) {
      line << "(" << map_x << ", " << map_y << ", " <<
        map_yaw * 180.0 / M_PI << "deg)";
    } else {
      line << "unavailable";
    }

    line << " | Odom: ";
    if (has_odom_) {
      line << "(" << odom_x_ << ", " << odom_y_ << ", " <<
        odom_yaw_ * 180.0 / M_PI << "deg, wz=" << odom_angular_speed_ << ")";
    } else {
      line << "unavailable";
    }

    line << " | CmdVel: ";
    if (has_command_) {
      line << "(vx=" << command_linear_speed_ << ", wz=" <<
        command_angular_speed_ << ")";
    } else {
      line << "unavailable";
    }

    line << " | Localization: " << localization_mode_ <<
      " age=" << correction_age_ << "s distance=" <<
      distance_since_correction_ << "m | RawMarker: ";
    if (raw_marker_received_at_.nanoseconds() != 0 &&
      (this->now() - raw_marker_received_at_).seconds() <= 1.0)
    {
      line << "ID=" << raw_marker_id_ << " bearing=" <<
        raw_marker_bearing_ * 180.0 / M_PI << "deg";
    } else {
      line << "none";
    }
    line << " | " << route_status_;
    appendToLogs(line.str());
  }

  bool getRobotPose(double & x, double & y, double & yaw)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static void appendLine(const std::string & path, const std::string & line)
  {
    std::ofstream output(path, std::ios::app);
    if (output.is_open()) {
      output << line << '\n';
    }
  }

  void appendToLogs(const std::string & line) const
  {
    appendLine(
      (std::filesystem::path(session_directory_) / "navigation.log").string(),
      line);
    appendLine(latest_log_path_, line);
  }

  static std::string sanitizeTag(const std::string & tag)
  {
    std::string result = tag;
    for (char & character : result) {
      const bool valid =
        (character >= 'a' && character <= 'z') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9') || character == '_';
      if (!valid) {
        character = '_';
      }
    }
    return result.empty() ? "EVENT" : result;
  }

  std::string output_directory_;
  std::string session_directory_;
  std::string latest_log_path_;
  std::string map_frame_;
  std::string base_frame_;
  std::string route_status_;
  std::string localization_mode_{"UNKNOWN"};
  bool localization_fresh_{false};
  bool has_odom_{false};
  bool has_command_{false};
  int last_marker_id_{-1};
  int raw_marker_id_{-1};
  int snapshot_counter_{0};
  double correction_age_{-1.0};
  double distance_since_correction_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  double odom_angular_speed_{0.0};
  double command_linear_speed_{0.0};
  double command_angular_speed_{0.0};
  double raw_marker_bearing_{0.0};
  rclcpp::Time raw_marker_received_at_{0, 0, RCL_ROS_TIME};
  cv::Mat latest_image_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_fresh_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr marker_id_sub_;
  rclcpp::Subscription<aruco_localizer::msg::MarkerObservation>::SharedPtr
    marker_observation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_age_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_distance_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

}  // namespace aruco_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_localizer::NavigationDebugRecorder>());
  rclcpp::shutdown();
  return 0;
}
