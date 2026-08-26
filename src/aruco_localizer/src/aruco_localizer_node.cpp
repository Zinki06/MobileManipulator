#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>

struct MarkerInfo {
    double size;
    Eigen::Isometry3d T_map_marker;
};

class ArucoLocalizer : public rclcpp::Node {
public:
    ArucoLocalizer() : Node("aruco_localizer_node"), has_cam_info_(false), has_valid_map_odom_(false) {
        // 1. 파라미터 선언 및 취득
        this->declare_parameter<std::string>(
            "marker_yaml_path", 
            "/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map_markers.yaml"
        );
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");

        std::string yaml_path = this->get_parameter("marker_yaml_path").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();

        loadMarkerYaml(yaml_path);

        // 2. ArUco 사전 및 파라미터 구성
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // 3. TF 버퍼, 리스너, 브로드캐스터 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. ROS 구독자 생성
        auto qos = rclcpp::SensorDataQoS();
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", qos,
            std::bind(&ArucoLocalizer::cameraInfoCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", qos,
            std::bind(&ArucoLocalizer::imageCallback, this, std::placeholders::_1));

        // 5. 30Hz 주기로 map -> odom TF 지속 브로드캐스트 (마커 유실 시에도 TF 유지)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArucoLocalizer::publishMapToOdom, this));

        RCLCPP_INFO(this->get_logger(), "Aruco Localization Node successfully initialized.");
    }

private:
    void loadMarkerYaml(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            for (const auto& m : config["markers"]) {
                int id = m["id"].as<int>();
                double size = m["size"].as<double>();
                double x = m["x"].as<double>();
                double y = m["y"].as<double>();
                double yaw_deg = m["yaw"].as<double>();
                double yaw_rad = yaw_deg * M_PI / 180.0;

                Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
                Eigen::AngleAxisd roll(0.0, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitch(0.0, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yaw(yaw_rad, Eigen::Vector3d::UnitZ());
                T.rotate(yaw * pitch * roll);
                T.translation() = Eigen::Vector3d(x, y, 0.0);

                markers_[id] = {size, T};
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu markers from YAML.", markers_.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!has_cam_info_) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
            has_cam_info_ = true;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_cam_info_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids, detector_params_, rejected_candidates);

        if (marker_ids.empty()) return;

        // optical_frame -> base_link TF 조회
        Eigen::Isometry3d T_cam_to_base;
        try {
            auto tf_cam_msg = tf_buffer_->lookupTransform(
                msg->header.frame_id, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
            T_cam_to_base = tf2::transformToEigen(tf_cam_msg);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Cam->Base TF Lookup Failed: %s", ex.what());
            return;
        }

        // odom -> base_link TF 조회
        Eigen::Isometry3d T_odom_to_base;
        try {
            auto tf_odom_msg = tf_buffer_->lookupTransform(
                odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
            T_odom_to_base = tf2::transformToEigen(tf_odom_msg);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odom->Base TF Lookup Failed: %s", ex.what());
            return;
        }

        double total_weight = 0.0;
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        double weighted_cos = 0.0;
        double weighted_sin = 0.0;
        int valid_marker_count = 0;

        for (size_t i = 0; i < marker_ids.size(); ++i) {
            int id = marker_ids[i];
            if (markers_.find(id) == markers_.end()) continue;

            const auto& marker = markers_[id];
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<std::vector<cv::Point2f>> single_corner = {marker_corners[i]};

            cv::aruco::estimatePoseSingleMarkers(single_corner, marker.size, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            double dist = std::sqrt(tvecs[0][0]*tvecs[0][0] + tvecs[0][1]*tvecs[0][1] + tvecs[0][2]*tvecs[0][2]);
            if (dist < 0.1) continue;

            double weight = 1.0 / (dist * dist);

            cv::Mat R_cv;
            cv::Rodrigues(rvecs[0], R_cv);
            Eigen::Matrix3d R_eigen;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R_eigen(r, c) = R_cv.at<double>(r, c);
                }
            }

            Eigen::Isometry3d T_cam_to_marker = Eigen::Isometry3d::Identity();
            T_cam_to_marker.linear() = R_eigen;
            T_cam_to_marker.translation() = Eigen::Vector3d(tvecs[0][0], tvecs[0][1], tvecs[0][2]);

            // T_map_to_base = T_map_marker * inv(T_cam_to_marker) * T_cam_to_base
            Eigen::Isometry3d T_marker_to_cam = T_cam_to_marker.inverse();
            Eigen::Isometry3d T_map_to_base = marker.T_map_marker * T_marker_to_cam * T_cam_to_base;

            // T_map_to_odom = T_map_to_base * inv(T_odom_to_base)
            Eigen::Isometry3d T_map_to_odom_single = T_map_to_base * T_odom_to_base.inverse();

            double odom_x = T_map_to_odom_single.translation().x();
            double odom_y = T_map_to_odom_single.translation().y();
            double odom_yaw = std::atan2(T_map_to_odom_single.rotation()(1, 0), T_map_to_odom_single.rotation()(0, 0));

            weighted_x += weight * odom_x;
            weighted_y += weight * odom_y;
            weighted_cos += weight * std::cos(odom_yaw);
            weighted_sin += weight * std::sin(odom_yaw);
            total_weight += weight;
            valid_marker_count++;
        }

        if (valid_marker_count == 0 || total_weight <= 0.0) return;

        // 가중 평균 및 원형 각도 평균 계산
        double avg_x = weighted_x / total_weight;
        double avg_y = weighted_y / total_weight;
        double avg_yaw = std::atan2(weighted_sin, weighted_cos);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, avg_yaw);

        latest_tf_map_odom_.header.frame_id = map_frame_;
        latest_tf_map_odom_.child_frame_id = odom_frame_;
        latest_tf_map_odom_.transform.translation.x = avg_x;
        latest_tf_map_odom_.transform.translation.y = avg_y;
        latest_tf_map_odom_.transform.translation.z = 0.0;
        latest_tf_map_odom_.transform.rotation = tf2::toMsg(q);

        has_valid_map_odom_ = true;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "[Tracking] Fused %d markers | Map->Odom Offset: x = %.3f m, y = %.3f m, yaw = %.2f deg",
            valid_marker_count, avg_x, avg_y, avg_yaw * 180.0 * M_1_PI
        );
    }

    void publishMapToOdom() {
        if (!has_valid_map_odom_) return;

        latest_tf_map_odom_.header.stamp = this->get_clock()->now();
        tf_broadcaster_->sendTransform(latest_tf_map_odom_);
    }

    // 멤버 변수
    std::string base_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    bool has_cam_info_;
    bool has_valid_map_odom_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::unordered_map<int, MarkerInfo> markers_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped latest_tf_map_odom_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
