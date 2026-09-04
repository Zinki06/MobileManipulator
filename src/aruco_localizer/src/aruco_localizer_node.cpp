#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
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
#include <limits>

struct MarkerInfo {
    double size;
    Eigen::Isometry3d T_map_marker;
};

class ArucoLocalizer : public rclcpp::Node {
public:
    ArucoLocalizer()
    : Node("aruco_localizer_node"),
      has_cam_info_(false),
      has_valid_map_odom_(false),
      relocalization_candidate_count_(0),
      last_valid_marker_id_(-1)
    {
        // 1. 파라미터 선언 및 취득
        this->declare_parameter<std::string>(
            "marker_yaml_path",
            "/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map_markers.yaml"
        );
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<double>("filter_alpha", 0.15); // EMA 필터 계수 (0.05 ~ 0.3)
        this->declare_parameter<double>("max_pos_jump", 0.40);  // 이상치 판정 위치 오차 (단위: m)
        this->declare_parameter<double>("max_yaw_jump", 0.50);  // 이상치 판정 회전각 오차 (단위: rad, 약 28도)
        this->declare_parameter<double>("marker_freshness_timeout", 1.0);
        this->declare_parameter<int>("relocalization_consistency_count", 5);
        this->declare_parameter<double>("min_marker_dist", 0.40); // 40cm 이내 근접 시 왜곡 방지 차단
        this->declare_parameter<double>("max_marker_dist", 2.50);

        std::string yaml_path = this->get_parameter("marker_yaml_path").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        filter_alpha_ = this->get_parameter("filter_alpha").as_double();
        max_pos_jump_ = this->get_parameter("max_pos_jump").as_double();
        max_yaw_jump_ = this->get_parameter("max_yaw_jump").as_double();
        marker_freshness_timeout_ = this->get_parameter("marker_freshness_timeout").as_double();
        relocalization_consistency_count_ =
            this->get_parameter("relocalization_consistency_count").as_int();
        min_marker_dist_ = this->get_parameter("min_marker_dist").as_double();
        max_marker_dist_ = this->get_parameter("max_marker_dist").as_double();

        loadMarkerYaml(yaml_path);

        // 2. ArUco 사전 및 서브픽셀 코너 검출 파라미터 구성 (지터 억제)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.05;

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

        localization_fresh_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/aruco/localization_fresh", 10);
        last_marker_id_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/aruco/last_marker_id", 10);

        // 5. 30Hz 주기로 map -> odom TF 지속 브로드캐스트
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArucoLocalizer::publishMapToOdom, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Aruco Localization Node Initialized. (EMA Filter Alpha: %.2f, SubPix: Enabled)",
            filter_alpha_
        );
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

        // 1. 카메라 시점 타임스탬프 기반 TF 동기화 조회
        Eigen::Isometry3d T_cam_to_base;
        try {
            auto tf_cam_msg = tf_buffer_->lookupTransform(
                msg->header.frame_id, base_frame_, msg->header.stamp, tf2::durationFromSec(0.05));
            T_cam_to_base = tf2::transformToEigen(tf_cam_msg);
        } catch (const tf2::TransformException&) {
            // 타임스탬프 조회 실패 시 최신(TimePointZero)으로 폴백
            try {
                auto tf_cam_msg = tf_buffer_->lookupTransform(
                    msg->header.frame_id, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
                T_cam_to_base = tf2::transformToEigen(tf_cam_msg);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Cam->Base TF Lookup Failed: %s", ex.what());
                return;
            }
        }

        // 2. 오도메트리 TF 동기화 조회
        Eigen::Isometry3d T_odom_to_base;
        try {
            auto tf_odom_msg = tf_buffer_->lookupTransform(
                odom_frame_, base_frame_, msg->header.stamp, tf2::durationFromSec(0.05));
            T_odom_to_base = tf2::transformToEigen(tf_odom_msg);
        } catch (const tf2::TransformException&) {
            try {
                auto tf_odom_msg = tf_buffer_->lookupTransform(
                    odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
                T_odom_to_base = tf2::transformToEigen(tf_odom_msg);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odom->Base TF Lookup Failed: %s", ex.what());
                return;
            }
        }

        double total_weight = 0.0;
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        double weighted_cos = 0.0;
        double weighted_sin = 0.0;
        int valid_marker_count = 0;
        int nearest_marker_id = -1;
        double nearest_marker_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < marker_ids.size(); ++i) {
            int id = marker_ids[i];
            if (markers_.find(id) == markers_.end()) continue;

            const auto& marker = markers_[id];
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<std::vector<cv::Point2f>> single_corner = {marker_corners[i]};

            cv::aruco::estimatePoseSingleMarkers(single_corner, marker.size, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            double dist = std::sqrt(tvecs[0][0]*tvecs[0][0] + tvecs[0][1]*tvecs[0][1] + tvecs[0][2]*tvecs[0][2]);
            // 거리 기반 유효성 검사 (0.4m 이내 근접 시 왜곡이 크므로 오도메트리에 위임, 2.5m 이상 제외)
            if (dist < min_marker_dist_ || dist > max_marker_dist_) continue;

            if (dist < nearest_marker_dist) {
                nearest_marker_dist = dist;
                nearest_marker_id = id;
            }

            // 가까울수록 기하급수적으로 높은 가중치 부여 (1 / dist^2)
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

        // 마커 측정 가중 평균 계산
        double meas_x = weighted_x / total_weight;
        double meas_y = weighted_y / total_weight;
        double meas_yaw = std::atan2(weighted_sin, weighted_cos);

        Eigen::Vector2d meas_pos(meas_x, meas_y);
        Eigen::Quaterniond meas_q(Eigen::AngleAxisd(meas_yaw, Eigen::Vector3d::UnitZ()));

        // 3. 이상치(Outlier) 검사 및 EMA(Exponential Moving Average) 필터링
        if (!has_valid_map_odom_) {
            // 첫 초기화
            smoothed_pos_ = meas_pos;
            smoothed_q_ = meas_q;
            has_valid_map_odom_ = true;
            relocalization_candidate_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "[Initial Lock] Map->Odom initialized: (%.3f, %.3f, %.1f°)",
                meas_x, meas_y, meas_yaw * 180.0 / M_PI);
        } else {
            // 이전 추정치 대비 점프 크기 검사
            double pos_diff = (meas_pos - smoothed_pos_).norm();
            double angle_diff = std::abs(meas_q.angularDistance(smoothed_q_));

            if (pos_diff > max_pos_jump_ || angle_diff > max_yaw_jump_) {
                // 같은 위치에서 반복되는 큰 보정만 재정합으로 인정한다.
                const bool candidate_is_consistent =
                    relocalization_candidate_count_ > 0 &&
                    (meas_pos - relocalization_candidate_pos_).norm() < 0.15 &&
                    std::abs(meas_q.angularDistance(relocalization_candidate_q_)) < 0.20;

                if (candidate_is_consistent) {
                    relocalization_candidate_pos_ =
                        0.5 * relocalization_candidate_pos_ + 0.5 * meas_pos;
                    relocalization_candidate_q_ =
                        relocalization_candidate_q_.slerp(0.5, meas_q).normalized();
                    relocalization_candidate_count_++;
                } else {
                    relocalization_candidate_pos_ = meas_pos;
                    relocalization_candidate_q_ = meas_q;
                    relocalization_candidate_count_ = 1;
                }

                if (relocalization_candidate_count_ >= relocalization_consistency_count_) {
                    smoothed_pos_ = relocalization_candidate_pos_;
                    smoothed_q_ = relocalization_candidate_q_;
                    relocalization_candidate_count_ = 0;
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[Relocalized] Accepted %d consistent ArUco measurements.",
                        relocalization_consistency_count_);
                } else {
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "Outlier pending: pos=%.3f yaw=%.2f rad consistency=%d/%d",
                        pos_diff, angle_diff, relocalization_candidate_count_,
                        relocalization_consistency_count_);
                    return;
                }
            } else {
                relocalization_candidate_count_ = 0;
                // EMA 저주파 필터 (지터 90% 이상 제거)
                smoothed_pos_ = (1.0 - filter_alpha_) * smoothed_pos_ + filter_alpha_ * meas_pos;
                smoothed_q_ = smoothed_q_.slerp(filter_alpha_, meas_q);
                smoothed_q_.normalize();
            }
        }

        // 최신 평활화된 TF 업데이트
        latest_tf_map_odom_.header.frame_id = map_frame_;
        latest_tf_map_odom_.child_frame_id = odom_frame_;
        latest_tf_map_odom_.transform.translation.x = smoothed_pos_.x();
        latest_tf_map_odom_.transform.translation.y = smoothed_pos_.y();
        latest_tf_map_odom_.transform.translation.z = 0.0;
        latest_tf_map_odom_.transform.rotation.x = smoothed_q_.x();
        latest_tf_map_odom_.transform.rotation.y = smoothed_q_.y();
        latest_tf_map_odom_.transform.rotation.z = smoothed_q_.z();
        latest_tf_map_odom_.transform.rotation.w = smoothed_q_.w();

        last_valid_marker_time_ = this->now();
        last_valid_marker_id_ = nearest_marker_id;
        publishLocalizationStatus(true);

        double cur_yaw = std::atan2(
            2.0 * (smoothed_q_.w() * smoothed_q_.z() + smoothed_q_.x() * smoothed_q_.y()),
            1.0 - 2.0 * (smoothed_q_.y() * smoothed_q_.y() + smoothed_q_.z() * smoothed_q_.z()));

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[Aruco Track] Fused %d markers | Map->Odom: x=%.3f m, y=%.3f m, yaw=%.1f°",
            valid_marker_count, smoothed_pos_.x(), smoothed_pos_.y(), cur_yaw * 180.0 / M_PI
        );
    }

    void publishMapToOdom() {
        bool localization_fresh = false;
        if (has_valid_map_odom_) {
            localization_fresh =
                (this->now() - last_valid_marker_time_).seconds() <= marker_freshness_timeout_;
        }
        publishLocalizationStatus(localization_fresh);

        if (!has_valid_map_odom_) return;

        latest_tf_map_odom_.header.stamp = this->get_clock()->now();
        tf_broadcaster_->sendTransform(latest_tf_map_odom_);
    }

    void publishLocalizationStatus(bool fresh) {
        std_msgs::msg::Bool fresh_msg;
        fresh_msg.data = fresh;
        localization_fresh_pub_->publish(fresh_msg);

        if (fresh && last_valid_marker_id_ >= 0) {
            std_msgs::msg::Int32 marker_msg;
            marker_msg.data = last_valid_marker_id_;
            last_marker_id_pub_->publish(marker_msg);
        }
    }

    // 멤버 변수
    std::string base_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    double filter_alpha_;
    double max_pos_jump_;
    double max_yaw_jump_;
    double marker_freshness_timeout_;
    int relocalization_consistency_count_;
    double min_marker_dist_;
    double max_marker_dist_;

    bool has_cam_info_;
    bool has_valid_map_odom_;
    int relocalization_candidate_count_;
    int last_valid_marker_id_;

    Eigen::Vector2d smoothed_pos_;
    Eigen::Quaterniond smoothed_q_;
    Eigen::Vector2d relocalization_candidate_pos_;
    Eigen::Quaterniond relocalization_candidate_q_;
    rclcpp::Time last_valid_marker_time_;

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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_fresh_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr last_marker_id_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
