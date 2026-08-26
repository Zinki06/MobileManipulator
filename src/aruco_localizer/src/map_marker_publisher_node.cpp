#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <filesystem>

class MapMarkerPublisher : public rclcpp::Node {
public:
    MapMarkerPublisher() : Node("map_marker_publisher_node") {
        // 파라미터 선언
        this->declare_parameter<std::string>(
            "map_yaml_path", 
            "/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map.yaml"
        );
        this->declare_parameter<std::string>(
            "marker_yaml_path", 
            "/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map_markers.yaml"
        );
        this->declare_parameter<std::string>("map_frame", "map");

        map_yaml_path_ = this->get_parameter("map_yaml_path").as_string();
        marker_yaml_path_ = this->get_parameter("marker_yaml_path").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();

        // TF Static Broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Latched QoS (Transient Local) 설정 - 늦게 켜진 RViz도 즉시 맵을 수신할 수 있도록 구성
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local();
        map_qos.reliable();

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        marker_visual_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_visuals", map_qos);

        // 1. 맵 로드 및 메시지 생성
        if (loadMapData()) {
            map_pub_->publish(occupancy_grid_);
            RCLCPP_INFO(this->get_logger(), "Successfully published /map OccupancyGrid.");
        }

        // 2. 마커 YAML 로드, Static TF 및 RViz Visuals 발행
        loadAndPublishMarkers();

        // 1초 주기로 맵과 시각화 마커를 재발행하는 타이머
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                map_pub_->publish(occupancy_grid_);
                marker_visual_pub_->publish(marker_array_);
            }
        );
    }

private:
    bool loadMapData() {
        try {
            YAML::Node map_cfg = YAML::LoadFile(map_yaml_path_);
            std::string image_name = map_cfg["image"].as<std::string>();
            double resolution = map_cfg["resolution"].as<double>();
            std::vector<double> origin = map_cfg["origin"].as<std::vector<double>>();
            double occupied_thresh = map_cfg["occupied_thresh"].as<double>(0.65);
            double free_thresh = map_cfg["free_thresh"].as<double>(0.25);
            int negate = map_cfg["negate"].as<int>(0);

            // YAML 파일 기준 pgm 절대 경로 추출
            std::filesystem::path yaml_dir = std::filesystem::path(map_yaml_path_).parent_path();
            std::filesystem::path pgm_path = yaml_dir / image_name;

            cv::Mat img = cv::imread(pgm_path.string(), cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read PGM image from %s", pgm_path.string().c_str());
                return false;
            }

            // ROS OccupancyGrid는 좌하단(Bottom-Left) 기준이므로 이미지 상하 반전 필요
            cv::Mat flipped_img;
            cv::flip(img, flipped_img, 0);

            occupancy_grid_.header.stamp = this->now();
            occupancy_grid_.header.frame_id = map_frame_;
            occupancy_grid_.info.map_load_time = this->now();
            occupancy_grid_.info.resolution = resolution;
            occupancy_grid_.info.width = flipped_img.cols;
            occupancy_grid_.info.height = flipped_img.rows;
            occupancy_grid_.info.origin.position.x = origin[0];
            occupancy_grid_.info.origin.position.y = origin[1];
            occupancy_grid_.info.origin.position.z = origin[2];

            // Origin yaw 각도 적용
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, origin.size() > 2 ? origin[2] : 0.0);
            occupancy_grid_.info.origin.orientation = tf2::toMsg(q);

            occupancy_grid_.data.resize(flipped_img.rows * flipped_img.cols);

            for (int r = 0; r < flipped_img.rows; ++r) {
                for (int c = 0; c < flipped_img.cols; ++c) {
                    uint8_t pixel = flipped_img.at<uint8_t>(r, c);
                    double occ = negate ? (pixel / 255.0) : ((255.0 - pixel) / 255.0);
                    int8_t cost;

                    if (occ > occupied_thresh) {
                        cost = 100; // 장애물 (검은색)
                    } else if (occ < free_thresh) {
                        cost = 0;   // 자유 영역 (흰색)
                    } else {
                        cost = -1;  // 미지의 영역 (회색)
                    }
                    occupancy_grid_.data[r * flipped_img.cols + c] = cost;
                }
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing map yaml: %s", e.what());
            return false;
        }
    }

    void loadAndPublishMarkers() {
        try {
            YAML::Node marker_cfg = YAML::LoadFile(marker_yaml_path_);
            std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

            int text_id = 1000;
            for (const auto& m : marker_cfg["markers"]) {
                int id = m["id"].as<int>();
                double size = m["size"].as<double>();
                double x = m["x"].as<double>();
                double y = m["y"].as<double>();
                double yaw_deg = m["yaw"].as<double>();
                double yaw_rad = yaw_deg * M_PI / 180.0;

                // 1. Static TF (map -> marker_ID)
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = this->now();
                tf_msg.header.frame_id = map_frame_;
                tf_msg.child_frame_id = "marker_" + std::to_string(id);
                tf_msg.transform.translation.x = x;
                tf_msg.transform.translation.y = y;
                tf_msg.transform.translation.z = 0.005; // 맵 위에 살짝 띄움

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw_rad);
                tf_msg.transform.rotation = tf2::toMsg(q);
                static_transforms.push_back(tf_msg);

                // 2. RViz 시각화용 사각 마커 판 (Black Box)
                visualization_msgs::msg::Marker cube;
                cube.header.frame_id = map_frame_;
                cube.header.stamp = this->now();
                cube.ns = "aruco_plates";
                cube.id = id;
                cube.type = visualization_msgs::msg::Marker::CUBE;
                cube.action = visualization_msgs::msg::Marker::ADD;
                cube.pose.position.x = x;
                cube.pose.position.y = y;
                cube.pose.position.z = 0.002;
                cube.pose.orientation = tf2::toMsg(q);
                cube.scale.x = size;
                cube.scale.y = size;
                cube.scale.z = 0.004;
                cube.color.r = 0.1f;
                cube.color.g = 0.1f;
                cube.color.b = 0.1f;
                cube.color.a = 1.0f;
                marker_array_.markers.push_back(cube);

                // 3. RViz 시각화용 ID 텍스트
                visualization_msgs::msg::Marker text;
                text.header.frame_id = map_frame_;
                text.header.stamp = this->now();
                text.ns = "aruco_labels";
                text.id = text_id++;
                text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::msg::Marker::ADD;
                text.pose.position.x = x;
                text.pose.position.y = y;
                text.pose.position.z = 0.15; // 마커 상단에 ID 표시
                text.scale.z = 0.12;        // 글자 크기
                text.color.r = 1.0f;
                text.color.g = 0.2f;
                text.color.b = 0.2f;
                text.color.a = 1.0f;
                text.text = "ID: " + std::to_string(id);
                marker_array_.markers.push_back(text);
            }

            // Static TF 브로드캐스트
            tf_static_broadcaster_->sendTransform(static_transforms);
            RCLCPP_INFO(this->get_logger(), "Broadcasted %zu static TFs for markers.", static_transforms.size());

            // 마커 시각화 토픽 1회 발행
            marker_visual_pub_->publish(marker_array_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing marker yaml: %s", e.what());
        }
    }

    std::string map_yaml_path_;
    std::string marker_yaml_path_;
    std::string map_frame_;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    visualization_msgs::msg::MarkerArray marker_array_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_visual_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}