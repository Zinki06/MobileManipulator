#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

class RealSenseTfNode : public rclcpp::Node
{
private:
  using ObjectMaskSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

public:
  RealSenseTfNode()
  : Node("realsense_tf_node"),
    camera_info_received_(false)
  {
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<std::string>(
      "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>(
      "info_topic", "/camera/camera/color/camera_info");
    this->declare_parameter<std::string>(
      "rgb_topic", "/camera/camera/color/image_raw");
    this->declare_parameter<std::string>("obj_mask_topic", "/object_mask");

    target_frame_ = this->get_parameter("target_frame").as_string();
    const std::string depth_topic = this->get_parameter("depth_topic").as_string();
    const std::string info_topic = this->get_parameter("info_topic").as_string();
    const std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    const std::string obj_mask_topic = this->get_parameter("obj_mask_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    depth_sub_.subscribe(this, depth_topic);
    info_sub_.subscribe(this, info_topic);
    rgb_sub_.subscribe(this, rgb_topic);
    obj_mask_sub_.subscribe(this, obj_mask_topic);

    constexpr int queue_size = 5;
    sync_ = std::make_shared<message_filters::Synchronizer<ObjectMaskSyncPolicy>>(
      ObjectMaskSyncPolicy(queue_size),
      depth_sub_, info_sub_, rgb_sub_, obj_mask_sub_);

    sync_->registerCallback(
      std::bind(
        &RealSenseTfNode::sync_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4));

    object_pc_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("object_pointcloud", 10);
    object_centroid_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("object_centroid", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "RealSense TF Node started. Synchronizing Depth, RGB, and Object Mask. Target Frame: %s",
      target_frame_.c_str());
  }

private:
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & obj_mask_msg)
  {
    update_camera_info(info_msg);
    if (!camera_info_received_) {
      return;
    }

    try {
      cv::Mat depth_img = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      cv::Mat rgb_img = cv_bridge::toCvCopy(
        rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat obj_mask = cv_bridge::toCvCopy(
        obj_mask_msg, sensor_msgs::image_encodings::MONO8)->image;

      if (depth_img.size() != obj_mask.size()) {
        cv::resize(
          obj_mask, obj_mask, depth_img.size(), 0, 0, cv::INTER_NEAREST);
      }
      if (depth_img.size() != rgb_img.size()) {
        cv::resize(
          rgb_img, rgb_img, depth_img.size(), 0, 0, cv::INTER_LINEAR);
      }

      process_and_publish(depth_img, obj_mask, rgb_img, depth_msg->header);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Callback error: %s", e.what());
    }
  }

  void process_and_publish(
    const cv::Mat & depth_img,
    const cv::Mat & mask_img,
    const cv::Mat & rgb_img,
    const std_msgs::msg::Header & header)
  {
    auto cloud_cam = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    for (int v = 0; v < depth_img.rows; ++v) {
      const uint16_t * d_ptr = depth_img.ptr<uint16_t>(v);
      const uint8_t * m_ptr = mask_img.ptr<uint8_t>(v);
      const cv::Vec3b * r_ptr = rgb_img.ptr<cv::Vec3b>(v);

      for (int u = 0; u < depth_img.cols; ++u) {
        if (m_ptr[u] <= 128) {
          continue;
        }

        const uint16_t depth_raw = d_ptr[u];
        if (depth_raw <= 100 || depth_raw >= 4000) {
          continue;
        }

        const float depth_m = static_cast<float>(depth_raw) / 1000.0f;
        pcl::PointXYZRGB point;

        point.z = depth_m;
        point.x = (static_cast<float>(u) - cx_) * depth_m / fx_;
        point.y = (static_cast<float>(v) - cy_) * depth_m / fy_;

        point.b = r_ptr[u][0];
        point.g = r_ptr[u][1];
        point.r = r_ptr[u][2];

        if (std::isfinite(point.x) &&
            std::isfinite(point.y) &&
            std::isfinite(point.z)) {
          cloud_cam->points.push_back(point);
        }
      }
    }

    if (cloud_cam->empty()) {
      return;
    }

    auto cloud_downsampled =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_cam);
    voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel_grid.filter(*cloud_downsampled);

    if (cloud_downsampled->empty()) {
      return;
    }

    auto cloud_ror = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloud_downsampled);
    ror.setRadiusSearch(0.05f);
    ror.setMinNeighborsInRadius(10);
    ror.filter(*cloud_ror);

    if (cloud_ror->empty()) {
      return;
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(cloud_ror);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extractor;
    cluster_extractor.setClusterTolerance(0.08f);
    cluster_extractor.setMinClusterSize(10);
    cluster_extractor.setMaxClusterSize(static_cast<int>(cloud_ror->size()));
    cluster_extractor.setSearchMethod(tree);
    cluster_extractor.setInputCloud(cloud_ror);
    cluster_extractor.extract(cluster_indices);

    if (cluster_indices.empty()) {
      return;
    }

    const auto largest_cluster = std::max_element(
      cluster_indices.begin(), cluster_indices.end(),
      [](const pcl::PointIndices & a, const pcl::PointIndices & b) {
        return a.indices.size() < b.indices.size();
      });

    auto cloud_filtered =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_filtered->points.reserve(largest_cluster->indices.size());

    for (const int index : largest_cluster->indices) {
      cloud_filtered->points.push_back(cloud_ror->points[index]);
    }

    cloud_filtered->width = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = true;

    if (cloud_filtered->empty()) {
      return;
    }

    auto cloud_transformed =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    try {
      const geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer_->lookupTransform(
          target_frame_, header.frame_id, tf2::TimePointZero);

      const Eigen::Matrix4f tf_matrix =
        tf2::transformToEigen(tf_msg).matrix().cast<float>();
      pcl::transformPointCloud(
        *cloud_filtered, *cloud_transformed, tf_matrix);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF Error transforming %s to %s: %s",
        header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_transformed, output_msg);
    output_msg.header.stamp = header.stamp;
    output_msg.header.frame_id = target_frame_;
    object_pc_pub_->publish(output_msg);

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (const auto & point : cloud_transformed->points) {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }

    const double point_count =
      static_cast<double>(cloud_transformed->points.size());
    const double centroid_x = sum_x / point_count;
    const double centroid_y = sum_y / point_count;
    const double centroid_z = sum_z / point_count;

    geometry_msgs::msg::PointStamped centroid_msg;
    centroid_msg.header.stamp = header.stamp;
    centroid_msg.header.frame_id = target_frame_;
    centroid_msg.point.x = centroid_x;
    centroid_msg.point.y = centroid_y;
    centroid_msg.point.z = centroid_z;
    object_centroid_pub_->publish(centroid_msg);

    geometry_msgs::msg::TransformStamped object_tf;
    object_tf.header.stamp = header.stamp;
    object_tf.header.frame_id = target_frame_;
    object_tf.child_frame_id = "object_frame";
    object_tf.transform.translation.x = centroid_x;
    object_tf.transform.translation.y = centroid_y;
    object_tf.transform.translation.z = centroid_z;
    object_tf.transform.rotation.x = 0.0;
    object_tf.transform.rotation.y = 0.0;
    object_tf.transform.rotation.z = 0.0;
    object_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(object_tf);
  }

  void update_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    if (!camera_info_received_ && info_msg->k[0] != 0.0) {
      fx_ = static_cast<float>(info_msg->k[0]);
      fy_ = static_cast<float>(info_msg->k[4]);
      cx_ = static_cast<float>(info_msg->k[2]);
      cy_ = static_cast<float>(info_msg->k[5]);
      camera_info_received_ = true;
    }
  }

  std::string target_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> obj_mask_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

  std::shared_ptr<message_filters::Synchronizer<ObjectMaskSyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_centroid_pub_;

  bool camera_info_received_;
  float fx_ = 0.0f;
  float fy_ = 0.0f;
  float cx_ = 0.0f;
  float cy_ = 0.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealSenseTfNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}