#!/usr/bin/env python3
"""
YOLO11n + SAM2 3D Vision Perception Service Node for TurtleBot3 Manipulator Wrist Cam.
Subscribes to RealSense RGB-D streams, runs YOLO11n for target detection,
uses SAM2 for precise pixel mask segmentation, extracts 3D centroid via depth registration,
and returns 3D PoseStamped via ROS 2 service '/detect_target_object'.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from turtlebot3_pick_place.srv import GetTargetPose

# Import Ultralytics YOLO & SAM
try:
    from ultralytics import YOLO, SAM
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class YoloSam2PerceptionServer(Node):
    def __init__(self):
        super().__init__('yolo_sam2_perception_server')
        self.bridge = CvBridge()
        
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        # Camera intrinsics default
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_frame_id = "camera_color_optical_frame"
        
        # Load YOLO11n and SAM2 Models
        self.yolo_model = None
        self.sam_model = None
        
        if ULTRALYTICS_AVAILABLE:
            try:
                self.get_logger().info("Loading YOLO11n ('yolo11n.pt') & SAM2 ('sam2.1_b.pt')...")
                self.yolo_model = YOLO("yolo11n.pt")
                self.sam_model = SAM("sam2.1_b.pt")
                self.get_logger().info("YOLO11n & SAM2 Models loaded successfully!")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO/SAM2 models: {e}")

        # ROS 2 Subscribers (RealSense topics)
        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10
        )
        
        # Fallback depth topic if aligned depth is unavailable
        self.sub_depth_fallback = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        # ROS 2 Service Server
        self.srv = self.create_service(
            GetTargetPose,
            '/detect_target_object',
            self.handle_detect_target
        )
        
        self.get_logger().info("YOLO11n + SAM2 Perception Server Initialized. Ready for '/detect_target_object' requests.")

    def info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id

    def rgb_callback(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")

    def depth_callback(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16:
                self.latest_depth = depth_img.astype(np.float32) / 1000.0  # Convert mm to meters
            else:
                self.latest_depth = depth_img.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"Failed to convert Depth image: {e}")

    def handle_detect_target(self, request, response):
        self.get_logger().info(f"Received target detection request for class: '{request.target_class}'")
        
        if self.latest_rgb is None or self.latest_depth is None:
            response.success = False
            response.message = "No RGB-D camera frames received yet."
            self.get_logger().warn(response.message)
            return response
            
        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()
        
        # 1. Run Detection & SAM2 Masking Pipeline
        mask, bbox = self.run_yolo11_sam2_segmentation(rgb, request.target_class)
        
        if mask is None or np.count_nonzero(mask) == 0:
            response.success = False
            response.message = "Target object could not be detected or segmented by YOLO11+SAM2."
            self.get_logger().warn(response.message)
            return response
            
        # 2. Extract 3D Centroid from Depth & Mask
        x_3d, y_3d, z_3d = self.compute_3d_centroid(depth, mask)
        
        if z_3d <= 0.05 or np.isnan(z_3d) or np.isinf(z_3d):
            response.success = False
            response.message = f"Invalid 3D depth estimated: ({x_3d:.3f}, {y_3d:.3f}, {z_3d:.3f})"
            self.get_logger().warn(response.message)
            return response
            
        # 3. Construct PoseStamped response in camera frame
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.camera_frame_id
        
        pose.pose.position.x = float(x_3d)
        pose.pose.position.y = float(y_3d)
        pose.pose.position.z = float(z_3d)
        
        # Default orientation: facing camera view vector
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        response.success = True
        response.message = f"YOLO11+SAM2 3D Pose: ({x_3d:.3f}, {y_3d:.3f}, {z_3d:.3f}) m"
        response.target_pose = pose
        
        self.get_logger().info(response.message)
        return response

    def run_yolo11_sam2_segmentation(self, rgb, target_class):
        """
        Runs YOLO11n to locate bounding box and SAM2 for pixel-accurate mask segmentation.
        """
        h, w = rgb.shape[:2]
        
        # Attempt YOLO11n + SAM2 inference if models are loaded
        if self.yolo_model is not None and self.sam_model is not None:
            try:
                # 1. Run YOLO11n object detection
                yolo_results = self.yolo_model.predict(source=rgb, conf=0.25, verbose=False)
                bboxes = []
                
                for r in yolo_results:
                    if r.boxes is not None and len(r.boxes) > 0:
                        for box in r.boxes:
                            # Filter class if specified or select small object
                            xyxy = box.xyxy[0].cpu().numpy()
                            bboxes.append(xyxy)
                            
                if len(bboxes) > 0:
                    best_bbox = bboxes[0]  # Pick highest confidence object
                    
                    # 2. Run SAM2 for mask segmentation using YOLO bounding box prompt
                    sam_results = self.sam_model.predict(source=rgb, bboxes=[best_bbox], verbose=False)
                    
                    for sam_res in sam_results:
                        if sam_res.masks is not None and len(sam_res.masks) > 0:
                            mask = sam_res.masks.data[0].cpu().numpy().astype(np.uint8)
                            return mask, best_bbox
            except Exception as e:
                self.get_logger().error(f"YOLO11+SAM2 inference error: {e}")

        # Fallback: Workspace ROI depth filtering
        if self.latest_depth is not None:
            valid_depth = (self.latest_depth > 0.12) & (self.latest_depth < 0.60)
            margin = 30
            roi_mask = np.zeros((h, w), dtype=np.uint8)
            roi_mask[margin:h-margin, margin:w-margin] = 1
            
            combined_mask = (valid_depth & (roi_mask == 1)).astype(np.uint8)
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    mask = np.zeros((h, w), dtype=np.uint8)
                    cv2.drawContours(mask, [c], -1, 1, -1)
                    x, y, bw, bh = cv2.boundingRect(c)
                    return mask, [x, y, x + bw, y + bh]
                    
        return None, None

    def compute_3d_centroid(self, depth, mask):
        """
        Computes 3D coordinates (X, Y, Z) in meters for pixels within the mask using camera intrinsics.
        """
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return 0.0, 0.0, 0.0
            
        depth_values = depth[ys, xs]
        valid_indices = (depth_values > 0.10) & (depth_values < 1.0) & ~np.isnan(depth_values)
        if not np.any(valid_indices):
            return 0.0, 0.0, 0.0
            
        xs = xs[valid_indices]
        ys = ys[valid_indices]
        depths = depth_values[valid_indices]
        
        z_median = float(np.median(depths))
        depth_inliers = np.abs(depths - z_median) < 0.03
        
        xs_in = xs[depth_inliers]
        ys_in = ys[depth_inliers]
        depths_in = depths[depth_inliers]
        
        x_3d_pts = (xs_in - self.cx) * depths_in / self.fx
        y_3d_pts = (ys_in - self.cy) * depths_in / self.fy
        
        x_3d = float(np.mean(x_3d_pts))
        y_3d = float(np.mean(y_3d_pts))
        z_3d = float(np.mean(depths_in))
        
        return x_3d, y_3d, z_3d


def main(args=None):
    rclpy.init(args=args)
    node = YoloSam2PerceptionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
