import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from rclpy.executors import MultiThreadedExecutor

from ultralytics import YOLO, SAM

DEBUG_VISUALIZATION = True


class SegmentationTrackingNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self.bridge = CvBridge()
        self.current_image_msg = None
        
        # --- 1. Model Loading ---
        self.get_logger().info('Loading YOLO & SAM2 Models...')
        
        # Model file paths in package resource directory
        package_dir = os.path.dirname(os.path.dirname(__file__))
        resource_dir = os.path.join(package_dir, 'resource')
        
        yolo_path = os.path.join(resource_dir, 'yolo.pt')
        sam_path = os.path.join(resource_dir, 'sam2_t.pt')
        
        if not os.path.exists(yolo_path):
            yolo_path = 'yolo11n.pt'
        if not os.path.exists(sam_path):
            sam_path = 'sam2.1_b.pt'

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        try:
            self.yolo_model = YOLO(yolo_path)
            self.sam_model = SAM(sam_path)
            self.get_logger().info('YOLO & SAM2 models loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO/SAM2 models: {e}')

        # --- 2. State Variables ---
        self.is_tracking = False
        self.lost_frame_count = 0
        self.current_frame_idx = 0
        self.last_bbox = None

        # --- 3. ROS 2 Communication Setup ---
        # Parameterize RGB image topic (Default matching RealSense bringup: /camera/camera/color/image_raw)
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        rgb_topic = self.get_parameter('rgb_topic').as_string()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Image, rgb_topic, self.image_callback, sensor_qos)
        
        self.object_mask_publisher = self.create_publisher(Image, '/object_mask', 10)
        self.hand_mask_publisher = self.create_publisher(Image, '/hand_mask', 10)
        self.llm_vid_publisher = self.create_publisher(Image, '/llm_vid', 10)

        self.get_logger().info(f'SegmentationTrackingNode initialized. Subscribed to: {rgb_topic}')

    def image_callback(self, msg):
        try:
            self.current_image_msg = msg
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None:
                return

            annotated_image = cv_image.copy()
            hand_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            object_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

            # Process YOLO + SAM2 segmentation
            object_mask, hand_mask = self.process_segmentation(cv_image, annotated_image)

            self.publish_mask(hand_mask, self.hand_mask_publisher)
            self.publish_mask(object_mask, self.object_mask_publisher)
            
            if DEBUG_VISUALIZATION:
                self.publish_annotated_image(annotated_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def process_segmentation(self, cv_image, annotated_image):
        h, w = cv_image.shape[:2]
        object_mask = np.zeros((h, w), dtype=np.uint8)
        hand_mask = np.zeros((h, w), dtype=np.uint8)

        if self.yolo_model is None or self.sam_model is None:
            return object_mask, hand_mask

        # 1. Run YOLO object detection
        yolo_results = self.yolo_model.predict(source=cv_image, conf=0.25, verbose=False)
        bboxes = []

        if yolo_results and len(yolo_results) > 0 and yolo_results[0].boxes:
            for box in yolo_results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy()) if box.cls is not None else 0
                bboxes.append((xyxy, cls_id))

        if len(bboxes) > 0:
            # Pick best target box
            best_bbox, cls_id = bboxes[0]
            self.last_bbox = best_bbox

            # 2. Run SAM2 for mask segmentation
            try:
                sam_results = self.sam_model.predict(source=cv_image, bboxes=[best_bbox], verbose=False)
                if sam_results and len(sam_results) > 0 and sam_results[0].masks:
                    mask_data = sam_results[0].masks.data[0].cpu().numpy()
                    target_mask = (mask_data > 0.5).astype(np.uint8) * 255
                    
                    if cls_id == 0:  # Class 0: Hand / Object split
                        object_mask = target_mask
                    else:
                        object_mask = target_mask

                    if DEBUG_VISUALIZATION:
                        self.draw_mask_overlay(annotated_image, object_mask, color=(255, 0, 0))
                        self.draw_hud(annotated_image, "MODE: TRACKING (YOLO+SAM2)", (0, 255, 0))
            except Exception as e:
                self.get_logger().error(f"SAM2 prediction error: {e}")
        else:
            self.draw_hud(annotated_image, "MODE: SEARCHING (Waiting for Target)", (0, 165, 255))

        return object_mask, hand_mask

    def draw_hud(self, image, text, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_w, text_h = text_size
        
        x, y = 15, 35
        overlay = image.copy()
        cv2.rectangle(overlay, (x - 5, y - text_h - 10), (x + text_w + 5, y + 10), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        cv2.putText(image, text, (x, y), font, font_scale, color, thickness)

    def draw_mask_overlay(self, image, mask, color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, color, 2)
        colored_mask = np.zeros_like(image)
        colored_mask[mask == 255] = color
        cv2.addWeighted(colored_mask, 0.4, image, 1.0, 0, image)

    def publish_mask(self, mask_np, publisher):
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_np, "mono8")
            if self.current_image_msg:
                mask_msg.header = self.current_image_msg.header
            publisher.publish(mask_msg)
        except Exception as e:
            pass

    def publish_annotated_image(self, image_np):
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image_np, "bgr8")
            if self.current_image_msg:
                image_msg.header = self.current_image_msg.header
            self.llm_vid_publisher.publish(image_msg)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationTrackingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
