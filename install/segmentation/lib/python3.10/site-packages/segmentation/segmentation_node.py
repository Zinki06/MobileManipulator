import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch

from ultralytics import YOLO
from efficient_track_anything.build_efficienttam import (
    build_efficienttam_camera_predictor,
)


DEBUG_VISUALIZATION = True


class EfficientTAMObjectTrackingNode(Node):
    def __init__(self):
        super().__init__("efficienttam_object_tracking_node")

        self.bridge = CvBridge()
        self.current_image_msg = None

        # =========================================================
        # 1. 모델 로드
        # =========================================================
        self.get_logger().info("Loading YOLO and EfficientTAM models...")

        self.yolo_model = YOLO(
            "/home/user/turtlebot3_ws/src/segmentation/best.pt"
        )

        tam_checkpoint = (
            "/home/user/REALTIME_SAM2/checkpoints/"
            "efficienttam_ti_512x512.pt"
        )
        model_cfg = "configs/efficienttam/efficienttam_ti_512x512.yaml"

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.predictor = build_efficienttam_camera_predictor(
            model_cfg,
            tam_checkpoint,
            device=self.device,
        )

        # 추적 대상은 물체 하나뿐
        self.num_classes = 1

        # =========================================================
        # 2. 추적 상태
        # =========================================================
        self.is_tracking = False
        self.lost_frame_count = 0
        self.current_frame_idx = 0

        # 연속 N프레임 동안 마스크가 사라지면 YOLO 재검출
        self.max_lost_frames = 5
        self.min_object_pixels = 20

        # =========================================================
        # 3. ROS 통신
        # =========================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            sensor_qos,
        )

        # EfficientTAM이 생성한 물체 마스크
        self.object_mask_publisher = self.create_publisher(
            Image,
            "object_mask",
            1,
        )

        # YOLO 박스와 EfficientTAM 마스크가 표시된 디버그 영상
        self.llm_vid_publisher = self.create_publisher(
            Image,
            "llm_vid",
            1,
        )

        self.get_logger().info("EfficientTAM object tracking node started.")

    def image_callback(self, msg):
        try:
            self.current_image_msg = msg

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None:
                return

            annotated_image = cv_image.copy()
            object_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

            autocast_enabled = self.device.type == "cuda"

            with torch.inference_mode():
                with torch.autocast(
                    device_type=self.device.type,
                    dtype=torch.bfloat16,
                    enabled=autocast_enabled,
                ):
                    if not self.is_tracking:
                        object_mask = self.initialize_tracking(
                            cv_image,
                            annotated_image,
                        )
                    else:
                        object_mask = self.process_tracking(
                            cv_image,
                            annotated_image,
                        )

            self.publish_mask(
                object_mask,
                self.object_mask_publisher,
            )

            if DEBUG_VISUALIZATION:
                self.publish_annotated_image(annotated_image)

        except Exception as exc:
            self.get_logger().error(
                f"Error in image_callback: {exc}"
            )

    def initialize_tracking(self, cv_image, annotated_image):
        """
        YOLO로 물체를 검출하고, 가장 신뢰도가 높은 박스를
        EfficientTAM의 초기 프롬프트로 사용한다.
        """
        empty_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        yolo_results = self.yolo_model.predict(
            source=cv_image,
            verbose=False,
        )

        target_box = None
        target_conf = -1.0
        target_class_id = None

        if yolo_results and yolo_results[0].boxes is not None:
            boxes = yolo_results[0].boxes

            for box in boxes:
                confidence = float(box.conf[0].cpu().item())

                if confidence > target_conf:
                    xyxy = box.xyxy[0].cpu().numpy().astype(np.float32)
                    target_box = xyxy
                    target_conf = confidence
                    target_class_id = int(box.cls[0].cpu().item())

        if target_box is None:
            self.draw_hud(
                annotated_image,
                "MODE: INIT (Waiting for YOLO object)",
                (0, 165, 255),
            )
            return empty_mask

        x1, y1, x2, y2 = target_box.astype(int)

        # YOLO 검출 박스 표시
        cv2.rectangle(
            annotated_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 255),
            2,
        )

        class_name = str(target_class_id)
        if target_class_id in self.yolo_model.names:
            class_name = self.yolo_model.names[target_class_id]

        cv2.putText(
            annotated_image,
            f"{class_name} {target_conf:.2f}",
            (x1, max(25, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        rgb_image = cv2.cvtColor(
            cv_image,
            cv2.COLOR_BGR2RGB,
        )

        # EfficientTAM 메모리 초기화
        self.predictor.load_first_frame(
            rgb_image,
            self.num_classes,
        )

        # 클래스 0 = 물체
        self.predictor.add_new_prompts_during_track(
            0,
            boxes=np.array([target_box], dtype=np.float32),
            first_hit=np.bool_(True),
            frame=rgb_image,
        )

        # 초기 프레임 마스크 생성
        _, _, out_mask_logits = self.predictor.finalize_new_input()

        object_mask = self.logits_to_mask(
            out_mask_logits,
            class_index=0,
        )

        if np.count_nonzero(object_mask) == 0:
            self.get_logger().warn(
                "EfficientTAM returned an empty initial object mask."
            )
            self.draw_hud(
                annotated_image,
                "MODE: INIT (Empty TAM mask)",
                (0, 0, 255),
            )
            return empty_mask

        self.is_tracking = True
        self.lost_frame_count = 0
        self.current_frame_idx = 0

        if DEBUG_VISUALIZATION:
            self.draw_mask_overlay(
                annotated_image,
                object_mask,
                color=(255, 0, 0),
            )

        self.draw_hud(
            annotated_image,
            "MODE: INIT (Object locked)",
            (0, 255, 255),
        )

        return object_mask

    def process_tracking(self, cv_image, annotated_image):
        """
        EfficientTAM 메모리를 이용하여 다음 프레임의 물체 마스크를 추적한다.
        """
        self.current_frame_idx += 1

        rgb_image = cv2.cvtColor(
            cv_image,
            cv2.COLOR_BGR2RGB,
        )

        _, out_mask_logits = self.predictor.track(rgb_image)

        object_mask = self.logits_to_mask(
            out_mask_logits,
            class_index=0,
        )

        object_pixels = np.count_nonzero(object_mask)

        if DEBUG_VISUALIZATION and object_pixels > 0:
            self.draw_mask_overlay(
                annotated_image,
                object_mask,
                color=(255, 0, 0),
            )

        if object_pixels < self.min_object_pixels:
            self.lost_frame_count += 1

            self.draw_hud(
                annotated_image,
                (
                    "MODE: TRACKING "
                    f"(Lost {self.lost_frame_count}/"
                    f"{self.max_lost_frames})"
                ),
                (0, 0, 255),
            )
        else:
            self.lost_frame_count = 0

            self.draw_hud(
                annotated_image,
                "MODE: TRACKING (EfficientTAM object)",
                (0, 255, 0),
            )

        if self.lost_frame_count >= self.max_lost_frames:
            self.get_logger().warn(
                "Object mask lost. Resetting to YOLO initialization."
            )

            self.is_tracking = False
            self.lost_frame_count = 0
            self.current_frame_idx = 0

        return object_mask

    @staticmethod
    def logits_to_mask(out_mask_logits, class_index=0):
        """
        EfficientTAM 출력 logits를 mono8 마스크로 변환한다.
        """
        if isinstance(out_mask_logits, torch.Tensor):
            mask_logits = out_mask_logits.detach().float().cpu().numpy()
        else:
            mask_logits = np.asarray(out_mask_logits)

        class_logits = np.squeeze(mask_logits[class_index])

        return (
            (class_logits > 0.0).astype(np.uint8) * 255
        )

    @staticmethod
    def draw_hud(image, text, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2

        text_size, _ = cv2.getTextSize(
            text,
            font,
            font_scale,
            thickness,
        )

        text_w, text_h = text_size
        x, y = 15, 35

        overlay = image.copy()

        cv2.rectangle(
            overlay,
            (x - 5, y - text_h - 10),
            (x + text_w + 5, y + 10),
            (0, 0, 0),
            -1,
        )

        cv2.addWeighted(
            overlay,
            0.6,
            image,
            0.4,
            0,
            image,
        )

        cv2.putText(
            image,
            text,
            (x, y),
            font,
            font_scale,
            color,
            thickness,
        )

    @staticmethod
    def draw_mask_overlay(image, mask, color):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        cv2.drawContours(
            image,
            contours,
            -1,
            color,
            2,
        )

        colored_mask = np.zeros_like(image)
        colored_mask[mask == 255] = color

        cv2.addWeighted(
            colored_mask,
            0.4,
            image,
            1.0,
            0,
            image,
        )

    def publish_mask(self, mask_np, publisher):
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(
                mask_np,
                "mono8",
            )
            mask_msg.header = self.current_image_msg.header
            publisher.publish(mask_msg)

        except Exception as exc:
            self.get_logger().error(
                f"Failed to publish object mask: {exc}"
            )

    def publish_annotated_image(self, image_np):
        try:
            image_msg = self.bridge.cv2_to_imgmsg(
                image_np,
                "bgr8",
            )
            image_msg.header = self.current_image_msg.header
            self.llm_vid_publisher.publish(image_msg)

        except Exception as exc:
            self.get_logger().error(
                f"Failed to publish annotated image: {exc}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = EfficientTAMObjectTrackingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()