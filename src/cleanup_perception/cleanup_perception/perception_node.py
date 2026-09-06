"""Request-driven YOLO and depth perception for station cleanup scans."""

from dataclasses import replace

from ament_index_python.packages import get_package_prefix
import json
import os
from pathlib import Path
import threading
import time

from builtin_interfaces.msg import Time
from cleanup_interfaces.msg import ObjectObservation
from cleanup_interfaces.srv import CaptureObjects
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, JointState
from rclpy.time import Time as RosTime
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transforms
from tf2_ros import Buffer
from tf2_ros import TransformListener

from cleanup_perception.camera_calibration import CameraCalibration
from cleanup_perception.depth_geometry import central_grasp_sample, robust_depth_sample
from cleanup_perception.depth_geometry import DepthSample
from cleanup_perception.candidate_selection import select_body_sample
from cleanup_perception.grasp_anchor import GraspAnchors
from cleanup_perception.object_registry import confirm_burst
from cleanup_perception.object_registry import ConfirmedDetection
from cleanup_perception.object_registry import Detection
from cleanup_perception.object_registry import ObjectRegistry


class ScanPerceptionNode(Node):
    """Capture stable, map-grounded detections only when explicitly requested."""

    def __init__(self):
        super().__init__('scan_perception_node')
        self.declare_parameter(
            'model_path',
            '/home/user/turtlebot3_ws/src/segmentation/best.pt',
        )
        self.declare_parameter(
            'sam_model_path',
            '/home/user/turtlebot3_ws/src/segmentation/sam2_t.pt',
        )
        self.declare_parameter('use_sam_refinement', True)
        self.declare_parameter('sam_min_mask_pixels', 30)
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info'
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('allowed_classes', ['banana'])
        self.declare_parameter('min_confidence', 0.45)
        self.declare_parameter('association_gate', 0.25)
        self.declare_parameter('capture_timeout', 4.0)
        self.declare_parameter('empty_scene_frames', 2)
        self.declare_parameter('sync_tolerance', 0.15)
        self.declare_parameter('min_depth', 0.10)
        self.declare_parameter('max_depth', 2.50)
        self.declare_parameter(
            'debug_output_root', '/home/user/turtlebot3_ws/cleanup_debug'
        )
        self.declare_parameter('device', '')
        self.declare_parameter('use_grasp_camera_calibration', False)
        self.declare_parameter('grasp_camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('grasp_camera_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_camera_rotation', np.eye(3).ravel().tolist())
        self._camera_calibration = None
        if self.get_parameter('use_grasp_camera_calibration').value:
            self._camera_calibration = CameraCalibration(
                self.get_parameter('grasp_camera_translation').value,
                self.get_parameter('grasp_camera_rotation').value,
                self.get_parameter('grasp_camera_frame').value)
            self.get_logger().warning(
                'Local grasp camera calibration enabled; clipped-mask anchors disabled')

        self._use_body_candidates = self.declare_parameter('use_body_candidates', False).value
        self._forward_offset = self.declare_parameter('grasp_forward_offset', 0.0).value
        self._floor_height = self.declare_parameter('floor_height', 0.0).value
        self._align_grasp_floor = self.declare_parameter('align_grasp_floor', False).value
        self._min_body_depth = self.declare_parameter('grasp_min_body_depth', 0.006).value
        if not np.isfinite(self._min_body_depth) or not 0.004 <= self._min_body_depth <= 0.020:
            raise ValueError('Minimum body depth must be within 4..20mm')
        if self._use_body_candidates and self._camera_calibration is None:
            raise ValueError('Body candidate cleanup requires calibrated camera extrinsics')
        if not np.isfinite(self._forward_offset) or abs(self._forward_offset) > 0.030:
            raise ValueError('Forward correction must be within 30mm')
        self._latest_joints = None
        self._candidate_reports = {}
        self.get_logger().info(
            f'GRASP_CONFIG body_candidates={self._use_body_candidates}; '
            f'camera_calibration={self._camera_calibration is not None}; '
            f'forward_offset={self._forward_offset}; align_grasp_floor={self._align_grasp_floor}; '
            f'min_body_depth={self._min_body_depth}')

        self._model_path = self.get_parameter('model_path').value
        self._sam_model_path = self.get_parameter('sam_model_path').value
        self._use_sam_refinement = bool(
            self.get_parameter('use_sam_refinement').value
        )
        self._sam_min_mask_pixels = int(
            self.get_parameter('sam_min_mask_pixels').value
        )
        self._map_frame = self.get_parameter('map_frame').value
        self._allowed_classes = {
            str(name).strip().lower()
            for name in self.get_parameter('allowed_classes').value
        }
        self._min_confidence = float(
            self.get_parameter('min_confidence').value
        )
        self._association_gate = float(
            self.get_parameter('association_gate').value
        )
        self._capture_timeout = float(
            self.get_parameter('capture_timeout').value
        )
        self._empty_scene_frames = int(self.get_parameter('empty_scene_frames').value)
        self._sync_tolerance = float(
            self.get_parameter('sync_tolerance').value
        )
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._device = str(self.get_parameter('device').value)

        self._validate_parameters()
        self._bridge = CvBridge()
        self._model = None
        self._model_error = ''
        self._frame_yolo_count = 0
        self._sam_model = None
        self._sam_error = ''
        self._registry = ObjectRegistry(self._association_gate)
        self._active_mission_id = ''
        self._capture_count = 0
        self._capture_mutex = threading.Lock()
        self._frame_condition = threading.Condition()
        self._rgb_sequence = 0
        self._depth_sequence = 0
        self._latest_rgb = None
        self._latest_depth = None
        self._latest_info = None

        self._output_root = Path(
            os.path.expanduser(
                str(self.get_parameter('debug_output_root').value)
            )
        )
        self._output_root.mkdir(parents=True, exist_ok=True)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._grasp_anchors = GraspAnchors(self._tf_buffer)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            Image,
            str(self.get_parameter('rgb_topic').value),
            self._rgb_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._depth_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, qos_profile_sensor_data,
            callback_group=callback_group)
        # Warm the model during bringup, not while parked at the first scan heading.
        self._ensure_model()
        self._ensure_sam_model()
        self.create_service(
            CaptureObjects,
            '/cleanup/capture_objects',
            self._capture_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            f'Scan perception ready; evidence root: {self._output_root}'
        )

    def _validate_parameters(self):
        if not self._allowed_classes:
            raise ValueError('allowed_classes must not be empty')
        if not 0.0 <= self._min_confidence <= 1.0:
            raise ValueError('min_confidence must be between zero and one')
        if self._association_gate <= 0.0:
            raise ValueError('association_gate must be positive')
        if self._capture_timeout <= 0.0 or self._sync_tolerance <= 0.0:
            raise ValueError('capture and synchronization times must be positive')
        if not 2 <= self._empty_scene_frames <= 100:
            raise ValueError('empty_scene_frames must be between 2 and 100')
        if self._min_depth <= 0.0 or self._max_depth <= self._min_depth:
            raise ValueError('depth limits are invalid')
        if self._sam_min_mask_pixels < 1:
            raise ValueError('sam_min_mask_pixels must be positive')

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def _rgb_callback(self, message):
        with self._frame_condition:
            self._latest_rgb = message
            self._rgb_sequence += 1
            self._frame_condition.notify_all()

    def _depth_callback(self, message):
        with self._frame_condition:
            self._latest_depth = message
            self._depth_sequence += 1
            self._frame_condition.notify_all()

    def _camera_info_callback(self, message):
        with self._frame_condition:
            self._latest_info = message

    def _ensure_model(self):
        if self._model is not None:
            return True
        if self._model_error:
            return False
        try:
            from ultralytics import YOLO

            self._model = YOLO(self._model_path)
            arguments = {'source': np.zeros((640, 640, 3), np.uint8), 'verbose': False}
            if self._device:
                arguments['device'] = self._device
            self._model.predict(**arguments)
            self.get_logger().info(f'Loaded YOLO model: {self._model_path}')
            return True
        except Exception as exception:
            self._model = None
            self._model_error = f'{type(exception).__name__}: {exception}'
            self.get_logger().error(
                f'Unable to load YOLO model: {self._model_error}'
            )
            return False

    def _ensure_sam_model(self):
        if not self._use_sam_refinement:
            return False
        if self._sam_model is not None:
            return True
        if self._sam_error:
            return False
        try:
            from ultralytics import SAM

            self._sam_model = SAM(self._sam_model_path)
            self.get_logger().info(
                f'Loaded SAM2 refinement model: {self._sam_model_path}'
            )
            return True
        except Exception as exception:
            self._sam_error = str(exception)
            self.get_logger().warning(
                'SAM2 unavailable; using bounded-box depth fallback: '
                f'{self._sam_error}'
            )
            return False

    def _next_synchronized_frame(
            self, last_rgb_sequence, last_depth_sequence, deadline):
        while time.monotonic() < deadline:
            with self._frame_condition:
                remaining = deadline - time.monotonic()
                self._frame_condition.wait_for(
                    lambda: (
                        self._rgb_sequence > last_rgb_sequence and
                        self._depth_sequence > last_depth_sequence
                    ),
                    timeout=max(0.0, remaining),
                )
                if (self._rgb_sequence <= last_rgb_sequence or
                        self._depth_sequence <= last_depth_sequence):
                    return None
                rgb_sequence = self._rgb_sequence
                depth_sequence = self._depth_sequence
                rgb = self._latest_rgb
                depth = self._latest_depth
                info = self._latest_info

            if rgb is None or depth is None or info is None:
                last_rgb_sequence = rgb_sequence
                last_depth_sequence = depth_sequence
                continue
            rgb_time = self._stamp_seconds(rgb.header.stamp)
            depth_time = self._stamp_seconds(depth.header.stamp)
            if abs(rgb_time - depth_time) > self._sync_tolerance:
                last_rgb_sequence = rgb_sequence
                last_depth_sequence = depth_sequence
                continue
            return rgb_sequence, depth_sequence, rgb, depth, info
        return None

    def _capture_callback(self, request, response):
        if not request.mission_id.strip() or not request.station_name.strip():
            response.success = False
            response.message = 'mission_id and station_name must not be empty'
            return response
        if request.burst_frames < 1 or request.min_confirmations < 1:
            response.success = False
            response.message = 'burst_frames and min_confirmations must be positive'
            return response
        if request.min_confirmations > request.burst_frames:
            response.success = False
            response.message = 'min_confirmations exceeds burst_frames'
            return response
        if not self._capture_mutex.acquire(blocking=False):
            response.success = False
            response.message = 'another capture request is already active'
            return response
        started, cpu_started = time.perf_counter(), time.process_time()
        try:
            if not self._ensure_model():
                self._capture_count += 1
                response.success = False
                response.message = f'YOLO unavailable: {self._model_error}'
                return response
            if request.mission_id != self._active_mission_id:
                self._registry = ObjectRegistry(self._association_gate)
                self._active_mission_id = request.mission_id
                self._grasp_anchors.clear()
                self.get_logger().info(
                    f'Reset object registry for mission {request.mission_id}'
                )
            return self._perform_capture(request, response)
        except Exception as exception:
            response.success = False
            response.message = f'Capture failed: {type(exception).__name__}: {exception}'
            self.get_logger().error(response.message)
            return response
        finally:
            elapsed = time.perf_counter() - started
            cpu = time.process_time() - cpu_started
            self.get_logger().info(
                f'[CAPTURE_PERF] station={request.station_name} '
                f'heading={request.heading_index} wall_ms={1000 * elapsed:.1f} '
                f'process_cpu_ms={1000 * cpu:.1f}')
            self._capture_mutex.release()

    def _joint_callback(self, message):
        self._latest_joints = message

    def _body_sample(self, mask, depth, info, header, sample, index):
        """Select using the same body geometry and IK planner as the successful trial."""
        joints = self._latest_joints
        if joints is None:
            raise ValueError('Candidate planning requires joint feedback')
        age = (self.get_clock().now() - RosTime.from_msg(joints.header.stamp)).nanoseconds * 1e-9
        if not 0 <= age < 0.25:
            raise ValueError('Candidate planning requires fresh joint feedback')
        positions = dict(zip(joints.name, joints.position))
        start = [positions['joint' + str(i)] for i in range(1, 5)]
        stamp = RosTime.from_msg(header.stamp)
        wrist = self._tf_buffer.lookup_transform(
            'link1', 'link5', stamp, timeout=Duration(seconds=0.2)).transform
        floor = PointStamped()
        floor.header.frame_id = self._map_frame
        floor.header.stamp = header.stamp
        floor.point.z = self._floor_height
        floor = self._tf_buffer.transform(floor, 'link1', timeout=Duration(seconds=0.2))
        k = np.asarray(info.k, dtype=float).reshape(3, 3).copy()
        k[0] *= depth.shape[1] / info.width
        k[1] *= depth.shape[0] / info.height
        planner = (Path(get_package_prefix('pick_and_place')) /
                   'lib/pick_and_place/candidate_grasp_plan')
        selected, report = select_body_sample(
            mask, depth, k, wrist, self._camera_calibration,
            self._forward_offset, floor.point.z, start, planner,
            align_floor=self._align_grasp_floor, min_body_depth=self._min_body_depth)
        self._candidate_reports[index] = report
        strategy = 'candidate_body' if selected['plan'].get('feasible') else 'candidate_approach'
        reason = (f"{strategy}; forward={self._forward_offset:.3f}; "
                  f"body={selected['body_percentile']}%; width={selected['width_m']:.3f}; "
                  f"pitch={selected['plan'].get('pitch_degrees', 'unreachable')}; "
                  f"floor_z_correction={selected.get('floor_correction_m', 0.):.4f}")
        return DepthSample(*selected['pixel'], selected['depth_m'],
                           selected['depth_uncertainty_m'], 30), strategy, reason

    def _perform_capture(self, request, response):
        self._capture_heading = request.heading_index
        if request.heading_index in (249, 251) and hasattr(self, '_grasp_anchors'):
            self._grasp_anchors.clear()
        self._capture_count += 1
        deadline = time.monotonic() + self._capture_timeout
        last_rgb_sequence = self._rgb_sequence
        last_depth_sequence = self._depth_sequence
        detections = []
        best_annotated = None
        best_evidence_score = -1.0
        last_stamp = Time()
        collected_frames = 0
        captured_frames = []
        last_error = ''
        seen_yolo = False
        empty_limit = min(self._empty_scene_frames, request.burst_frames)

        while collected_frames < request.burst_frames:
            frame = self._next_synchronized_frame(
                last_rgb_sequence, last_depth_sequence, deadline
            )
            if frame is None:
                break
            (last_rgb_sequence, last_depth_sequence, rgb_message,
             depth_message, info_message) = frame
            try:
                image = self._bridge.imgmsg_to_cv2(rgb_message, 'bgr8')
                depth = self._bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding='passthrough'
                )
                frame_detections, annotated = self._detect_frame(
                    image,
                    depth,
                    info_message,
                    depth_message.header,
                    collected_frames,
                )
                has_yolo = self._frame_yolo_count > 0 or bool(frame_detections)
                if has_yolo and not seen_yolo:
                    self._save_evidence(request.mission_id, request.station_name,
                                        int(request.heading_index), image, suffix='_raw')
                seen_yolo = seen_yolo or has_yolo
                detections.extend(frame_detections)
                captured_frames.append((
                    image,
                    depth,
                    info_message,
                    depth_message.header,
                    annotated,
                ))
                evidence_score = sum(
                    detection.confidence for detection in frame_detections
                )
                if evidence_score > best_evidence_score:
                    best_evidence_score = evidence_score
                    best_annotated = annotated
                last_stamp = depth_message.header.stamp
                collected_frames += 1
                # Empty headings need neither a positive confirmation burst nor photos.
                # Any YOLO target, even without valid depth, keeps the full capture path.
                if not seen_yolo and collected_frames >= empty_limit:
                    response.success = True
                    response.image_reference = ''
                    response.message = (f'YOLO: no target in {collected_frames} frames; '
                                        'empty heading skipped; no image saved')
                    return response
                if confirm_burst(detections, self._association_gate,
                                 int(request.min_confirmations)):
                    break
            except Exception as exception:
                last_error = f'{type(exception).__name__}: {exception}'
                self.get_logger().warning(
                    f'Ignoring unusable capture frame: {exception}'
                )

        if collected_frames < request.min_confirmations:
            response.image_reference = self._save_evidence(
                request.mission_id,
                request.station_name,
                int(request.heading_index),
                best_annotated if seen_yolo else None,
            )
            response.success = False
            response.message = (
                f'only {collected_frames} synchronized frames arrived; '
                f'{request.min_confirmations} required; last_error={last_error}'
            )
            return response

        confirmed = confirm_burst(
            detections,
            self._association_gate,
            int(request.min_confirmations),
        )
        confirmed, refined_evidence = self._refine_confirmed_with_sam(
            confirmed, captured_frames
        )
        if refined_evidence is not None:
            best_annotated = refined_evidence
        image_path = self._save_evidence(
            request.mission_id,
            request.station_name,
            int(request.heading_index),
            best_annotated if seen_yolo else None,
        )
        response.image_reference = image_path
        messages = []
        assigned_ids = set()
        for index, item in enumerate(confirmed):
            detection = item.representative
            object_uuid = self._registry.associate(detection, assigned_ids)
            assigned_ids.add(object_uuid)
            observation = ObjectObservation()
            if detection.frame_index < len(captured_frames):
                observation.header.stamp = captured_frames[
                    detection.frame_index
                ][3].stamp
            else:
                observation.header.stamp = last_stamp
            observation.header.frame_id = self._map_frame
            observation.observation_id = (
                f'{request.station_name}-h{int(request.heading_index)}-'
                f'c{self._capture_count}-o{index}'
            )
            observation.object_uuid = object_uuid
            observation.class_name = detection.class_name
            observation.confidence = float(detection.confidence)
            observation.bbox = list(detection.bbox)
            observation.centroid.x = detection.x
            observation.centroid.y = detection.y
            observation.centroid.z = detection.z
            observation.position_uncertainty = float(detection.uncertainty)
            observation.station_name = request.station_name
            observation.heading_index = request.heading_index
            # Each bbox must refer to the exact source frame sent to the VLM.
            evidence = captured_frames[detection.frame_index][4].copy()
            cv2.putText(
                evidence, f'UUID: {object_uuid}', (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1,
            )
            cv2.putText(evidence, detection.grasp_reason, (8, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 0) if detection.grasp_valid else (0, 0, 255), 1)
            observation.image_reference = self._save_evidence(
                request.mission_id, request.station_name,
                int(request.heading_index), evidence, suffix=f'_object_{index}',
            )
            replay = getattr(self, '_grasp_replay', {}).get(index)
            if replay is not None and observation.image_reference:
                try:
                    np.savez_compressed(
                        Path(observation.image_reference).with_suffix('.npz'),
                        **replay, grasp_pixel=np.asarray([detection.grasp_u, detection.grasp_v]),
                        map_point=np.asarray([detection.x, detection.y, detection.z]),
                        camera_frame=np.asarray(
                            captured_frames[detection.frame_index][3].frame_id),
                    )
                except OSError as exception:
                    self.get_logger().warning(f'Grasp replay save failed: {exception}')
            observation.detector_track_id = detection.detector_track_id
            observation.confirmation_count = item.confirmation_count
            observation.grasp_valid = detection.grasp_valid
            observation.grasp_reason = detection.grasp_reason
            observation.grasp_strategy = detection.grasp_strategy
            candidate_report = getattr(self, '_candidate_reports', {}).get(index)
            if candidate_report is not None and observation.image_reference:
                Path(observation.image_reference).with_suffix('.candidates.json').write_text(
                    json.dumps(candidate_report, indent=2), encoding='utf-8')
            observation.grasp_pixel = [detection.grasp_u, detection.grasp_v]
            messages.append(observation)

        response.success = True
        response.observations = messages
        response.message = (
            f'captured {collected_frames} frames and confirmed '
            f'{len(messages)} objects'
        )
        metadata = {
            'frames': collected_frames,
            'grounded_detections': len(detections),
            'confirmed_objects': len(messages),
            'image_reference': image_path,
            'objects': [
                {'uuid': item.object_uuid,
                 'bbox': [int(value) for value in item.bbox],
                 'class_name': item.class_name,
                 'grasp_valid': item.grasp_valid,
                 'grasp_reason': item.grasp_reason,
                 'grasp_strategy': item.grasp_strategy,
                 'grasp_pixel': [float(value) for value in item.grasp_pixel],
                 'centroid': [item.centroid.x, item.centroid.y, item.centroid.z]}
                for item in messages
            ],
        }
        if image_path:
            Path(image_path).with_suffix('.json').write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8'
            )
        self.get_logger().info(
            f'[CAPTURE_COMPLETE] {response.message}; evidence={image_path}'
        )
        return response

    def _detect_frame(self, image, depth, camera_info, header, frame_index):
        self._frame_yolo_count = 0
        arguments = {
            'source': image,
            'verbose': False,
            'conf': self._min_confidence,
        }
        if self._device:
            arguments['device'] = self._device
        results = self._model.predict(**arguments)
        annotated = image.copy()
        detections = []
        if not results or results[0].boxes is None:
            return detections, annotated

        names = self._model.names
        for box in results[0].boxes:
            class_id = int(box.cls[0].cpu().item())
            class_name = str(
                names.get(class_id, class_id)
                if isinstance(names, dict) else names[class_id]
            ).strip().lower()
            confidence = float(box.conf[0].cpu().item())
            if class_name not in self._allowed_classes:
                continue
            self._frame_yolo_count += 1
            bbox = box.xyxy[0].cpu().numpy().astype(int)
            point, uncertainty = self._bbox_point_in_map(
                bbox, image.shape, depth, camera_info, header
            )
            if point is None:
                x_min, y_min, x_max, y_max = bbox
                cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max),
                              (0, 0, 255), 2)
                cv2.putText(annotated, f'{class_name}: NO DEPTH/TF',
                            (x_min, max(24, y_min - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                self.get_logger().warning(
                    f'[DETECTION_REJECTED] {class_name} confidence={confidence:.3f}; '
                    'no valid depth/map point'
                )
                continue
            track_id = -1
            if box.id is not None:
                track_id = int(box.id[0].cpu().item())
            detections.append(
                Detection(
                    class_name=class_name,
                    confidence=confidence,
                    x=point.point.x,
                    y=point.point.y,
                    z=point.point.z,
                    uncertainty=uncertainty,
                    bbox=tuple(int(value) for value in bbox),
                    frame_index=frame_index,
                    stamp_sec=self._stamp_seconds(header.stamp),
                    detector_track_id=track_id,
                )
            )
            x_min, y_min, x_max, y_max = bbox
            cv2.rectangle(
                annotated,
                (x_min, y_min),
                (x_max, y_max),
                (0, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                f'{class_name} {confidence:.2f}',
                (x_min, max(24, y_min - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
        return detections, annotated

    def _run_sam_masks(self, image, bboxes):
        arguments = {
            'source': image,
            'bboxes': np.asarray(bboxes, dtype=np.float32),
            'verbose': False,
            'save': False,
        }
        if self._device:
            arguments['device'] = self._device
        try:
            results = self._sam_model.predict(**arguments)
            if not results or results[0].masks is None:
                raise ValueError('SAM2 returned no masks')
            masks = results[0].masks.data.detach().cpu().numpy()
            if len(masks) != len(bboxes):
                raise ValueError('SAM2 mask count does not match bbox count')
            normalized = []
            for mask in masks:
                if mask.shape != image.shape[:2]:
                    mask = cv2.resize(
                        mask,
                        (image.shape[1], image.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                normalized.append(mask > 0.5)
            return normalized
        except Exception as exception:
            self._sam_error = str(exception)
            self._sam_model = None
            self._use_sam_refinement = False
            self.get_logger().warning(
                'SAM2 refinement failed; disabling it and using bbox depth: '
                f'{self._sam_error}'
            )
            return []

    def _refine_confirmed_with_sam(self, confirmed, captured_frames):
        self._grasp_replay = {}
        self._candidate_reports = {}
        if not confirmed or not self._ensure_sam_model():
            return confirmed, None

        groups = {}
        for result_index, item in enumerate(confirmed):
            frame_index = item.representative.frame_index
            if 0 <= frame_index < len(captured_frames):
                groups.setdefault(frame_index, []).append(result_index)

        refined = list(confirmed)
        best_image = None
        best_score = -1.0
        for frame_index, result_indices in groups.items():
            image, depth, info, header, yolo_annotated = captured_frames[
                frame_index
            ]
            bboxes = [
                confirmed[index].representative.bbox
                for index in result_indices
            ]
            masks = self._run_sam_masks(image, bboxes)
            if len(masks) != len(result_indices):
                continue

            annotated = yolo_annotated.copy()
            refined_count = 0
            confidence_sum = 0.0
            for result_index, mask in zip(result_indices, masks):
                item = confirmed[result_index]
                bbox = item.representative.bbox
                clipped = min(bbox[0], bbox[1], image.shape[1] - bbox[2],
                              image.shape[0] - bbox[3]) <= 3
                # SAM can include adjacent objects; constrain it to this detection.
                bounded = np.zeros_like(mask, dtype=bool)
                bounded[max(0, bbox[1]):bbox[3], max(0, bbox[0]):bbox[2]] = True
                mask = mask & bounded
                depth_mask = mask
                if mask.shape != depth.shape:
                    depth_mask = cv2.resize(mask.astype(np.uint8),
                                            (depth.shape[1], depth.shape[0]),
                                            interpolation=cv2.INTER_NEAREST) > 0
                sample = None if clipped else central_grasp_sample(
                    depth_mask, depth, self._min_depth, self._max_depth,
                    self._sam_min_mask_pixels)
                self._grasp_replay[result_index] = {
                    'mask': depth_mask.astype(np.uint8), 'depth': depth,
                    'camera_k': np.asarray(info.k),
                    'calibration_enabled': np.asarray(
                        getattr(self, '_camera_calibration', None) is not None),
                    'calibration_translation': np.asarray(
                        self._camera_calibration.translation
                        if getattr(self, '_camera_calibration', None) is not None else []),
                    'calibration_rotation': np.asarray(
                        self._camera_calibration.rotation
                        if getattr(self, '_camera_calibration', None) is not None else []),
                    'camera_size': np.asarray([info.width, info.height]),
                    'image_stamp': np.asarray([header.stamp.sec, header.stamp.nanosec])
                    if hasattr(header, 'stamp') else np.zeros(2),
                }
                anchored = False
                if clipped and getattr(self, '_capture_heading', -1) == 250 and \
                        hasattr(self, '_grasp_anchors') and \
                        getattr(self, '_camera_calibration', None) is None:
                    sample = self._grasp_anchors.sample(
                        item.representative.class_name, depth_mask, depth, info, header)
                    anchored = sample is not None
                strategy = 'legacy_center'
                reason = ('Reobserved odom-anchored body center with matching mask/depth'
                          if anchored else 'Central body (ends excluded), fitted local depth')
                if sample is not None and getattr(self, '_use_body_candidates', False):
                    if getattr(self, '_capture_heading', -1) in (249, 250):
                        try:
                            sample, strategy, reason = self._body_sample(
                                depth_mask, depth, info, header, sample, result_index)
                        except Exception as exception:
                            refined[result_index] = ConfirmedDetection(replace(
                                item.representative, grasp_valid=False,
                                grasp_reason=f'Candidate selection failed: {exception}'),
                                item.confirmation_count)
                            continue
                    else:
                        strategy = 'candidate_approach'
                if result_index in self._candidate_reports:
                    point, uncertainty = self._candidate_point_in_map(result_index, header)
                else:
                    point, uncertainty = self._sample_point_in_map(
                        sample, image.shape, depth, info, header)
                if sample is not None:
                    self._grasp_replay[result_index]['grasp_depth_m'] = np.asarray(sample.distance)
                if point is None:
                    refined[result_index] = ConfirmedDetection(replace(
                        item.representative, grasp_reason=(
                            'Object clipped by image boundary' if clipped else
                            'Full mask central depth unavailable')), item.confirmation_count)
                    continue
                detection = replace(
                    item.representative,
                    x=point.point.x,
                    y=point.point.y,
                    z=point.point.z,
                    uncertainty=uncertainty,
                    grasp_valid=True,
                    grasp_reason=reason,
                    grasp_strategy=strategy,
                    grasp_u=sample.u * image.shape[1] / depth.shape[1],
                    grasp_v=sample.v * image.shape[0] / depth.shape[0],
                )
                refined[result_index] = ConfirmedDetection(
                    detection, item.confirmation_count
                )
                if getattr(self, '_capture_heading', -1) == 249 and \
                        hasattr(self, '_grasp_anchors') and \
                        getattr(self, '_camera_calibration', None) is None:
                    self._grasp_anchors.remember(
                        detection.class_name, point, header, observation_key=result_index)
                refined_count += 1
                confidence_sum += detection.confidence
                overlay = annotated.copy()
                overlay[mask] = (255, 0, 255)
                annotated = cv2.addWeighted(
                    annotated, 0.72, overlay, 0.28, 0.0
                )
                contours, _ = cv2.findContours(
                    mask.astype(np.uint8),
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE,
                )
                cv2.drawContours(annotated, contours, -1, (255, 0, 255), 2)
                cv2.drawMarker(annotated, (int(detection.grasp_u), int(detection.grasp_v)),
                               (0, 255, 0), cv2.MARKER_CROSS, 22, 2)

            # UUID images must use the same SAM mask and grasp marker as the summary.
            captured_frames[frame_index] = (image, depth, info, header, annotated)

            score = refined_count * 100.0 + confidence_sum
            if refined_count > 0 and score > best_score:
                best_score = score
                best_image = annotated
        return refined, best_image

    def _bbox_point_in_map(self, bbox, rgb_shape, depth, info, header):
        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth.shape[:2]
        scale_x = float(depth_width) / float(rgb_width)
        scale_y = float(depth_height) / float(rgb_height)
        x_min, y_min, x_max, y_max = bbox.astype(float)
        width = max(1.0, x_max - x_min)
        height = max(1.0, y_max - y_min)
        roi_x1 = int((x_min + width * 0.25) * scale_x)
        roi_x2 = int((x_max - width * 0.25) * scale_x)
        roi_y1 = int((y_min + height * 0.25) * scale_y)
        roi_y2 = int((y_max - height * 0.25) * scale_y)
        roi_x1 = max(0, min(depth_width - 1, roi_x1))
        roi_x2 = max(roi_x1 + 1, min(depth_width, roi_x2))
        roi_y1 = max(0, min(depth_height - 1, roi_y1))
        roi_y2 = max(roi_y1 + 1, min(depth_height, roi_y2))
        depth_mask = np.zeros((depth_height, depth_width), dtype=bool)
        depth_mask[roi_y1:roi_y2, roi_x1:roi_x2] = True
        sample = robust_depth_sample(
            depth_mask,
            depth,
            self._min_depth,
            self._max_depth,
        )
        return self._sample_point_in_map(sample, rgb_shape, depth, info, header)

    def _candidate_point_in_map(self, index, header):
        """Publish the evaluated observed point, including its bounded height alignment."""
        selected = self._candidate_reports[index]['selected']
        point = PointStamped()
        point.header.stamp = header.stamp
        point.header.frame_id = 'link1'
        # Forward +30mm belongs to the executor, not this observation boundary.
        point.point.x, point.point.y, point.point.z = selected['observed_link1']
        try:
            transformed = self._tf_buffer.transform(
                point, self._map_frame, timeout=Duration(seconds=0.2))
            return transformed, selected['depth_uncertainty_m']
        except Exception as exception:
            self.get_logger().warning(f'No timestamped candidate-to-map transform: {exception}')
            return None, 0.0

    def _sample_point_in_map(self, sample, rgb_shape, depth, info, header):
        if sample is None:
            return None, 0.0
        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth.shape[:2]
        info_width = float(info.width) if info.width else float(rgb_width)
        info_height = float(info.height) if info.height else float(rgb_height)
        intrinsic_scale_x = float(depth_width) / info_width
        intrinsic_scale_y = float(depth_height) / info_height
        fx = float(info.k[0]) * intrinsic_scale_x
        fy = float(info.k[4]) * intrinsic_scale_y
        cx = float(info.k[2]) * intrinsic_scale_x
        cy = float(info.k[5]) * intrinsic_scale_y
        if fx <= 0.0 or fy <= 0.0:
            return None, 0.0

        camera_point = PointStamped()
        camera_point.header = header
        camera_point.point.x = (sample.u - cx) * sample.distance / fx
        camera_point.point.y = (sample.v - cy) * sample.distance / fy
        camera_point.point.z = sample.distance
        try:
            calibration = getattr(self, '_camera_calibration', None)
            if calibration is not None:
                camera_point = calibration.to_wrist(camera_point)
            map_point = self._tf_buffer.transform(
                camera_point,
                self._map_frame,
                timeout=Duration(seconds=0.20),
            )
        except Exception as exception:
            self.get_logger().warning(
                f'No timestamped camera-to-map transform: {exception}'
            )
            return None, 0.0
        return map_point, sample.uncertainty

    def _save_evidence(
            self, mission_id, station_name, heading_index, image, suffix=''):
        if image is None:
            return ''
        mission = ''.join(
            character if character.isalnum() or character in '-_'
            else '_'
            for character in mission_id
        )
        station = ''.join(
            character if character.isalnum() or character in '-_'
            else '_'
            for character in station_name
        )
        directory = self._output_root / mission / station
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (
            f'heading_{heading_index}_capture_{self._capture_count:03d}{suffix}.jpg'
        )
        if not cv2.imwrite(str(path), image):
            raise OSError(f'Failed to save evidence image: {path}')
        self.get_logger().info(f'[IMAGE_SAVED] {path}')
        return str(path)


def main(args=None):
    """Run the request-driven scan perception node."""
    rclpy.init(args=args)
    node = ScanPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
