"""Capture, rank body/pitch candidates and optionally move an open gripper to hover."""

import argparse
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_prefix
from cleanup_interfaces.srv import CaptureObjects
from control_msgs.action import FollowJointTrajectory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from cleanup_perception.camera_calibration import CameraCalibration
from cleanup_perception.depth_geometry import central_grasp_sample, DepthSample
from cleanup_perception.grasp_candidates import (
    body_candidates, choose_candidate, NoFeasibleCandidate)


def rotation_matrix(quaternion):
    """Convert an xyzw TF quaternion to an orthonormal rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = x*x + y*y + z*z + w*w
    if not np.isfinite(norm) or abs(norm - 1.) > 0.001:
        raise ValueError('Invalid TF quaternion')
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class HoverTrial:
    """Run one bounded hover with fresh sensors and no close or base command."""

    def __init__(self, options):
        self.options = options
        self.node = rclpy.create_node('candidate_hover_trial')
        self.root = Path(options.output) / (
            'candidate_hover_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.root.mkdir(parents=True, exist_ok=False)
        self.report = {}
        self.sensors = {}
        self.subscriptions = []
        for name, topic, kind in [
            ('joints', '/joint_states', JointState), ('odom', '/odom', Odometry),
            ('rgb', '/camera/camera/color/image_raw', Image),
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw', Image),
            ('info', '/camera/camera/color/camera_info', CameraInfo),
        ]:
            self.subscriptions.append(self.node.create_subscription(
                kind, topic, lambda msg, key=name: self.sensors.update({key: msg}),
                qos_profile_sensor_data))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.arm = ActionClient(
            self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.handle = None
        self.complete = False

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _wait(self, future, seconds):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('Request timed out; inspect before another movement')
        return future.result()

    def _call(self, kind, name, request, seconds):
        client = self.node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Service unavailable: ' + name)
        result = self._wait(client.call_async(request), seconds)
        if not result.success:
            raise RuntimeError(name + ': ' + result.message)
        return result

    def _fresh(self, message):
        age = (self.node.get_clock().now() - Time.from_msg(message.header.stamp))
        if not 0 <= age.nanoseconds * 1e-9 < 0.25:
            raise RuntimeError('Stale sensor feedback')

    def _base(self):
        msg = self.sensors['odom']
        self._fresh(msg)
        if abs(msg.twist.twist.linear.x) > 0.005 or abs(msg.twist.twist.angular.z) > 0.02:
            raise RuntimeError('Base moving')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return np.array([p.x, p.y, yaw])

    def _joints(self):
        msg = self.sensors['joints']
        self._fresh(msg)
        joints = dict(zip(msg.name, msg.position))
        values = [joints['joint' + str(i)] for i in range(1, 5)]
        if not np.isfinite(values).all():
            raise RuntimeError('Invalid arm feedback')
        if self.options.execute_hover and (
                not np.isfinite(joints['gripper_left_joint']) or
                abs(joints['gripper_left_joint'] - 0.019) > 0.0015):
            raise RuntimeError('Gripper is not fully open')
        return values

    def _geometry(self, obj, calibration):
        with np.load(Path(obj.image_reference).with_suffix('.npz')) as archive:
            data = {key: archive[key] for key in archive.files}
        if not bool(data['calibration_enabled']):
            raise RuntimeError('Capture does not use the supplied experimental calibration')
        if str(data['camera_frame']) != calibration.camera_frame:
            raise RuntimeError('Capture camera frame mismatch')
        stamp = data['image_stamp']
        acquisition = Time(seconds=int(stamp[0]), nanoseconds=int(stamp[1]),
                           clock_type=self.node.get_clock().clock_type)
        wrist = self.buffer.lookup_transform(
            'link1', 'link5', acquisition, timeout=Duration(seconds=0.2)).transform
        q, t = wrist.rotation, wrist.translation
        rw = rotation_matrix([q.x, q.y, q.z, q.w])
        rotation = rw @ calibration.rotation
        translation = np.array([t.x, t.y, t.z]) + rw @ calibration.translation
        target = PointStamped(header=deepcopy(obj.header), point=obj.centroid)
        target.header.stamp = Time().to_msg()
        local = self.buffer.transform(target, 'link1', timeout=Duration(seconds=0.2))
        original = central_grasp_sample(data['mask'], data['depth'], 0.1, 2.5, 30)
        if 'grasp_depth_m' in data:
            # Integrated perception may already have chosen another body section.
            original = DepthSample(*data['grasp_pixel'], float(data['grasp_depth_m']), 0., 30)
        if original is None:
            raise RuntimeError('Missing original depth support')
        ray = np.linalg.inv(data['camera_k'].reshape(3, 3)) @ np.array([
            original.u, original.v, 1.])
        reconstructed = rotation @ (ray * original.distance) + translation
        if np.linalg.norm(reconstructed - np.array([
                local.point.x, local.point.y, local.point.z])) > 0.010:
            raise RuntimeError('Supplied calibration disagrees with perception target')
        floor_point = deepcopy(target)
        floor_point.point.z = self.options.floor_height
        floor = self.buffer.transform(floor_point, 'link1', timeout=Duration(seconds=0.2))
        self.report['geometry'] = {
            'rotation': rotation.tolist(), 'translation': translation.tolist(),
            'floor_link1': floor.point.z, 'acquisition_stamp': stamp.tolist(),
            'original_pixel': [original.u, original.v],
            'original_link1': reconstructed.tolist(),
        }
        np.savez_compressed(self.root / 'capture.npz', **data)
        return data, rotation, translation, floor.point.z, acquisition

    def run(self):
        """Save a selected plan, optionally executing only its open hover waypoint."""
        cfg = yaml.safe_load(Path(self.options.camera_calibration).read_text())
        cfg = cfg['scan_perception_node']['ros__parameters']
        calibration = CameraCalibration(cfg['grasp_camera_translation'],
                                        cfg['grasp_camera_rotation'], cfg['grasp_camera_frame'])
        deadline = time.monotonic() + 10
        while not all(k in self.sensors for k in ('odom', 'joints', 'rgb', 'depth', 'info')):
            if time.monotonic() > deadline:
                raise RuntimeError('Sensor discovery timeout')
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._spin(0.4)
        base = self._base()
        if self.options.execute_hover:
            self._call(Trigger, '/open_gripper', Trigger.Request(), 5)
            self._call(Trigger, '/observe_floor', Trigger.Request(), 5)
            self._spin(0.4)
        capture = self._call(CaptureObjects, '/cleanup/capture_objects', CaptureObjects.Request(
            mission_id=self.root.name, station_name='station_0_arm_only', heading_index=249,
            burst_frames=3, min_confirmations=3), 20)
        objects = [o for o in capture.observations if o.class_name == 'banana' and o.grasp_valid]
        if len(objects) != 1:
            raise RuntimeError('Expected one complete banana')
        obj = objects[0]
        self.report['image'] = obj.image_reference
        data, rotation, translation, floor, acquisition = self._geometry(obj, calibration)
        candidates = body_candidates(data['mask'], data['depth'], data['camera_k'],
                                     rotation, translation, self.options.forward_offset)
        self.report['candidates'] = candidates
        self._spin(0.1)
        start = self._joints()
        self.report['start_joints'] = start
        planner = (Path(get_package_prefix('pick_and_place')) /
                   'lib/pick_and_place/candidate_grasp_plan')
        try:
            selected, evaluated = choose_candidate(candidates, floor, start, planner)
        except NoFeasibleCandidate as error:
            self.report['candidates'] = error.evaluated
            self.report['reapproach_required'] = True
            raise
        self.report.update(selected=selected, candidates=evaluated, start_joints=start)
        preview = cv2.imread(obj.image_reference)
        if preview is not None:
            for candidate in evaluated:
                pixel = tuple(round(v) for v in candidate['pixel'])
                color = (0, 200, 255) if candidate['plan']['feasible'] else (0, 0, 255)
                cv2.circle(preview, pixel, 5, color, 1)
            pixel = tuple(round(v) for v in selected['pixel'])
            cv2.circle(preview, pixel, 12, (255, 255, 0), 3)
            cv2.putText(preview, 'CYAN: selected body point before forward offset',
                        (15, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.imwrite(str(self.root / 'candidate_preview.jpg'), preview)
        print(json.dumps({'selected': selected, 'output': str(self.root)}), flush=True)
        self._spin(0.1)
        if np.max(np.abs(self._base() - base)) > 0.005:
            raise RuntimeError('Base moved during capture/planning')
        if not self.options.execute_hover:
            return
        if not self.arm.wait_for_server(timeout_sec=3):
            raise RuntimeError('Arm action unavailable')
        if not 0 <= (self.node.get_clock().now() - acquisition).nanoseconds * 1e-9 < 5:
            raise RuntimeError('Target expired before motion')
        if np.max(np.abs(np.array(self._joints()) - start)) > 0.02:
            raise RuntimeError('Arm moved during planning')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        waypoint = JointTrajectoryPoint(positions=selected['plan']['joints'])
        waypoint.time_from_start.sec = 4
        goal.trajectory.points = [waypoint]
        self.handle = self._wait(self.arm.send_goal_async(goal), 3)
        if not self.handle.accepted:
            raise RuntimeError('Hover goal rejected')
        result = self._wait(self.handle.get_result_async(), 8)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('Hover controller failed')
        self.complete = True
        self._spin(0.5)
        self.report.update(hover_complete=True, actual_joints=self._joints(),
                           base_delta=(self._base() - base).tolist(),
                           gripper_position=dict(zip(self.sensors['joints'].name,
                                                     self.sensors['joints'].position))[
                               'gripper_left_joint'])
        self._record_hover_evidence()
        print('HOVER_COMPLETE_OPEN_GRIPPER ' + str(self.root), flush=True)

    def _record_hover_evidence(self):
        try:
            self._save_hover_evidence()
        except Exception as error:
            # A recording failure must not invite repetition of a completed movement.
            self.report['evidence_error'] = str(error)
            self.node.get_logger().warning('Hover held; evidence recording: ' + str(error))

    def _save_hover_evidence(self):
        deadline = time.monotonic() + 3
        while True:
            self._spin(0.05)
            try:
                self._fresh(self.sensors['rgb'])
                self._fresh(self.sensors['depth'])
                break
            except (KeyError, RuntimeError):
                if time.monotonic() >= deadline:
                    raise RuntimeError('No fresh camera evidence after completed hover')
        cv2.imwrite(str(self.root / 'hover_rgb.jpg'),
                    CvBridge().imgmsg_to_cv2(self.sensors['rgb'], 'bgr8'))
        depth = self.sensors['depth']
        self._fresh(depth)
        wrist = self.buffer.lookup_transform(
            'link1', 'link5', Time.from_msg(depth.header.stamp),
            timeout=Duration(seconds=0.2)).transform
        self.report['wrist_at_depth'] = {
            'translation': [wrist.translation.x, wrist.translation.y, wrist.translation.z],
            'quaternion': [wrist.rotation.x, wrist.rotation.y, wrist.rotation.z, wrist.rotation.w],
        }
        np.savez_compressed(self.root / 'hover_depth.npz',
                            depth=CvBridge().imgmsg_to_cv2(depth),
                            k=np.asarray(self.sensors['info'].k),
                            stamp=[depth.header.stamp.sec, depth.header.stamp.nanosec])

    def close(self):
        """Cancel this trial's unfinished action and persist its outcome."""
        if self.handle is not None and self.handle.accepted and not self.complete:
            try:
                self._wait(self.handle.cancel_goal_async(), 2)
            except Exception as error:
                self.report['cancel_error'] = str(error)
        (self.root / 'result.json').write_text(json.dumps(self.report, indent=2))
        self.node.destroy_node()


def main(args=None):
    """Expose capture-only planning by default and an explicit open-hover mode."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-hover', action='store_true')
    parser.add_argument('--camera-calibration', required=True)
    parser.add_argument('--forward-offset', type=float, default=0.0)
    parser.add_argument('--floor-height', type=float, default=0.0)
    parser.add_argument('--output', default='cleanup_debug')
    options = parser.parse_args(args)
    if (not np.isfinite(options.forward_offset) or abs(options.forward_offset) > 0.03 or
            not np.isfinite(options.floor_height)):
        parser.error('Invalid offset or floor height')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    trial = None
    try:
        trial = HoverTrial(options)
        trial.run()
    except BaseException as error:
        if trial is not None:
            trial.report['error'] = str(error)
        raise
    finally:
        if trial is not None:
            trial.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
