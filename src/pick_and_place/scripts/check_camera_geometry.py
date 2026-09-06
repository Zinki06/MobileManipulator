"""Measure floor consistency at safe wrist poses; never command the mobile base.

This diagnostic reports a candidate hand-eye calibration, but never applies it.
Run only with a clear arm workspace and an approximately level floor.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def rotation(q):
    """Convert a ROS unit quaternion to a rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def floor_plane(points, expected):
    """Fit the dominant approximately horizontal floor, rejecting object surfaces."""
    rng = np.random.default_rng(7)
    best = np.zeros(len(points), bool)
    for _ in range(200):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        if n @ expected < 0:
            n = -n
        if n @ expected < 0.8:
            continue
        keep = np.abs((points-a) @ n) < 0.006
        if keep.sum() > best.sum():
            best = keep
    if best.sum() < 500 or best.mean() < 0.4:
        raise RuntimeError('Insufficient unambiguous floor support')
    center = points[best].mean(0)
    _, _, vt = np.linalg.svd(points[best]-center, full_matrices=False)
    n = vt[-1]
    if n @ expected < 0:
        n = -n
    return n, float(-n @ center), float(np.std(points[best] @ n)), int(best.sum())


def main():
    """Capture three wrist-only poses and restore the initial pose afterwards."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move-wrist', action='store_true')
    parser.add_argument('--poses', nargs=3, type=float, default=[1.2, 1.5707, 1.7707])
    options = parser.parse_args()
    if not options.move_wrist:
        parser.error('Explicit --move-wrist is required for the three-pose diagnostic')
    if any(not 1.2 <= pose <= 1.78 for pose in options.poses):
        parser.error('Diagnostic wrist poses must remain within [1.2, 1.78]')
    rclpy.init()
    node = rclpy.create_node('camera_geometry_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841 - retain subscriptions
    frames = {}
    for key, topic, kind in [
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw', Image),
            ('rgb', '/camera/camera/color/image_raw', Image),
            ('info', '/camera/camera/color/camera_info', CameraInfo)]:
        node.create_subscription(kind, topic, lambda msg, k=key: frames.update({k: msg}),
                                 qos_profile_sensor_data)
    arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    root = Path('/home/user/turtlebot3_ws/cleanup_debug') / (
        'camera_geometry_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    root.mkdir()
    bridge = CvBridge()

    def move(wrist):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint(positions=[0., -0.523, -0.523, wrist])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        future = arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result() or not future.result().accepted:
            raise RuntimeError('Wrist goal not accepted')
        handle = future.result()
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done, timeout_sec=8)
        if not done.done():
            handle.cancel_goal_async()
            raise RuntimeError('Wrist timeout')
        if done.result().status != 4 or done.result().result.error_code != 0:
            raise RuntimeError('Wrist action failed')

    results = []
    moved = False
    try:
        if not arm.wait_for_server(timeout_sec=5):
            raise RuntimeError('Arm unavailable')
        for index, wrist in enumerate(options.poses):
            moved = True
            move(wrist)
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            msg, info = frames['depth'], frames['info']
            stamp = Time.from_msg(msg.header.stamp)
            if (node.get_clock().now()-stamp).nanoseconds * 1e-9 > 0.5:
                raise RuntimeError('Depth is stale')
            tf = buffer.lookup_transform('base_footprint', 'link5', stamp,
                                         timeout=Duration(seconds=0.2)).transform
            extr = buffer.lookup_transform('link5', msg.header.frame_id, stamp,
                                           timeout=Duration(seconds=0.2)).transform
            rw, rc = rotation(tf.rotation), rotation(extr.rotation)
            tw = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            tc = np.array([extr.translation.x, extr.translation.y, extr.translation.z])
            depth = bridge.imgmsg_to_cv2(msg).copy()
            k = np.asarray(info.k).reshape(3, 3)
            yy, xx = np.indices(depth.shape)
            z = depth[::8, ::8].astype(float).ravel()/1000.
            rays = np.column_stack((xx[::8, ::8].ravel(), yy[::8, ::8].ravel(),
                                    np.ones(len(z)))) @ np.linalg.inv(k).T
            points = rays[(z > 0.2) & (z < 2.0)] * z[(z > 0.2) & (z < 2.0), None]
            normal, distance, rms, support = floor_plane(points, (rw @ rc).T[:, 2])
            result = dict(wrist=wrist, normal=normal.tolist(), distance=distance,
                          wrist_normal=rw.T[:, 2].tolist(), wrist_z=float(tw[2]),
                          rms=rms, support=support,
                          predicted_height=float((tw+rw @ tc)[2]),
                          nominal_translation=tc.tolist(), nominal_rotation=rc.tolist())
            results.append(result)
            print(json.dumps(result), flush=True)
            np.savez_compressed(root / f'pose_{index}.npz', depth=depth, k=k,
                                wrist_rotation=rw, wrist_translation=tw,
                                camera_rotation=rc, camera_translation=tc)
            cv2.imwrite(str(root / f'pose_{index}.jpg'), bridge.imgmsg_to_cv2(
                frames['rgb'], desired_encoding='bgr8'))
        normals = np.array([r['normal'] for r in results])
        world = np.array([r['wrist_normal'] for r in results])
        u, _, vt = np.linalg.svd(normals.T @ world)
        candidate_rotation = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
        b = np.array([r['distance']-r['wrist_z'] for r in results]) - world[:, 1]*tc[1]
        txz, _, _, _ = np.linalg.lstsq(world[:, [0, 2]], b, rcond=None)
        print('CANDIDATE_ONLY', json.dumps(dict(
            translation=[float(txz[0]), float(tc[1]), float(txz[1])],
            rotation=candidate_rotation.tolist(),
            normal_residual=float(np.linalg.norm(normals @ candidate_rotation.T-world)),
            height_residual=float(np.linalg.norm(world[:, [0, 2]] @ txz-b)))), flush=True)
    finally:
        (root / 'measurements.json').write_text(json.dumps(results, indent=2))
        if moved:
            move(1.5707)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
