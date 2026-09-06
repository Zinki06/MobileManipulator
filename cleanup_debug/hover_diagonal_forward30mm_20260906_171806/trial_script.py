"""One open-gripper hover, with no close, descent, return, or base command."""
import json
import math
from pathlib import Path
import subprocess
import time
from datetime import datetime
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from cleanup_perception.depth_geometry import central_grasp_sample
from cleanup_perception.jaw_geometry import jaw_center_sample
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from cleanup_interfaces.srv import CaptureObjects
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from trajectory_msgs.msg import JointTrajectoryPoint

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = rclpy.create_node('open_gripper_hover_trial')
root = Path('/home/user/turtlebot3_ws/cleanup_debug') / ('hover_diagonal_forward30mm_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
root.mkdir()
s = {}
for key, topic, kind in [('joints', '/joint_states', JointState), ('odom', '/odom', Odometry),
                         ('rgb', '/camera/camera/color/image_raw', Image),
                         ('depth', '/camera/camera/aligned_depth_to_color/image_raw', Image),
                         ('info', '/camera/camera/color/camera_info', CameraInfo)]:
    n.create_subscription(kind, topic, lambda m, k=key: s.update({k: m}), qos_profile_sensor_data)
buffer = Buffer()
listener = TransformListener(buffer, n)
arm = ActionClient(n, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
events = n.create_publisher(String, '/cleanup/events', 20)
handle = None
complete = False
report = {}

def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)

def wait(future, seconds):
    rclpy.spin_until_future_complete(n, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError('Timeout; inspect before another command')
    return future.result()

def call(kind, name, request, seconds):
    client = n.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError('Service unavailable: ' + name)
    result = wait(client.call_async(request), seconds)
    if not result.success:
        raise RuntimeError(result.message)
    return result

def fresh(message):
    age = (n.get_clock().now() - Time.from_msg(message.header.stamp)).nanoseconds * 1e-9
    if not 0 <= age < .25:
        raise RuntimeError('Stale sensor feedback')

def position(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return np.array([p.x, p.y, yaw])

try:
    deadline = time.monotonic() + 10
    while not all(k in s for k in ('odom', 'joints', 'rgb', 'depth', 'info')):
        if time.monotonic() > deadline:
            raise RuntimeError('Sensor discovery timeout; no movement')
        rclpy.spin_once(n, timeout_sec=.05)
    spin_for(.4)
    fresh(s['odom'])
    base = position(s['odom'])
    if abs(s['odom'].twist.twist.linear.x) > .005 or abs(s['odom'].twist.twist.angular.z) > .02:
        raise RuntimeError('Base is moving')
    call(Trigger, '/observe_floor', Trigger.Request(), 5)
    spin_for(.4)
    capture = call(CaptureObjects, '/cleanup/capture_objects', CaptureObjects.Request(
        mission_id=root.name, station_name='station_0_arm_only', heading_index=249,
        burst_frames=3, min_confirmations=3), 20)
    objects = [o for o in capture.observations if o.class_name == 'banana' and o.grasp_valid]
    if len(objects) != 1:
        raise RuntimeError('Expected one complete banana; no hover movement')
    obj = objects[0]
    report['image'] = obj.image_reference
    data = np.load(Path(obj.image_reference).with_suffix('.npz'))
    acquisition = Time(seconds=int(data['image_stamp'][0]), nanoseconds=int(data['image_stamp'][1]))
    wrist = buffer.lookup_transform('link1', 'link5', acquisition,
                                   timeout=Duration(seconds=.2)).transform
    rw = Rotation.from_quat([wrist.rotation.x, wrist.rotation.y,
                             wrist.rotation.z, wrist.rotation.w]).as_matrix()
    tw = np.array([wrist.translation.x, wrist.translation.y, wrist.translation.z])
    cfg = yaml.safe_load(Path('/home/user/turtlebot3_ws/src/cleanup_perception/config/grasp_camera_20260906.yaml').read_text())['scan_perception_node']['ros__parameters']
    rc = np.asarray(cfg['grasp_camera_rotation']).reshape(3, 3)
    tc = np.asarray(cfg['grasp_camera_translation'])
    if not bool(data['calibration_enabled']):
        raise RuntimeError('Capture does not use experimental calibration')
    original = central_grasp_sample(data['mask'], data['depth'], .1, 2.5, 30)
    sample, detail = jaw_center_sample(data['mask'], data['depth'], data['camera_k'],
                                       rw @ rc, tw + rw @ tc, original)
    report['jaw_geometry'] = detail
    report['jaw_pixel'] = [sample.u, sample.v]
    report['jaw_depth_m'] = sample.distance
    report['acquisition_stamp'] = data['image_stamp'].tolist()
    preview = cv2.imread(obj.image_reference)
    if preview is not None:
        cv2.circle(preview, (round(sample.u), round(sample.v)), 12, (255,255,0), 2)
        cv2.putText(preview, 'CYAN: jaw center before +30mm forward', (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (255,255,0), 2)
        cv2.imwrite(str(root / 'jaw_target_preview.jpg'), preview)
    report['pixel'] = [float(v) for v in obj.grasp_pixel]
    call(Trigger, '/open_gripper', Trigger.Request(), 5)
    spin_for(.25)
    fresh(s['joints']); fresh(s['odom'])
    if np.max(np.abs(position(s['odom'])-base)) > .005:
        raise RuntimeError('Base moved during capture')
    joints = dict(zip(s['joints'].name, s['joints'].position))
    if abs(joints['gripper_left_joint']-.019) > .0015:
        raise RuntimeError('Gripper did not open fully')
    if (n.get_clock().now()-Time.from_msg(obj.header.stamp)).nanoseconds*1e-9 > 5:
        raise RuntimeError('Target too old')
    target = PointStamped(header=obj.header, point=obj.centroid)
    target.header.stamp = Time().to_msg()
    local = buffer.transform(target, 'link1', timeout=Duration(seconds=.2))
    floor = PointStamped()
    floor.header.frame_id = 'map'
    floor.point.x, floor.point.y = target.point.x, target.point.y
    local_floor = buffer.transform(floor, 'link1', timeout=Duration(seconds=.2))
    start = [joints['joint'+str(i)] for i in range(1, 5)]
    xyz = list(detail['jaw_center_link1'])
    report['capture_centroid_link1'] = [local.point.x, local.point.y, local.point.z]
    if np.linalg.norm(np.asarray(detail['original_link1']) - np.array(report['capture_centroid_link1'])) > .010:
        raise RuntimeError('Timestamped jaw geometry disagrees with perception centroid')
    report['observed_target_link1'] = xyz.copy()
    report['alignment_offset_link1'] = [0.030, 0.0, 0.0]
    # User-authorized horizontal robot-forward correction, NOT along tilted tool X.
    xyz[0] += 0.030
    report['target_link1'] = xyz
    report['start_joints'] = start
    raw = subprocess.run(['/tmp/grasp_hover_plan'], input=' '.join(map(str, xyz+[local_floor.point.z]+start)),
                         text=True, capture_output=True, check=True)
    plan = json.loads(raw.stdout)
    report['plan'] = plan
    print(json.dumps(report), flush=True)
    if not arm.wait_for_server(timeout_sec=3):
        raise RuntimeError('Arm unavailable')
    spin_for(.1)
    fresh(s['joints']); fresh(s['odom'])
    if (n.get_clock().now() - acquisition).nanoseconds * 1e-9 > 5:
        raise RuntimeError('Target expired before movement')
    latest = dict(zip(s['joints'].name, s['joints'].position))
    if max(abs(latest['joint'+str(i+1)]-start[i]) for i in range(4)) > .02:
        raise RuntimeError('Arm moved during planning')
    if np.max(np.abs(position(s['odom'])-base)) > .005:
        raise RuntimeError('Base moved during planning')
    events.publish(String(data=f'TARGET_VIEW_START|UUID={obj.object_uuid}; hover forward30mm; no pick'))
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    p = JointTrajectoryPoint(positions=plan['joints'])
    p.time_from_start.sec = 4
    goal.trajectory.points = [p]
    handle = wait(arm.send_goal_async(goal), 3)
    if not handle.accepted:
        raise RuntimeError('Hover rejected')
    result = wait(handle.get_result_async(), 8)
    if result.status != 4 or result.result.error_code != 0:
        raise RuntimeError('Hover controller failed')
    complete = True
    spin_for(.5)
    fresh(s['joints'])
    after = dict(zip(s['joints'].name, s['joints'].position))
    report['actual_joints'] = after
    report['hover_complete'] = True
    fresh(s['odom'])
    report['base_delta'] = (position(s['odom'])-base).tolist()
    report['rgb_stamp'] = [s['rgb'].header.stamp.sec, s['rgb'].header.stamp.nanosec]
    bridge = CvBridge()
    cv2.imwrite(str(root / 'hover_rgb.jpg'), bridge.imgmsg_to_cv2(s['rgb'], 'bgr8'))
    depth_msg = s['depth']
    wrist = buffer.lookup_transform('link1', 'link5', Time.from_msg(depth_msg.header.stamp),
                                    timeout=Duration(seconds=.2)).transform
    report['wrist_at_depth'] = dict(translation=[wrist.translation.x, wrist.translation.y, wrist.translation.z],
        quaternion=[wrist.rotation.x, wrist.rotation.y, wrist.rotation.z, wrist.rotation.w])
    np.savez_compressed(root / 'hover_depth.npz', depth=bridge.imgmsg_to_cv2(depth_msg),
                        k=np.asarray(s['info'].k), stamp=[depth_msg.header.stamp.sec, depth_msg.header.stamp.nanosec])
    print('HOVER_COMPLETE_OPEN_GRIPPER', str(root), json.dumps(after), flush=True)
except BaseException as error:
    report['error'] = str(error)
    if handle is not None and handle.accepted and not complete:
        try:
            wait(handle.cancel_goal_async(), 2)
        except Exception:
            pass
    raise
finally:
    import shutil
    shutil.copy2(__file__, root / 'trial_script.py')
    shutil.copy2('/tmp/grasp_hover_plan.cpp', root / 'hover_plan.cpp')
    shutil.copy2('/home/user/turtlebot3_ws/src/cleanup_perception/cleanup_perception/jaw_geometry.py', root / 'jaw_geometry.py')
    (root / 'result.json').write_text(json.dumps(report, indent=2))
    events.publish(String(data='OBJECT_FAILED|Hover diagnostic ended; no pick requested'))
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
