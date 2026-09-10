"""Exercise depth-only planning and carrying transitions with real Nav2 nodes."""

import os
from pathlib import Path
import struct
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

from robot_motion.depth_costmaps import DepthCostmaps


def wait_for(predicate, seconds=12):
    """Wait for a bounded ROS operation."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    assert predicate(), 'Nav2 test deadline exceeded'


def test_depth_detour_and_carry_transition(tmp_path, monkeypatch):
    """Retain static walls while disabling and clearing depth obstacles for carrying."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init()
    node = rclpy.create_node('depth_planning_fixture')
    group = ReentrantCallbackGroup()
    layers = DepthCostmaps(node, group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/map', qos)
    carry = node.create_publisher(Bool, '/cleanup/carrying', qos)
    depth = node.create_publisher(PointCloud2, '/cleanup/obstacle_points', 1)
    grids = {}
    for name in ('local', 'global'):
        node.create_subscription(OccupancyGrid, f'/{name}_costmap/costmap',
                                 lambda msg, key=name: grids.update({key: msg}), 1)
    world = OccupancyGrid()
    world.header.frame_id = 'map'
    world.info.width, world.info.height, world.info.resolution = 100, 80, .05
    world.info.origin.position.x, world.info.origin.position.y = -1., -2.
    world.info.origin.orientation.w = 1.
    world.data = [0] * 8000
    # Static landmark at (1, 1.2), separate from the dynamic obstacle at (1, 0).
    for row in range(62, 66):
        for col in range(38, 42):
            world.data[row * 100 + col] = 100
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.rotation.w = 1.
        transforms.append(msg)
    tf.sendTransform(transforms)
    state = {'carry': False, 'obstacle': True}

    def publish():
        world.header.stamp = node.get_clock().now().to_msg()
        map_pub.publish(world)
        carry.publish(Bool(data=state['carry']))
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'base_link', world.header.stamp
        cloud.height = 1
        cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                        for i, name in enumerate('xyz')]
        cloud.point_step = 12
        points = [] if state['carry'] else [
            (1. if state['obstacle'] else 2.8, (i-15)*.02, .3) for i in range(31)]
        cloud.width = len(points)
        cloud.row_step = 12 * cloud.width
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        depth.publish(cloud)

    node.create_timer(.05, publish)
    source = Path(__file__).parents[2] / 'aruco_localizer/config/nav2_params.yaml'
    processes, logs = [], []
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose', callback_group=group)
    try:
        for package, executable in [('nav2_planner', 'planner_server'),
                                    ('nav2_controller', 'controller_server')]:
            log = (tmp_path / (executable + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen([
                str(Path(get_package_prefix(package)) / 'lib' / package / executable),
                '--ros-args', '--params-file', str(source)], stdout=log, stderr=subprocess.STDOUT))
        for name in ('planner_server', 'controller_server'):
            client = node.create_client(ChangeState, f'/{name}/change_state', callback_group=group)
            assert client.wait_for_service(timeout_sec=15)
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                wait_for(future.done, 20)
                assert future.result().success
        wait_for(lambda: len(grids) == 2 and layers.depth_stamp > 0)
        assert layers.prepare(lambda: False) == ''
        assert planner.wait_for_server(timeout_sec=5)
        goal = ComputePathToPose.Goal()
        goal.planner_id, goal.use_start = 'GridBased', True
        for pose, x in [(goal.start, 0.), (goal.goal, 2.)]:
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
        accepted = planner.send_goal_async(goal)
        wait_for(accepted.done)
        assert accepted.result().accepted
        result = accepted.result().get_result_async()
        wait_for(result.done)
        assert result.result().status == 4
        assert max(abs(p.pose.position.y) for p in result.result().result.path.poses) > .3
        state['carry'] = True
        wait_for(lambda: layers.carrying)
        assert layers.prepare(lambda: False) == ''
        grid = grids['global']
        assert grid.data[40 * 100 + 40] == 0, 'dynamic depth obstacle survived transition'
        assert grid.data[64 * 100 + 40] == 100, 'static landmark was lost'
        state.update(carry=False, obstacle=False)
        wait_for(lambda: not layers.carrying)
        assert layers.prepare(lambda: False) == ''
        assert grids['global'].data[40 * 100 + 40] == 0
        state['obstacle'] = True
        wait_for(lambda: any(grids['global'].data[row * 100 + col] == 100
                             for row in range(38, 43) for col in range(38, 42)))
        # A costmap that stops updating must not satisfy the next transition.
        inactive = node.create_client(ChangeState, '/controller_server/change_state',
                                      callback_group=group)
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = inactive.call_async(request)
        wait_for(future.done)
        assert future.result().success
        state['carry'] = True
        wait_for(lambda: layers.carrying)
        assert 'timed out' in layers.prepare(lambda: False)
        assert layers.mode is None
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        planner.destroy()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
