"""Exercise the installed C++ manager against isolated fake ROS action/services."""

import math
import os
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from cleanup_interfaces.msg import ObjectObservation
from cleanup_interfaces.srv import CaptureObjects, EvaluateGrasp, PlaceObject, PlanCleanup
from geometry_msgs.msg import PointStamped, TransformStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml


def test_precision_tree_has_explicit_controller_and_checker():
    """The manipulation tolerance must not use the ordinary navigation checker."""
    tree = Path(__file__).parents[1] / 'behavior_trees' / 'precision_approach.xml'
    follower = ET.parse(tree).getroot().find('.//FollowPath')
    assert follower.get('controller_id') == 'ApproachPath'
    assert follower.get('goal_checker_id') == 'manipulation_goal_checker'


def test_collection_point_is_outside_marker_three_route():
    """The base approaches the green point without placing objects on the scan line."""
    root = Path(__file__).parents[1]
    cleanup = yaml.safe_load((root / 'config/task_zones.yaml').read_text())['cleanup']
    marker_file = root.parent / 'aruco_localizer/map/new_map_markers.yaml'
    markers = yaml.safe_load(marker_file.read_text())['markers']
    marker = next(m for m in markers if m['id'] == 3)
    drop = cleanup['drop_pose']
    point = drop['placement']
    assert drop['name'] == 'marker_3_outer_collection'
    assert point['y'] == marker['y']
    assert point['x'] == pytest.approx(marker['x'] - 0.4)
    assert drop['x'] - point['x'] == pytest.approx(0.24)
    assert drop['yaw'] == pytest.approx(math.pi)
    assert all(point['x'] < station['x'] for station in cleanup['stations'])


class FakeRobot(Node):
    """Provide no-hardware ROS endpoints and deterministic banana observations."""

    def __init__(self, failure):
        super().__init__('cleanup_test_robot')
        self.failure = failure
        self.x, self.y, self.yaw = 1.0, -0.5, math.pi
        self.picks = 0
        self.plans = 0
        self.releases = 0
        self.placements = []
        self.approaches = 0
        self.station2_spins = 0
        self.nav_attempts = 0
        self.captures = []
        self.capture_headings = []
        self.events = []
        self.last_pick_target = None
        self.last_capture_stamp = None
        self.tf = TransformBroadcaster(self)
        self.mode = self.create_publisher(String, '/aruco/localization_mode', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.maps = [self.create_publisher(OccupancyGrid, f'/{name}_costmap/costmap', 1)
                     for name in ('global', 'local')]
        self.guard = self.create_publisher(
            String, '/motion_guard/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/cleanup/events',
                                 lambda msg: self.events.append(msg.data), 50)
        self.create_subscription(PointStamped, '/cleanup/pick_target',
                                 lambda msg: setattr(self, 'last_pick_target', msg), 10)
        self.timer = self.create_timer(0.02, self.publish_pose)
        self.nav = ActionServer(self, NavigateToPose, '/navigate_to_pose', self.navigate)
        self.spin = ActionServer(self, Spin, '/spin', self.rotate)
        self.create_service(CaptureObjects, '/cleanup/capture_objects', self.capture)
        self.create_service(PlanCleanup, '/cleanup/plan_objects', self.plan)
        self.create_service(EvaluateGrasp, '/cleanup/evaluate_grasp', self.evaluate)
        self.create_service(Trigger, '/execute_pick_and_place', self.pick)
        self.create_service(Trigger, '/open_gripper', self.release)
        self.create_service(PlaceObject, '/cleanup/place_object', self.place)
        self.create_service(Trigger, '/park_arm', self.ok)
        self.create_service(Trigger, '/observe_floor', self.ok)

    def publish_pose(self):
        """Publish a connected, current map/base tree on the test domain only."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.rotation.z = math.sin(self.yaw / 2)
        msg.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(msg)
        world = TransformStamped()
        world.header.stamp = msg.header.stamp
        world.header.frame_id = 'map'
        world.child_frame_id = 'odom'
        world.transform.rotation.w = 1.0
        self.tf.sendTransform(world)
        self.mode.publish(String(data='TRACKING'))
        guard = ('BLOCKED: depth stale'
                 if self.failure == 'nav_blocked' and self.nav_attempts else 'READY')
        if self.failure == 'nav_fault' and any(
                event.startswith('NAVIGATION_RETRY_WAIT|') for event in self.events):
            guard = 'FAULT: fixture localization fault'
        if self.failure == 'nav_once' and self.nav_attempts == 1:
            guard = 'BLOCKED: command timeout'
        self.guard.publish(String(data=guard))
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        if self.failure == 'nav_moving' and self.nav_attempts:
            odom.twist.twist.linear.x = 0.1
        self.odom.publish(odom)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = msg.header.stamp, 'map'
        grid.info.width, grid.info.height, grid.info.resolution = 2, 2, 0.05
        grid.info.origin.orientation.w = 1.0
        grid.data = [0, 100, -1, 0]
        for publisher in self.maps:
            publisher.publish(grid)

    def navigate(self, handle):
        """Apply a navigation pose without moving any physical device."""
        pose = handle.request.pose.pose
        self.nav_attempts += 1
        if self.failure.startswith('nav_') and (
                self.failure != 'nav_once' or self.nav_attempts == 1):
            handle.abort()
            return NavigateToPose.Result()
        is_drop = pose.position.x < 0.0 and abs(pose.position.y - 1.5) < 0.01
        if is_drop:
            assert handle.request.behavior_tree.endswith('precision_approach.xml')
        if handle.request.behavior_tree and not is_drop:
            assert handle.request.behavior_tree.endswith('precision_approach.xml')
            self.approaches += 1
            if self.failure == 'never_reach' or (
                    self.failure == 'early_arrival' and self.approaches == 1):
                handle.succeed()
                return NavigateToPose.Result()
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.publish_pose()
        handle.succeed()
        return NavigateToPose.Result()

    def rotate(self, handle):
        """Simulate under-rotation so observation alignment must check residuals."""
        if abs(self.y - 0.5) < 0.01 and abs(self.x) < 0.01:
            self.station2_spins += 1
            if (self.failure == 'spin' and 'scan_station_2' in self.captures) or (
                    self.failure == 'spin_once' and self.station2_spins == 2):
                handle.abort()
                return Spin.Result()
        self.yaw += handle.request.target_yaw * 0.80
        self.publish_pose()
        handle.succeed()
        return Spin.Result()

    def capture(self, request, response):
        """Return one persistent UUID until it is picked, or inject failure."""
        self.captures.append(request.station_name)
        self.capture_headings.append((request.station_name, request.heading_index))
        if self.failure == 'carry_occluded' and request.heading_index == 251:
            response.message = 'Camera occluded by carried banana'
            return response
        if self.failure == 'model':
            response.message = 'YOLO unavailable: fixture error'
            return response
        response.success = True
        response.message = 'fixture capture'
        response.image_reference = '/tmp/fixture.jpg'
        if request.station_name.startswith('scan_station_0') and (
                not self.picks or self.failure in {'pick_remains', 'empty_grasp'} and
                self.picks == 1):
            if self.failure == 'review' and request.heading_index == 249:
                return response
            obj = ObjectObservation(
                object_uuid='banana-fixture', class_name='banana', confidence=0.9,
                image_reference='/tmp/fixture.jpg', confirmation_count=3,
                grasp_strategy=('legacy_center' if self.failure == 'legacy_candidate'
                                else 'candidate_body'),
                grasp_valid=self.failure != 'invalid_centroid' and not (
                    self.failure == 'pick_remains' and request.heading_index == 251),
                grasp_reason='fixture body' if self.failure != 'invalid_centroid' else 'clipped',
            )
            obj.header.frame_id = 'map'
            obj.header.stamp = self.get_clock().now().to_msg()
            if request.station_name.endswith('_approach') and self.failure in {
                    'processed_frame', 'expired_frame'}:
                # Replay actual capture-to-pick latency without sleeping or forging a fresh stamp.
                delay = 2.07 if self.failure == 'processed_frame' else 7.0
                obj.header.stamp = (self.get_clock().now() - Duration(seconds=delay)).to_msg()
                self.last_capture_stamp = obj.header.stamp
            obj.centroid.x, obj.centroid.y, obj.centroid.z = 1.0, 0.2, 0.03
            response.observations = [obj]
        return response

    def plan(self, request, response):
        """Assert planning happens only after a precise observation."""
        assert 'scan_station_0_observation' in self.captures
        self.plans += 1
        response.success = True
        response.action = 'collect_to_drop_zone'
        response.object_uuid = request.observations[0].object_uuid
        response.planner_name = 'fixture'
        return response

    def pick(self, request, response):
        """Record a simulated grasp."""
        assert self.plans == 1
        if self.failure == 'processed_frame':
            assert self.last_pick_target is not None
            assert self.last_pick_target.header.stamp == self.last_capture_stamp
        assert math.hypot(self.x - 1.0, self.y - 0.2) < 0.26
        self.picks += 1
        if self.failure == 'empty_grasp' and self.picks == 1:
            response.success = False
            response.message = 'EMPTY_GRASP: no stable finger obstruction after closing'
            return response
        if self.failure == 'pick':
            response.success = False
            response.message = 'Object grasped, but arm failed to return home'
            return response
        return self.ok(request, response)

    def evaluate(self, request, response):
        """Return a feasible base pose; arrival alone cannot satisfy this check."""
        point = request.target.point
        dx, dy = point.x - self.x, point.y - self.y
        response.success = True
        if self.failure in {'geometry_refresh', 'geometry_rejected'} and (
                self.failure == 'geometry_rejected' or
                ('scan_station_0_approach', 249) not in self.capture_headings):
            response.message = 'surface_height=0.021; insufficient_body_depth=27'
            return response
        response.reachable = math.hypot(dx, dy) < 0.26
        response.approach_available = True
        yaw = math.atan2(dy, dx)
        pose = response.approach_pose
        pose.header.frame_id = 'map'
        pose.pose.position.x = point.x - 0.20 * math.cos(yaw)
        pose.pose.position.y = point.y - 0.20 * math.sin(yaw)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return response

    def release(self, request, response):
        """Count explicit releases, including partially completed pick recovery."""
        self.releases += 1
        return self.ok(request, response)

    def place(self, request, response):
        """Require the map collection point, not the robot's navigation position."""
        assert abs(self.x + 0.16) < 0.01 and abs(self.y - 1.5) < 0.01
        point = request.floor_target.point
        assert request.floor_target.header.frame_id == 'map'
        assert abs(point.x + 0.40) < 0.001 and abs(point.y - 1.5) < 0.001
        assert request.release_height == 0.035
        self.placements.append((point.x, point.y))
        if self.failure == 'place':
            response.message = 'fixture lowering rejected'
            return response
        response.success = True
        response.released = True
        return response

    def ok(self, request, response):
        """Acknowledge simulated arm operations."""
        response.success = True
        return response


@pytest.mark.parametrize('failure', [
    '', 'model', 'review', 'pick', 'pick_remains', 'empty_grasp', 'early_arrival', 'never_reach',
    'spin', 'spin_once',
    'invalid_centroid', 'legacy_candidate', 'place', 'geometry_refresh', 'geometry_rejected',
    'processed_frame', 'expired_frame',
    'carry_occluded', 'nav_once', 'nav_always', 'nav_blocked', 'nav_moving', 'nav_fault', 'nav_stop',
])
def test_mission_flow(tmp_path, monkeypatch, failure):
    """Test scan/review/plan/pick/drop order and bounded perception failure paths."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(160 + os.getpid() % 50))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    config_path = Path(__file__).parents[1] / 'config' / 'task_zones.yaml'
    config = yaml.safe_load(config_path.read_text())
    config['cleanup']['scan']['settle_seconds'] = 0.05
    local_config = tmp_path / 'task.yaml'
    local_config.write_text(yaml.safe_dump(config))
    rclpy.init()
    robot = FakeRobot(failure)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    executable = Path(get_package_prefix('cleanup_task_manager')) / 'lib' / \
        'cleanup_task_manager' / 'cleanup_task_manager_node'
    approach_bt = Path(__file__).parents[1] / 'behavior_trees' / 'precision_approach.xml'
    output = (tmp_path / 'manager_output.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', f'task_config_path:={local_config}',
        '-p', f'debug_output_root:={tmp_path / "evidence"}',
        '-p', f'approach_behavior_tree:={approach_bt}',
        '-p', 'require_candidate_grasp:=true',
        '-p', f'verify_pick_with_camera:={str(failure != "carry_occluded").lower()}',
        '-p', 'navigation_retry_delay:=0.5', '-p', 'navigation_retry_timeout:=1.5',
    ], stdout=output, stderr=subprocess.STDOUT)
    try:
        client = robot.create_client(Trigger, '/start_cleanup')
        assert client.wait_for_service(timeout_sec=10)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            future = client.call_async(Trigger.Request())
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert future.done()
            if future.result().success:
                break
            time.sleep(0.1)
        else:
            pytest.fail('Manager dependencies did not become ready')
        deadline = time.monotonic() + 30
        nav_failure = failure.startswith('nav_') and failure != 'nav_once'
        terminal = ('MISSION_FAILED|' if
                    failure in {'model', 'spin', 'spin_once', 'place'} or nav_failure
                    else 'MISSION_COMPLETE|')
        if failure == 'nav_stop':
            while time.monotonic() < deadline and not any(
                    e.startswith('NAVIGATION_RETRY_WAIT|') for e in robot.events):
                time.sleep(0.02)
            stop = robot.create_client(Trigger, '/stop_cleanup')
            assert stop.wait_for_service(timeout_sec=3)
            stopped = stop.call_async(Trigger.Request())
            while not stopped.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert stopped.done() and stopped.result().success
            time.sleep(1.7)
            assert robot.nav_attempts == 1
            assert robot.releases == 1
            assert any(e.startswith("MISSION_RELEASE_COMPLETE|") for e in robot.events)
            assert not any(e.startswith('NAVIGATION_RETRY|') for e in robot.events)
            return
        while time.monotonic() < deadline:
            if any(event.startswith(terminal) for event in robot.events):
                break
            assert process.poll() is None
            time.sleep(0.05)
        assert any(event.startswith(terminal) for event in robot.events), robot.events
        assert any(e.startswith("MISSION_RELEASE_COMPLETE|") for e in robot.events)
        if failure.startswith('nav_'):
            files = list((tmp_path / 'evidence').glob('*/navigation_failure_*/context.yaml'))
            assert files
            context = yaml.safe_load(files[0].read_text())
            assert context['robot_pose_available']
            assert context['goal_frame'] == 'map'
            grid = yaml.safe_load((files[0].parent / 'global_costmap.yaml').read_text())
            assert grid['available'] and grid['data'] == [0, 100, -1, 0]
            retries = sum(e.startswith('NAVIGATION_RETRY|') for e in robot.events)
            assert retries == (1 if failure == 'nav_once' else 2 if failure == 'nav_always'
                               else 0)
        if nav_failure:
            assert robot.nav_attempts == (3 if failure == 'nav_always' else 1)
            assert not robot.picks and not robot.captures
        elif failure == 'model':
            assert not robot.plans and not robot.picks
            assert len(robot.captures) == 1
        elif failure in {'spin', 'spin_once'}:
            assert not any(event.startswith('SCAN_RECENTER_START|') for event in robot.events)
            assert not any(event.startswith('MISSION_COMPLETE|') for event in robot.events)
        elif failure == 'place':
            assert robot.placements == [(-0.4, 1.5)]
            assert robot.releases == 1
            assert not any(event.startswith('OBJECT_COLLECTED|') for event in robot.events)
        else:
            assert sum(event.startswith('SCAN_CAPTURE_START|')
                       for event in robot.events) == 6 * config['cleanup']['scan']['turns']
            assert any(event.startswith('OBSERVATION_ALIGN_START|') for event in robot.events)
            no_pick = failure in {'review', 'never_reach', 'invalid_centroid', 'legacy_candidate',
                                 'geometry_rejected', 'expired_frame'}
            assert robot.picks == (0 if no_pick else
                                   2 if failure in {'pick_remains', 'empty_grasp'} else 1)
            if failure in {'pick_remains', 'empty_grasp'}:
                assert ('scan_station_0_approach', 249) in robot.capture_headings
            assert robot.plans == (0 if failure == 'review' else 1)
            if failure == 'early_arrival':
                assert robot.approaches == 2
            if failure == 'never_reach':
                assert robot.approaches == 3
            if failure in {'processed_frame', 'expired_frame'}:
                assert any(e.startswith('PICK_OBSERVATION_AGE|') and 'limit=6.000000s' in e
                           for e in robot.events)
                if failure == 'processed_frame':
                    assert any(e.startswith('OBJECT_COLLECTED|') for e in robot.events)
                    assert not any(e.startswith('OBJECT_FAILED|') for e in robot.events)
                else:
                    assert any('outside processing budget' in e for e in robot.events)
                    assert robot.last_pick_target is None
            if failure in {'geometry_refresh', 'geometry_rejected'}:
                assert robot.capture_headings.count(('scan_station_0_approach', 249)) == 1
                assert sum(e.startswith('GRASP_REOBSERVE_REQUIRED|') for e in robot.events) == 1
                if failure == 'geometry_rejected':
                    assert robot.approaches == 0
                    assert any('approaches_executed=0' in e and 'fresh_reobservation=true' in e
                               for e in robot.events if e.startswith('OBJECT_FAILED|'))
                else:
                    assert robot.approaches == 1
            if failure == 'pick':
                assert robot.releases == 2
                assert any(event.startswith('RECOVERY_RELEASE_START|')
                           for event in robot.events)
                assert not any(event.startswith('OBJECT_COLLECTED|')
                               for event in robot.events)
            if failure == 'carry_occluded':
                assert not any(heading == 251 for _, heading in robot.capture_headings)
                assert any('carrying on odometry' in e for e in robot.events)
                assert robot.placements == [(-0.4, 1.5)]
                assert any(e.startswith('OBJECT_COLLECTED|') for e in robot.events)
            if not failure:
                assert any(event.startswith('OBJECT_COLLECTED|') for event in robot.events)
                assert robot.placements == [(-0.4, 1.5)]
                assert robot.releases == 1
    finally:
        process.terminate()
        process.wait(timeout=10)
        output.close()
        executor.shutdown()
        thread.join(timeout=5)
        robot.destroy_node()
        rclpy.shutdown()
