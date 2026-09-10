"""One admission, supervision and encoder-turn path for patrol and cleanup."""

import json
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from robot_motion.depth_costmaps import DepthCostmaps
from robot_motion.policy import ProgressBudget, guard_allows_motion, spin_velocity, wrap


class MotionExecutor(Node):
    """Serialize robot motion, forward translation to Nav2, close turns on odometry."""

    def __init__(self):
        super().__init__('motion_executor')
        self.wait_limit = float(self.declare_parameter('safety_wait_timeout', 30.0).value)
        self.nav_limit = float(self.declare_parameter('navigation_timeout', 120.0).value)
        if not 1.0 <= self.wait_limit <= 30.0 or not 5.0 <= self.nav_limit <= 300.0:
            raise ValueError('Invalid motion time limits')
        self.spin_max = float(self.declare_parameter('spin_max_velocity', 0.30).value)
        self.spin_gain = float(self.declare_parameter('spin_gain', 1.0).value)
        self.spin_acceleration = float(self.declare_parameter('spin_acceleration', 0.4).value)
        self.readiness_hold = float(self.declare_parameter('readiness_hold', 0.30).value)
        self.rotation_settle = float(self.declare_parameter('rotation_settle', 0.25).value)
        self.reuse_readiness = self.declare_parameter('reuse_stationary_readiness', False).value
        spin_velocity(1., maximum=self.spin_max, gain=self.spin_gain,
                      acceleration=self.spin_acceleration)
        if not 0.15 <= self.readiness_hold <= 1.0 or not 0.10 <= self.rotation_settle <= 1.0:
            raise ValueError('Invalid encoder stabilization duration')
        self._stationary_since = None
        self._odom_received = 0.0
        self._lock = threading.Lock()
        self._reserved = False
        self._fault = ''
        self._guard = ('', 0.0)
        self._odom = None
        self._last_event = ''
        self._motion_id = ''
        self._motion_kind = ''
        self._group = ReentrantCallbackGroup()
        self._costmaps = DepthCostmaps(self, self._group) if self.declare_parameter(
            'sync_depth_costmaps', True).value else None
        self._commands = self.create_publisher(Twist, '/cmd_vel_nav', 1)
        self._events = self.create_publisher(String, '/motion/events', 30)
        self._inhibit = self.create_publisher(
            String, '/motion/inhibit', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/motion_guard/status', self._on_guard,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                 callback_group=self._group)
        self._navigation = ActionServer(
            self, NavigateToPose, '/motion/navigate_to_pose', self.navigate,
            goal_callback=self._accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group)
        self._rotation = ActionServer(
            self, Spin, '/motion/spin', self.rotate,
            goal_callback=self._accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group)

    def _on_guard(self, msg):
        self._guard = (msg.data, time.monotonic())

    def destroy_node(self):
        """Release native action handles before their ROS context is shut down."""
        self._navigation.destroy()
        self._rotation.destroy()
        self._nav.destroy()
        return super().destroy_node()

    def _on_odom(self, msg):
        now = time.monotonic()
        stopped = (abs(msg.twist.twist.linear.x) < 0.015 and
                   abs(msg.twist.twist.angular.z) < 0.03)
        if not stopped or now - self._odom_received > 0.15:
            self._stationary_since = None
        if stopped and self._stationary_since is None:
            self._stationary_since = now
        self._odom_received = now
        self._odom = msg

    def _accept(self, request):
        with self._lock:
            if self._reserved or self._fault:
                return GoalResponse.REJECT
            if isinstance(request, Spin.Goal):
                if not math.isfinite(request.target_yaw) or abs(request.target_yaw) > 2 * math.pi:
                    return GoalResponse.REJECT
            else:
                p, q = request.pose.pose.position, request.pose.pose.orientation
                if request.pose.header.frame_id != 'map' or not all(
                        math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
                    return GoalResponse.REJECT
                if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01:
                    return GoalResponse.REJECT
            self._reserved = True
            return GoalResponse.ACCEPT

    def _health(self):
        if self._fault:
            return 'FAULT: ' + self._fault
        if self._costmaps and self._costmaps.issue():
            return 'FAULT: ' + self._costmaps.issue()
        status, received = self._guard
        if time.monotonic() - received > 0.5:
            return 'BLOCKED: safety status stale'
        odom = self._odom
        if odom is None:
            return 'BLOCKED: odometry missing'
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        q = odom.pose.pose.orientation
        twist = odom.twist.twist
        if not -0.1 <= age < 0.5 or not all(
                math.isfinite(v) for v in (
                    q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)) or abs(
                        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01:
            return 'BLOCKED: odometry stale/invalid'
        return status

    def _yaw(self):
        q = self._odom.pose.pose.orientation
        return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def _event(self, state, **details):
        event = json.dumps({'state': state, 'goal_id': self._motion_id,
                            'action': self._motion_kind, **details}, ensure_ascii=False)
        if event != self._last_event:
            self._events.publish(String(data=event))
            self.get_logger().info('[MOTION] ' + event)
            self._last_event = event

    def _stop(self):
        if self.context.ok():
            self._commands.publish(Twist())

    def _finish(self, goal, result, reason='', success=False):
        if goal.is_cancel_requested:
            goal.canceled()
            self._event('FAULT' if self._fault else 'CANCELED', reason=self._fault or reason)
        elif success:
            goal.succeed()
            self._event('SUCCEEDED')
        else:
            goal.abort()
            self._event('FAILED', reason=reason)
        return result

    def _ready(self, goal):
        if self._costmaps:
            reason = self._costmaps.prepare(lambda: goal.is_cancel_requested or not rclpy.ok())
            if reason:
                self._fault = reason
                self._inhibit.publish(String(data=reason))
                return reason
        started, stable = time.monotonic(), None
        while rclpy.ok() and not goal.is_cancel_requested:
            health = self._health()
            if health.startswith('FAULT:'):
                return health
            odom = self._odom
            stopped = odom is not None and abs(odom.twist.twist.linear.x) < 0.015 and \
                abs(odom.twist.twist.angular.z) < 0.03
            if guard_allows_motion(health) and stopped:
                if self.reuse_readiness:
                    stable = self._stationary_since if \
                        time.monotonic() - self._odom_received <= 0.15 else None
                else:
                    stable = stable or time.monotonic()
                if stable is not None and time.monotonic() - stable >= self.readiness_hold:
                    return ''
            else:
                stable = None
            self._event('WAITING_READINESS', reason=(
                'waiting for stationary encoders' if guard_allows_motion(health) else health))
            if time.monotonic() - started > self.wait_limit:
                return 'readiness timeout: ' + health
            time.sleep(0.02)
        return 'canceled/shutdown'

    def rotate(self, goal):
        """Turn by measured angle; safety-blocked time is not active turn time."""
        result = Spin.Result()
        self._motion_id = bytes(goal.goal_id.uuid).hex()
        self._motion_kind = 'spin'
        try:
            reason = self._ready(goal)
            if reason:
                return self._finish(goal, result, reason)
            last_yaw, measured = self._yaw(), 0.0
            last_tick = started = time.monotonic()
            budget = ProgressBudget(30.0, self.wait_limit)
            settled = None
            while rclpy.ok() and not goal.is_cancel_requested:
                now, health = time.monotonic(), self._health()
                if health.startswith('FAULT:'):
                    return self._finish(goal, result, health)
                blocked = not guard_allows_motion(health)
                reason = budget.update(now - last_tick, blocked)
                last_tick = now
                if reason or now - started > 90.0:
                    return self._finish(goal, result, reason or 'total rotation deadline')
                # Never integrate a stale sample; cancellation/timeout remains available.
                if 'odometry' in health:
                    self._stop()
                    time.sleep(0.02)
                    continue
                yaw = self._yaw()
                measured += wrap(yaw - last_yaw)
                last_yaw = yaw
                error = goal.request.target_yaw - measured
                command = Twist()
                command.angular.z = spin_velocity(
                    error, maximum=self.spin_max, gain=self.spin_gain,
                    acceleration=self.spin_acceleration)
                # Continue intent through the safety chain so collision clearing can be
                # observed. The final guard alone authorizes motor output.
                self._commands.publish(command)
                self._event('WAITING_SAFETY' if blocked else 'ROTATING',
                            reason=health if blocked else '')
                feedback = Spin.Feedback()
                feedback.angular_distance_traveled = measured
                goal.publish_feedback(feedback)
                if abs(error) <= 0.025 and abs(self._odom.twist.twist.angular.z) < 0.03:
                    settled = settled or now
                    if now - settled >= self.rotation_settle:
                        self._event('TURN_MEASURED', requested=goal.request.target_yaw,
                                    measured=measured, residual=error,
                                    active_seconds=budget.active)
                        return self._finish(goal, result, success=True)
                else:
                    settled = None
                time.sleep(0.02)
            return self._finish(goal, result,
                                'cancel requested' if goal.is_cancel_requested else 'shutdown')
        except Exception as error:
            return self._finish(goal, result, str(error))
        finally:
            self._stop()
            with self._lock:
                self._reserved = False

    def _wait_future(self, future, timeout):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout)
        return future.done()

    def _cancel_backend(self, handle, result_future):
        confirmed = False
        try:
            handle.cancel_goal_async()
            if self._wait_future(result_future, 3.0):
                confirmed = result_future.result().status in (
                    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_SUCCEEDED)
        except Exception:
            confirmed = False
        if not confirmed:
            self._fault = 'Nav2 cancellation unconfirmed; restart after inspection'
            self._inhibit.publish(String(data=self._fault))

    def navigate(self, goal):
        """Proxy one Nav2 goal with admission, safety supervision and confirmed cancel."""
        result, backend, pending = NavigateToPose.Result(), None, None
        submitted = False
        self._motion_id = bytes(goal.goal_id.uuid).hex()
        self._motion_kind = 'navigate_to_pose'
        try:
            reason = self._ready(goal)
            if reason:
                return self._finish(goal, result, reason)
            if not self._nav.wait_for_server(timeout_sec=2.0):
                return self._finish(goal, result, 'Nav2 server unavailable')
            submitted = True
            send = self._nav.send_goal_async(
                goal.request, feedback_callback=lambda msg: goal.publish_feedback(msg.feedback)
                if goal.is_active else None)
            if not self._wait_future(send, 3.0):
                self._fault = 'Nav2 acceptance unknown; restart after inspection'
                self._inhibit.publish(String(data=self._fault))
                # A late accepted goal must also be canceled, never orphaned.
                send.add_done_callback(lambda future: future.result().cancel_goal_async()
                                       if future.result() and future.result().accepted else None)
                return self._finish(goal, result, self._fault)
            backend = send.result()
            if not backend.accepted:
                return self._finish(goal, result, 'Nav2 goal rejected')
            pending = backend.get_result_async()
            budget = ProgressBudget(self.nav_limit, self.wait_limit)
            previous = started = time.monotonic()
            while rclpy.ok() and not pending.done():
                now, health = time.monotonic(), self._health()
                blocked = not guard_allows_motion(health)
                reason = budget.update(now - previous, blocked)
                if now - started >= self.nav_limit + self.wait_limit:
                    reason = 'total navigation deadline expired'
                previous = now
                if goal.is_cancel_requested or health.startswith('FAULT:') or reason:
                    self._cancel_backend(backend, pending)
                    return self._finish(goal, result, reason or health)
                self._event('WAITING_SAFETY' if blocked else 'NAVIGATING',
                            reason=health if blocked else '')
                time.sleep(0.02)
            if not pending.done():
                self._cancel_backend(backend, pending)
                return self._finish(goal, result, 'shutdown')
            wrapped = pending.result()
            if not guard_allows_motion(self._health()):
                return self._finish(goal, result, 'Safety not ready at Nav2 completion')
            return self._finish(goal, wrapped.result, 'Nav2 aborted',
                                wrapped.status == GoalStatus.STATUS_SUCCEEDED)
        except Exception as error:
            if backend is not None and backend.accepted and pending is not None:
                self._cancel_backend(backend, pending)
            elif submitted:
                self._fault = 'Nav2 request state unknown after exception: ' + str(error)
                self._inhibit.publish(String(data=self._fault))
                if backend is not None and backend.accepted:
                    backend.cancel_goal_async()
            return self._finish(goal, result, str(error))
        finally:
            with self._lock:
                self._reserved = False


def main(args=None):
    """Run without starting any motion until an action request arrives."""
    rclpy.init(args=args)
    node = MotionExecutor()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
