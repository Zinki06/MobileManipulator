"""Synchronize depth obstacle layers with the verified carrying state."""

import time

from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class DepthCostmaps:
    """Gate each motion on acknowledged layer changes and refreshed costmaps."""

    def __init__(self, node, group):
        """Connect only to depth, carrying state, and Nav2 costmap services."""
        self.node = node
        self.mode = None
        self.carrying = False
        self.carry_received = 0.0
        self.depth_stamp = 0.0
        self.maps = {}
        self.clients = []
        node.create_subscription(Bool, '/cleanup/carrying', self._carry, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PointCloud2, '/cleanup/obstacle_points', self._depth,
                                 qos_profile_sensor_data)
        for name in ('local', 'global'):
            root = f'/{name}_costmap'
            self.clients.append((
                node.create_client(SetParameters, f'{root}/{name}_costmap/set_parameters',
                                   callback_group=group),
                node.create_client(ClearEntireCostmap, f'{root}/clear_entirely_{name}_costmap',
                                   callback_group=group)))
            node.create_subscription(OccupancyGrid, f'{root}/costmap',
                                     lambda msg, key=name: self._map(key, msg), 1)

    def _carry(self, msg):
        self.carrying = msg.data
        self.carry_received = time.monotonic()

    def _depth(self, msg):
        if msg.width * msg.height >= 20:
            self.depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _map(self, name, msg):
        if msg.info.width and msg.info.height and any(v >= 0 for v in msg.data):
            self.maps[name] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def issue(self):
        """Reject a changed or expired carrying lease during an admitted motion."""
        if self.carrying and time.monotonic() - self.carry_received >= 0.75:
            return 'carrying lease expired'
        if self.mode is not None and self.carrying != self.mode:
            return 'carrying state changed during motion'
        return ''

    def prepare(self, interrupted):
        """Apply a bounded transition; never infer success from a service timeout."""
        target = self.carrying
        if self.mode == target and not self.issue():
            return ''
        deadline = time.monotonic() + 10.0
        self.mode = None

        def wait(predicate):
            while time.monotonic() < deadline and not interrupted():
                if self.carrying != target:
                    raise RuntimeError('carrying changed during costmap synchronization')
                if target and time.monotonic() - self.carry_received >= 0.75:
                    raise RuntimeError('carrying lease expired')
                if predicate():
                    return
                time.sleep(0.02)
            raise RuntimeError('costmap synchronization canceled or timed out')

        def call(client, request):
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()

        def enable(client, value):
            result = call(client, SetParameters.Request(parameters=[Parameter(
                name='depth_layer.enabled', value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=value))]))
            if len(result.results) != 1 or not result.results[0].successful:
                raise RuntimeError('depth layer parameter rejected')

        try:
            # Disable before clearing, so a carried object cannot be reinserted.
            for parameters, clear in self.clients:
                enable(parameters, False)
                call(clear, ClearEntireCostmap.Request())
            if not target:
                for parameters, _ in self.clients:
                    enable(parameters, True)
            barrier = self.node.get_clock().now().nanoseconds * 1e-9
            # Wait past a full global update period; static layers must repopulate.
            wait(lambda: all(self.maps.get(key, 0.0) > barrier + 1.0
                             for key in ('local', 'global')) and
                 (target or (self.depth_stamp > barrier and
                             0 <= self.node.get_clock().now().nanoseconds * 1e-9 -
                             self.depth_stamp < 0.5)))
            self.mode = target
            return ''
        except Exception as error:
            return str(error)
