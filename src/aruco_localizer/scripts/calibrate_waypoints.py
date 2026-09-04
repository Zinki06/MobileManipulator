#!/usr/bin/env python3
"""Interactively record virtual route waypoints in the map frame."""

import math
from pathlib import Path
import shutil
import threading
import time

import rclpy
from rclpy.node import Node
import tf2_ros
import yaml


DEFAULT_ROUTE_PATH = (
    Path.home() / 'turtlebot3_ws' / 'src' / 'aruco_localizer' /
    'config' / 'routes.yaml'
)


class WaypointCalibrator(Node):
    """Record robot base poses without modifying physical marker landmarks."""

    def __init__(self):
        """Initialize the TF listener and load the route configuration."""
        super().__init__('waypoint_calibrator')
        self.declare_parameter('route_yaml_path', str(DEFAULT_ROUTE_PATH))
        self.route_path = Path(
            self.get_parameter('route_yaml_path').get_parameter_value().string_value
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.config = self._load_config()
        self.recorded = {}

    def _load_config(self):
        """Load the existing waypoint and route definitions."""
        try:
            with self.route_path.open('r', encoding='utf-8') as stream:
                config = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as exception:
            raise RuntimeError(
                f'Unable to load route config {self.route_path}: {exception}'
            ) from exception

        waypoints = config.get('waypoints')
        routes = config.get('routes')
        if not isinstance(waypoints, dict) or not isinstance(routes, dict):
            raise RuntimeError(
                'Route config must contain waypoints and routes mappings.'
            )
        return config

    def robot_pose(self):
        """Return the latest map-frame base pose, or None when unavailable."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except Exception as exception:  # tf2 exception types vary by distro.
            self.get_logger().debug(f'Map pose unavailable: {exception}')
            return None

        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            math.atan2(sin_yaw, cos_yaw),
        )

    def record(self, waypoint_name):
        """Record the current robot pose for an existing waypoint name."""
        waypoints = self.config['waypoints']
        if waypoint_name not in waypoints:
            print(f"[오류] 알 수 없는 웨이포인트: '{waypoint_name}'")
            return

        pose = self.robot_pose()
        if pose is None:
            print('[오류] map → base_link TF를 읽을 수 없습니다.')
            return

        x, y, yaw = pose
        self.recorded[waypoint_name] = {
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(yaw, 6),
        }
        print(
            f"[기록] {waypoint_name}: x={x:.4f}, y={y:.4f}, "
            f'yaw={math.degrees(yaw):.1f}deg'
        )

    def save(self):
        """Write recorded poses to the source route file with a backup."""
        if not self.recorded:
            print('[안내] 새로 기록한 웨이포인트가 없습니다.')
            return

        backup_path = self.route_path.with_name(
            f'{self.route_path.name}.bak_{int(time.time())}'
        )
        shutil.copyfile(self.route_path, backup_path)

        for name, pose in self.recorded.items():
            waypoint = self.config['waypoints'][name]
            waypoint.update(pose)

        with self.route_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                self.config,
                stream,
                allow_unicode=True,
                sort_keys=False,
            )

        print(f'[저장] {self.route_path}')
        print(f'[백업] {backup_path}')
        print('[안내] aruco_localizer를 다시 빌드한 뒤 재실행하세요.')
        self.recorded.clear()

    def print_summary(self):
        """Print configured poses and mark unsaved recordings."""
        print('\n가상 웨이포인트')
        print('-' * 78)
        print(f"{'name':32} {'x':>9} {'y':>9} {'yaw(deg)':>11} {'role':>10}")
        for name, stored in self.config['waypoints'].items():
            pose = self.recorded.get(name, stored)
            marker = '*' if name in self.recorded else ' '
            yaw_degrees = math.degrees(float(pose['yaw']))
            print(
                f"{marker}{name:31} {float(pose['x']):9.4f} "
                f"{float(pose['y']):9.4f} {yaw_degrees:11.1f} "
                f"{stored.get('role', 'transit'):>10}"
            )
        print('-' * 78)
        print('* = 아직 저장하지 않은 기록')


def input_loop(node):
    """Run the interactive command loop while ROS spins separately."""
    time.sleep(1.0)
    while rclpy.ok():
        pose = node.robot_pose()
        if pose is None:
            pose_text = 'map TF 대기 중'
        else:
            x, y, yaw = pose
            pose_text = (
                f'x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}deg'
            )

        print(f'\n현재 base_link: {pose_text}')
        print("명령: 'p'=목록, 'r <name>'=현재 위치 기록, 's'=저장, 'q'=종료")
        command = input('> ').strip()
        if command == 'p':
            node.print_summary()
        elif command.startswith('r '):
            node.record(command[2:].strip())
        elif command == 's':
            node.save()
        elif command == 'q':
            rclpy.shutdown()
            break
        else:
            print('[안내] p, r <name>, s, q 중 하나를 입력하세요.')


def main(args=None):
    """Start the waypoint calibration utility."""
    rclpy.init(args=args)
    try:
        node = WaypointCalibrator()
    except RuntimeError as exception:
        print(f'[오류] {exception}')
        rclpy.shutdown()
        return

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()
    try:
        input_loop(node)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
