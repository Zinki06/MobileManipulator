#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Bool
import math
import yaml
import shutil
import os
import sys
import threading
import time

class WaypointCalibrator(Node):
    def __init__(self):
        super().__init__('waypoint_calibrator')

        self.yaml_path = '/home/user/turtlebot3_ws/src/aruco_localizer/map/new_map_markers.yaml'
        self.install_yaml_path = '/home/user/turtlebot3_ws/install/aruco_localizer/share/aruco_localizer/map/new_map_markers.yaml'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_imu_yaw = 0.0
        self.last_marker_id = -1
        self.localization_fresh = False

        self.create_subscription(Imu, '/imu_broadcaster/imu', self.imu_cb, 10)
        self.create_subscription(Int32, '/aruco/last_marker_id', self.marker_id_cb, 10)
        self.create_subscription(Bool, '/aruco/localization_fresh', self.fresh_cb, 10)

        # 현재 설정된 마커 정보 로드
        self.markers = self.load_current_markers()
        self.recorded_markers = {}

    def load_current_markers(self):
        markers = {}
        if os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                    for m in data.get('markers', []):
                        markers[m['id']] = {
                            'x': float(m['x']),
                            'y': float(m['y']),
                            'yaw': float(m.get('yaw', 0.0)),
                            'size': float(m.get('size', 0.133))
                        }
            except Exception as e:
                self.get_logger().error(f"Error loading YAML: {e}")
        return markers

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_imu_yaw = math.atan2(siny_cosp, cosy_cosp)

    def marker_id_cb(self, msg: Int32):
        self.last_marker_id = msg.data

    def fresh_cb(self, msg: Bool):
        self.localization_fresh = msg.data

    def get_robot_pose(self):
        # 1. Try map -> base_link
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw, 'map'
        except Exception:
            pass

        # 2. Fallback to odom -> base_link
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw, 'odom'
        except Exception:
            return None, None, None, 'none'

    def record_marker(self, marker_id):
        x, y, yaw, frame = self.get_robot_pose()
        if x is None:
            print(f"\n[오류] TF를 읽을 수 없습니다. 터틀봇 노드가 실행 중인지 확인하세요.")
            return

        size = self.markers.get(marker_id, {}).get('size', 0.133)
        self.recorded_markers[marker_id] = {
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(math.degrees(yaw), 1),
            'size': size
        }
        print(f"\n✅ [기록 완료] 마커 ID {marker_id}: x={x:.4f}m, y={y:.4f}m, yaw={math.degrees(yaw):.1f}° (기준 frame: {frame}, IMU: {math.degrees(self.current_imu_yaw):.1f}°)")

    def save_yaml(self):
        if not self.recorded_markers:
            print("\n[경고] 기록된 마커가 없습니다. 먼저 마커 번호를 입력해 기록하세요.")
            return

        # 백업 생성
        if os.path.exists(self.yaml_path):
            backup_path = self.yaml_path + f".bak_{int(time.time())}"
            shutil.copyfile(self.yaml_path, backup_path)
            print(f"📦 기존 YAML 백업 완료: {backup_path}")

        # 기존 마커에 기록된 마커 덮어쓰기
        final_markers = dict(self.markers)
        for m_id, data in self.recorded_markers.items():
            final_markers[m_id] = data

        yaml_data = {
            'map_size_meters': {'width': 8.5, 'height': 10.95},
            'origin': 'center',
            'markers': []
        }

        # ID 순서대로 정렬하여 저장 (5 -> 4 -> 3 -> 2 -> 1 -> 0)
        order = [5, 4, 3, 2, 1, 0]
        for m_id in order:
            if m_id in final_markers:
                m = final_markers[m_id]
                yaml_data['markers'].append({
                    'id': m_id,
                    'x': float(m['x']),
                    'y': float(m['y']),
                    'yaw': float(m.get('yaw', 0.0)),
                    'size': float(m.get('size', 0.133))
                })

        with open(self.yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, sort_keys=False)

        if os.path.exists(os.path.dirname(self.install_yaml_path)):
            with open(self.install_yaml_path, 'w') as f:
                yaml.dump(yaml_data, f, sort_keys=False)

        print(f"\n🎉 [저장 성공] {self.yaml_path} 파일이 새로운 캘리브레이션 좌표로 업데이트되었습니다!")
        self.print_summary()

    def print_summary(self):
        print("\n" + "="*65)
        print("                 현재 마커 맵 좌표 현황")
        print("="*65)
        print(f"{'ID':<5} | {'X (m)':<10} | {'Y (m)':<10} | {'Yaw (deg)':<10} | {'상태':<10}")
        print("-"*65)
        order = [5, 4, 3, 2, 1, 0]
        for m_id in order:
            if m_id in self.recorded_markers:
                m = self.recorded_markers[m_id]
                status = "✨ 새로 기록됨"
            elif m_id in self.markers:
                m = self.markers[m_id]
                status = "기존 값"
            else:
                continue
            print(f"{m_id:<5} | {m['x']:<10.4f} | {m['y']:<10.4f} | {m['yaw']:<10.1f} | {status}")
        print("="*65 + "\n")


def user_input_loop(node: WaypointCalibrator):
    time.sleep(1.0)
    while rclpy.ok():
        x, y, yaw, frame = node.get_robot_pose()
        pos_str = f"x={x:.3f}m, y={y:.3f}m, yaw={math.degrees(yaw):.1f}° [{frame}]" if x is not None else "TF 대기 중..."
        
        print("\n" + "="*65)
        print("   🤖 터틀봇3 마커 위치 캘리브레이션 툴 (Live Calibrator)")
        print("="*65)
        print(f" 현재 실시간 로봇 위치: {pos_str}")
        print(f" IMU Yaw: {math.degrees(node.current_imu_yaw):.1f}° | 카메라 인식 마커 ID: {node.last_marker_id}")
        print("-"*65)
        print(" [사용법]")
        print("  1) 별도 터미널에서 teleop으로 로봇을 마커 정중앙/회전 포인트로 이동")
        print("  2) 해당 마커 번호 (0, 1, 2, 3, 4, 5) 입력 후 Enter ➔ 현재 위치 기록")
        print("  3) 'p' + Enter ➔ 전체 마커 좌표 확인")
        print("  4) 's' + Enter ➔ new_map_markers.yaml 파일에 영구 저장")
        print("  5) 'q' + Enter ➔ 캘리브레이션 종료")
        print("="*65)

        cmd = input("👉 마커 번호 또는 명령 입력 (0~5, p, s, q): ").strip().lower()

        if cmd in ['0', '1', '2', '3', '4', '5']:
            node.record_marker(int(cmd))
        elif cmd == 'p':
            node.print_summary()
        elif cmd == 's':
            node.save_yaml()
        elif cmd == 'q':
            print("캘리브레이션을 종료합니다.")
            rclpy.shutdown()
            break
        else:
            print("[알림] 0~5 번호나 p, s, q 중 하나를 입력하세요.")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCalibrator()

    # 백그라운드에서 ROS 2 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        user_input_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
