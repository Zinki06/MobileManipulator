"""Record stationary gripper trials without commanding the arm or wheels."""

import json
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from rcl_interfaces.msg import Log
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState


class Recorder(Node):
    """Capture feedback and execute bounded, sequential gripper actions."""

    def __init__(self, directory):
        super().__init__('manual_gripper_debug')
        self.stream = (directory / 'events.jsonl').open('w', buffering=1)
        self.lock = threading.Lock()
        self.last_joint = 0.0
        self.client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_subscription(
            JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.rosout, 1000)

    def record(self, kind, data):
        with self.lock:
            self.stream.write(json.dumps({
                'unix_time': time.time(), 'monotonic': time.monotonic(),
                'kind': kind, 'data': data,
            }, ensure_ascii=False) + '\n')

    def joints(self, msg):
        now = time.monotonic()
        if now - self.last_joint >= 0.1:
            self.last_joint = now
            self.record('joint_states', message_to_ordereddict(msg))

    def rosout(self, msg):
        self.record('rosout', message_to_ordereddict(msg))

    def wait(self, future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done():
            if not rclpy.ok() or time.monotonic() > deadline:
                raise RuntimeError('액션 응답 시간 초과 또는 ROS 종료')
            time.sleep(0.02)
        return future.result()

    def move(self, label, position):
        effort = 10.0 if position > 0 else 15.0
        print(f'[{label}] position={position}, max_effort={effort}', flush=True)
        self.record('goal', {'label': label, 'position': position,
                             'max_effort': effort})
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        handle = self.wait(self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.record(
                'action_feedback', {'label': label,
                                    **message_to_ordereddict(msg.feedback)})), 10)
        if not handle.accepted:
            raise RuntimeError(f'{label}: 액션 거부')
        try:
            response = self.wait(handle.get_result_async(), 15)
        except BaseException:
            handle.cancel_goal_async()
            raise
        result = message_to_ordereddict(response.result)
        self.record('result', {'label': label, 'status': response.status,
                               **result})
        print(f'[{label}] status={response.status}, result={dict(result)}', flush=True)
        if response.status != 4:
            raise RuntimeError(f'{label}: 액션 실패')
        if position > 0 and abs(response.result.position - position) > 0.002:
            raise RuntimeError(f'{label}: 열림 미확인. 추가 닫기 테스트 중단')


def main():
    """Run empty and banana trials, saving telemetry throughout prompts."""
    directory = Path(sys.argv[1]).resolve()
    print(f'로그 경로: {directory}')
    print('19V 사용 금지. 정품 배터리/적합한 12V 전원, 자동 미션 정지 상태에서 실행.')
    input('빈 그리퍼와 주변 여유 공간을 확인하고 손을 뺀 뒤 Enter: ')
    rclpy.init()
    node = Recorder(directory)
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    code = 0
    try:
        if not node.client.wait_for_server(timeout_sec=10):
            raise RuntimeError('그리퍼 액션 서버가 없습니다. 브링업 상태 확인 필요')
        node.record('start', {'action': '/gripper_controller/gripper_cmd'})
        time.sleep(1)
        node.move('empty_open', 0.019)
        time.sleep(2)
        node.move('empty_close', -0.010)
        time.sleep(2)
        node.move('empty_reopen', 0.019)
        input('바나나를 그리퍼 사이의 낮은 받침에 배치하고 손을 뺀 뒤 Enter: ')
        node.move('banana_close', -0.010)
        node.record('hold_start', {'seconds': 5})
        print('닫기 액션 종료. 5초 후 열기 명령 전송…', flush=True)
        time.sleep(5)
        node.move('banana_release', 0.019)
        time.sleep(2)
        node.record('complete', {})
    except (Exception, KeyboardInterrupt) as exc:
        code = 1
        node.record('error', {'message': repr(exc)})
        print(f'테스트 중단: {exc!r}. 추가 동작/토크 해제는 전송하지 않습니다.')
        if rclpy.ok():
            time.sleep(2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=3)
        node.destroy_node()
        node.stream.close()
        print(f'로그 저장 완료: {directory}')
    return code


if __name__ == '__main__':
    sys.exit(main())
