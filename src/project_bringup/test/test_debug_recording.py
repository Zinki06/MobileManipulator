# Copyright 2026 TurtleBot3 Project Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise hidden-topic recording and capture after launch shutdown begins."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String


def test_hidden_topic_survives_shutdown_signal(tmp_path, monkeypatch):
    """Close a playable bag containing a message sent after supervisor SIGINT."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(170 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init()
    node = rclpy.create_node('recording_test_publisher')
    pub = node.create_publisher(String, '/debug_test/_action/feedback', 10)
    script = Path(__file__).parents[1] / 'scripts/record_debug.py'
    process = subprocess.Popen([
        sys.executable, str(script), '--root', str(tmp_path),
        '--launch-logs', str(tmp_path / 'ros'), '--profile', str(tmp_path / 'unused'),
        '--shutdown-tail', '2',
    ])
    try:
        deadline = time.monotonic() + 15
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert process.poll() is None
            rclpy.spin_once(node, timeout_sec=0.1)
        assert pub.get_subscription_count() > 0
        process.send_signal(signal.SIGINT)
        time.sleep(0.3)
        pub.publish(String(data='shutdown_tail_received'))
        assert process.wait(timeout=20) == 0
        bag = next(tmp_path.glob('full_*/bag'))
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        values = []
        while reader.has_next():
            topic, payload, _stamp = reader.read_next()
            if topic == '/debug_test/_action/feedback':
                values.append(deserialize_message(payload, String).data)
        assert 'shutdown_tail_received' in values
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=25)
        node.destroy_node()
        rclpy.shutdown()
