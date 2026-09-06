"""Record event-triggered grasp video in a passive process, never in motion callbacks."""

from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ClipPolicy:
    """Capture approach and grasp, with bounded duration and a short outcome tail."""

    START = {'PLAN_ACCEPTED', 'OBJECT_REAPPROACH_START', 'TARGET_VIEW_START', 'PICK_START'}
    END = {'PICK_VERIFIED', 'OBJECT_FAILED', 'OBJECT_COLLECTED', 'MISSION_FAILED',
           'MISSION_COMPLETE', 'MISSION_STOPPED'}

    def __init__(self):
        self.until = 0.0
        self.event = ''
        self.uuid = ''

    def update(self, message, now):
        """Ignore empty scans; retain an object identity and the most recent phase."""
        tag = message.split('|', 1)[0]
        self.event = message
        match = re.search(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', message)
        if match:
            self.uuid = match.group(0)
        if tag in self.START:
            self.until = now + 90.0
        elif tag in self.END and self.until > now:
            self.until = now + 3.0


class ClipWriter:
    """Own one MJPEG AVI and its per-frame timestamp sidecar in the writer thread."""

    def __init__(self, directory, name, fps=10.0, size=(960, 540)):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (name + '.avi')
        self.size = size
        self.video = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'MJPG'), fps, size)
        if not self.video.isOpened():
            self.video.release()
            raise RuntimeError('MJPEG encoder unavailable')
        try:
            self.metadata = (directory / (name + '.frames.jsonl')).open('x', buffering=1)
        except Exception:
            self.video.release()
            raise
        self.count = 0

    def write(self, frame, stamp, event, uuid):
        """Persist source time alongside each encoded frame, including dropped-frame gaps."""
        resized = cv2.resize(frame, self.size)
        self.video.write(resized)
        self.metadata.write(json.dumps({'frame': self.count, 'image_stamp': stamp,
                                        'event': event, 'uuid': uuid}) + '\n')
        self.count += 1

    def close(self):
        """Finalize the AVI index and metadata."""
        self.video.release()
        self.metadata.close()


class GraspVideoRecorder(Node):
    """Isolate image conversion, codec and disk I/O from control and perception."""

    def __init__(self):
        super().__init__('grasp_video_recorder')
        self.enabled = self.declare_parameter('enabled', True).value
        self.root = Path(self.declare_parameter(
            'output_directory', '/home/user/turtlebot3_ws/motion_debug').value)
        self.directory = self.root / datetime.now().strftime('video_%Y%m%d_%H%M%S_%f')
        self.policy = ClipPolicy()
        self._lock = threading.Lock()
        self._image = None
        self._stop = threading.Event()
        self._bridge = CvBridge()
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                                 self._on_image, qos_profile_sensor_data)
        for topic in ('/cleanup/events', '/pick/events'):
            self.create_subscription(String, topic, self._on_event, 30)
        self._worker = threading.Thread(target=self._record, daemon=True)
        self._worker.start()

    def _on_image(self, msg):
        # One latest-frame slot: disk lag must never build an unbounded image queue.
        if self.enabled:
            with self._lock:
                self._image = msg

    def _on_event(self, msg):
        with self._lock:
            self.policy.update(msg.data, time.monotonic())

    def _record(self):
        writer, started, total_bytes, previous, sequence = None, 0.0, 0, None, 0
        try:
            while not self._stop.wait(0.1):
                with self._lock:
                    active = self.enabled and time.monotonic() < self.policy.until
                    frame, event, uuid = self._image, self.policy.event, self.policy.uuid
                if writer and (not active or time.monotonic() - started >= 90.0):
                    writer.close()
                    total_bytes += writer.path.stat().st_size
                    writer = None
                if not active or frame is None:
                    continue
                stamp = frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9
                if stamp == previous:
                    continue
                if writer is None:
                    # Never delete old recordings. Disable new recording before filling disk.
                    self.root.mkdir(parents=True, exist_ok=True)
                    if shutil.disk_usage(self.root).free < 1024**3 or total_bytes >= 512 * 1024**2:
                        raise RuntimeError('Video disk budget reached (1GiB reserve / 512MiB run)')
                    sequence += 1
                    writer = ClipWriter(self.directory, f'clip_{sequence:03d}_{uuid or "target"}')
                    started = time.monotonic()
                    self.get_logger().info(f'[GRASP_VIDEO] Recording {writer.path}')
                if writer.count % 10 == 0 and (
                        shutil.disk_usage(self.root).free < 1024**3 or
                        total_bytes + writer.path.stat().st_size >= 512 * 1024**2):
                    raise RuntimeError('Video storage limit reached')
                writer.write(self._bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8'),
                             stamp, event, uuid)
                previous = stamp
        except Exception as error:
            self.enabled = False
            self.get_logger().error(
                f'[GRASP_VIDEO] Recording disabled; motion unaffected: {error}')
        finally:
            if writer:
                writer.close()

    def close(self):
        """Stop the passive worker without sending any robot commands."""
        self._stop.set()
        self._worker.join(timeout=5.0)


def main(args=None):
    """Run the event-triggered recorder."""
    rclpy.init(args=args)
    node = GraspVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
