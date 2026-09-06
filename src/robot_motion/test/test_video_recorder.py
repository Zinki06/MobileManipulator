"""Test event gating and actual codec output without opening robot devices."""

import json

import cv2
import numpy as np

from robot_motion.video_recorder import ClipPolicy, ClipWriter


def test_no_video_for_empty_scan_and_bounded_tail():
    """Start only for object actions, not every ninety-degree image preview."""
    policy = ClipPolicy()
    policy.update('SCAN_CAPTURE_START|station=0', 10.0)
    assert policy.until == 0.0
    policy.update('PLAN_ACCEPTED|UUID=12345678-1234-1234-1234-123456789abc', 12.0)
    assert policy.until == 102.0
    policy.update('INSERT|depth=0.012', 20.0)
    assert policy.until == 102.0
    policy.update('OBJECT_FAILED|not held', 25.0)
    assert policy.until == 28.0


def test_avi_is_readable_and_timestamped(tmp_path):
    """Decode the saved clip to detect unavailable codecs or corrupt output."""
    writer = ClipWriter(tmp_path, 'sample', size=(160, 120))
    for index in range(5):
        writer.write(np.full((240, 320, 3), 30 * index, dtype=np.uint8),
                     100.0 + index * 0.1, 'INSERT', 'test')
    writer.close()
    reader = cv2.VideoCapture(str(writer.path))
    count = 0
    while True:
        ok, frame = reader.read()
        if not ok:
            break
        assert frame.shape[:2] == (120, 160)
        count += 1
    reader.release()
    assert count == 5
    metadata = [json.loads(line) for line in
                (tmp_path / 'sample.frames.jsonl').read_text().splitlines()]
    assert metadata[-1]['image_stamp'] == 100.4
    assert metadata[-1]['event'] == 'INSERT'
