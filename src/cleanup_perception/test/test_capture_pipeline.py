"""Regression checks for model loading and saved capture evidence without ROS I/O."""

from pathlib import Path
import sys
import threading
from types import MethodType
from types import SimpleNamespace
from unittest.mock import Mock

from cleanup_interfaces.srv import CaptureObjects
from cv_bridge import CvBridge
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from cleanup_perception.perception_node import ScanPerceptionNode
from cleanup_perception.object_registry import Detection
from cleanup_perception.object_registry import ConfirmedDetection
from cleanup_perception.grasp_anchor import GraspAnchors


def test_candidate_publication_retains_floor_alignment_and_single_forward_offset():
    """The evaluated height reaches the manager; executor alone adds forward correction."""
    from builtin_interfaces.msg import Time

    selected = {'observed_link1': [0.4, 0.01, -0.069],
                'target_link1': [0.43, 0.01, -0.069], 'depth_uncertainty_m': 0.0015}

    def transform(point, frame, **kwargs):
        assert point.header.frame_id == 'link1'
        assert point.header.stamp.sec == 42
        assert frame == 'map'
        assert point.point.x == 0.4  # Never publish the +30mm execution target.
        assert point.point.z == -0.069
        point.point.z += 0.101
        return point

    node = SimpleNamespace(_candidate_reports={0: {'selected': selected}}, _map_frame='map',
                           _tf_buffer=SimpleNamespace(transform=transform))
    point, uncertainty = ScanPerceptionNode._candidate_point_in_map(
        node, 0, SimpleNamespace(stamp=Time(sec=42)))
    assert abs(point.point.z - 0.032) < 1e-9
    assert uncertainty == 0.0015


def perception(tmp_path):
    """Bind production callbacks to local frames without opening DDS or hardware."""
    node = SimpleNamespace(
        _model=None, _model_error='', _model_path='fixture.pt',
        _empty_scene_frames=2,
        _capture_count=0, _capture_timeout=4.0, _rgb_sequence=0,
        _depth_sequence=0, _association_gate=0.25, _map_frame='map',
        _active_mission_id='', _capture_mutex=threading.Lock(),
        _grasp_anchors=GraspAnchors(Mock(transform=Mock(side_effect=RuntimeError('No TF')))),
        _frame_condition=threading.Condition(), _latest_rgb=None,
        _bridge=CvBridge(), _output_root=tmp_path,
        _use_sam_refinement=False, _device='', _frame_yolo_count=0,
        get_logger=lambda: Mock(),
    )
    for name in (
        '_ensure_model', '_capture_callback', '_perform_capture',
        '_save_evidence', '_detect_frame', '_refine_confirmed_with_sam',
        '_ensure_sam_model',
    ):
        setattr(node, name, MethodType(getattr(ScanPerceptionNode, name), node))
    node._min_confidence = 0.45
    return node


def request():
    """Create an ordinary station capture request."""
    return CaptureObjects.Request(
        mission_id='test_run', station_name='station_0', heading_index=0,
        burst_frames=3, min_confirmations=3,
    )


def test_first_request_loads_yolo_and_skips_empty_scene(tmp_path, monkeypatch):
    """Two negative frames exit early without image or metadata file writes."""
    node = perception(tmp_path)
    model = Mock()
    model.predict.return_value = []
    loader = Mock(return_value=model)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=loader))
    rgb = node._bridge.cv2_to_imgmsg(np.zeros((64, 96, 3), np.uint8), 'bgr8')
    depth = node._bridge.cv2_to_imgmsg(np.ones((64, 96), np.uint16), '16UC1')
    node._next_synchronized_frame = Mock(
        side_effect=[(i, i, rgb, depth, CameraInfo()) for i in range(1, 4)]
    )
    response = node._capture_callback(request(), CaptureObjects.Response())
    assert response.success, response.message
    assert not response.observations
    loader.assert_called_once_with('fixture.pt')
    assert model.predict.call_count == 3  # warmup plus two negative frames
    assert not response.image_reference
    assert not list(tmp_path.rglob('*'))
    assert node._next_synchronized_frame.call_count == 2
    assert node._ensure_model() is True
    loader.assert_called_once()


def test_model_failure_is_explicit_without_photo(tmp_path, monkeypatch):
    """A bad model never produces a successful empty scan or an unsolicited photo."""
    node = perception(tmp_path)
    node._latest_rgb = node._bridge.cv2_to_imgmsg(
        np.zeros((64, 96, 3), np.uint8), 'bgr8'
    )
    loader = Mock(side_effect=RuntimeError('invalid weights'))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=loader))
    response = node._capture_callback(request(), CaptureObjects.Response())
    assert not response.success
    assert 'YOLO unavailable: RuntimeError: invalid weights' == response.message
    assert not response.image_reference
    assert node._capture_mutex.acquire(blocking=False)


def test_confirmed_banana_has_uuid_image_and_serializable_metadata(tmp_path, monkeypatch):
    """Positive detection traverses ROS message creation and image/JSON saving."""
    node = perception(tmp_path)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock()))
    image = np.zeros((64, 96, 3), np.uint8)
    rgb = node._bridge.cv2_to_imgmsg(image, 'bgr8')
    depth = node._bridge.cv2_to_imgmsg(np.ones((64, 96), np.uint16), '16UC1')
    node._next_synchronized_frame = Mock(
        side_effect=[(i, i, rgb, depth, CameraInfo()) for i in range(1, 4)]
    )
    node._detect_frame = Mock(side_effect=[
        ([Detection('banana', 0.9, 1.0, 0.2, 0.03, 0.01,
                    (10, 20, 30, 40), i, float(i))], image.copy())
        for i in range(3)
    ])
    response = node._capture_callback(request(), CaptureObjects.Response())
    assert response.success, response.message
    assert len(response.observations) == 1
    obj = response.observations[0]
    assert obj.object_uuid and obj.confirmation_count == 3
    assert Path(obj.image_reference).is_file()
    assert Path(response.image_reference).with_suffix('.json').is_file()


def test_image_save_failure_is_not_reported_as_a_path(tmp_path, monkeypatch):
    """Disk write failure cannot silently become a valid VLM image reference."""
    node = perception(tmp_path)
    monkeypatch.setattr('cv2.imwrite', lambda *_: False)
    with pytest.raises(OSError, match='Failed to save'):
        node._save_evidence('run', 'station', 0, np.zeros((4, 4, 3), np.uint8))


def test_sam_grasp_point_is_the_saved_object_frame(tmp_path):
    """The UUID evidence must include the actual body target, not an unrelated YOLO frame."""
    node = perception(tmp_path)
    node._ensure_sam_model = lambda: True
    node._sam_min_mask_pixels, node._min_depth, node._max_depth = 20, 0.1, 2.5
    image = np.zeros((100, 160, 3), np.uint8)
    depth = np.full((100, 160), 600, np.uint16)
    mask = np.zeros(depth.shape, dtype=bool)
    mask[30:70, 20:140] = True
    node._run_sam_masks = lambda *_: [mask]
    point = SimpleNamespace(point=SimpleNamespace(x=1.0, y=2.0, z=0.05))
    node._sample_point_in_map = lambda sample, *_: (point, sample.uncertainty)
    frames = [(image, depth, CameraInfo(), SimpleNamespace(), image.copy())]
    item = ConfirmedDetection(Detection('banana', 0.9, 1, 2, 0.05, 0.01,
                                        (20, 30, 140, 70), 0, 1.0), 3)
    refined, annotated = node._refine_confirmed_with_sam([item], frames)
    target = refined[0].representative
    assert target.grasp_valid
    assert abs(target.grasp_u - 80) < 2 and abs(target.grasp_v - 50) < 2
    assert frames[0][4] is annotated
    assert np.array_equal(annotated[int(target.grasp_v), int(target.grasp_u)], [0, 255, 0])


def test_calibrated_clipped_object_never_uses_nominal_anchor(tmp_path):
    """Reject cropped detections when only an uncalibrated inverse projection exists."""
    node = perception(tmp_path)
    node._camera_calibration = SimpleNamespace(translation=np.zeros(3), rotation=np.eye(3))
    node._capture_heading = 250
    node._grasp_anchors = Mock()
    node._ensure_sam_model = lambda: True
    node._sam_min_mask_pixels, node._min_depth, node._max_depth = 20, 0.1, 2.5
    image = np.zeros((100, 160, 3), np.uint8)
    depth = np.full((100, 160), 600, np.uint16)
    mask = np.ones(depth.shape, dtype=bool)
    node._run_sam_masks = lambda *_: [mask]
    node._sample_point_in_map = lambda *_: (None, 0.0)
    frames = [(image, depth, CameraInfo(), SimpleNamespace(), image.copy())]
    item = ConfirmedDetection(Detection('banana', 0.9, 1, 2, 0.05, 0.01,
                                        (20, 30, 140, 100), 0, 1.0), 3)
    refined, _ = node._refine_confirmed_with_sam([item], frames)
    assert not refined[0].representative.grasp_valid
    node._grasp_anchors.sample.assert_not_called()
    node._grasp_anchors.remember.assert_not_called()


def capture_frames(node, count):
    """Provide distinct frames and a bounded end without opening a camera."""
    image = np.zeros((64, 96, 3), np.uint8)
    rgb = node._bridge.cv2_to_imgmsg(image, 'bgr8')
    depth = node._bridge.cv2_to_imgmsg(np.ones((64, 96), np.uint16), '16UC1')
    node._next_synchronized_frame = Mock(
        side_effect=[(i, i, rgb, depth, CameraInfo()) for i in range(1, count + 1)] + [None])
    return image


def test_target_on_second_frame_keeps_positive_confirmation(tmp_path, monkeypatch):
    """One empty frame must not skip a target or weaken its three confirmations."""
    node = perception(tmp_path)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock()))
    image = capture_frames(node, 5)
    node._detect_frame = Mock(side_effect=[([], image)] + [
        ([Detection('banana', .9, 1., .2, .03, .01,
                    (10, 20, 30, 40), i, float(i))], image.copy())
        for i in range(1, 5)])
    capture = request()
    capture.burst_frames = 5
    response = node._capture_callback(capture, CaptureObjects.Response())
    assert response.success, response.message
    assert node._next_synchronized_frame.call_count == 4
    assert len(response.observations) == 1
    assert response.observations[0].confirmation_count == 3
    assert Path(response.observations[0].image_reference).is_file()


def test_yolo_without_depth_is_not_a_negative_heading(tmp_path, monkeypatch):
    """A visible target retains the full burst even when its depth is rejected."""
    node = perception(tmp_path)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock()))
    image = capture_frames(node, 5)
    counts = iter([1, 0, 0, 0, 0])

    def detect(*_):
        node._frame_yolo_count = next(counts)
        return [], image.copy()

    node._detect_frame = detect
    capture = request()
    capture.burst_frames = 5
    response = node._capture_callback(capture, CaptureObjects.Response())
    assert response.success, response.message
    assert not response.observations
    assert node._next_synchronized_frame.call_count == 5
    assert 'empty heading skipped' not in response.message
    assert Path(response.image_reference).is_file()


@pytest.mark.parametrize('failed_inference', [False, True])
def test_incomplete_or_failed_frames_do_not_report_empty(tmp_path, monkeypatch, failed_inference):
    """Missing images and YOLO errors are failures, not evidence of an empty view."""
    node = perception(tmp_path)
    model = Mock()
    model.predict.return_value = []
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock(return_value=model)))
    capture_frames(node, 1)
    if failed_inference:
        node._detect_frame = Mock(side_effect=RuntimeError('inference failed'))
    response = node._capture_callback(request(), CaptureObjects.Response())
    assert not response.success
    assert not response.image_reference
    assert not list(tmp_path.rglob('*'))


@pytest.mark.parametrize('limit', [2, 5])
def test_negative_frame_limit_is_configurable_without_saving(tmp_path, monkeypatch, limit):
    """Extra negative confirmation can be restored independently of image saving."""
    node = perception(tmp_path)
    node._empty_scene_frames = limit
    model = Mock()
    model.predict.return_value = []
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock(return_value=model)))
    capture_frames(node, 5)
    capture = request()
    capture.burst_frames = 5
    response = node._capture_callback(capture, CaptureObjects.Response())
    assert response.success, response.message
    assert node._next_synchronized_frame.call_count == limit
    assert not response.image_reference
    assert not list(tmp_path.rglob('*'))
