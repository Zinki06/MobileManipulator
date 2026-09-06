"""Verify the local camera boundary independently of global navigation TF."""

from geometry_msgs.msg import PointStamped
import numpy as np
import pytest

from cleanup_perception.camera_calibration import CameraCalibration


def test_translation_rotation_and_timestamp():
    """Apply a known rigid transform without modifying the input header."""
    rotation = [0., -1., 0., 1., 0., 0., 0., 0., 1.]
    calibration = CameraCalibration([0.1, 0.02, 0.03], rotation, 'optical')
    point = PointStamped()
    point.header.frame_id = 'optical'
    point.header.stamp.sec = 123
    point.point.x, point.point.y, point.point.z = 0.2, 0.3, 0.4
    result = calibration.to_wrist(point)
    assert np.allclose([result.point.x, result.point.y, result.point.z], [-0.2, 0.22, 0.43])
    assert result.header.frame_id == 'link5'
    assert result.header.stamp.sec == 123
    assert point.header.frame_id == 'optical'
    point.header.frame_id = 'wrong_camera'
    with pytest.raises(ValueError):
        calibration.to_wrist(point)


def test_invalid_calibration_rejected():
    """Fail closed on malformed or reflected extrinsics."""
    for translation, rotation in [
            ([float('nan'), 0, 0], np.eye(3)),
            ([1, 0, 0], np.eye(3)),
            ([0, 0, 0], np.diag([-1, 1, 1])),
            ([0, 0, 0], np.zeros((3, 3)))]:
        with pytest.raises(ValueError):
            CameraCalibration(translation, rotation, 'optical')


def test_saved_calibration_generalizes_to_independent_wrist_sweep():
    """The second sweep calibration predicts floor heights in the first sweep."""
    from pathlib import Path
    import yaml

    config = yaml.safe_load((Path(__file__).parents[1] / 'config' /
                             'grasp_camera_20260906.yaml').read_text())
    parameters = config['scan_perception_node']['ros__parameters']
    calibration = CameraCalibration(parameters['grasp_camera_translation'],
                                    parameters['grasp_camera_rotation'],
                                    parameters['grasp_camera_frame'])
    # Frozen measurements from the independent 16:32 sweep, not fitting inputs.
    measurements = [
        (0.4071240421244192, 0.42278956453045763,
         [-0.1719431447160276, 0.0, 0.9851068749050345]),
        (0.4070711282120869, 0.38363071143432664,
         [-0.5130655480935493, 0.0, 0.8583494296377588]),
        (0.40686296019395396, 0.3629795345015544,
         [-0.6767479873270826, 0.0, 0.7362147523982]),
    ]
    for wrist_height, floor_distance, wrist_normal in measurements:
        predicted = wrist_height + np.array(wrist_normal) @ calibration.translation
        assert abs(predicted - floor_distance) < 0.001
