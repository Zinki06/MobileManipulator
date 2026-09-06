"""Tests for cleanup perception's ROS-independent identity policy."""

from cleanup_perception.object_registry import confirm_burst
from cleanup_perception.object_registry import Detection
from cleanup_perception.object_registry import ObjectRegistry


def detection(x, frame, track_id=-1, class_name='banana'):
    """Create a compact map-frame detection fixture."""
    return Detection(
        class_name=class_name,
        confidence=0.9,
        x=x,
        y=1.0,
        z=0.02,
        uncertainty=0.01,
        bbox=(1, 2, 3, 4),
        frame_index=frame,
        stamp_sec=float(frame),
        detector_track_id=track_id,
    )


def test_burst_requires_distinct_frame_confirmations():
    """Repeated boxes in one frame must not manufacture stability."""
    result = confirm_burst(
        [detection(0.0, 0), detection(0.01, 0), detection(0.02, 1)],
        association_gate=0.25,
        min_confirmations=3,
    )
    assert result == []


def test_track_id_change_keeps_spatial_identity():
    """YOLO track IDs are hints and never the registry primary key."""
    registry = ObjectRegistry(association_gate=0.25)
    first = registry.associate(detection(0.0, 0, track_id=4))
    second = registry.associate(detection(0.08, 1, track_id=99))
    assert first == second


def test_distant_objects_keep_different_identity():
    """Separate bananas outside the gate must receive distinct UUIDs."""
    registry = ObjectRegistry(association_gate=0.25)
    first = registry.associate(detection(0.0, 0))
    second = registry.associate(detection(0.40, 1))
    assert first != second


def test_close_simultaneous_objects_are_not_dropped_or_merged():
    """Two boxes coexisting in every frame remain two confirmed UUIDs."""
    items = [detection(x, frame) for frame in range(3) for x in (0.0, 0.15)]
    confirmed = confirm_burst(items, 0.25, 3)
    assert len(confirmed) == 2
    registry = ObjectRegistry(0.25)
    assigned = set()
    for item in confirmed:
        assigned.add(registry.associate(item.representative, assigned))
    assert len(assigned) == 2
