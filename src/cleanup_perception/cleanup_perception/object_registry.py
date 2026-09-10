"""Spatial association and burst confirmation for cleanup observations."""

from dataclasses import dataclass
from dataclasses import replace
import math
from statistics import median
import uuid


@dataclass
class Detection:
    """Detector result transformed into the map frame."""

    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    uncertainty: float
    bbox: tuple
    frame_index: int
    stamp_sec: float
    detector_track_id: int = -1
    grasp_valid: bool = False
    grasp_reason: str = 'Full SAM mask and central depth not verified'
    grasp_u: float = -1.0
    grasp_v: float = -1.0
    grasp_strategy: str = 'legacy_center'


@dataclass
class ConfirmedDetection:
    """Spatially consistent result aggregated across a frame burst."""

    representative: Detection
    confirmation_count: int


@dataclass
class RegistryEntry:
    """Mission-lifetime identity independent of detector track IDs."""

    object_uuid: str
    class_name: str
    x: float
    y: float
    z: float
    last_seen: float
    observation_count: int = 1


def _distance_xy(left, right):
    return math.hypot(left.x - right.x, left.y - right.y)


def confirm_burst(detections, association_gate, min_confirmations):
    """Merge same-class detections and require distinct-frame agreement."""
    clusters = []
    for detection in sorted(detections, key=lambda item: -item.confidence):
        selected = None
        selected_distance = float('inf')
        for cluster in clusters:
            representative = cluster[0]
            distance = _distance_xy(detection, representative)
            if (detection.class_name == representative.class_name and
                    all(item.frame_index != detection.frame_index for item in cluster) and
                    distance <= association_gate and
                    distance < selected_distance):
                selected = cluster
                selected_distance = distance
        if selected is None:
            clusters.append([detection])
        elif all(item.frame_index != detection.frame_index for item in selected):
            selected.append(detection)

    confirmed = []
    for cluster in clusters:
        if len(cluster) < min_confirmations:
            continue
        strongest = max(cluster, key=lambda item: (item.frame_index, item.confidence))
        center_x = median(item.x for item in cluster)
        center_y = median(item.y for item in cluster)
        spread = median(
            math.hypot(item.x - center_x, item.y - center_y)
            for item in cluster
        )
        representative = replace(
            strongest,
            x=center_x,
            y=center_y,
            z=median(item.z for item in cluster),
            uncertainty=max(
                median(item.uncertainty for item in cluster), spread
            ),
        )
        confirmed.append(ConfirmedDetection(representative, len(cluster)))
    return confirmed


class ObjectRegistry:
    """Associate confirmed map observations with stable mission UUIDs."""

    def __init__(self, association_gate):
        self._association_gate = association_gate
        self._entries = {}
        self._retired = set()

    @property
    def entries(self):
        """Return a read-only-style view used by diagnostics and tests."""
        return tuple(self._entries.values())

    def retire(self, object_uuids):
        """Exclude collected identities for the remainder of this mission."""
        self._retired.update(object_uuids)

    def associate(self, detection, excluded_ids=()):
        """Return a stable UUID using class and map-plane proximity."""
        candidates = [
            entry for entry in self._entries.values()
            if entry.object_uuid not in excluded_ids and
            entry.object_uuid not in self._retired and
            entry.class_name == detection.class_name and
            math.hypot(entry.x - detection.x, entry.y - detection.y) <=
            self._association_gate
        ]
        if candidates:
            entry = min(
                candidates,
                key=lambda item: math.hypot(
                    item.x - detection.x, item.y - detection.y),
            )
            weight = min(entry.observation_count, 5)
            entry.x = (entry.x * weight + detection.x) / (weight + 1)
            entry.y = (entry.y * weight + detection.y) / (weight + 1)
            entry.z = (entry.z * weight + detection.z) / (weight + 1)
            entry.last_seen = detection.stamp_sec
            entry.observation_count += 1
            return entry.object_uuid

        object_uuid = str(uuid.uuid4())
        self._entries[object_uuid] = RegistryEntry(
            object_uuid=object_uuid,
            class_name=detection.class_name,
            x=detection.x,
            y=detection.y,
            z=detection.z,
            last_seen=detection.stamp_sec,
        )
        return object_uuid
