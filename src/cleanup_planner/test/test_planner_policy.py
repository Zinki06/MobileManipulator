"""Tests for prompt boundaries and Gemini response validation."""

import json

import pytest

from cleanup_planner.planner_policy import build_prompt
from cleanup_planner.planner_policy import deterministic_fallback
from cleanup_planner.planner_policy import validate_decision


CANDIDATES = [{
    'object_uuid': 'banana-1',
    'class_name': 'banana',
    'confidence': 0.91,
    'map_position': {'x': 1.0, 'y': 2.0, 'z': 0.02},
    'position_uncertainty': 0.01,
    'station_name': 'scan_station_1',
    'heading_index': 2,
}]


def test_prompt_contains_allowlist_and_read_only_candidates():
    """Prompt must clearly carry bounded actions and UUID evidence."""
    prompt = build_prompt(
        'mission-1', 'scan_station_1', 'scan_station_2',
        ['collect_to_drop_zone'], CANDIDATES,
    )
    assert 'collect_to_drop_zone' in prompt
    assert 'banana-1' in prompt
    assert '하나의 행동만' in prompt


def test_validation_rejects_hallucinated_uuid():
    """A syntactically valid model answer cannot invent an object."""
    raw = json.dumps({
        'object_uuid': 'invented',
        'action': 'collect_to_drop_zone',
        'priority': 80,
        'reason': '바나나',
    })
    with pytest.raises(ValueError):
        validate_decision(raw, ['collect_to_drop_zone'], CANDIDATES)


def test_validation_rejects_unlisted_action():
    """The model cannot turn language into direct robot capabilities."""
    raw = json.dumps({
        'object_uuid': 'banana-1',
        'action': 'drive_somewhere_else',
        'priority': 80,
        'reason': '임의 행동',
    })
    with pytest.raises(ValueError):
        validate_decision(raw, ['collect_to_drop_zone'], CANDIDATES)


def test_validation_rejects_prioritized_skip():
    """Skip is a no-op and cannot reserve hidden priority semantics."""
    raw = json.dumps({
        'object_uuid': '',
        'action': 'skip',
        'priority': 80,
        'reason': '건너뜀',
    })
    with pytest.raises(ValueError):
        validate_decision(raw, ['collect_to_drop_zone'], CANDIDATES)


def test_fallback_collects_highest_confidence_banana():
    """Network failure still gives the deterministic MVP behavior."""
    decision = deterministic_fallback(
        CANDIDATES, ['collect_to_drop_zone'], 'timeout'
    )
    assert decision.object_uuid == 'banana-1'
    assert decision.action == 'collect_to_drop_zone'
