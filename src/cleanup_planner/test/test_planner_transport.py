"""Verify real request encoding, response validation and persistent API audit."""

import base64
import json
from types import MethodType
from types import SimpleNamespace
from unittest.mock import Mock

from cleanup_interfaces.msg import ObjectObservation
from cleanup_interfaces.srv import PlanCleanup
import cv2
import numpy as np
import pytest

from cleanup_planner.planner_node import GeminiPlannerNode


def planner(tmp_path):
    """Use production service methods without opening ROS connections."""
    node = SimpleNamespace(
        _planner_mode='gemini', _model='gemini-3.5-flash-lite',
        _api_key_environment='CLEANUP_TEST_KEY', _timeout=12.0,
        _allowed_image_root=tmp_path, _max_inline_image_bytes=12000000,
        get_logger=lambda: Mock(),
    )
    for name in (
        '_plan_callback', '_plan', '_audit', '_call_gemini',
        '_image_parts', '_fallback_response',
    ):
        setattr(node, name, MethodType(getattr(GeminiPlannerNode, name), node))
    node._candidate = GeminiPlannerNode._candidate
    node._fill_response = GeminiPlannerNode._fill_response
    return node


def plan_request(tmp_path):
    """Create a candidate whose bbox refers to an actual JPEG fixture."""
    path = tmp_path / 'scene.jpg'
    assert cv2.imwrite(str(path), np.zeros((64, 96, 3), np.uint8))
    observation = ObjectObservation(
        object_uuid='banana-1', class_name='banana', confidence=0.9,
        bbox=[10, 20, 30, 40], image_reference=str(path),
    )
    return PlanCleanup.Request(
        mission_id='test_run', station_name='station_0', drop_zone_name='station_2',
        allowed_actions=['collect_to_drop_zone', 'skip'], observations=[observation],
    )


def test_http_request_contains_image_and_response_is_audited(tmp_path, monkeypatch):
    """The callback sends JPEG bytes, mapped UUIDs and validates the HTTP reply."""
    node = planner(tmp_path)
    request = plan_request(tmp_path)
    monkeypatch.setenv('CLEANUP_TEST_KEY', 'test-key-do-not-log')
    calls = []

    def transport(http_request, timeout):
        calls.append(http_request)
        payload = json.loads(http_request.data)
        assert payload['generationConfig']['responseFormat']['text']['mimeType'] == \
            'APPLICATION_JSON'
        parts = payload['contents'][0]['parts']
        encoded = next(part['inline_data']['data'] for part in parts if 'inline_data' in part)
        assert base64.b64decode(encoded) == (tmp_path / 'scene.jpg').read_bytes()
        assert 'banana-1' in parts[1]['text']
        assert timeout == 12.0
        response = Mock(status=200)
        decision = json.dumps(dict(object_uuid='banana-1', action='collect_to_drop_zone',
                                   priority=80, reason='바닥 바나나'))
        response.read.return_value = json.dumps({'candidates': [{
            'content': {'parts': [{'text': decision}]}, 'finishReason': 'STOP',
        }]}).encode()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        return response

    monkeypatch.setattr('urllib.request.urlopen', transport)
    response = node._plan_callback(request, PlanCleanup.Response())
    assert len(calls) == 1
    assert response.success and not response.fallback_used
    assert response.object_uuid == 'banana-1'
    audit = (tmp_path / 'test_run' / 'planner.jsonl').read_text()
    assert 'GEMINI_REQUEST_START' in audit and 'GEMINI_HTTP_RESPONSE' in audit
    assert 'PLANNER_RESULT' in audit and '"image_count": 1' in audit
    assert 'test-key-do-not-log' not in audit


def test_missing_key_has_explicit_not_sent_event(tmp_path, monkeypatch):
    """Fallback is distinguishable from a successful Gemini request."""
    monkeypatch.delenv('CLEANUP_TEST_KEY', raising=False)
    node = planner(tmp_path)
    response = node._plan_callback(plan_request(tmp_path), PlanCleanup.Response())
    assert response.fallback_used
    audit = (tmp_path / 'test_run' / 'planner.jsonl').read_text()
    assert 'GEMINI_NOT_SENT' in audit
    assert 'GEMINI_REQUEST_START' not in audit


def test_missing_image_cannot_silently_create_text_only_request(tmp_path):
    """All supplied candidates must carry a readable evidence image."""
    node = planner(tmp_path)
    request = plan_request(tmp_path)
    (tmp_path / 'scene.jpg').unlink()
    with pytest.raises(ValueError, match='missing'):
        node._image_parts(request.observations)
