"""ROS service that calls Gemini through a strict, validated JSON boundary."""

import base64
import json
import os
from pathlib import Path
import re
import time
import urllib.error
import urllib.request

from cleanup_interfaces.srv import PlanCleanup
import rclpy
from rclpy.node import Node

from cleanup_planner.planner_policy import build_prompt
from cleanup_planner.planner_policy import decision_schema
from cleanup_planner.planner_policy import deterministic_fallback
from cleanup_planner.planner_policy import SYSTEM_PROMPT
from cleanup_planner.planner_policy import validate_decision


class GeminiPlannerNode(Node):
    """Expose Gemini as advisory planning with deterministic fail-closed input."""

    def __init__(self):
        super().__init__('gemini_planner_node')
        self.declare_parameter('planner_mode', 'gemini')
        self.declare_parameter('model', 'gemini-3.5-flash-lite')
        self.declare_parameter('api_key_environment', 'GEMINI_API_KEY')
        self.declare_parameter('timeout_seconds', 12.0)
        self.declare_parameter(
            'allowed_image_root', '/home/user/turtlebot3_ws/cleanup_debug'
        )
        self.declare_parameter('max_inline_image_bytes', 12_000_000)

        self._planner_mode = str(self.get_parameter('planner_mode').value)
        self._model = str(self.get_parameter('model').value)
        self._api_key_environment = str(
            self.get_parameter('api_key_environment').value
        )
        self._timeout = float(self.get_parameter('timeout_seconds').value)
        self._allowed_image_root = Path(
            str(self.get_parameter('allowed_image_root').value)
        ).expanduser().resolve()
        self._max_inline_image_bytes = int(
            self.get_parameter('max_inline_image_bytes').value
        )
        if self._planner_mode not in {'gemini', 'deterministic'}:
            raise ValueError('planner_mode must be gemini or deterministic')
        if not re.fullmatch(r'[A-Za-z0-9._-]+', self._model):
            raise ValueError('Gemini model name contains invalid characters')
        if self._timeout <= 0.0 or self._max_inline_image_bytes < 1:
            raise ValueError('Gemini timeout and image limit must be positive')

        self.create_service(
            PlanCleanup, '/cleanup/plan_objects', self._plan_callback
        )
        self.get_logger().info(
            f'Cleanup planner ready (mode={self._planner_mode}, '
            f'model={self._model}, api_key_configured='
            f'{bool(os.environ.get(self._api_key_environment, "").strip())}).'
        )

    @staticmethod
    def _candidate(observation):
        return {
            'object_uuid': observation.object_uuid,
            'class_name': observation.class_name.strip().lower(),
            'confidence': round(float(observation.confidence), 4),
            'map_position': {
                'x': round(float(observation.centroid.x), 4),
                'y': round(float(observation.centroid.y), 4),
                'z': round(float(observation.centroid.z), 4),
            },
            'position_uncertainty': round(
                float(observation.position_uncertainty), 4
            ),
            'station_name': observation.station_name,
            'heading_index': int(observation.heading_index),
            'observation_id': observation.observation_id,
            'bbox': [int(value) for value in observation.bbox],
        }

    def _plan_callback(self, request, response):
        response = self._plan(request, response)
        self._audit(request, 'PLANNER_RESULT', {
            'planner': response.planner_name,
            'fallback_used': response.fallback_used,
            'action': response.action,
            'object_uuid': response.object_uuid,
            'reason': response.reason,
            'raw_response': response.raw_response,
        })
        return response

    def _audit(self, request, event, details):
        record = {'time': time.time(), 'event': event,
                  'station': request.station_name, **details}
        self.get_logger().info(
            f'[{event}] ' + json.dumps(record, ensure_ascii=False)
        )
        mission = re.sub(r'[^\w-]', '_', request.mission_id)
        if not mission:
            return
        try:
            directory = self._allowed_image_root / mission
            directory.mkdir(parents=True, exist_ok=True)
            with (directory / 'planner.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(record, ensure_ascii=False) + '\n')
        except OSError as exception:
            self.get_logger().error(f'Planner audit save failed: {exception}')

    def _plan(self, request, response):
        candidates = [self._candidate(item) for item in request.observations]
        allowed_actions = list(request.allowed_actions)
        if not candidates:
            response.success = True
            self._fill_response(
                response,
                deterministic_fallback(candidates, allowed_actions, '후보 없음'),
                fallback=True,
                planner='deterministic',
                raw='',
            )
            return response

        if self._planner_mode == 'deterministic':
            self._fill_response(
                response,
                deterministic_fallback(
                    candidates, allowed_actions, '결정론 모드'
                ),
                fallback=False,
                planner='deterministic',
                raw='',
            )
            return response

        api_key = os.environ.get(self._api_key_environment, '').strip()
        if not api_key:
            self._audit(request, 'GEMINI_NOT_SENT', {'reason': 'API key missing'})
            return self._fallback_response(
                response,
                candidates,
                allowed_actions,
                f'{self._api_key_environment}가 설정되지 않음',
            )

        try:
            raw_text = self._call_gemini(
                api_key,
                request,
                candidates,
                allowed_actions,
            )
            decision = validate_decision(
                raw_text, allowed_actions, candidates
            )
            self._fill_response(
                response,
                decision,
                fallback=False,
                planner=f'gemini:{self._model}',
                raw=raw_text,
            )
            return response
        except Exception as exception:
            self._audit(request, 'GEMINI_FAILED', {'reason': str(exception)})
            self.get_logger().warning(
                f'Gemini plan rejected; deterministic fallback: {exception}'
            )
            return self._fallback_response(
                response, candidates, allowed_actions, str(exception)
            )

    def _call_gemini(self, api_key, request, candidates, allowed_actions):
        prompt = build_prompt(
            request.mission_id,
            request.station_name,
            request.drop_zone_name,
            allowed_actions,
            candidates,
        )
        parts = [{'text': prompt}]
        image_parts = self._image_parts(request.observations)
        if not image_parts:
            raise ValueError('No readable evidence images; Gemini was not called')
        parts.extend(image_parts)
        payload = {
            'systemInstruction': {'parts': [{'text': SYSTEM_PROMPT}]},
            'contents': [{'role': 'user', 'parts': parts}],
            'generationConfig': {
                'temperature': 0,
                'candidateCount': 1,
                'maxOutputTokens': 1024,
                'responseFormat': {
                    'text': {
                        'mimeType': 'APPLICATION_JSON',
                        'schema': decision_schema(allowed_actions),
                    }
                },
            },
        }
        url = (
            'https://generativelanguage.googleapis.com/v1beta/models/'
            f'{self._model}:generateContent'
        )
        http_request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Content-Type': 'application/json',
                'x-goog-api-key': api_key,
            },
            method='POST',
        )
        self._audit(request, 'GEMINI_REQUEST_START', {
            'model': self._model, 'candidate_count': len(candidates),
            'image_count': sum('inline_data' in part for part in image_parts),
            'images': [item.image_reference for item in request.observations],
            'prompt': prompt,
        })
        try:
            with urllib.request.urlopen(
                    http_request, timeout=self._timeout) as result:
                response = json.loads(result.read().decode('utf-8'))
                self._audit(request, 'GEMINI_HTTP_RESPONSE', {
                    'status': result.status,
                    'finish_reasons': [item.get('finishReason') for item in
                                       response.get('candidates', [])],
                })
        except urllib.error.HTTPError as exception:
            detail = exception.read(1024).decode('utf-8', errors='replace')
            raise RuntimeError(
                f'Gemini HTTP {exception.code}: {detail}'
            ) from exception
        candidates_response = response.get('candidates', [])
        if not candidates_response:
            raise ValueError('Gemini returned no candidate')
        response_parts = candidates_response[0].get('content', {}).get(
            'parts', []
        )
        text_parts = [
            item.get('text', '') for item in response_parts
            if isinstance(item, dict) and item.get('text')
        ]
        if not text_parts:
            raise ValueError('Gemini returned no JSON text')
        return ''.join(text_parts)

    def _image_parts(self, observations):
        parts = []
        seen = set()
        total_bytes = 0
        for observation in observations:
            reference = observation.image_reference.strip()
            if not reference:
                raise ValueError('Candidate has no evidence image')
            if reference in seen:
                continue
            if len(seen) >= 4:
                raise ValueError('More than four evidence images requested')
            seen.add(reference)
            path = Path(reference).expanduser().resolve()
            try:
                path.relative_to(self._allowed_image_root)
            except ValueError:
                raise ValueError(f'Refusing image outside evidence root: {path}')
            if not path.is_file():
                raise ValueError(f'Evidence image missing: {path}')
            if path.stat().st_size > self._max_inline_image_bytes - total_bytes:
                raise ValueError('Evidence images exceed inline byte limit')
            image_bytes = path.read_bytes()
            total_bytes += len(image_bytes)
            if not image_bytes.startswith(b'\xff\xd8\xff'):
                raise ValueError(f'Evidence is not JPEG: {path}')
            parts.append({'text': json.dumps({
                'image_candidates': [
                    {'object_uuid': item.object_uuid,
                     'bbox': [int(value) for value in item.bbox]}
                    for item in observations if item.image_reference == reference
                ]
            })})
            parts.append({
                'inline_data': {
                    'mime_type': 'image/jpeg',
                    'data': base64.b64encode(image_bytes).decode('ascii'),
                }
            })
        return parts

    def _fallback_response(self, response, candidates, allowed, reason):
        decision = deterministic_fallback(candidates, allowed, reason)
        self._fill_response(
            response,
            decision,
            fallback=True,
            planner='deterministic_fallback',
            raw='',
        )
        return response

    @staticmethod
    def _fill_response(response, decision, fallback, planner, raw):
        response.success = True
        response.fallback_used = fallback
        response.planner_name = planner
        response.object_uuid = decision.object_uuid
        response.action = decision.action
        response.priority = decision.priority
        response.reason = decision.reason
        response.raw_response = raw[:2000]


def main(args=None):
    """Run the constrained Gemini cleanup planner."""
    rclpy.init(args=args)
    node = GeminiPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
