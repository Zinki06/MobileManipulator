# 바닥 정리 태스크 장애 수정 및 검증 (2026-09-05)

현재 로그 확인 방법은 [DEBUG_LOG_GUIDE.md](DEBUG_LOG_GUIDE.md)를 참고한다.
이 문서는 2026-09-05 당시의 장애·검증 기록이며, 아래 처리 흐름과 실행 설정은
현재 구현과 다를 수 있다. 현재 운반·종료 동작은
[CARRY_RELEASE_20260906.md](CARRY_RELEASE_20260906.md)에 정리되어 있다.

당시 19:27 실험의 충돌/파지 수정은
[COLLISION_AND_GRASP_FIX.md](COLLISION_AND_GRASP_FIX.md)를 참고한다.
아래 빈 화면 저장·8프레임 스캔 설명은 이전 구현 기록이다.

## 확인된 원인

실패 실행 `cleanup_debug/run_20260905_180814_8021/mission.log`에서 모든
캡처가 `YOLO unavailable:`을 반환했다. `_ensure_model()`이 모델을 로드하는
코드 없이 끝났고, 실제 로딩 블록은 `_ensure_sam_model()`의 return 뒤에
있었다. 따라서 사진 수집, 객체 등록, Gemini 요청, 접근/집기가 모두
시작되지 않았다. 빈 오류 문자열은 모델 경로 문제의 증거가 아니라 이
함수 분기 오류의 결과였다.

인식 오류를 방향별로 건너뛰다가 마지막에 `MISSION_COMPLETE collected=0`을
기록하는 동작도 혼동을 만들었다. 이제 모델 불가와 전체 방향 촬영 실패는
`MISSION_FAILED`로 구분한다.

실제 API 검증에서는 기존 `responseFormat.text.mimeType="application/json"`이
HTTP 400을 만들었다. SDK 예제와 달리 이 REST 필드는 enum을 요구하며
`APPLICATION_JSON`으로 수정한 뒤 HTTP 200 및 schema 검증을 통과했다.
근거: [Gemini REST TextResponseFormat 명세](https://ai.google.dev/api/generate-content#TextResponseFormat).

## 현재 처리 흐름

1. 스테이션 0~5를 방문하고 각 스테이션에서 네 방향을 촬영한다.
2. YOLO 후보를 depth/TF로 map에 투영하고 burst에서 확인해 UUID를 연결한다.
3. 스캔이 끝나면 후보별 방향으로 회전한다. TF의 남은 각도 오차를 확인하고
   최대 세 번 보정한다. 기본 허용 오차는 0.10 rad다.
4. 정지 후 YOLO+SAM으로 재관찰한다. 동일 UUID를 확인하면 최신 좌표와
   이미지를 채택하고, 못 찾으면 해당 후보만 제외한다.
5. Gemini에 최대 네 후보와 실제 JPEG, 이미지별 UUID/bbox 대응을 전송한다.
6. 응답 UUID/action 검증 후 팔 TF/IK로 접근 자세를 계산하고 Nav2 정밀 접근,
   동일 UUID 재검출, IK 재평가 후 집기를 수행한다.
7. 2번 배출 위치로 운반하고 그리퍼 열림·팔 파킹 후 수집 완료 처리한다.

정면 재관찰은 물체의 map 좌표가 확인된 경우에 수행한다. 유효한 depth/TF
없이 화면 일부만 보이는 물체를 추정해서 이동하지 않는다. 이런 검출은
`DETECTION_REJECTED`와 빨간 주석으로 남긴다. 위아래 잘림은 차체의 수평
회전만으로 해결되지 않을 수 있으며 카메라 시야/접근 간격의 현장 검증이
필요하다.

## 증거 저장과 관찰

`cleanup_debug/run_.../` 안에 다음 기록을 남긴다.

- `mission.log`: 재관찰, 계획, 접근, 집기, 배출 단계와 실패 사유
- `object_registry.json`: 객체 UUID와 완료/실패 상태
- `planner.jsonl`: 전송 시도, 이미지 수, HTTP 상태, 응답/폴백 사유
- `scan_station_*/heading_*_raw.jpg`: 추론 전 원본
- `heading_*_object_N.jpg`: 후보의 실제 검출 프레임과 UUID
- 캡처별 `.json`: 검출/확정 개수 및 UUID/bbox

Gemini 성공은 `GEMINI_REQUEST_START → GEMINI_HTTP_RESPONSE(status=200) →
PLANNER_RESULT(fallback_used=false)`로 확인한다. 키가 없으면
`GEMINI_NOT_SENT`, HTTP·파싱 실패면 `GEMINI_FAILED`가 기록된다. API 키는
로그나 이미지 메타데이터에 저장하지 않는다.

## 검증 결과와 한계

- 기존 실제 `best.pt` 로딩 및 CPU 추론 성공, banana 클래스 존재 확인.
- 최초 요청 모델 로딩, 검출 0개 이미지 저장, 확정 바나나 이미지/JSON 저장,
  모델/파일 저장 실패를 회귀 테스트로 확인.
- 같은 화면의 가까운 두 bbox를 버리거나 같은 UUID에 동시에 연결하지
  않도록 수정하고 테스트했다. 위치만으로 완전히 겹치는 물체의 identity를
  보장하는 것은 아니며 appearance 기반 연결은 후속 확장 사항이다.
- 실제 C++ manager와 가짜 Nav2/팔/인식 서비스를 별도 ROS domain에서
  연결해 24회 촬영, 정렬 오차 보정, 정면 재관찰, 계획, 집기, 배출 순서를
  검증했다. 모델 장애 중단과 재관찰 실패 시 다음 스테이션 진행도 검증했다.
- 집기 Trigger는 파지 전 실패와 파지 후 팔 복귀 실패를 구분하지 못하므로,
  호출 직전부터 물체를 쥐었을 가능성을 유지한다. 부분 실패 시에도 그리퍼를
  해제하고 팔을 파킹한 뒤 다음 스테이션으로 진행하며, 수집 성공으로
  기록하지 않는다. 이 부분 실패 경로도 격리된 ROS 시험으로 검증한다.
- `.env` 키로 검은 JPEG 한 장을 실제 Gemini에 전송했다. HTTP 200,
  `planner=gemini:gemini-3.5-flash`, `fallback=false`, `action=skip`을 확인했다.
  모델의 사유는 이미지가 어두워 바나나/안전을 확인할 수 없다는 것이었다.
  이 검증은 이미지 전송과 응답 경계를 증명하며 실물 바나나 판별/집기
  성공을 증명하지 않는다.
- 실제 세 바나나의 집기·운반 시험과 회전 정확도/카메라 잘림 검증은 남았다.

## 재시험 명령

### 19:04 실험에서 확인한 추가 장애와 수정

근거 세션: `cleanup_debug/run_20260905_190451_37258/`.
이 실행은 Gemini HTTP 200 및 바나나 수거 결정을 받았다. 하지만 대상까지
약 10 cm인 짧은 접근이 일반 도착 허용 오차 12 cm 안에 들어 즉시 성공으로
처리됐다. 팔 좌표의 실제 대상은 `(0.434, -0.034, -0.053) m`로 IK 범위 밖이었다.
`link1`이 `base_link`보다 9.2 cm 뒤에 장착되는 점도 고정 차체 접근 거리만으로는
반영되지 않았다. 이후 2번 스테이션 이동은 성공했고, 두 번째 스캔 회전에서
`Collision Ahead - Exiting Spin`이 반복돼 전체 임무가 종료됐다.

수정 사항:

- `cleanup_interfaces/EvaluateGrasp` 서비스로 팔 TF와 기존 IK를 재사용한다.
  팔을 움직이기 전에 파지/인양 가능성과 차체 접근 자세를 계산한다.
- 일반 주행은 기존 12 cm 허용 오차를 유지한다. 물체 접근만 2 cm/0.08 rad,
  저속 RPP 제어로 분리하고 충돌 검사는 유지한다. 두 일반 Nav2 BT에는
  `general_goal_checker`를 명시해 다중 checker의 기본 선택 실패를 방지한다.
- 도착 후 같은 UUID를 재검출하고 IK를 다시 검사한다. 여전히 멀면 최대 세 번
  접근하며, IK가 불가능하면 집기 명령을 보내지 않는다.
- 스캔은 시작 방향에 대한 각도 오차를 검사·보정한다. 회전 실패 시 스테이션
  자세로 재정렬 후 재시도하고, 계속 막히면 미완료로 기록하고 나머지 순회를
  진행한다. 장애물을 무시하거나 충돌 검사를 끄지는 않는다.
- 기본 모델을 `gemini-3.5-flash-lite`로 변경했다.
  [공식 모델 문서](https://ai.google.dev/gemini-api/docs/models/gemini-3.5-flash-lite).
  저장된 실제 바나나 이미지로 HTTP 200, fallback=false, collect_to_drop_zone을
  확인했다. 단일 요청은 약 17.1초였으며 항상 이 지연을 보장하지 않는다.
  기록: `cleanup_debug/api_probe_flash_lite_1788603583/planner.jsonl`.

회귀 시험은 일반 성공, 모델 장애, 재관찰 실패, 부분 집기 실패, 이른 도착
후 재접근, 도달 불가 시 세 번 제한, 회전 장애 후 순회 지속, 일시 회전 장애
복구를 포함한다. 실제 팔/바퀴를 움직이는 최종 현장 시험은 별도로 필요하다.
실제 Nav2 controller_server도 별도 ROS domain에서 configure까지만 실행해
`FollowPath`/`ApproachPath`와 두 goal checker 로딩을 확인했다. activate나
주행 명령은 보내지 않았다. 미완료 스캔 수는 최종 `MISSION_COMPLETE`의
`incomplete_scans`와 registry의 `incomplete_scan_count`에 별도로 남는다.

### 실행

기존 브링업 종료 후 변경된 설치 환경에서 실행한다. 이번 수정은 빌드했으며
실행 중인 노드는 재시작해야 적용된다.

터미널 1:

```bash
source /home/user/turtlebot3_ws/install/setup.bash && ros2 launch project_bringup project.launch.py
```

터미널 2:

```bash
source /home/user/turtlebot3_ws/install/setup.bash && ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'
```

첫 시험은 배출 제외 영역(2번 위치 반경 0.45m) 밖의 한 바나나로 수행한다.
`OBSERVATION_COMPLETE`, `PLAN_ACCEPTED`, `PICK_START`, `OBJECT_COLLECTED`와
실제 Gemini 성공 기록을 확인한 뒤 세 바나나 시험으로 확장한다.

개발 회귀 시험:

```bash
source /opt/ros/humble/setup.bash
source /home/user/turtlebot3_ws/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select cleanup_perception cleanup_planner cleanup_task_manager
colcon test-result --test-result-base build/cleanup_perception --verbose
colcon test-result --test-result-base build/cleanup_planner --verbose
colcon test-result --test-result-base build/cleanup_task_manager --verbose
```

현재 머신의 사용자 설치 anyio pytest plugin과 시스템 pytest 버전이 충돌하므로
회귀 명령에서 외부 pytest plugin 자동 로딩을 비활성화했다. 테스트 자체와
ament_flake8/ament_pep257 검사는 계속 실행한다.
manager의 CMake 테스트 등록 시에도 같은 환경변수가 필요하다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install --packages-select cleanup_task_manager --cmake-force-configure
```
