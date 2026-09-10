# cleanup_task_manager

현재 이동·중단 정책: [깊이 기반 우회 및 실패 복구](../../docs/CLEANUP_RECOVERY_AND_DEPTH_NAVIGATION.md).

최신 변경은 [충돌·파지점 수정](../../docs/COLLISION_AND_GRASP_FIX.md)을 참고한다.
스캔은 45도 간격 8방향, 물체 검출 확인은 3~5프레임이며 빈 방향은 기본 2프레임 검사 후
사진·JSON 저장 없이 다음 방향으로 넘어간다. 통합 launch에서는 카메라 보정·+30mm·몸통 후보/pitch 선택을
적용하고 `candidate_body`로 검증된 물체만 집는다. 큰 위치 불일치는 주행을 멈추며 자동 해제하지 않는다.

이 패키지는 바닥 정리 임무의 순서와 실패 복구만 담당한다. 위치 추정은
`aruco_localizer`, 이동·회전은 Nav2, 스캔 인식은 `cleanup_perception`, 제한된
행동 선택은 `cleanup_planner`, 팔 제어는 `pick_and_place`가 각각 소유한다.

## 임무 흐름

```text
팔 파킹
  -> 그리퍼 개방 및 최신 하드웨어/엔코더 피드백 확인
  -> scan_station_0 ... scan_station_5 순차 이동
  -> 각 스테이션에서 우회전 45도 + 최대 5프레임 촬영을 여덟 번 수행
  -> 후보별 map 방향으로 회전하고 TF 오차 확인 → 정지 → YOLO+SAM 재촬영
  -> 확인된 banana 후보를 Gemini planner에 전달
  -> 팔의 실제 TF와 파지/인양 IK로 도달 가능성 평가
  -> 닿지 않으면 계산한 자세로 정밀·저속 접근, 동일 UUID 재검출 및 IK 재평가
  -> 집기 및 원래 위치 재관측
  -> ID 3 바깥쪽 수거 지점으로 정밀 이동
  -> 팔을 낮추어 내려놓기 → 개방 확인 → 팔 후퇴·파킹
  -> 원래 스캔 스테이션으로 돌아와 남은 후보 또는 다음 스테이션 처리
```

Gemini는 인식된 UUID 중 하나와 allowlist의 행동만 선택할 수 있다. 모델이
없는 UUID나 행동을 만들거나, JSON 검증에 실패하거나, API timeout이 발생하면
가장 신뢰도 높은 확인된 banana를 고르는 결정론적 정책으로 fallback한다.
Gemini가 좌표, Nav2 goal, 팔 궤적을 만들거나 ROS 제어를 직접 호출하지 않는다.

주행 전 `/cleanup/prepare_gripper`에서 개방을 확인한다. 실패하면 첫 이동도
시작하지 않는다. 집기는 `/cleanup/execute_pick`의 `ExecutePick` 응답으로
실패 단계, 결과 코드, 닫기 시작 여부, 물체 보유 상태를 받는다.
`GripperCommand`의 SUCCEEDED만으로 개방이나 파지 성공을 판정하지 않는다.
그리퍼 고장과 닫기 후 피드백 소실은 임무를 중단하며, 실제 완전 닫힘이
확인된 `EMPTY_GRASP`만 빈 파지 재시도 대상으로 삼는다. 보유 상태가
불명확하면 자동 개방하거나 다음 물체로 이동하지 않는다.

## 설정과 안전 경계

`config/task_zones.yaml`은 여섯 스캔 자세, 360도 스캔 파라미터, 물체 위치
association gate, 대상 거리 제한, 배출 자세를 한곳에서 관리한다. 접근 거리는
고정된 `approach_standoff`가 아니라 `pick_and_place`의
`/cleanup/evaluate_grasp`가 실제 팔 장착 TF와 IK로 계산한다.

정리 목적지는 그림의 **ID 3 바깥쪽 초록 점**에 해당하는
`marker_3_outer_collection`이다. 지도에서 ID 3은 `(0, 1.5)`이며,
그림의 위쪽 바깥 방향은 map의 `-X`이다. 그림에 실측 거리가 없으므로
현재는 ID 3에서 바깥쪽40cm를 가정했다.

- 물체를 놓을 바닥 좌표: `drop_pose.placement = (-0.40, 1.50)`.
- 놓는 자세에서 성공한 수동 테스트와 동일하게 닫기(-0.010, effort 15) 완료 후
  5초 대기하고 열기(0.019, effort 10)를 한 번 실행합니다. 각 액션 결과는 최대
  15초 기다리며 신선한 엔코더로 열림을 확인한 뒤에만 후퇴합니다.
  대기 중 정지 요청도 처리합니다. `place_service_timeout` 기본값은 65초입니다.
  수동 테스트 순서를 재현하는 변경이며, 실제 모터 정체 해결 여부는 실기 확인이 필요합니다.
- 로봇 정지 자세: `drop_pose = (-0.16, 1.50, yaw=π)`.
- 내려놓을 때 nominal TCP 높이: 바닥 위35mm.
- 재수거 제외 반경: **물체 위치** 중심20cm. 로봇 정지점 중심이 아니다.

배출 위치는 물체 좌표와 로봇 정지 좌표를 구분한다. 도착 후
`/cleanup/place_object`가 도달성/바닥 여유/잡힘을 검사하고,
상공 접근→하강→그리퍼 개방 확인→후퇴를 수행한다. 내려놓기 실패 시
그리퍼 개방이나 수거 완료 처리를 대신 실행하지 않고 임무를 중단한다.
이미 개방한 뒤 후퇴가 실패한 경우에는 released 상태를 따로 보존한다.

실측 거리가 달라지면 두 X 좌표를 함께 조정하여 물체가 로봇 전방24cm에
오도록 유지한다. 물체를 놓는 점은 주행 스캔선 `x=0` 바깥에 있다.
현장 벽/바닥 및 물체가 쌓였을 때의 여유는 전체 실물 실행 전에 확인해야 한다.
로봇 정지 좌표는 다음 명령으로 측정할 수 있다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

물체 identity는 YOLO track ID가 아니라 클래스와 `map` XY 거리로 생성한 임무
내 UUID다. 프레임마다 track ID가 바뀌어도 0.25 m gate 안의 같은 banana는
같은 UUID를 유지한다. 새 mission ID가 시작되면 registry는 초기화된다.
`cleanup_perception`은 기존 `segmentation/best.pt`로 후보를 확인한 다음 기존
`segmentation/sam2_t.pt`로 확정된 bbox만 분할해 mask 기반 depth 중심점을
계산한다. SAM2를 사용할 수 없으면 bbox 중심 depth로 자동 fallback한다.

한 물체의 depth·재검출·IK 실패와 빈손이 확인된 집기 실패는 해당 UUID만 제외하고 나머지 순회를
계속한다. 스캔 회전은 시작 방향 기준의 절대 목표 각도로 오차를 보정한다.
회전 실패 시 스테이션 자세로 재정렬하고 한 번 재시도한다. 계속 막히면
`STATION_SCAN_INCOMPLETE`로 기록하고 이미 관찰한 후보를 처리한 뒤 다음
스테이션을 진행한다. localization이 `DEGRADED`가 되거나 필수 이동이
실패하면 전체 임무를 중단한다. 잡힘 가능성이 있는 실패 경로에서는 동작을 취소하고 그리퍼를 유지한 채 FAULT로 종료한다.
자동 개방하거나 다음 스테이션으로 진행하지 않는다. Ctrl+C 전체 종료의 개방은 유지한다.

## 실행과 관찰

워크스페이스 루트의 `/home/user/turtlebot3_ws/.env`를 열어 API 키를 입력한다.
이 파일은 Git에서 제외되며 브링업이 값을 자동으로 Gemini 노드에만 전달한다.
키가 없거나 API가 실패해도 임무는 결정론적 fallback으로 계속된다.
기본 모델은 `gemini-3.5-flash-lite`이며 `gemini_model` 런치 인자로 변경한다.

```bash
GEMINI_API_KEY=YOUR_API_KEY
```

저장한 다음 브링업을 실행한다.

```bash
ros2 launch project_bringup project.launch.py
```

다른 터미널에서 다음 명령을 실행한다.

```bash
source /home/user/turtlebot3_ws/install/setup.bash
ros2 service call /start_cleanup std_srvs/srv/Trigger "{}"
```

중지는 `/stop_cleanup`, 상태 확인은 `/cleanup/status`, 이벤트 확인은
`/cleanup/events`를 사용한다. 매 실행은
`cleanup_debug/run_YYYYMMDD_HHMMSS_PID/`에 `mission.log`,
`object_registry.json`, 스테이션별 주석 영상을 남긴다.

추가로 `planner.jsonl`에 실제 Gemini 요청 시작, 이미지 수, HTTP 응답,
결정/폴백 사유가 기록된다. `GEMINI_REQUEST_START`는 전송 시도이며
`GEMINI_HTTP_RESPONSE`의 status=200과 `PLANNER_RESULT`의 fallback_used=false를
함께 확인해야 Gemini 계획 성공이다. `MISSION_START`의 planner=gemini만으로는
호출 성공을 의미하지 않는다.

`*_raw.jpg`는 추론 전 원본이고 `*_object_N.jpg`는 해당 UUID의 bbox가 나온
대표 프레임이다. 검출 0개여도 촬영이 성공하면 원본·주석 JPEG가 저장된다.
YOLO 로딩 실패는 전체 임무 실패이며, 한 스테이션의 네 방향 촬영이 모두
실패해도 정상 완료로 처리하지 않는다. 한 후보의 정면 재관찰 실패는 그
UUID만 제외한다. 재관찰은 기본 최대 세 번 회전 보정하며 TF 각도 오차
0.10 rad 이내에서 재촬영한다 (`observation_max_turns`,
`observation_yaw_tolerance` 노드 파라미터).

변경 원인과 검증 기록: [정리 태스크 디버깅](../../docs/CLEANUP_DEBUGGING.md).

## 도달 가능성 기반 접근

평가 서비스는 팔을 움직이지 않는다. `map -> base_link -> link1` 변환 후
실제 집기에 쓰는 동일한 IK로 파지와 4~6 cm 인양 가능성을 검사한다.
불가능하면 차체 앞 0.30~0.18 m 범위에서 접근 자세를 찾는다. 2 cm 종방향
도착 여유도 검사하되 장애물/경로 검사는 Nav2에 맡긴다. 도착 후에는
항상 재검출·IK 재검사를 통과해야 집는다. 기본 접근 상한은 세 번이며
`max_approach_attempts` 파라미터로 제한한다.

물체 접근 BT만 `ApproachPath`(최대 목표 속도 0.06 m/s)와
`manipulation_goal_checker`(위치 0.02 m, 방향 0.08 rad)를 사용한다.
일반 스테이션 이동은 기존 `FollowPath`와 `general_goal_checker`를 유지한다.
두 기본 Nav2 BT에도 checker ID를 명시해 다중 checker 선택 오류를 방지한다.
이 설정은 IK 가능성을 높이는 것이며 실물 충돌/파지 성공을 보장하지 않는다.
