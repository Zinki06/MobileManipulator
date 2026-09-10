# 디버깅 로그 확인 방법

2026-09-10 현재 프로젝트 코드 기준. 아래 명령은 저장 파일을 읽거나 ROS 토픽을
구독한다. 브링업·정리 임무·팔 동작을 시작하는 명령은 포함하지 않는다.

## 1. 어느 로그부터 볼까

기본 작업 폴더는 `/home/user/turtlebot3_ws`다.

| 위치 | 확인할 내용 |
| --- | --- |
| `cleanup_debug/run_날짜_시각_PID/mission.log` | 정리 임무의 단계, 물체 선택, 접근·파지·배치 시도, 건너뛴 이유 |
| 같은 폴더의 `object_registry.json` | 물체 UUID별 관찰 및 수집·실패 결과 |
| 같은 폴더의 `planner.jsonl` | Gemini 요청/응답, 선택한 행동, fallback 여부 |
| 같은 폴더의 `scan_station_*/*` | 실제 검출 RGB, 파지 중심 픽셀, 후보별 거절 이유 |
| 같은 폴더의 `navigation_failure_*/` | 주행 실패 당시 목표·로봇 위치·오도메트리·costmap |
| `motion_debug/run_날짜_시각_PID/telemetry.jsonl` | 속도 명령 단계별 값, 실제 속도, 관절 위치, 근접 장애물, CPU·수신 주기 |
| `motion_debug/video_날짜_시각_*/clip_*.avi` | 접근·파지 등의 이벤트에 맞춰 저장한 RGB 영상 |
| 영상과 같은 이름의 `.frames.jsonl` | 프레임 시각, 이벤트, UUID 대응 |
| `nav_debug/run_*/navigation.log` | 별도 마커 경로 주행(`/route_navigation/*`)의 이벤트·상태 |
| `~/.ros/log/`의 노드별 `.log` | 팔 실행 단계, 충돌 감시, Nav2 내부 오류, 센서 처리, 종료 처리 |
| `performance_debug/` (있을 때) | 개발자가 별도로 저장한 빌드·시험·오프라인 분석 결과 |

`nav_debug/navigation.log`는 최신 recorder 실행 때 초기화되는 편의 파일이다.
이전 실행을 보려면 `nav_debug/run_*/navigation.log`를 선택한다. 마커 경로 주행과
정리 임무는 이벤트 토픽이 다르므로 `nav_debug`만 보고 정리 임무가 실행되지 않았다고
판단하지 않는다. 정리 임무는 **`mission.log`부터** 확인한다.

ROS 로그 경로는 `ROS_LOG_DIR`, `ROS_HOME` 설정에 따라 달라질 수 있다.
`launch.log` 하나에 모든 노드의 상세 출력이 모인다고 가정하지 말고 노드별 파일도 본다.

### 실행 폴더 선택

```bash
cd /home/user/turtlebot3_ws
python3 - <<'PY'
from pathlib import Path
for root, pattern in [('cleanup_debug', 'run_*'), ('motion_debug', 'run_*'),
                      ('motion_debug', 'video_*'), ('nav_debug', 'run_*')]:
    print(f'\n{root}/{pattern}:')
    paths = sorted(p for p in Path(root).glob(pattern) if p.is_dir())
    for path in paths[-5:]:
        print(path)
PY
```

폴더명의 시각은 각 기록 노드/임무가 시작한 시각이다. 같은 실행이어도
`cleanup_debug`는 정리 시작, `motion_debug`는 브링업 시작 시각이므로 이름이 다르다.
각 폴더의 가장 최신 파일을 무조건 짝짓지 말고 **문제가 발생한 시각과 UUID**로 연결한다.

과거 날짜별 문서가 가리키던 디버그 자료는 현재 작업 트리에 없다.
목록이 비어 있으면 분석할 실행 로그가 없는 상태다. 새 실행 후 위 목록에서
문제가 발생한 시각의 폴더를 골라 절대경로를 입력한다. 이후 명령은 같은 터미널에서 사용한다.

```bash
read -r -p '분석할 cleanup run 절대경로: ' CLEANUP_RUN
read -r -p '같은 시각 motion run 절대경로: ' MOTION_RUN
ROS_LOG_ROOT="${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"

tail -n 80 "$CLEANUP_RUN/mission.log"
rg -n 'FAILED|FAILURE|REJECT|SKIP|RETRY|PICK|GRASP|RELEASE|COLLECTED|MISSION_' \
  "$CLEANUP_RUN/mission.log"
```

`rg`가 매칭 결과 없이 종료 코드 1을 반환하면 해당 문구가 없다는 뜻이다.
이벤트 한 줄의 성공/실패만 보지 말고 전후 단계까지 함께 확인한다.

## 2. 실행 중에도 확인할 수 있다

`mission.log`는 이벤트마다 flush하고, `telemetry.jsonl`은 줄 단위로 기록한다.
**실행이 끝날 때까지 기다릴 필요가 없다.** 선택한 실행 폴더에서 다음처럼 본다.

```bash
tail -n 40 -F "$CLEANUP_RUN/mission.log"
```

위 `tail`만 실행한 별도 터미널에서 Ctrl-C를 누르면 로그 보기만 종료된다.
새 임무는 새 폴더를 만들기 때문에 그때는 `CLEANUP_RUN`을 다시 선택해야 한다.

이미 브링업 중인 로봇의 상태는 별도 터미널에서 다음 토픽을 하나씩 구독할 수 있다.
로봇과 같은 `ROS_DOMAIN_ID` 및 ROS 통신 환경을 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/user/turtlebot3_ws/install/setup.bash

ros2 topic echo /cleanup/events
# 위 구독을 Ctrl-C로 끝낸 뒤 필요한 항목을 선택한다.
ros2 topic echo /pick/events
ros2 topic echo /motion_guard/status --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /cleanup/carrying --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /aruco/localization_mode
```

각 `echo`는 계속 대기한다. 한꺼번에 실행하는 스크립트가 아니라 선택용 명령 목록이다.
`/cleanup/carrying`은 손가락 피드백으로 확인된 운반 상태이고,
`DEAD_RECKONING`은 마지막 지도 보정을 유지하며 오도메트리로 위치를 갱신하는 상태다.
운반 중 `DEAD_RECKONING` 자체는 오류가 아니다. `DEGRADED`와 구분한다.

## 3. 증상별 확인 순서

### 바나나를 봤는데 집으려 하지 않고 넘어간다

```bash
rg -n 'STATION_SCAN|SCAN_CAPTURE|OBSERVATION|PLAN|GRASP|APPROACH|PICK|OBJECT_FAILED' \
  "$CLEANUP_RUN/mission.log"
rg -n 'GEMINI_|PLANNER_RESULT|fallback|collect_to_drop_zone|skip' \
  "$CLEANUP_RUN/planner.jsonl"
```

1. 검출 → 확정된 UUID → 재관찰 → 수거 결정 → 파지 가능성 검사 → `PICK_START`
   중 어느 단계까지 진행했는지 확인한다. YOLO가 바나나를 봤다는 사실만으로
   파지 요청까지 전달된 것은 아니다.
2. 검출/재관찰 `.json`의 `confirmed_objects`, `uuid`, `grasp_valid`, `grasp_reason`,
   `grasp_strategy`, `grasp_pixel`, `centroid`를 확인한다.
3. `.candidates.json`에서 몸통 후보와 거절 이유를 본다.
   `insufficient_body_depth`, IK/바닥 간섭 등의 이유와 `GRASP_REOBSERVE_REQUIRED`,
   `OBJECT_FAILED`를 시간순으로 연결한다.
4. `PICK_OBSERVATION_AGE`의 실제 나이와 허용 시간을 비교한다.
   `TARGET_EXPIRED`는 노드별 ROS 로그에도 남는다. 현재 기본 프로파일의
   `target_max_age`는 6초지만, 해당 실행에 찍힌 값이 기준이다.

`PLAN_ACCEPTED` 후 `PICK_START`가 없다면 planner API만 반복해서 볼 것이 아니라
그 사이의 접근/기하 검사 이유를 확인한다. 기존 높이 오차와 관측 유효시간 사례는
[과거 변경 근거](MAINTENANCE.md#과거-변경에서-남긴-판단-근거)에 요약되어 있다.

### 중심점을 어디로 잡았는지 / 끝부분을 잡으려 하는지 확인한다

```bash
rg --files "$CLEANUP_RUN" -g '*.jpg' -g '*.json' -g '*.npz'
```

| 파일 | 용도 |
| --- | --- |
| `heading_*_raw.jpg` | 주석 없는 원본 RGB |
| `heading_*_capture_*.jpg` | 캡처의 검출/파지 주석 이미지 |
| `heading_*_object_N.jpg` | 특정 물체 후보의 증거 이미지 |
| 캡처 `.json` | UUID, bbox, `grasp_pixel=[u,v]`, map 좌표 `centroid` 등 |
| 물체 `.candidates.json` | 후보 계산과 선택/거절 근거. 보정 전후 정보가 있으면 함께 비교 |
| 물체 `.npz` | 저장된 RGB/depth/기하 입력 등을 이용한 오프라인 재현용 데이터 |

`grasp_pixel`은 이미지에서 선택한 픽셀이고 `centroid`는 map 좌표다.
팔 로그의 `target_link1`과 좌표계가 다르므로 숫자만 직접 비교하지 않는다.
팔의 `PLAN` 로그에 `strategy=candidate_body`, `forward_offset=0.030000`가 적용됐는지도
확인한다. 같은 UUID의 마지막 접근 재관찰 이미지가 실제 파지 입력인지 확인해야 한다.

`heading` 0~7은 기본 8방향 스캔 인덱스다. 249는 전체 몸통 재관찰,
250은 접근 후 재검출, 251은 선택적으로 활성화하는 파지 후 시각 검증이다.
현재 기본 프로파일은 파지 후 시각 검증을 생략하므로 251 이미지가 없어도 정상이다.

기본 빈 화면 조기 종료는 YOLO 음성 프레임 2개를 확인하면 사진 저장 없이 끝난다.
따라서 **사진이 없다는 이유만으로 검출 노드 실패라고 판단하지 않는다.**
`SCAN_CAPTURE` 이벤트와 인식 노드의 `[CAPTURE_PERF]` 등 처리 로그를 함께 본다.

### 잡았는데 들어 올리지 않거나, 잡고도 이동하지 않는다

먼저 `mission.log`에서 `PICK_START`, `PICK_VERIFIED`,
`NAVIGATION_START purpose=drop`까지 도달했는지 확인한다.
팔의 더 자세한 단계는 `/pick/events` 또는 노드별 ROS 로그에 있다.

```bash
rg -n 'PICK_PHASE|GRIPPER_RESULT|GRIPPER_FEEDBACK|EMPTY_GRASP|GRASP_HOLD_CONFIRMED|CARRY_MUTED|CARRY_SELF_FILTER|StopFootprint|Failed to make progress|MOTION_GUARD|MOTION_SENSOR' \
  "$ROS_LOG_ROOT" -g '*.log'
```

이 검색은 여러 실행을 포함하므로 출력 파일명과 시각으로 분석 대상 실행만 고른다.
대표 순서는 `OPEN → PREGRASP → DESCEND → CLOSE → LIFT → RETURN_HOME →
GRASP_HOLD_CONFIRMED`다. 배치는 `PLACE_APPROACH → PLACE_LOWER → PLACE_OPEN →
PLACE_RETREAT → PLACE_COMPLETE`를 확인한다.

- `CLOSE`만 있고 `LIFT`가 없다면 손가락 피드백 또는 팔 액션 실패 이유를 본다.
  액션 성공 응답만으로 물체가 잡혔다고 판단하지 않는다.
- `GRASP_HOLD_CONFIRMED` 후 drop 주행이 시작됐다면 `telemetry.jsonl`에서
  **속도 명령이 어느 단계에서 0이 되는지** 확인한다.
- `StopFootprint`로 멈췄다면 `sensors.depth`와 `sensors.laser`의
  `count_inside_22cm`, `points_inside_22cm`를 비교한다. 점 좌표는 base_link 기준이다.
  좌표 목록은 최대 32개이므로 전체 개수보다 적을 수 있다.
- 현재 두 프로파일은 `carry_clear_all_obstacles=true`다. 운반·파킹·신선한 관절
  피드백 조건을 통과하면 **외부 물체를 포함한 모든 depth 점을 비우고** `[CARRY_MUTED]`를 기록한다.
  빈 cloud만 보고 주변이 비었다고 판단하지 않는다. 이때 LiDAR와 센서/TF 감시는 유지된다.
- `carry_clear_all_obstacles=false`인 설정에서만 `[CARRY_SELF_FILTER] removed=... retained=...`가
  나오며 지정한 자기 물체 영역 밖의 depth 점을 보존한다.
  실행에 적용된 파라미터와 [운반 제한](CONFIGURATION.md#운반-중-깊이-장애물-처리)을 함께 확인한다.

속도 경로는 다음과 같다. 선속도 단위는 m/s, 각속도는 rad/s다.

```text
/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel_collision_checked → /cmd_vel
    Nav2          속도 평활화             충돌 감시              최종 guard
                                                                  ↓
                                                    /odom: 실제 측정 속도
```

`nav`부터 0이면 목표/경로/제어기 상태를 보고, `smoothed`는 움직이는데
`collision_checked`가 0이면 충돌 감시를 본다. 마지막 `/cmd_vel`에서만 0이면
guard 상태와 센서/TF 신선도를 본다. 최종 명령이 있는데 실제 속도가 0이면
제어기/하드웨어 로그와 오도메트리 시각을 확인한다.
각 단계 값은 비동기 수신된 마지막 값이므로 한 표본만으로 원인을 확정하지 않는다.

최근 20개 텔레메트리 표본을 간추려 보는 명령:

```bash
python3 - "$MOTION_RUN/telemetry.jsonl" <<'PY'
from collections import deque
from datetime import datetime
import json
import sys

recent = deque(maxlen=20)
with open(sys.argv[1]) as stream:
    for line in stream:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue  # 실행 중 아직 다 쓰이지 않은 줄은 다음 읽기에서 확인한다.
        if row.get('kind') == 'telemetry':
            recent.append(row)
for row in recent:
    value = row['value']
    sensors = value.get('sensors', {})
    print(datetime.fromtimestamp(row['time']).astimezone().isoformat(timespec='milliseconds'),
          'guard=', value.get('guard'),
          'nav=', value.get('/cmd_vel_nav'),
          'smooth=', value.get('/cmd_vel_smoothed'),
          'collision=', value.get('/cmd_vel_collision_checked'),
          'final=', value.get('/cmd_vel'),
          'odom=', value.get('odom'),
          'near_depth=', sensors.get('depth', {}).get('count_inside_22cm'),
          'near_laser=', sensors.get('laser', {}).get('count_inside_22cm'))
PY
```

속도 배열은 `[linear.x, angular.z]`, `odom`은 `[x, y, yaw, linear.x, angular.z]`다.
`sensor_stamps`와 `odom_stamp`도 함께 확인한다. recorder가 반복해서 적은 값이
실제 새 센서 데이터라는 뜻은 아니다. 종료된 파일에서 JSON 파싱 실패가 반복되면
파일 손상 여부를 별도로 확인한다.

과거 운반 정지 사례와 종료 변경 이유는
[유지보수 기록](MAINTENANCE.md#과거-변경에서-남긴-판단-근거)에 있다.

### ID 4 부근 등에서 localization fault가 난다

```bash
rg -n 'LOCALIZATION_FAULT|robot_position_error|offset_translation_error|DEGRADED|MOTION_GUARD' \
  "$ROS_LOG_ROOT" -g '*.log'
```

같은 시각의 odom, 마커 ID와 원시/필터 보정, TF 시각을 맞춰 본다.
`map → odom` translation 성분끼리 뺀 값은 실제 로봇 위치 점프와 다르다.
현재 코드는 같은 odom 로봇 점에 두 변환을 적용해 비교한다.
운반 중 `DEAD_RECKONING`은 정상일 수 있지만 `DEGRADED`/FAULT는 별개다.
원인을 확인하지 않고 보정 한도를 늘리거나 반복 재시작으로 진행하지 않는다.

### 주행 실패·재시도 또는 회전 후 오래 정지한다

```bash
rg -n 'NAVIGATION_|SCAN_TURN|SCAN_CAPTURE|SCAN_SETTL|OBSERVATION|PLAN|MISSION_FAILED' \
  "$CLEANUP_RUN/mission.log"
rg --files "$CLEANUP_RUN" -g 'context.yaml' -g '*costmap.yaml'
```

`navigation_failure_*/context.yaml`의 `reason`, `motion_guard`, `goal`, `robot`,
`robot_pose_available`, `odom_stamp`, `stationary`를 확인한다. costmap YAML의
`available`도 확인한다. 이 파일은 실패 순간의 스냅샷이므로 직전 몇 초간의
충돌 정지는 텔레메트리와 노드 로그에서 찾아야 한다. 스냅샷에 `READY`가 찍혔다고
직전 주행 내내 정상 명령이 전달됐다는 뜻은 아니다.

기본 스캔은 8방향, 약 45도 간격이다. 회전 종료부터 다음 이벤트까지의 시각 차이를
계산해 정지 확인, 인식, planner 응답, 주행 재시도 중 어디서 대기했는지 구분한다.
노드 로그의 `[CAPTURE_PERF]`, `[DEPTH_PERF]`와 아래 성능 기록도 확인한다.
스테이션/복귀/배출 이동의 `ABORTED`에는 기본 2초 대기 후 최대 2회 재시도가 있다.
센서·TF·odom 정지 조건을 확인하며 각 대기의 준비 제한은 10초다.
거절/취소, 물체 접근 실패, 회전 액션 실패는 같은 재시도 정책이 아니다.

### 종료했는데 그리퍼가 열리지 않는다

```bash
rg -n 'MISSION_RELEASE|MISSION_STOPPED|MISSION_COMPLETE|MISSION_FAILED' \
  "$CLEANUP_RUN/mission.log"
rg -n 'SHUTDOWN_RELEASE|GRIPPER_FEEDBACK_REJECTED|GRIPPER_RESULT' \
  "$ROS_LOG_ROOT" -g '*.log'
```

정상 완료·중단·실패는 `MISSION_RELEASE_START → MISSION_RELEASE_COMPLETE`를 확인한다.
`MISSION_RELEASE_FAILED` 또는 `MISSION_RELEASE_TIMEOUT`이면 개방 완료가 확인되지
않은 것이다. Ctrl-C 전체 종료는 하드웨어 wrapper의
`SHUTDOWN_RELEASE_START → SHUTDOWN_RELEASE_COMPLETE`를 확인한다.
`SHUTDOWN_RELEASE_FAILED`의 정지 피드백/취소/개방 실패 이유를 읽는다.
전체 종료 시 `mission.log`가 먼저 끊길 수 있으므로 ROS 노드 로그도 필요하다.

## 4. 시간·성능 값 읽기

`mission.log` 줄 앞의 값, JSONL의 `time`, ROS 로그의 `[1788693334....]`는 초 단위
시각이다. 해당 실행의 숫자를 바꿔 KST로 변환할 수 있다.

```bash
python3 - <<'PY'
from datetime import datetime
from zoneinfo import ZoneInfo
print(datetime.fromtimestamp(1788693334.463944, ZoneInfo('Asia/Seoul')).isoformat())
PY
```

`telemetry.jsonl`은 약 0.25초마다 `kind="telemetry"`, 약 5초마다
`kind="performance"`를 기록한다. 마지막 성능 기록은 다음처럼 확인한다.

```bash
python3 - "$MOTION_RUN/telemetry.jsonl" <<'PY'
import json
import sys
last = None
with open(sys.argv[1]) as stream:
    for line in stream:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get('kind') == 'performance':
            last = row
print(json.dumps(last, ensure_ascii=False, indent=2))
PY
```

- `processes[].cpu_percent_one_core`: 한 코어 100% 기준. 첫 표본은 비교값이 없어
  `null`일 수 있다. `executable`이 `python3`이면 PID와 해당 실행 로그를 함께 확인한다.
- `rss_mib`: 프로세스 메모리 사용량.
- `topics.*.received_hz`: 진단 노드가 실제 받은 빈도. 발행 노드 내부 제어 주기와 다르다.
- `topics.*.max_input_age_seconds`: 수신 순간 메시지 헤더의 최대 나이.
  헤더 없는 속도 명령은 `null`일 수 있다.
- 성능 기록이 없는 오래된 실행에서는 이 정보를 복원할 수 없다.

## 5. 문제를 다시 분석할 때 남길 것

문제가 발생한 스테이션 ID, 대략적인 KST 시각, 마지막으로 관찰한 동작을 적고
해당 `cleanup_debug/run_*`, `motion_debug/run_*`, 영상 폴더와 같은 시각의 ROS 로그를
함께 보관한다. `PICK_START`만 잘라내기보다 그 전의 재관찰·파지 검사와 이후 실패
구간까지 남기면 같은 UUID와 시각으로 원인을 추적할 수 있다.

현재 동작은 [구조](ARCHITECTURE.md), 실제 적용값은 [설정](CONFIGURATION.md),
파지 기하는 [몸통 후보](GRASP_CANDIDATES.md), 과거 원인과 남은 검증은
[유지보수 기록](MAINTENANCE.md)에 정리되어 있다.
