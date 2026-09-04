# ArUco 보정 기반 가상 웨이포인트 자율주행 구현

## 1. 문서 목적

이 문서는 TurtleBot3 모바일 매니퓰레이터의 현재 자율주행 구조와 다음
하드웨어 오류를 해결한 과정을 정리한다.

- 바닥 ArUco 마커가 카메라에서 사라지면 주행이 중단되던 문제
- 코너 목표 약 25cm 앞에서 전진하지 않고 계속 회전하던 문제
- 이전 Nav2 목표의 지연 취소 결과가 다음 목표를 실패시킨 문제
- 마커 좌표와 로봇의 이동·회전·작업 좌표가 결합되어 있던 문제

최종 구조의 핵심은 다음과 같다.

> ArUco 마커는 전역 위치를 가끔 보정하는 물리 랜드마크이고, 로봇은
> 마커와 독립된 `map` 좌표의 가상 웨이포인트로 이동한다.

## 2. 전체 구조

```text
카메라 + ArUco 지도                 diff_drive /odom
        │                                 │
        └──── aruco_localizer_node ───────┘
                       │
                 map → odom TF
                       │
                       ▼
routes.yaml ── aruco_waypoint_navigator_node ── NavigateToPose ── Nav2
                       │                                      │
                       ├── route status/events                └── /cmd_vel
                       ▼
          navigation_debug_recorder_node
                       │
            navigation.log + snapshots
```

### 컴포넌트별 책임

| 컴포넌트 | 책임 | 하지 않는 일 |
|---|---|---|
| `aruco_localizer_node` | ArUco 관측과 `/odom`으로 `map -> odom` 추정·유지 | 경로 결정, 정지 여부 결정 |
| `aruco_waypoint_navigator_node` | 이름 있는 가상 경로를 Nav2 목표로 순차 실행 | 영상 처리, 직접 `/cmd_vel` 제어 |
| Nav2 | 전역 경로 계획, 장애물 회피, 위치·최종 yaw 수렴 | 마커 표시 여부로 경로 단계를 제어 |
| `navigation_debug_recorder_node` | 이벤트, 주행 텔레메트리, 영상 스냅샷 저장 | 로봇 제어 |
| `cleanup_task_manager_node` | 순찰·물체 접근·집기·배출·복귀의 상위 상태 관리 | 지역 경로 제어 |

제어, 위치 추정, 로그 저장을 서로 다른 노드로 나눴기 때문에 카메라 저장
오류가 주행 제어에 영향을 주거나, 경로 코드가 마커 검출 상태를 직접
판단하는 결합을 피한다.

## 3. 물리 마커와 가상 경로 분리

두 종류의 좌표는 서로 다른 설정 파일이 소유한다.

- `src/aruco_localizer/map/new_map_markers.yaml`
  - 실제 바닥에 부착된 마커의 `id`, 크기, `map` 좌표와 방향
  - 위치 추정 전용
- `src/aruco_localizer/config/routes.yaml`
  - 로봇 `base_link`가 도달해야 할 `x`, `y`, `yaw`
  - 안전한 코너 피벗, 복도 주행점, 작업 접근점

`to_5` 경로는 현재 다음 순서로 실행된다.

```text
lower_corner_to_north
  → upper_corner_to_east
  → endpoint_5
```

`to_0`은 반대 방향용 웨이포인트를 사용한다. 서비스 이름은 기존 운영
호환성을 위해 `/navigate_to_marker_0`, `/navigate_to_marker_5`를 유지하지만,
서비스가 실제로 실행하는 대상은 마커 ID가 아니라 `routes.yaml`의 가상
경로이다.

체크인된 좌표는 기존 지도 형상을 보존한 초기값이다. 실제 초록색 회전점은
로봇을 그 위치와 방향에 놓고 캘리브레이션 도구로 기록해야 한다. 마커
좌표를 가상 웨이포인트에 복사해서는 안 된다.

## 4. ArUco 위치 보정과 오도메트리 연속 주행

### 4.1 TF 계산

검출한 마커 한 개에 대해 로컬라이저는 다음 관계로 `map -> odom`을
계산한다.

```text
T_map_base = T_map_marker × inverse(T_camera_marker) × T_camera_base
T_map_odom = T_map_base × inverse(T_odom_base)
```

유효한 여러 마커가 보이면 거리의 제곱에 반비례하는 가중치를 적용한다.
위치는 가중 평균, yaw는 사인·코사인 평균을 사용한다. 허용된 측정은 EMA로
반영하며 한 영상 프레임의 최대 보정량은 다음과 같이 제한한다.

- 평행 이동: `0.03m`
- yaw: `0.03rad`

### 4.2 보정 수용 정책

근거리 영상 왜곡과 회전 중 자세 흔들림이 Nav2의 기준 좌표를 끌고 가지
않도록 보정 허용 판단을 `localization_policy` 모듈로 분리했다.

기본 수용 조건은 모두 만족해야 한다.

- `base_link`에서 마커까지 바닥 평면거리: `0.40m` 이상
- 카메라에서 마커까지 3차원 거리: `2.50m` 이하
- `/odom` 각속도의 절댓값: `0.15rad/s` 이하
- 거리와 속도 값이 유한한 수

중요한 변경은 근접 판정을 카메라 사선거리가 아니라 `base_link` 기준 바닥
평면거리로 계산한다는 점이다. 카메라가 바닥보다 높기 때문에, 과거에는
로봇이 마커와 수평으로 25cm 떨어져 있어도 사선거리가 40cm보다 크게
계산되어 불안정한 관측이 계속 수용됐다.

근접하거나 회전 중인 관측은 버리되 마지막 `map -> odom`을 계속
브로드캐스트한다. 따라서 마커가 안 보이는 동안에도 TF 트리는 끊기지 않고
`/odom` 변화로 로봇 자세가 계속 진행한다.

현재 하드웨어의 diff drive 설정은 `open_loop: true`이므로 `/odom`은 명령
속도 적분 기반이다. 바닥 미끄러짐에 따른 장기 오차는 생길 수 있으며,
직선 구간에서 받아들이는 ArUco 관측이 이를 간헐적으로 보정한다.

### 4.3 위치 상태

로컬라이저는 `/aruco/localization_mode`에 다음 상태를 발행한다.

| 상태 | 의미 |
|---|---|
| `UNINITIALIZED` | 아직 유효한 마커 보정을 한 번도 받지 못함 |
| `MARKER_CORRECTED` | 최근 1초 이내에 유효한 보정을 받음 |
| `DEAD_RECKONING` | 마지막 보정을 고정하고 `/odom`으로 주행 중 |
| `DEGRADED` | 마지막 보정 후 120초 또는 누적 이동 8m 초과 |

웨이포인트 주행은 초기 `map -> base_link`가 만들어진 뒤에는
`MARKER_CORRECTED`에서 `DEAD_RECKONING`으로 바뀌어도 중단하지 않는다.

## 5. 가상 웨이포인트 실행 상태 머신

네비게이터의 주요 상태는 다음과 같다.

```text
IDLE
  → SENDING_GOAL
  → NAVIGATING
      ├── 성공 → 다음 웨이포인트 또는 ROUTE_COMPLETE
      ├── 실패 → RETRY_WAIT → SENDING_GOAL
      └── 정지 요청 → CANCELING → 최종 취소 결과 확인 → IDLE
```

각 웨이포인트의 위치와 최종 yaw를 하나의 Nav2 `NavigateToPose` 목표로
전송한다. 네비게이터가 직접 `/cmd_vel`을 발행하거나 IMU 회전 루프를
실행하지 않으므로, 이동과 최종 회전에 대한 단일 제어권은 Nav2가 가진다.

비동기 액션 콜백에는 증가하는 `current_goal_id`를 함께 캡처한다. 이전
웨이포인트의 지연 응답이나 취소 결과는 현재 ID와 다르면 폐기한다. 정상
실패에는 최대 2회 재시도와 500ms 쿨다운을 적용한다. 정지 요청은 취소
요청을 보낸 즉시 `IDLE`로 바꾸지 않고, Nav2의 최종 결과를 받은 뒤에만
경로를 종료한다.

## 6. 25cm 무한 회전 오류

### 6.1 증상과 로그

수정 전 주행에서 첫 번째 목표까지 남은 거리는 다음과 같이 변했다.

```text
1.436 → 1.036 → 0.636 → 0.261 → 0.236 → 0.242 → 0.250m
```

이후 `0.250m`에서 계속 회전했고, `controller_server`는 12초 뒤 다음 오류를
발행했다.

```text
Failed to make progress
```

### 6.2 근본 원인

두 개의 독립적인 허용 반경이 서로 달랐다.

```text
DWB RotateToGoal: 0.25m 안에서 직선 궤적 금지, 회전만 허용
SimpleGoalChecker: 0.12m 안에 들어와야 위치 도착 인정
```

로봇이 목표 25cm 지점에 들어오면 DWB는 전진 후보를 무효화했다. 그러나
GoalChecker는 아직 13cm를 더 가야 성공할 수 있었다. 결과적으로 로봇은
전진할 수 없고 목표도 완료할 수 없는 교착 상태에 들어갔다.

기존 `SimpleProgressChecker`는 평행 이동만 진행으로 판단했기 때문에 제자리
회전을 정상 진행으로 인정하지 않는 문제도 있었다.

### 6.3 수정

`nav2_params.yaml`의 기준을 다음처럼 정리했다.

| 설정 | 값 | 역할 |
|---|---:|---|
| `general_goal_checker.xy_goal_tolerance` | `0.12m` | 최종 위치 성공 반경 |
| `FollowPath.xy_goal_tolerance` | `0.10m` | DWB 회전 전용 진입 반경 |
| `general_goal_checker.yaw_goal_tolerance` | `0.20rad` | 최종 방향 성공 오차 |
| ProgressChecker | `PoseProgressChecker` | 위치와 회전 진행을 모두 인정 |
| `required_movement_angle` | `0.10rad` | 진행으로 인정할 각도 변화 |

DWB의 회전 전용 반경을 GoalChecker 성공 반경 안쪽에 배치했으므로, 직선
이동이 금지되는 순간에는 이미 위치 성공 조건을 만족한다. 이 불변 조건은
`test_nav2_config.cpp`에서 자동 검증한다.

## 7. 수정 후 실제 주행 검증

2026-09-04의 `to_5` 하드웨어 주행 결과는 다음과 같다.

| 항목 | 결과 |
|---|---|
| 시작 자세 | `(1.677, -0.497, -176.9°)` |
| 첫 번째 코너 | `lower_corner_to_north` 성공 |
| 두 번째 코너 | `upper_corner_to_east` 성공 |
| 종점 | `endpoint_5` 성공 |
| 재시도 | 전체 구간 `retry=0` |
| 전체 시간 | 약 `36.2초` |
| 최종 결과 | `ROUTE_COMPLETE` |

첫 번째 코너 회전 중 기록:

```text
Pose(map): (0.096, -0.516, 128.635deg)
CmdVel: (vx=0.000, wz=-0.440)
Localization: DEAD_RECKONING
```

두 번째 코너 회전 중에도 `vx=0`, `wz=-0.750`,
`Localization: DEAD_RECKONING`이 기록됐다. 즉 마커 보정이 끊긴 상태에서도
오도메트리 자세가 진행됐고, 각 코너가 `WAYPOINT_REACHED`로 정상 완료됐다.

성공 로그:

```text
nav_debug/run_20260904_200927_12795/navigation.log
```

## 8. 진단 로그

`navigation_debug_recorder_node`는 실행마다 다음 디렉터리를 만든다.

```text
nav_debug/run_YYYYMMDD_HHMMSS_PID/
├── navigation.log
└── snap_NNN_EVENT.jpg
```

주행 중에는 2초마다 `[TELEMETRY]`를 기록한다.

- `map -> base_link` 위치와 yaw
- `/odom` 위치, yaw, 각속도
- `/cmd_vel` 선속도와 각속도
- 위치 추정 모드, 마지막 보정 경과 시간과 이동거리
- 경로, 현재 웨이포인트, 재시도 횟수

실시간 확인:

```bash
tail -f /home/user/turtlebot3_ws/nav_debug/navigation.log
```

근접 또는 회전 때문에 마커 보정을 버리면 로컬라이저 ROS 로그에 다음처럼
표시된다.

```text
[Correction Frozen] reason=TOO_CLOSE ...
[Correction Frozen] reason=ROTATING ...
```

## 9. 빌드, 테스트, 실행

### 빌드와 자동 테스트

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select aruco_localizer
source install/setup.bash
colcon test --packages-select aruco_localizer
colcon test-result --verbose
```

현재 회귀 테스트는 다음을 검증한다.

- 가상 경로 YAML 로딩과 잘못된 좌표 거부
- 바닥 평면 근거리 마커 거부
- 회전 중 마커 보정 거부
- 비정상 거리 값 거부
- DWB 회전 반경이 GoalChecker 반경보다 크지 않음
- 회전을 인식하는 ProgressChecker 사용

검증 결과는 `13 tests, 0 errors, 0 failures, 0 skipped`이다.

### 하드웨어 실행

터미널 1:

```bash
source /opt/ros/humble/setup.bash
source /home/user/turtlebot3_ws/install/setup.bash
ros2 launch project_bringup project.launch.py
```

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /home/user/turtlebot3_ws/install/setup.bash
ros2 service call /navigate_to_marker_5 std_srvs/srv/Trigger "{}"
```

반대 방향은 `/navigate_to_marker_0`, 정지는 `/stop_marker_patrol` 서비스를
사용한다.

## 10. 가상 웨이포인트 캘리브레이션

통합 브링업으로 `map -> base_link`가 정상 발행되는 상태에서 로봇을 원하는
초록색 피벗 위치와 최종 방향에 놓고 실행한다.

```bash
ros2 run aruco_localizer calibrate_waypoints.py
```

도구 명령:

```text
p                         현재 설정 출력
r lower_corner_to_north   현재 로봇 자세 기록
r upper_corner_to_east    현재 로봇 자세 기록
s                         원본 백업 후 routes.yaml 저장
q                         종료
```

저장 후 `aruco_localizer`를 다시 빌드하고 새 터미널에서
`install/setup.bash`를 source해야 설치된 설정이 갱신된다.

## 11. 알려진 제한과 다음 검증

- 실제 초록색 피벗 좌표는 현장에서 캘리브레이션해야 한다.
- 명령 적분 방식 `/odom`은 바닥 미끄러짐을 직접 측정하지 못한다.
- `ABORTED` 복구 도착 판정은 현재 위치 거리 15cm를 사용한다. 회전
  웨이포인트에서 반복 실패가 발생한다면 역할별 yaw 복구 조건을 추가해야
  한다.
- 다음 단계에서는 `to_0` 왕복, 마커를 가린 주행, 장애물을 둔 주행을 각각
  별도 세션으로 검증한다.

