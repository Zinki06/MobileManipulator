# 터틀봇3 복도 마커 캘리브레이션 및 룰베이스 네비게이션 계획서

> 이 문서는 2026-09-04 이전의 마커 중심 경로 설계를 기록한 자료입니다.
> 현재 구현에서는 물리 마커와 가상 웨이포인트를 분리했습니다. 최신 구조와
> 검증 순서는 `localization_route_cleanup_architecture.md`를 참고하세요.

## 1. 개요 및 목적 (Overview)
본 문서는 로봇 차체 중심(`base_link`)과 카메라/매니퓰레이터 간의 물리적 오프셋, 실제 복도 규격 차이로 인해 발생하는 코너링 벽 간섭(충돌) 문제를 해결하기 위한 **수동 티칭 기반 마커 캘리브레이션 및 룰베이스 네비게이션 운영 계획**을 정리합니다.

---

## 2. 문제점 및 해결 방안 (Problem & Solution)

### 문제점
- 하드코딩된 이론적 마커 좌표(1.0m 간격)를 사용할 경우, 로봇의 회전 중심(바퀴 축)이 코너 지점에 도달하기 전에 회전을 시작하여 차체 앞부분이나 카메라가 코너 안쪽 벽에 충돌하는 현상 발생.

### 해결 방안 (수동 티칭 캘리브레이션)
- 사용자가 직접 키보드(`teleop_twist_keyboard`)로 로봇을 조종하여 **5번 ➔ 4번 ➔ 3번 ➔ 2번 ➔ 1번 ➔ 0번 마커 정중앙 및 코너 회전 안전 위치**로 이동.
- 각 지점에서 실제 90도 회전 자세를 맞추고 실시간 TF(`map -> base_link`) 및 IMU Yaw 각도를 캡처하여 `new_map_markers.yaml`에 영구 반영.

---

## 3. 캘리브레이션 워크플로우 (Teaching Workflow)

```mermaid
flowchart LR
    A[5번 마커 정중앙] -->|직진 티칭| B[4번 마커 코너 & 90도 좌회전]
    B -->|직진 티칭| C[3번 마커]
    C -->|직진 티칭| D[2번 마커]
    D -->|직진 티칭| E[1번 마커 코너 & 90도 좌회전]
    E -->|직진 티칭| F[0번 마커 정중앙 도착]
    F -->|저장 (s)| G[new_map_markers.yaml 업데이트]
```

### 단계별 티칭 상세
1. **5번 마커 (출발 지점)**:
   - 5번 마커 정중앙에 로봇을 위치시키고 마커 방향(서쪽, Yaw = 180°)으로 정렬 후 `5` 기록.
2. **4번 마커 (상단 코너 피벗)**:
   - 4번 마커 정중앙으로 이동 후, 로봇 몸통이 코너 벽에 걸리지 않도록 **정확히 90도 좌회전(남쪽, Yaw = -90°)**하여 3번 마커 방향을 바라보게 정렬 후 `4` 기록.
3. **3번 / 2번 마커 (중앙 직선 복도)**:
   - 중앙 복도를 따라 3번, 2번 마커 정중앙에 순서대로 정지하여 `3`, `2` 기록.
4. **1번 마커 (하단 코너 피벗)**:
   - 1번 마커 정중앙으로 이동 후, **정확히 90도 좌회전(동쪽, Yaw = 0°)**하여 0번 마커 방향을 바라보게 정렬 후 `1` 기록.
5. **0번 마커 (도착 지점)**:
   - 0번 마커 정중앙에 위치시킨 후 `0` 기록.
6. **저장 (`s`)**:
   - 기존 파일 자동 백업 (`new_map_markers.yaml.bak_<timestamp>`) 후 새로운 실측 좌표 저장.

---

## 4. 캘리브레이션 실행 가이드 (How to Run)

### 터미널 1: 프로젝트 전체 런처 실행
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch project_bringup project.launch.py
```

### 터미널 2: 키보드 원격 조종 (Teleop)
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 터미널 3: 대화형 캘리브레이션 툴 실행
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run aruco_localizer calibrate_waypoints.py
```

#### 캘리브레이터 명령어 요약
| 키 입력 | 기능 |
|:---|:---|
| **`0` ~ `5`** | 현재 로봇 위치를 해당 마커 번호의 좌표로 즉시 등록 |
| **`p`** | 현재 등록된 마커 좌표 목록 출력 및 확인 |
| **`s`** | `new_map_markers.yaml` 파일에 영구 저장 (백업 자동 생성) |
| **`q`** | 캘리브레이터 종료 |

---

## 5. 자율주행 및 검증 (Autonomous Execution)

캘리브레이션 완료 후, 로봇은 실측된 정밀 경유지와 IMU 폐루프 90도 회전을 바탕으로 안전하게 주행합니다.

### 5번 ➔ 0번 이동 명령
```bash
ros2 service call /navigate_to_marker_0 std_srvs/srv/Trigger "{}"
```

### 0번 ➔ 5번 역방향 이동 명령
```bash
ros2 service call /navigate_to_marker_5 std_srvs/srv/Trigger "{}"
```

### 셔틀 왕복 순환 패트롤
```bash
ros2 service call /start_marker_patrol std_srvs/srv/Trigger "{}"
```

---

## 6. 블랙박스 진단 및 로깅 (Diagnostics & Logging)
주행 중 발생하는 모든 주요 이벤트(도착, 회전 시작/완료, 마커 인식)는 아래 위치에 자동 기록됩니다:
- **텍스트 로그**: `/home/user/turtlebot3_ws/nav_debug/navigation.log`
- **카메라 스냅샷 이미지**: `/home/user/turtlebot3_ws/nav_debug/snap_*.jpg`

---

## 7. 안전 종료 절차 (Safe Shutdown Procedure)
로봇팔 모터(Dynamixel)의 자유낙하 방지 및 기구 보호를 위해 아래 순서로 종료합니다:

1. **로봇팔 안전 주차 (Park)**:
   ```bash
   ros2 service call /park_arm std_srvs/srv/Trigger "{}"
   ```
2. **런처 종료**: 런처 터미널에서 `Ctrl + C` 입력
3. **하드웨어 전원 OFF**: OpenCR 보드 전원 스위치 OFF
