# 인수인계와 실행

## 인수할 시스템

로봇에 연결된 컴퓨터에서 하드웨어·카메라·주행·팔·정리 노드를 실행하고,
원격 컴퓨터에서 RViz로 확인한다. 기본 임무는 스테이션 0~5의 바나나 수거다.
기존 지도와 장착 상태에 맞춘 설정이므로 장비를 옮긴 뒤 지도·마커·수거 위치를 확인한다.

2026-09-10 작업 환경에서 읽은 정보다. 원격 컴퓨터의 환경이나 장치 통신은 확인하지 않았다.

| 항목 | 확인값 |
| --- | --- |
| 워크스페이스 | `/home/user/turtlebot3_ws` |
| OS / CPU 아키텍처 | Ubuntu 22.04.5 LTS / aarch64 |
| ROS / Python | Humble / 3.10.12 |
| 현재 셸 환경 | `ROS_DOMAIN_ID=10`, `ROS_LOCALHOST_ONLY=0`, `LDS_MODEL=LDS-02`, `TURTLEBOT3_MODEL=waffle` |
| Python 패키지 메타데이터 | NumPy 1.26.4, opencv-python 4.9.0.80, torch 2.8.0, ultralytics 8.4.115, PyYAML 5.4.1 |
| YOLO / SAM2 가중치 | `src/segmentation/best.pt` / `src/segmentation/sam2_t.pt` 존재 확인 |
| 장치 연결 | 이번 점검 환경에서는 `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/serial/by-id`가 보이지 않았음 |

위 버전은 재설치용 의존성 잠금 파일이 아니다. 같은 컴퓨터를 넘기므로 기존 설치를
유지하고, Python/ROS 패키지 업데이트는 별도 검증 후 진행한다. 가중치와 `.env` 같은
로컬 파일은 Git clone만으로 복원된다고 가정하지 않는다.

## 처음 실행하기 전

1. 로봇·팔·그리퍼·카메라·LiDAR, 전원/충전기, USB 케이블과 ArUco 마커 배치를 함께 인계한다.
   실제 장치 경로, 접속 계정, 원격 컴퓨터 주소는 현장에서 확인해 전달한다.
2. 워크스페이스, 모델 가중치, 지도, 캘리브레이션 파일을 보존한다.
   `install/`은 재생성 가능하지만 현재 컴퓨터의 실행에 쓰이므로 정리 과정에서 지우지 않는다.
3. 로봇과 원격 컴퓨터를 같은 네트워크/ROS 도메인으로 맞추고 시각 차이를 확인한다.
   이 환경의 도메인은 10이다. 헤더 시각과 TF 신선도를 검사하므로 시각 차이는 오류 원인이 된다.
4. 지도에 없는 가구를 치운 시험 구역에서 시작하고, 팔이 초기 자세로 움직일 공간을 확보한다.
   현재 costmap은 실시간 장애물을 경로에 반영하지 않는다.
5. [운반 중 깊이 처리](CONFIGURATION.md#운반-중-깊이-장애물-처리)와
   [알려진 제한](MAINTENANCE.md#현장-확인이-남은-사항)을 확인한다.

API를 사용할 경우 `.env.example` 형식에 맞춰 루트 `.env`의 `GEMINI_API_KEY`를 설정한다.
기존 키 값은 문서에 복사하지 않고 인계받을 계정의 사용 여부를 정한다.
환경변수에 키가 있으면 `.env`보다 우선한다. `.env`에서 읽은 키는 planner 노드에 전달된다.
키가 없거나 API 요청/응답 검증에 실패하면 결정론적 banana 선택으로 fallback한다.
이 fallback도 planner 노드가 실행되어 있어야 한다.

## 로봇 측 실행

기존 설치가 있으면 다음으로 시작한다. 최초 설치/재빌드는 [개발 안내](DEVELOPMENT.md)를 따른다.

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export LDS_MODEL=LDS-02
export TURTLEBOT3_MODEL=waffle
ros2 launch project_bringup project.launch.py
```

하드웨어 bringup은 팔을 `[0, -0.523, -0.523, 1.5707]` 초기 자세로 이동한다.
정리 임무는 자동으로 시작되지 않는다. 카메라/모델/제어기와 초기 ArUco 위치 보정이
준비된 뒤 별도 터미널에서 실행한다.

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'
```

서비스의 `success`는 시작 요청 수락 여부다. 임무 완료는 `/cleanup/events`의
`MISSION_COMPLETE`와 수집 개수로 확인한다. `collected=0`이나 `incomplete_scans`가
있으면 전체 수거 성공으로 해석하지 않는다.

```bash
ros2 topic echo /cleanup/status
# 위 구독을 Ctrl-C로 끝내고 필요할 때 실행
ros2 topic echo /cleanup/events
```

## 중지와 종료

정리 중지는 별도 터미널에서 요청한다.

```bash
ros2 service call /stop_cleanup std_srvs/srv/Trigger '{}'
```

중지 응답 이후 `FINISHING`에서 그리퍼 개방을 기다린다.
`MISSION_RELEASE_COMPLETE`와 최종 상태를 확인한다. `MISSION_RELEASE_TIMEOUT`이면
요청이 남아 있어 새 임무가 차단된다. 실패/중지 시에도 물체를 놓을 수 있다.

전체 종료는 **브링업 터미널에서 Ctrl-C**다. 하드웨어 wrapper가 베이스 정지 확인,
진행 중 동작 취소, 그리퍼 개방 확인 후 제어기를 종료한다. ROS 로그에서
`SHUTDOWN_RELEASE_COMPLETE` 또는 `SHUTDOWN_RELEASE_FAILED`를 확인한다.
물체를 쥐고 있으면 개방 시 떨어질 수 있으므로 받칠 준비를 한다.
전원 차단·강제 종료는 이 소프트웨어 절차의 완료를 보장하지 않는다.

## 주행만 확인하기

인식·planner·pick·정리 매니저를 끄고 로봇/카메라/주행을 시작한다.
이 실행도 하드웨어와 초기 팔 자세 이동을 포함한다.

```bash
ros2 launch project_bringup project.launch.py \
  start_cleanup_perception:=false start_cleanup_planner:=false \
  start_pick_and_place:=false start_cleanup_manager:=false
```

아래 서비스는 목적에 맞게 **하나씩** 사용한다. 경로 순회와 정리 임무를 동시에 요청하지 않는다.

| 서비스 (`std_srvs/srv/Trigger`) | 동작 |
| --- | --- |
| `/navigate_to_marker_5` | 이름은 호환용이며 `routes.yaml`의 `to_5` 가상 경로 실행 |
| `/navigate_to_marker_0` | `to_0` 경로 실행 |
| `/start_marker_patrol` | 경로 왕복 순회 |
| `/start_station_scan_test` | 스테이션 0~5에서 각 90° 우회전 4회, 촬영·파지 없음 |
| `/stop_marker_patrol` | 위 경로/주행 시험 중지 |

```bash
ros2 service call /start_station_scan_test std_srvs/srv/Trigger '{}'
```

이 주행 시험의 4방향 회전은 **정리 임무의 45° × 8방향 인식**과 다르다.
`use_fake_hardware:=true`는 ros2_control만 가짜로 바꾸며 LiDAR/카메라 등은 별도로 시작한다.
전체 무장치 시뮬레이터로 사용하지 않는다. 현재 feedback launch는 `use_sim:=true`를 거부한다.

## 원격 컴퓨터의 RViz

같은 워크스페이스의 description/RViz 리소스를 설치하고 환경을 source한 뒤 실행한다.
로봇 측 통합 launch에는 RViz를 띄우지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d "$(ros2 pkg prefix --share turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
```

연결이 안 되면 도메인, 네트워크, 방화벽, 시각, `/map`·`/tf`·`/scan` 수신 순으로 확인한다.
계정 비밀번호나 API 키는 문서에 기록하지 않는다.

## 현장 인계 완료 기준

- 로봇과 원격 컴퓨터를 재시작한 뒤 위 명령으로 같은 환경이 올라온다.
- 원격 RViz에서 지도·로봇·센서가 보이고 ArUco 초기 위치가 실제 배치와 맞는다.
- 빈 구역에서 경로/회전·정지와 스테이션 도착을 확인한다.
- 바나나 한 개로 검출 → 접근 → 몸통 파지 → 인양 → ID 3 배치 → 복귀를 관찰한다.
- 여러 물체로 수집 개수, 재수거 제외 영역, 실패 처리와 종료 시 개방을 확인한다.
- 실행 시각, 프로파일, 물체 배치와 로그 위치를 인수자에게 남긴다.

이번 문서 정리에서는 실물 주행·파지·Gemini 호출을 실행하지 않았다.
과거 한 번의 파지 성공과 현재 전체 임무의 검증 상태는 구분한다.
