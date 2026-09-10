# 개발과 검증

## 수정 범위와 구조

워크스페이스 규칙은 [AGENTS.md](../AGENTS.md)를 따른다.
`src/realsense_bringup`, `src/segmentation`, `src/turtlebot3_manipulation`과
개인 `MEMO.md`는 수정하지 않는다. 외부 LiDAR 패키지는 해당 `CONTRIBUTING.md`도 따른다.

모든 패키지는 `src/` 아래에 둔다. C++ 구현은 패키지 `src/`, 공개 헤더는 `include/`,
Python 모듈은 패키지 이름의 하위 폴더, 설정/launch/지도/BT는 패키지 리소스 폴더에 둔다.
`build/`, `install/`, `log/`, 디버그 결과와 `.env`는 생성/로컬 자료다.

```bash
# 새 패키지를 만들 때 워크스페이스 src/에서 용도에 맞는 한 명령을 사용
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
```

C++/CMake 2칸, Python 4칸 들여쓰기를 사용한다. `snake_case` 이름,
C++ 타입 `PascalCase`, `-Wall -Wextra -Wpedantic`을 유지한다.
의존성은 `package.xml`과 `CMakeLists.txt`/`setup.py`에 함께 반영한다.
현재 큰 노드의 분리는 [리팩터링 검토](MAINTENANCE.md#검토-결론)를 기준으로 작게 진행한다.

## 빌드

현재 장비의 설치가 동작하면 인수인계만을 위해 시스템 의존성을 다시 설치할 필요는 없다.
새 환경에서는 ROS Humble을 설치한 뒤 다음을 수행한다.

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

이전 문서에는 보호 패키지의 `opencv2`/`pytest` rosdep key와 `pcl_ros` 관련 문제가 기록되어 있다.
새 설치에서 오류가 나면 실제 출력으로 원인을 확인한다. 현재 정리에서 의존성 설치를 재검증한 것은 아니다.
ROS 의존성 설치만으로 YOLO/SAM 가중치나 Python ML 환경이 복원되지는 않는다.

통합 진입점까지 빌드:

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-up-to project_bringup --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

전체 워크스페이스를 빌드할 때는 `--packages-up-to project_bringup`을 생략한다.
변경 패키지만 반복 빌드할 때는 `--packages-select aruco_localizer`처럼 선택한다.
인터페이스 변경 시에는 사용하는 패키지도 다시 빌드하고 기존 노드를 모두 재시작한다.
두 컴퓨터에서 코드를 사용하는 경우 메시지 정의와 설치 환경도 맞춘다.

## 테스트

아래 명령은 **개발 시험용 별도 터미널**에서 실행한다. 정상 운용 도메인 10과 분리하고,
시험 후 운영 터미널에서 `ROS_DOMAIN_ID=10`, `ROS_LOCALHOST_ONLY=0`을 사용한다.
고른 시험 도메인에 실제 장치를 연결한 노드가 없어야 한다.

```bash
source /opt/ros/humble/setup.bash
source /home/user/turtlebot3_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=191
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export OPENBLAS_NUM_THREADS=1
colcon test --executor sequential --packages-select \
  aruco_localizer robot_motion cleanup_perception cleanup_planner \
  cleanup_task_manager pick_and_place project_bringup
colcon test-result --verbose
```

외부 pytest plugin 자동 로딩을 끄는 것은 이 컴퓨터의 pytest/plugin 충돌을 피하기 위한 설정이다.
테스트 자체나 ament lint를 끄는 옵션은 아니다. CMake configure에서도 같은 변수를 주어
Python 시험 탐지 실패를 피한다. 등록된 시험이 없으면 성공으로 간주하기 전에
해당 패키지의 `build/<package>/CTestTestfile.cmake`와 테스트 결과를 확인한다.

| 변경 범위 | 관련 기존 시험 |
| --- | --- |
| launch/설정 | `project_bringup/test/test_feedback_config.py`, `robot_motion/test/test_performance.py`, `aruco_localizer/test/test_navigation_trees.py`, C++ config 검사 |
| 임무/복구 | `cleanup_task_manager/test/test_mission_flow.py`, `test_task_config.cpp` |
| 팔/배치 | `pick_and_place/test/test_pick_execution.py`, `test_grasp_kinematics.cpp` |
| 인식/기하 | `cleanup_perception/test/test_capture_pipeline.py`와 camera/floor/body/UUID 시험 |
| planner | `cleanup_planner/test/test_planner_policy.py`, `test_planner_transport.py` (가짜 HTTP) |
| 회전/정지/종료 | `robot_motion/test/`의 executor/hardware 시험, `aruco_localizer/test/`의 sensor/localization 시험 |

Python 변경은 `ament_flake8`, `ament_pep257`도 통과해야 한다.
동작 변경에는 회귀 테스트를 추가하고 실물 시험은 [인수인계 완료 기준](HANDOVER.md#현장-인계-완료-기준)에
따라 별도로 기록한다. 가짜 센서·컨트롤러 시험이 실물 성공을 증명하지 않는다.

## 이번 문서 정리의 검증

2026-09-10 런타임 소스/설정을 바꾸지 않고 아래 기존 시험을 실행했다.
ROS 연결이나 하드웨어 제어를 시작하지 않는 launch 해석, 순수 함수,
가짜 모델/HTTP callback 시험으로 **56 passed**를 확인했다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
  ROS_LOG_DIR=/tmp/turtlebot_handover_ros_logs \
  python3 -m pytest -q -o cache_dir=/tmp/turtlebot_handover_pytest_cache \
  src/project_bringup/test/test_feedback_config.py \
  src/aruco_localizer/test/test_navigation_trees.py \
  src/robot_motion/test/test_performance.py \
  src/cleanup_perception/test/test_capture_pipeline.py \
  src/cleanup_perception/test/test_camera_calibration.py \
  src/cleanup_perception/test/test_floor_alignment.py \
  src/cleanup_perception/test/test_grasp_candidates.py \
  src/cleanup_perception/test/test_object_registry.py \
  src/cleanup_planner/test/test_planner_policy.py \
  src/cleanup_planner/test/test_planner_transport.py \
  --junitxml=/tmp/turtlebot_handover_tests.xml
```

임시 JUnit 결과는 `/tmp/turtlebot_handover_tests.xml`에 있다. 영구 인수인계 증거가 필요한
실물 시험 결과는 실행 시각/프로파일과 함께 별도로 보관한다.
추가로 통합 launch의 `--show-args`, 문서 11개의 로컬 링크 67개와 셸 예제 34개의
문법을 확인했다. 보호 폴더와 `MEMO.md`의 파일 153개는 작업 전후 SHA-256이 일치한다.
배치 스케치는 바이트 변경 없이 이동했고, `src/`의 변경은 두 패키지 README뿐이다.
전체 colcon 재빌드·전체 ROS 통합 시험·실물 주행/파지·실제 Gemini 호출은 이번에 수행하지 않았다.

## 문서 유지 규칙

운영 방법은 `HANDOVER.md`, 설정은 `CONFIGURATION.md`, 구조는 `ARCHITECTURE.md`,
실험 도구는 `GRASP_CANDIDATES.md`, 장애 분석은 `DEBUG_LOG_GUIDE.md`를 갱신한다.
새 날짜별 문서에 운영 절차를 복제하지 않는다. 변경 이유와 남은 일은 `MAINTENANCE.md`에 요약한다.
기본값을 바꾸면 launch override와 두 프로파일을 확인하고 단독 노드 기본값과 구분한다.

리뷰에는 문제와 바뀐 동작, 영향 패키지, 실제 실행한 검사, 실물 미검증 범위를 기록한다.
기존 사용자 변경을 임의로 stage/commit하지 않는다. 외부 패키지의 DCO 요구는 해당 기여 규칙을 따른다.
