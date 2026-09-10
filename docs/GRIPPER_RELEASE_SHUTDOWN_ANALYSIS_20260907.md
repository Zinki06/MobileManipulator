# 그리퍼 릴리즈 및 Ctrl+C 종료 원인 분석

분석 대상: 2026-09-07 17:29 실행, `cleanup_debug/run_20260907_173034_4531`.
최초 조사에서는 실행 코드와 설정을 변경하지 않았다. 후속 수정 및 실제 읽기
결과는 아래에 추가했다. 수정 금지 영역은 읽기만 수행했다.

## 후속: 종료 회귀 수정 및 실제 OpenCR 읽기

- Git 이력에서 `aded23e`는 ros2_control_node 직접 실행, `016429b`는 종료 래퍼
  도입 및 3초 child.wait 제한을 사용한다. 기존 3초 소멸자 정리와 새 제한이 충돌한다.
- 래퍼의 정상 정리 시간을 10초, SIGTERM 후 대기를 5초로 변경했다. 바깥 launch
  제한도 30초/10초로 맞췄다. release 실패, 강제 종료, 자식 비정상 종료는 종료 코드
  1로 보고하고 신호 escalation 및 실제 자식 returncode를 기록한다.
- 3.2초 정리 뒤 종료하는 가짜 컨트롤러, 릴리즈 실패 후에도 정리 완료, 비정상 종료,
  20초 정리로 제한을 초과하는 경우까지 포함해 관련 pytest 9개가 통과했다.
  robot_motion/project_bringup 빌드 및 Python 스타일 검사도 통과했다.
- 하드웨어 명령을 보내 재실행하지 않았으므로 실제 토크 해제 검증은 남아 있다.

컨트롤러가 포트를 사용하지 않는 것을 확인하고 2026-09-07 UNIX 시각
1788770606.7~1788770607.0에 `/dev/ttyACM0`, ID 200의 OpenCR 테이블을
READ 명령으로 3회 읽었다. heartbeat/torque/position/reboot 쓰기는 보내지 않았다.

| 항목 | 3회 관측값 | 해석 제한 |
| --- | --- | --- |
| ROS 연결 | 0 | 현재 상위 제어 연결 없음 |
| 모터 체인 연결 | 1 | 개별 모터 보호 상태를 뜻하지 않음 |
| 바퀴 토크 | 1 | 종료 후 토크 잔류를 뒷받침 |
| 팔 전체 토크 | 0 | 모터 하나만 OFF여도 0; 전부 OFF라는 뜻 아님 |
| 그리퍼 목표 | 2048 tick | 소멸자의 init_gripper(0.0) 목표와 일치 |
| 그리퍼 현재 위치 | 2198 tick | 실패 때 약 -0.0035m와 유사 |
| 그리퍼 현재 전류 | -1 raw | 현재 무부하/토크 OFF/캐시 등 구별 불가 |
| 그리퍼 목표 전류 | 80 raw | OpenCR 저장값; 모터 적용 확인과 별개 |
| OpenCR 전압 | 18.81V | 모터 단자 전압과 동일한지 확인 필요 |

보드 millis 값은 매번 증가했다. 전압은 주소 42의 uint32 값을 100으로 나눈
값이며 공식 펌웨어의 저장 방식과 일치한다. XM430 공식 입력 사양은
10.0~14.8V(권장 12V)이므로 공급 전압 및 모터별 Present Input Voltage를
우선 확인해야 한다. 이번 읽기는 실패 후 시점이므로 실패 순간 과전압을 입증하지 않는다.

추가로 ROBOTIS 공식 Basic Operation 문서는 그리퍼 ID 15에 Current-based
Position Control Mode를 요구한다. 현재 하드웨어 초기화에는 Operating Mode
설정/검증이 없다. 처음 빈 상태의 열림 성공은 모드 5 설정이나 장시간 부하 중
정상 상태를 입증하지 않는다. 미확인 모드, 지속 닫힘 목표, 공급 전압/보호 오류를
확인하기 전에는 단순 릴리즈 명령 변경을 근본 해결로 취급할 수 없다.

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide_basic_operation/
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#specifications

## 결론과 확실성

- **확정:** 놓기 명령은 세 번 모두 ROS 컨트롤러에 수락되었다. 목표 0.019m에
  비해 보고된 위치는 약 -0.0035m에 머물렀다. 재시도 누락, 목표 미전달,
  작은 허용오차 때문에만 발생한 실패가 아니다.
- **확정:** 현재 위치 제어 경로에서 액션의 `max_effort`는 실제 전류 제어에
  연결되지 않는다. 결과의 `effort=10`도 측정 힘이 아니다.
- **확정:** 하드웨어 비활성화 함수는 토크 해제 없이 성공을 반환한다.
  소멸자의 3초 대기 후 토크 해제와 종료 래퍼의 3초 후 SIGTERM이 충돌한다.
  릴리즈 실패 및 자식 비정상 종료도 Ctrl+C 시 래퍼 실패 코드로 전달되지 않는다.
- **미확정:** 실제 그리퍼 무동작의 물리적 원인. 모터 보호 정지, 토크 상태,
  기계적 접촉/끼임, 전류 제한, OpenCR 하위 통신 실패를 현재 로그로 구분할 수 없다.
- **미확정:** 이번 실제 종료에서 SIGTERM이 토크 해제 전에 프로세스를 죽였는지.
  해당 자식 종료 코드 및 모터별 Torque Enable 읽기 결과가 없다.

## 1. 실제 실패 시간선

근거: `/home/user/.ros/log/pick_and_place_4501_1788769798352.log` 및
`/home/user/.ros/log/ros2_control_node_4903_1788769803699.log`.

| UNIX 시각 | 관측 |
| --- | --- |
| 1788769934.198 | 집기 전 열기 명령 |
| 1788769935.603 | 위치 0.018983m, 정상 열림 확인 |
| 1788769942.212 | 닫기 명령, 목표 -0.010m |
| 1788769944.364 | 위치 -0.007478m, stalled=1, reached_goal=0 |
| 1788769952.206 | 들어 올리고 복귀한 뒤 위치 -0.004487m |
| 1788769999.253 | 내려놓은 자세에서 열기 1/3 |
| 1788770002.273 | 열기 2/3 |
| 1788770005.342 | 열기 3/3 |
| 1788770008.114 | 세 번 모두 실패, 후퇴 없이 종료 |
| 1788770031.430 | Ctrl+C 종료 릴리즈 시작 |
| 1788770033.246 | 종료 릴리즈도 열림 확인 실패 |
| 1788770033.247 | 컨트롤러가 SIGINT/SIGTERM 수신 |
| 1788770033.256 | 컨트롤러 매니저 종료 로그 |
| 1788770036.528 | launch는 래퍼를 finished cleanly로 표시 |

세 열기 액션 결과는 모두 약 0.7~0.75초 후 `stalled=1`, `reached_goal=0`.
설정의 `stall_timeout=0.7`과 일치한다. 마지막 위치도 -0.003474m로, 목표와
약 22.5mm 차이가 난다. 1.5mm 열림 허용오차를 완화할 근거가 없다.
`canceled=0`이며 액션 수락 로그가 존재한다. 별도 종료 릴리즈 클라이언트도
실패하므로 놓기 재시도 루프만의 문제로 설명되지 않는다.

## 2. 실제 명령 경로와 누락된 상태

`pick_and_place → /gripper_controller/gripper_cmd → position GripperActionController
→ gripper_left_joint/position → hardware.write() → OpenCR 주소 216/221 → 모터 Goal Position`

- `src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/config/hardware_controller_manager.yaml`:
  그리퍼는 `position_controllers/GripperActionController`.
- `hardware_interface_adapter.hpp`의 위치 제어 구현은 목표 위치만 전달하고
  반환값으로 요청 `max_effort`를 돌려준다. 컨트롤러가 이것을 결과 effort에 넣는다.
- `turtlebot3_manipulation_hardware/src/turtlebot3_manipulation_system.cpp`의
  `write()` 역시 그리퍼 위치만 쓴다. 전류는 활성화 시 한 번 설정한다.
- `opencr_definitions.hpp`의 `GOAL_CURRENT=80`. XM430 기본 사양 단위로 환산하면
  약 215mA이나, 실제 적용 여부와 운영 모드는 레지스터를 읽어야 확인된다.
  ROS 액션의 10→15 변경은 이 경로의 전류를 바꾸지 않는다.
- `allow_stalling=true`는 닫을 때 물체에 막힌 상태를 액션 성공으로 반환하도록
  허용한다. 공식 컨트롤러 구현은 stall 시 액션 핸들을 끝내지만 위치 목표를
  현재 위치로 바꾸지 않는다. 따라서 닫힘 stall 뒤 약 55초 동안 원래 -0.010m
  목표가 유지된다. 이것이 과부하를 일으켰는지는 미측정이다.
- `set_gripper_position()`은 목표 쓰기 결과만 반환하고 뒤따르는 trigger 쓰기
  결과는 `void write_byte()`에서 버린다. `hardware.write()`는 오류를 찍어도
  `return_type::OK`를 반환한다. ROS 액션 수락은 모터 적용 확인이 아니다.
- 실제 전류, 모터별 토크, Hardware Error Status, 온도와 전압을 이 릴리즈
  판정 및 로그 경로에서 수집하지 않는다.

공식 컨트롤러 구현:
https://github.com/ros-controls/ros2_controllers/blob/humble/gripper_controllers/include/gripper_controllers/hardware_interface_adapter.hpp
https://github.com/ros-controls/ros2_controllers/blob/humble/gripper_controllers/include/gripper_controllers/gripper_action_controller_impl.hpp
로컬 설치 패키지 버전: ros-humble-gripper-controllers 2.53.1.
위 링크는 비교한 Humble 브랜치 소스이며 설치 바이너리 전체와의 동일성을 입증한 것은 아니다.

### 새 ROS 시각은 새 모터 측정 보장이 아니다

`OpenCR::read_all()`이 실패하면 이전 버퍼를 유지하고 하드웨어 `read()`는
그 버퍼에서 상태를 계속 채운다. 또한 비교한 공식 OpenCR 펌웨어는 모터
syncRead 성공 때만 저장 위치를 바꾸지만 상위 상태 갱신 함수는 실패 반환값을
처리하지 않는다. 따라서 3~13ms 전 `/joint_states`라도 하위 측정은 오래됐을
수 있다. 이번에 실제 캐시 정체가 발생했다는 뜻은 아니며 로그로 배제할 수 없다는 뜻이다.

공식 펌웨어 비교 소스(실제 보드 펌웨어 버전은 확인하지 못함):
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/open_manipulator_driver.cpp

## 3. Ctrl+C 종료의 구체적 결함

1. `controlled_hardware.py`가 `node.release()` 실패 반환값을 무시한다.
2. 하드웨어 `on_deactivate()`는 종료음과 로그만 출력한다. 토크 해제가 없다.
   이 클래스에는 명시적인 `on_shutdown()`도 없다.
3. `OpenCR::~OpenCR()`은 별도 팔 자세 및 그리퍼 0.0m를 명령하고 `sleep(3)` 후
   토크를 끈다. 앞서 열어둔 0.019m를 종료 과정에서 다시 0.0m로 바꿀 수도 있다.
4. 상위 래퍼는 SIGINT 후 정확히 3초 기다렸다 SIGTERM, 다시 2초 후 SIGKILL을
   보낸다. 정리 작업 자체가 3초 이상이므로 여유가 없고, 토크 해제를 보장하지 않는다.
5. `if not stopping and child.returncode` 때문에 Ctrl+C 종료 중 자식이 비정상
   종료되어도 래퍼는 정상 반환한다. launch의 clean exit는 래퍼 상태만 나타낸다.
6. 토크 쓰기 결과도 읽어 검증하지 않는다. 단순 대기 연장만으로 모터 토크 해제가
   확인되는 것은 아니다.
7. 비교한 공식 OpenCR 펌웨어의 heartbeat 단절 처리는 바퀴 속도를 0으로 만들지만
   그 지점에서 매니퓰레이터 전체 토크를 끄지는 않는다.

### 하드웨어 없는 종료 재현

실제 `controlled_hardware.main()` AST를 그대로 실행하고 ROS 연결만 스텁으로 대체했다.
자식은 SIGINT 뒤 3.2초 정리 후 토크 해제 표시를 남기도록 했다.

```text
elapsed=3.069s
child_returncode=-15 (SIGTERM)
wrapper_returned_normally=True
destructor_started=True
torque_off_reached=False
```

이는 종료 시간 충돌과 실패 은폐를 재현한 결과다. 실제 rclcpp의 신호 처리 및
실제 보드 동작을 재현한 테스트는 아니므로, 실제 사례가 SIGTERM으로 끝났다고
단정하지 않는다. 임시 재현 산출물: `/tmp/release_shutdown_audit_lbm3i5e9`.
현재 검사 시 실행 중인 controller/wrapper 프로세스는 발견되지 않았다.

## 4. 빨간 LED와 올바른 사용 절차

ROBOTIS의 OpenMANIPULATOR-X 사양은 XM430-W350-T를 사용한다.
XM430 공식 문서상 연속 점멸은 Shutdown Error 표시이며, 과부하·과열·전압·
엔코더 등의 오류 비트로 구분한다. 빨간색만으로 어떤 오류인지 판정할 수 없다.
보호 shutdown이면 Torque Enable이 0으로 내려가며, 재가동에는 원인 해소와
모터 reboot가 필요하다. ROS launch 재시작은 모터 reboot와 같지 않다.

전원 공급, Torque Enable, 그리퍼 열림, ROS 프로세스 종료는 서로 다른 상태다.
Ctrl+C가 배터리/어댑터 공급을 물리적으로 차단하지는 않는다. 종료 뒤 저항이
있거나 LED가 점멸하면 손으로 억지로 돌려 정상 종료 여부를 시험하지 말고,
팔과 물체를 지지한 상태에서 토크 및 오류를 확인해야 한다.

TurtleBot3에 부착된 현재 구성은 전용 TurtleBot3 Manipulation 펌웨어/bringup
경로다. 독립형 OpenMANIPULATOR의 직접 DYNAMIXEL 드라이버/포트 설정을
그대로 혼용하거나 실행 중 같은 포트를 다른 프로그램에서 열면 안 된다.
이번 조사에서는 포트 열기, 모터 명령, reboot, 펌웨어 업로드를 수행하지 않았다.

공식 문서:
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#shutdown63

## 5. 다음 수정과 검증 우선순위

1. 종료 시 릴리즈 성공과 토크 해제를 분리한다. 명령 생산을 중단하고 팔을
   지지 가능한 상태로 만든 뒤 모터별 토크 해제/읽기 확인을 수행한다.
   확인 실패는 실패 종료로 기록하며 숨기지 않는다. 소멸자 뒤쪽에만 맡기지 않는다.
2. 종료 신호와 자식 종료 코드, 토크 요청/확인 로그를 남기고 종료 예산을 정합화한다.
   기존 테스트의 즉시 종료 가짜 컨트롤러를 넘어 지연 정리·강제 종료를 검증한다.
3. 릴리즈 실패 순간 Goal/Present Position, Operating Mode(11), Torque Enable(64),
   Hardware Error Status(70), Goal Current(102), Present Current(126),
   Present Input Voltage(144), Present Temperature(146)를 모터에서 확인한다.
   이들은 XM430 주소이며 OpenCR 가상 제어 테이블 주소와 혼용하면 안 된다.
4. 오류 shutdown이면 먼저 그 원인을 해결한다. 토크가 켜져 있고 목표도 실제
   적용됐는데 전류만 높고 움직이지 않을 때 접촉/끼임/전류 제한을 구분한다.
   읽기 자체가 실패하면 오래된 관측으로 표시한다.
5. 부하 없는 열기 → 물체 집기 직후 열기 → 운반 시간만큼 유지 후 열기 →
   실제 놓기 자세에서 열기를 순서대로 비교한다. 반복 전체 미션보다 실패 단계를
   분리해야 한다. 팔 들어 올리기 복구는 사용자가 선택하지 않았으므로 추가하지 않는다.

현재 모델상 놓기 자세(-45도, 높이 35mm)의 손가락 최저점 여유는 약 13.8mm다.
모델이 바닥 충돌을 예상하지 않더라도 실제 바닥/보정/물체 형상 오차에 의한
접촉은 배제되지 않는다. 단순 재시도 횟수·stall timeout·허용오차·max_effort
수정만으로 문제를 해결했다고 선언할 근거는 없다.

기존 가짜 액션 테스트는 ROS 흐름을 검증했지만 실제 전류, 보호 오류, OpenCR
명령 적용, 지연된 토크 해제를 모델링하지 않았다. 이전 테스트 통과를 하드웨어
릴리즈 해결의 근거로 취급해서는 안 된다.
