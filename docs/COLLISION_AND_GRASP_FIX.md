# 충돌·촬영 지연·파지점 수정 (2026-09-05)

## 실제 실험과 원인

대상은 `cleanup_debug/run_20260905_192727_54450`이다. 사용자는 3번 이후
의자 바퀴에 걸림, 잘못된 방향으로 회전·이동, 벽 충돌을 보고했다.
Nav2의 `Goal succeeded`는 물리적 도착/무충돌의 증거로 취급할 수 없다.

- 3번에서 Gemini는 skip을 반환했고 manager는 4번 스테이션 goal을 보냈다.
  바나나 접근 goal을 보낸 기록은 없다. 하지만 잘못된 자기 위치로 실행한
  스테이션 goal은 실물에서 바나나/의자 방향으로 움직일 수 있다.
- 하드웨어 설정은 `diff_drive_controller.open_loop: true`였다. 바퀴 피드백이
  아닌 명령으로 odometry를 계산하므로 걸려도 소프트웨어상 이동할 수 있다.
  하드웨어 구현은 OpenCR의 실제 바퀴 위치/속도를 이미 state interface에 제공한다.
  [공식 컨트롤러 설명](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).
- 전역 costmap에는 실시간 obstacle layer가 없었다. 4번 재정합에서는
  마지막 기록 -37.4도에서 +122.7도로 map/odom 방향이 크게 변했고, manager가
  이를 스캔 회전 오차로 받아 약 -170도 회전을 보냈다.
- 매 방향 8프레임 추론과 빈 화면 JPEG 저장을 수행했다. 첫 YOLO predictor
  초기화도 첫 스캔 요청 안에서 수행됐다.
- depth가 유효한 픽셀들의 좌표 중앙값은 전체 물체 중심이 아니다. 일부가
  잘리거나 중앙 depth가 빠지면 끝부분으로 편향될 수 있었다. SAM이 실패해도
  bbox depth를 그대로 파지에 사용할 수 있었고 UUID 사진에는 실제 SAM 결과와
  파지점이 표시되지 않았다.

## 구조와 변경

### 피드백과 주행 안전

`project_bringup/launch/feedback_robot.launch.py`는 기존 description과 hardware
plugin, controller YAML을 재사용한다. YAML은 임시 파일로 재작성하며 원본은
수정하지 않는다. 차체 `open_loop=false`, 팔 `open_loop_control=false`를 적용한다.
회전 배율 기본값은 1.0이다. 엔코더 피드백에서 기존 1.2배 보정을 쓰지 않는다.
초기 팔 자세는 기존 `[0, -0.523, -0.523, 1.5707]`이다.
`use_sim=true`는 실물 실행으로 오인되지 않도록 명시적으로 거부한다.

모든 Nav2 제어/회전 명령은 `cmd_vel_nav`로 합쳐지고 velocity_smoother,
Collision Monitor, motion_guard를 순서대로 통과해 최종 `/cmd_vel`에 전달된다.
behavior_server의 기존 직접 `/cmd_vel` 출력도 이 경로로 변경했다.
node-qualified remap은 ROS resolver로 실제 해석을 확인했다.

- 차체 전진 최대 0.10 m/s, 후진 0.05 m/s, 회전 0.35 rad/s.
- 가속 제한과 명령/센서/TF timeout. 장애물 정지 명령은 가속 제한으로 지연하지 않는다.
- `obstacle_depth_node`는 YOLO와 별도 프로세스에서 약 5 Hz, 8픽셀 간격으로
  depth를 투영한다. 사진은 저장하지 않으며 물체 인식과 무관하게 계속 동작한다.
- LiDAR와 depth cloud를 전역·지역 지도 및 Collision Monitor에 연결한다.
  Humble의 Collision Monitor만으로 센서 timeout 정지를 가정하지 않고
  최종 motion_guard가 센서 timestamp와 TF를 별도로 검사한다.
- 일반 주행 BT에서 실패 후 자동 Spin/BackUp/지도 지우기 복구를 제거했다.
  정상 재계획은 유지하되 경로/제어 실패는 상위 상태 머신으로 돌려준다.
- 큰 ArUco reset은 반복 관측만으로 적용하지 않는다. 기존 TF를 유지하고
  DEGRADED로 고정해 정지한다. motion_guard도 TF 불연속을 감시하며 FAULT는
  자동 해제하지 않는다. 위치/센서 상태를 확인한 후 전체 재시작이 필요하다.
- 스캔 각도는 연속적인 odom에서 검사한다. map 재보정을 추가 회전량으로
  오인하거나 배율 보정 후 반대 방향으로 되돌리는 문제를 줄인다.

### 촬영과 중심 파지

- YOLO/SAM 로딩을 브링업으로 옮겼다. 초기 predictor warmup을 첫 스캔 전에 한다.
- 빈 방향은 동기 프레임 두 장으로 YOLO preview 후 사진 없이 종료한다.
  물체 확인은 최소 3프레임, 최대 5프레임이다. 확인되면 조기 종료한다.
  파지 성공 확인에는 빈 프레임도 최소 confirmation count를 사용한다.
- `_raw`, 일반 주석, `_object_N`은 같은 관측의 저장 변형이며 각각 별도 회전이 아니다.
- 전체 SAM 마스크의 기하학적 중심에 가까운 두꺼운 몸통 내부를 고른다.
  선택한 위치 주변의 depth만 사용한다. 중앙 depth가 없으면 끝점으로 이동하지 않는다.
  굽은 바나나의 기하학적 중심이 마스크 밖이면 가까운 몸통 내부를 사용한다.
- `ObjectObservation`에 `grasp_valid`, `grasp_reason`, `grasp_pixel`을 추가했다.
  bbox fallback은 탐색/identity용이며 파지 허가가 아니다. 잘린 물체나 불완전한
  마스크는 invalid로 표시하고 파지 명령을 금지한다.
- UUID JPEG에도 SAM 마스크와 녹색 파지점 십자를 표시하고 JSON에 좌표/사유를 저장한다.
  실제 전체 centroid를 볼 수 없는 근거리에서는 무리하게 집지 않는다. 이 경우
  카메라 시야와 파지 가능한 차체 위치가 겹치도록 현장 조정이 필요하다.
- 파지 검증 촬영 실패를 수집 성공으로 취급하지 않는다.

### Gemini의 역할

가구나 배경 전선이 가까이 보인다는 이유만으로 skip하지 않도록 했다.
실제로 물체가 전선에 얽혀 있거나 가구 아래에 갇혀 있거나 사람/동물과 접촉하는
증거는 여전히 skip 사유다. 단일 이미지로 통로의 거리/충돌 가능성을 단정하지 않으며
실제 경로 검증은 Nav2와 센서 계층이 맡는다.

같은 3번 사진을 실제 `gemini-3.5-flash-lite`에 다시 전송해 HTTP 200,
fallback=false, collect_to_drop_zone을 확인했다(2.63초, 1회 측정).
기록: `cleanup_debug/api_probe_furniture_1788605745/planner.jsonl`.

## 검증과 남은 물리적 한계

- 빈 화면 무저장/조기 종료, 중앙 depth 없음, 이미지 경계 잘림, 곡선 물체 중심,
  depth 단위/투영, invalid centroid 시 팔 호출 금지 회귀 시험.
- 실제 collision_monitor와 motion_guard를 격리된 ROS domain에서 실행했다.
  깊이 장애물 정지, depth 중단 정지, 명령 timeout, 속도 상한, TF jump 후
  정지 유지 시험을 통과했다. 이 테스트에는 실물 하드웨어가 없다.
- 원본 vendor YAML 불변, feedback/속도 override, Nav2 checker/BT 선택 시험.
- 엔코더와 실제 차체 이동의 일치, 팔 calibration, 접촉 위치, 파지 성공은
  아직 실물 검증하지 않았다. 이번 변경을 물리적 무충돌 보장으로 해석하면 안 된다.
- motion_guard는 차체 속도를 차단한다. 이미 호출된 Trigger 기반 팔 시퀀스는
  이 정지 경로로 선점 취소되지 않으므로 별도 하드웨어 비상 정지가 필요하다.
- 기본 차체 반경 0.20 m, 정지 반경 0.22 m는 실제 장착물 포함 외곽과 대조해야 한다.
  depth 장애물 높이는 0.08~1.5 m이며 8 cm 미만 물체, 센서 사각지대,
  엔코더가 회전하는 바퀴 미끄럼은 검출 한계가 남는다. 기존 마커·웨이포인트
  좌표가 실물 배치와 맞는지도 다시 확인해야 한다.

세 보호 폴더는 수정하지 않았다. 기존 브링업을 종료하고 환경을 다시 source한 뒤
동일한 `project_bringup project.launch.py`와 `/start_cleanup`을 사용한다.
전체 순회 전에 넓고 빈 공간에서 피드백/정지 확인과 바나나 한 개 시험이 필요하다.
