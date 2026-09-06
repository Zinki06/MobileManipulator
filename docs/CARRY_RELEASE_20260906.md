# 2026-09-06 파지 후 정지 및 종료 시 그리퍼 해제

## 실제 정지 원인

`cleanup_debug/run_20260906_201332_82213/mission.log`에서 `PICK_VERIFIED`
직후 `NAVIGATION_START purpose=drop`은 정상 실행되었다. 팔은 인양과 운반 자세
복귀까지 완료했다. Nav2는 경로를 계속 받았으나 `Failed to make progress`로 재시도했다.

`collision_monitor_82131_1788693174804.log`는 같은 시각 `Robot to stop due to
StopFootprint polygon`을 기록한다. `motion_debug/run_20260906_201259_82112/telemetry.jsonl`
운반 구간 138개 표본 중 136개에서 깊이 점이 정지 조건을 만들었다. 같은 구간 LiDAR
정지 점은 없었다. 처음 6~9개 점은 base_link 기준 x=0.119~0.124 m,
y=-0.028~-0.014 m, z=0.219~0.252 m로 운반 자세의 손/잡힌 바나나 부근이다.
map→odom과 odom→base_link는 계속 갱신되고 있었다.

설정한 운반 물체 영역으로 기록된 깊이 점을 다시 분류하면 138개 표본 모두
영역 밖에 정지 조건을 만드는 점이 남지 않는다. 이는 저장 로그의 재분류 결과이며,
실제 주행 성공을 측정한 결과는 아니다. 집계: `performance_debug/carry_stop_diagnosis.json`.

## 변경 동작

- 실행 노드는 인양 및 운반 자세 복귀 후 안정적인 비어 있지 않은 손가락 피드백을
  확인한 경우에만 `/cleanup/carrying`을 발행한다. 200 ms 주기, 수신 유효시간 750 ms.
  손가락 피드백이 250 ms보다 오래되거나 빈 손 상태이면 운반 신호가 해제된다.
- 운반 중 로컬라이저는 마커 영상으로 map→odom을 수정하지 않는다. 마지막 보정을
  계속 방송하고 바퀴 오도메트리로 base 위치를 갱신한다. 기존 초기 위치 확인과
  오도메트리/TF 감시, 120초/8 m 보정 없는 이동 한도는 유지한다.
- 기본 프로젝트 프로파일은 `verify_pick_with_camera: false`이다. 실제 실행 노드가
  손가락·인양·복귀를 확인한 성공 응답을 받은 후 바로 정리 위치로 출발한다.
  운반 중 가려진 RGB로 다시 파지를 판정하지 않는다. 독립 manager의 기본값은
  기존 시각 검증을 유지하고, 필요하면 프로파일에서 다시 활성화할 수 있다.
- 깊이 점 제거는 운반 신호와 실제 운반 자세가 함께 확인될 때만 적용한다.
  joint1~4는 `[0, -0.523, -0.523, 1.5707]`에서 0.08 rad 이내,
  속도 0.05 rad/s 미만, 관절 피드백 250 ms 이내여야 한다.
  한 번의 영상 시각 TF 조회로 점을 base_link에 대조한다. 출력은 기존 카메라 좌표와
  `/cleanup/obstacle_points` 인터페이스를 유지한다.
- 프로파일의 `carry_mask_min: [0.06, -0.09, 0.16]`,
  `carry_mask_max: [0.18, 0.09, 0.35]` 안의 점만 제거한다(미터).
  외부 깊이 장애물과 LiDAR는 그대로 정지에 사용한다. 유효 깊이가 전부 손/바나나인
  경우 빈 클라우드는 허용하지만 원본 깊이 결손, TF 실패, 센서 중단은 허용하지 않는다.
- 완료·중단·실패는 `FINISHING` 상태에서 `/open_gripper` 완료를 기다린다.
  진행 중 팔/손가락 동작을 취소하고 최대 개방 위치 0.019 m를 확인한다.
  열기가 끝나기 전에 새 임무를 시작할 수 없다. 열기 실패를 정상 완료로 표시하지 않는다.
  30초 응답 초과는 로그에 남기고 미완료 요청이 끝날 때까지 새 임무를 차단한다.
- 프로젝트 하드웨어 런치는 `robot_motion/controlled_hardware`를 통해 기존
  ros2_control을 동일한 파라미터/리매핑으로 시작한다. Ctrl-C/SIGTERM 시 제어기를
  즉시 종료하지 않고 베이스 정지 확인 → 동작 취소 → 그리퍼 개방 피드백 확인 →
  제어기 종료 순서로 처리한다. 결과는 `SHUTDOWN_RELEASE_COMPLETE/FAILED`에 기록된다.
  전원 차단이나 SIGKILL, 제어기 고장에서는 물리적 개방을 보장할 수 없다.

## 검증

실제 로봇에 동작 명령을 보내지 않았다. ROS 테스트는 localhost 및 분리된 도메인에서
가짜 카메라/관절/액션 서버로 실행한다.

- 원본 및 필터 적용 PointCloud 형식, 자기 물체 제거, 외부 점 보존, 전부 자기 물체인
  영상, 잘못된 팔 자세/오래된 피드백/운반 신호 만료/깊이 결손.
- 실제 collision_monitor와 motion_guard에서 운반 중 빈 클라우드 허용,
  외부 깊이 장애물 및 LiDAR 정지, 센서/TF/명령 시간 초과와 map 점프 정지.
- 합성 ArUco 영상으로 초기화 후 RGB 없이 오도메트리 운반, 운반 중 보정 고정,
  놓은 뒤 마커 보정 재개.
- 가짜 관절 제어기에서 인양 후 운반 신호, 빈 손 실패, 정상 열기,
  파지 도중 중단 후 뒤늦은 CLOSE 방지.
- 가짜 전체 임무에서 가려진 카메라로도 place 도달, 정상/실패/중단 종료 시 개방.
- 실제 프로세스 그룹 SIGINT로 하드웨어 자식 프로세스보다 개방이 먼저 일어나는지,
  열림 피드백이 없으면 실패로 기록하는지 확인.

빌드 및 시험 결과: `performance_debug/carry_release_build.log`,
`carry_release_colcon_tests.log`, `carry_localization_safety_tests.log`,
`carry_depth_tests.log`, `shutdown_release_test.log`.

실기 재시험에서는 `GRASP_HOLD_CONFIRMED` 후 `CARRY_SELF_FILTER` 제거 개수,
`DEAD_RECKONING` 상태, drop 목적지 도착 및 낮춰 놓기, 종료 시 손가락 실제 개방을
확인한다. 물체 크기/운반 자세가 바뀌면 영역을 로그와 대조해야 하며 전체 깊이
장애물 감시를 끄는 방식으로 확대하지 않는다.

최종 검증: 전체 20개 패키지 빌드 성공. 수정한 5개 패키지의 colcon 시험에서
오류/실패 0개. robot_motion의 기존 copyright 검사 1개는 원래 설정대로 skip.
가짜 전체 임무 26개 및 파지 실행 11개 시험 통과.
패키지별 집계는 `performance_debug/carry_release_validation.json` 참고.
추가 시험 로그: `carry_aruco_tests.log`, `carry_pick_tests.log`.
