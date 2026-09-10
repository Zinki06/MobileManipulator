# 깊이 기반 우회와 정리 임무 실패 복구

## 이동 입력

프로젝트 bringup은 LiDAR를 실행하지 않는다. Nav2 경로 지도, collision_monitor,
motion_guard에서도 `/scan`을 사용하거나 필수로 기다리지 않는다.
차체 이동은 기존 오도메트리·아르코 보정·Nav2 경로를 유지한다.

빈손 이동에서는 local/global costmap의 `depth_layer`가
`/cleanup/obstacle_points`의 높이 0.08~1.5m 장애물을 반영한다.
1초 주기 경로 재계획을 유지하며 경로가 없으면 기존 제한 시간에 따라 실패한다.
자동 후진·회전 복구는 추가하지 않는다.

`carry_clear_all_obstacles: true`는 유지한다. 따라서 운반 중에는 새 장애물을
깊이 카메라로 감지해 우회하지 않으며 정적 지도·오도메트리로 이동한다.
`robot_motion`은 동작 접수 시 운반 상태와 두 costmap의 계층 상태를 동기화한다.
깊이 계층을 비활성화하고 잔상을 지운 뒤 정적 지도의 재갱신을 기다린다.
빈손으로 돌아오면 계층을 활성화하고 최신 깊이와 지도 갱신을 확인한다.
전환 제한 시간은 10초다. 실패·취소로 전환 완료가 불명확하면 이동 fault를 유지한다.
이동 중 운반 상태가 바뀌거나 운반 heartbeat가 0.75초 이상 끊기면 동작을 중단한다.
TF·오도메트리·깊이 입력 신선도와 최종 속도 제한 검사는 계속 적용한다.

`sync_depth_costmaps` 기본값은 true다. false는 Nav2 계층을 제공하지 않는
독립 모의 이동 테스트용이며 통합 bringup에서는 true를 사용한다.

## 중단과 개방

- 정상 배치: 하강·그리퍼 개방·후퇴를 수행한다.
- `/stop_cleanup` 또는 임무 실패: 팔·그리퍼·차체 동작을 취소하고 종료 및
  차체 정지를 확인한다. 그리퍼 개방이나 임의 파킹은 보내지 않는다.
- 빈손이 확실한 대상 실패: 기존 파킹 후 다음 대상 처리를 유지한다.
- 잡힘 가능성, 조작 서비스 시간 초과 또는 취소 미확인: `FAULT`로 남으며
  새 임무를 거절한다. 원인 확인 후 관련 노드를 재시작해야 한다.
- `/open_gripper`: 명시적 개방 요청이다. 팔 노드는 미종료 액션이나
  액션 fault가 있으면 개방을 거절한다.
- Ctrl+C 전체 종료: 기존 하드웨어 종료 래퍼의 정지 확인 및 개방을 유지한다.

새 `/cleanup/stop_manipulation` 서비스는 `std_srvs/srv/Trigger`를 사용한다.
팔과 그리퍼의 소유 액션을 취소하고 최종 액션 결과를 기다린다.
그리퍼 위치를 바꾸는 명령은 보내지 않는다.
액션 접수 응답 제한 시간은 3초이며 늦게 접수된 목표도 취소한다.
취소 후 종료 확인 제한 시간도 3초다. 불명확한 상태는 자동 해제하지 않는다.

배치 응답의 `released=true`는 뒤이은 후퇴·파킹 실패와 별도로 보존한다.
이미 놓은 물체를 미수거 상태로 되돌리거나 두 번째로 개방하지 않는다.

## 서비스 제한 시간

매니저 파라미터이며 초 단위다. steady clock으로 측정한다.

| 파라미터 | 기본값 |
| --- | ---: |
| `capture_service_timeout` | 60 |
| `planner_service_timeout` | 20 |
| `evaluation_service_timeout` | 5 |
| `pick_service_timeout` | 60 |
| `place_service_timeout` | 30 |
| `park_service_timeout` | 15 |
| `observe_service_timeout` | 10 |
| `stop_service_timeout` | 10 |

시간 초과 시 `SERVICE_TIMEOUT`을 기록하고 대기 요청을 제거한다.
세션·작업 번호가 지난 응답은 무시한다. 읽기 작업은 대상·스캔의 실패 정책을
따르며, 조작의 시간 초과는 종료·fault 처리한다. 종료 단계도 무기한 대기하지 않는다.
주요 종료 이벤트는 `MISSION_STOP_START`, `MISSION_STOP_COMPLETE`,
`MISSION_STOP_FAILED`이며 최종 임무 결과와 `/cleanup/status`의 상태를 함께 확인한다.

## UUID와 재빌드

`cleanup_interfaces/srv/CaptureObjects` 요청에 `string[] retired_object_uuids`를
추가했다. 매니저는 임무 내 수거 UUID 전체를 매 인식 요청에 전달한다.
인식 레지스트리는 이 UUID를 같은 임무에서 다시 공간 연결하지 않는다.
중복 전달은 무해하며 새 mission ID에서 초기화된다.
실패한 대상은 기존 ID를 유지해 반복 재시도를 막는다.

인터페이스가 바뀌었으므로 실행 중인 스택을 종료하고 관련 패키지를 함께 재빌드한 뒤
새 터미널에서 `install/setup.bash`를 source하고 전체 bringup을 다시 시작한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to project_bringup
source install/setup.bash
```

## 검증

`robot_motion/test/test_depth_costmaps.py`는 실제 Nav2 planner/controller를
모의 지도·TF·깊이 점군으로 실행한다. 우회 경로, 운반 전후 동적 장애물 제거,
정적 벽 보존과 빈손 복귀 후 장애물 재표시를 검사하며 하드웨어를 실행하지 않는다.
팔 테스트는 잡은 상태에서 중단, 늦은 접수, 취소 후 실행 지속을 검사한다.
매니저 테스트는 서비스 지연, 늦은 응답 무시, 실패 시 비개방과 UUID 전달을 검사한다.
현장 확인은 빈손 우회, 단일 물체 운반, 중단·실패 복구 순서로 별도 수행한다.
