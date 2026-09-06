# 공통 주행 실행 계층과 파지 재관측

> 후속 실험으로 확인된 원인과 변경: [2026-09-06 수정](GRASP_AND_LOCALIZATION_20260906.md).
> 특히 map→odom translation 차이는 로봇 위치 점프와 같지 않으며, 이후 코드는
> 로봇 점에서의 변환 차이로 검사한다. 파지·영상 기록도 후속 문서가 우선한다.

2026-09-05. 이 문서는 기존 회전 재시도/스캔 재중심 이동 정책보다 우선한다.
실물 성공 판정이 아니라, 구현 변경과 하드웨어 없는 검증의 범위를 기록한다.

## 반복 오류의 원인

| 관측 | 확인한 원인 | 변경 |
| --- | --- | --- |
| 목표 앞에서 명령은 있지만 정지 | DWB가 0.008m/s를 선택. OpenCR 인터페이스의 `int32_t(v * 100)`에서 0으로 변환 | 두 경로 제어기를 RPP로 통일, 최소 접근 속도 0.025m/s. 최종 안전 출력은 증폭하지 않음 |
| 스테이션 3에서 멈췄다가 큰 각도로 재회전 | 20:26 실행에서 충돌 모니터가 약 28초 정지. 기존 Spin의 30초 제한 소진 후 manager가 스테이션 재중심 이동/회전을 요청 | 공통 엔코더 회전, 안전 대기 시간 별도 계산, 실패 후 재중심 이동과 중복 회전 재시도 삭제 |
| 스테이션 1의 Gemini collect 후 집지 않음 | 접근 전에는 전체 바나나가 보였지만 접근 후 bbox 하단이 720px 영상 끝에 닿아 파지 유효성 탈락 | 검증된 몸통 중심을 odom에 잠시 저장하고, 접근 후 현재 마스크·깊이로 같은 중심을 재검증 |
| 스테이션 4 정지 | 0.439m 위치 보정 점프가 안전 한도를 초과해 localization fault | 영상 시각의 TF만 사용. 원시 보정/필터 결과와 엔코더 기록 추가. 안전 한도 유지 |

근거: `cleanup_debug/run_20260905_202621_36808`, 이전 `run_20260905_201838_32916`,
동시간대 ROS 로그. 스테이션 3의 최초 물리적 파지 실패 원인과 충돌을 유발한 실제 물체는
기존 기록만으로 확정할 수 없다. 스테이션 4의 큰 보정도 바퀴 보정, 미끄러짐,
카메라 TF, 마커 배치 중 무엇이 지배적인지 실측이 필요하다.

## 책임 분리

```text
aruco_waypoint_navigator ─┐
                         ├─ /motion/navigate_to_pose ─ robot_motion ─ Nav2 RPP
cleanup_task_manager ────┤
                         └─ /motion/spin ───────────── robot_motion ─ 엔코더 회전
                                                            │
                      /cmd_vel_nav → smoother → collision_monitor → motion_guard → /cmd_vel

cleanup_perception: YOLO/SAM/깊이/물체 UUID/짧은 odom 파지 앵커
cleanup_planner:    Gemini의 제한된 UUID·행동 선택, 직접 속도 명령 금지
pick_and_place:     기존 IK·팔 실행·파지 후 확인 재사용
motion_diagnostics: 읽기 전용 명령/TF/센서 기록
```

- `robot_motion`은 신규 ament_python 패키지다. 두 실행 모드의 **이동·회전 실행**을
  공통화한다. 스테이션 순서, 마커 찾기, 물체 선택은 각 상위 노드에 남는다.
- 기존 마커 재관측의 짧은 BackUp 동작은 Nav2 behavior와 동일한 최종 안전 체인을
  사용한다. 이 동작까지 신규 gateway로 합친 것은 아니다.
- 하나의 이동 또는 회전만 수락한다. 동시에 도착한 다른 요청은 거절한다.
  순회와 정리 서비스를 동시에 실행하지 않는다.
- Nav2 취소 결과가 3초 내 확인되지 않거나 goal 수락 여부가 불명확하면
  `/motion/inhibit`를 latch하여 최종 guard도 정지시킨다. 늦게 수락된 goal도 취소한다.
  상태가 불명확한 채 새 이동을 시작하지 않는다.
- production launch에서 두 노드를 `/motion/*`로 연결한다. C++ 노드의 raw Nav2 기본값은
  독립 테스트 호환용이므로, 실제 사용은 통합 launch를 사용한다.

## 회전과 안전 대기

- 회전은 map yaw가 아닌 연속 odom yaw 변화량으로 닫힌 고리 제어한다.
  스캔 상위 계층은 스테이션 시작 방향 기준의 절대 목표를 유지해 네 번의 잔차를 보정한다.
- 종료: 각도 잔차 ≤0.025rad(약 1.43°), 측정 각속도 <0.03rad/s가 0.25초 유지.
  이는 **엔코더 기준** 정확도이며 실물 각도 보증이 아니다.
- 정상 회전 명령은 최대 0.30rad/s, 최소 비영 속도 0.08rad/s.
  최종 guard의 가속/속도 제한과 충돌 정지는 그대로 적용한다.
- active 회전 30초, 연속 안전 대기 최대 30초, 전체 회전 최대 90초.
  안전 차단 중 명령 의도는 모니터에 전달해 장애물 해제를 검사하지만 바퀴 출력은 0이다.
  안전이 풀리면 **원래 남은 각도**를 수행한다. 제한 초과 시 실패하고 임의 재중심 이동을 하지 않는다.
- Nav2 이동에는 외부 안전 대기/120초 active 제한을 추가한다. Nav2 내부 progress timeout을
  멈추거나 성공으로 바꾸는 것은 아니다. 내부 실패 시 기존 제한된 transit 재시도 정책이 적용된다.
  이동 전체 시간에도 150초 상한을 두어 간헐적인 차단/해제가 대기를 무한 연장하지 못한다.
- guard는 smoothing 이후 명령과 collision monitor 이후 출력을 비교해 정지 원인을 구분한다.
  `collision monitor stopped motion`은 파이프라인 비교에 근거한 추론이다.
  특정 센서/물체가 원인이라는 뜻은 아니다. 상세 points와 시간 정보를 함께 확인한다.
- Humble collision monitor는 stop 상태에서 `stop_pub_timeout` 후 zero 발행을 멈춘다.
  [Humble 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp)을
  확인해 이 값을 3600초로 늘렸다. 입력 스트림 자체가 끊기면 guard의 0.25초 watchdog은 계속 작동한다.
- cleared-area static-map 경로 정책은 유지한다. 센서 장애물 우회 계획은 하지 않지만
  최종 LiDAR/depth 충돌 정지, TF 신선도, map 점프 차단은 해제하지 않는다.

## 잘린 물체의 파지 조건

1. 접근 전 정면 재관측(`249`)을 시작할 때 앵커를 비운다. 그 관측의 전체 SAM 마스크에서
   몸통 중심·깊이가 유효할 때만 중심과 카메라 원점을 odom에 저장한다.
   한 capture 내부의 동일 확인 물체만 갱신하며, 가까운 두 바나나를 공간 거리만으로 합치지 않는다.
2. 접근 후 재관측(`heading_index=250`)에서만 앵커 사용을 허용한다.
3. 같은 클래스, 20초 미만, 카메라 이동 ≤0.15m를 요구한다.
4. 현재 영상에 투영한 중심이 영상 가장자리에서 8px 이상, 현재 마스크 내부에서 4px 이상
   떨어져 있어야 한다. 중심 주변 깊이가 예측과 0.025m 이내이고 불확실성 ≤0.01m이어야 한다.
5. 일치하는 앵커가 정확히 하나일 때만 사용한다. 여러 바나나가 겹쳐 모호하면 거절한다.
6. 부분 관측은 앵커 수명을 연장하지 않는다. 임무 변경과 파지 후 확인(`251`)에서 초기화한다.

YOLO의 프레임별 ID는 동일성 판단에 사용하지 않는다. 기존 map UUID registry는 유지한다.
앵커는 UUID를 대체하는 장기 추적기가 아니라 짧은 접근 구간의 몸통 중심 재검증 수단이다.
중심 자체가 가려지거나 영상 밖이면 끝점을 대신 집지 않는다. 이 경우 객체 실패로 남기며,
자율 카메라 시야 최적화나 보이지 않는 물체의 형태 복원까지 구현한 것은 아니다.

## 기록과 테스트

`motion_debug/run_YYYYMMDD_HHMMSS_PID/telemetry.jsonl`에 4Hz 상태와 이벤트를 저장한다.

- `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel_collision_checked`, `/cmd_vel`
- odom 위치/각도/속도/시각, wheel/arm joint positions, guard 상태
- map→odom, odom→base_link, 계획 경로
- 센서 시각, TF 실패, 반경 0.22m 안 점 개수와 최대 32개 점
- 마커 원시 map→odom 측정 및 필터 이전 값, 이미지 시각, 관측 ID, odom 회전 속도

이 기록기는 목표나 속도를 발행하지 않는다. 센서 점 기록은 샘플링된 진단 정보이며
collision monitor 내부의 정확한 stop polygon/TTC 결정을 재현한 값은 아니다.

회귀 테스트는 별도 localhost ROS domain에서 가짜 센서/엔코더/Nav2를 사용한다.
검사 항목: 네 번의 90° 회전, hardware 속도 정수화, 안전 정지/재개/시간 초과,
동시 요청 거절, 취소 확인 및 미확인 시 차단, clipped 중심/깊이/동일성 조건,
manager의 회전 실패 후 재중심 이동 금지, 실제 collision monitor/guard의 정지.

이번 변경 후 `robot_motion`, `cleanup_perception`, `aruco_localizer`,
`cleanup_task_manager`, `project_bringup`의 선택 빌드와 colcon 시험을 완료했다.
시험 오류/실패는 0이다. Python 패키지의 기본 저작권 템플릿 검사 2개는 skip이며,
flake8/pep257 및 동작 시험은 실행했다. 실물 주행·파지와 실제 Gemini 호출은 수행하지 않았다.

## 실물 검증 순서

빌드 후 기존 프로세스를 정상 종료하고 각 터미널에서 workspace 환경을 다시 source한다.
먼저 물체 없이 순회 시험을 수행하고, 통과 후 바나나 한 개, 마지막에 세 개로 늘린다.

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_bringup project.launch.py
```

별도 터미널에서 환경을 source한 뒤 **둘 중 하나만** 실행한다.

```bash
# 순회 + 각 스테이션 360° 스캔 (Gemini/파지 요청 없음)
ros2 service call /start_station_scan_test std_srvs/srv/Trigger '{}'
# 순회 시험 종료 후 정리 시험
ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'
```

회전은 바닥 기준선/외부 측정 각도와 odom을 비교한다. 엔코더 90°인데 실물이 75°면
controller tolerance 문제가 아니라 바퀴 기구 상수·엔코더 변환·미끄러짐을 조사해야 한다.
고정된 팔 자세에서 정지 전후 마커 추정도 비교해 카메라 TF/marker-map 오차를 분리한다.
증거 없이 wheel separation/radius나 map-jump 안전 한도를 바꾸지 않는다.
실물 충돌 위험이 있거나 localization fault가 발생하면 시험을 중단하고 새 telemetry를 검토한다.

수정 금지 패키지 `realsense_bringup`, `segmentation`, `turtlebot3_manipulation`은 수정하지 않았다.
