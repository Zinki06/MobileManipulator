# 스테이션 이동 중 경로 실패 대응 (2026-09-05)

## 확인된 원인

`cleanup_debug/run_20260905_200354_23331/mission.log` 기준으로 0번 스캔은 완료됐다.
1번 목표 `(0, -0.5)`로 이동하던 중 20:04:37에 GridBased 재계획이 실패했고,
기존 BT가 실패를 반환하면서 매니저가 전체 태스크를 종료했다.
이후 `command timeout`은 종료로 인해 속도 명령이 없어졌다는 뜻이다.
20:04:46의 센서 경고는 이 종료의 직접 원인이 아니다.

당시 costmap은 저장되지 않았다. 실제 장애물, depth 오검출, 위치 보정에 따른
장애물 지도 불일치 중 무엇 때문에 경로가 막혔는지는 아직 확정할 수 없다.
이번 변경은 일시적인 실패에 대한 제한적 복구와 원인 확인을 위한 증거 저장이다.
실제 경로가 계속 막혀 있으면 정지하며, 장애물이 없다고 가정하지 않는다.

## 변경된 흐름

스테이션/복귀/수거 장소 이동의 Nav2 결과가 `ABORTED`일 때:

1. 실패 시점의 최신 수신 costmap, 목표, TF 로봇 위치, odom, 안전 상태를 저장한다.
2. 정지 상태에서 기본 2초 대기한다. 직접 속도 명령은 발행하지 않는다.
3. 안전 상태와 위치 정보가 최신이고, odom 속도가 정지 수준인 상태가
   0.5초 유지됐는지 확인한다.
4. 같은 목표로 Nav2를 다시 요청한다. 기본 최대 2회 재시도한다.
5. 준비 상태가 10초 이내에 충족되지 않거나 재시도가 모두 실패하면 종료한다.

센서 단절, TF 이상, `DEGRADED`/안전 FAULT는 우회하지 않는다.
정지 중의 `BLOCKED: command timeout`만 정상적인 대기 상태로 인정한다.
실제 정지 확인에는 `/odom` 타임스탬프와 선속도 <0.015 m/s, 각속도 <0.03 rad/s를 쓴다.
작업 중지 시 대기 타이머를 취소하고 세션/operation ID로 오래된 콜백을 무효화한다.
거절되거나 취소된 목표는 이 재시도 대상이 아니다.
물체 접근과 스캔 재정렬은 기존 별도 실패 처리 정책을 유지한다.

자동 회전/후진, 장애물 지도 강제 삭제, 목표 임의 이동은 추가하지 않았다.
센서 신선도 0.5초와 속도 제한도 완화하지 않았다.

매니저 ROS 파라미터:

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `max_navigation_retries` | 2 | 최초 시도 외 재시도 횟수 (0~3) |
| `navigation_retry_delay` | 2.0 | 재요청 전 최소 대기 시간(초) |
| `navigation_retry_timeout` | 10.0 | 각 대기의 준비 상태 제한 시간(초) |

## 진단 파일과 로그

`cleanup_debug/run_*/navigation_failure_<operation>/`:

- `context.yaml`: 실패 이유, 목표, 로봇 TF, odom 위치/속도, 안전 상태.
- `global_costmap.yaml`, `local_costmap.yaml`: frame, stamp, age, 원점, 해상도,
  가로/세로 및 전체 occupancy 데이터. 수신하지 못했으면 `available: false`.

원본 센서 동시 캡처가 아니라 **실패 시점에 보유한 최신 지도**다. 저장된 age로
얼마나 오래된 지도인지 확인해야 한다. 전체 지도를 놓치지 않도록 Nav2의
`always_send_full_costmap`을 켰다 (global 1 Hz, local 2 Hz).
지도 기록/정지 확인은 `NavigationEvidence` 모듈로 분리했다.

`NAVIGATION_RETRY_WAIT`, `NAVIGATION_RETRY`, `NAVIGATION_FAILURE_EVIDENCE` 이벤트를
mission.log에 기록한다. motion_guard의 통합 경고는 `laser stale`, `depth stale`,
`TF unavailable`, `missing`, `future timestamp`로 구분하고, `MOTION_SENSOR`에는
각 센서 frame/age와 TF 오류를 제한된 빈도로 기록한다.

## 검증과 현장 테스트

격리된 ROS 도메인의 가짜 action/센서로 다음 회귀 시나리오를 검증했다:
일회성 실패 후 완료, 영구 실패의 재시도 한도, 센서 차단/주행 중 재출발 방지,
안전 FAULT, 대기 중 사용자 중지, 지도 증거 저장, 기존 스캔/수거 동작.
실제 collision_monitor/guard 테스트에서도 depth 단절 시 정지가 유지됨을 확인했다.
이 테스트들은 실제 로봇 또는 Gemini API를 실행하지 않는다.

변경 적용에는 기존 브링업 종료 후 재실행이 필요하다.
실물에서 0→1 이동 성공은 별도로 확인해야 한다. 재시도도 실패하면 안전 조건을
풀지 말고 새 `navigation_failure_*` 자료로 경로 차단 원인을 확인한다.
