# 바닥 정리 태스크 구현 계획

## 1. 목표와 이번 MVP의 범위

최종 목표는 ArUco 마커를 따라가는 로봇이 아니라, 마커로 전역 위치를
간헐적으로 보정하면서 별도의 안전한 스캔·접근·배출 좌표에서 바닥 물체를
정리하는 모바일 매니퓰레이터다.

이번 MVP는 다음 조건으로 제한한다.

- 정리 대상은 `banana` 한 종류다.
- 6개 스캔 스테이션을 `0 -> 1 -> 2 -> 3 -> 4 -> 5` 순서로 방문한다.
- 각 스테이션에서 오른쪽으로 90도씩 네 번 회전하고, 매 회전 후 정지
  상태에서 영상을 수집한다.
- 바나나는 서로 다른 세 스테이션 부근에 하나씩 있다고 가정한다.
- 확인된 바나나는 안전하게 접근해 집은 뒤 하나의 배출 구역에 모은다.
- ArUco 마커 좌표, 로봇 스캔 좌표, 물체 접근 좌표, 배출 좌표는 서로
  독립적으로 관리한다.

이번 MVP에서 VLM은 기본 경로에 넣지 않는다. `banana`가 검출되면 수행할
행동은 항상 `collect_to_drop_zone`이므로, VLM 호출은 결과를 더 좋게 만들지
않으면서 지연·네트워크·비결정성이라는 실패 지점만 추가한다. VLM은 여러
물체 종류와 여러 정리 규칙이 생기는 다음 단계에서 선택적으로 연결한다.

## 2. 현재 코드에서 확인한 사실

### 2.1 이미 사용할 수 있는 기능

- `aruco_localizer_node`
  - ArUco 관측으로 `map -> odom`을 보정한다.
  - 마커가 보이지 않거나 로봇이 회전 중이면 마지막 보정을 고정하고
    오도메트리로 자세를 계속 갱신한다.
- `aruco_waypoint_navigator_node`
  - 마커가 아닌 `map` 좌표의 가상 웨이포인트를 Nav2로 실행한다.
  - 단순 왕복 경로의 검증과 수동 운용에 계속 사용한다.
- `cleanup_task_manager_node`
  - 순찰 정지, 물체 접근, 집기, 배출, 팔 파킹을 순서대로 호출한다.
  - 물체를 선택한 순간의 `map` 좌표를 고정해 `/cleanup/pick_target`으로
    전달한다.
- `pick_and_place`
  - 전달받은 3차원 목표를 팔 좌표계로 변환해 집기 시퀀스를 수행한다.

### 2.2 현재 구조로는 부족한 부분

현재 인식 파이프라인은 다중 물체 목록이나 안정적인 물체 ID를 제공하지
않는다.

- `segmentation_node`는 YOLO 박스 중 신뢰도가 가장 높은 물체 하나만
  EfficientTAM 추적 대상으로 선택한다.
- 첫 검출 이후에는 매 프레임 YOLO를 다시 수행하는 것이 아니라 동일한
  물체의 마스크를 추적한다.
- 외부 출력은 `/object_mask`, `/llm_vid`이며 클래스, 신뢰도, bbox, YOLO
  track ID는 메시지로 발행하지 않는다.
- `realsense_tf_node`는 마스크의 가장 큰 3차원 클러스터 하나를 골라 단일
  `/object_centroid`를 발행한다.
- 현재 `cleanup_task_manager_node`는 첫 centroid를 받는 즉시 전체 순찰을
  중단하며 후보 목록, 중복 병합, 재식별, 스테이션별 360도 스캔을 하지
  않는다.
- 현재 경로는 3개의 가상 주행점으로 구성되어 있으며, 0~5의 스캔
  스테이션 여섯 개가 아직 설정돼 있지 않다.
- `task_zones.yaml`의 배출 위치는 `configured: false`라서 안전 위치를
  교시하기 전에는 정리 태스크가 시작되지 않는다.

따라서 YOLO가 임시로 부여한 ID가 깨지는 문제를 무시하는 것으로는 해결할
수 없다. 지금은 사용할 수 있는 YOLO ID 자체가 외부 인터페이스에 없고,
단일 표적만 전달되기 때문이다.

## 3. 핵심 설계 원칙

### 3.1 마커, 스캔, 행동 좌표 분리

```text
ArUco 마커 좌표       위치 보정 전용
스캔 스테이션 좌표    로봇이 멈춰 360도 관찰할 안전한 base_link 자세
물체 접근 좌표        검출한 물체와 현재 로봇 자세로 매번 계산
배출 좌표             물체를 한쪽에 내려놓기 위한 별도 base_link 자세
```

마커 ID 0~5는 스테이션 이름과 연결할 수 있지만, 마커 중심 좌표를 스캔
좌표로 복사하지 않는다. 스캔 위치는 로봇 회전 반경, 카메라 시야, 팔과 주변
장애물의 여유를 확인해 따로 교시한다.

### 3.2 책임을 노드별로 유지

```text
카메라/깊이
    |
    v
scan_perception_node ---- 원시 관측(ObjectObservation[])
    |                                  |
    |                                  v
    |                         object_registry
    |                     관측 병합·안정 ID·상태 관리
    |                                  |
    +------------------------> cleanup_task_manager
                                 |       |       |
                              Nav2    pick     drop

ArUco 관측 -> aruco_localizer -> map/odom TF
```

- 위치 추정은 `aruco_localizer`만 담당한다.
- 이동과 회전 속도 생성은 Nav2만 담당한다.
- 스캔 시점의 YOLO·깊이 결합은 새 인식 어댑터가 담당한다.
- 중복 제거와 안정적인 물체 identity는 `object_registry`가 담당한다.
- 태스크 순서와 복구 정책은 `cleanup_task_manager`가 담당한다.
- 팔 궤적과 그리퍼는 `pick_and_place`가 담당한다.
- 자유 형식 VLM 출력이 Nav2나 팔을 직접 호출하지 못하게 한다.

`src/realsense_bringup`, `src/segmentation`,
`src/turtlebot3_manipulation`은 저장소 규칙에 따라 수정하지 않는다.

## 4. 권장 실행 전략: 스테이션 단위 스캔 후 즉시 정리

처음부터 0~5 전체를 조사한 뒤 나중에 모든 물체를 집는 방식보다, MVP에서는
각 스테이션의 스캔을 완료한 직후 그 스테이션에서 확인된 물체를 처리한다.

```text
스테이션 0 이동
  -> 360도 스캔
  -> 관측 병합
  -> 해당 스테이션의 확인된 바나나 정리
  -> 스테이션 1 이동
  -> ...
  -> 스테이션 5 완료
  -> 남은 확인 대상 재검사
  -> 임무 종료
```

이 전략을 우선하는 이유는 다음과 같다.

- 조사와 집기 사이 시간이 짧아 물체 위치가 오래되지 않는다.
- 현재 단일 물체 마스크/centroid 파이프라인과 단계적으로 통합할 수 있다.
- 로봇이 움직이는 동안 생긴 오도메트리 오차의 영향을 줄인다.
- 문제 발생 시 어느 스테이션의 인식·접근·집기가 실패했는지 명확하다.

배출 구역까지 왕복 거리가 길어져 전체 이동 시간은 늘어난다. 세 개 바나나로
신뢰성을 검증한 뒤, 2단계에서 전체 조사 후 거리 최적화 순서로 수집하는
모드를 추가한다.

## 5. 360도 스캔 절차

각 스테이션은 다음 절차를 사용한다.

1. 팔이 파킹됐는지 확인한다.
2. 스테이션의 교시된 `map` 자세로 Nav2 이동한다.
3. `/cmd_vel`과 `/odom`이 정지 기준 안에 일정 시간 머무는지 확인한다.
4. Nav2 회전 행동으로 오른쪽 90도를 회전한다.
5. 다시 완전히 정지한 뒤 짧은 프레임 묶음을 수집한다.
6. 4~5를 네 번 반복해 총 360도를 회전하고 원래 방향으로 돌아온다.
7. 네 방향의 관측을 하나의 스테이션 관측 집합으로 병합한다.

한 장만 저장하지 않고 기본 0.5~1.0초 동안 5~10프레임을 사용한다. 최소
3프레임에서 공간적으로 일치하고 유효한 깊이가 있는 후보만 `CONFIRMED`로
올린다. 이렇게 하면 블러, 순간 오검출, 빈 depth 픽셀의 영향을 줄일 수
있다.

회전은 직접 `/cmd_vel`을 발행하지 않고 Nav2의 Spin 행동으로 실행한다.
각 비동기 목표에는 증가하는 goal/session ID를 부여해 이전 회전이나 이동의
늦은 콜백이 현재 상태를 바꾸지 못하게 한다. 회전 중 ArUco 보정이
`ROTATING`으로 거부되더라도 `map -> odom`은 고정되고 오도메트리 자세가
진행되므로 스캔 자체는 계속된다.

기본 파라미터의 시작값은 다음처럼 두고 하드웨어 로그로 조정한다.

| 항목 | 시작값 |
|---|---:|
| 회전 단위 | `-pi/2 rad` |
| 스캔 횟수/스테이션 | `4` |
| 정지 확인 시간 | `0.7 s` |
| 프레임 묶음 | `8 frames` |
| 프레임 내 최소 확인 | `3` |
| 스캔 재시도 | `1` |

## 6. YOLO 인식 인터페이스 개선

### 6.1 새 패키지와 명시적 메시지

보호된 인식 패키지를 고치지 않고 다음 패키지를 추가한다.

- `cleanup_interfaces` (`ament_cmake`)
  - `ObjectObservation.msg`
  - `CaptureObjects.srv` 또는 `CaptureObjects.action`
- `cleanup_perception` (`ament_python` 권장)
  - 스캔 요청이 왔을 때만 RGB·aligned depth·camera info를 묶어 처리
  - YOLO의 모든 `banana` bbox, confidence와 3차원 위치를 반환
  - 디버그 원본/주석 이미지를 실행별 디렉터리에 저장

개념적인 원시 관측 필드는 다음과 같다.

```text
header                 영상 획득 시각과 카메라 frame
observation_id         프레임 관측 고유 ID, 영구 물체 ID가 아님
class_name             banana
confidence             YOLO confidence
bbox                   x_min, y_min, x_max, y_max
centroid               map 좌표의 3차원 중심
position_uncertainty   깊이 분산 기반 불확실도
station_name           scan_0 ... scan_5
heading_index          0 ... 3
image_reference        저장된 증거 영상의 경로 또는 세션 키
detector_track_id      존재할 때만 진단용으로 보존
```

`cleanup_perception`은 bbox 전체의 raw depth 평균을 쓰지 않는다. 바닥과
배경이 섞이지 않도록 bbox 내부 중심 영역이나 마스크에서 유효 depth의
median과 이상치 제거를 사용한다. 카메라 점은 영상 header 시각의 TF를
사용해 `map`으로 변환한다. 회전 직후에도 최신 TF(`TimePointZero`)를
무조건 사용하는 방식은 피한다.

### 6.2 기존 단일 표적 파이프라인과의 관계

- 스캔·후보 목록 생성은 `cleanup_perception`이 담당한다.
- 실제 집기 직전의 세밀한 target은 기존 `/object_mask`와
  `/object_centroid`를 이용해 다시 확인할 수 있다.
- 두 YOLO 모델을 동시에 GPU에 올렸을 때 메모리 부족이나 프레임 저하가
  발생하면, 스캔 detector와 EfficientTAM tracker의 활성 구간을 서비스로
  나누거나 장기적으로 하나의 인식 노드로 통합한다.
- 보호된 `segmentation`을 변경하는 통합안은 유지관리자 승인 없이는
  실행하지 않는다.

바나나가 각 스테이션에 하나씩만 있고 같은 화면에 둘 이상 나타나지 않는
MVP라면, 먼저 기존 `/object_centroid`를 프레임 묶음으로 안정화하는
호환 모드를 구현할 수 있다. 그러나 이 모드는 다중 물체 목록, class 검증,
YOLO confidence를 제공하지 못하므로 최종 구조로 취급하지 않는다.

## 7. YOLO ID 대신 사용할 grounding과 물체 수명주기

YOLO의 track ID는 카메라가 다른 방향을 보거나, 물체가 가려지거나, 노드가
재시작되면 바뀔 수 있다. 따라서 정리 순서와 완료 여부의 기본 키로 사용하지
않는다.

### 7.1 안정 ID 생성과 관측 연결

`object_registry`가 첫 확인 관측에 임무 내부 UUID를 만든다. 이후 관측은
다음 조건으로 기존 UUID와 연결한다.

1. 클래스가 `banana`로 동일하다.
2. `map` 평면상의 거리가 association gate 안이다.
3. 높이와 depth 불확실도가 허용 범위 안이다.
4. 후보가 둘 이상이면 거리, confidence, bbox crop의 appearance 점수를
   함께 사용한다.

초기 association gate는 `0.20~0.30m` 범위에서 하드웨어 측정으로 정한다.
게이트를 지나치게 키우면 가까운 바나나 둘을 하나로 합치고, 지나치게
줄이면 동일한 바나나가 여러 ID로 쪼개진다. 위치 공분산 또는 관측 분산에
따라 게이트를 가변적으로 적용하는 것이 최종 형태다.

YOLO track ID는 있더라도 빠른 단기 연결을 돕는 보조 힌트와 로그로만
사용한다. “ID가 같으니 같은 물체” 또는 “ID가 다르니 다른 물체”라고
단정하지 않는다.

### 7.2 상태 머신

각 물체는 다음 상태를 가진다.

```text
CANDIDATE -> CONFIRMED -> RESERVED -> APPROACHING -> REACQUIRING
          -> PICKING -> CARRIED -> COLLECTED
                              \-> RETRYABLE / FAILED
```

- `CANDIDATE`: 한두 프레임에서만 보인 후보
- `CONFIRMED`: 시간·공간 합의와 유효 depth를 통과한 후보
- `RESERVED`: 현재 처리 대상으로 잠근 후보
- `REACQUIRING`: 접근 후 예상 위치 주변에서 다시 찾는 단계
- `COLLECTED`: 집기와 배출 검증까지 끝난 후보
- `FAILED`: 재시도 횟수를 넘겨 이번 임무에서 제외한 후보

물체 선택 후 도착할 때까지 계속 들어오는 centroid가 active target을
몰래 바꾸지 못하게 한다. 기존 코드처럼 접근에 쓰는 `map` 목표는 잠그되,
집기 직전에는 예상 위치 주변에서 재검출해 새로운 시각의 정밀 target을
발행한다. 오래된 관측의 timestamp만 현재 시각으로 덮어써서 집기에 쓰지
않는다.

### 7.3 집기·배출 확인

그리퍼 서비스 성공만으로 `COLLECTED`를 선언하지 않는다.

- 집기 후 원래 위치를 짧게 재관측한다.
- association gate 안에서 같은 바나나가 계속 보이면 집기 실패로 보고
  제한 횟수만 재시도한다.
- 배출 후 그리퍼 열림과 팔 파킹 성공을 확인한다.
- 배출 구역 반경 안의 바나나는 이미 모은 물체로 취급해 새 작업 큐에
  넣지 않는다.

이 배출 구역 exclusion이 없으면 내려놓은 바나나를 YOLO가 다시 발견해
무한히 집는 루프가 생긴다.

## 8. VLM 연결 원칙

바나나 MVP의 정책은 설정 파일로 고정한다.

```yaml
task_policy:
  allowed_classes: [banana]
  banana: collect_to_drop_zone
  planner: deterministic
```

다중 물체 단계에서만 `planner: vlm`을 선택할 수 있게 한다. VLM에는 전체
ROS 제어권이 아니라 다음 정보만 준다.

- 원본 또는 crop 이미지
- 안정화된 물체 UUID, class, confidence, map 위치
- 배출 구역과 금지 구역
- 사용할 수 있는 제한된 행동 목록

응답은 예를 들어 다음처럼 JSON schema로 제한한다.

```json
{
  "object_uuid": "...",
  "action": "collect_to_drop_zone",
  "priority": 10,
  "reason": "banana on open floor"
}
```

허용되지 않은 action, 없는 UUID, timeout, JSON 파싱 실패는 모두 거부하고
결정론적 정책으로 fallback한다. VLM은 물체의 실제 3차원 좌표를 만들거나
Nav2/팔 명령을 직접 생성하지 않는다.

## 9. 정리 태스크 상태 머신 개편

현재 단일 표적 즉시 반응 상태 머신을 다음과 같이 단계화한다.

```text
IDLE
 -> VALIDATING
 -> NAVIGATING_TO_STATION
 -> SETTLING
 -> ROTATING_90
 -> CAPTURING
 -> MERGING_OBSERVATIONS
 -> SELECTING_TARGET
    -> APPROACHING_TARGET
    -> REACQUIRING_TARGET
    -> PICKING
    -> VERIFYING_PICK
    -> NAVIGATING_TO_DROP
    -> RELEASING
    -> VERIFYING_DROP
    -> PARKING_ARM
    -> RETURNING_TO_STATION
 -> ADVANCING_STATION
 -> FINAL_RECHECK
 -> COMPLETE
```

한 개 물체 실패가 전체 임무를 즉시 종료하지 않도록 실패를 구분한다.

- 임무 중단: localization `DEGRADED`, Nav2 서버 소실, 안전 정지 요청,
  설정 오류
- 물체만 건너뜀: depth 불량, 재검출 실패, IK 불가, 제한된 집기 재시도 실패
- 스캔만 건너뜀: 한 방향 영상 timeout 후 1회 재시도 실패

모든 비동기 Nav2, 회전, 인식, 팔 요청에는 mission/session ID와 operation
ID를 캡처한다. 정지 또는 다음 단계로 넘어간 뒤 도착한 이전 콜백은 상태를
변경하지 않는다.

## 10. 설정 파일의 단일 책임

`cleanup_task_manager/config/task_zones.yaml`을 임무 설정의 중심으로 확장한다.
물리 ArUco 지도는 계속 `new_map_markers.yaml`에만 둔다.

```yaml
cleanup:
  drop_pose:
    configured: false
    x: 0.0
    y: 0.0
    yaw: 0.0
    exclusion_radius: 0.45

  scan:
    turns: 4
    turn_angle: -1.57079632679
    settle_seconds: 0.7
    burst_frames: 8
    min_confirmations: 3

  stations:
    - {name: scan_0, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_1, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_2, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_3, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_4, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_5, x: 0.0, y: 0.0, yaw: 0.0}

  association:
    xy_gate: 0.25
    min_confidence: 0.0  # 모델 측정 후 결정

  task_policy:
    planner: deterministic
    allowed_classes: [banana]
```

0으로 채운 스테이션 값이나 `configured: false`인 배출 위치로는 임무를
시작하지 못하게 validation gate를 둔다. 같은 좌표를 여러 파일에 복제하지
않는다. 기존 `routes.yaml`은 독립적인 자율주행 검증 경로로 유지한다.

## 11. 구현 순서

### 단계 A: 설정과 스캔 전용 주행

- 여섯 스캔 스테이션과 배출 자세를 현장에서 교시한다.
- `cleanup_task_manager` 설정 parser와 validation을 확장한다.
- 정리 manager가 각 스테이션으로 직접 Nav2 goal을 보내도록 변경한다.
- Nav2 Spin action, 정지 판정, 4회 회전, session ID 격리를 구현한다.
- 인식을 연결하지 않고 6곳에서 24회 정지·스캔 이벤트가 정확히 발생하는지
  확인한다.

### 단계 B: 스캔 인식과 월드 모델

- `cleanup_interfaces`와 `cleanup_perception` 패키지를 생성한다.
- 요청 시 프레임 묶음에서 모든 banana 관측과 3차원 `map` 좌표를 반환한다.
- 순수 로직 `object_registry`에 공간 association, 안정 UUID, 상태 전이를
  구현한다.
- 중복 관측, ID 변경, 가까운 두 물체, stale timestamp에 대한 단위 테스트를
  추가한다.

### 단계 C: 한 개 바나나 end-to-end

- 한 스테이션의 바나나를 `CONFIRMED -> RESERVED`로 선택한다.
- standoff 위치로 이동하고 집기 직전 재검출한다.
- 집기, 원래 위치 재확인, 배출, exclusion 처리를 연결한다.
- 실패 시 팔을 파킹하고 안전하게 다음 상태로 복구하는지 확인한다.

### 단계 D: 세 스테이션의 바나나 세 개

- 스테이션별로 하나씩 놓고 전체 0~5 임무를 수행한다.
- 이미 정리한 바나나가 다른 스캔이나 배출 구역에서 재등록되지 않는지
  확인한다.
- 한 바나나의 집기를 실패시켜도 나머지 스테이션 순회가 계속되는지
  확인한다.

### 단계 E: 선택적 VLM과 경로 최적화

- 여러 class와 정리 규칙이 생긴 뒤 schema 기반 VLM planner를 feature
  flag로 추가한다.
- 전체 조사 후 수집, 가장 가까운 물체 우선, 배출 왕복 비용을 포함한 순서
  최적화를 비교한다.

## 12. 테스트와 합격 기준

### 자동 테스트

- YAML에서 중복 스테이션 이름, 비정상 좌표, 미설정 drop pose 거부
- 스캔 회전 횟수와 각도 범위 검증
- 동일 바나나의 여러 관측이 하나의 UUID로 병합됨
- 떨어진 바나나 두 개가 다른 UUID를 유지함
- detector track ID가 바뀌어도 공간적으로 같은 관측은 같은 UUID 유지
- stale 관측과 유효하지 않은 depth 거부
- 배출 exclusion 구역의 관측은 작업 큐에 들어가지 않음
- 늦은 이전 goal callback이 새 mission 상태를 변경하지 않음
- 물체 하나 실패 시 다음 물체/스테이션으로 진행

### 하드웨어 단계별 합격 기준

1. 빈 바닥에서 0~5 이동과 24회 스캔을 완료하고 원치 않는 `/cmd_vel`
   제어권 충돌이 없다.
2. 같은 바나나를 인접한 두 방향에서 보아도 registry에는 한 개만 남는다.
3. 카메라를 돌려 detector ID가 사라지거나 바뀌어도 map 위치로 재연결된다.
4. 집기 전 재검출 좌표가 reachability와 freshness 검사를 통과한다.
5. 세 바나나가 각각 한 번만 배출되고 배출 구역에서 재수집되지 않는다.
6. 최종 상태에 `collected=3`, `pending=0` 또는 명시적인 실패 목록이 남는다.

각 실행은 navigation 로그와 별도로 다음 증거를 남긴다.

```text
cleanup_debug/run_YYYYMMDD_HHMMSS_PID/
├── mission.log
├── object_registry.json
└── scan_0/
    ├── heading_0.jpg
    ├── heading_1.jpg
    ├── heading_2.jpg
    └── heading_3.jpg
```

## 13. 구현 전에 확정할 사항

1. 배출 대상이 바닥의 한 지점인지, 상자·트레이인지와 안전한 base pose,
   물체 release pose를 실제 로봇으로 교시해야 한다.
2. 첫 MVP에서 VLM 호출을 반드시 시연해야 하는지 결정해야 한다. 권장은
   `planner: deterministic`으로 성공시킨 뒤 VLM을 feature flag로 추가하는
   것이다.
3. “세 개 웨이포인트에 바나나”가 각 위치에 정확히 하나씩이며, 같은 카메라
   화면에 바나나 둘이 동시에 들어올 가능성이 없는지 확인해야 한다.

이 세 항목 중 배출 위치는 안전 때문에 구현 시작 전 필수이며, 나머지 두
항목은 위의 권장 기본값으로 진행할 수 있다.
