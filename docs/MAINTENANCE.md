# 유지보수와 리팩터링 검토

2026-09-10 인수인계 검토. 통합 launch, 설정, 주요 C++/Python 실행 경로와 기존 테스트를
대조했다. 이번 변경은 문서 통합과 배치 그림 이동이며 런타임 소스·설정·ROS 인터페이스는 유지했다.

## 검토 결론

패키지별 책임과 공통 기하/정책 모듈은 이미 분리되어 있다. 인수 직전에 전체 구조를 바꾸기보다
운영 문서의 충돌을 먼저 없애고, 이후 설정 중복 → 경로 의존성 → 큰 노드의 책임 순서로
작게 나누는 편이 검증하기 쉽다. 아래는 **검토 결과이며 아직 구현하지 않은 개선안**이다.

| 우선순위 | 코드에서 확인한 대상 | 개선안과 유지해야 할 계약 | 변경 후 확인 |
| --- | --- | --- | --- |
| 1 | `routes.yaml`과 `task_zones.yaml`에 같은 스테이션 6개 중복, `calibrate_waypoints.py`는 routes만 저장 | 공통 스테이션 정의를 두 로더에서 읽거나 명시적 동기화 도구 제공. 물리 마커·일반 경로 코너·배출점은 별도 유지 | 두 소비자의 최종 x/y/yaw와 주행 순서 일치, 기존 파일 호환, config C++ 시험 |
| 1 | `project.launch.py`, `perception_node.py`, manager, diagnostics/video의 `/home/user/turtlebot3_ws` 경로; nav/calibrator는 `Path.home()` 계열 | 워크스페이스/모델/로그 루트를 launch에서 전달하고 노드 기본값은 호환 유지. 모델 경로만 바꾸고 planner 이미지 허용 루트를 빠뜨리지 않기 | 기존 경로와 공백 포함 다른 경로에서 가중치·로그·planner 이미지 접근, launch 해석 |
| 1 | manager `requestPick()`의 표적 토픽 발행 + 200ms 타이머 + Trigger, pick의 전역 최신 표적 수신 | 표적·UUID·관측 시각을 포함하는 요청으로 묶고 장기 실행은 action 전환 검토. 기존 서비스는 adapter 유지 | 표적 유실/교체, 중복 요청, 취소, 부분 파지 실패와 stale 관측. 인터페이스 변경이므로 단순 파일 분리와 별도 작업 |
| 2 | `cleanup_task_manager_node.cpp` 1,807줄에 상태 전이·서비스/action callback·재시도·JSON 기록 혼재 | `NavigationEvidence`/`TaskConfig`처럼 객체 처리와 증거 저장부터 분리. session/operation ID로 오래된 callback을 무시하는 규칙 유지 | `test_mission_flow.py`의 중지·부분 실패·재시도·배출 회귀, 응답이 오지 않는 서비스의 종료 경로 추가 검증 |
| 2 | `perception_node.py` 989줄에 모델 로딩·RGB-D 동기화·UUID·SAM/후보·파일 저장 혼재 | 모델 adapter, capture pipeline, evidence writer로 순차 분리. 이미 분리한 `floor_alignment`, `grasp_candidates`, `object_registry` 재사용 | 음성 2프레임 무저장, 양성 3회 확인, SAM 실패, 좌표/시각 보존, 기록 실패, 후보 재생 |
| 2 | `aruco_waypoint_navigator_node.cpp` 1,647줄에 경로·스캔·마커 재탐색·취소 혼재 | 마커 재탐색과 route 상태를 분리. `robot_motion`에 이미 있는 실행 로직을 새로 복제하지 않기 | 마커 미관측 중 진행, 회전 실패 중단, 늦은 goal 수락 취소, 제한적 BackUp |
| 3 | 두 성능 YAML, launch의 +30mm, 정리/팔의 target age처럼 공유해야 할 값 | 공통값과 프로파일 차이를 분리하되 최종 생성 파라미터 비교를 먼저 확보 | 두 프로파일의 최종 값 비교, guard/smoother/하드웨어 상한과 perception/pick 기하 일치 |

관련 소스: [manager](../src/cleanup_task_manager/src/cleanup_task_manager_node.cpp),
[perception](../src/cleanup_perception/cleanup_perception/perception_node.py),
[navigator](../src/aruco_localizer/src/aruco_waypoint_navigator_node.cpp),
[planner](../src/cleanup_planner/cleanup_planner/planner_node.py),
[설정 로더](../src/robot_motion/robot_motion/performance.py).

## 현장 확인이 남은 사항

| 항목 | 현재 확인 가능한 사실과 다음 확인 |
| --- | --- |
| 운반 중 depth 제거 | 두 프로파일이 `carry_clear_all_obstacles=true`. 운반/파킹 조건에서 외부 장애물 점까지 비움. [설정 설명](CONFIGURATION.md#운반-중-깊이-장애물-처리)에 한계와 제한적 필터 분기를 명시 |
| 지도에 없는 물체 | 현재 costmap은 정적 지도와 inflation만 사용. 동적 장애물 우회가 필요한 장소는 설정/운용 재검토 필요 |
| 전체 임무 성공 | 과거 후보 한 개의 파지/인양 관찰은 남아 있으나 현재 프로파일의 전체 반복 수거 실물 합격을 확인한 것은 아님 |
| 파지 정렬 | +30mm와 카메라 외부 파라미터는 실험 보정. 여러 pitch/물체 배치에서 몸통 접촉과 바닥 여유 확인 필요 |
| ID 4 주변 정지 | 과거 map 원점 차이 오판은 수정됨. 현재 실제 위치 점프·미끄러짐·카메라/마커 오차 가능성까지 해결됐다는 뜻은 아님. 새 실행 로그로 구분 |
| 수거점 | ID 3 바깥 40cm는 가정. 베이스 정지점, 물체점, 벽/적재물 여유를 실측해야 함 |
| 전원·그리퍼 | 과거 개방 정체와 종료 예외가 기록됨. 당시 원인과 현재 장치 정상 여부는 이번 정리에서 재검증하지 않음 |
| 이전 환경 복원 | 가중치/로컬 키/설치 라이브러리와 장치 설정은 Git만으로 완전 복원되지 않음. 같은 컴퓨터의 설치 보존 필요 |

이 중 운반 정책 변경, 새로운 장애물 우회, 속도 조정은 동작 변경이다.
이번 문서 정리에서 자동으로 적용하지 않았으며 실물 확인 범위를 정해 별도로 변경한다.

## 삭제하지 않은 항목

- 세 보호 폴더, 모델 가중치, 지도/보정 파일, 개인 `MEMO.md`.
- `build/`, `install/`, `log/`와 기존 로컬 환경. 재생성 가능해도 현재 인계 장비 실행에 필요하다.
- Python depth 구현과 시험 도구. C++와 비교하는 테스트 및 독립 진단 진입점이 남아 있다.
- `grasp_trial`과 `check_camera_geometry.py`. 후자는 설치되지 않은 실험 도구로 표시했다.
  기본 launch에 없다는 이유만으로 사용하지 않는 코드라고 단정해 삭제하지 않았다.
- LiDAR와 외부 하드웨어 패키지. 현재 기본 분기 외 장치 지원과 원본 기여 규칙을 유지한다.

작업 시작 전에 이미 존재하던 디버그 자료 삭제와 `.gitignore` 변경은 되돌리거나 추가 정리하지 않았다.
루트 `image.png`는 중복 이미지가 아니라 원래 배치 스케치여서
`docs/collection_zone_reference.png`로 내용 변경 없이 이동했다.

## 과거 변경에서 남긴 판단 근거

아래는 이전 문서의 요약이다. `cleanup_debug`, `motion_debug`, `performance_debug`의
당시 원본 자료는 현재 작업 트리에 없으므로 **이번에 재실행하거나 다시 검증한 결과가 아니다**.
필요한 원문은 Git 이력에서 해당 문서를 확인한다.

| 시기/문제 | 남길 설계 이유와 현재 반영 상태 |
| --- | --- |
| 9월 4일 가상 경로 | 물리 마커와 베이스 목표를 분리. 과거 DWB의 25cm 회전 진입/12cm 목표 반경 충돌 분석은 역사적 원인이고 현재는 `StationPath`/`ApproachPath` RPP |
| 9월 5일 모델/API | YOLO 초기 로딩 오류, REST 응답 형식 오류를 수정. 현재 로딩 실패는 빈 스캔 성공이 아니며 API 전송·HTTP 200·fallback 여부를 따로 기록 |
| 짧은 접근의 즉시 성공 | 일반 목표 12cm 허용 오차가 물체 접근을 완료 처리하던 문제. 전용 2cm/0.08rad checker와 실제 팔 TF/IK 기반 접근으로 분리 |
| 명령은 있으나 실물 정체 | OpenCR의 작은 속도 정수 절삭과 open-loop odometry 문제. encoder feedback, RPP 저속 설정, 공통 실행/최종 속도 감시로 대응 |
| 회전 실패 후 엉뚱한 재이동 | 중복 회전/스테이션 재중심 복구를 제거하고 공통 실행기에 대기·취소·회전 제어를 모음 |
| ID 4 보정 fault | 한 기록의 transform translation 차이 0.4555m가 실제 로봇 점 기준 0.0616m였음. 로봇 점에 적용한 SE(2) 차이로 검사하도록 수정 |
| 9월 6일 몸통 파지 | 카메라 보정, 전방 +30mm, 몸통 후보/pitch와 손가락 기하를 반영. 17:41 세션에 사용자 파지 확인과 추가 인양 기록, 전체 임무 성공과 구분 |
| ID 1·2 접근 전 거절 | 주변 바닥과 기준 높이의 약 1cm 차이가 후보 높이를 낮춤. 최대 15mm 상향 평면 정렬, 초기 거절 후 재관측 추가 |
| 처리 지연 2.07초 거절 | 고정 2초 표적 제한 때문에 파지 직전 실패. manager/pick의 관측 시각 예산 6초 통일, 원본 timestamp 유지 |
| 파지 후 운반 정지 | 손/물체 depth를 장애물로 오인한 기록에서 운반 필터가 도입됨. 이후 현재 설정은 전체 depth 점 제거이므로 옛 제한 필터 설명으로 운용하면 안 됨 |
| 종료 시 개방 | 임무 `FINISHING`과 하드웨어 wrapper 종료 순서에 개방 확인 추가. 강제 종료/전원 차단까지 보장하지 않음 |
| 성능 최적화 | 후보 계산 중복/ROI, C++ depth 투영, 최적화 빌드와 대기 개선. 예전 마이크로벤치마크 수치를 현재 전체 임무 시간으로 인용하지 않음 |

## 문서 통합 내역

중복 운영 절차와 더 이상 기본값이 아닌 실험 계획을 제거했다. 날짜별 문서의
상충하는 “현재 상태”를 그대로 archive로 복사하지 않고 현행 설명과 위 판단 근거로 정리했다.

| 이전 문서 | 현재 위치 |
| --- | --- |
| `PLAN.md` | `ARCHITECTURE.md`, `HANDOVER.md`, 이 문서의 미확인 사항 |
| `localization_route_cleanup_architecture.md`, `MOTION_ARCHITECTURE.md` | `ARCHITECTURE.md` |
| `virtual_waypoint_navigation_implementation.md`, `waypoint_calibration_and_navigation_plan.md` | `CONFIGURATION.md`, `HANDOVER.md`, 위 변경 근거 |
| `STATIC_MAP_TEST_MODE.md` | `CONFIGURATION.md`, `ARCHITECTURE.md` |
| `CLEANUP_DEBUGGING.md`, `COLLISION_AND_GRASP_FIX.md`, `NAVIGATION_RETRY_FIX.md` | `DEBUG_LOG_GUIDE.md`, `ARCHITECTURE.md`, 위 변경 근거 |
| `GRASP_AND_LOCALIZATION_20260906.md`, `GRASP_SESSION_20260906.md`, `GRASP_STATION_SKIP_20260906.md` | `GRASP_CANDIDATES.md`, `CONFIGURATION.md`, 위 변경 근거 |
| `CARRY_RELEASE_20260906.md`, `PERFORMANCE_OPTIMIZATION_20260906.md` | `CONFIGURATION.md`, `DEBUG_LOG_GUIDE.md`, 위 변경 근거 |
| 기존 `GRASP_CANDIDATES.md`, `DEBUG_LOG_GUIDE.md` | 경로 유지, 현재 코드/설정과 일치하도록 갱신 |
| `src/project_bringup/README.md`, `src/cleanup_task_manager/README.md` | 패키지 설명과 통합 문서 링크만 유지 |

원문 확인 예시(읽기 전용):

```bash
git log --oneline -- docs/GRASP_SESSION_20260906.md
git show 016429b:docs/GRASP_SESSION_20260906.md
```

`016429b`는 정리 시작 시 HEAD다. 인수 완료 후에는 변경을 검토해 새 커밋을 남기되,
기존 사용자 변경까지 이번 정리의 변경으로 오인하지 않는다.
