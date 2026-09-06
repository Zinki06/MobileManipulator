# ID 1·2 검출 후 파지를 시도하지 않은 원인

분석 대상: `cleanup_debug/run_20260906_192010_47639`.
로봇 명령, 실제 주행/팔 동작, 브링업 재시작 없이 저장 자료로 분석하고 수정했다.

## 실행에서 확인한 사실

- `mission.log:81–84`, `126–129`: 두 물체 모두 YOLO/SAM 확인 및 Gemini
  `collect_to_drop_zone` 선택까지 성공했다.
- 두 번 모두 `GRASP_REACHABILITY_START attempt=0` 직후 `OBJECT_FAILED`.
  접근 명령과 파지 명령은 실행되지 않았다.
- 각 관측의 `.candidates.json`에서 몸통 후보 9개 각각의 pitch 27개가 모두
  `insufficient_body_depth=27`로 탈락했다. IK 도달성 검사를 시작하기 전이다.
- `candidate_body` 실행기와 전방 +30mm 보정은 이번 실행에 적용되어 있었다.
  개선 코드 미적용이나 Gemini의 skip 결정이 이번 두 실패 원인은 아니다.
- 기존 메시지 `no IK-feasible pose`, `after bounded approaches`는 원인을 충분히
  구분하지 못했다. 실제로는 접근 0회에서 몸체 높이 조건 때문에 포기했다.

## 저장 깊이로 확인한 기준 바닥 불일치

마스크 바깥 12–90픽셀 영역의 바닥을 독립적으로 fitting했다.
정수 깊이는 미터로 변환하고, 저장 intrinsics 및 calibrated optical→link1 변환을
적용했다. 선택점은 두 장면 모두 몸통 중앙이며 바닥 픽셀을 선택한 것은 아니다.

| 관측 | 기존 물체 높이 | 관측 바닥의 기준 대비 오차 | 주변 바닥 기준 물체 높이 | 평면 RMS |
|---|---:|---:|---:|---:|
| ID 1, capture 017 | 21.359mm | -11.049mm | 32.407mm | 0.581mm |
| ID 2, capture 026 | 23.152mm | -9.169mm | 32.321mm | 0.548mm |

물체가 실제로 21–23mm 두께라는 판단에는 약 1cm의 바닥 기준 오차가 포함되어 있다.
이는 깊이→팔 좌표계와 알려진 바닥 사이의 불일치 증거다. 카메라 외부 파라미터,
관절 영점, 기구학 또는 실제 바닥 변화 중 어느 항목이 얼마나 기여했는지는
이 두 프레임만으로 분리할 수 없다.

## 코드 변경

- `cleanup_perception/floor_alignment.py`: 근처 바닥의 평면 fitting으로 후보
  높이를 정렬한다. 유효 깊이 ≥70%, inlier ≥80%, 200점 이상, RMS ≤2mm,
  기울기 ≤5°, 후보 양옆의 바닥 지지를 요구한다. 최대 **15mm 위쪽** 보정만
  허용한다. 아래쪽/초과 보정이나 불충분한 바닥은 원래 후보를 유지하고 이유를
  저장한다. 한 물체당 점 수는 최대 4,000개다.
- `candidate_selection.py`: IK 판정 전에 높이 정렬을 적용하고 원시 좌표,
  보정량, 평면 품질을 후보 JSON에 보존한다.
- `perception_node.py`: 실제 검사한 observed link1 점을 촬영 시각 TF로 map에
  변환한다. raw depth 재투영으로 높이 보정이 지워지지 않는다. 전방 +30mm는
  계속 실행기에서 한 번만 적용한다.
- `config/grasp_camera_20260906.yaml`: 프로젝트 실험 보정 프로필에서
  `align_grasp_floor: true`. 일반 perception 기본값은 false이며 기존 외부
  파라미터 수치는 변경하지 않았다.
- `candidate_grasp.hpp`, `pick_and_place_node.cpp`: 실제 표면 높이와 실패 단계별
  카운터를 EvaluateGrasp 응답/미션 로그에 보존한다. 현재 자세와 최접근 자세의
  거절 사유를 구분한다. topic/service/action 인터페이스 변경 없음.
- `cleanup_task_manager_node.cpp`: 처음 접근 후보도 없다고 판정되면 기존
  `/observe_floor`를 통한 **재관측 1회** 후 재판정한다. 반복 실패 시
  `approaches_executed=0; fresh_reobservation=true`처럼 실제 경과를 기록한다.

고정된 물리 바닥 높이, 손가락 목표 바닥 여유 8mm, 경로 여유 6mm,
최소 몸체 진입 깊이 6mm, 관절 한계, 도착 오차 2cm, 신선한 관절/TF 요구는
완화하지 않았다. 높이 보정만으로 파지 실행을 허용하지 않는다.

## 오프라인 재생과 실기 확인 범위

`performance_debug/replay_station_floor.py` 및 `grasp_floor_replay.json`에
기존/변경 높이와 모든 접근 거리의 계획 결과를 저장했다. 저장 관절 상태,
base→link1 장착 오프셋 및 기존 +30mm 보정으로 재생했다.

- 변경 전: ID 1·2 모두 30→18cm 접근 후보 전체 탈락.
- 변경 후: 둘 다 베이스 기준 20cm 앞 접근 후보가 도착 오차 2cm를 포함해 통과.
- 평면 처리 추가 비용은 이 장비에서 한 물체당 약 49–55ms였다.
- 고정 바닥 가정의 계산 검증이다. 실제 손가락 접촉/인양 성공은 다음 사용자
  지시 실행에서 확인해야 한다. 로봇은 이번 수정 작업에서 움직이지 않았다.

다음 실행 시 `floor_z_correction`, `surface_height`, `OBJECT_REAPPROACH_START`,
`TARGET_REACQUIRE_START` 순서를 확인한다. 재관측에서도 보정이 안정적으로
유지되는지, body 후보로 파지·인양하는지 확인한다. 평면 검사 탈락 또는 기존
안전 조건 탈락은 로그에 기록하고 강제 파지하지 않는다.

## 검증 완료

- 전체 `colcon build --symlink-install`: **20개 패키지 성공**.
- perception: **56 passed, 1 skipped**, Python import/syntax, flake8/pep257 포함.
- C++ 파지 기구학 **10개**, 가짜 ROS 컨트롤러 파지/인양 **8개** 통과.
- 미션 가짜 ROS 시나리오 **23개**, task config **5개** 통과.
- project_bringup launch/config 검사 및 관련 lint 통과.
- 변경 3패키지 `rosdep check`: 모든 시스템 의존성 충족.
- 첫 전체 테스트에서 기존 nav_stop 사례의 `/start_cleanup` 응답이 DDS 초기
  연결 timeout으로 유실되었다. 코드 변경 없이 단독 재시험 통과 후 미션 테스트
  전체를 다시 실행하여 23개 모두 통과했다. 실제 중지 기능 회귀로 재현되지 않았다.
- 최종 `colcon test-result`는 4패키지 모두 error/failure 0.
  로그: `performance_debug/grasp_floor_full_build.log`,
  `grasp_floor_colcon_tests.log`, `grasp_floor_manager_retest.log`,
  `grasp_floor_dependencies.log`. 앞선 실패 기록도 보존했다.

## 후속 사용자 요청: 과도한 파지 거절 조건 완화

사용자가 실제 시도를 우선하도록 요청하여 기본 `performance.yaml`을 조정했다.
위 재생 결과는 이 완화 이전의 6mm/2cm 조건으로 검증한 기록이다.

| 조건 | 기존/보수적 프로필 | 새 기본 프로필 |
|---|---:|---:|
| `grasp_min_body_depth`: 최소 몸통 진입 깊이 | 6mm | 4mm |
| `approach_error_margin`: 접근 계획에 추가하는 가상 도착 오차 | 20mm | 0mm |

첫 번째는 파지 품질 조건이다. 원래의 깊은 파지 우선 점수는 유지하면서,
4–6mm 진입 후보도 실행 가능 후보로 받는다. 저장된 이전 거절 좌표
`link1=(.337221,-.047522,-.069773)`는 새 기준에서 pitch -20도,
body depth 5.780mm로 통과한다. 손가락 목표 바닥 여유 8mm와 경로 검사 6mm는
변경하지 않았다. 관절 한계, 신선한 센서/TF, 베이스 정지 및 물체 유지 검사는 유지한다.

두 번째는 실제 위치가 아닌 가정한 2cm 오차 때문에 접근조차 포기하는 검사를
기본 프로필에서 해제한다. 명목 접근 위치는 여전히 완전한 파지·인양 계획을
통과해야 한다. 실제 도착 후 기존 재관측 및 파지 가능성 검사를 다시 수행하고,
실제 위치가 범위를 벗어나면 기존 횟수 제한 안에서 재접근한다.
재접근 목표가 Nav2의 현재 도착 허용 범위 안이고 회전도 필요 없다면 더 가까운
후보를 선택한다. 이미 도착했다고 처리되는 같은 위치에 반복 명령하지 않는다.
`approach_goal_tolerance=0.02`는 실제 Nav2 manipulation checker와 일치시켰다.

perception의 C++ 후보 검사와 실행기에 같은 `grasp_min_body_depth`를 전달한다.
CLI의 기존 표준입력 형식은 유지하고 선택 인자 `--min-body-depth`만 추가했다.
독립 실행 기본값과 `performance_conservative.yaml`에는 6mm/2cm를 유지하여
이전 조건과 비교할 수 있다. 로봇은 움직이지 않았고 다음 브링업에 적용된다.

완화 후 검증: 최종 전체 빌드20패키지 성공. C++ 기구학11개, 가짜 ROS
파지/인양9개(완화 모드와 동일 위치 재접근 방지 포함), perception57개 통과
(copyright1개 skip), project launch/lint 및 motion profile4개 통과.
설치된 CLI에서도 같은 저장 좌표에 대해 보수적 모드 거절/기본 모드 허용을 확인했다.
최종 로그는 `performance_debug/grasp_relaxed_full_build_final.log`,
`grasp_relaxed_final_tests.log`, `grasp_relaxed_colcon_tests.log`,
`grasp_relaxed_replay.json`, `grasp_relaxed_profile_tests.log`에 있다.

## 다음 실행에서 확인한 별도 차단: 관측 처리 시간 제한

`run_20260906_200254_74817`에서는 ID 1 접근 및 도착 후 파지 가능 판정까지
통과했다. 그러나 바로 다음에 `Reacquired pick observation is stale`로 취소했다.
관리 코드 `requestPick()`의 하드코딩2초와 실행기의 기본2초가 남아 있었다.

`scan_station_1_approach/heading_250_capture_018_object_0.npz` 촬영 timestamp는
1788692674.238778이며 거절 이벤트는1788692676.308766이었다. 영상 나이
2.069988초는 인식/SAM/좌표 검사/저장 시간을 포함한다. 계산 가능한 파지를
처리 시간70ms 초과 때문에 시작하지 않은 것이다.

관리 노드/실행기에 동일한 `target_max_age=6.0`을 적용했다. 두 프로필과 독립
노드 기본값까지 통일하고, 촬영 timestamp를 최신 시각으로 바꾸는 우회는 하지 않았다.
현재 관절과 odom 검사도 유지한다. 로그에는 실제 영상 나이와 한도를 기록한다.
실제 좌표·2.07초 지연을 넣은 가짜 ROS 파지/미션 회귀를 추가했다.

검증 완료: 전체20패키지 빌드 성공. 가짜 ROS 파지 실행10개, C++ 기구학11개,
미션25개 및 config5개, project launch/lint 모두 통과. 처리 지연2.07초의
영상 timestamp를 그대로 전달하여 파지 명령·닫기·인양까지 이어짐을 확인했다.
7초 지난 영상 및 미래 timestamp를 최신 시각으로 바꿔 통과시키지는 않는다.
로그: `performance_debug/grasp_age_full_build.log`, `grasp_age_colcon_tests.log`,
`grasp_age_diagnosis.json`. 이 수정 작업에서 실물 로봇 명령은 실행하지 않았다.
