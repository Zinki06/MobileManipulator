# 몸통 후보·pitch 선택과 자동 정리 통합

검출 중심 하나에 고정 pitch로 접근하던 방식 대신, 바나나 몸통 여러 구간을
비교하여 현재 4축 팔이 접근하고 내려갈 수 있는 위치와 pitch를 고른다.
독립적인 손목 yaw를 추가하는 기능은 아니다. 같은 위치에서 yaw를 강제하지 않고,
각 후보의 XY로 결정되는 yaw와 몸통 국소 방향의 차이를 평가한다.

2026-09-06 후속 수정으로 `project_bringup/project.launch.py`의 자동 정리에도
동일한 후보 생성·점수화·IK를 연결했다. 카메라 보정 `grasp_camera_20260906.yaml`,
link1 +X 30mm 보정이 기본 적용된다. perception은 보정 전 관측 몸통점을 전달하며,
30mm는 pick 노드가 도달성 검사와 실행에서 각각 한 번 적용한다.

`candidate_body` 관측만 자동 집기를 허용한다. 후보의 경로가 모두 불가능하면
`candidate_approach`로 표시해 베이스 접근에만 사용하며, 재촬영에서 실행 가능한
몸통 후보가 나오지 않으면 집지 않는다. 기존 얕은 `planGrasp`로 자동 대체하지 않는다.
자동 실행은 열기→상공→10점 몸통 하강→닫기·hold 확인→하강 경로 전체 역순 인양→파킹이다.

`candidate_hover_trial`은 계속 촬영/계획 또는 열린 그리퍼 상공 시험만 수행한다.
독립 실행한 pick 노드는 호환성을 위해 기본 legacy 모드를 유지하므로, 자동 정리에는
위 통합 launch를 사용한다. 새 통합은 저장된 성공 실험 데이터와 모의 컨트롤러로 검증했으며,
수정 후 전체 임무의 실물 성공은 아직 검증하지 않았다.

## 선택과 거절 조건

- 마스크 내부의 유효 깊이를 link1 좌표로 변환한다. 실험 camera extrinsic과
  촬영 timestamp의 wrist TF를 사용하고 기존 perception target과 일치하는지 검사한다.
- 몸통 PCA 투영의 30~70 percentile 구간에서 여러 지점을 만들고, 각 지점의
  닫힘 축 양쪽 경계 가운데를 다시 찾는다. 새로운 픽셀의 깊이를 재측정한다.
- 관측 표면 폭 25~65mm, 양 끝까지 투영 거리 최소20mm 조건을 적용한다.
  폭은 표면 관측값이며 실제 손가락 접촉면/충돌/파지 안정성을 증명하지 않는다.
- pitch -80~-15도를 2.5도 간격으로 탐색한다. 손가락 최저점이 표면 위50mm인
  상공 목표, 현재 관절→상공 경로, 상공→몸통 하강 10개 waypoint를 검사한다.
- 상공 이동은 모델상 손가락이 표면보다 최소45mm 위에 있도록 검사한다.
  하강은 바닥 여유8mm, 관절 한계 여유0.025rad, TCP 표면 아래 최소6mm 조건이다.
  경로 보간에도 바닥 여유를 확인한다. 물체/팔 전체 충돌과 force closure는 미검증이다.
- 몸통 중앙, 적당한 폭, 국소 방향, 깊이 지지, 몸통 진입 깊이와 기존 -45도에서의
  pitch 변화량을 함께 점수화한다. 완전한 접근/하강 계획이 없는 후보는 선택하지 않는다.
- 해가 없으면 실패 이유와 후보들을 저장하고 정지한다. 옛 중심으로 대체하거나
  그리퍼를 닫거나 바퀴를 움직이는 fallback은 없다.

## 실행

워크스페이스에서 ROS 환경과 `install/setup.bash`를 source한다.
기존 bringup, 실험 camera calibration을 적용한 perception, pick 노드가 필요하다.
두 시험 명령을 동시에 실행하지 않는다.

```bash
colcon build --packages-select pick_and_place cleanup_perception --symlink-install
source install/setup.bash

# 현재 카메라 자세에서 새 촬영과 후보 계산만 수행한다. 팔 명령 없음.
OPENBLAS_NUM_THREADS=1 ros2 run cleanup_perception candidate_hover_trial \
  --camera-calibration src/cleanup_perception/config/grasp_camera_20260906.yaml \
  --forward-offset 0.03

# 그리퍼 개방 → 관측 자세 → 새 촬영/선택 → 선택 후보의 상공에서 개방 정지.
OPENBLAS_NUM_THREADS=1 ros2 run cleanup_perception candidate_hover_trial \
  --execute-hover \
  --camera-calibration src/cleanup_perception/config/grasp_camera_20260906.yaml \
  --forward-offset 0.03
```

`--forward-offset` 기본값은0이며, +30mm는 이 세션에서 사용자 관찰로 정한
link1 수평 +X 경험적 보정이다. 새로운 pitch에서 정확성을 재확인해야 한다.
기본 출력은 현재 디렉터리의 `cleanup_debug/candidate_hover_<시간>/`이다.

`result.json`에는 후보별 계획/탈락 이유, 선택 점수, 좌표 변환, 실제 관절값을 저장한다.
`capture.npz`와 `geometry`의 rotation/translation으로 원래 후보 계산을 재생할 수 있다.
`candidate_preview.jpg`의 청록색 원은 전방 보정 전 선택한 몸통 픽셀이다.
완료 시 `hover_rgb.jpg`, `hover_depth.npz`, depth timestamp의 wrist TF도 저장한다.

`candidate_grasp_plan`은 ROS를 시작하지 않는 계산 전용 C++ 실행 파일이다.
표준 입력 한 줄에 `x y surface_z floor_z joint1 joint2 joint3 joint4`를 받아
JSON 한 줄로 상공/하강 계획 또는 탈락 집계를 출력한다. 하드웨어 명령은 없다.

## 검증과 현재 한계

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test \
  --packages-select pick_and_place cleanup_perception
colcon test-result --test-result-base build/pick_and_place --verbose
colcon test-result --test-result-base build/cleanup_perception --verbose
```

테스트에는 몸통 내부 후보, 폭/깊이 거절, 보정 범위, 도달 불가능 후보 제외,
실제 로그 좌표의 상공/하강 바닥 여유 및 거절 사유 회귀가 포함된다.

2026-09-06 실험에서는 촬영 간 표면 높이 추정 약2mm 차이로 후보 통과 여부가
바뀌었다. 따라서 이 조건에서의 해는 검증된 파지 성공이나 오차에 강한 파지를
의미하지 않는다. 실제 파지를 연결하기 전에 여러 프레임/팔 자세에서의 보정 일관성,
전체 손가락·물체 충돌, 접촉 깊이와 유지 여부를 확인해야 한다.
