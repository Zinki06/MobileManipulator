# 몸통 후보 파지와 진단 도구

## 자동 정리에서 쓰는 파지

통합 launch는 `grasp_camera_20260906.yaml`, 몸통 후보/pitch 선택,
link1 +X 30mm 보정을 사용한다. 카메라 보정은 파지 perception에만 적용하며
전역 TF/URDF를 변경하지 않는다. perception이 전달하는 map 표적은 전방 보정 전
관측점이고, pick의 도달성 검사와 실행이 각자 같은 +30mm 보정을 한 번 적용한다.

`candidate_body` 관측만 자동 집기를 허용한다. 실행 계획이 없는
`candidate_approach`는 베이스 접근에만 쓰며, 재촬영에서 실행 가능한 몸통 후보가
나오지 않으면 집지 않는다. SAM 실패 시 bbox depth는 탐색/identity에 사용할 수 있지만
통합 자동 파지의 몸통 검증을 대신하지 못한다.

실행 순서는 개방 → 상공 → 10점 몸통 하강 → 닫기/유지 확인 →
하강 경로 역순 인양 → 파킹/유지 확인이다. 독립 실행 pick의 legacy 모드는
통합 기본 동작과 다르므로 정리 운용은 [통합 실행](HANDOVER.md)을 사용한다.

## 기하와 확인 조건

- 마스크 내부 깊이를 촬영 시각 wrist TF와 보정 카메라 기하로 link1에 변환한다.
- 몸통 PCA 길이의 30~70 percentile에서 후보를 만들고 닫힘 축 양쪽 경계 중앙의
  깊이를 다시 측정한다. 관측 폭 25~65mm, 양 끝 여유 최소 20mm를 요구한다.
- pitch -80~-15°를 2.5° 간격으로 비교하고 상공·하강 전체 경로를 검사한다.
  4축 팔의 후보 XY에 따라 yaw가 정해지며 독립 손목 yaw를 추가하는 기능은 아니다.
- 모델상 손가락 바닥 여유는 목표 8mm, 경로 6mm다.
  최소 몸통 진입 깊이는 기본 프로파일 4mm, conservative/계산 도구 기본값 6mm다.
- `align_grasp_floor=true`는 주변 바닥 평면이 충분히 지지될 때 후보 높이를
  최대 15mm 위쪽으로만 정렬한다. 원시 좌표·보정량·평면 품질은 증거에 남는다.
- 기본 `approach_error_margin=0`은 가상 도착 오차의 추가 검사만 없앤다.
  실제 접근 후 새 표적/IK 검사, 관절/바닥 조건과 접근 횟수 제한은 유지한다.
- 관측의 `target_max_age=6s`는 인식·기하 처리 시간을 포함한다.
  timestamp를 새로 찍어 오래된 관측을 통과시키지 않는다.

깊은 몸통, 폭, 중심부, 국소 방향, 깊이 지지, pitch 등을 함께 평가한다.
모델상 경로가 가능하다는 사실만으로 전체 손가락/물체 충돌이나 안정적인 실제 파지가
증명되지는 않는다. 카메라 y 이동은 바닥 평면 실험으로 보정되지 않았고
+30mm는 특정 배치에서 관찰한 경험적 보정이다.

기존 nominal 모드에는 접근 후 잘린 물체의 중심을 짧게 재검증하는 odom 앵커가 있다.
**현재 보정 모드에서는 nominal 앵커 fallback을 사용하지 않는다.**
중심이 가려졌다고 끝점으로 바꿔 집는 방식으로 해석하지 않는다.

## 진단 도구

ROS 환경을 source하고 필요한 노드가 실행 중인 상태에서 사용하는 개발 도구다.
자동 정리 임무와 동시에 실행하지 않는다. 아래 첫 명령은 촬영/계획만 요청하고,
두 번째는 실제 팔·그리퍼를 움직인다. 도구는 베이스 자동 접근을 수행하지 않는다.

```bash
# 현재 카메라 자세에서 새 촬영과 후보 계산만 수행
OPENBLAS_NUM_THREADS=1 ros2 run cleanup_perception candidate_hover_trial \
  --camera-calibration src/cleanup_perception/config/grasp_camera_20260906.yaml \
  --forward-offset 0.03
```

```bash
# 그리퍼 개방 → 관측 자세 → 촬영/선택 → 상공에서 개방 정지
OPENBLAS_NUM_THREADS=1 ros2 run cleanup_perception candidate_hover_trial \
  --execute-hover \
  --camera-calibration src/cleanup_perception/config/grasp_camera_20260906.yaml \
  --forward-offset 0.03
```

`--forward-offset` 기본값은 0이다. 도구 옵션은 통합 launch의 설정을 자동으로
복제하지 않으므로 같은 조건을 비교하는지 확인한다. 상공 시험은 하강/닫기/수거 완료가 아니다.

| 도구 | 범위 |
| --- | --- |
| `candidate_hover_trial` | 촬영·후보 계산; `--execute-hover`일 때 개방과 팔 상공 이동 |
| `candidate_grasp_plan` (pick_and_place) | ROS 초기화 없는 C++ 계산 전용 CLI |
| `grasp_trial` | 촬영/평가; `--execute`를 주면 실제 단일 파지, 자동 배출 없음 |
| `src/pick_and_place/scripts/check_camera_geometry.py` | 손목 자세를 바꿔 보정 후보를 계산하는 실험 스크립트. 현재 CMake 설치 대상이 아님 |

계산 CLI는 표준 입력 한 줄에
`x y surface_z floor_z joint1 joint2 joint3 joint4`를 받고 JSON 계획/거절 사유를 출력한다.
`--min-body-depth 0.004` 옵션으로 통합 기본 진입 깊이를 지정할 수 있다.

## 결과와 검증 범위

`candidate_hover_trial` 기본 출력은 현재 작업 폴더의
`cleanup_debug/candidate_hover_<시각>/`이다.

| 파일 | 용도 |
| --- | --- |
| `result.json` | 후보 선택/탈락, 변환, 관절과 시험 단계 결과 |
| `capture.npz` | RGB/depth/기하 입력의 재생 자료 |
| `candidate_preview.jpg` | 전방 보정 전 선택 몸통 픽셀 |
| `hover_rgb.jpg`, `hover_depth.npz` | 상공 단계 이후 관측 |

자동 임무의 UUID별 사진·후보 JSON은 [로그 안내](DEBUG_LOG_GUIDE.md)를 따른다.
2026-09-06 세션 문서에는 한 후보의 실제 파지와 추가 인양을 사용자가 확인한 기록이 있다.
현재 통합 프로파일로 여러 물체를 집고 운반·배치하는 전체 실물 성공의 증거는 아니다.
개발 시험 범위와 이번 검사 결과는 [개발 안내](DEVELOPMENT.md)에 있다.
