# 파지 실험 중간 인계 — 2026-09-06

사용자가 권한 모드를 바꿔 재시작하기 위해 중단을 요청했다. **파지 개선은 미완료**이며,
아래 카메라 보정 후보는 아직 런타임에 적용하지 않았다.

## 안전 범위와 종료 상태

- 충전선 연결 상태. ID 0 부근에서만 시험, 허용 이동은 전후뿐이다.
- 이번 실험은 바퀴 명령 없이 팔/손목/그리퍼만 사용했다. 전체 정리 태스크는 시작하지 않았다.
- 브링업: `ros2 launch project_bringup project.launch.py start_cleanup_manager:=false start_cleanup_planner:=false start_segmentation:=false`
- 마지막 손목 진단은 초기 자세 `[0, -0.523, -0.523, 1.5707]`로 복귀했고
  컨트롤러의 성공 응답을 확인했다. 이후 소유한 브링업 PTY에 Ctrl-C를 보냈다.
- 종료 과정에서 perception의 중복 shutdown 예외와 motion_guard의 exit -6이 보였다.
  주행 중 오류가 아니라 종료 시점 오류이며 후속 점검 대상으로 남긴다.
- 보호 폴더 `src/realsense_bringup`, `src/segmentation`, `src/turtlebot3_manipulation`은
  읽기만 했다. 기존 사용자 변경은 보존했다.

## 적용·빌드한 변경

1. `cleanup_perception/depth_geometry.py`: 긴 마스크의 PCA 길이축 양끝을 제외하고
   중앙 몸통의 두꺼운 내부를 선택. 단순 주변 depth 중앙값 대신 중앙 광선에서 평가한
   국소 평면 depth 사용. 한쪽에만 depth가 있는 경우/중앙 결손/과도한 잔차는 거절한다.
2. `pick_and_place/grasp_kinematics.hpp`: 표면 아래 몸통으로
   `min(20mm, 표면 높이의 40%)` 하강한 뒤 기존 최대 12mm 삽입.
   기존 바닥 여유와 모든 IK 경로 검사는 유지했다. **아래 실제 손가락 메쉬 검토를
   완료하기 전 카메라 보정 후 낮아진 목표에 무작정 적용하지 말 것.**
3. `pick_and_place_node.cpp`: `/joint_states`의 실제 손가락 위치를 확인한다.
   개방 목표 오차 1.5mm 이내, fresh 250ms 이내, 400ms 안정 상태를 요구한다.
   닫기/인양/홈 복귀 후 `-0.0085 < q < 0.0175`의 안정된 손가락 간격을 요구한다.
   이 값은 바나나 시험용 보수적 비어 있지 않음 검사이지 힘 센서나 보편적인 파지 증명은 아니다.
   빈손 또는 유지 실패를 `EMPTY_GRASP:`로 반환하며, 복귀 실패는 별도 오류다.
4. `project_bringup/launch/feedback_robot.launch.py`: vendor 파일은 보존하고 생성된
   컨트롤러 YAML에 gripper `goal_tolerance=0.001`, `allow_stalling=true`,
   `stall_timeout=0.7` 적용. 이전 기본 허용오차 0.01m는 1cm 덜 열거나 닫아도 성공이었다.
5. cleanup manager: `EMPTY_GRASP:`를 받으면 제한된 재관측/재시도를 수행한다.
   임무 경로/회전/스테이션 계획은 변경하지 않았다.
6. 증거에 mask/depth/intrinsics/timestamp/선택 픽셀/map 좌표 `.npz` 저장 추가.
7. `ros2 run cleanup_perception grasp_trial`: 촬영/도달 검사만.
   `--execute`를 명시하면 관측 자세→새 촬영→바나나 하나 확인→도달 검사→파지 1회.
   **이 명령은 어떤 바퀴 명령도 보내지 않으며**, 멀면 자동 접근 없이 실패한다.
   성공해도 물체를 자동으로 놓지 않는다.

자동 시험: pick_and_place 10, cleanup_task_manager 24,
cleanup_perception 24(기본 copyright 1 skip), project_bringup 16.
확인한 오류/실패 0. 가짜 컨트롤러에서 정상 유지/빈손/인양 중 놓침/불완전 개방 시험 통과.
실물 시험 명령의 float32 JSON 직렬화 오류는 float 변환으로 수정했다.
새 `check_camera_geometry.py` 진단 스크립트는 실물에서 두 번 완료했으나
별도 자동 단위 시험과 패키지 의존성/설치 정리는 아직 필요하다.

## 실물 시험 증거

이전 순회 실패 분석:
`cleanup_debug/run_20260906_155850_26634/mission.log`
및 `motion_debug/video_20260906_155817_588685/`.
SAM 파지 픽셀은 몸통 중앙이었으나 실제로 끝을 집었다. ID 5 닫기 후 손가락은
`-0.00996m`까지 닫혔는데 바닥에서 안 보인다는 이유만으로 운반을 시작했다.
1→2→1 이동은 2번 수거 지점에 놓고 원래 스테이션으로 복귀하는 기존 정책이었다.

이번 브링업 로그:
`/home/user/.ros/log/2026-09-06-16-22-41-837588-ubuntu-36057/`
`/home/user/.ros/log/pick_and_place_36320_1788679363609.log`
`/home/user/.ros/log/ros2_control_node_36105_1788679362933.log`
`motion_debug/run_20260906_162247_36181/telemetry.jsonl`

- 16:23 첫 촬영: 바나나 화면 아래 잘림 → 거절.
- 관측 손목 +0.20rad: 바나나 전체 중앙 검출 성공.
- 16:26 파지 1회:
  `cleanup_debug/grasp_trial_20260906_162607/station_0_arm_only/`
  목표 link1 `(0.30363, -0.01076, -0.04408)`, 픽셀 `(749,499)`.
  영상: `motion_debug/video_20260906_162248_382820/clip_001_5ae8724c-54cc-43f6-bbff-6d163956bcce.avi`.
  닫기 결과 -0.009043, 이후 약 -0.00918 → 새 검증은 EMPTY_GRASP로 거절 후 복귀.
  사용자는 "이번엔 성공했지만 여전히 끝을 잡는다"고 관찰했다.
  따라서 매우 얇은 끝을 잡아도 현재 간격 기준에서 거절될 수 있음을 구분해야 한다.
  사용자는 이후 바나나를 꺼내 다시 바닥에 놓고 "됐어"라고 알렸다.

## 핵심 발견: 카메라 위치 모델 오차 후보가 두 번 재현됨

기존 `link5 → camera_color_optical_frame` 런타임 TF:
translation `[0.06206036, 0.03223192, 0.03581965]` m.

`src/pick_and_place/scripts/check_camera_geometry.py`는 바퀴 명령 없이
초기 팔 자세에서 손목만 바꾸고, 깊이의 지배적인 바닥 평면과
`base_footprint → link5`를 비교한다. 후보만 출력하고 자동 적용하지 않는다.

1차 `cleanup_debug/camera_geometry_20260906_163231/`:
손목 1.2 / 1.5707 / 1.7707rad.
후보 translation `[0.10151115, 0.03223192, 0.03355809]` m.
height residual 0.000205m, normal residual 0.006400.

2차 `cleanup_debug/camera_geometry_20260906_163512/`:
손목 1.35 / 1.65 / 1.77rad.
후보 translation `[0.10229101, 0.03223192, 0.03408074]` m.
height residual 0.000546m, normal residual 0.004079.
후보 rotation:

```text
[-0.01575923, -0.15802102,  0.98730998]
[-0.99974710,  0.01833360, -0.01302342]
[-0.01604297, -0.98726554, -0.15826998]
```

즉 손목 기준 camera x가 **약 +4cm 더 전방**인 유효 보정값이 반복 측정된다.
중앙을 골라도 실제 손가락은 로봇 쪽 끝을 잡는 현상을 설명할 수 있다.
다만 이는 평평한 바닥/기구학을 가정한 유효 보정이며, x/z와 회전 후보를 검증한 것이다.
**y 이동은 바닥 평면으로 식별할 수 없어 기존 값을 유지했다.**
각 pose의 depth/intrinsics/TF `.npz`, RGB `.jpg`, `measurements.json`이 보존돼 있다.
현재 TF/URDF/파지 좌표 변환에는 **이 후보를 적용하지 않았다**.

## 다음에 할 일 (안전상 중요)

1. 카메라 보정을 파지 perception에만 적용하는 명확한 설정/변환 경계를 설계한다.
   자율주행 TF/마커 보정은 이번 사용자 범위 밖이므로 임의 변경하지 않는다.
   보정된 좌표를 사용하면 기존 GraspAnchors의 nominal camera 역투영도 함께
   일관되게 바꾸거나 보정 모드에서 해당 fallback을 차단해야 한다.
2. **실제 손가락 바닥 여유 검사 보강**: 현재 IK `L4=0.126m`는 URDF의
   `end_effector_link`이지 전체 손가락 메쉬의 최전단이 아니다.
   보호 파일을 읽어 확인한 메쉬:
   `gripper_left_palm.stl` local x max 65mm, joint origin x=81.7mm → 실제 mesh 앞끝 x=146.7mm.
   끝의 z 범위 ±7.8566mm. virtual TCP보다 앞끝이 20.7mm 더 나간다.
   하향 -60°에서 virtual TCP의 z만 12mm 이상이라고 검사하면 실제 메쉬 바닥 여유를
   보장하지 못한다. 카메라 보정 후 표면 높이가 낮아질 수 있으므로, 보정 적용 전에
   이 기하를 포함한 바닥 검사와 필요시 하강/삽입량 제한을 먼저 추가해야 한다.
   위 개선은 아직 코드에 적용하지 않았다.
3. 보정 적용 후 ID 0에서 새 촬영→도달 검사→팔 파지 1회. 바퀴는 우선 그대로 둔다.
   필요하면 사용자 허용 범위 내 극소량 전후 이동만, 회전/순회는 금지한다.
4. 손가락 간격만으로 얇은 끝 파지를 단정하지 않는다. 중앙 접촉 여부/인양 유지 여부를
   영상·사용자 관찰과 함께 확인한다. 현재 wrist 영상에는 실제 손가락 접촉면이 잘 안 보인다.
5. 바닥 깊이 보정 회귀, 실제 메쉬 바닥 여유, 독립 파지 명령에 대한 시험/문서를 보강한다.

GPD는 도입하지 않았다. 현재 증거는 모델 종류보다 카메라-팔 위치 불일치가
우선 문제임을 지지한다. 재시작 후 이 문서부터 읽고 이어서 진행할 것.

## 이어서 진행 — 16:40~16:50, 중심점 시각화 요청으로 동작 중지

### 새 변경과 검증

- `grasp_kinematics.hpp`: 보호 폴더의 손가락 STL은 읽기만 하고, X/Z convex hull을
  독립 헤더에 반영했다. 가상 TCP 대신 실제 손가락의 최저점을 검사한다.
  waypoint 8mm / 관절 선형 보간 검사 6mm 바닥 여유를 요구한다.
  표면이 낮으면 하강/삽입을 제한한다. 이번 낮은 바나나는 삽입 0mm,
  TCP는 관측 표면보다 약 2.4mm 위였으며 실제 손가락은 더 아래에 있다.
- 팔 명령마다 fresh 250ms 이내의 4개 관절값에서 시작하는 경로도 검사한다.
  이는 모델상 손가락-평평한 바닥 검사이며 완전한 물체/팔 충돌 검사는 아니다.
- `cleanup_perception/camera_calibration.py`와 `config/grasp_camera_20260906.yaml`:
  16:35의 후보를 camera optical → link5 → timestamped map 경계에서 적용한다.
  글로벌 TF/주행 카메라 모델은 바꾸지 않았다. 보정 모드에서는 cropped-mask
  anchor fallback을 차단한다. 기본 설정은 nominal이며 실험에서는 별도 YAML을 켰다.
- `project.launch.py`의 `cleanup_camera_calibration_file` 인자로 보정 파일을 선택할 수 있다.
  기본은 설치된 `grasp_camera_nominal.yaml` (보정 비활성)이다.
- 두 번째 손목 sweep 보정을 첫 번째 sweep에 독립 적용했을 때 바닥 높이 오차가
  기존 8.95/22.35/28.26mm → +0.32/+0.21/-0.25mm였다.
  **이 결과는 바닥 일관성 검증이지, 실물 중앙 접촉 보증은 아니다.**
- pick_and_place: colcon 결과 9 tests, 0 errors/failures. 정상/빈손/인양 중 놓침/
  불완전 개방 가짜 컨트롤러 시험 및 실제 손가락 바닥 여유 회귀 포함.
- cleanup_perception: pytest 27 passed, 1 skipped; 패키지 디렉터리에서 flake8/pep257 통과.
  이후 cleanup_perception/project_bringup의 colcon test도 통과했다.
- 첫 빌드 중 소스 편집과 빌드를 겹쳐 링크 실패가 발생했다.
  겹친 빌드가 모두 종료된 뒤 `--cmake-clean-first`로 다시 빌드하여 성공했다.
  루트에서 실행한 pytest lint가 보호/생성 폴더까지 훑은 오류는 파일을 고치지 않고
  패키지 범위로 재실행하여 해결했다. pytest는 로컬 anyio 플러그인 충돌을 피하려고
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 사용했다.

### 실물 결과: 아직 끝 파지

- 16:43 보정 후 중심점 `(860,454)`, link1 `(0.350074,-0.066425,-0.073848)`:
  팔 도달 불가. 전방 4cm 미세 이동을 준비했으나 `FAULT: localization degraded`가
  이미 latch되어 admission에서 중단됐다. **양의 바퀴 속도는 보내지 않았다.**
  안전 장치를 우회하거나 localization 설정을 변경하지 않았다.
- 사용자에게 바나나를 조금 로봇 쪽으로 옮겨 달라고 요청했고 사용자가 옮겼다.
- 16:46 파지 1회: UUID `37ea0c19-baa6-452e-8f18-9e7f6b612b0d`.
  `cleanup_debug/grasp_trial_20260906_164603/station_0_arm_only/`
  중심점 `(845,533)`, link1 `(0.310666,-0.053414,-0.073150)`, pitch -60°.
  닫힘 q=-0.008997m, 인양/초기 자세 복귀까지 유지했지만 기존 -0.0085 임계값으로
  EMPTY_GRASP를 반환했다. 사용자는 **실제로 들어 올렸지만 끝부분을 잡았다**고 확인했다.
  따라서 카메라 보정만으로 중앙 접촉 문제는 해결되지 않았다.
  아직 성공/중앙 파지로 기록하면 안 된다. 간격 임계값도 아직 변경하지 않았다.
- 영상: `motion_debug/video_20260906_164042_535860/clip_001_37ea0c19-baa6-452e-8f18-9e7f6b612b0d.avi`.
  telemetry: `motion_debug/run_20260906_164041_61119/telemetry.jsonl`.
- 16:49 검증한 lift 경로를 역순으로 사용하여 같은 자리에 내려놓았다.
  사용자가 중심점 시각화를 요청해 reset 스크립트를 중지했다. 당시 release와
  인양은 완료됐고, 이미 수락된 홈 복귀도 성공했다. cancel 응답 goals_canceling=[].
  마지막 관절 약 `[0,-0.517,-0.517,1.571]`, gripper `0.019006`, cmd_vel `[0,0]` 확인.
- 사용자가 바나나를 직접 다시 놓겠다고 했으므로 **추가 팔 동작 없이 새 배치 완료를
  기다리고 RGB 검출 중심점 시각화를 우선 진행할 것**.
  마지막 실제 파지 이미지는 위 경로의 `heading_249_capture_003_object_0.jpg`다.

### 남은 점

1. RGB 중심점은 몸통 안에 있지만 실제 접촉은 끝이다. 실제 TCP/접촉 위치의 오차,
   관측 자세와 파지 자세 사이의 extrinsic/기구학 일관성을 추가 검증해야 한다.
2. joint q가 거의 -0.010이어도 물체가 유지될 수 있다. STL의 접촉면 간격은 단순히
   `2*(q+0.010)`이 아니다(손가락 형상에 따른 고정 오프셋 존재).
   빈손 baseline과 실물 접촉 evidence 없이 임계값만 느슨하게 바꾸지 않았다.
3. 진단 `check_camera_geometry.py`의 설치/의존성 정리는 아직 미완료다.
4. 브링업과 별도 perception/pick 노드는 켜져 있고 임무 manager/planner는 꺼져 있다.
   `/tmp/grasp_resume_bringup.log`, `/tmp/grasp_resume_perception.log`,
   `/tmp/grasp_resume_pick.log`, `/tmp/grasp_resume_trial_1.log`,
   `/tmp/grasp_resume_reset_1.log`에 로그가 있다.
   재시작하면 별도 perception은 보정 YAML을 지정해야 한다.

### 16:52 새 RGB 중심점 시각화

- 사용자가 재배치를 마쳤다. 초기 팔 자세의 capture_004는 아래 잘림으로 확정 검출 0.
- 기존 `/observe_floor` 서비스로 손목만 +0.20rad 관측 자세 이동 후 capture_005:
  banana 1개, confidence 0.8694, bbox `[727,412,912,683]`,
  실제 선택 파지 픽셀 **`(848,540)`**, grasp_valid=true.
- 이미지: `cleanup_debug/grasp_center_preview_20260906/station_0_arm_only/heading_249_capture_005_object_0.jpg`
- 파지/바퀴 이동은 실행하지 않았다. 팔은 관측 자세에서 정지 중이다.

### RGB 중심은 맞는데 끝을 잡는 원인: 저장 깊이 재분석 (로봇 동작 없음)

- 실제 파지 capture_003: 중앙 raw depth 374mm, 7x7 범위 372~375mm,
  결손 0/49, 국소 fitting residual 0.594mm.
- 새 preview capture_005: 중앙 raw depth 374mm, 7x7 범위 373~376mm,
  결손 0/49, fitting residual 0.820mm.
- 각 프레임의 마스크 바깥 바닥 평면을 독립 fitting하면, 같은 광선의 바닥 깊이는
  각각 407.89mm / 406.05mm. 바나나 표면은 fitted floor 위 29.99mm / 28.19mm다.
  따라서 선택 픽셀이 바닥을 읽거나 깊이가 튀었다는 증거는 없다.
  평탄성/작은 잔차는 절대 깊이의 systematic bias가 없다는 보장은 아니다.
- 관측 자세에서 깊이 +10mm 오차의 link1 좌표 영향은 대략
  x +4.76mm, y -2.29mm, z -8.93mm다. 몇 cm 길이방향 오차를 작은 depth noise로
  설명하기 어렵다.
- 실제 파지 닫기 시점 encoder FK TCP는 `(0.31259,-0.05358,-0.06754)`.
  저장 프레임에서 25ms 이내 관절값으로 재계산한 surface target은
  `(0.31024,-0.05341,-0.07366)`: 차이는 x +2.35mm, y -0.17mm, z +6.12mm.
  z 차이는 표면 위 2.37mm의 계획상 floor 제한과 수 mm의 제어 추종 오차를 포함한다.
  **encoder FK 일치는 실제 손가락/카메라 장착 모델의 정확성 검증과 다르다.**
- 분석 수치: `cleanup_debug/grasp_center_preview_20260906/depth_diagnosis.json`.
- 우선 검증 대상은 관측 자세→파지 자세 간 카메라/팔 변환 일관성과
  가상 TCP 대 실제 접촉 위치다. 손가락을 연 채 상공 정렬을 확인하는 시험 및
  같은 고정 물체의 두 관측 자세에서 좌표 일관성 검증이 다음 후보다.

### 16:58 열린 그리퍼 상공 정렬 진단 — 사용자 관찰 대기

- 사용자가 손을 뺀 뒤 "준비됐어"라고 확인했다. 파지/닫기/바퀴 동작 없이
  새 촬영→개방 검증→상공 정지 1회만 실행했다.
- 첫 진단 프로세스는 DDS odom 수신 전 KeyError로 종료됐고 로봇 동작은 없었다.
  센서 discovery 대기를 추가한 뒤 다시 실행하여 완료했다.
- 새 pixel `(848,539)`, link1 target `(0.311063,-0.054551,-0.071739)`.
- -60° 피치를 유지하며 실제 finger hull 최저점이 표면보다 50mm 위가 되도록
  가상 TCP는 표면보다 72.219mm 높게 계획했다. 전 관절 보간 경로의 최저
  finger-floor clearance는 79.26mm였으며 기존 FK/path checker를 사용했다.
- 목표 joints `[-0.180423,0.961289,-0.795110,0.881019]`.
  완료 직후 feedback `[-0.182544,0.977146,-0.771592,0.883573]`;
  gripper `0.019006`으로 개방 유지. 컨트롤러 Goal reached 성공.
- **현재 팔은 초기 자세가 아니라 열린 그리퍼 상공 진단 자세에서 정지 중**.
  자동 복귀/하강/파지 명령은 없다. 사용자의 중앙 정렬/끝 치우침 관찰을 기다린다.
- 증거: `cleanup_debug/hover_20260906_165840/result.json`, `hover_rgb.jpg`,
  `hover_depth.npz`, `station_0_arm_only/heading_249_capture_006_object_0.jpg`.
  스크립트 `/tmp/grasp_hover_trial.py`, 검증기 `/tmp/grasp_hover_plan.cpp`.
  로그 `/tmp/grasp_hover_trial.log`.
- 영상 clip_002는 recorder가 이전 UUID를 유지하여 이전 UUID 이름으로 생성했다.
  이 clip은 **새 상공 진단**이며 이전 pick 재시도가 아니다.

### 17:02 상공 진단 관찰 및 자세 간 재투영 불일치

- 사용자는 상공에서도 로봇 쪽 끝에 치우쳐 있어 더 전진해야 한다고 확인했다.
  TF/RealSense 기준점 문제인지 질문했다. 추가 로봇 동작 없이 저장 데이터를 분석했다.
- runtime `use_grasp_camera_calibration=true` 확인. 카메라 점을 그대로 팔 명령에
  넣는 코드는 아니며 optical→보정 link5→map→link1 변환을 거친다.
- 초기/상공 RGB의 바나나 얼룩을 SIFT 상호 일치로 추적했다. 전체 후보 8쌍 중
  바나나 마스크 내부 4쌍이 시각적으로 일치했고, TF 기반 재투영에 비슷한 오차가 났다.
  중앙 부근 동일 얼룩: before `(852.17,537.09)` → observed hover `(967.09,503.42)`.
  현재 보정/기구학 모델의 예측은 `(983.34,827.25)`: 수직 약 324px 차이.
  4쌍의 수직 오차는 -322~-366px. 근거리이므로 이 pixel 수를 직접 cm로 바꾸면 안 된다.
- 최초 전역 feature matching은 다대일 중복으로 퇴화한 homography였다. 이것은
  증거로 사용하지 않았다. 상호 일치 + 바나나 마스크 + 시각 검토 결과만 해석한다.
  마스크 밖 나머지 4쌍은 오대응이 포함되어 보정에 쓰지 않았다.
- 각 얼룩이 움직이기 전과 같은 높이에 있다는 조건으로, 현재 카메라 광선을 그
  높이에 교차시키면 link1 전방 차이는 27.9/29.6/31.0/39.5mm (median 30.3mm),
  좌우 median -9.7mm다. **가정하의 오차 추정이며 새 보정값이나 명령에 적용하지 않았다.**
- 상공 aligned depth는 해당 얼룩 주변이 모두 0이므로, 상공 깊이로 독립 3D 위치를
  검증할 수 없었다. 움직이기 전 깊이와 RGB 대응/관절 TF를 이용한 재투영 검사다.
- 결과는 사용자의 약간 더 앞으로 가야 한다는 관찰과 일치하며, 카메라/팔 기하
  모델이 서로 다른 팔 자세에서도 일관되지 않는다는 증거다. 카메라 장착 extrinsic,
  관절 영점/기구학, 실제 그리퍼 기준점 중 어느 것이 원인인지는 아직 분리하지 못했다.
  이전 wrist-only floor calibration은 한 shoulder/elbow 자세에서만 바닥 높이를 맞춘
  유효 보정이므로, 이 extended-arm 위치에서 정확성을 보장하지 않는다.
- 파일: `cleanup_debug/hover_20260906_165840/reprojection_check.json`,
  `mutual_correspondences.jpg`. 분석 `/tmp/check_hover_reprojection.py`.
- 팔은 여전히 열린 상공 자세에서 정지 중. 전진/하강/닫기/홈 복귀는 수행하지 않았다.

### 17:07 사용자 요청: 그리퍼 중심 목표 전방 +30mm 상공 재시험

- 사용자가 양손가락 사이를 중심으로 전방 3cm 보정, 초기 자세 복귀 후 상공 정지를 요청했다.
- `/park_arm` 성공 후, 전체 물체 촬영에 필요한 기존 `/observe_floor` 자세로 이동했다.
  새 검출 pixel `(851,534)`, 관측 link1 target `(0.312927,-0.055959,-0.070983)`.
- **실험용 파지 목표에만 link1 수평 +X 0.030m 추가**:
  `(0.342927,-0.055959,-0.070983)`.
  기울어진 손목 로컬 X 방향 삽입이나 카메라 TF 변경이 아니다.
  실사용 pick 노드/글로벌 TF 기본값에 아직 이 경험적 보정을 적용하지 않았다.
- 기존 -60°로는 이 x/높이 조합이 IK 범위 밖이었다. -60→-45°를 2.5° 단위로 검사해
  도달 가능한 -45°로 변경했다. 이 각도 변경은 사용자에게 동작 전에 안내했다.
- 손가락 hull 최저점이 표면보다 50mm 높게 계획했다(TCP는 71.19mm 위).
  현재 관절→목표 선형 경로의 finger-floor 여유 80.02mm 검증.
- 목표 joints `[-0.167512,1.097932,-0.902015,0.589481]`;
  완료 직후 actual `[-0.168738,1.115204,-0.877437,0.593651]`.
  모터 추종 오차와 기구학 모델 오차가 있어 계획상 5cm는 실측 높이 보증이 아니다.
- 컨트롤러 Goal reached 성공, gripper `0.019006`, base_delta `[0,0,0]`.
  **현재 보정한 상공 자세에서 개방 정지 중이며 닫기/추가 하강/자동 복귀 없음.**
- 사용자 중앙 정렬 관찰 대기. 두 자세 비교 시 forward offset뿐 아니라 pitch도
  -60→-45° 바뀌었음을 반드시 고려할 것.
- 증거: `cleanup_debug/hover_forward30mm_20260906_170719/result.json`,
  `hover_rgb.jpg`, `hover_depth.npz`, `trial_script.py`, `hover_plan.cpp`.
  로그 `/tmp/grasp_hover_3cm_trial.log`, `/tmp/grasp_hover_3cm_park.log`.

### 17:09 사용자 정렬 확인 및 비스듬한 배치 준비

- 사용자는 +30mm 보정 상공 자세를 보고 "좋아 그상태에서 잡으면 될 것 같아"라고 확인했다.
  실제 닫기/파지는 아직 시행하지 않았으므로 성공 파지로 기록하지 않는다.
- 사용자가 바나나를 비스듬히 놓기 위해 우선 초기 자세 복귀만 요청했다.
- `/park_arm` 성공, 로그에서 initial safe pose 복귀 완료 확인.
  그리퍼 개방 유지, 바퀴 이동/파지 없음.
- **현재 팔은 초기 자세에서 정지 중**. 사용자의 비스듬한 배치 완료 신호를 기다린다.
  이후에는 새로운 마스크/중심/장축 방향을 재관측해야 하며 이전 목표를 재사용하지 않는다.

### 17:10 비스듬한 배치 관측 — 아직 파지하지 않음

- 사용자가 비스듬한 배치를 마치고 이 경우의 파지 전략/더 가까운 접근 필요성을 질문했다.
  `/observe_floor` 성공 후 capture만 실행했다. 파지/바퀴 이동 없음.
- 이미지 `cleanup_debug/diagonal_grasp_preview_20260906/station_0_arm_only/heading_249_capture_008_object_0.jpg`.
  pixel `(800,533)`, confidence 0.9032, full-mask valid, depth 약 376.09mm.
- timestamp 근접 관절값/기존 실험 extrinsic으로 계산한 link1 target은
  `(0.314629,-0.035268,-0.072171)` (근사 FK이며 live TF 값과 수 mm 차이 가능).
- depth surface footprint PCA: 몸통 장축 약 +50.19°, +30mm 보정 목표의 팔 yaw 약 -6.05°;
  장축 대 접근 방향 약 56.24°. 현재 계획은 object orientation을 입력받지 않고
  target x/y에 따라 joint1 yaw만 정하므로 방향 변화에 적응하지 않는다.
- 접근 방향으로 중심 ±5mm 단면에서 닫힘 축 투영 폭은 약 47.67mm.
  현재 선택점 양쪽의 범위는 -35.45/+12.22mm로 비대칭이다.
  global mask centroid/PCA 중앙영역 선택과 jaw-closing 방향 단면 중심은 다를 수 있다.
  전체 마스크 닫힘 축 투영 폭 122.29mm는 전체 길이 영향이므로 필요한 파지 폭으로
  오해하면 안 된다. 국소 표면 footprint만으로 완전한 finger collision을 증명할 수 없다.
- URDF 관절: joint1 Z 회전, joint2~4 Y 회전, 양손가락 ±Y 병진.
  동일 위치에서 gripper yaw를 독립적으로 맞추는 손목 관절은 없다.
  방향을 바꾸려면 목표/로봇 접근 위치의 조정이 필요할 수 있으며,
  전후로만 더 가까이 가는 것이 방향 정렬 문제를 자동 해결하지 않는다.
- 다음 후보: jaw-closing 방향의 양쪽 경계 사이로 중앙 점을 조정하고,
  사용자가 확인한 +30mm 보정을 유지한 상공 정렬→여유를 검증한 하강.
  아직 위 방향 기반 target 변경/자동 파지는 적용하지 않았다.
- 분석 `/tmp/analyze_diagonal_grasp.py`, 증거 `orientation_analysis.json` (위 capture 폴더).
  현재 팔은 관측 자세에서 개방 정지 중.

### 17:18 비스듬한 바나나 — 단면 중심 +30mm 보정, 개방 상공 정지 완료

- 사용자 요청: 이번에도 잡지 않고 이전과 같이 5cm 앞에서 정지. 이전 실험과 동일하게
  **손가락 hull 최저점이 관측 표면보다 계획상 50mm 위**인 hover로 수행했다.
- 실험용 `cleanup_perception/jaw_geometry.py` 추가. 현재 접근 yaw에 수직인 닫힘 축의
  표면 경계(접근 방향 ±5mm 밴드, 2/98 percentile) 가운데로 후보를 옮긴 뒤,
  마스크 내부의 새 pixel/depth를 재추출한다. 제한 범위/깊이 지지 없으면 중단.
  실사용 perception/pick 경로에는 연결하지 않았고 이번 hover script에서만 사용했다.
- 첫 시도 `hover_diagonal_forward30mm_20260906_171806`는 이동 전 Time clock type
  불일치로 중단. 관측/개방만 수행했으며 hover action은 전송하지 않았다.
  acquisition Time을 ROS node clock type으로 수정한 뒤 새 촬영으로 재시도했다.
- 최종 capture 원래 pixel `(801,530)` → 단면 중심 `(831,532)`, depth 371.0mm.
  link1 original `(0.314185,-0.035595,-0.072105)` → 단면 중심
  `(0.311099,-0.046994,-0.068640)`; 닫힘 축 후보 보정 -12.44mm,
  새 깊이 반영 최종 y 변화 -11.40mm. 단면 표면 폭 약 47.11mm.
- acquisition timestamp의 `link1 <- link5` TF와 실험 camera calibration으로 계산했다.
  원래 target은 perception centroid를 link1으로 변환한 값과 수치상 일치 확인.
- 이전 사용자 승인한 link1 수평 +X 30mm 추가 후 최종 target
  `(0.341099,-0.046994,-0.068640)`. +30mm는 카메라 TF/production pick에 넣지 않았다.
- 최종 재촬영 위치에서 도달 가능한 pitch **-45°** 선택. TCP 표면 위 71.19mm,
  손가락 최저점 표면 위 50mm, 계획 경로 최소 바닥 여유 82.36mm.
  목표 joints `[-0.141838,1.042996,-0.810268,0.552670]`,
  실제 `[-0.142660,1.059981,-0.790000,0.556835]`.
  기구학/추종 오차가 있으므로 50mm는 계획값이며 실측 보장은 아니다.
- controller 성공, gripper 좌우 `0.019006`, base_delta `[0,0,0]`.
  **현재 비스듬한 바나나 상공에서 그리퍼 열린 상태로 정지. 닫기/하강/복귀 없음.**
- 증거: `cleanup_debug/hover_diagonal_forward30mm_20260906_171829/result.json`,
  `jaw_target_preview.jpg`(cyan은 +30mm 전 단면 중심), `hover_rgb.jpg`,
  `hover_depth.npz`, `trial_script.py`, `hover_plan.cpp`, `jaw_geometry.py`.
  실행 `/tmp/grasp_hover_diagonal_trial.py`, 로그 `/tmp/grasp_hover_diagonal_trial.log`.
- 검증: cleanup_perception package pytest 전체 **30 passed, 1 skipped**;
  새 단면 중심 회귀 테스트 3개 및 flake8/pep257 통과.
- 다음: 사용자가 실제 양손가락 사이와 바나나 몸통 정렬을 평가한 후 후속 동작.

### 후속 사용자 평가: 위치 정렬 확인, orientation 개선 필요

- 사용자는 비스듬한 배치 hover의 위치 정렬은 맞다고 확인했으며, 바나나 긴 방향에 맞춰
  gripper RPY도 회전해야 하지 않느냐고 질문했다. 파지 성공을 의미하지 않는다.
- URDF와 IK 재확인: joint1 Z yaw, joint2~4 Y pitch; 독립 wrist yaw/roll 없음.
  TCP 위치를 유지한 pitch 조절은 IK 도달 범위 내 가능하지만 고정 XY에서 yaw는
  atan2(y,x-0.012)로 제한된다. joint1만 돌리면 그리퍼 중심도 원호로 움직인다.
  현 목표 회전축 반경 약 0.33244m에서 yaw 5도 변경 시 중심 변위 약 29mm.
- 긴 몸통에 손가락 길이 방향을 맞추고 닫힘 축을 몸통 가로 방향으로 두는 것이
  검토할 방향 조건이다. 직전 trial은 단면 중심만 조정했으며 orientation 정렬은 아님.
- 고정 목표의 yaw 개선에는 로봇 접근 위치 재배치(단순 제자리 회전과 다름),
  선택 파지 지점 변경 또는 물체 재배치 등이 필요. 이번 질문에 따른 동작 명령 없음.
  현재 기존 hover 자세에서 개방 정지 유지.

### 개선 방향 검토 — 물체 정렬 강제 대신 파지 후보/접근 위치 선택

- 사용자가 물체를 항상 가지런히 놓아야 하는지, 개선 방법을 요청했다.
  실제 팔/바퀴는 움직이지 않고 직전 capture로 오프라인 후보 평가를 수행했다.
- 몸통 PCA 투영 분포의 20~80 percentile에서 5 간격으로 13개 seed 생성,
  각 seed를 jaw-closing 단면 중심으로 재조정하고 기존 +X30mm 보정 적용.
  pre-motion 관절로 회전 근사, 저장된 original target으로 translation 복원;
  정확한 새 캡처/실물검증을 대신하는 결과가 아니다.
- 보이는 단면 폭 약 35.9~94.9mm. 35/40 percentile 몸통 후보는 45.4/47.9mm,
  기록 pixel `(851,548)` / `(842,538)`.
- -80~0도 pitch, 2.5도 간격으로 hover 및 하강 IK/관절 보간 바닥 여유 검사.
  floor link1=-.101m 가정; 하강 TCP는 표면 아래 min(20mm,몸체높이*0.4)를
  목표로 하되 finger-floor >=8mm로 제한하고 표면 아래 최소4mm 진입 조건 사용.
  첫 시도 고정 하강 깊이는 바닥 여유를 충족하지 못했으므로, 기존 production
  planGrasp의 바닥 여유에 따른 깊이 제한과 같은 방식으로 수정하여 재계산했다.
- 이 제한된 검사에서 13개 중 7개 후보(20~50 percentile)에 가능한 pitch 존재.
  예: 35 percentile 첫 pitch -37.5도, 40 percentile -40도.
  55~80 percentile은 탐색한 동일 pitch hover→하강 경로에서 해 없음.
  **객체/팔 충돌, 접촉 법선, 진짜 접촉 폭, 미끄럼, force closure는 미검증**.
  따라서 성공 파지/완전 충돌검증 결과로 표현하지 않는다.
- 제안: 곡선 몸통의 여러 지점에서 국소 방향/폭/양쪽 접촉 가능성/끝까지 거리/
  도달성과 하강 여유를 평가해 파지 위치와 pitch를 함께 선택. 장축 하나를
  무조건 맞추는 대신 현재 팔이 가능한 yaw에서 안정적인 후보를 선택한다.
  전방30mm 보정은 경험값이므로 pitch 변화 시 재확인 필요.
- 후보가 모두 부적합하면 base 접근 위치를 물체 주변으로 재배치하여 yaw 제약 해결.
  단순 제자리 회전만으로는 고정 물체에 대한 그리퍼 방향을 자유롭게 정할 수 없다.
  현재 충전선/기존 이동 범위/localization fault 상태에서 이를 실행하지 않았다.
- 장기적으로 독립 손목 회전 추가 또는 더 많은 자유도의 팔은 하드웨어 대안이나,
  우선순위는 기존 하드웨어에서 후보 선택/깊이/보정을 검증하는 소프트웨어 개선.
- ROBOTIS 공식 스펙: 4 arm DOF + gripper 1 DOF, gripper stroke 20~75mm.
  https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/
  보이는 표면 폭과 공식 stroke 비교만으로 실제 수용/접촉 성공을 보장하지 않는다.
- 결과/재현 소스는 직전 hover 폴더의 `candidate_feasibility.json`,
  `analyze_grasp_candidates.py`, `evaluate_grasp_candidate.cpp`에 저장.
  현재 실제 그리퍼는 기존 hover에서 개방 정지 유지.

### 17:25~17:36 후보 위치 + pitch 선택 구현 및 실물 상공 시험

- 사용자 "개선해" 요청으로 재사용 가능한 후보 생성/경로 계산/실물 hover 명령 구현.
  변경 범위: `cleanup_perception`, `pick_and_place`, 문서. 보호 폴더 수정 없음.
- `grasp_candidates.py`: 몸통 30~70 percentile의 여러 단면, 닫힘 축 중심 재측정,
  표면 폭25~65mm와 끝까지 최소20mm 조건, 국소 방향/깊이/중앙성 점수.
- `candidate_grasp.hpp` + 설치 실행파일 `candidate_grasp_plan`: pitch -80~-15도 탐색,
  finger hull 기준 상공50mm, 접근 전체경로 표면여유45mm, 하강 바닥여유8mm,
  관절한계여유0.025rad, nominal TCP 표면 아래 최소6mm 진입 조건.
  같은 pitch로 상공→몸통까지 하강10 waypoint 검증. 실패원인 집계 제공.
- `candidate_hover_trial` 새 ROS 실행 명령에 위 두 모듈 연결. 기본은 촬영/계산만,
  `--execute-hover`일 때 개방→관측→새촬영→선택 상공 waypoint만 실행한다.
  **닫기/하강/운반/바퀴 명령은 없다. 기존 자동 수거 pick 경로는 변경하지 않았다.**
  camera calibration과 +X30mm는 명시적 CLI 입력. TF/센서/목표 freshness 검증.
- 원래 17:18 capture 재생: 필터 통과8개 중 하강 가능한 후보4개(최종 여유 조건),
  선택 `(842,538)`, pitch -20도, 표면 아래 모델상7.389mm.
  재생 자료는 근사 pre-motion 회전이므로 실제 동작 목표로 재사용하지 않았다.
- 첫 live `candidate_hover_20260906_173215`: 관측/개방 후 새 촬영에 완전한 경로가
  없어 정지. 상공 action 전송 없음. 표면 높이가 이전 관측과 약2mm 달라진 조건.
  이 결과를 우회하지 않았고 바닥/관절 여유나6mm 진입 기준을 완화하지 않았다.
- 읽기 전용 재촬영 `candidate_hover_20260906_173419`: 팔 명령 없음.
  `(829,545)`, pitch -20도 후보 통과. 촬영 사이 약2mm 높이 변화로 통과 여부가
  바뀌므로 현재 추정치 근처의 파지 가능성은 오차에 민감하다.
- 두 번째 live `candidate_hover_20260906_173447`:
  - 새 capture pixel 원래 `(799,541)` → 선택 `(819,532)` (body percentile50).
  - depth371.691mm, 보이는 단면 폭40.679mm, 끝까지 투영거리52.055mm.
  - observed link1 `(0.312491,-0.042243,-0.067938)`;
    수평+X30mm 후 `(0.342491,-0.042243,-0.067938)`.
  - 선택 **pitch -22.5도** (기존 -45도보다 수평에 가까움), yaw 약-7.28도.
    국소 방향과 접근 yaw의 차이는 약40.16도로 남아 있다. yaw 정렬 완료가 아니다.
  - 계획상 finger 최저점 표면 위50mm; 접근 경로 최소바닥여유83.062mm.
    계산만 한 하강 끝점은 표면 아래 nominal TCP7.090mm. 실제 하강/닫기는 안 했다.
  - 목표 joints `[-0.127130,0.993590,-0.313303,-0.287587]`;
    완료 실제 `[-0.127320,1.009359,-0.309864,-0.282252]`.
  - controller 성공, `hover_complete=true`, gripper0.019006, base_delta `[0,0,0]`.
  - 완료 후 RGB 저장 단계에서 stale sensor 오류. 이 기록 오류로 프로세스는 exit1이지만
    **동작 자체는 이미 성공했고 새 상공에 개방 정지**했다. 원본 result.json에 둘 다 기록.
- 기록 오류를 동작 실패와 구분하도록 수정. 완료 후 카메라 freshness 최대3초 대기,
  기록 실패는 `evidence_error`로 남기며 동작 재시도를 유도하지 않도록 처리.
  해당 동작/기록 분리 회귀 테스트 추가. 실제 팔 재이동은 하지 않았다.
- 영상 복구 `candidate_hover_20260906_173613`: HoverTrial.run() 호출 없이 센서/TF만
  읽고 `hover_rgb.jpg`, `hover_depth.npz`, wrist TF 저장 완료.
  실제 joints `[-0.127320,1.009359,-0.308330,-0.280718]`로 같은 상공 유지 확인.
- 최종 검증: 두 패키지 빌드 성공. pick_and_place **13 tests, 0 failures**;
  cleanup_perception 최종 pytest **36 passed, 1 skipped**, flake8/pep257 통과.
  새 테스트는 후보 폭/깊이/끝 회피/도달성 필터, 로그 경로 여유/탈락 사유,
  기록 오류와 동작 성공 분리를 포함한다.
- 사용법/제한 `docs/GRASP_CANDIDATES.md`. 로그
  `/tmp/grasp_candidate_live_trial_second.log`, `/tmp/grasp_candidate_final_tests.log`,
  `/tmp/grasp_candidate_final_python_tests.log`.
- **현재 팔은 새 pitch -22.5도 계획의 상공에서 개방 정지. 파지 성공은 미검증.**
  다음에는 이 자세의 실제 몸통 정렬/기울기를 사용자와 확인하고, 하강을 연결하기 전에
  2mm 수준의 관측 변화/보정 일관성과 전체 물체-손가락 충돌을 검증해야 한다.

### 17:40 사용자 승인: 초기 복귀 후 새 후보로 1회 파지 및 짧은 인양

- 사용자가 "한번 초기 상태 갔다가 잡아봐"라고 실제 파지를 승인했다.
- `/tmp/candidate_pick_once.py`는 새 후보 hover 경로를 호출한 뒤, 검증된 하강
  waypoint만 실행하고 닫기→안정된 encoder hold 확인→하강 일부 역재생 인양으로 끝낸다.
  바퀴 이동, 운반, 파지 재시도, 잡은 상태의 자동 초기 복귀 없음.
- 최초 17:39:37 프로세스는 odom discovery 이전 조회로 동작 전 중단.
  기존10초 센서 discovery 방식으로 수정. 실제 초기 복귀/파지는 다음 시도1회만 수행.
- `candidate_hover_20260906_173957`:
  `/park_arm` 성공, 관측 자세 이동 후 새 capture014.
  선택 pixel `(839,543)`, body percentile45, 표면 폭37.062mm,
  depth364.052mm. observed `(0.302793,-0.048629,-0.066933)`;
  +X30mm 후 target `(0.332793,-0.048629,-0.066933)`.
  새 관측에 따라 pitch **-17.5도**, 모델상 body depth9.178mm로 선택됨.
- 상공 controller 성공 후, 최신 관절에서 하강경로를 다시 C++로 검사.
  하강4초 동안 주기적으로 최신 관절/odom 및 finger-floor 여유6mm 이상 감시.
  실제 하강 완료 floor 여유 모델상14.819mm, 관절
  `[-0.150330,1.494097,-0.590583,-0.622796]`.
- `/close_gripper` 성공. 기존 hold 조건(-0.0085<q<0.0175, 안정0.4초 이상,
  새 timestamp4개 이상)을 완화 없이 적용하여 close 후 통과.
- 검증된 descent 마지막3구간 역재생으로 nominal약22.8mm 짧은 인양 실행.
  실제 FK 손가락 높이는 종료값 비교시 약15.6mm 상승; 모델/추종 오차가 있다.
  인양 후에도 encoder hold 조건 통과.
- final gripper `-0.00791534`, final joints
  `[-0.153398,1.334563,-0.404971,-0.627398]`.
  final base_delta 약 `[-8.4e-9,-2.9e-9,7.1e-15]` (수치오차 수준).
- 결과 `ENCODER_HOLD_AFTER_SHORT_LIFT`. **물체의 실제 들림/몸통 접촉 위치는
  사용자 시각 확인 대기이며, encoder만으로 안정적인 몸통 파지 성공을 단정하지 않는다.**
- 현재 **그리퍼 닫힌 상태로 짧게 인양한 위치에서 유지 중**.
  물체 유지 가능성이 있으므로 무조건 open/park/관측 자세로 이동하지 말 것.
- 증거: 위 root의 result.json, pick_trial_script.py, check_candidate_motion.cpp,
  candidate_preview.jpg, capture.npz, hover_rgb.jpg/hover_depth.npz.
  마지막 hover 이름의 영상은 최종 인양 후 화면으로 덮어 저장되었음을 유의.
  로그 `/tmp/candidate_pick_once.log`. 실행 완료 exit0. production 파지 경로 수정 없음.

### 17:41 사용자 파지 확인 및 추가 인양 완료

- 사용자가 "아주 잘 잡았어. 근데 왜 안 들어올리지?"라고 파지 성공을 시각적으로
  확인했다. 직전 짧은 인양은 nominal22.8mm/실제 FK약15.6mm여서 충분한 들림으로
  보이지 않은 상황. 사용자에게 짧은 검증 인양이었다고 설명하고 약5cm 추가 인양 진행.
- `/tmp/candidate_lift_held.py`는 초기 복귀/재촬영/재파지 없이 직전 plan의 남은
  상승 경로(하강 waypoint6개 역순+원래 상공점)를3.5초로 실행했다.
- 시작 전 직전 실제 관절에서 변화0.04rad 이하 및 stable encoder hold 확인.
  최신 관절/odom/finger-floor 여유를 감시하며 인양 controller 성공.
  이후 encoder hold 유지 확인. 그리퍼를 열거나 바퀴를 움직이지 않았다.
- 결과 `cleanup_debug/candidate_hover_20260906_174137/result.json`,
  hover_rgb.jpg/hover_depth.npz는 추가 인양 후 증거.
  로그 `/tmp/candidate_lift_held.log`, exit0, `LIFT_DONE_HOLDING True`.
- **현재 바나나를 잡은 채 원래 후보 상공 높이로 추가 인양하여 유지 중**.
  후속 요청 전 그리퍼 개방/초기 복귀/운반을 자동 실행하지 않는다.

### 후속 요청: ID 3 바깥쪽 수거 지점으로 전체 정리 목적지 변경

- 사용자가 `image.png`의 초록 점(ID 3 바깥쪽)을 정리 목적지로 지정하고 코드 수정을 요청.
  지도상 ID 3 `(0,1.5)`, 그림의 바깥 방향은 map -X임을 마커 배열과 대조.
- 그림에 실제 거리 정보가 없어 바깥쪽40cm를 기본 가정으로 안내하고 텍스트 질문을
  보냈다. 아직 실측 거리 응답은 없으므로 **40cm는 확인된 측정치가 아닌 가정**이다.
- `task_zones.yaml`: zone `marker_3_outer_collection`, 물체 바닥점 `(-0.40,1.50)`,
  base 정지 `(-0.16,1.50,yaw=π)`, nominal release TCP 높이35mm.
  주행 스캔선x=0 바깥에 물체를 놓고, 제외 반경20cm를 물체점 중심으로 계산.
- 기존 ID 2 도착 후 높은 파킹 자세에서 단순 open 하던 동작을 교체:
  `PlaceObject.srv` → `/cleanup/place_object`가 IK/바닥경로/holding 확인,
  상공→하강→open 확인→후퇴 수행. manager는 성공 후 park→스테이션 복귀.
  실패 시 수거 완료 처리/대체 open 없음. release 후 후퇴 실패도 released별도 기록.
  drop navigation에도 precision approach BT 적용.
- 좌표/설정 설명 `src/cleanup_task_manager/README.md`, 도식
  `docs/collection_zone_layout.png`. 현재 지도 저장값에서 물체점 및 base점은 free cell;
  현장의 실제 여유/40cm 치수까지 확인한 것은 아니다.
- 새 구성은 **기존 /start_cleanup 파이프라인의 목적지/내려놓기 변경**이다.
  앞서 실물로 확인한 candidate grasp 실험 스크립트를 자동 pick 서비스에 연결하는
  별도 변경은 이번 수거 지점 변경에 포함하지 않았다. 자동 manager의 pick는 여전히
  /execute_pick_and_place이며, 실제 전체 태스크 전 이 차이를 인지해야 한다.

### 사용자 지시: 실행은 사용자가 시키며, 로봇 브링업 종료

- 사용자가 "실행은 내가 시킬거야. 끝나면 말해. 그리고 로봇 브링업 ... 꺼봐" 지시.
  **이후 사용자의 명시적 실행 지시 전 실제 bringup/주행/팔 동작을 시작하지 않는다.**
- 바나나를 잡고 있으므로 종료 시 팔/물체를 받치도록 안내 후 종료했다.
  별도 perception와 pick process group에 SIGINT, project.launch PID61030에 SIGINT.
  controller PID61049, RealSense PID61057, 라이다/주행/영상/진단 프로세스 종료 확인.
  perception 종료 로그의 rcl_shutdown 중복 오류는 종료 시 발생했으며 프로세스는 종료됨.
- **현재 실제 로봇 bringup OFF. 종료 후 물체/팔의 물리 상태는 재확인하지 않았다.**
  이후 진행한 ROS 테스트는 ROS_LOCALHOST_ONLY 및 격리 ROS_DOMAIN_ID의 모의 컨트롤러뿐.

- 수거 지점 변경 최종 검증: 3개 패키지 빌드 완료. C++ unit 테스트는
  pick_and_place9개, cleanup_task_manager5개 통과. 격리 모의 컨트롤러4개 통과
  (`/tmp/collection_place_controller_tests.log`), 모의 전체 임무20개 통과
  (`/tmp/collection_mission_tests.log`, 66.94초).
  정상 운반/내려놓기, 하강 실패 시 미개방, 잘못된 목표 거절, ID3 목적지 전달,
  placement 실패 시 미수거 처리 및 기존 실패/재시도 경로 포함.
- 이 환경에서는 CMake의 pytest 탐지 경고로 colcon이 Python test를 등록하지 않아,
  두 Python 통합 테스트 파일을 명시적으로 pytest로 실행해 검증했다.
- 수정/검증 완료 후 실제 로봇을 다시 시작하지 않았다.

### 사용자 후속 실행 17:55~17:58 실패 로그 분석 (동작 명령 없음)

- 실행 `cleanup_debug/run_20260906_175553_156099`은 scan_station_1 바나나 검출,
  접근, 재검출, IK 검사까지 통과. 17:58:07 OPEN에서 gripper가 움직이지 않아
  실패했고, 복구 OPEN도 같은 stalled 결과로 17:58:12 임무 실패.
- goal+0.019 대비 actual 약-0.002853, stalled1/reached_goal0.
  전체926개 telemetry의 그리퍼 변화폭은 약0.023mm뿐. 모터/구동 경로 근본 원인은
  servo error/torque/current/goal register가 없으므로 현재 로그만으로 확정 못 함.
- 실제 자동파지는 기존 -60도 planGrasp이며 성공했던 candidate 파지/+30mm 보정이
  연결되지 않았고, camera calibration도 nominal OFF였다. 즉시 OPEN 실패와는
  별도인 자동 태스크 통합 차이로 사용자에게 알려야 함.
- 상세 원인/근거/후속 순서 `cleanup_debug/run_20260906_175553_156099/failure_analysis.md`.
- 분석만 수행. 로봇 실행/모터/torque/재부팅 명령 및 구현 수정 없음.

### 18:03~18:08 사용자 "확인해봐": 직접 OpenCR 점검, 전원 확인 대기

- 전체 bringup OFF 유지. raw Dynamixel SDK로 ID200/1Mbps/protocol2 직접 read 성공.
  그리퍼 present2172tick(q=-0.002853204m), velocity0, current int16=-1.
  aggregate torque_joints0, wheels torque0. 현재 OFF 상태 정지만으로 고장 단정 못 함.
- firmware register222의 goal refresh만 요청(모터 목표 쓰기 아님), 이후 goal2172.
  초기 proxy cache2048을 실제 servo goal로 오해하지 말 것.
- **보드 입력전압 raw1881=18.81V 반복 확인**. 개별 모터 입력전압/오류비트는
  현재 proxy에서 읽을 수 없어 과전압/보호정지 확정은 아니다.
- 표준 XM430 권장12V/범위10~14.8V이므로 전원 종류 및 어댑터 정격 출력 질문.
  확인 전 torque ON/개방 시험/재부팅 안 함. 사용자의 전원 응답 대기.
- `cleanup_debug/gripper_diagnostic_20260906/diagnosis.md` 및 read 로그에 근거/한계.
  최초 read_before.log의 float 전압 출력은 잘못된 해석; raw×.01이 올바름.
- production OpenCR 소멸자는 팔 park 및 gripper0 명령 후 torque OFF하므로
  단독 진단에 해당 클래스를 생성하지 않는다. proxy torque199는 팔 전체를 제어.
- 제한 폴더/production 코드 변경 없음. 전체 task candidate 통합 차이도 여전히 남음.

### 후속 18:14 실행 분석 및 자동 파지 통합 (실물 실행 없음)

- 사용자: 개선 미적용/바나나 끝 파지/ID2 미집기. 통합 누락을 인정하고 수정했다.
- run_20260906_181448_4734: 기존PLAN-60/-55도, camera calibration OFF,
  +30mm/몸통후보 미연결. 두 번 OPEN 통과 후 EMPTY_GRASP. 전압으로 설명할 근거 없음.
- ID2 네 방향 모두2프레임 무검출→0후보→ID3. 사진미저장으로 미검출근본원인 불명.
  실제cameraFOV70.22도 vs scan90도 사각 설계 확인.
- project 기본 calibratedcamera/bodycandidates/candidateexecutor/link1X+.03 연결.
  perception은 관측점, CPP가 보정한번. candidate_body만 집기, candidate_approach는
  접근전용. evaluator도 candidate모드 요구. 새 관측/평가 ROS인터페이스 사용.
- 동일 body_candidates/choose_candidate/C++candidate_grasp_plan 공유.
  OPEN→hover→10점하강→CLOSE/hold→전체역순인양→park. legacyINSERT생략.
- 45도8방향, 무검출5프레임 후 RGB/JSON 저장. 후보.candidates.json/선택깊이NPZ.
- 성공17:39 RGB-D 재생에서pixel839,543/pitch-17.5 및 모든하강관절목표 동일 확인.
- 빌드5패키지, 모의controller7/mission21, C++9+5, launchconfig2 통과.
- 상세: cleanup_debug/run_20260906_181448_4734/failure_analysis.md.
- 실제동작/bringup/모터명령 실행 안 함. 전체실물성공 아직미검증. 다음 사용자실행 시
  ROS인터페이스/설정 반영 위해 전체bringup재시작 및 install/setup.bash source 필요.


### 후속 사용자 요청: 무검출 스캔 사진 생략

- 로봇 실행 중 상태를 유지한 채 코드만 변경. 기존 45도8방향은 RGB FOV70.22도에 따른 관측 중첩을 위해 유지.
- 이전의 무검출5프레임+empty JPG/JSON 저장은 더 이상 기본 동작이 아님.
- `scan_perception_node.empty_scene_frames=2`: 처음 두 프레임 모두 대상 YOLO 무검출이면 파일 저장 없이 다음 heading으로 진행.
- 대상이 한 번이라도 보이면 depth/TF 실패여도 full burst 유지. 정상 물체는 기존3회확인·SAM·body 후보·30mm보정으로 진행.
- 프레임 부족/추론 실패를 무검출 성공으로 바꾸지 않음. YAML 값을5로 바꿔 더 확인할 수 있으나 무검출 사진은 저장하지 않음.
- 현재 실행에는 자동 반영되지 않음. 다음 perception node 시작부터 반영. 이번 실행의 완료 결과는 아직 평가하지 않음.

### 후속 분석: ID 1·2 바나나 검출 후 접근 0회에서 건너뜀

- 완료된 사용자 실행 `run_20260906_192010_47639`의 해당 스테이션 로그 분석.
  YOLO/SAM와 Gemini 수거 선택은 성공했으며 body 실행기·전방30mm도 적용되어 있었다.
- 관측 높이21.36/23.15mm 때문에 모든 pitch가 `insufficient_body_depth`로 탈락.
  실제로는 주변 관측 바닥이 기준보다11.05/9.17mm 낮았으며, 주변 바닥 기준
  바나나 높이는32.41/32.32mm였다. 자세별 카메라/팔/바닥 좌표 불일치 증거다.
- `align_grasp_floor=true`인 실험 보정 프로필에서, 검증된 주변 바닥으로
  최대15mm 위쪽만 후보 높이를 정렬한다. 원시 좌표/보정량/평면 품질을 기록한다.
  관절/바닥 여유/깊은 몸통 파지 제한 및 전방30mm 단일 적용은 유지한다.
- 첫 파지/접근 후보 거절 시 기존 관측 자세로 재관측1회 후 판단하도록 수정.
  실제 실패 사유와 접근 횟수를 기록하여 'IK 실패/접근 후 실패' 오표기를 제거했다.
- 저장 장면 재생에서 두 대상 모두 베이스20cm 접근 후보가 도착오차2cm 포함 통과.
  로봇 실행/재시작/움직임은 하지 않았다. 세부 근거와 한계는
  `docs/GRASP_STATION_SKIP_20260906.md`, `performance_debug/grasp_floor_replay.json` 참조.

### 후속 요청: 과도한 파지 품질/접근 여유 조건 완화

- 기본 `performance.yaml`의 `grasp_min_body_depth`를 perception/실행기 모두
  4mm로 설정했다. 6mm 미만이라서 탈락하던 파지도 4mm 이상이면 평가한다.
- `approach_error_margin=0`: 가정한 도착 오차2cm 검사는 기본값에서 해제.
  명목 위치의 전체 파지·인양 및 실제 도착 후 재관측/판정은 계속 수행한다.
- `performance_conservative.yaml`과 개별 노드 기본값은 이전6mm/2cm 조건 유지.
- 실제 손가락 바닥 여유8/6mm, 관절 한계, 정지/센서 유효성 검사는 유지했다.
- 과거 거절 좌표(.337221,-.047522,-.069773)가 새 기준에서
  pitch -20도, body depth5.780mm로 통과한다. 실기 실행은 하지 않았다.

### 20:02 실행 분석: 파지 가능 판정 후 2초 시간 제한에서 취소

- `run_20260906_200254_74817`: ID 1에서 관측 높이31.24mm로 보정하고,
  베이스22cm 접근 목표를 실제 요청했다. 도착 후 새 몸통 후보가
  link1(.307559,.031291,-.068912), 실행 pitch -20도로 도달성 검사를 통과했다.
- 바로 다음 `requestPick()`의 별도 하드코딩 `age > 2.0`에서
  `Reacquired pick observation is stale`로 파지 명령을 보내지 않았다.
- capture018 촬영 시각1788692674.238778, 거절 시각1788692676.308766:
  실제 영상 나이 **2.069988초**. YOLO/SAM/후보/증거 저장 처리 직후
  70ms 초과로 실패했다. 관측/기구학 개선 미적용이 이번 원인은 아니다.
- 관리 노드와 파지 실행기 모두 `target_max_age=6.0`으로 통일했다.
  두 성능 프로필과 개별 노드 기본값에도 적용하여 한쪽만 완화되는 것을 방지했다.
  촬영 timestamp는 그대로 보존하고, 관절/odom의 실시간 유효성 검사는 유지한다.
- `PICK_OBSERVATION_AGE`, `TARGET_ACCEPTED`, `TARGET_EXPIRED`에서 실제 영상
  나이와 한도를 기록한다. 실제 좌표와2.07초 처리 지연으로 파지→닫기→인양
  가짜 ROS 회귀를 추가했다. 7초 지난 영상과 미래 timestamp는 거절하는지도 검사한다.
- 검증 완료: 전체20패키지 빌드, C++기구학11개, 파지 가짜ROS10개,
  미션25개/config5개 및 launch/lint 통과. 실제 로봇 명령은 보내지 않았다.

## 파지 후 운반 정지 / 종료 시 그리퍼 해제

20:13 실행은 파지·인양 성공 후 운반 중 바나나/손 부근 깊이 점을 StopFootprint가
장애물로 처리해 멈췄다. 카메라 재검출 없이 확인된 파지를 오도메트리로 운반하고,
운반 자세에서 자기 물체 점만 필터하도록 수정했다. 임무 완료·중단·실패 및
프로젝트 런치 Ctrl-C 시 그리퍼 개방을 추가했다. 원인 수치, 변경 조건, 하드웨어
종료 순서 및 검증 내용은 [CARRY_RELEASE_20260906.md](CARRY_RELEASE_20260906.md) 참고.
실제 로봇 동작은 실행하지 않았다.
