# 18:14:48 실행: 끝부분 파지와 ID 2 미검출

성공한 수동 실험을 자동 정리 경로에 연결하지 않은 통합 누락이다.
수거 목적지만 변경했고 candidate grasp, +30mm, 카메라 보정은 실험에만 남아 있었다.

## 확인된 원인

- 첫 pick 18:16:58 PLAN: pitch=-60, insertion=.000956,
  target_link1=(.315432,-.015686,-.049953).
- 재시도 18:17:17 PLAN: pitch=-55, insertion=.002825,
  target_link1=(.324734,-.025157,-.048702).
- 실제 코드는 기존 planGrasp. 접근 capture NPZ의 calibration_enabled=False.
  project 기본값은 grasp_camera_nominal.yaml이었고 +X30mm도 없었다.
  RGB 중심이 옳아도 카메라→팔 변환·그리퍼 보정·진입 경로가 실험과 달라진다.
  정확한 물리 오차 크기는 측정하지 않았다.
- 두 번 OPEN 통과, CLOSE 뒤 EMPTY_GRASP. 새 telemetry에서 gripper q≈+.018983.
  이전 OPEN 무응답/18.81V 보드 표시와 별개이며 이번 실패를 전압 탓으로 확정 못 한다.
- ID2는 네 방향 모두 YOLO2프레임 무검출→confirmed_objects0→ID3 이동.
  파지나 수거영역 제외 단계에서 거절된 것이 아니라 인식 목록이 비었음.
- ID2 무검출 사진을 저장하지 않았고 object 전용 영상도 이 구간을 포함하지 않아
  영상밖/가림/낮은 검출신뢰도 중 어떤 원인인지 확정 불가.
- 저장 camera_k/1280x720에서 수평FOV70.2204도, 촬영90도 간격.
  인접 촬영 사이 약19.78도 사각이 생길 수 있는 설계 결함. ID2 물체가 그 방향에
  있었다는 증거는 아니다. 해당 스캔 telemetry는 팔park/base정지/guardREADY.

## 수정

- project launch 기본 calibrated camera ON, body 후보 ON, candidate 실행 ON.
- perception은 같은 body_candidates/choose_candidate/C++candidate_grasp_plan 사용.
  관측 몸통점을 map으로 전달하고 CPP가 도달성/실행에서 link1+X30mm를 한 번 적용.
- ObjectObservation.grasp_strategy=candidate_body만 집기 허용.
  candidate_approach는 접근용이며 실행 가능한 몸통 경로가 없으면 집지 않는다.
- EvaluateGrasp.require_candidate로 legacy 실행기에 잘못 연결된 경우도 거절.
- OPEN→hover→10점 몸통하강→CLOSE/hold→전체 하강경로 역순인양→park.
  candidate모드에서 legacy INSERT 생략. 기존 바닥여유/hold 검사는 유지.
- 45도8방향 스캔. 무검출도 요청5프레임 검사 후 RGB/JSON 보존.
- 후보별.candidates.json, 선택pixel/depth/calibration NPZ 및 startup GRASP_CONFIG,
  PLAN strategy/offset 기록. standalone hover도 새 perception 선택점 검사에 대응.

## 검증

- cleanup_interfaces/pick_and_place/cleanup_perception/cleanup_task_manager/
  project_bringup 5패키지 빌드 성공.
- 성공 실험 candidate_hover_20260906_173957 RGB-D를 새 automatic helper로 재생:
  동일pixel(839,543), correctedlink1(.332792816,-.048628555,-.066932601),
  pitch-17.5도, bodydepth.009178108m, 모든 descent 관절 목표 동일.
  결과 integration_success_replay.json. 이번 실패 현장에서 새 파지 성공을 확인한 것은 아님.
- 실제 C++pick+격리 모의controller7개 통과(legacy4,candidate정상/빈파지/개방실패3).
  correction1회, INSERT생략, 10점하강/10점인양/상공복귀 확인.
- 모의mission21개 통과(8방향, legacy관측집기거절, 기존실패복구, ID3수거점).
- C++kinematics9/taskconfig5, 실제launchparameterresolve2 통과.
- perception 기능/flake8/pep257 검증. /tmp/candidate_*tests*.log 참조.
- 실제로봇/전체임무 재실행 안 함. 보호폴더 변경 없음.
- ROS인터페이스 변경으로 다음 실물 실행은 전체bringup재시작+install환경source 필요.
  실물 ID2검출/새파지/운반/배치 성공은 사용자 지시 실행에서 확인해야 함.
