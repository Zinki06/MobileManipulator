# 17:58 파지 전 정지 분석

직접적인 정지 원인은 그리퍼 개방 실패다. 검출, Gemini 선택, Nav2 접근,
재검출, IK 도달성 검사는 통과했다. 실제 파지 하강은 시작하지 않았다.

## 확인된 타임라인 (KST)

- 17:58:07.099: 새 목표 publish, PICK_START.
- 17:58:07.305: OPEN, 목표 그리퍼 joint position +0.019.
- 17:58:08.058: position=-0.002853, stalled=1, reached_goal=0.
- 17:58:10.073: 최신 엔코더에서 개방 폭 확인 실패 → pick 실패.
- 17:58:10.075: 복구용 개방 요청을 컨트롤러가 다시 수락.
- 17:58:10.827: 같은 position=-0.002853, stalled=1, reached_goal=0.
- 17:58:12.843: 복구도 실패하여 MISSION_FAILED.

그리퍼가 목표 근처까지 움직여 허용오차에 걸린 상황이 아니다.
telemetry 926개 샘플 전체에서 gripper_left_joint는
-0.00287621398 ~ -0.00285320427 사이 두 값뿐이며, 변화폭은 약0.023mm다.
두 개방 시도 구간의28개 샘플도 동일하다. 로그 시작부터 사실상 움직이지 않았다.
두 finger joint 값은 하드웨어에서 같은 gripper 값을 복제하므로 독립적인 두 센서
검증으로 해석하지 않는다.

## 소프트웨어 동작과 아직 모르는 구동 원인

컨트롤러의 action goal 수락과 fresh joint feedback은 확인된다.
allow_stalling=true는 잡을 때의 정지를 허용하지만, pick 노드는 실제 개방 폭을
별도로 확인하므로 stalled 결과만 보고 진행하지 않았다. 이번 중단은 이 검사가
막은 것이다. 목표 개방 +0.019에서 약21.85mm 차이가 있어 tolerance 완화로
정상 동작을 만들 수 있는 사례가 아니다.

서보 토크/보호상태, 기계적 걸림, OpenCR 이후 명령 전달 실패 또는 그리퍼 측
feedback 문제 중 어느 것이 구동 불응을 만들었는지는 현재 로그로 확정할 수 없다.
서보별 Hardware Error Status, Torque Enable, Present Current/Temperature,
실제 Goal Position 및 쓰기 트리거 결과가 기록되지 않았다.
시작 로그의 전체 joints torque ON은 개별 그리퍼의 실제 상태를 보증하지 않는다.
GripperResult effort=10은 측정된 그리퍼 부하로 취급하면 안 된다. 현재 하드웨어는
position command interface를 쓰며 ROS action의 max_effort를 직접 구동 전류로
전달하지 않는다.

명령 시간에 보이는 motion_guard의 BLOCKED: command timeout은 Nav2 정지 후
속도 명령이 끝난 상태와 일치한다. mission 실패 메시지는 gripper 개방 실패이며,
이 실행에서 FAULT/localization DEGRADED가 실패 전이를 유발한 기록은 없다.
종료 시 찍힌 일부 process 오류는 17:58:55 이후 SIGINT 종료 시점으로,
17:58:12의 임무 중단 원인과 구분해야 한다.

## 별도로 확인된 전체 태스크 연결 차이

- 실제 경로는 /execute_pick_and_place의 기존 planGrasp다.
- 실제 선택 pitch=-60°, insertion=0, target link1=(0.316315,-0.008669,-0.050977).
- 성공했던 실험용 body-candidate/pitch 선택과 +X30mm 보정은 이 경로에 없다.
- launch는 grasp_camera_nominal.yaml을 사용했다. capture010 NPZ에도
  calibration_enabled=False로 저장되어 있다.

이 연결 차이는 이번 OPEN 실패의 직접 원인은 아니지만, 그리퍼 문제를 해결해도
성공했던 파지 실험과 같은 조건으로 전체 임무가 동작하지 않는 별도 문제다.
수거 목적지/내려놓기 수정은 적용됐으나 이번 실행은 그 단계에 도달하지 않았다.

## 후속 점검 순서

1. 사용자 실행 승인하에 그리퍼 단독의 Goal/Present Position, Torque Enable,
   Hardware Error 및 실제 움직임을 확인해 구동 불응 원인을 분리한다.
   현재 요청은 분석만이므로 모터 명령, torque 변경, reboot는 하지 않았다.
2. 전체 스캔 이전에 그리퍼 응답을 확인하는 준비 단계를 둔다. 현재는 park만 하므로
   응답 없는 그리퍼를 가진 채 약3분 스캔한 뒤에야 실패가 검출됐다.
3. 실험에서 확인한 후보 선택/보정/잡힘 검증을 전체 임무의 pick 경로에 명시적으로
   연결하고, 기본 launch의 calibration 설정과 일치하는지 검사한다.

근거 파일:
- mission.log 46~58행.
- /home/user/.ros/log/pick_and_place_156078_1788684897494.log 9~19행.
- /home/user/.ros/log/ros2_control_node_155878_1788684896934.log.
- motion_debug/run_20260906_175502_155961/telemetry.jsonl.
- scan_station_1_approach/heading_250_capture_010_object_0.npz.
