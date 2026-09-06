# 2026-09-06 18:03~18:08 그리퍼 직접 점검

사용자 "확인해봐" 요청에 따라 OpenCR USB를 직접 점검했다.
전체 임무/브링업/팔 초기화/그리퍼 개방/torque ON은 실행하지 않았다.

## 확인된 사실

- 포트 `/dev/ttyACM0` 점유 프로세스 없음. ID200, protocol2, 1Mbps ping 성공,
  model20480. 모든 표준 상태 read 성공/error0. millis 증가 및 joint1 1tick 변화.
- ROS 연결0, manipulator 연결1, wheels 연결1. 연결 flag는 개별 모터의
  현재 건강 상태를 보장하지 않는다. torque_joints0은 전체 관절 torque가
  모두 ON이 아니라는 aggregate 값이며, 각 모터 모두 OFF를 독립 증명하지 않는다.
  직전 정상 종료로 torque OFF가 명령된 상태와 일치한다. wheels torque0.
- gripper present2172tick = ROS q 약-0.002853204m, velocity0, current raw65535
  (int16 -1). 이전 실패 위치와 동일. 토크가 꺼진 현재 정지만으로 jam/고장을 단정 못 함.
- proxy cached goal2048에서 firmware의 refresh-only register222=1로
  goal read-back을 요청하면 gripper goal2172, arm goals도 present와 같은 값으로 변경.
  이 refresh는 모터 목표를 쓰는 명령이 아니다. 초기 cache2048을 실제 servo goal로
  해석하면 안 된다. 내부 read 실패 여부는 OpenCR proxy 응답만으로 확정 불가.
- USB에서 ID15 직접 ping은 timeout(-3001). USB slave ID200 경로가 내부 서보 버스를
  직접 노출하지 않으므로 이를 서보 단선의 증거로 해석하지 않는다.
- 배터리/입력전압 register42 raw1881 = **18.81V** (기존 driver와 공식 firmware는
  uint32 ×0.01). 최초 read_before.log의 float 출력은 잘못된 디코딩이며 폐기한다.
  raw 값은 유효하며 read_refreshed.log는 올바른 단위를 표시한다.
- 이 값은 보드 ADC 값이다. 개별 모터 Present Input Voltage/Hardware Error Status/
  온도는 현재 firmware proxy가 노출하지 않아 읽지 못했다. 과전압/보호정지 확정 아님.

## 동작 시험을 보류한 이유

표준 OpenMANIPULATOR-X는 XM430-W350-T를 사용하며 입력12V, XM430 공식 범위
10~14.8V(권장12V). OpenCR 자체 입력 허용범위와 모터 허용범위는 구분해야 한다.
공식 REVH 회로도 p7의 DXL_PWR은 DC_BB_VCCIN에서 MOSFET으로 연결되며,
외부 12V 출력단의 존재만으로 DXL 전압이12V로 제한된다고 볼 수 없다.
보드 revision, 실제 공급 전압/배선, ADC 정확도는 현장에서 미확인.
18.81V 보고값 때문에 전원 확인 전 torque를 켜거나 움직임을 반복하지 않았다.
사용자에게 배터리/어댑터 여부 및 어댑터 출력전압을 질문한 상태다.

또한 proxy torque199는 gripper만이 아닌 모든 팔 관절을 함께 제어한다.
전체 bringup 또는 production OpenCR 객체는 초기/종료 때 팔 자세를 변경하므로
단독 진단에 사용하지 않았다. raw Dynamixel SDK만 사용했다.

## 다음 단계

1. 사용자의 실제 전원 종류/출력 전압 확인. 필요하면 전원 OFF 상태에서 정격 전원으로
   변경하고, 그리퍼 LED 점멸 여부와 간섭/끼임 확인. 현 입력값만으로 부품 교체 지시 금지.
2. 정상 전압 확인 후 현재 자세 유지 조건을 갖춘 bounded 개방 시험.
   개별 servo torque/error/voltage는 별도 진단 버스 등 접근 방법이 필요할 수 있다.
3. 보호정지라면 원인 해소 후 재부팅 필요할 수 있지만, 현재 보호정지를 읽어 확정하지
   않았고 재부팅/펌웨어 교체/제한값 증대는 하지 않았다.
4. 전체 task에는 여전히 성공한 candidate grasp/+30mm/camera calibration 통합이
   별도로 남아 있다. 그리퍼가 회복되어도 전체 파지 성공이 검증된 것은 아니다.

## 증거 및 출처

- `read_opencr.cpp`, `read_before.log`, `read_refreshed.log`
- https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp
- https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/open_manipulator_driver.cpp
- https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
- https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/
- https://github.com/ROBOTIS-GIT/OpenCR-Hardware/blob/master/Schematic/OpenCR_REVH.pdf

공식 master source는 동작 해석 참고이며 현재 보드 firmware binary와 동일 revision임을
검증한 것은 아니다. 제한 폴더 및 production code 변경 없음.
