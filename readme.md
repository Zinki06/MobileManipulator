ssh user@192.168.0.28

# 아래 네 파일은 '절대' 건들지 않기
- src/coin_d4_driver
- src/ld08_driver
- src/realsense_bringup (※ 이번에 realsense_tf.cpp 파일만 수정됨)
- src/turtlebot3_manipulation

### humble docs 주소
https://docs.ros.org/en/humble/index.html

# 패키지 만드는 방법
파이썬 패키지
ros2 pkg create --build-type ament_python <package_name>

CPP패키지
ros2 pkg create --build-type ament_cmake <package_name>

# 패키지 실행 방법
1. 런치파일 실행
ros2 launch <패키지 이름> <launch파일 내부에 있는 .launch.py로 끝나는 파일 이름>

2. 런파일 실행
ros2 run <패키지 이름> <cmakeList.txt에 있는 실행파일 이름> //cmake 패키지의 경우
ros2 run <패키지 이름> <setup.cfg에 있는 실행파일 이름> //python 패키지의 경우

# ROS2에서는 C++ 구현(ament_cmake)을 Python 구현(ament_python)보다 우선적으로 사용하세요

---

# 🤖 ROS 2 분산 환경 구동 가이드 (TurtleBot SBC ↔ Remote PC)

> **전제 조건**: 터틀봇(SBC)과 원격 PC는 동일한 Wi-Fi 네트워크에 연결되어 있어야 하며, 동일한 `ROS_DOMAIN_ID`를 설정해야 합니다.
> ```bash
> export ROS_DOMAIN_ID=10 # (터틀봇과 PC 공통 입력)
> ```

---

## 1. 🐢 터틀봇(SBC) 실행 명령 (모든 센서/연산/제어 처리)

모든 데이터 연산(RealSense 비전, YOLO+SAM 세그멘테이션, PCL 포인트클라우드 C++ 필터링 및 3D Centroid 추정, MoveIt 2 모션 플래닝, 모터 제어)은 터틀봇 온보드 컴퓨터에서 수행됩니다.

### [터미널 1] 모터 및 로봇 하드웨어 드라이버 Bringup
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

### [터미널 2] RealSense Wrist Cam 드라이버 & RealSense TF 실행
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch realsense_bringup realsense_tf.launch.py
```

### [터미널 3] MoveIt 2 Planning Server 실행
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_manipulation_moveit_config move_group.launch.py
```

### [터미널 4] YOLO+SAM 세그멘테이션 노드 실행
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_pick_place segmentation_node
```

---

## 2. 💻 원격 PC(Remote PC) 실행 명령 (시각화 및 3D 데이터 모니터링)

원격 PC에서는 데이터 시각화(RViz2) 및 추정된 3D Centroid 포즈 모니터링만 수행합니다.

### [터미널 1] RViz2 데이터 시각화 (로봇 모델, TF, 카메라 영상, PointCloud, MoveIt Scene)
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_manipulation_moveit_config moveit_rviz.launch.py
```

### [터미널 2] 3D Centroid 좌표 & TF 수신 확인 (모니터링용)
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# 물체 3D 중심점 (base_link 기준 X, Y, Z 좌표) 토픽 확인
ros2 topic echo /object_centroid

# 물체 TF frame 위치 확인 (base_link -> object_frame)
ros2 run tf2_ros tf2_echo base_link object_frame
```