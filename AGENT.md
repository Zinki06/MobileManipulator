# 아래 네 파일은 '절대' 건들지 않기
- src/coin_d4_driver
- src/ld08_driver
- src/realsense_bringup
- src/turtlebot3_manipulation

###  humble docs 주소
https://docs.ros.org/en/humble/index.html

# 패키지 만드는 방법

파이썬 패키지
ros2 pkg create --build-type ament_python <package_name>

CPP패키지
ros2 pkg create --build-type ament_cmake <package_name>

# ROS2에서는 C++ 구현(ament_cmake)을 Python 구현(ament_python)보다 우선적으로 사용하세요