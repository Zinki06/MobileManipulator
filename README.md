
## 아래 3개의 폴더는 건들지 않기
- src/realsense_bringup 
- src/segmentation
- src/turtlebot3_manipulation

## 패키지 만드는 법
- ros2 pkg create --build-type ament_cmake <package_name> //cpp
- ros2 pkg create --build-type ament_python <package_name> //python

## MEMO
- 아르코 튀던거 다음 코드 수정해서 잡기 "src/aruco_localizer/src/aruco_localizer_node.cpp"

- 그래스핑 IK & path planning 고치기
- 네비게이션 아르코 위를 직선으로 가도록 수정하기 // nvidia labs에 좋은 거 많으니깐 시간 많으면 좋은 알고리즘 사용해보기

# 커맨드

## 터틀봇
ros2 launch project_bringup project.launch.py

## 원격 컴퓨터
rviz2 -d "$(ros2 pkg prefix --share turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
