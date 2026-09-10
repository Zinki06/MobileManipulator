
## 아래 3개의 폴더는 건들지 않기
- src/realsense_bringup 
- src/segmentation
- src/turtlebot3_manipulation

## 패키지 만드는 법
- ros2 pkg create --build-type ament_cmake <package_name> //cpp
- ros2 pkg create --build-type ament_python <package_name> //python

## 디버깅 문서

- [현재 디버깅 로그 확인 방법](docs/DEBUG_LOG_GUIDE.md): 실행 폴더 선택, 실시간 확인,
  검출·파지·운반·종료 문제별 검색 명령과 성능 지표 읽기.

## MEMO



# 커맨드

## 터틀봇
ros2 launch project_bringup project.launch.py

ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'
ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'

## 원격 컴퓨터
rviz2 -d "$(ros2 pkg prefix --share turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"


codex --dangerously-bypass-approvals-and-sandbox
