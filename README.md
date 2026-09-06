
## 아래 3개의 폴더는 건들지 않기
- src/realsense_bringup 
- src/segmentation
- src/turtlebot3_manipulation

## 패키지 만드는 법
- ros2 pkg create --build-type ament_cmake <package_name> //cpp
- ros2 pkg create --build-type ament_python <package_name> //python

## MEMO
- grasping 개선 필요. 좀 더 들어가야하는데 안 들어가서 바로 앞에서 놓침
- id=4에서 멈춰있는 경향이 있음 : TION_FAULT] Rejected map reset: translation=0.455m yaw=0.160rad. Motion inhibited; inspect localization before restarting.
[cleanup_task_manager_node-28] [INFO] [1788609821.714293660] [cleanup_task_manager]: [MISSION_FAILED] Localization became DEGRADED during cleanup.
[motion_guard_node-14] [INFO] [1788609821.722754228] [motion_guard]: [MOTION_GUARD] FAULT: localization degraded
[motion_executor-15] [INFO] [1788609821.736368842] [motion_executor]: [MOTION] {"state": "CANCELED", "goal_id": "d72a88bd8a9365c8999530e1113f314e", "action": "spin", "reason": "shutdown"}


# 커맨드

## 터틀봇
ros2 launch project_bringup project.launch.py

ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'

## 원격 컴퓨터
rviz2 -d "$(ros2 pkg prefix --share turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
