# TurtleBot3 바닥 정리 프로젝트

TurtleBot3 매니퓰레이터가 ArUco로 위치를 보정하고, 정해진 스테이션에서 바나나를
찾아 집은 뒤 ID 3 바깥 수거 지점에 내려놓는 ROS 2 Humble 워크스페이스다.
장비를 인수하면 **[인수인계·실행 안내](docs/HANDOVER.md)**부터 읽는다.

## 문서

- [문서 목차](docs/README.md): 실행, 구조, 설정, 파지, 로그, 개발 안내.
- [현재 제한과 리팩터링 검토](docs/MAINTENANCE.md): 남은 실물 확인과 개선 우선순위.
- [디버깅 로그 확인](docs/DEBUG_LOG_GUIDE.md): 검출·파지·주행·종료 문제 분석.

## 기본 실행

기존 컴퓨터에서 장치 연결과 설치 환경을 확인한 뒤 로봇 측 터미널에서 실행한다.
브링업은 제어기를 시작하고 팔을 초기 자세로 움직인다.

```bash
cd /home/user/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_bringup project.launch.py
```

별도 터미널에서 같은 환경을 source하고 임무를 시작한다.

```bash
ros2 service call /start_cleanup std_srvs/srv/Trigger '{}'
```

중지·종료와 원격 RViz 명령은 [인수인계 안내](docs/HANDOVER.md)에 있다.
현재 설정은 정리된 시험 구역용이며, 운반 중 깊이 장애물 감시 범위는
[현재 설정의 제한](docs/CONFIGURATION.md#운반-중-깊이-장애물-처리)을 확인한다.

## 수정 범위

다음 세 폴더와 개인 메모는 수정하지 않는다.

- `src/realsense_bringup`
- `src/segmentation`
- `src/turtlebot3_manipulation`
- `MEMO.md`

패키지 생성·빌드·테스트 규칙은 [AGENTS.md](AGENTS.md)와
[개발 안내](docs/DEVELOPMENT.md)를 따른다. `build/`, `install/`, `log/`,
실행 디버그 자료와 `.env`는 Git에 추가하지 않는다.
