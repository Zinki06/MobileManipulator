# project_bringup

하드웨어·카메라·ArUco/Nav2·정리 인식·planner·팔·매니저의 통합 실행 패키지다.
`feedback_robot.launch.py`는 원본 하드웨어 패키지를 보존하며 피드백 제어와 종료 절차를 적용한다.

- [인수인계와 실행](../../docs/HANDOVER.md)
- [launch 인자와 설정](../../docs/CONFIGURATION.md)
- [전체 구조](../../docs/ARCHITECTURE.md)
- [개발과 검증](../../docs/DEVELOPMENT.md)

실행 진입점은 `ros2 launch project_bringup project.launch.py`다.
브링업은 초기 팔 자세 이동을 포함한다. 자세한 운영 절차는 위 문서를 기준으로 한다.
