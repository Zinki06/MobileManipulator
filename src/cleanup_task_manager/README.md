# cleanup_task_manager

스테이션 순회, 인식·계획 요청, 재접근, 파지·배치·복귀와 실패 처리를 조정한다.
이동 실행은 `robot_motion`, 인식은 `cleanup_perception`, 팔 실행은 `pick_and_place`가 담당한다.

- [임무 흐름과 ROS 인터페이스](../../docs/ARCHITECTURE.md)
- [스테이션·수거점 설정](../../docs/CONFIGURATION.md)
- [시작·중지·종료](../../docs/HANDOVER.md)
- [로그와 실패 원인 확인](../../docs/DEBUG_LOG_GUIDE.md)

임무 설정은 [config/task_zones.yaml](config/task_zones.yaml)에 있다.
통합 실행의 성능 프로파일이 일부 기본값을 덮어쓰므로 설정 문서와 함께 확인한다.
