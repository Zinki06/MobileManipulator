# nav_debug 진단 로그 디렉터리

이 디렉터리는 터틀봇 자율주행(`aruco_waypoint_navigator`)의 블랙박스 진단 로그와 카메라 스냅샷을 관리합니다.

---

## 📁 디렉터리 구조

```text
nav_debug/
├── run_YYYYMMDD_HHMMSS/      # 🚀 매 실행(노드 기동)마다 자동 생성되는 독립 세션 폴더
│   ├── navigation.log        # 해당 회차의 전체 주행 텍스트 로그
│   ├── snap_001_....jpg      # 주행 상태 전환 시점의 카메라 화상 스냅샷
│   └── ...
├── navigation.log            # ⚡ 가장 최근 실행의 실시간 로그 (tail -f 모니터링용)
└── README.md                 # 본 설명 문서
```

---

## 🔍 실시간 모니터링 명령어

주행 중 로봇의 위치, 마커 잠금 여부, 다음 스텝 진행 상황을 실시간으로 확인하려면:

```bash
tail -f /home/user/turtlebot3_ws/nav_debug/navigation.log
```

---

## 📝 로그 이벤트 태그 설명

- **`[NODE_INIT]`**: 네비게이터 노드 기동 완료
- **`[ROUTE_START]`**: 웨이포인트 경로 주행 시작
- **`[STEP_START]`**: 세부 단계(직진/회전/마커확인) 시작
- **`[NAV_PROXIMITY_ARRIVED]`**: 목표 마커 근접 도달 완료 (다음 회전 단계로 바통 터치)
- **`[ROTATE_INIT]` / `[ROTATE_DONE]`**: IMU 피벗 제자리 90도 회전 시작 및 완료
- **`[MARKER_LOCKED]`**: 회전 후 다음 마커 시야 확보 및 확인 완료
- **`[ROUTE_COMPLETE]`**: 전체 코리더 주행 완주 성공
- **`[NAV_RETRY]` / `[NAV_FAILED]` / `[ROUTE_ABORTED]`**: Nav2 액션 재시도 및 실패/중단 진단 기록
