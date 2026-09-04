# nav_debug 진단 로그 디렉터리

이 디렉터리는 터틀봇 자율주행(`aruco_waypoint_navigator`)의 블랙박스 진단 로그와 카메라 스냅샷을 관리합니다.

---

## 📁 디렉터리 구조

```text
nav_debug/
├── run_YYYYMMDD_HHMMSS_PID/  # 매 실행마다 생성되는 독립 세션 폴더
│   ├── navigation.log        # 해당 회차의 전체 주행 텍스트 로그
│   ├── snap_001_....jpg      # 주행 상태 전환 시점의 카메라 화상 스냅샷
│   └── ...
├── navigation.log            # ⚡ 가장 최근 실행의 실시간 로그 (tail -f 모니터링용)
└── README.md                 # 본 설명 문서
```

---

## 🔍 실시간 모니터링 명령어

주행 중 로봇 위치, 최근 마커 보정 여부, 가상 웨이포인트 진행 상황을 확인하려면:

```bash
tail -f /home/user/turtlebot3_ws/nav_debug/navigation.log
```

---

## 📝 로그 이벤트 태그 설명

- **`[NODE_INIT]`**: 가상 웨이포인트 네비게이터 기동 완료
- **`[ROUTE_START]`**: `routes.yaml`에 정의된 경로 시작
- **`[WAYPOINT_START]`**: 이름, 좌표, yaw를 포함한 Nav2 목표 전송
- **`[NAV_ACCEPTED]`**: Nav2가 목표를 수락함
- **`[WAYPOINT_REACHED]`**: 위치와 yaw를 포함한 Nav2 목표 완료
- **`[WAYPOINT_RECOVERY_REACHED]`**: Nav2가 중단됐지만 15cm 이내 도달을 확인함
- **`[NAV_RETRY]`**: Nav2 거부·취소·중단 후 제한적 재시도
- **`[TELEMETRY]`**: 주행 중 2초마다 `map`/`odom` 자세, `cmd_vel`,
  위치 보정 모드, 웨이포인트 상태 기록
- **`[ROUTE_COMPLETE]` / `[ROUTE_ABORTED]`**: 전체 경로 완료 또는 중단

마커가 보이지 않을 때 `ArUcoRecent: NO`로 기록되는 것은 오류가 아닙니다.
로컬라이저가 초기화된 상태라면 마지막 `map -> odom` 보정을 유지한 채 휠
오도메트리로 이동합니다. 실시간 위치 모드는 다음 명령으로 확인할 수 있습니다.
근접 마커와 제자리 회전 중 마커는 보정에 사용하지 않으며,
`[Correction Frozen]` 로 로컬라이저 ROS 로그에 기록됩니다.

```bash
ros2 topic echo /aruco/localization_mode
```
