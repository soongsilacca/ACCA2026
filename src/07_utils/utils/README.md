# utils

MORAI & ROS 통합 제어 대시보드 GUI 패키지입니다.  
모든 ROS 노드를 카테고리별로 관리하고 실시간으로 모니터링합니다.

## 노드

| 노드 | 설명 |
|------|------|
| `launcher_gui.py` | Tkinter 기반 통합 제어 GUI |

## Launch 파일

```bash
roslaunch utils launcher_gui.launch
# 또는
rosrun utils launcher_gui.py
```

## 기능

### 카테고리별 프로세스 카드

| 카테고리 | 포함 노드 |
|----------|-----------|
| System | ROS Core, MORAI Simulator, RViz |
| Scenario | Load Specific / Random Scenario |
| Sensor | MORAI UDP Bridge, Traffic Light Controller |
| Perception | LiDAR Ground & Crop Filter |
| Localization | Global EKF, NDT Localization |
| Planning | Global Path, Static Path, Local Planner, MGeo Marker |
| Control | Stanley (Global/Local), Pure Pursuit |

### 주요 기능

- **Start / Stop / Restart**: 각 노드를 독립적으로 제어
- **실시간 로그 터미널**: 노드별 stdout 출력 실시간 표시
- **센서 상태 모니터링**: GPS / IMU / Ego / LiDAR / Localization 토픽 ON/OFF 표시 (2초 기준)
- **차량 모드 전환**: Manual / Auto / Parking 모드 UDP 직접 전송
- **커스텀 명령 추가**: GUI에서 임의의 ROS 명령 추가 실행 가능

## 신호등 제어

차량의 현재 링크 ID를 기반으로 인접 신호등을 탐색하고,  
Red / Yellow / Green 상태를 UDP(포트 7607)로 강제 설정할 수 있습니다.
