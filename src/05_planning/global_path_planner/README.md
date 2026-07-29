# global_path_planner

MGeo HD Map의 Link 토폴로지를 기반으로 출발지에서 목적지까지의 **전역 경로**를 계획하는 패키지입니다.

## 노드 목록

| 노드 | 설명 |
|------|------|
| `global_path_planner_node.py` | MGeo Link 그래프 탐색 기반 전역 경로 계획 |
| `waypoint_global_path_planner_node.py` | Waypoint 시퀀스 기반 전역 경로 발행 |
| `static_path_publisher_node.py` | 사전 저장된 정적 경로 마커 발행 |

## Launch 파일

```bash
# MGeo 기반 전역 경로 계획
roslaunch global_path_planner global_path_planner.launch

# Waypoint 기반 전역 경로 발행
roslaunch global_path_planner waypoint_global_path_planner.launch

# 정적 경로 마커만 발행 (시각화용)
rosrun global_path_planner static_path_publisher_node.py
```

## 토픽

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/localization/kinematic_state` | 입력 | `Odometry` | 자차 현재 위치 |
| `/global_path` | 출력 | `Path` | 전역 경로 |
| `/global_path_marker` | 출력 | `MarkerArray` | RViz 시각화용 경로 마커 |
