# 05_planning — 경로 계획

전역 경로 계획과 장애물 회피 지역 경로 계획을 담당하는 패키지입니다.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `global_path_planner` | MGeo 기반 전역 경로 계획 |
| `local_path_planner` | Lattice Rollout 기반 지역 경로 계획 |

## 빠른 시작

```bash
# 전역 경로 계획
roslaunch global_path_planner waypoint_global_path_planner.launch

# 지역 경로 계획 (장애물 회피)
roslaunch local_path_planner local_path_planner.launch
```
