# local_path_planner

**Lattice Rollout** 기반 실시간 장애물 회피 지역 경로 계획 패키지입니다.  
전역 경로 주변에 다수의 후보 경로를 생성하고, 비용 함수로 최적 경로를 선택합니다.

## 노드 목록

| 노드 | 설명 |
|------|------|
| `local_path_planner_node.py` | Lattice Rollout 기반 지역 경로 계획 |
| `mgeo_local_planner_node.py` | MGeo Link 제약 기반 지역 경로 계획 |

## Launch 파일

```bash
roslaunch local_path_planner local_path_planner.launch
```

## 토픽

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/global_path` | 입력 | `Path` | 추종할 전역 경로 |
| `/localization/kinematic_state` | 입력 | `Odometry` | 자차 위치/속도 |
| `/velodyne_points` | 입력 | `PointCloud2` | LiDAR 장애물 감지 |
| `/local_path` | 출력 | `Path` | 선택된 지역 회피 경로 |
| `/local_path_candidates` | 출력 | `MarkerArray` | 후보 경로 시각화 |

## 알고리즘

1. **Rollout 생성**: 전역 경로 기준 좌우 오프셋(d)으로 N개 후보 궤적 샘플링
2. **충돌 검사**: 각 후보 경로와 LiDAR 포인트 간 거리 계산
3. **비용 계산**: 충돌 위험 + 경로 이탈 거리 가중합
4. **경로 선택**: 최소 비용 경로를 `/local_path`로 발행
