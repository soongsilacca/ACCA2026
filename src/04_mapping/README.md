# 04_mapping — HD Map 생성 및 로드

MGeo HD Map 파싱/시각화와 LiDAR 기반 Point Cloud Map 생성 패키지입니다.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `hdmap_loader` | MGeo HD Map 파서 & RViz 마커 발행 |
| `mapping` | LiDAR 기반 컬러 Point Cloud 맵 생성 |

## 빠른 시작

```bash
# HD Map 마커 시각화
roslaunch hdmap_loader mgeo_marker.launch

# 오프라인 컬러맵 생성
roslaunch mapping colored_map_creator.launch

# 온라인 실시간 맵 생성
roslaunch mapping online_colored_map_creator.launch
```
