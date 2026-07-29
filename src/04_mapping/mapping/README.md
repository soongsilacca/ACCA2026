# mapping

LiDAR 포인트클라우드를 기반으로 컬러 3D 맵을 생성하는 패키지입니다.

## Launch 파일

```bash
# 사전 수집된 bag 파일로 컬러맵 생성
roslaunch mapping colored_map_creator.launch

# 실시간 주행 중 컬러맵 생성
roslaunch mapping online_colored_map_creator.launch
```

## 출력

생성된 PCD 맵은 NDT 기반 로컬라이제이션(`hdl_localization`)의 입력 맵으로 사용됩니다.
