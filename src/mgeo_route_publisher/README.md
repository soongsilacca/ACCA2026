# mgeo_route_publisher

`map_viz`의 MGeo JSON, global path CSV, `/localization/kinematic_state`를 사용해
multimodal learning 입력 토픽 두 개를 발행하는 ROS1 패키지입니다.

## Published topics

- `/local_route` (`nav_msgs/Path`): `base_link` 기준 meter 단위 local route
- `/mgeo_tokens` (`std_msgs/Float32MultiArray`): row-major `N x 8` (`N <= 128`)

MGeo feature 순서는 다음과 같습니다.

```text
[x/radius, y/radius, sin(relative_yaw), cos(relative_yaw),
 stop_line, signal_class/6, is_link, distance/radius]
```

유효한 token은 거리순으로 발행됩니다. 128행 고정 padding과 mask 생성은
`multimodal_learning`의 dataset extractor가 담당합니다.

```bash
catkin_make
source devel/setup.bash
roslaunch mgeo_route_publisher mgeo_route_publisher.launch
```

기본 CSV는 `~/acca_ws/global_path/global_path.csv`이며 `x`, `y` 열이 필요합니다.

```bash
roslaunch mgeo_route_publisher mgeo_route_publisher.launch \
  global_path_csv:=/path/to/global_path.csv
```

노드 실행 전에 `/localization/kinematic_state`가 발행되어야 합니다.

## Recorded path MGeo approximation

기록 경로의 주행 분기를 유지하면서 가까운 MGeo link 방향으로 부드럽게 보정합니다.

```bash
rosrun mgeo_route_publisher snap_global_path_to_mgeo.py \
  /tmp/global_path.csv \
  --link-file "$(rospack find map_viz)/scripts/link_set.json" \
  --plot /tmp/global_path_mgeo_comparison.png
```

입력 파일을 덮어쓸 때 최초 원본은 `.original` 확장자로 백업됩니다. 저장된 yaw가
정지 구간에서 불안정할 수 있으므로 link 진행 방향은 CSV 점 순서로 계산합니다.
