# safety_monitor

ACCA 2026 후방 안전 감시 패키지 (ROS1 Noetic, Ubuntu 20.04).
기술보고서 6장 전체를 한 패키지로 구현한다.

| 보고서 | 노드 | 언어 |
| --- | --- | --- |
| 6.1.1 후방 LiDAR 인지 | `rear_lidar_perception_node` | C++ (PCL) |
| 6.1.2 충돌 판정 | `rear_collision_node.py` | Python |
| 6.2 센서 신호 감시 | `sensor_health_node.py` | Python |
| E-STOP 발행 | `estop_gate_node.py` | Python |

기존 `rear_lidar_perception` 패키지를 합친 것이다. 토픽 이름은 그대로(`/rear_lidar/*`)이고
메시지 타입만 `safety_monitor/TrackedObject(Array)`로 바뀌었다.

## 구성

```
MORAI VLP-16 ─▶ velodyne driver ─/rear_lidar/points_raw─▶ rear_lidar_perception_node
                                                             │ /rear_lidar/tracked_objects
/morai/ego_vehicle_status, /local_route, MGeo ──────────────▶ rear_collision_node ─/safety/collision_estop─┐
/imu, /rear_lidar/points_raw, /morai/ego_vehicle_status ────▶ sensor_health_node  ─/safety/sensor_estop───┤
제어기 ─/ctrl_cmd_raw───────────────────────────────────────────────────────────────────────────────────▶ estop_gate_node ─/ctrl_cmd─▶ MORAI
```

```
safety_monitor/
├── msg/                      TrackedObject, TrackedObjectArray
├── src/perception/           rear_lidar_perception_node.cpp
├── src/safety_monitor/       판정 로직 (ROS를 import하지 않음)
├── nodes/                    Python ROS 노드 (입출력만)
├── scripts/                  bag 추출, 재생 비교, 추적 분석
├── config/                   rear_lidar / collision / health / gate .yaml, rviz/
├── launch/                   bringup / rear_lidar / safety_monitor / rear_lidar_report
└── test/                     pytest (ROS 불필요)
```

`src/safety_monitor` 모듈 대응

| 모듈 | 보고서 |
| --- | --- |
| `collision/target_selector.py` | 6.1.2 ① 목표 차선 결정, 차선 변경 종료, 접근 차량 이력 선정 |
| `collision/overlap.py` | ② 진입 시각, ③ 종방향 겹침 구간, ④ 결합 |
| `collision/judge.py` | 대상 1대 판정 + 예외 첫째(정지 중 v_ref) |
| `collision/pass_hold.py` | 예외 둘째(통과 차량 정지 유지) |
| `collision/monitor.py` | 한 프레임 전체 흐름 + 2/5 프레임 발행·해제 |
| `health/signal_watch.py` | 6.2 UNINIT/OK/LOST, T_i = N_i / f_i |
| `lanes/` | MGeo link 매칭, 차선 중심선 이어 붙이기, Frenet 변환 |
| `adapters/tracked_object.py` | 인지 출력 → 판정 입력 |

## 빌드와 실행

기존 `rear_lidar_perception` 패키지는 워크스페이스에서 뺀다 (노드 이름과 토픽이 겹친다).

```bash
sudo apt install ros-noetic-velodyne ros-noetic-pcl-ros python3-scipy
cd ~/catkin_ws/src && mv rear_lidar_perception ~/rear_lidar_perception.bak
cp -r <받은 폴더>/safety_monitor .
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 전체 실행
roslaunch safety_monitor bringup.launch mgeo_path:=<link_set.json 폴더>

# 인지만 (RViz 포함)
roslaunch safety_monitor rear_lidar.launch rviz:=true

# 보고서 캡처
roslaunch safety_monitor rear_lidar_report.launch

# 판정 로직 테스트 (ROS 불필요)
cd ~/catkin_ws/src/safety_monitor && python3 -m pytest -q test
```

제어기 출력 토픽을 `/ctrl_cmd_raw`로 remap해야 gate가 명령을 덮어쓸 수 있다.
`<remap from="/ctrl_cmd" to="/ctrl_cmd_raw"/>`

### multimodal_learning 주행 스택과 같이 실행

```bash
roslaunch safety_monitor multimodal_safety.launch                      # Planner V2 (inference.launch) + MPC
roslaunch safety_monitor multimodal_safety.launch model:=tcp_inference # TCP 직접 제어
# model: inference | tcp_inference | simlingo_inference | transfuser_inference
```

```
multimodal_sensors.launch (카메라, 전방 VLP-16, GPS/IMU, ego status, localization, /local_route, 송신기)
<remap /ctrl_cmd → /ctrl_cmd_raw>  모델 launch (launch_sensors:=false): 모델, MPC 또는 TCP 직접 제어
bringup.launch: 후방 인지, 충돌 판정, 센서 감시, gate(/ctrl_cmd_raw → /ctrl_cmd)
morai_ego_ctrl_sender: /ctrl_cmd → UDP 9093
```

- 송신기(`morai_ego_ctrl_sender`)는 remap 그룹 밖에 둔다. 안에 두면 송신기도 `/ctrl_cmd_raw`를 구독해 gate를 우회한다.
  그래서 모델 launch의 센서 포함 부분은 끄고 `multimodal_sensors.launch`를 따로 띄운다.
- 전방 VLP-16이 2368을 쓰므로 후방 LiDAR는 `rear_lidar_port:=2370`(기본값)으로 받는다. MORAI 후방 LiDAR Destination Port를 2370으로 맞춘다.
- MGeo는 `mgeo_route_publisher`와 같은 `map_viz/scripts/link_set.json`을 쓴다.
- ego status는 `ego_status_port:=9011`로 받은 `/morai/ego_vehicle_status` 하나만 쓴다.
- MPC(`mpc_node.py`)는 10 Hz로 발행한다. gate `cmd_timeout` 0.2 s는 한 주기 누락까지만 허용한다.
- 송신기는 `/morai/episode_status`에 LOADED가 오면 3 s 동안 가속 0.3을 직접 보낸다 (gate 우회).
  `scenario_episode_manager`를 같이 띄울 때는 송신기의 `startup_accel_duration_sec`를 0으로 둔다.

MORAI 네트워크 설정 (후방 LiDAR)

| MORAI 항목 | 값 |
| --- | --- |
| Host Sensor Port | 2369 |
| Destination IP / Port | 127.0.0.1 / 2368 |
| Pose | x=-0.82, y=0, z=0.22, roll=pitch=yaw=0 |

전방 LiDAR 수신기가 2368을 쓰고 있으면 후방 Destination Port를 바꾸고 `destination_port:=`로 맞춘다.

### bag 분석

```bash
rosbag record /rear_lidar/tracked_objects /morai/ego_vehicle_status /local_route -O lc.bag
rosrun safety_monitor bag_to_npz.py lc.bag
python3 scripts/replay_sweep.py lc.npz --mgeo <폴더> --set horizon=2.5,3.0 --frames lc.csv
rosrun safety_monitor analyze_tracking_bag.py lc.bag --track-id <ID>   # 인지 추적 확인
```
`analyze_tracking_bag.py`는 `/rear_lidar/ego_odometry`를 읽는다. 기본 설정(ego status만 사용)에서는
이 토픽이 없으므로 `launch_ego_odometry:=true odom_topic:=/rear_lidar/ego_odometry ego_status_topic:=""`로
녹화하거나 스크립트의 토픽을 바꿔서 쓴다.

## 설계 결정

인지 (기존 패키지 대비 변경점)
- 선회 중 새 트랙 확정 조건의 폭을 `min(dx, dy)` 대신 `dy`(base_link 횡방향 길이)로 계산. 후방 차량은 앞면만 보여서 깊이 약 0.3 m가 폭으로 잡혔다.
- 선회 전에 확정된 동적 트랙은 yaw rate가 0.08 rad/s를 넘는 동안 점수를 깎지 않음 (`keep_confirmed_during_turn`). 차선 변경 중 yaw rate는 0.12~0.25 rad/s.
- 자차 이동 정보 출처는 `/morai/ego_vehicle_status` 하나만 사용. `odom_topic`, `fallback_odom_topic`을 빈 문자열로 두면 구독하지 않도록 수정. 출처가 바뀌면 트랙 전체가 초기화되기 때문.
- 인지 노드 `required="false"`. 인지가 죽으면 충돌 감시 발행이 끊기고 gate가 제동한다. 이전처럼 `required="true"`이면 gate까지 함께 종료되어 MORAI가 마지막 명령을 유지할 수 있다.

인지 수정 검증 (PCL 1.14 + ROS stub으로 추적기만 빌드해서 시험, 대상 13 m/s, 자차 8 m/s, 10프레임 직진 후 15프레임 선회)

| 경우 | 수정 전 | 수정 후 |
| --- | --- | --- |
| 직진 중 확정된 차량, 선회 중 동적 유지 | 7/15 프레임 후 해제 | 15/15 유지 |
| 선회 중 새로 나타난 차량 | 확정 안 됨 | 나타난 뒤 13프레임에 확정 (7.3 Hz 기준 약 1.8 s) |

충돌 판정은 `is_dynamic`을 쓰지 않으므로 선회 중 새 차량의 확정이 늦어도 판정에는 영향이 없다.
영향은 추적 유지 시간(동적 2.0 s, 정적 0.3 s)이다.

충돌 판정
- 목표 차선 Frenet(MGeo 차선 중심선)에서 횡방향 진입 시각과 종방향 겹침 구간을 구해 결합한다.
- 목표 차선은 local route가 현재 차선 중심선에서 `lane_half_width` 이상 벗어나는 첫 지점의 차선이다. 외부 토픽으로 차선 변경 여부를 받지 않는다.
- 차선 변경 종료는 목표 차선 기준 횡오프셋 ≤ 0.3 m, heading ≤ 3°. 종료 전에는 대상이 자차 현재 차선으로 재분류되어도 감시한다.
- 감시 대상은 `is_dynamic` 대신 목표 차선 방향 속도와 이력 임계(진입 0.5, 제외 −0.5 m/s)로 고른다. 예측 구간 안에 겹침에 도달할 수 있는 거리 안의 차량은 속도와 무관하게 유지한다.
- 종방향 거리는 직선 거리 대신 목표 차선 중심선의 s 차이로 계산한다.
- 발행 2프레임 연속, 해제 5프레임 연속 (LiDAR 7.3 Hz 기준 약 0.27 s / 0.68 s).

인지 출력 처리 (`adapters/tracked_object.py`)
- pose는 보이는 점들의 축정렬 박스 중심이다. 후방 차량은 앞면만 보이므로 앞 끝 = pose.x + dim_x/2, 전장은 가정값 4.6 m.
- 폭은 dim_y를 [1.6, 2.2] m로 제한해 쓴다.
- 속도는 `velocity`(자차 이동 보정된 지면 속도)를 map으로 회전해 중심선 접선에 투영한다.

예외 처리
- 정지 중(< 0.3 m/s)에는 진입 시각 계산에 재출발 가정 속도 4.0 m/s를 쓴다. 이미 횡방향으로 겹쳤으면 진입 시각 0.
- 통과 차량: 감시 중이던 트랙이 측정되지 않은 첫 프레임(coasting 시작 또는 삭제)에서 간격이 2.0 m 이하이면 마지막 측정 시각 + t_end + 0.68 s까지 정지 유지. 속도는 박스가 ROI(x ≤ 1.5)에 잘리기 전 프레임 중앙값. 상대속도가 작으면 재관측까지 유지하되 최대 5 s (보고서의 "재관측까지"와 다름: 앞으로 지나간 차량은 후방 LiDAR에 다시 잡히지 않음).
- 차선 변경 중 자차 이동 보정이 안 된 트랙이 있으면 정지.
- 자차 상태가 0.3 s 이상 끊기면 정지. 차선 변경 중 local route가 끊겨도 정지.
- 판정 중 예외가 나면 정지.
- 자차/목표 차선을 MGeo에 매칭하지 못하면 기본값은 정지하지 않음 (`stop_on_map_failure`).

센서 신호 감시
- 시각은 `time.monotonic()` 사용 (시뮬레이터 시계 정지에도 두절 검출).
- 모든 신호가 첫 유효 메시지를 받기 전까지 미준비 → 정지.

gate
- 두 감시 결과와 제어 명령이 하나라도 수신 전이거나 끊기면 정지 (받지 못한 것을 false로 보지 않음).
- 정지 명령: `cmd_type=1, accel=0, brake=1`, 조향은 마지막 제어 명령 값 유지.
- gate가 죽으면 MORAI는 마지막 명령을 유지할 수 있다. gate에는 판정 로직을 두지 않는다.

## 알려진 한계

- 추적기 연관 거리 2.5 m (+0.25·속도). 새 트랙은 속도 0에서 시작하므로 7.3 Hz에서 지면 속도 약 18 m/s(65 km/h)를 넘는 차량은 첫 연관에 실패해 매 프레임 새 트랙이 된다. 15 Hz면 약 37 m/s. 대회 속도에서 문제가 되면 `association_distance`를 늘린다.
- 선회 중 새로 나타난 차량의 동적 확정은 약 13프레임 걸린다 (위 표).
- 대상 전장은 가정값 4.6 m. 트럭 등 긴 차량은 옆면이 보일 때만 실제 길이가 반영된다.

## 실행 전 확인할 것

- [ ] `catkin_make` 빌드 (C++은 PCL 1.14 + ROS stub으로 문법·링크만 확인, 실제 ROS 빌드 전)
- [ ] IMU 토픽 이름 (`bringup.launch imu_topic:=`)
- [ ] 제어기 출력 remap과 발행 주기 (`gate.yaml` `cmd_timeout`)
- [ ] 제어기가 쓰는 `cmd_type`
- [ ] 신호별 실제 주기 `rostopic hz` → `health.yaml` `rate_hz`
- [ ] `/local_route` 메시지 형식이 `nav_msgs/Path`이고 frame이 base_link인지
- [ ] MGeo 좌표와 `/morai/ego_vehicle_status` 좌표가 같은 원점인지 (RViz `Safety targets and target lane`의 선이 실제 차선 위에 그려지는지)
- [ ] `expected_ground_z=-0.46`이 후방 LiDAR 높이에 맞는지 (RViz Debug Ground)
- [ ] `/rear_lidar/status`의 `processing_ms`와 실제 처리 주기
- [ ] `multimodal_safety.launch`: MORAI 후방 LiDAR Destination Port 2370, `rostopic info /ctrl_cmd` 발행자가 `/estop_gate` 하나뿐인지
- [ ] 모델 추론이 멈췄을 때 MPC가 `/ctrl_cmd_raw`를 계속 발행하는지 (계속 발행하면 gate의 `cmd_stale`로는 잡히지 않음)
