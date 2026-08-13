# multimodal_learning — Planner V2

Notion 설계 문서의 **Multi-View Temporal Trajectory Planner V2** 구현입니다. 허용된 Camera, VLP16, GPS, IMU, Vehicle Status와 내부 Offline MGeo/Route만 사용합니다.

## 모델 구조

```text
Front 640x360 ─┐
Left  320x240 ─┼─ Shared ImageNet ResNet18 + Camera-ID embedding
Right 320x240 ─┘
VLP16 ─ 3-channel BEV 256x256 ─ Light CNN 32→64→128→192
Ego/Localization 13-D ─ 3-layer MLP
                    ↓
      2-layer Dynamic Query Cross-Attention
             frame당 8x192 tokens
                    ↓
       5 frame, 0.25초 간격, 최근 1초
       slot-shared 1-layer GRU hidden 192
                    ↓
MGeo 64x8 ─ Point MLP + 2-layer Transformer ─┐
Route 64x4 ─ 2-layer MLP ───────────────────┤
                    2-layer Route Cross-Attention
                    ↓
K=3 x 20 x [x, y, relative_yaw, target_speed]
+ learned mode score
+ Signal/Stop/Hazard/Lane auxiliary heads
```

- 전체 파라미터: 약 15.9M
- 출력 간격: 0.2초
- 예측 horizon: 4초, 20점
- 출력 좌표: 현재 `base_link` 기준 absolute local point. delta 좌표가 아닙니다.
- 선택: K=3 중 learned mode score가 가장 높은 후보
- Auxiliary head는 학습·진단 전용이며 주행 후보를 외부 규칙으로 교체하지 않습니다.

## 토픽 계약

설정 파일은 [topics.yaml](config/topics.yaml)입니다.

| 입력 | Type | V2 규격 |
| --- | --- | --- |
| `/camera/front/image/compressed` | `sensor_msgs/CompressedImage` | JPEG quality 80, 모델 resize `640x360` |
| `/camera/left/image/compressed` | `sensor_msgs/CompressedImage` | JPEG quality 80, 모델 resize `320x240` |
| `/camera/right/image/compressed` | `sensor_msgs/CompressedImage` | JPEG quality 80, 모델 resize `320x240` |
| `/velodyne_points_filtered` | `sensor_msgs/PointCloud2` | occupancy/max-height/log-density BEV |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | `map → base_link` fused state |
| `/imu` | `sensor_msgs/Imu` | angular velocity, acceleration |
| `/morai/ego_vehicle_status` | `morai_msgs/EgoVehicleStatus` | velocity, steering, acceleration |
| `/gps` | `sensor_msgs/NavSatFix` | valid/status/age 계산 |
| `/mgeo_tokens` | `std_msgs/Float32MultiArray` | 정확히 `64x8`, label=`mgeo_v2_points` |
| `/local_route` | `nav_msgs/Path` | 정확히 64 pose, frame=`base_link` |

MGeo feature 순서는 다음과 같습니다.

```text
[x/r, y/r, tangent_x, tangent_y, curvature*10,
 lane_half_width/r, lane_type/10, speed_zone_flag]
```

Route feature는 Path로 받은 64점을 다음 형식으로 변환합니다.

```text
[x/r, y/r, cos(relative_yaw), sin(relative_yaw)]
```

출력:

| 출력 | Type | 내용 |
| --- | --- | --- |
| `/multimodal_learning/predicted_path` | `nav_msgs/Path` | 선택 후보 20점, frame=`base_link` |
| `/multimodal_learning/target_speeds` | `std_msgs/Float32MultiArray` | 선택 후보의 20개 속도 |
| `/multimodal_learning/mode_scores` | `std_msgs/Float32MultiArray` | K=3 softmax score |

## 빌드 및 센서 실행

```bash
cd ~/acca_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg mgeo_route_publisher multimodal_learning
source devel/setup.bash
roslaunch multimodal_learning multimodal_sensors.launch
```

센서 launch는 전방·좌측·우측 raw 카메라를 받아 다음 compressed 토픽을 항상 함께 발행합니다.

```text
/camera/front/image/compressed
/camera/left/image/compressed
/camera/right/image/compressed
```

JPEG quality 기본값은 80이며 실행 시 변경할 수 있습니다.

```bash
roslaunch multimodal_learning multimodal_sensors.launch camera_jpeg_quality:=75
```

### Raw 카메라를 제외한 학습용 bag 녹화

센서 실행과 동시에 녹화하려면 다음 한 줄만 실행합니다.

```bash
roslaunch multimodal_learning multimodal_sensors.launch record_bag:=true
```

기본 저장 위치는 `~/acca_ws/multimodal_YYYY-MM-DD-HH-MM-SS.bag`입니다. 저장 prefix를 바꾸려면:

```bash
roslaunch multimodal_learning multimodal_sensors.launch \
  record_bag:=true bag_prefix:=/data/multimodal_
```

이 녹화는 세 compressed 카메라와 학습 필수 센서만 명시적으로 저장합니다. `/camera/*/image_raw`, `/velodyne_packets`, `/velodyne_points`, `/velodyne_points_cropped`는 bag에서 제외됩니다. 기존처럼 `rosbag record -a`를 사용하면 raw 카메라가 다시 포함되므로 학습용 녹화에는 사용하지 않습니다.

센서를 이미 실행한 상태에서 녹화만 별도로 시작할 수도 있습니다.

```bash
roslaunch multimodal_learning record_training_bag.launch
```

이 launch는 `morai_connect`의 sensor/Ego component launch를 직접 포함합니다. 따라서 GPS, IMU, 3개 카메라, Ego/Collision, Velodyne 및 센서 TF를 위해 `roslaunch morai_udp_bridge morai_bridge.launch`를 별도로 실행하면 안 됩니다. 별도로 실행하면 같은 ROS 노드 이름과 Velodyne UDP 2368 포트가 중복됩니다.

실행 전에 예전 센서 launch가 남아 있지 않은지 확인합니다.

```bash
rosnode list | grep -E 'morai_|velodyne_nodelet_manager'
ss -uapn | grep ':2368'
```

`multimodal_sensors.launch` 실행 후 UDP 2368을 사용하는 Velodyne manager는 하나만 보여야 합니다.

기존에 실행 중인 `mgeo_route_publisher`는 이전 Python 코드를 메모리에 유지합니다. 변경 후에는 반드시 launch를 종료하고 다시 시작해야 V2 토픽이 나옵니다.

토픽 규격 확인:

```bash
rostopic echo -n 1 /mgeo_tokens/layout
rostopic echo -n 1 /local_route/header
rostopic echo -n 1 /local_route | grep -c '^  - header:'
```

정상 기준:

```text
/mgeo_tokens label: mgeo_v2_points
/mgeo_tokens size: 64 x 8, data 512 floats
/local_route frame_id: base_link
/local_route poses: 64
```

## 데이터 추출

V2 추출기는 토픽을 엄격하게 검사합니다. V1의 MGeo `128x8` 또는 Route 32 pose를 자동 변환하거나 잘라 쓰지 않습니다.

새 bag은 기본 compressed 카메라 설정으로 바로 추출합니다. 변경 전에 raw 카메라로 녹화한 기존 bag은 호환 설정을 지정합니다.

```bash
cd ~/acca_ws
source devel/setup.bash

rosrun multimodal_learning extract_bag_dataset.py \
  /path/to/v2_recorded.bag \
  --output ~/.ros/multimodal_dataset_v2
```

기존 raw-camera bag:

```bash
rosrun multimodal_learning extract_bag_dataset.py \
  /home/acca/acca_ws/2026-07-20-13-34-37.bag \
  --topics $(rospack find multimodal_learning)/config/topics_raw_legacy.yaml \
  --output ~/.ros/multimodal_dataset_v2
```

Smoke test:

```bash
rm -rf /tmp/multimodal_v2_smoke
rosrun multimodal_learning extract_bag_dataset.py \
  /path/to/v2_recorded.bag \
  --output /tmp/multimodal_v2_smoke \
  --max-samples 2
```

샘플 shape:

```text
camera_front  (5, 3, 360, 640) uint8
camera_left   (5, 3, 240, 320) uint8
camera_right  (5, 3, 240, 320) uint8
lidar         (5, 3, 256, 256) float32
ego           (5, 13) float32
map_tokens    (64, 8) float32
route_tokens  (64, 4) float32
trajectory    (20, 4) float32
```

한 샘플은 최근 1초와 미래 4초가 모두 필요합니다. 따라서 5초 이하 bag에서는 학습 샘플이 생성되지 않습니다.

## RTX 2080 학습

현재 기준 환경:

```text
PyTorch 2.4.1+cu121
RTX 2080 SUPER, compute capability 7.5
```

```bash
rosrun multimodal_learning train.py \
  ~/.ros/multimodal_dataset_v2 \
  --epochs 30 \
  --batch-size 2 \
  --gradient-accumulation 8 \
  --frozen-backbone-epochs 3 \
  --lr 1e-4 \
  --output ~/planner_v2.pt
```

AMP FP16을 사용하며 첫 3 epoch는 camera backbone을 freeze합니다. CUDA OOM이면 `--batch-size 1`로 낮추고 gradient accumulation은 유지합니다.

Loss:

```text
1.0 trajectory + 0.4 speed + 0.2 yaw + 0.3 mode
+ 0.4 signal + 0.4 stop + 0.3 hazard + 0.3 lane + 0.05 smooth
```

Bag에 없는 Signal/Hazard/Lane label은 `-1` mask로 저장되어 해당 loss에서 제외됩니다. Stop label은 실제 미래 speed profile로 생성합니다.

## 평가와 추론

```bash
rosrun multimodal_learning evaluate.py \
  ~/.ros/multimodal_dataset_v2 ~/planner_v2.pt --batch-size 2
```

```bash
roslaunch multimodal_learning inference.launch \
  checkpoint:=/home/acca/planner_v2.pt \
  use_cuda:=true
```

추론은 새 sensor frame만 encode해 1초 feature cache를 유지합니다. Prediction이 0.3초 이상 stale이면 출력을 중단합니다. Controller 연결은 별도이며, 모델 출력 Path는 `base_link`이므로 필요하면 해당 예측 timestamp의 TF로 `map` frame에 변환합니다.

## 2026-07-20-12-35-06.bag 판정

`/home/acca/acca_ws/2026-07-20-12-35-06.bag`은 ROS bag 자체와 센서 데이터는 정상이나 **V2 학습 입력으로는 부적합**합니다.

- indexed, 손상 없음, 890MiB
- duration 6.455초, 총 2,911 message
- Camera 3대, LiDAR, odometry, IMU, Vehicle Status, GPS 모두 존재
- Camera/IMU/LiDAR frame 정상, timestamp 단조 증가
- 문제 1: `/mgeo_tokens`가 V1 `128x8`, label=`tokens`
- 문제 2: `/local_route`가 V1 32 pose
- 문제 3: 6.46초는 pipeline smoke test 수준이며 실제 학습 주행으로 너무 짧음

새 `mgeo_route_publisher`를 재시작한 뒤 다시 녹화해야 V2 추출이 가능합니다.
