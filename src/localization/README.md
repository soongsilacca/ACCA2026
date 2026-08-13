# Localization Package

This package handles the vehicle localization for the MORAI simulator environment, specifically for the **2023 Hyundai Ioniq 5** model.

## 1. Vehicle Specifications: 2023 Hyundai Ioniq 5

| Parameter | Value |
| :--- | :--- |
| **Model** | 2023 Hyundai Ioniq 5 |
| **Wheelbase** | 3.000 m |
| **Length / Width / Height** | 4.635 m / 1.892 m / 2.434 m |
| **Front / Rear Overhang** | 0.845 m / 0.79 m |
| **Min Turning Radius** | 5.87 m |
| **Max Wheel Angle** | 40 deg |

---

## 2. Sensor Specifications

### 🛰️ GPS
- **Topic**: `/gps`
- **Msg Type**: `morai_msgs/GPSMessage`
- **Frame ID**: `gps_link`
- **Network**: ROS

### 🧭 IMU
- **Topic**: `/imu`
- **Msg Type**: `sensor_msgs/Imu`
- **Frame ID**: `imu_link`
- **Network**: ROS

### 🔦 3D LiDAR (Front & Rear)
- **Topic**: `/lidar_front`, `/lidar_rear`
- **Msg Type**: `sensor_msgs/PointCloud2`
- **Frame ID**: `lidar_link_front`, `lidar_link_rear`
- **Models**: VLP16 / HDL32 (TBD)
- **Network**: UDP or ROS

### Encoder 
topic : /Competition_topic
msg type : morai_msgs/EgoVehicleStatus
using data : 
    velocity.x [m/s]
    wheel_angle: [deg]
    accel
    brake

---

## 3. TF Tree Hierarchy

The following transformation tree is utilized for the localization and sensor fusion:

- `map`
    - `odom`
        - `base_link`
            - `gps_link`
            - `imu_link`
            - `lidar_link_front`
            - `lidar_link_rear`

---

## 4. Implemented Nodes

### `gps_node.py`
- **Function**: Converts `morai_msgs/GPSMessage` to `geometry_msgs/PoseStamped`.
- **Logic**: Uses the `utm` package to convert Latitude/Longitude to local Cartesian coordinates and applies the map offset.
- **Output Topic**: `/gps_pose`


---

## 5. 향후 과제 (To-Do)

- [ ] **센서 드라이버 설치**: 하드웨어 통합을 위한 필수 ROS 드라이버 설치 및 설정
- [ ] **TF 트리 구현**: 정적 및 동적 변환 트리를 방송하기 위한 런치 파일 개발
- [ ] **센서 주기(Hz) 설정**: GPS, IMU, LiDAR 센서의 데이터 전송 주기 최적화
- [ ] **로컬 오도메트리 구현**: 로버스트한 센서 퓨전을 위한 EKF/UKF 노드 개발 및 설정

---

## 6. 컬러 맵 생성 (Colored Map Creator)

`colored_map_creator.launch`는 오프라인 백파일(rosbag)을 읽어서 FastGICP로 위치를 보정하고, 카메라 이미지를 투영하여 컬러 PCD 지도를 생성하는 노드입니다.

### 실행 방법
```bash
roslaunch localization colored_map_creator.launch
```

### 주요 파라미터 (launch 파일에서 수정 가능)
- `bag_path`: 읽어올 bag 파일 경로 (기본값: `/home/jinju/bag_file/kcity_3.bag`)
- `save_dir`: 저장될 PCD 파일 경로
- `voxel_size_global`: 최종 저장될 지도의 해상도 (기본값: 0.1m)