# 03_localization — 위치 추정 (Localization Stack)

MORAI 시뮬레이터 환경에서 **GPS, IMU, Wheel Odometry, LiDAR NDT** 데이터를 센서 융합(Sensor Fusion)하여 차량의 정밀 전역/지역 위치를 추정하는 위치 추정 프레임워크입니다.

---

## 1. 전체 프레임워크 구조 (Localization Framework)

```mermaid
flowchart TD
    subgraph SIM["Simulation System"]
        SIM_RAW["MORAI Simulator<br/>(Raw Sensor Data via UDP)"]
        BRIDGE["morai_udp_bridge<br/>(ROS Network Module)"]
        SIM_RAW -->|UDP Packets| BRIDGE
    end

    subgraph TOPICS["ROS Sensor Topics"]
        GPS_TOPIC["/gps (NavSatFix)"]
        IMU_TOPIC["/imu (Imu)"]
        STATUS_TOPIC["/morai/ego_vehicle_status"]
        LIDAR_TOPIC["/velodyne_points_cropped"]
    end

    BRIDGE --> GPS_TOPIC
    BRIDGE --> IMU_TOPIC
    BRIDGE --> STATUS_TOPIC
    BRIDGE --> LIDAR_TOPIC

    subgraph PREPROC["Preprocessing & Odometry Nodes"]
        GPS_NODE["gps_node.py<br/>(MGeo Map Origin Offset)"]
        WHEEL_NODE["wheel_odom_node.py<br/>(Kinematic Dead-Reckoning)"]
    end

    GPS_TOPIC --> GPS_NODE
    IMU_TOPIC --> GPS_NODE
    IMU_TOPIC --> WHEEL_NODE
    STATUS_TOPIC --> WHEEL_NODE

    subgraph NDT["LiDAR NDT Matching"]
        MAP_PCD[("globalmap.pcd")]
        NDT_NODE["hdl_localization_nodelet<br/>(NDT_OMP)"]
        MAP_PCD --> NDT_NODE
        LIDAR_TOPIC --> NDT_NODE
        IMU_TOPIC --> NDT_NODE
    end

    subgraph EKF["Dual-Stage EKF Fusion (robot_localization)"]
        LOCAL_EKF["Local EKF (ekf_se_local)<br/>[odom ➔ base_link]"]
        GLOBAL_EKF["Global EKF (ekf_se_global)<br/>[map ➔ odom]"]
        INIT["ekf_global_initializer.py"]
    end

    WHEEL_NODE -->|/wheel_odometry| LOCAL_EKF
    WHEEL_NODE -->|/wheel_odometry| GLOBAL_EKF

    GPS_NODE -->|/gps_odom| GLOBAL_EKF
    GPS_NODE -->|/gps_pose| GPS_POSE["/gps_pose"]

    NDT_NODE -->|/ndt_pose| GLOBAL_EKF

    INIT -->|/set_pose_global| NDT_NODE
    INIT -->|/set_pose_global| GLOBAL_EKF

    subgraph OUTPUT["Fused State Outputs & TF"]
        LOCAL_EKF -->|/odometry/filtered_local| OUT_LOCAL["Local Odometry"]
        LOCAL_EKF -.->|TF| TF_LOCAL["TF: odom ➔ base_link"]
        
        GLOBAL_EKF -->|/localization/kinematic_state| OUT_GLOBAL["Global Pose"]
        GLOBAL_EKF -.->|TF| TF_GLOBAL["TF: map ➔ odom"]
    end

    style SIM_RAW fill:#e1f5fe,stroke:#0288d1,stroke-width:2px
    style BRIDGE fill:#e8eaf6,stroke:#3f51b5,stroke-width:2px
    style LOCAL_EKF fill:#e8f5e9,stroke:#388e3c,stroke-width:2px
    style GLOBAL_EKF fill:#e8f5e9,stroke:#388e3c,stroke-width:2px
    style OUT_GLOBAL fill:#fff3e0,stroke:#f57c00,stroke-width:2px
```

---

## 2. TF (TransForm) 좌표계 변환 구조

```mermaid
graph LR
    map["map (Global Frame)"] -->|ekf_se_global| odom["odom (Local Frame)"]
    odom -->|ekf_se_local| base_link["base_link (Vehicle Frame)"]
    
    subgraph SENSORS["Sensors (Static TF)"]
        gps["gps"]
        imu["imu"]
        velodyne["velodyne"]
    end

    base_link -->|sensor_tf.launch| gps
    base_link -->|sensor_tf.launch| imu
    base_link -->|sensor_tf.launch| velodyne

    style map fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px
    style odom fill:#e1f5fe,stroke:#0288d1,stroke-width:2px
    style base_link fill:#fff3e0,stroke:#ef6c00,stroke-width:2px
```

---

## 3. 주요 구성 요소 (Components)

| 모듈/노드 | 역할 및 기능 | 주요 입출력 |
|-----------|--------------|-------------|
| **`gps_node.py`** | MORAI GPS 수신, MGeo `global_info.json` 원점 기준 UTM 좌표 변환 | **In:** `/gps`, `/imu`<br>**Out:** `/gps_pose`, `/gps_odom` |
| **`wheel_odom_node.py`** | Ego 차량 속도(`signed_vel`) 및 IMU 헤딩 기반 Dead Reckoning 계산 | **In:** `/morai/ego_vehicle_status`, `/imu`<br>**Out:** `/wheel_odometry` |
| **`ekf_global_initializer.py`** | 최초 GPS/IMU 수신 시 Global EKF 및 NDT 초깃값 자동 주입 | **Out:** `/set_pose_global`, `/initialpose` |
| **`ekf_se_local`** | Wheel Odom + IMU 고속 로컬 융합 (드리프트 억제) | **Out:** TF `odom` → `base_link`, `/odometry/filtered_local` |
| **`hdl_localization`** | LiDAR Point Cloud와 사전 등록된 PCD 지도 NDT 맵 매칭 | **In:** `/velodyne_points_cropped`, `/globalmap`<br>**Out:** `/ndt_pose` |
| **`ekf_se_global`** | GPS Odom + NDT Pose + Wheel Odom 최종 글로벌 위치 융합 | **Out:** TF `map` → `odom`, `/localization/kinematic_state` |

---

## 4. 패키지 구성

| 패키지 | 설명 | 출처 |
|--------|------|------|
| `localization` | EKF 기반 GPS/Odom/NDT 융합 로컬라이제이션 스택 및 전처리 노드 | 자체 개발 |
| `hdl_localization` | HDL Graph SLAM 기반 3D LiDAR NDT 맵 매칭 노드 | GitHub (koide3) |
| `ndt_omp` | OpenMP 기반 Multi-thread NDT 가속 라이브러리 | GitHub |
| `fast_gicp` | GICP/NDT 고속 정렬 C++ 라이브러리 | GitHub |

---

## 5. 빠른 시작 (Quick Start)

```bash
# 1. 전체 Global Localization 스택 실행 (GPS + Wheel Odom + NDT + Dual EKF)
roslaunch localization localization_global.launch

# 2. Local Localization 스택만 실행 (Wheel Odom + IMU EKF)
roslaunch localization localization_local.launch

# 3. NDT 맵 매칭 단독 실행
roslaunch localization ndt_localization.launch
```
