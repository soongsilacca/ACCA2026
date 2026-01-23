# Localization Package

ERP42 로봇의 정밀 위치 추정을 위한 센서 융합 패키지입니다. **Dual EKF** (Local/Global) 구조를 기반으로 **NDT-OMP** 매칭과 **FAST-LIO** 오도메트리를 결합하여 다양한 환경(GPS 음영 지역 포함)에서 강건한 성능을 보장합니다.

## 📋 목차

- [개요](#개요)
- [시스템 구조](#시스템-구조)
- [노드 상세 설명](#노드-상세-설명)
- [TF Frame 구조](#tf-frame-구조)
- [서비스 목록](#서비스-목록)
- [사용 방법](#사용-방법)
- [설정 파일 가이드](#설정-파일-가이드)

---

## 개요

### 주요 특징
1. **Multi-Modal Sensor Fusion**: 
   - Wheel Odometry + IMU (Local)
   - GPS + NDT Matching + FAST-LIO (Global)
2. **Robust Localization**:
   - GPS 신호가 좋은 곳: **GPS + EKF**
   - 터널/건물 사이 (GPS 음영): **NDT (Scan-to-Map) + FAST-LIO**
3. **High Performance**:
   - `ndt_omp`: OpenMP 병렬 처리를 이용한 고속 매칭
   - `fastlio_odometry_adapter`: FAST-LIO와 EKF의 유연한 통합

### 좌표계 표준 (NWU)
- 모든 센서 데이터는 **NWU (North-West-Up)** 좌표계로 통일되어 처리됩니다.
  - **IMU**: ENU -> NWU 변환 (`imu_nwu_adapter`)
  - **GPS**: UTM -> NWU 변환 (`gps_to_odometry`)

---

## 시스템 구조

```mermaid
graph TD
    subgraph Sensors
        Wheel[Wheel Encoder]
        IMU[IMU]
        GPS[GPS]
        LiDAR[Velodyne LiDAR]
    end

    subgraph Preprocessing
        Wheel -->|/erp_status| WheelOdom[wheel_odometry_node]
        IMU -->|/imu/data| IMUAdapter[imu_nwu_adapter]
        GPS -->|/fix| GPSConv[gps_to_odometry]
        LiDAR -->|/points| NDT[ndt_localization_node]
        LiDAR -->|/points| FASTLIO[FAST-LIO]
    end

    subgraph Adapters
        FASTLIO -->|/Odometry| FastLIOAdapter[fastlio_odometry_adapter]
        FastLIOAdapter -->|/fastlio/odom_aligned| EKF_Global
    end

    subgraph Fusion(Dual EKF)
        WheelOdom -->|/wheel/odom| EKF_Local[EKF Local]
        IMUAdapter -->|/imu/data_nwu| EKF_Local
        
        WheelOdom -->|/wheel/odom| EKF_Global[EKF Global]
        IMUAdapter -->|/imu/data_nwu| EKF_Global
        GPSConv -->|/gps/odom| EKF_Global
        NDT -->|/ndt_pose| EKF_Global
    end

    subgraph Map
        PCD[school.pcd] --> PcdPub[pcd_map_publisher]
        PcdPub -->|/map_cloud| NDT
    end

    EKF_Local -->|odom->base_link| TF
    EKF_Global -->|map->odom| TF
```

---

## 노드 상세 설명

### 1. **ndt_localization_node** (C++, New)
- **기능**: 정밀지도(PCD)와 현재 라이다 스캔을 매칭하여 로봇의 절대 위치 계산
- **입력**: `/velodyne_points`, `/map_cloud`, `/odometry/local` (Initial Guess용)
- **출력**: `/ndt_pose` (EKF Global 입력용)
- **특징**:
  - **Robust Initial Guess**: Wheel Odometry의 이동량을 반영하여 GPS가 튀어도 안정적
  - **Adaptive Covariance**: 매칭 점수가 나쁠수록 공분산을 키워 EKF 오염 방지
  - **Deadlock Free**: `/publish_map` 서비스 자동 호출 및 Raw Scan 사용

### 2. **fastlio_odometry_adapter** (Python, New)
- **기능**: FAST-LIO의 오도메트리를 로봇 좌표계(base_link) 및 EKF 프레임(odom)에 맞게 변환
- **입력**: `/Odometry` (from FAST-LIO), `/imu/data_nwu`
- **출력**: `/fastlio/odom_aligned`
- **특징**:
  - **Auto Alignment**: 실행 시 IMU/EKF Yaw를 기준으로 좌표축 자동 정렬
  - **Dynamic Re-initialization**: 주행 중 방향 틀어짐(Crabbing) 감지 시 자동 재정렬

### 3. **pcd_map_publisher** (Python, Updated)
- **기능**: `school.pcd` 파일을 로드하여 토픽으로 발행
- **서비스**: `/publish_map` (NDT 노드가 시작 시 자동 호출)
- **QoS**: `Transient Local` 적용 (늦게 켜진 노드도 맵 수신 가능)

### 4. **ekf_global_node** (robot_localization)
- **기능**: 모든 센서 데이터를 융합하여 최적의 위치(`map`->`odom`) 추정
- **융합 소스**:
  - **Wheel Odom**: 기본 추측 항법
  - **IMU**: 자세(Roll, Pitch, Yaw) 보정
  - **GPS**: 절대 위치 보정 (개방된 곳)
  - **NDT**: 절대 위치 보정 (GPS 음영 지역)
  - **FAST-LIO**: 정밀 오도메트리 보조

---

## TF Frame 구조

```
map (Global Frame)
 └─ odom (Published by EKF Global)
     └─ base_link (Published by EKF Local)
         ├─ imu_link
         ├─ gps
         └─ velodyne
```

---

## 사용 방법

### 1. 빌드
```bash
cd ~/ws_2026
colcon build --packages-select localization ndt_localization lio_sam fast_lio
source install/setup.bash
```

### 2. 실행
전체 시스템(센서 처리, NDT, EKF, 어댑터 등)을 한 번에 실행합니다.
```bash
ros2 launch localization dual_ekf_localization.launch.py use_sim_time:=true
# 실차 주행 시 use_sim_time:=false
```

### 3. 확인
**NDT 동작 확인**:
```bash
ros2 topic echo /ndt_pose --once
# 또는 RViz에서 /ndt_debug/aligned_cloud (초록색 점군) 확인
```

**FAST-LIO 동작 확인**:
```bash
ros2 topic hz /fastlio/odom_aligned
```

---

## 서비스 목록

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/publish_map` | `std_srvs/Trigger` | **PCD 맵 로딩 및 발행 트리거**. <br> NDT 노드가 시작될 때 자동으로 호출됩니다. 수동으로 맵을 다시 불러오고 싶을 때 다음 명령어를 사용하세요. |

**서비스 호출 명령어:**
```bash
ros2 service call /publish_map std_srvs/srv/Trigger
```

---

## 설정 파일 가이드

- **`config/ekf_local.yaml`**: 로컬 EKF 설정. Odometry + IMU 융합 파라미터.
- **`config/ekf_global.yaml`**: 글로벌 EKF 설정. GPS, NDT 사용 여부 및 공분산 조정.
- **`config/map_anchor.yaml`**: GPS 좌표(위경도)와 Map 좌표(0,0) 간의 기준점 설정. 실험 장소가 바뀌면 수정 필수.
