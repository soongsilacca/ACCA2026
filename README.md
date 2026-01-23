# ERP42 Localization & Mapping Project (2026)

이 프로젝트는 ERP42 로봇을 위한 고정밀 위치 추정 및 매핑 시스템입니다. 다양한 센서(LiDAR, GPS, IMU, Encoder)를 융합하여 GPS 음영 지역을 포함한 도심 환경에서 강건한 자율 주행을 지원합니다.

## 🚀 주요 기능

### 1. Robust Localization (Dual EKF + NDT + FAST-LIO)
- **Local EKF**: Wheel Odometry + IMU (High Frequency, Smooth)
- **Global EKF**: 
  - **개방된 곳**: GPS (RTK/UTM)
  - **GPS 음영 지역**: NDT Matching (Scan-to-Map) + FAST-LIO Odometry
- **특징**: GPS가 튀거나 끊겨도 NDT와 FAST-LIO가 위치를 보정하여 연속적인 주행 가능

### 2. Sensor Fusion Architecture
- **Coordinates**: 모든 센서(IMU, GPS)를 **NWU (North-West-Up)** 좌표계로 통일
- **Timestamp Sync**: 하드웨어 시간차로 인한 TF 에러 방지 (Software Sync 구현)

---

## 📦 패키지 구성

| 패키지명 | 설명 | 비고 |
|----------|------|------|
| `localization` | 메인 로컬라이제이션 패키지 (Launch, Config, EKF, Tools) | 핵심 |
| `ndt_localization` | NDT (Normal Distributions Transform) 매칭 노드 (C++) | 신규 |
| `fast_lio` | LiDAR-Inertial Odometry | 서브모듈 |
| `lio_sam` | LiDAR Mapping & Odometry | 서브모듈 |
| `livox_ros_driver2` | Livox LiDAR 드라이버 | 드라이버 |

---

## 🛠️ 설치 및 빌드

### 1. 필수 의존성 설치
```bash
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-robot-localization ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install libpcl-dev
```

### 2. 워크스페이스 빌드
```bash
# 워크스페이스 루트에서 실행
cd ~/ws_2026

# 전체 빌드 (순서대로)
colcon build --packages-select ndt_omp
colcon build --packages-select livox_ros_driver2
colcon build --packages-select fast_lio lio_sam
colcon build --packages-select localization ndt_localization

# 환경 설정 로드
source install/setup.bash
```

---

## ▶️ 실행 가이드

### 통합 로컬라이제이션 실행
센서 드라이버(LiDAR, GPS, IMU, Encoder)를 켠 상태에서 다음 순서로 실행합니다.

1. **FAST-LIO 실행** (Odom Scan Matching)
   ```bash
   ros2 launch fast_lio mapping.launch.py
   ```

2. **Localization Stack 실행** (EKF Fusion + NDT)
   ```bash
   # 시뮬레이션 시간 사용 (Rosbag 재생 시)
   ros2 launch localization dual_ekf_localization.launch.py use_sim_time:=true

   # 실차 주행 시
   ros2 launch localization dual_ekf_localization.launch.py use_sim_time:=false
   ```

### 동작 확인
1. **NDT 상태 확인** (GPS 음영 지역 핵심)
   ```bash
   ros2 topic echo /ndt_pose --once
   ```
2. **맵 수동 로딩** (필요시)
   ```bash
   ros2 service call /publish_map std_srvs/srv/Trigger
   ```

---

## ⚠️ 주의사항

1. **GPS Datum 설정**: 실험 장소가 변경되면 `src/localization/config/map_anchor.yaml`의 위경도 원점을 반드시 수정해야 합니다.
2. **PCD 파일**: `src/localization/map/school.pcd` 파일이 있어야 NDT가 동작합니다.
3. **IMU 방향**: IMU는 x축이 전방을 향하도록 장착되어야 하며, 데이터는 ENU 또는 NWU여야 합니다 (내부적으로 NWU 변환됨).
