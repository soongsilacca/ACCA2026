# 🚗 ROS1 Autonomous Driving Workspace

MORAI 시뮬레이터 기반 자율주행 풀스택 ROS1(Noetic) 워크스페이스.  
**Connect → Perception → Localization → Mapping → Planning → Control** 파이프라인을 통합 구현합니다.

---

## 📁 패키지 구조

```
ros1_ws/src/
├── 01_connect/           # MORAI 시뮬레이터 연동 (UDP 통신)
│   └── morai_connect/
│       ├── lib/          # UDP 패킷 정의 (ctypes 구조체)
│       └── morai_udp_bridge/   # ROS 노드 & 런치 파일
│
├── 02_perception/        # LiDAR 인식
│   └── lidar_filtering/  # Ground Filter & CropBox Pipeline
│
├── 03_localization/      # 자차 위치 추정
│   ├── localization/     # GPS/Wheel Odom EKF 융합
│   ├── hdl_localization/ # HDL NDT 기반 맵 매칭
│   ├── fast_gicp/        # Fast GICP ICP 라이브러리
│   └── ndt_omp/          # OpenMP NDT 구현
│
├── 04_mapping/           # HD Map 생성 & 로드
│   ├── hdmap_loader/     # MGeo HD Map 파서 & 마커 시각화
│   └── mapping/          # Point Cloud 컬러맵 생성
│
├── 05_planning/          # 경로 계획
│   ├── global_path_planner/  # 전역 경로 계획
│   └── local_path_planner/   # Lattice Rollout 지역 경로 계획
│
├── 06_control/           # 차량 제어
│   └── control/          # Stanley / Pure Pursuit 컨트롤러
│
└── 07_utils/             # 유틸리티
    └── utils/            # GUI 런처 & RViz 설정
```

---

## 🧩 패키지 상세

### 01. Connect — MORAI UDP Bridge

MORAI 시뮬레이터와 UDP로 통신하는 브릿지 레이어입니다.

| 노드 | 포트 | 설명 |
|------|------|------|
| `gps_publisher_node.py` | UDP 수신 | GPS → `/gps` 토픽 발행 |
| `imu_publisher_node.py` | UDP 수신 | IMU → `/imu` 토픽 발행 |
| `ego_vehicle_status_publisher_node.py` | UDP 수신 | 차량 상태 발행 |
| `ego_ctrl_sender_node.py` | UDP 송신 | 제어 명령 전송 |
| `camera_*_publisher_node.py` | UDP 수신 | 전/좌/우 카메라 발행 |
| `collision_data_publisher_node.py` | UDP 수신 | 충돌 데이터 발행 |
| `traffic_light_controller_node.py` | `7607` 송신 | 신호등 강제 제어 |
| `scenario_load_sender_node.py` | `9095` 송신 | 시나리오 UDP 로드 |
| `domain_randomizer.py` | — | 시나리오 JSON 도메인 랜덤화 |

**Launch 파일:**

```bash
# 전체 UDP 브릿지 (GPS, IMU, Ego, Cmd)
roslaunch morai_udp_bridge morai_bridge.launch

# 특정 시나리오 로드
roslaunch morai_udp_bridge load_scenario.launch

# 시나리오 랜덤화 후 로드
roslaunch morai_udp_bridge randomize_and_load.launch

# 신호등 컨트롤러
roslaunch morai_udp_bridge traffic_light_controller.launch
```

---

### 02. Perception — LiDAR Filtering

| 노드 | 설명 |
|------|------|
| Ground Filter | VoxelGrid 기반 지면 제거 |
| CropBox Filter | 자차 범위 포인트 제거 |

```bash
roslaunch lidar_filtering lidar_pipeline.launch
```

---

### 03. Localization — 위치 추정

#### EKF 글로벌 Localization
GPS + Wheel Odometry를 EKF로 융합하여 전역 위치를 추정합니다.

```bash
roslaunch localization localization_global.launch
roslaunch localization localization_local.launch
```

#### HDL NDT Map Matching
사전 제작된 PCD 맵에 LiDAR를 NDT 매칭하여 정밀 위치를 추정합니다.

```bash
roslaunch hdl_localization hdl_localization.launch
```

**주요 토픽:**

| 토픽 | 설명 |
|------|------|
| `/gps` | 원시 GPS 입력 |
| `/imu` | IMU 입력 |
| `/localization/kinematic_state` | EKF 위치 출력 |
| `/ndt_pose` | NDT 위치 출력 |

---

### 04. Mapping — HD Map

#### MGeo HD Map Loader
MGeo 형식의 HD Map을 파싱하여 Link/Node/차선 정보를 RViz 마커로 시각화합니다.

```bash
roslaunch hdmap_loader mgeo_marker.launch
```

#### Point Cloud Map Creator
LiDAR 데이터를 기반으로 컬러 포인트클라우드 맵을 생성합니다.

```bash
roslaunch mapping colored_map_creator.launch
roslaunch mapping online_colored_map_creator.launch
```

---

### 05. Planning — 경로 계획

#### Global Path Planner
MGeo HD Map 토폴로지를 기반으로 목적지까지의 전역 경로를 계획합니다.

```bash
roslaunch global_path_planner global_path_planner.launch
roslaunch global_path_planner waypoint_global_path_planner.launch

# 정적 경로 마커 발행
rosrun global_path_planner static_path_publisher_node.py
```

#### Local Path Planner — Lattice Rollout
장애물 회피를 위한 Lattice 기반 지역 경로를 실시간으로 생성합니다.

```bash
roslaunch local_path_planner local_path_planner.launch
```

**주요 토픽:**

| 토픽 | 설명 |
|------|------|
| `/global_path` | 전역 경로 입력 |
| `/local_path` | 지역 경로 출력 |
| `/velodyne_points` | LiDAR 장애물 감지 입력 |

---

### 06. Control — 차량 제어

#### Stanley Controller
Stanley 방법론 기반 횡방향 제어기. 전역/지역 경로 모두 지원합니다.

```bash
# 전역 경로 추종
roslaunch control stanley_global_path.launch

# 지역 경로 추종 (동적 속도 프로파일)
roslaunch control stanley_local_path.launch
```

#### Pure Pursuit Controller
Look-ahead 거리 기반 Pure Pursuit 횡방향 제어기.

```bash
roslaunch control pure_pursuit.launch
```

**주요 토픽:**

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/local_path` | 입력 | 추종 경로 |
| `/morai/ego_vehicle_status` | 입력 | 차량 속도/조향 상태 |
| `/cmd` | 출력 | 제어 명령 (`CtrlCmd`) |

---

### 07. Utils — GUI 런처

Tkinter 기반 통합 제어 대시보드. 모든 ROS 노드를 GUI에서 실행/정지/재시작합니다.

```bash
roslaunch utils launcher_gui.launch
# 또는
rosrun utils launcher_gui.py
```

**기능:**
- 카테고리별 노드 카드 (System / Scenario / Sensor / Perception / Localization / Planning / Control)
- 실시간 센서 토픽 모니터링 (GPS / IMU / Ego / LiDAR / Localization)
- 차량 모드 전환 (Manual / Auto / Parking)
- 신호등 강제 제어 UDP 인터페이스
- 프로세스별 로그 터미널

---

## 🚀 실행 순서

```bash
# 1. ROS Core
roscore

# 2. MORAI 시뮬레이터 실행
cd ~/MoraiLauncher_Lin && ./MORAISim.sh

# 3. 시나리오 로드 (랜덤화)
roslaunch morai_udp_bridge randomize_and_load.launch

# 4. UDP 브릿지 (센서 수신 + 제어 송신)
roslaunch morai_udp_bridge morai_bridge.launch

# 5. LiDAR Perception
roslaunch lidar_filtering lidar_pipeline.launch

# 6. Localization
roslaunch localization localization_global.launch

# 7. Planning
roslaunch global_path_planner waypoint_global_path_planner.launch
roslaunch local_path_planner local_path_planner.launch

# 8. Control
roslaunch control stanley_local_path.launch

# 9. Visualization
rviz -d $(rospack find utils)/rviz/default.rviz
```

> 💡 **GUI 런처**를 사용하면 위 모든 과정을 버튼 하나로 관리할 수 있습니다:
> ```bash
> roslaunch utils launcher_gui.launch
> ```

---

## 🛠 환경 설정 & 설치

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Python | 3.8+ |
| Simulator | MORAI MoraiLauncher_Lin |

### 원클릭 설치 (권장)

```bash
git clone <this-repo> ~/ros1_ws
cd ~/ros1_ws
bash setup_workspace.sh
source devel/setup.bash
```

`setup_workspace.sh` 가 자동으로 수행합니다:
1. ROS apt 패키지 설치 (`robot_localization`, `velodyne`, `pcl_ros` 등)
2. Python pip 패키지 설치 (`requirements.txt`)
3. GitHub 서브 패키지 클론 (`fast_gicp`, `ndt_omp`, `hdl_localization`)
4. `catkin_make` 빌드

### 수동 설치

```bash
# ROS 의존 패키지
sudo apt install ros-noetic-robot-localization \
                 ros-noetic-velodyne-pointcloud \
                 ros-noetic-pcl-ros \
                 ros-noetic-cv-bridge

# Python 의존 패키지
pip3 install -r requirements.txt

# GitHub 서브 패키지
cd src/03_localization
git clone https://github.com/SMRT-AIST/fast_gicp.git
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/koide3/hdl_localization.git

# 빌드
cd ~/ros1_ws
catkin_make
source devel/setup.bash
```

### ⚠️ 별도 설치 필요

| 항목 | 경로 | 설명 |
|------|------|------|
| MORAI Simulator | `~/MoraiLauncher_Lin/` | MORAI에서 별도 제공 |
| 시나리오 파일 | `~/MoraiLauncher_Lin/.../Scenario/` | 시뮬레이터 내 포함 |
| PCD Map 파일 | 별도 경로 지정 | NDT 로컬라이제이션용 |

---

## 📡 주요 UDP 포트

| 포트 | 방향 | 용도 |
|------|------|------|
| `7777` | 수신 | GPS |
| `7778` | 수신 | IMU |
| `7779` | 수신 | Ego Vehicle Status |
| `7780` | 수신 | LiDAR 2D |
| `7785` | 수신 | Camera (Front) |
| `7607` | 송신 | 신호등 제어 |
| `7600` | 송신 | Ego 제어 명령 |
| `9095` | 송신 | 시나리오 로드 |

---

## 🔗 관련 링크

- [MORAI Simulator](https://www.morai.ai/)
- [ROS Noetic Documentation](https://wiki.ros.org/noetic)
- [MGeo HD Map Format](https://github.com/morai-developergroup/morai_standard_map)
