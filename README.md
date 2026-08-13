# ACCA 2026 Autonomous Driving Software Stack

![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Language](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B-orange.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

## 🚘 Team ACCA 소개
**ACCA (Automated Car Club Association)**는 숭실대학교 자율주행 동아리/연구팀입니다.  
본 저장소는 대학생 자율주행 경진대회 및 MORAI 시뮬레이션/실차 환경에서의 **인지(Perception), 측위(Localization), 판단(Planning), 제어(Control)** 통합 소프트웨어 스택을 포함하고 있습니다.

---

## 📌 주요 모듈 및 패키지 구조

| 패키지명 | 설명 |
|---|---|
| `localization` | NDT Map Matching 및 EKF 센서 퓨전(GPS/IMU/Odometry) 기반 차량 위치 추정 |
| `control` | 경로 추종(Pure Pursuit / MPC) 및 하위 제어기 |
| `multimodal_learning` | LiDAR, Camera, 센서 퓨전 기반 종/횡적 이동 경로 생성 딥러닝 인퍼런스 노드 |
| `depth_perception` | 비전 기반 객체 인식 및 거리 측정 모듈 |
| `TTC` | 충돌 위험 시간(Time-To-Collision) 산출 및 HFSM(계층적 유한 상태 머신) 주행 상태 관리 |
| `global_path_planner` / `recorder` | 글로벌 경로 생성, 주행 경로 획득 및 경로 저장 |
| `sensor_tf_broadcaster` | 센서 간 좌표계(TF) 변환 방송 |
| `morai_msgs` / `morai_connect` | MORAI 자율주행 시뮬레이터 통신 노드 및 커스텀 메시지 정의 |
| `lidar_filtering` | 라이다 포인트 클라우드 전처리 및 전방 장애물 필터링 |
| `map_viz` | RViz 기반 맵 및 주행 상태 시각화 |

---

## 🚀 시작하기

### 1. 전제 조건 (Prerequisites)
- Ubuntu 20.04 LTS
- ROS Noetic Desktop Full
- Python 3.8+ 및 PyTorch / CUDA (multimodal_learning 모듈 사용 시)

### 2. 빌드 방법 (Build)

```bash
# 워크스페이스 이동
cd ~/acca_ws

# 의존성 설치 및 빌드
catkin_make
source devel/setup.bash
```

---

## 🛠 실행 방법 (Usage)

### 센서 TF 방송 및 위치 추정 (Localization)
```bash
roslaunch sensor_tf_broadcaster static_tf.launch
roslaunch localization ekf_localization.launch
```

### 글로벌 경로 및 제어 (Control)
```bash
roslaunch control mpc_control.launch
```

### 멀티모달 딥러닝 인퍼런스 (Multimodal Learning)
```bash
roslaunch multimodal_learning tcp_inference.launch
```

---

## 📝 Git 기여 및 유의사항
- 용량이 큰 데이터 파일(`*.bag`, `*.pcd`, `*.zip` 등)은 `.gitignore`에 등록되어 있습니다.
- 새로운 모델 파라미터나 포인트클라우드 파일 공유 시 LFS 또는 외부 저장소를 활용해 주세요.
