#!/bin/bash
# =============================================================
# ros1_ws 초기 설치 스크립트
# Ubuntu 20.04 + ROS Noetic 환경에서 실행하세요.
# =============================================================

set -e
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "========================================"
echo " ROS1 Autonomous Driving Workspace Setup"
echo "========================================"

# 1. ROS 의존 패키지
echo "[1/4] ROS apt 패키지 설치..."
sudo apt-get update -q
sudo apt-get install -y \
    ros-noetic-robot-localization \
    ros-noetic-velodyne-pointcloud \
    ros-noetic-ndt-cpu \
    ros-noetic-tf2-ros \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    python3-catkin-tools \
    python3-tk

# 2. Python 의존 패키지
echo "[2/4] Python pip 패키지 설치..."
pip3 install -r "$WORKSPACE_DIR/requirements.txt"

# 3. GitHub 서브 패키지 클론 (03_localization)
echo "[3/4] GitHub 패키지 클론..."
LOC_DIR="$WORKSPACE_DIR/src/03_localization"

if [ ! -d "$LOC_DIR/fast_gicp/.git" ]; then
    git clone https://github.com/SMRT-AIST/fast_gicp.git "$LOC_DIR/fast_gicp"
fi
if [ ! -d "$LOC_DIR/ndt_omp/.git" ]; then
    git clone https://github.com/koide3/ndt_omp.git "$LOC_DIR/ndt_omp"
fi
if [ ! -d "$LOC_DIR/hdl_localization/.git" ]; then
    git clone https://github.com/koide3/hdl_localization.git "$LOC_DIR/hdl_localization"
fi

# 4. catkin_make 빌드
echo "[4/4] catkin_make 빌드..."
cd "$WORKSPACE_DIR"
source /opt/ros/noetic/setup.bash
catkin_make

echo ""
echo "========================================"
echo " 설치 완료!"
echo " 다음 명령어로 환경을 활성화하세요:"
echo "   source $WORKSPACE_DIR/devel/setup.bash"
echo ""
echo " [주의] MORAI 시뮬레이터는 별도 설치 필요:"
echo "   ~/MoraiLauncher_Lin/ 에 설치 후 실행"
echo "========================================"
