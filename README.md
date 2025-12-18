# ACCA_2025
Autonomous Vihicle Project with ERP42 

## repo 소개
ACCA팀 주요 코드


## 빌드
    git clone https://github.com/beomseok3/ACCA_2025

    cd ACCA_2025

*msg 먼저 빌드해주기!!* (권장)

    colcon build --symlink-install --packages-select adaptive_clustering_msgs

*msg 빌드 적용을 위한 소스*

    . install/setup.bash

*전체 패키지 빌드*

    colcon build --symlink-install

## prerequisite
    mavros_msgs
    nmea_msgs
    ...
    

