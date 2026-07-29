# localization

GPS + IMU + Wheel Odometry를 **EKF(Extended Kalman Filter)** 로 융합하는 자차 위치 추정 패키지입니다.

## 노드 목록

| 노드 | 설명 |
|------|------|
| `gps_node.py` | GPS 원시 데이터 파싱 및 `/gps` 발행 |
| `wheel_odom_node.py` | Ego 속도 기반 Wheel Odometry 계산 및 `/wheel_odom` 발행 |
| `ekf_global_initializer.py` | 글로벌 EKF 초기화 (GPS 기준) |
| `ekf_local_initializer.py` | 로컬 EKF 초기화 (NDT 기준) |

## Launch 파일

```bash
# 글로벌 Localization (GPS + Wheel Odom EKF)
roslaunch localization localization_global.launch

# 로컬 Localization (NDT + Wheel Odom EKF)
roslaunch localization localization_local.launch

# NDT용 Sensor TF 설정
roslaunch localization sensor_tf.launch
```

## 토픽

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/gps` | 입력 | 원시 GPS (위도/경도/고도) |
| `/imu` | 입력 | IMU (가속도/자이로/쿼터니언) |
| `/morai/ego_vehicle_status` | 입력 | 차량 속도 (Wheel Odom 계산용) |
| `/wheel_odom` | 출력 | Wheel Odometry (`nav_msgs/Odometry`) |
| `/localization/kinematic_state` | 출력 | EKF 융합 위치 (`nav_msgs/Odometry`) |

## 파라미터

`localization_global.launch` 에서 `robot_localization` EKF 노드를 구동합니다.  
EKF 설정 파라미터는 `config/ekf_global.yaml` 에서 관리합니다.