# ACCA (2026)
Autonomous Car Core Architecture  
Development in progress

---

## Localization

### Input Topics
| Topic | Type | Description |
|---|---|---|
| `/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | GPS absolute position |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation + Angular rate + Linear acceleration |
| `/erp42_feedback` | `erp42_msgs/msg/SerialFeedBack` | Vehicle speed, steering, gear, encoder |

### Output Topics
| Topic | Type | Frame | Description |
|---|---|---|---|
| `/odometry/local` | `nav_msgs/msg/Odometry` | `odom` | Local odometry (IMU + Vehicle Feedback) |
| `/odometry/global` | `nav_msgs/msg/Odometry` | `map` | Global fused odometry (GPS + Local DR) |
| `/odometry/gps` | `nav_msgs/msg/Odometry` | `map` | GPS ground-truth |

---

## Architecture
- Local odometry: IMU + wheel feedback 기반 Dead-reckoning
- Global odometry: GPS 기반 map 보정
- Frame 구조: `map → odom → base_link`

---

## 실행 방법

Localization 실행:

```bash
ros2 launch localization dual_ekf_localization.launch.py



## 성능 분석

RMSE 기반 성능 분석 :

```bash
ros2 run localization rmse_analyzer