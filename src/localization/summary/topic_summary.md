# ROS Topic Summary

This document summarizes the Input and Output topics for the nodes in the active workspace.

## Package: `localization`

### Node: `gps_to_odometry_node`
**File:** `localization/gps_to_odometry.py`
**Frequency:** Input Driven (depends on `/ublox_gps_node/fix` rate)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | Raw GPS fix data |
| **Output** | `/navsat/odometry` | `nav_msgs/msg/Odometry` | Odometry converted from GPS (in fixed Map datum) |

### Node: `erp42_odometry_node`
**File:** `localization/wheel_odometry_erp.py`
**Frequency:** Input Driven (depends on `/erp42_feedback` rate)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/erp42_feedback` | `erp42_msgs/msg/SerialFeedBack` | Raw wheel speed and steering data |
| **Output** | `/wheel/odom` | `nav_msgs/msg/Odometry` | Computed wheel odometry |
| **Output** | `/tf` | `tf2_msgs/msg/TFMessage` | Transform from `odom` to `base_link` (optional) |

### Node: `ekf_global_initializer`
**File:** `localization/ekf_global_initializer.py`
**Frequency:** One-shot (on first GPS fix)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | Raw GPS fix data (first fix used) |
| **Output** | `/set_pose/global` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose for Global EKF |

### Node: `ekf_local_initializer`
**File:** `localization/ekf_local_initializer.py`
**Frequency:** One-shot (on first IMU message)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/navheading` | `sensor_msgs/msg/Imu` | IMU data for initial orientation |
| **Output** | `/set_pose/local` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose for Local EKF |

### Node: `imu_nwu_adapter`
**File:** `localization/imu_nwu_adapter.py`
**Frequency:** Input Driven (depends on `/imu/data` rate)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/imu/data` | `sensor_msgs/msg/Imu` | Raw IMU data (likely ENU) |
| **Output** | `/imu/data_nwu` | `sensor_msgs/msg/Imu` | Rotated IMU data (NWU aligned) |

### Node: `odom_map_republisher`
**File:** `localization/odom_map_republisher.py`
**Frequency:** Input Driven (depends on `/odometry/local` rate)

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/odometry/local` | `nav_msgs/msg/Odometry` | Output from Local EKF |
| **Output** | `/odometry/local_map_aligned` | `nav_msgs/msg/Odometry` | Same odom but with `map` frame_id |

---

## Package: `robot_localization` (used by `localization` launch)

### Node: `ekf_local_node`
**Config:** `localization/config/ekf_local.yaml`
**Frequency:** 50 Hz

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/wheel/odom` | `nav_msgs/msg/Odometry` | Wheel odometry |
| **Input** | `/imu/data_nwu` | `sensor_msgs/msg/Imu` | IMU data (NWU) |
| **Output** | `/odometry/local` | `nav_msgs/msg/Odometry` | Filtered local odometry |
| **Output** | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` -> `base_link` transform |

### Node: `ekf_global_node`
**Config:** `localization/config/ekf_global.yaml`
**Frequency:** 50 Hz

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/odometry/local_map_aligned` | `nav_msgs/msg/Odometry` | Local odom aligned to map frame |
| **Input** | `/navsat/odometry` | `nav_msgs/msg/Odometry` | GPS converted to odom |
| **Input** | `/imu/data_nwu` | `sensor_msgs/msg/Imu` | IMU data (NWU) |
| **Output** | `/odometry/global` | `nav_msgs/msg/Odometry` | Filtered global odometry |
| **Output** | `/tf` | `tf2_msgs/msg/TFMessage` | `map` -> `odom` transform |

---

## Package: `scanmatcher` (part of `lidarslam` stack)

### Node: `scan_matcher`
**File:** `scanmatcher/src/scanmatcher_component.cpp`
**Frequency:** 10 Hz (scan_period: 0.1s)
**Map Publish Frequency:** Every 15.0s

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `initial_pose` | `geometry_msgs/msg/PoseStamped` | Initial pose for SLAM |
| **Input** | `imu` | `sensor_msgs/msg/Imu` | IMU data for deskewing/fusion |
| **Input** | `input_cloud` | `sensor_msgs/msg/PointCloud2` | LiDAR PointCloud (e.g. `/velodyne_points`) |
| **Output** | `current_pose` | `geometry_msgs/msg/PoseStamped` | Current estimated pose |
| **Output** | `map` | `sensor_msgs/msg/PointCloud2` | Current local map cloud |
| **Output** | `map_array` | `lidarslam_msgs/msg/MapArray` | Array of submaps |
| **Output** | `path` | `nav_msgs/msg/Path` | Trajectory path |
| **Output** | `/tf` | `tf2_msgs/msg/TFMessage` | Transform (`map` -> `base_link` or `map` -> `odom`) |

---

## Package: `graph_based_slam` (part of `lidarslam` stack)

### Node: `graph_based_slam`
**File:** `graph_based_slam/src/graph_based_slam_component.cpp`
**Frequency:** 1 Hz (Loop detection period: 1000ms)

**Saved Files:**
- `pose_graph.g2o`: Saved graph optimization data.
- `map.pcd`: Saved point cloud map.
*Files are saved when `/map_save` service is called or during loop closure if `use_save_map_in_loop` is true.*

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `map_array` | `lidarslam_msgs/msg/MapArray` | Input submaps from specific scanmatcher |
| **Output** | `modified_map` | `sensor_msgs/msg/PointCloud2` | Optimized global map |
| **Output** | `modified_map_array` | `lidarslam_msgs/msg/MapArray` | Optimized submaps |
| **Output** | `modified_path` | `nav_msgs/msg/Path` | Optimized trajectory |
