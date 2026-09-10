# MORAI rear LiDAR perception (ROS1 Noetic)

This package runs independently from `morai_privileged_expert/auto_episode.launch`.
It receives a MORAI VLP-16 stream, removes the ego vehicle, road and large wall
planes, clusters obstacle points, tracks the clusters, and publishes a
constant-velocity dead-reckoning prediction.

## MORAI network setup

Use the values supplied for the rear sensor:

| MORAI field | Value |
|---|---|
| UDP IP | `127.0.0.1` |
| Host Sensor Port | `2369` |
| Destination IP | `127.0.0.1` |
| Destination Port | `2368` |
| Pose | `x=-0.73, y=0, z=0.22, roll=0, pitch=0, yaw=0` |

`velodyne_driver` listens on the **Destination Port**, so the launch argument
is `destination_port:=2368`. Host Sensor Port 2369 is MORAI's source side and
is not a ROS driver argument.

Only one process can reliably consume a given LiDAR UDP destination port. Do
not run the existing front VLP-16 receiver on port 2368 at the same time. To
run front and rear LiDAR simultaneously, change the rear sensor's MORAI
Destination Port (for example to 2370) and launch with
`destination_port:=2370`.

## Build and run on Ubuntu 20.04

Install the standard Noetic Velodyne/PCL dependencies if they are not already
present:

```bash
sudo apt update
sudo apt install ros-noetic-velodyne ros-noetic-pcl-ros
```

Place this package under the Ubuntu catkin workspace's `src`, then build:

```bash
cd ~/acca_ws
catkin_make
source devel/setup.bash
roslaunch rear_lidar_perception rear_lidar_only.launch
```

The standalone launch also starts the existing MORAI EgoVehicleStatus receiver
on UDP 9009. This is used only to compensate ego motion, which prevents parked
walls and barriers from looking dynamic while the ego vehicle moves. Configure
MORAI EgoVehicleStatus to send to `127.0.0.1:9009`. When the normal MORAI bridge
already owns that port, the perception node can instead consume
`/morai/ego_vehicle_status` directly. Motion-source priority is dedicated odom,
`/localization/kinematic_state`, then MORAI ego status.

If another localization node already publishes odometry, disable that receiver
and use its topic:

```bash
roslaunch rear_lidar_perception rear_lidar_only.launch \
  launch_ego_odometry:=false odom_topic:=/localization/kinematic_state
```

To inspect a bag or an already-published point cloud without opening UDP:

```bash
roslaunch rear_lidar_perception rear_lidar_only.launch \
  launch_driver:=false launch_ego_odometry:=false \
  input_topic:=/your/rear_points
```

To bypass road/wall removal while diagnosing an empty filtered cloud:

```bash
roslaunch rear_lidar_perception rear_lidar_only.launch \
  enable_ground_filter:=false enable_wall_filter:=false
```

## Outputs

| Topic | Type | Description |
|---|---|---|
| `/rear_lidar/points_raw` | `sensor_msgs/PointCloud2` | Decoded VLP-16 points |
| `/rear_lidar/obstacles` | `sensor_msgs/PointCloud2` | Road/wall/ego removed cloud in `base_link` |
| `/rear_lidar/debug/cropped` | `sensor_msgs/PointCloud2` | Pose-transformed ROI before road/wall removal |
| `/rear_lidar/tracked_objects` | `rear_lidar_perception/TrackedObjectArray` | IDs, boxes, velocities and predictions |
| `/rear_lidar/markers` | `visualization_msgs/MarkerArray` | Green static / red dynamic boxes and prediction lines |
| `/rear_lidar/debug/ground` | `sensor_msgs/PointCloud2` | Removed road points |
| `/rear_lidar/debug/walls` | `sensor_msgs/PointCloud2` | Removed large vertical planes |
| `/rear_lidar/status` | `std_msgs/String` | Per-frame counts or a no-input diagnostic |

For objects classified as dynamic, `predicted_positions` contains one point
every 0.5 s for 4 s by default. Static objects have no prediction path. With
valid odometry, both object and ego vehicle use constant-velocity dead
reckoning and predictions are expressed in the ego vehicle's predicted future
`base_link`. Without any fresh ego-motion source, `odometry_compensated` is
false, dynamic classification and absolute prediction are disabled, and only
`relative_velocity` is populated. This avoids classifying the entire static
scene as dynamic while the ego vehicle moves.

## Tuning checks

Start with the supplied RViz view and enable `Debug Ground` and `Debug Walls`.
The most useful parameters in `config/rear_lidar.yaml` are:

- `expected_ground_z` and `ground_distance_threshold` if road points remain or
  low vehicle points disappear.
- `wall_min_length` and `wall_min_occupancy_ratio` if a barrier remains; the
  defaults require a continuous 10 m plane so vehicle side faces are retained.
- `cluster_tolerance`, `cluster_range_scale`, and `min_cluster_points` for
  long-range sparse detections.
- `dynamic_speed_threshold` and `association_distance` for high-speed traffic.
- ROI `roi_min_x/max_x/min_y/max_y` to cover the required rear lanes.
- `vehicle_min/max_x/y` for the padded ego footprint. The supplied values use
  the rear-axle `base_link`: x=-1.05..4.10 m and y=-1.15..1.15 m.

Quick health checks:

```bash
rostopic hz /rear_lidar/points_raw
rostopic hz /rear_lidar/obstacles
rostopic echo -n 1 /rear_lidar/tracked_objects
rosnode info /rear_lidar_perception
```

Interpret `/rear_lidar/status` as follows:

- `PROCESSING raw=N`: the perception subscriber receives the cloud, but the
  current frame is still in PCL filtering/clustering.
- `OK raw=... cropped=... obstacles=... processing_ms=...`: the complete
  pipeline is running; use the counts to locate an over-aggressive filter.
- `NO_POINTCLOUD`: the perception subscriber has not received the raw topic.
- No status publisher at all: the perception executable did not start; inspect
  `rosnode info /rear_lidar_perception` and the roslaunch terminal.
