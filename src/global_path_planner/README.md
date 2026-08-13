# global_path_planner

A* and Dijkstra-based Global Path Planner ROS Package. It reads MGeo map files to find the optimal sequence of links from a starting pose to a goal pose, supporting smooth lane change blending, upstream state constraints, and multi-candidate junction alignment.

---

## Features

- **Upstream State-Constraint A* Search**:
  - Operates on a link-to-link transition graph with dynamic costs.
  - Integrates segment-level state tracking `(segment_idx, interpolation_t)` to prevent illegal shortcuts or immediate u-turns on parallel lanes.
  - Enforces loop-arounds (360-degree detours) when the goal is upstream of the start position on the same lane.
- **Multi-Candidate Junction Matcher**:
  - Selects multiple candidate links within `1.0m` from start and goal clicks.
  - Performs cross-validation on heading alignments (`start_yaw` / `goal_yaw`) to prevent detour bugs caused by misassociated lane selections in dense overlapping junctions.
- **Smooth Lane Change Blending**:
  - Supports continuous lane change trajectory generation.
  - Interpolates path coordinates between source and target links dynamically without hard geometric kinks or spline-interpolation detours.
- **RViz Real-time Integration**:
  - Listens to RViz `2D Pose Estimate` (`/initialpose` topic) and `2D Nav Goal` (`/move_base_simple/goal` topic).
  - Publishes target path markers and paths for motion controller consumption.

---

## Directory Structure

```text
global_path_planner/
├── launch/
│   └── global_path_planner.launch  # Launch file for planner node
├── scripts/
│   └── global_path_planner_node.py # Main ROS path planner script
├── CMakeLists.txt
└── package.xml
```

---

## How to Run

1. Launch the map visualizer first:
   ```bash
   roslaunch map_viz mgeo_marker.launch
   ```
2. Launch the global path planner node:
   ```bash
   roslaunch global_path_planner global_path_planner.launch
   ```
3. Open RViz, add Path and Marker topics, and:
   - Click `2D Pose Estimate` on a lane to set start pose.
   - Click `2D Nav Goal` on a lane to set goal pose.
   - The planned path will be instantly visualized in green (raw clicks in red/green).

---

## Parameters

| Param Name | Default Value | Description |
|---|---|---|
| `node_file` | `$(find map_viz)/scripts/node_set.json` | Path to MGeo node json file |
| `link_file` | `$(find map_viz)/scripts/link_set.json` | Path to MGeo link json file |
| `global_info_file` | `$(find map_viz)/scripts/global_info.json` | Path to coordinate offset json file |
| `frame_id` | `map` | ROS TF frame for visualization |
| `lane_change_penalty` | `15.0` | Cost multiplier for lane change transitions |
| `left_turn_penalty` | `10.0` | Cost multiplier for left turn classification |
| `right_turn_penalty` | `5.0` | Cost multiplier for right turn classification |
| `uturn_penalty` | `9999.0` | Massive penalty to discourage u-turns at intersections |
| `use_smoothing` | `true` | Apply interpolation / smoothing to final output path |
| `resample_interval` | `0.5` | Target distance interval between path points (meters) |

---

## Topics

### Subscribed Topics
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)

### Published Topics
- `/global_path` (`nav_msgs/Path`)
  - Target route coordinates for the vehicle controller.
- `/planner_markers` (`visualization_msgs/MarkerArray`)
  - Clicks and matched waypoint projections visualizer.
