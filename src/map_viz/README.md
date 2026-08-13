# map_viz

MGeo (JSON format) Map Data Visualization ROS Package. This package parses MGeo nodes and links to publish them as `visualization_msgs/MarkerArray` topic, making it easy to inspect roads, junctions, and direction arrows in RViz.

---

## Features

- **MGeo Map Parsing**: Reads `node_set.json`, `link_set.json`, and `global_info.json` mapping files.
- **Visualizer Node**: 
  - Publishes nodes as point markers (red circles for general nodes).
  - Publishes links as line strips with distinct color schemes matching traffic features (straight: Electric Blue, left/uturn: Neon Magenta, unprotected: Vivid Amber, normal: Slate Grey).
  - Visualizes direction arrows on each link indicating target heading directions.
- **Dynamic Configuration**: Fully configurable TF frame, publisher frequency, and visibility controls.

---

## Directory Structure

```text
map_viz/
├── launch/
│   └── mgeo_marker.launch       # Launch file for visualizer node
├── scripts/
│   ├── node_set.json            # MGeo nodes coordinates metadata
│   ├── link_set.json            # MGeo lanes connections metadata
│   ├── global_info.json         # Coordinate offset parameters
│   └── mgeo_marker_node.py      # ROS marker publishing node
├── CMakeLists.txt
└── package.xml
```

---

## How to Run

Run the launch file to fire up the node:

```bash
roslaunch map_viz mgeo_marker.launch
```

### Parameters

The following parameters can be customized via launch arguments or ROS Parameter Server:

| Param Name | Default Value | Description |
|---|---|---|
| `node_file` | `$(find map_viz)/scripts/node_set.json` | Path to MGeo node json file |
| `link_file` | `$(find map_viz)/scripts/link_set.json` | Path to MGeo link json file |
| `global_info_file` | `$(find map_viz)/scripts/global_info.json` | Path to global coordinate offset json file |
| `frame_id` | `map` | ROS TF frame for visualization |
| `publish_hz` | `0.5` | Publishing frequency of MarkerArray topic (Hz) |
| `show_arrows` | `true` | Show direction arrows on lanes |
| `publish_static_tf` | `true` | Broadcasts static TF `map` -> `odom` -> `base_link` for convenient RViz setups |

---

## Published Topics

- `/mgeo_map_markers` (`visualization_msgs/MarkerArray`)
  - Topic containing node points, lane lines, and heading arrows.
