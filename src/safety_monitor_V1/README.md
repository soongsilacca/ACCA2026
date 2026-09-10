# safety_monitor

Collision Imminent E-STOP — report section 8.5.2.

An independent, non-learning safety layer. It judges collision imminence from
raw LiDAR and ego velocity alone and never subscribes to model, planner, or
controller output, so it keeps working when those fail.

## Install

    cp -r safety_monitor ~/catkin_ws/src/
    cd ~/catkin_ws && catkin_make && source devel/setup.bash
    chmod +x src/safety_monitor/scripts/collision_estop_node.py

## Run

    roslaunch safety_monitor collision_estop.launch

## Interface

Subscribes

| topic | type | note |
|---|---|---|
| `/velodyne_points_filtered` | `sensor_msgs/PointCloud2` | velodyne frame, 15 Hz |
| `/morai/ego_vehicle_status` | `morai_msgs/EgoVehicleStatus` | velocity in km/h |
| `/safety/ego_speed_mps` | `std_msgs/Float32` | fallback when morai_msgs is missing |

Publishes

| topic | type | note |
|---|---|---|
| `/safety/estop_trigger` | `std_msgs/Bool` | true while E-STOP is latched |
| `/safety/estop_reason` | `std_msgs/String` | latch cause, for logging |
| `/safety/ttc` | `std_msgs/Float32` | seconds, 999.9 when safe |
| `/safety/d_obs` | `std_msgs/Float32` | metres from the front bumper |
| `/safety/corridor` | `visualization_msgs/MarkerArray` | RViz overlay |

## First run

The node self-checks its frame assumption over the first 15 clouds and logs
one of:

    [diag] frame assumption consistent with the data
    [diag] ... FIX THE PARAMETER          -> flip cloud_in_base_link in config
    [diag] ... ground removal already applied upstream

Also confirm the yaw-rate source:

    rosmsg show morai_msgs/EgoVehicleStatus

If there is no `heading_rate` field the corridor stays straight even in turns,
and the node needs an IMU subscription instead.

## Tuning

| symptom | parameter |
|---|---|
| fires during normal driving | raise `a_max` |
| fires on curves | lower `lateral_margin`, check yaw rate |
| latches and releases repeatedly | raise `release_hold_sec` |
| fires on momentary noise | raise `confirm_frames` |
| fires too late | raise `ttc_estop` |

`a_max` is not measured. It is set below the dry-asphalt literature range so
the predicted stopping distance is longer than reality — the conservative
direction. Values under about 5 push `d_stop` past normal following distance
and trip during ordinary car-following.

## safety_gate

`collision_estop_node` only reports danger. `safety_gate_node` is the part that
actually brakes, and it sits last on the path to the simulator so it still works
when everything upstream is dead.

    roslaunch safety_monitor safety_monitor.launch                    # judgement + gate
    roslaunch safety_monitor safety_monitor.launch gate_mode:=gate    # once MPC is wired

### Modes

| mode | while safe | on E-STOP |
|---|---|---|
| `override_only` (default) | publishes nothing, keyboard driving untouched | takes over, brakes |
| `gate` | relays `/ctrl_cmd_raw` | substitutes a brake command |

Use `override_only` for manual testing: the car is yours until the safety layer
decides otherwise. Switch to `gate` once the controller publishes to
`/ctrl_cmd_raw`, which also gets you a brake on controller timeout.

### Latch

Braking holds until the trigger clears **and** the vehicle has been under
`release_speed` for `release_hold_sec`. Releasing on the trigger alone would let
a still-moving car roll away. A momentary trigger therefore still produces a
full stop.

If `/safety/estop_trigger` goes silent for `trigger_timeout` after having been
seen at least once, the gate brakes — a dead judgement node is a fault, not a
green light.

### Before running

Confirm the topic the MORAI bridge actually subscribes to and set
`ctrl_cmd_topic` to match:

    rostopic info /ctrl_cmd

Also check that `ctrl_mode` and `gear` match this MORAI build's conventions.
Publishing on the control topic takes the vehicle out of keyboard mode, so the
first test should be at low speed in open space.
