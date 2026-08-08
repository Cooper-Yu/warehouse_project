# Warehouse Project - Checkpoint 11

ROS 2 Humble implementation for Checkpoint 11 warehouse mapping,
localization, and navigation with the RB1 robot.

## Status

- Task 1 mapping and saved-map serving: validated in simulation and on the
  real robot.
- Task 2 localization: validated in simulation and on the real robot, including
  particle-cloud visualization with the environment-specific AMCL profile.
- Task 3 navigation: simulation runtime validated in The Construct. A Nav2
  goal completed with `Feedback: reached`, `Distance remaining: 0.00 m`, and
  zero recoveries.
- Task 3 real navigation: validated in the real lab with successful goals,
  global/local paths, nonzero velocity commands, and physical AMCL movement.

## Packages

| Package | Responsibility |
| --- | --- |
| `cartographer_slam` | Build simulation and real warehouse maps |
| `map_server` | Load the selected saved map and display it in RViz |
| `localization_server` | Run map server and AMCL localization |
| `path_planner_server` | Run the complete Nav2 navigation stack |

## Checkpoint Launch Commands

```bash
# Mapping
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=true
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=false

# Saved map
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_keepout_sim.yaml
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml

# Localization
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_sim.yaml
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml

# Navigation
ros2 launch path_planner_server navigation.launch.py use_sim_time:=true
ros2 launch path_planner_server navigation.launch.py use_sim_time:=false
```

Use `2D Pose Estimate` after each fresh localization or navigation launch.

## Simulation Navigation

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select path_planner_server
source install/setup.bash
ros2 launch path_planner_server navigation.launch.py use_sim_time:=true
```

Set `2D Pose Estimate`, wait until the Navigation 2 panel reports active,
then send a `Nav2 Goal`.

## Debug Bundle

While navigation is running:

```bash
cd ~/ros2_ws
ros2 run path_planner_server collect_navigation_logs 45
```

The command creates `navigation_debug_YYYYMMDD_HHMMSS.tar.gz` in the current
directory with lifecycle, action, TF, path, velocity, AMCL, and ROS log data.

## Confirmed Real Robot Interface

| Role | Real robot value |
| --- | --- |
| Map frame | `map` |
| Odom frame | `robot_odom` |
| Navigation base frame | `robot_base_footprint` |
| Laser frame | `robot_front_laser_link` |
| Laser topic | `/scan` (`sensor_msgs/msg/LaserScan`, about 40 Hz) |
| Odometry topic | `/odom` (`nav_msgs/msg/Odometry`, about 49 Hz) |
| Velocity topic | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| Time source | system time (`use_sim_time: false`) |
| Controller frequency | `5.0` Hz, required by the checkpoint |

Verified real TF core:

```text
robot_odom -> robot_base_footprint -> robot_base_link
robot_base_footprint -> robot_front_laser_link
```

RB1 model evidence confirms a body envelope of approximately `0.500 m x
0.498 m`, matching the configured `+/-0.25 m` square footprint. The installed
controller is `diff_drive_controller/DiffDriveController`, so lateral velocity
remains disabled. The real profile uses conservative Nav2 limits below the
base-controller maximums and passed controlled real-lab validation. Its
controller frequency is fixed at the checkpoint-required 5 Hz.

## Real Robot Full Mission

Before starting, stop any previous mission, confirm the robot and shelf are
safe, and place the robot at the calibrated loading position. Synchronize and
build the workspace first:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/warehouse_project
git pull --ff-only origin master
cd ~/ros2_ws
colcon build --packages-select \
  localization_server path_planner_server shelf_detection_server nav2_apps \
  2>&1 | tee ~/build_real_mission.log
source ~/ros2_ws/install/setup.bash
```

Run the following in three terminals.

Terminal 1 — localization, AMCL, and the loaded-scan filter:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py \
  map_file:=warehouse_map_real.yaml \
  2>&1 | tee ~/real_localization.log
```

AMCL uses `/scan_localization`; the filter passes the raw scan while unloaded
and removes near self-returns after loading. The ordinary real map is used for
localization. Keepout is not part of the AMCL map.

Terminal 2 — Nav2, RViz, keepout mask, and shelf service:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py \
  use_sim_time:=False \
  2>&1 | tee ~/real_pathplanner.log
```

The launch automatically starts `filter_mask_server`,
`costmap_filter_info_server`, and `shelf_detection_server`. The keepout mask
is selected separately as `warehouse_map_keepout_real_mask.yaml` and modifies
Nav2 traversability without changing AMCL's map model.

Terminal 3 — complete shelf mission:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
timeout 500 python3 \
  ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship_real.py \
  2>&1 | tee ~/real_full_mission.log
```

The default integrated mission is:

```text
loading position -> unloaded footprint -> elevator down
-> multi-frame shelf centering -> segmented entry
-> final push (0.3703 m) -> elevator up -> loaded footprint
-> shipping navigation -> elevator down -> bounded shelf exit
-> CLEAR_OF_SHELF -> unloaded footprint -> return position
```

The attach client waits up to 180 seconds, while the shelf server retains its
own bounded motion watchdog. A failed stage publishes zero velocity and does
not continue to the next stage.

## Real Mission Checks and Logs

Before running the mission:

```bash
ros2 param get /amcl scan_topic
timeout 5 ros2 topic hz /scan_localization
ros2 node list | grep -E \
  'filter_mask_server|costmap_filter_info_server|planner_server|controller_server|bt_navigator|shelf_detection_server'
ros2 service list | grep approach_shelf
```

Useful post-run summary:

```bash
grep -Ei \
  'loading_position|safe-standoff|stepwise attach|center approach speed|blind entry|final push|elevator-up|shipping_position|CLEAR_OF_SHELF|unloaded_footprint|return|succeeded|failed|timeout|exception' \
  ~/real_full_mission.log \
  | tee ~/real_full_mission_summary.log
```

If motion must be stopped immediately:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
