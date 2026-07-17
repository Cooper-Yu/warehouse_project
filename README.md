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
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_sim.yaml
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml

# Localization
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
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
