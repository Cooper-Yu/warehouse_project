# Warehouse Project - Checkpoint 11

ROS 2 Humble implementation for Checkpoint 11 warehouse mapping,
localization, and navigation with the RB1 robot.

## Status

- Task 1 mapping and saved-map serving: validated in simulation and on the
  real robot.
- Task 2 localization: validated in simulation; real localization validation
  remains part of the real lab.
- Task 3 navigation: simulation runtime validated in The Construct. A Nav2
  goal completed with `Feedback: reached`, `Distance remaining: 0.00 m`, and
  zero recoveries.
- Task 3 real navigation: interface evidence collected; real Nav2 YAML and
  low-speed lab validation remain open.

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

Before real motion testing, confirm the RB1 footprint, lateral-motion support,
and safe velocity/acceleration limits from the course or robot configuration.
The current `+/-0.25 m` square footprint remains a conservative candidate, not
a final physical specification.
