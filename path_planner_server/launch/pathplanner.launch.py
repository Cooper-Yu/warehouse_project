from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _navigation_nodes(condition, use_sim_time, files):
    common_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    return [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            condition=condition,
            parameters=[files["planner"], common_time],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            condition=condition,
            parameters=[files["controller"], common_time],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            condition=condition,
            parameters=[files["behaviors"], common_time],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            condition=condition,
            parameters=[
                files["bt_navigator"],
                {
                    "default_nav_to_pose_bt_xml": files["bt_xml"],
                    **common_time,
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            condition=condition,
            parameters=[
                {
                    "autostart": True,
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "behavior_server",
                        "bt_navigator",
                    ],
                    **common_time,
                }
            ],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keepout = LaunchConfiguration("use_keepout")
    use_rviz = LaunchConfiguration("use_rviz")
    keepout_mask_file = LaunchConfiguration("keepout_mask_file")
    package_share = FindPackageShare("path_planner_server")
    map_package_share = FindPackageShare("map_server")

    keepout_mask_yaml = PathJoinSubstitution(
        [map_package_share, "config", keepout_mask_file]
    )

    common = {
        "bt_xml": PathJoinSubstitution(
            [package_share, "config", "navigate_w_replanning_and_recovery.xml"]
        )
    }
    sim_files = {
        **common,
        "planner": PathJoinSubstitution([package_share, "config", "planner_sim.yaml"]),
        "controller": PathJoinSubstitution([package_share, "config", "controller_sim.yaml"]),
        "behaviors": PathJoinSubstitution([package_share, "config", "recoveries_sim.yaml"]),
        "bt_navigator": PathJoinSubstitution([package_share, "config", "bt_navigator_sim.yaml"]),
    }
    real_files = {
        **common,
        "planner": PathJoinSubstitution([package_share, "config", "planner_real.yaml"]),
        "controller": PathJoinSubstitution([package_share, "config", "controller_real.yaml"]),
        "behaviors": PathJoinSubstitution([package_share, "config", "recoveries_real.yaml"]),
        "bt_navigator": PathJoinSubstitution([package_share, "config", "bt_navigator_real.yaml"]),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time for the navigation nodes.",
            ),
            DeclareLaunchArgument(
                "use_keepout",
                default_value=use_sim_time,
                description=(
                    "Start the keepout mask/filter chain. Defaults to enabled "
                    "for simulation and disabled for the still-gated real profile."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description=(
                    "Start the single Checkpoint 12 Navigation RViz. "
                    "Localization RViz remains disabled by default."
                ),
            ),
            DeclareLaunchArgument(
                "keepout_mask_file",
                default_value="warehouse_map_keepout_sim_mask.yaml",
                description="Keepout mask YAML file in map_server/config.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="filter_mask_server",
                output="screen",
                condition=IfCondition(use_keepout),
                parameters=[
                    {
                        "yaml_filename": keepout_mask_yaml,
                        "topic_name": "keepout_filter_mask",
                        "frame_id": "map",
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_server",
                output="screen",
                condition=IfCondition(use_keepout),
                parameters=[
                    {
                        "type": 0,
                        "filter_info_topic": "/costmap_filter_info",
                        "mask_topic": "/keepout_filter_mask",
                        "base": 0.0,
                        "multiplier": 1.0,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_keepout",
                output="screen",
                condition=IfCondition(use_keepout),
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            "filter_mask_server",
                            "costmap_filter_info_server",
                        ],
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            *_navigation_nodes(IfCondition(use_sim_time), use_sim_time, sim_files),
            *_navigation_nodes(UnlessCondition(use_sim_time), use_sim_time, real_files),
            Node(
                package="shelf_detection_server",
                executable="shelf_detection_server",
                name="shelf_detection_server",
                output="screen",
                condition=IfCondition(use_sim_time),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_navigation",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=[
                    "-d",
                    PathJoinSubstitution([package_share, "rviz", "navigation.rviz"]),
                ],
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
        ]
    )
