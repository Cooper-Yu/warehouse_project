from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def profile_nodes(condition, use_sim_time, files):
    """Create one conditionally launched Nav2 stack for a sim or real profile."""
    common_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            condition=condition,
            parameters=[{
                "yaml_filename": files["map"],
                **common_time,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            condition=condition,
            parameters=[files["amcl"], common_time],
        ),
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
            parameters=[files["recoveries"], common_time],
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
            parameters=[{
                "autostart": True,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = FindPackageShare("path_planner_server")
    map_share = FindPackageShare("map_server")
    localization_share = FindPackageShare("localization_server")

    bt_xml = PathJoinSubstitution([
        package_share,
        "config",
        "navigate_w_replanning_and_recovery.xml",
    ])

    # One argument switches every environment-specific file as a unit.
    sim_files = {
        "map": PathJoinSubstitution([
            map_share, "config", "warehouse_map_sim.yaml"
        ]),
        "amcl": PathJoinSubstitution([
            localization_share, "config", "amcl_config_sim.yaml"
        ]),
        "planner": PathJoinSubstitution([
            package_share, "config", "planner_sim.yaml"
        ]),
        "controller": PathJoinSubstitution([
            package_share, "config", "controller_sim.yaml"
        ]),
        "recoveries": PathJoinSubstitution([
            package_share, "config", "recoveries_sim.yaml"
        ]),
        "bt_navigator": PathJoinSubstitution([
            package_share, "config", "bt_navigator_sim.yaml"
        ]),
        "bt_xml": bt_xml,
    }

    real_files = {
        "map": PathJoinSubstitution([
            map_share, "config", "warehouse_map_real.yaml"
        ]),
        "amcl": PathJoinSubstitution([
            localization_share, "config", "amcl_config_real.yaml"
        ]),
        "planner": PathJoinSubstitution([
            package_share, "config", "planner_real.yaml"
        ]),
        "controller": PathJoinSubstitution([
            package_share, "config", "controller_real.yaml"
        ]),
        "recoveries": PathJoinSubstitution([
            package_share, "config", "recoveries_real.yaml"
        ]),
        "bt_navigator": PathJoinSubstitution([
            package_share, "config", "bt_navigator_real.yaml"
        ]),
        "bt_xml": bt_xml,
    }

    actions = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock in simulation; use system time on the real robot.",
        ),
    ]

    actions.extend(profile_nodes(IfCondition(use_sim_time), use_sim_time, sim_files))
    actions.extend(
        profile_nodes(UnlessCondition(use_sim_time), use_sim_time, real_files)
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_navigation",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    package_share,
                    "rviz",
                    "navigation.rviz",
                ]),
            ],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
            }],
        )
    )

    return LaunchDescription(actions)
