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
    package_share = FindPackageShare("path_planner_server")

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
            *_navigation_nodes(IfCondition(use_sim_time), use_sim_time, sim_files),
            *_navigation_nodes(UnlessCondition(use_sim_time), use_sim_time, real_files),
        ]
    )
