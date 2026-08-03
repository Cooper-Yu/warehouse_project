from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    use_rviz = LaunchConfiguration("use_rviz")
    # map_file selects matching clock and AMCL defaults; both remain overridable.
    use_sim_time_value = ParameterValue(use_sim_time, value_type=bool)

    map_yaml = PathJoinSubstitution([
        FindPackageShare("map_server"),
        "config",
        map_file,
    ])

    amcl_params = PathJoinSubstitution([
        FindPackageShare("localization_server"),
        "config",
        amcl_config,
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("localization_server"),
        "rviz",
        "localization.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value="warehouse_map_sim.yaml",
            description="Saved map YAML file to load from the map_server config folder.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=PythonExpression([
                "'sim' in '", map_file, "'.lower()"
            ]),
            description=(
                "Use simulation clock. Defaults from map_file: sim map=true, "
                "real map=false."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description=(
                "Start the localization-only RViz instance. Disabled by default "
                "because the Checkpoint 12 path planner launch owns RViz."
            ),
        ),
        DeclareLaunchArgument(
            "amcl_config",
            default_value=PythonExpression([
                "'amcl_config_sim.yaml' if '",
                use_sim_time,
                "'.lower() in ('true', '1') else 'amcl_config_real.yaml'",
            ]),
            description=(
                "AMCL parameter file. Defaults to the sim or real config "
                "selected by use_sim_time."
            ),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time_value,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params, {"use_sim_time": use_sim_time_value}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time_value,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time_value}],
        ),
    ])
