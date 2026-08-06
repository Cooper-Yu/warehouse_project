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
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    # map_file selects matching clock and AMCL defaults.
    use_sim_time_value = ParameterValue(use_sim_time, value_type=bool)
    auto_initial_pose_enabled = PythonExpression([
        "'true' if '",
        auto_initial_pose,
        "'.lower() in ('true', '1') and '",
        use_sim_time,
        "'.lower() in ('true', '1') else 'false'",
    ])
    initial_covariance_x = LaunchConfiguration("initial_covariance_x")
    initial_covariance_y = LaunchConfiguration("initial_covariance_y")
    initial_covariance_yaw = LaunchConfiguration("initial_covariance_yaw")

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
            default_value="warehouse_map_keepout_sim.yaml",
            description=(
                "Saved map YAML file to load from the map_server config "
                "folder."
            ),
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
                "Start the localization-only RViz instance. Disabled by "
                "default "
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
        DeclareLaunchArgument(
            "auto_initial_pose",
            default_value=use_sim_time,
            description=(
                "Initialize AMCL from the configured pose in simulation. "
                "Defaults to use_sim_time so the course localization launch "
                "establishes particles and map TF before the mission starts."
            ),
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "initial_covariance_x",
            default_value="0.25",
            description="Initial x variance published on /initialpose.",
        ),
        DeclareLaunchArgument(
            "initial_covariance_y",
            default_value="0.25",
            description="Initial y variance published on /initialpose.",
        ),
        DeclareLaunchArgument(
            "initial_covariance_yaw",
            default_value="0.06853891945200942",
            description=(
                "Initial yaw variance published on /initialpose "
                "(15 degree standard deviation by default)."
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
            package="localization_server",
            executable="loaded_scan_filter.py",
            name="loaded_scan_filter",
            output="screen",
            condition=IfCondition(use_sim_time),
            parameters=[{"use_sim_time": use_sim_time_value}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                amcl_params,
                {
                    "use_sim_time": use_sim_time_value,
                    # The helper below publishes a PoseWithCovarianceStamped.
                    # Keep AMCL's parameter-only zero-covariance path disabled.
                    "set_initial_pose": False,
                },
            ],
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
            package="localization_server",
            executable="auto_initial_pose.py",
            name="auto_initial_pose",
            output="screen",
            condition=IfCondition(auto_initial_pose_enabled),
            parameters=[{
                "use_sim_time": use_sim_time_value,
                "initial_x": ParameterValue(initial_x, value_type=float),
                "initial_y": ParameterValue(initial_y, value_type=float),
                "initial_yaw": ParameterValue(
                    initial_yaw, value_type=float
                ),
                "covariance_x": ParameterValue(
                    initial_covariance_x, value_type=float
                ),
                "covariance_y": ParameterValue(
                    initial_covariance_y, value_type=float
                ),
                "covariance_yaw": ParameterValue(
                    initial_covariance_yaw, value_type=float
                ),
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
