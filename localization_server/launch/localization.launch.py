from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

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
            default_value="true",
            description="Use simulation clock when true.",
        ),
        DeclareLaunchArgument(
            "amcl_config",
            default_value="amcl_config_sim.yaml",
            description="AMCL parameter file in the localization_server config folder.",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
