from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value="warehouse_map_sim.yaml",
            description="Map YAML file to load from the map_server config folder.",
        ),
    ])
