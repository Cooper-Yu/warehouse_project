import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_basename = LaunchConfiguration("configuration_basename")

    def launch_setup(context, *args, **kwargs):
        package_share = get_package_share_directory("cartographer_slam")
        config_dir = os.path.join(package_share, "config")
        rviz_config = os.path.join(package_share, "rviz", "mapping.rviz")

        selected_config = context.perform_substitution(configuration_basename)
        if not selected_config:
            sim_time_value = context.perform_substitution(use_sim_time).lower()
            selected_config = (
                "cartographer_sim.lua"
                if sim_time_value in ("true", "1", "yes")
                else "cartographer_real.lua"
            )

        cartographer_node = Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory",
                config_dir,
                "-configuration_basename",
                selected_config,
            ],
        )

        occupancy_grid_node = Node(
            package="cartographer_ros",
            executable="occupancy_grid_node",
            name="occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        )

        return [cartographer_node, occupancy_grid_node, rviz_node]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock when true.",
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="",
            description=(
                "Cartographer Lua file in the config folder. "
                "Empty selects cartographer_sim.lua for sim time and "
                "cartographer_real.lua for real time."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
