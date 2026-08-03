from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    parameters = PathJoinSubstitution([
        FindPackageShare("shelf_detection_server"),
        "config",
        "real_entry.yaml",
    ])

    return LaunchDescription([
        Node(
            package="shelf_detection_server",
            executable="shelf_detection_server",
            name="shelf_detection_server",
            output="screen",
            parameters=[parameters],
        ),
    ])
