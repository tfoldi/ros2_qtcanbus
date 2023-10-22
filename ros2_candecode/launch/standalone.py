from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_candecode"), "config", "candecode.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_candecode",
                executable="candecode",
                name="candecode_node",
                output="log",
                parameters=[config_file],
            ),
        ]
    )
