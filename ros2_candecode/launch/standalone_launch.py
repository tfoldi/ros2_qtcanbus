from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

logger = LaunchConfiguration("log-level")

config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_candecode"), "config", "candecode.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log-level",
                default_value=["info"],
                description="Logging level",
            ),
            Node(
                package="ros2_candecode",
                executable="candecode",
                name="candecode_node",
                output="log",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", logger],
            ),
        ]
    )
