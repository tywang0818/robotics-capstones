"""Launch the YOLO target detection observer node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("semantic_frontier_ros"), "config", "target_detection.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameter file for target_detection_node",
            ),
            Node(
                package="semantic_frontier_ros",
                executable="target_detection_node",
                name="target_detection_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
