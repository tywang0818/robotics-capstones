from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("semantic_frontier_ros"))
    default_config = package_share / "config" / "semantic_inference.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_config),
                description="YAML parameter file for semantic_inference_node.",
            ),
            Node(
                package="semantic_frontier_ros",
                executable="semantic_inference_node",
                name="semantic_inference_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )

