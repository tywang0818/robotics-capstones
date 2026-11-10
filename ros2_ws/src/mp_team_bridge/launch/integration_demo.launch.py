from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = Path(get_package_share_directory("mp_team_bridge"))
    pipeline_share = Path(get_package_share_directory("jaka_motion_pipeline"))
    bridge_config = bridge_share / "config" / "bridge.yaml"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(pipeline_share / "launch" / "automatic_demo.launch.py")
                ),
                launch_arguments={
                    "start_sample_environment": "false",
                    "send_sample_task": "false",
                }.items(),
            ),
            Node(
                package="mp_team_bridge",
                executable="environment_bridge",
                name="environment_bridge",
                output="screen",
                parameters=[str(bridge_config)],
            ),
            Node(
                package="mp_team_bridge",
                executable="decision_bridge",
                name="decision_bridge",
                output="screen",
                parameters=[str(bridge_config)],
            ),
        ]
    )

