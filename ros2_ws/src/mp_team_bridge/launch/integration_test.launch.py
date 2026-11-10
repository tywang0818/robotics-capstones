from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_share = Path(get_package_share_directory("mp_team_bridge"))

    plan_only = LaunchConfiguration("plan_only")
    environment_file = LaunchConfiguration("environment_file")
    task_file = LaunchConfiguration("task_file")
    task_delay_sec = LaunchConfiguration("task_delay_sec")
    timeout_sec = LaunchConfiguration("timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument("environment_file", default_value=""),
            DeclareLaunchArgument("task_file", default_value=""),
            DeclareLaunchArgument("task_delay_sec", default_value="0.5"),
            DeclareLaunchArgument("timeout_sec", default_value="90.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bridge_share / "launch" / "integration_demo.launch.py")
                )
            ),
            Node(
                package="mp_team_bridge",
                executable="integration_test_client",
                name="integration_test_client",
                output="screen",
                parameters=[
                    {
                        "plan_only": ParameterValue(plan_only, value_type=bool),
                        "environment_file": environment_file,
                        "task_file": task_file,
                        "task_delay_sec": ParameterValue(task_delay_sec, value_type=float),
                        "timeout_sec": ParameterValue(timeout_sec, value_type=float),
                    }
                ],
            ),
        ]
    )
