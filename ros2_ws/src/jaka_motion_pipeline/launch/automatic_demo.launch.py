from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config_dir = Path(get_package_share_directory("jaka_motion_pipeline"))
    jaka_config_dir = Path(get_package_share_directory("jaka_a5_moveit_config"))

    start_sample_environment = LaunchConfiguration("start_sample_environment")
    send_sample_task = LaunchConfiguration("send_sample_task")
    plan_only = LaunchConfiguration("plan_only")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_sample_environment", default_value="true"),
            DeclareLaunchArgument("send_sample_task", default_value="false"),
            DeclareLaunchArgument("plan_only", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(jaka_config_dir / "launch" / "demo.launch.py")
                )
            ),
            Node(
                package="jaka_motion_pipeline",
                executable="motion_planner",
                name="motion_planner",
                output="screen",
                parameters=[str(config_dir / "config" / "pipeline.yaml")],
            ),
            Node(
                package="jaka_motion_pipeline",
                executable="example_environment",
                name="example_environment",
                output="screen",
                condition=IfCondition(start_sample_environment),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="jaka_motion_pipeline",
                        executable="example_task",
                        name="example_task",
                        output="screen",
                        parameters=[
                            {"plan_only": ParameterValue(plan_only, value_type=bool)}
                        ],
                        condition=IfCondition(send_sample_task),
                    )
                ],
            ),
        ]
    )
