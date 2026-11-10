from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                respawn=True,
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    {"publish_frequency": 60.0},
                ],
            )
        ]
    )
