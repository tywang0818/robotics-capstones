from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    generated_srdf = Path.home() / ".ros" / "jaka_a5_board_states.srdf"
    if not generated_srdf.exists():
        return LaunchDescription(
            [
                LogInfo(
                    msg=(
                        "Missing ~/.ros/jaka_a5_board_states.srdf. Run "
                        "'ros2 run jaka_motion_pipeline export_board_states' first."
                    )
                )
            ]
        )

    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": generated_srdf.read_text()
    }
    description = generate_demo_launch(moveit_config)
    pipeline_config = (
        Path(get_package_share_directory("jaka_motion_pipeline"))
        / "config"
        / "pipeline.yaml"
    )
    description.add_action(
        Node(
            package="jaka_motion_pipeline",
            executable="motion_planner",
            name="motion_planner",
            output="screen",
            parameters=[str(pipeline_config)],
        )
    )
    description.add_action(
        Node(
            package="jaka_motion_pipeline",
            executable="example_environment",
            name="example_environment",
            output="screen",
        )
    )
    return description
