from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    remove = LaunchConfiguration("remove")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "remove",
                default_value="false",
                description="Remove the example obstacles instead of adding them.",
            ),
            Node(
                package="jaka_a5_moveit_config",
                executable="add_collision_objects.py",
                name="jaka_collision_object_loader",
                output="screen",
                parameters=[{"remove": ParameterValue(remove, value_type=bool)}],
            ),
        ]
    )
