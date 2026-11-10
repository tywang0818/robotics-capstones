"""Visualize the JAKA A5 + D435 URDF in RViz without MoveIt or Gazebo.

Use this to quickly confirm the description package builds, the xacro
parses cleanly, and the camera is mounted where we expect on the
end-effector.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="mock",
        description="URDF hardware backend: mock or gazebo",
    )
    use_joint_gui_arg = DeclareLaunchArgument(
        "use_joint_gui",
        default_value="true",
        description="If true, run joint_state_publisher_gui so joints can be moved.",
    )

    pkg_share = FindPackageShare("jaka_exploration_description")
    xacro_path = PathJoinSubstitution(
        [pkg_share, "urdf", "jaka_a5_with_d435.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " sim_mode:=",
            LaunchConfiguration("sim_mode"),
        ]
    )
    # Wrap as ParameterValue with explicit str type. Without this, ROS 2
    # Jazzy's parameter parser tries to YAML-parse the giant URDF string
    # and fails.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # GUI version when use_joint_gui:=true
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=__import__("launch.conditions").conditions.IfCondition(
            LaunchConfiguration("use_joint_gui")
        ),
    )
    # Non-GUI fallback when use_joint_gui:=false
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=__import__("launch.conditions").conditions.UnlessCondition(
            LaunchConfiguration("use_joint_gui")
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            use_joint_gui_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz,
        ]
    )
