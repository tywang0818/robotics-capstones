"""Offline FK verification: publish URDF + fake initial-scan joint_states, plus
the standard camera_link -> camera_color_optical_frame static TF that the
real RealSense driver would publish, so that a `tf2_echo
camera_color_optical_frame base_link` reveals where the base appears in the
image at the initial scan home pose.

Nothing here talks to the real arm or the real camera.  Pure TF math to
answer: given URDF conventions, which side of the image should base_link
project to at joints [0, pi/2, -pi/2, pi/2, pi/2, 0]?

Usage:
  ros2 launch jaka_exploration_core fk_verify.launch.py
  # then in another terminal:
  ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("jaka_exploration_description")
    urdf_xacro = os.path.join(
        desc_share, "urdf", "jaka_a5_with_d435.urdf.xacro"
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro, " sim_mode:=mock"]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"publish_frequency": 30.0}],
    )

    # Fake joint_state_publisher: pin all 6 joints to the initial-scan home
    # pose.  Using joint_state_publisher (not _gui) keeps this headless.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "source_list": [],
            "rate": 10.0,
            # zero_pose keys are joint names; values are the frozen positions.
            "zeros": {
                "joint_1": 0.0,
                "joint_2": 1.5707963267948966,
                "joint_3": -1.5707963267948966,
                "joint_4": 1.5707963267948966,
                "joint_5": 1.5707963267948966,
                "joint_6": 0.0,
            },
        }],
    )

    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        output="screen",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    # What realsense2_camera would normally publish for the RGB optical frame.
    # Body (X forward, Y left, Z up) -> Optical (X right, Y down, Z forward)
    # is rpy = (-pi/2, 0, -pi/2).
    camera_link_to_color_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_to_color_optical",
        output="screen",
        arguments=[
            "--roll", "-1.5707963267948966",
            "--pitch", "0",
            "--yaw", "-1.5707963267948966",
            "--frame-id", "camera_link",
            "--child-frame-id", "camera_color_optical_frame",
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        world_to_base,
        camera_link_to_color_optical,
    ])
