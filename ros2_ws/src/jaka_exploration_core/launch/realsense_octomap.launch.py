"""Build a fixed-frame OctoMap directly from a physical RealSense D435.

This launch intentionally does not use Gazebo or simulated joint states.
Until the real JAKA driver publishes joint_states, the map is fixed in the
camera_depth_optical_frame and is valid only while the physical arm is still.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    core_share = get_package_share_directory("jaka_exploration_core")
    realsense_share = get_package_share_directory("realsense2_camera")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Start RViz with the fixed RealSense optical frame.",
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution", default_value="0.03",
        description="OctoMap voxel edge length in metres.",
    )
    max_range_arg = DeclareLaunchArgument(
        "max_range", default_value="2.0",
        description="Maximum point-cloud ray range inserted into OctoMap in metres.",
    )
    save_octomap_arg = DeclareLaunchArgument(
        "save_octomap", default_value="false",
        description="Periodically save timestamped .bt and .ot OctoMap files.",
    )
    save_interval_arg = DeclareLaunchArgument(
        "save_interval", default_value="60.0",
        description="Seconds between periodic OctoMap snapshots.",
    )
    save_directory_arg = DeclareLaunchArgument(
        "save_directory", default_value="/root/ros2_ws/maps",
        description="Directory in which periodic OctoMap files are stored.",
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_profile": "640x480x15",
            # Avoid a USB hardware reset: under WSL USB/IP it may detach
            # the camera from the distribution.
            "initial_reset": "false",
        }.items(),
    )

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="realsense_octomap_server",
        output="screen",
        parameters=[{
            "resolution": ParameterValue(
                LaunchConfiguration("resolution"), value_type=float
            ),
            "frame_id": "camera_depth_optical_frame",
            "base_frame_id": "camera_depth_optical_frame",
            "sensor_model.max_range": ParameterValue(
                LaunchConfiguration("max_range"), value_type=float
            ),
            "sensor_model.hit": 0.7,
            "sensor_model.miss": 0.45,
            "sensor_model.min": 0.12,
            "sensor_model.max": 0.85,
            "occupancy_thres": 0.5,
            "filter_ground": False,
            "pointcloud_min_z": -10.0,
            "pointcloud_max_z": 10.0,
            "qos_overrides./camera/camera/depth/color/points.subscription.reliability": "best_effort",
            "qos_overrides./camera/camera/depth/color/points.subscription.durability": "volatile",
            "qos_overrides./camera/camera/depth/color/points.subscription.history": "keep_last",
            "qos_overrides./camera/camera/depth/color/points.subscription.depth": 5,
        }],
        remappings=[
            ("cloud_in", "/camera/camera/depth/color/points"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(core_share, "config", "realsense_octomap.rviz"),
            "-f",
            "camera_depth_optical_frame",
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    periodic_saver = Node(
        package="jaka_exploration_core",
        executable="periodic_octomap_saver",
        name="periodic_octomap_saver",
        output="screen",
        parameters=[{
            "save_interval": ParameterValue(
                LaunchConfiguration("save_interval"), value_type=float
            ),
            "save_directory": LaunchConfiguration("save_directory"),
        }],
        condition=IfCondition(LaunchConfiguration("save_octomap")),
    )

    return LaunchDescription([
        rviz_arg,
        resolution_arg,
        max_range_arg,
        save_octomap_arg,
        save_interval_arg,
        save_directory_arg,
        camera,
        octomap,
        rviz,
        periodic_saver,
    ])
