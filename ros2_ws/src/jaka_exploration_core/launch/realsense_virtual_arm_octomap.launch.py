"""Attach a physical D435 point cloud to an externally launched mock JAKA arm.

The motion-planning stack must already publish:
  world -> base_link -> ... -> tool0

This launch adds:
  tool0 -> camera_link -> camera_depth_optical_frame

It is an integration test only. If the physical camera does not reproduce the
virtual arm motion, the resulting world-frame map is geometrically invalid.
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
    realsense_share = get_package_share_directory("realsense2_camera")
    motion_planning_share = get_package_share_directory("mp_team_bridge")

    start_motion_planning_arg = DeclareLaunchArgument(
        "start_motion_planning", default_value="true",
        description="Start mock JAKA, MoveIt, its RViz, and the integration task.",
    )
    plan_only_arg = DeclareLaunchArgument(
        "plan_only", default_value="false",
        description="Plan without executing the virtual arm trajectory.",
    )
    environment_file_arg = DeclareLaunchArgument(
        "environment_file",
        default_value="",
        description=(
            "Optional environment JSON for motion planning. Empty starts "
            "without a pre-saved OctoMap environment."
        ),
    )
    task_file_arg = DeclareLaunchArgument(
        "task_file",
        default_value="/root/ros2_ws/examples/viewpoint_task.json",
        description="Viewpoint task JSON consumed by the motion-planning test.",
    )
    task_delay_sec_arg = DeclareLaunchArgument(
        "task_delay_sec", default_value="5.0",
        description="Delay before sending the motion-planning test task.",
    )
    timeout_sec_arg = DeclareLaunchArgument(
        "timeout_sec", default_value="90.0",
        description="Motion-planning integration test timeout.",
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution", default_value="0.03",
        description="OctoMap voxel edge length in metres.",
    )
    max_range_arg = DeclareLaunchArgument(
        "max_range", default_value="2.0",
        description="Maximum point-cloud ray range inserted into OctoMap.",
    )
    save_octomap_arg = DeclareLaunchArgument(
        "save_octomap", default_value="false",
        description="Periodically save timestamped .bt and .ot files.",
    )
    save_interval_arg = DeclareLaunchArgument(
        "save_interval", default_value="60.0",
        description="Seconds between periodic OctoMap snapshots.",
    )
    save_directory_arg = DeclareLaunchArgument(
        "save_directory", default_value="/root/ros2_ws/maps",
        description="Directory for periodic OctoMap files.",
    )

    motion_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                motion_planning_share, "launch", "integration_test.launch.py"
            )
        ),
        launch_arguments={
            "plan_only": LaunchConfiguration("plan_only"),
            "environment_file": LaunchConfiguration("environment_file"),
            "task_file": LaunchConfiguration("task_file"),
            "task_delay_sec": LaunchConfiguration("task_delay_sec"),
            "timeout_sec": LaunchConfiguration("timeout_sec"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_motion_planning")),
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
            "initial_reset": "false",
        }.items(),
    )

    # Match jaka_a5_with_d435.urdf.xacro: the D435 forward (+X) axis is
    # aligned with the tool flange outward (+Z) axis.
    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="virtual_camera_mount_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "-1.5707963267948966", "--yaw", "0",
            "--frame-id", "tool0",
            "--child-frame-id", "camera_link",
        ],
    )

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="virtual_arm_realsense_octomap_server",
        output="screen",
        parameters=[{
            "resolution": ParameterValue(
                LaunchConfiguration("resolution"), value_type=float
            ),
            "frame_id": "world",
            "base_frame_id": "base_link",
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

    periodic_saver = Node(
        package="jaka_exploration_core",
        executable="periodic_octomap_saver",
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
        start_motion_planning_arg,
        plan_only_arg,
        environment_file_arg,
        task_file_arg,
        task_delay_sec_arg,
        timeout_sec_arg,
        resolution_arg,
        max_range_arg,
        save_octomap_arg,
        save_interval_arg,
        save_directory_arg,
        motion_planning,
        camera,
        camera_mount_tf,
        octomap,
        periodic_saver,
    ])
