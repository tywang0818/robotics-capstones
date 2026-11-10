"""Real JAKA A5 + real RealSense D435 -> OctoMap in world frame.

Full stack for real-arm + real-camera initial-scan mapping.  Loads the URDF,
publishes the world->base_link static TF, drives the D435 with the standard
realsense2_camera driver, runs a self-filter that drops robot-body points,
and accumulates the remainder into an OctoMap in the world frame.

Three modes of use:

  1. Mapping only (default) --- do NOT talk to the arm.  Useful while the
     arm is being posed by a teach pendant.
       ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py

  2. Mapping + read-only arm observer (won't move the arm).  Adds
     jaka_state_publisher, which only calls login() + polls positions.
     Use this if you plan to command motion from a separate terminal
     (real_robot_test_basic/initial_scan.py etc).
       ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py \\
           start_jaka_reader:=true

  3. Single-command full run: mapping + 6-direction initial scan.
     Launches jaka_scan_and_publish which owns the sole JAKA connection
     and both publishes /joint_states AND runs the scan trajectory.
     Adds a 15s startup delay + 5s in-node countdown before motion so the
     mapping stack has time to stabilise.
       ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py \\
           auto_scan:=true

Modes 2 and 3 are MUTUALLY EXCLUSIVE --- JAKA firmware only allows one
active connection, so start_jaka_reader:=true + auto_scan:=true will make
one of them fail.  The launch does NOT enforce this; pick one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    core_share = get_package_share_directory("jaka_exploration_core")
    desc_share = get_package_share_directory("jaka_exploration_description")
    realsense_share = get_package_share_directory("realsense2_camera")

    urdf_xacro = os.path.join(
        desc_share, "urdf", "jaka_a5_with_d435.urdf.xacro"
    )
    # sim_mode:=mock keeps the URDF parseable but we do NOT start
    # controller_manager, so mock_components is never instantiated and it
    # cannot compete with the real arm's joint states.
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro, " sim_mode:=mock"]),
            value_type=str,
        )
    }

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="10.5.5.100",
        description="JAKA A5 controller IP address.",
    )
    jaka_rate_arg = DeclareLaunchArgument(
        "jaka_rate_hz", default_value="50.0",
        description="Rate at which jaka_state_publisher polls joint positions.",
    )
    jaka_standalone_arg = DeclareLaunchArgument(
        "jaka_standalone", default_value="false",
        description=(
            "If true, jaka_state_publisher also calls power_on + enable_robot. "
            "Leave false when a separate motion script owns the connection."
        ),
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution", default_value="0.03",
        description="OctoMap voxel edge length in metres.",
    )
    max_range_arg = DeclareLaunchArgument(
        "max_range", default_value="2.0",
        description="Maximum raycast range inserted into OctoMap in metres.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Start RViz with RobotModel + PointCloud + OctoMap displays.",
    )
    start_jaka_reader_arg = DeclareLaunchArgument(
        "start_jaka_reader", default_value="false",
        description=(
            "If true, launch jaka_state_publisher (read-only observer).  "
            "Set to false when jaka_scan_and_publish is used instead --- "
            "JAKA only allows one client at a time, so the two must not "
            "coexist."
        ),
    )
    auto_scan_arg = DeclareLaunchArgument(
        "auto_scan", default_value="false",
        description=(
            "If true, launch jaka_scan_and_publish inside this launch "
            "so that the entire real-arm initial scan runs from a single "
            "command.  Mutually exclusive with start_jaka_reader."
        ),
    )
    scan_delay_arg = DeclareLaunchArgument(
        "scan_delay", default_value="15.0",
        description=(
            "Seconds to wait after launch start before spawning "
            "jaka_scan_and_publish.  Gives realsense2_camera, "
            "robot_state_publisher and octomap_server time to be ready."
        ),
    )
    scan_run_mode_arg = DeclareLaunchArgument(
        "scan_run_mode", default_value="scan_only",
        description=(
            "run_mode passed to jaka_scan_and_publish.  "
            "'scan_only'         --- do the 6-dir scan and exit; "
            "'scan_then_explore' --- do the scan, then keep the "
            "FollowJointTrajectory action server running for MoveIt to "
            "drive exploration; "
            "'explore_only'      --- skip the scan, go straight to "
            "serving trajectory goals."
        ),
    )

    # ---- TF backbone -----------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"publish_frequency": 60.0, "use_sim_time": False},
        ],
    )
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    # ---- Real arm bridge (READS joint states only) -----------------------
    jaka_state_publisher = Node(
        package="jaka_exploration_core",
        executable="jaka_state_publisher",
        name="jaka_state_publisher",
        output="screen",
        parameters=[{
            "robot_ip": LaunchConfiguration("robot_ip"),
            "rate_hz": ParameterValue(
                LaunchConfiguration("jaka_rate_hz"), value_type=float
            ),
            "standalone": ParameterValue(
                LaunchConfiguration("jaka_standalone"), value_type=bool
            ),
            "use_sim_time": False,
        }],
        condition=IfCondition(LaunchConfiguration("start_jaka_reader")),
    )

    # ---- Real RealSense driver ------------------------------------------
    # NOTE: The URDF (jaka_a5_with_d435.urdf.xacro) already publishes the
    # tool0 -> camera_link joint via robot_state_publisher.  The realsense
    # driver internally publishes camera_link -> camera_depth_optical_frame
    # etc., completing the TF chain that OctoMap needs.  Do NOT add another
    # static_transform_publisher for tool0 -> camera_link here or the two
    # publishers will race.
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
            # ★ Decimation:在驱动层把深度图降到 320×240(点数 1/4),源头减负——
            #   DDS / self_filter / octomap 三处同时受益。0.03m 的地图在这个距离
            #   下 320×240 仍远够密(1m 处每体素仍 ~40 点,2m 处 ~10 点),地图
            #   质量不掉;octomap 光线数砍到 1/4 → 插入约 4× 快 → 有效帧率翻两番,
            #   scan 恢复 10°/s 也不会因静默丢帧留随机 unknown 缝。
            "decimation_filter.enable": "true",
            "decimation_filter.filter_magnitude": "2",
            "initial_reset": "false",
            "publish_tf": "true",
        }.items(),
    )

    # ---- OctoMap in world frame -----------------------------------------
    # Drop points that hit the arm itself (approximated with bounding
    # spheres per link, hand-tuned in self_filter.py).  Reads the raw
    # RealSense cloud, republishes a cleaned version with the SAME frame
    # and timestamp so octomap_server's raycasting still sees the correct
    # sensor origin.
    self_filter = Node(
        package="jaka_exploration_core",
        executable="self_filter",
        name="self_filter",
        output="screen",
        parameters=[{
            "input_topic": "/camera/camera/depth/color/points",
            "output_topic": "/camera/points_filtered",
            "tf_timeout_sec": 0.2,
            "use_sim_time": False,
        }],
    )

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="real_arm_realsense_octomap_server",
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
            "publish_free_space": True,
            "filter_ground": False,
            "pointcloud_min_z": -10.0,
            "pointcloud_max_z": 10.0,
            "use_sim_time": False,
            "qos_overrides./camera/points_filtered.subscription.reliability": "best_effort",
            "qos_overrides./camera/points_filtered.subscription.durability": "volatile",
            "qos_overrides./camera/points_filtered.subscription.history": "keep_last",
            "qos_overrides./camera/points_filtered.subscription.depth": 5,
        }],
        remappings=[
            ("cloud_in", "/camera/points_filtered"),
            # Redirect raw octomap output so octomap_free_region can
            # post-process it into the FINAL /octomap_binary that
            # downstream (motion_planner, viz, HPPT) subscribes to.
            ("octomap_binary", "/octomap_binary_raw"),
            ("octomap_full",   "/octomap_full_raw"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="real_arm_octomap_rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(core_share, "config", "exploration_view.rviz"),
        ],
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ---- Optional: single-launch scan (owns the JAKA connection) --------
    # Deferred by scan_delay so realsense/robot_state_publisher/octomap
    # come up before motion starts and the mapping stack captures the
    # entire trajectory.  TimerAction's `period` needs a concrete float,
    # so we resolve the LaunchConfigurations inside an OpaqueFunction.
    def _spawn_scan_with_delay(context):
        auto = LaunchConfiguration("auto_scan").perform(context).lower()
        if auto not in ("true", "1", "yes", "on"):
            return []
        delay = float(LaunchConfiguration("scan_delay").perform(context))
        rate = float(LaunchConfiguration("jaka_rate_hz").perform(context))
        ip = LaunchConfiguration("robot_ip").perform(context)
        run_mode = LaunchConfiguration("scan_run_mode").perform(context)
        return [
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package="jaka_exploration_core",
                        executable="jaka_scan_and_publish",
                        name="jaka_scan_and_publish",
                        output="screen",
                        emulate_tty=True,
                        parameters=[{
                            "robot_ip": ip,
                            "publish_rate_hz": rate,
                            "auto_confirm": True,
                            "run_mode": run_mode,
                            "use_sim_time": False,
                        }],
                    ),
                ],
            ),
        ]
    scan_and_publish_action = OpaqueFunction(function=_spawn_scan_with_delay)

    return LaunchDescription([
        robot_ip_arg,
        jaka_rate_arg,
        jaka_standalone_arg,
        resolution_arg,
        max_range_arg,
        rviz_arg,
        start_jaka_reader_arg,
        auto_scan_arg,
        scan_delay_arg,
        scan_run_mode_arg,
        robot_state_publisher,
        world_to_base_tf,
        jaka_state_publisher,
        camera,
        self_filter,
        octomap_server,
        rviz,
        scan_and_publish_action,
    ])
