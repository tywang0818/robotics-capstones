"""Use a physical D435 OctoMap as the mock JAKA MoveIt collision world.

This launch starts one integrated pipeline:

  physical D435 point cloud
    -> octomap_server (/octomap_binary, frame=world)
    -> octomap_to_json_bridge
    -> environment_bridge (/known_environment)
    -> motion_planner
    -> MoveIt PlanningScene.world.octomap

The mock JAKA stack publishes world -> ... -> tool0.  A static mount transform
connects tool0 to the RealSense TF tree.  This is an integration test: if the
physical camera does not reproduce the virtual tool motion, the accumulated
world-frame map is geometrically invalid even though collision checking works.
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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    realsense_share = get_package_share_directory("realsense2_camera")
    bridge_share = get_package_share_directory("mp_team_bridge")
    pipeline_share = get_package_share_directory("jaka_motion_pipeline")
    moveit_share = get_package_share_directory("jaka_a5_moveit_config")

    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.03",
        description="OctoMap voxel edge length in metres.",
    )
    max_range_arg = DeclareLaunchArgument(
        "max_range",
        default_value="2.0",
        description="Maximum point-cloud ray range inserted into OctoMap.",
    )
    environment_rate_arg = DeclareLaunchArgument(
        "environment_rate",
        default_value="1.0",
        description="Rate in Hz for forwarding complete OctoMap snapshots.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description=(
            "Start observation-only RViz with RobotModel/OctoMap displays. "
            "This does not load the MoveIt MotionPlanning panel."
        ),
    )
    start_hppt_arg = DeclareLaunchArgument(
        "start_hppt",
        default_value="false",
        description="Start HPP-PT so it can create tasks from the live OctoMap.",
    )
    save_octomap_arg = DeclareLaunchArgument(
        "save_octomap",
        default_value="false",
        description="Periodically save timestamped .bt and .ot files.",
    )
    save_interval_arg = DeclareLaunchArgument(
        "save_interval",
        default_value="60.0",
        description="Seconds between periodic OctoMap snapshots.",
    )
    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="/root/ros2_ws/maps",
        description="Directory for periodic OctoMap files.",
    )

    # Non-interactive mock JAKA + MoveIt stack.
    #
    # Do not include jaka_a5_moveit_config/demo.launch.py or
    # mp_team_bridge/integration_demo.launch.py here: those are convenient
    # demos, but they can bring up MoveIt RViz / manual planning affordances.
    # This launch intentionally exposes only the JSON task path:
    #   HPP-PT -> /team/viewpoint_task_json -> decision_bridge
    #          -> motion_planner -> move_group -> jaka_a5_controller
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 60.0},
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
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            os.path.join(moveit_share, "config", "ros2_controllers.yaml"),
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_a5_controller", "-c", "/controller_manager"],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    motion_planner = Node(
        package="jaka_motion_pipeline",
        executable="motion_planner",
        name="motion_planner",
        output="screen",
        parameters=[os.path.join(pipeline_share, "config", "pipeline.yaml")],
    )
    env_bridge = Node(
        package="mp_team_bridge",
        executable="environment_bridge",
        name="environment_bridge",
        output="screen",
        parameters=[os.path.join(bridge_share, "config", "bridge.yaml")],
    )
    dec_bridge = Node(
        package="mp_team_bridge",
        executable="decision_bridge",
        name="decision_bridge",
        output="screen",
        parameters=[os.path.join(bridge_share, "config", "bridge.yaml")],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="realsense_exploration_rviz",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("jaka_exploration_core"),
                "config",
                "exploration_view.rviz",
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
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
            # A USB reset can detach a usbipd device from WSL.
            "initial_reset": "false",
        }.items(),
    )

    # Match jaka_a5_with_d435.urdf.xacro: RealSense forward (+X) is aligned
    # with the tool flange outward axis (+Z).
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

    octomap_server = Node(
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
            "publish_free_space": True,
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

    # Converts the binary occupancy tree without changing its occupancy data.
    # environment_bridge converts it back to OctomapWithPose, and
    # motion_planner assigns it to PlanningScene.world.octomap before planning.
    planning_scene_bridge = Node(
        package="jaka_exploration_core",
        executable="octomap_to_json_bridge",
        name="realsense_octomap_to_environment",
        output="screen",
        parameters=[{
            "input_topic": "/octomap_binary",
            "output_topic": "/team/optimized_environment_json",
            "publish_rate_hz": ParameterValue(
                LaunchConfiguration("environment_rate"), value_type=float
            ),
            "min_data_bytes": 1,
        }],
    )

    hppt = Node(
        package="hppt_boundary_ros",
        executable="hppt_boundary_ros",
        name="hppt_boundary_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_hppt")),
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
        resolution_arg,
        max_range_arg,
        environment_rate_arg,
        rviz_arg,
        start_hppt_arg,
        save_octomap_arg,
        save_interval_arg,
        save_directory_arg,
        robot_state_publisher,
        world_to_base_tf,
        ros2_control,
        joint_state_broadcaster,
        arm_controller,
        move_group,
        motion_planner,
        env_bridge,
        dec_bridge,
        rviz,
        camera,
        camera_mount_tf,
        octomap_server,
        planning_scene_bridge,
        hppt,
        periodic_saver,
    ])
