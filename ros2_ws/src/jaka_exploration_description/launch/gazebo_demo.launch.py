"""Full Gazebo Harmonic + MoveIt 2 demo of JAKA A5 + D435.

What this brings up:
  - Gazebo Harmonic with an empty world.
  - Robot spawned from our xacro (sim_mode:=gazebo).
  - gz_ros2_control plugin auto-starts controller_manager INSIDE Gazebo.
  - Controllers spawned: joint_state_broadcaster + jaka_a5_controller.
  - ros_gz_bridge: bridges /clock and the D435 sensor topics into ROS 2.
  - MoveIt 2 move_group, configured with our URDF.
  - RViz 2 with the teammate's MoveIt RViz config (so the MotionPlanning
    panel works for you to drag the end-effector).

What you can do once it is up:
  - In RViz, drag the orange interactive marker on the end-effector,
    click Plan & Execute. The arm physically moves in Gazebo, and the
    point cloud / color image streams update live.
  - To view the color image in RViz, Add -> by topic ->
    /camera/image -> Image.
  - To view the depth point cloud, Add -> by topic ->
    /camera/points -> PointCloud2.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    # ---------- MoveIt config with OUR URDF (sim_mode=gazebo) ----------
    our_xacro = str(
        Path(get_package_share_directory("jaka_exploration_description"))
        / "urdf"
        / "jaka_a5_with_d435.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .robot_description(file_path=our_xacro, mappings={"sim_mode": "gazebo"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ---------- Gazebo Harmonic ----------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # ---------- Robot State Publisher ----------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time, {"publish_frequency": 60.0}],
    )

    # ---------- Spawn robot into Gazebo from /robot_description ----------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "jaka_a5",
            "-topic", "/robot_description",
            "-z", "0.05",
        ],
        output="screen",
    )

    # ---------- ROS <-> Gazebo bridge ----------
    # NOTE: With `<topic>camera</topic>` on the sensor SDF, the rgbd_camera
    # sensor in Gazebo Harmonic publishes:
    #   /camera/image          (gz.msgs.Image)         color image
    #   /camera/depth_image    (gz.msgs.Image)         depth image
    #   /camera/points         (gz.msgs.PointCloudPacked)
    #   /camera/camera_info    (gz.msgs.CameraInfo)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # ---------- Controllers: spawn AFTER the robot lands in Gazebo ----------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_a5_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Sequence: spawn robot -> joint_state_broadcaster -> jaka_a5_controller
    seq_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    seq_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # ---------- MoveIt 2 move_group ----------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    # ---------- RViz 2 ----------
    rviz_config = str(
        Path(get_package_share_directory("jaka_a5_moveit_config"))
        / "config"
        / "moveit.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            rsp,
            spawn,
            bridge,
            seq_after_spawn,
            seq_after_jsb,
            move_group,
            rviz,
        ]
    )
