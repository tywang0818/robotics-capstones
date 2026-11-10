"""Full Gazebo Harmonic + MoveIt 2 + RViz launch for JAKA A5 + D435.

What this brings up (one command, ~20 seconds to fully start):

  1. Gazebo Harmonic (empty world) with the JAKA A5 + D435 spawned.
  2. gz_ros2_control plugin auto-starts inside Gazebo and exposes a
     /controller_manager service.
  3. joint_state_broadcaster + jaka_a5_controller controllers are spawned
     once the robot is in Gazebo.
  4. ros_gz_bridge bridges /clock and the D435 camera topics.
  5. MoveIt 2 move_group with our URDF (sim_mode:=gazebo) + teammate's
     MoveIt SRDF + OMPL planning.
  6. RViz 2 with the MoveIt Motion Planning panel.

What you do in RViz:
  - In the MotionPlanning panel, drag the orange end-effector marker.
  - Click "Plan & Execute".
  - The arm physically moves in Gazebo, /camera/image and /camera/points
    update in real time, and you can watch them in rqt_image_view / RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    # ------------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH so Gazebo can find STL meshes referenced via
    # `package://jaka_a5_description/...` URIs in the URDF.
    # ------------------------------------------------------------------
    jaka_share_parent = os.path.dirname(
        get_package_share_directory("jaka_a5_description")
    )
    ours_share_parent = os.path.dirname(
        get_package_share_directory("jaka_exploration_description")
    )
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = ":".join(
        [p for p in [ours_share_parent, jaka_share_parent, existing_resource_path] if p]
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    # ------------------------------------------------------------------
    # MoveIt config with OUR URDF (sim_mode=gazebo). The robot name stays
    # "jaka_a5" so the teammate's SRDF still matches.
    # ------------------------------------------------------------------
    our_xacro = os.path.join(
        get_package_share_directory("jaka_exploration_description"),
        "urdf",
        "jaka_a5_with_d435.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .robot_description(file_path=our_xacro, mappings={"sim_mode": "gazebo"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # robot_state_publisher uses the SAME robot_description that
    # MoveIt + RViz will use.
    # ------------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time, {"publish_frequency": 60.0}],
    )

    # ------------------------------------------------------------------
    # Gazebo Harmonic + spawn + bridge (proven to work from minimal).
    # ------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

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

    # ------------------------------------------------------------------
    # Controller spawners.
    # gz_ros2_control launches its own controller_manager INSIDE Gazebo
    # after the model is loaded, so we sequence:
    #   spawn-robot -> spawn joint_state_broadcaster -> spawn arm controller.
    # ------------------------------------------------------------------
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_a5_controller", "-c", "/controller_manager"],
        output="screen",
    )

    seq_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawn_joint_state_broadcaster])
    )
    seq_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )

    # ------------------------------------------------------------------
    # MoveIt 2 move_group.
    # ------------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    # ------------------------------------------------------------------
    # RViz 2 with the teammate's MoveIt RViz config.
    # ------------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("jaka_a5_moveit_config"),
        "config",
        "moveit.rviz",
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
            set_gz_resource_path,
            rsp,
            gz_sim,
            spawn,
            bridge,
            seq_after_spawn,
            seq_after_jsb,
            move_group,
            rviz,
        ]
    )
