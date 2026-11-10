"""Stage 1: bring up the custom exploration world with the JAKA + D435,
plus the ROS<->Gazebo bridge. No MoveIt, no controllers, no perception,
no FSM yet.

Single goal: confirm the world loads, the robot spawns standing up, and
the D435 streams are visible in rqt_image_view.

Builds on the patterns proven in jaka_exploration_description/launch/
gazebo_minimal.launch.py (uses the same URDF, same bridge args, same
GZ_SIM_RESOURCE_PATH trick).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----- GZ_SIM_RESOURCE_PATH so meshes resolve via package:// URIs -----
    jaka_share_parent = os.path.dirname(
        get_package_share_directory("jaka_a5_description")
    )
    desc_share_parent = os.path.dirname(
        get_package_share_directory("jaka_exploration_description")
    )
    core_share_parent = os.path.dirname(
        get_package_share_directory("jaka_exploration_core")
    )
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = ":".join(
        [p for p in [core_share_parent, desc_share_parent, jaka_share_parent, existing] if p]
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    # ----- robot_description from our xacro (sim_mode=gazebo) -----
    xacro_path = PathJoinSubstitution(
        [FindPackageShare("jaka_exploration_description"),
         "urdf", "jaka_a5_with_d435.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path, " sim_mode:=gazebo"]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # ----- Gazebo Harmonic with our custom world -----
    world_path = os.path.join(
        get_package_share_directory("jaka_exploration_core"),
        "worlds", "exploration_scene.sdf",
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world_path}"}.items(),
    )

    # ----- spawn the robot -----
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "jaka_a5",
            "-topic", "/robot_description",
            # NOTE: the URDF has <link name="world"/>, which Gazebo treats as
            # the world frame itself, so -x/-y/-z on the spawn are IGNORED
            # for that link. To shift the robot down we instead put the
            # offset inside the URDF (see world_fixed joint).
            "-z", "0.0",
        ],
        output="screen",
    )

    # ----- ROS<->Gazebo bridge: clock + D435 streams -----
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

    return LaunchDescription([set_gz_resource_path, rsp, gz_sim, spawn_robot, bridge])
