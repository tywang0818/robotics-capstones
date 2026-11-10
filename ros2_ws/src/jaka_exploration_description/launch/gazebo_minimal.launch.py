"""Minimal Gazebo Harmonic launch for the JAKA A5 + D435.

No MoveIt, no controllers, no RViz here. The single purpose is:
  Spawn the robot + D435 sensor inside Gazebo and bridge the D435 topics
  into ROS 2 so we can confirm the camera is actually producing data.

Once verified (rqt_image_view shows the color and depth streams), we will
extend this launch with controllers and MoveIt incrementally.
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
    # ------------------------------------------------------------------
    # Gazebo Harmonic translates `package://pkg/file` URIs in URDFs into
    # `model://pkg/file`. Then it searches for a directory named `pkg`
    # under GZ_SIM_RESOURCE_PATH. The parent of every ROS package's share
    # directory is the right thing to put on that path.
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
    pkg = "jaka_exploration_description"
    xacro_path = PathJoinSubstitution(
        [FindPackageShare(pkg), "urdf", "jaka_a5_with_d435.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(
            [FindExecutable(name="xacro"), " ", xacro_path, " sim_mode:=gazebo"]
        ),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
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

    return LaunchDescription([set_gz_resource_path, rsp, gz_sim, spawn, bridge])
