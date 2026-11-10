"""Full demo: JAKA + D435 in Gazebo Harmonic, table + inspection_board as
physical obstacles, teammate's motion-planning pipeline running, MoveIt
plans a collision-free path to inspect the board from above, and the arm
physically executes the trajectory in Gazebo while the D435 stream updates.

Layout matches the teammate's examples/environment_v1.json so MoveIt and
Gazebo see the SAME obstacle geometry:
  - "table":           1.4 x 1.4 x 0.1 m  at (0.35, 0.0, -0.08)
  - "inspection_board" 0.3 x 0.4 x 0.03 m at (0.35, 0.0,  0.72)

The teammate's integration_test_client is spawned with a delay so that
move_group + bridges are ready before it sends environment + task JSON.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# ---------- inspection scene SDF strings ----------
# Geometry mirrors examples/environment_v1.json (in `world` frame). Both
# obstacles are static (no physics on them) so the arm collides into solid
# objects.
TABLE_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="table">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>1.4 1.4 0.1</size></box></geometry>
        <material>
          <ambient>0.55 0.40 0.25 1</ambient>
          <diffuse>0.65 0.50 0.30 1</diffuse>
        </material>
      </visual>
      <collision name="c">
        <geometry><box><size>1.4 1.4 0.1</size></box></geometry>
      </collision>
    </link>
    <pose>0.35 0.0 -0.08 0 0 0</pose>
  </model>
</sdf>"""

INSPECTION_BOARD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="inspection_board">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>0.3 0.4 0.03</size></box></geometry>
        <material>
          <ambient>0.20 0.65 0.20 1</ambient>
          <diffuse>0.25 0.80 0.25 1</diffuse>
          <specular>0.30 0.30 0.30 1</specular>
        </material>
      </visual>
      <collision name="c">
        <geometry><box><size>0.3 0.4 0.03</size></box></geometry>
      </collision>
    </link>
    <pose>0.35 0.0 0.72 0 0 0</pose>
  </model>
</sdf>"""


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    auto_run_test_arg = DeclareLaunchArgument(
        "auto_run_test", default_value="true",
        description="If true, spawn integration_test_client to send env + task automatically.",
    )
    plan_only_arg = DeclareLaunchArgument(
        "plan_only", default_value="false",
        description="When integration_test_client runs: only plan (no execute) if true.",
    )

    # ------------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH
    # ------------------------------------------------------------------
    jaka_share_parent = os.path.dirname(
        get_package_share_directory("jaka_a5_description")
    )
    ours_share_parent = os.path.dirname(
        get_package_share_directory("jaka_exploration_description")
    )
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = ":".join(
        [p for p in [ours_share_parent, jaka_share_parent, existing] if p]
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    # ------------------------------------------------------------------
    # MoveIt config (our URDF, sim_mode=gazebo)
    # ------------------------------------------------------------------
    our_xacro = os.path.join(
        get_package_share_directory("jaka_exploration_description"),
        "urdf", "jaka_a5_with_d435.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .robot_description(file_path=our_xacro, mappings={"sim_mode": "gazebo"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            use_sim_time,
            {"publish_frequency": 60.0},
        ],
    )

    # ------------------------------------------------------------------
    # Gazebo Harmonic + spawn robot + spawn obstacles + bridge
    # ------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "jaka_a5",
            "-topic", "/robot_description",
            "-z", "0.05",
        ],
        output="screen",
    )

    # NOTE: ros_gz_sim create ignores the `<pose>` element inside the SDF
    # when spawning via `-string`. The pose must be supplied through the
    # explicit -x/-y/-z (and -R/-P/-Y) flags. Otherwise both models land
    # at the world origin and overlap with the robot.
    spawn_table = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "table",
            "-string", TABLE_SDF,
            "-x", "0.35", "-y", "0.0", "-z", "-0.08",
        ],
        output="screen",
    )

    spawn_inspection_board = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "inspection_board",
            "-string", INSPECTION_BOARD_SDF,
            "-x", "0.35", "-y", "0.0", "-z", "0.72",
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
    # Controllers
    # ------------------------------------------------------------------
    spawn_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    spawn_arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["jaka_a5_controller", "-c", "/controller_manager"],
        output="screen",
    )

    seq_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])
    )
    seq_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm_ctrl])
    )

    # ------------------------------------------------------------------
    # MoveIt 2 + RViz
    # ------------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )
    rviz_config = os.path.join(
        get_package_share_directory("jaka_a5_moveit_config"),
        "config", "moveit.rviz",
    )
    rviz = Node(
        package="rviz2", executable="rviz2",
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

    # ------------------------------------------------------------------
    # Teammate's motion-planning pipeline (motion_planner + bridges)
    # ------------------------------------------------------------------
    pipeline_share = get_package_share_directory("jaka_motion_pipeline")
    bridge_share = get_package_share_directory("mp_team_bridge")

    motion_planner = Node(
        package="jaka_motion_pipeline", executable="motion_planner",
        name="motion_planner", output="screen",
        parameters=[
            os.path.join(pipeline_share, "config", "pipeline.yaml"),
            use_sim_time,
        ],
    )
    env_bridge = Node(
        package="mp_team_bridge", executable="environment_bridge",
        name="environment_bridge", output="screen",
        parameters=[
            os.path.join(bridge_share, "config", "bridge.yaml"),
            use_sim_time,
        ],
    )
    dec_bridge = Node(
        package="mp_team_bridge", executable="decision_bridge",
        name="decision_bridge", output="screen",
        parameters=[
            os.path.join(bridge_share, "config", "bridge.yaml"),
            use_sim_time,
        ],
    )

    # integration_test_client — sends env + task automatically after delay.
    example_dir = "/root/ros2_ws/examples"
    test_client = Node(
        package="mp_team_bridge", executable="integration_test_client",
        name="integration_test_client", output="screen",
        parameters=[
            {
                "plan_only": ParameterValue(LaunchConfiguration("plan_only"), value_type=bool),
                "environment_file": f"{example_dir}/environment_v1.json",
                "task_file": f"{example_dir}/viewpoint_task.json",
                "timeout_sec": 120.0,
            },
            use_sim_time,
        ],
    )

    # Teammate pipeline starts after the arm controller is alive; test
    # client lags by another few seconds so move_group is ready.
    pipeline_after_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_ctrl,
            on_exit=[motion_planner, env_bridge, dec_bridge],
        )
    )
    test_client_delayed = TimerAction(
        period=8.0,
        actions=[test_client],
    )

    return LaunchDescription(
        [
            auto_run_test_arg,
            plan_only_arg,
            set_gz_resource_path,
            rsp,
            gz_sim,
            spawn_robot,
            spawn_table,
            spawn_inspection_board,
            bridge,
            seq_after_robot,
            seq_after_jsb,
            move_group,
            rviz,
            pipeline_after_ctrl,
            test_client_delayed,
        ]
    )
