"""Stage 2 (revised): exploration backbone with real-time OctoMap.

What this brings up, in order:
  1. Custom Gazebo world (robot + simple box).
  2. Robot spawned, ros2_control + controllers active (gz_ros2_control).
  3. ROS<->Gazebo bridge (camera + clock).
  4. octomap_server fuses /camera/points into a growing global OctoMap.
  5. MoveIt 2 move_group (teammate's config, our URDF).
  6. Teammate's motion-planning pipeline:
     motion_planner_node + environment_bridge + decision_bridge.
  7. RViz 2 with a BLACK-background config that only shows the robot,
     the OctoMap, and nothing else — this is the "exploration view"
     where unknown space stays black.
  8. initial_scan node: feeds the teammate's pipeline a sequence of
     wrist-only viewpoint goals (through the JSON API) so the camera
     sweeps multiple views and the OctoMap fills in automatically.
     MoveIt does the planning + execution end-to-end.

Run:
  ros2 launch jaka_exploration_core exploration_stage2.launch.py
Or skip the auto initial scan (e.g. you want to drive the arm manually):
  ros2 launch jaka_exploration_core exploration_stage2.launch.py auto_initial_scan:=false
Or run Gazebo server-only while keeping RViz and the simulation active:
  ros2 launch jaka_exploration_core exploration_stage2.launch.py gazebo_gui:=false
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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    auto_initial_scan_arg = DeclareLaunchArgument(
        "auto_initial_scan", default_value="true",
        description=(
            "If true, run initial_scan (empty-env + 6 in-place directions) "
            "for map bootstrap. Mutually exclusive with auto_bootstrap; "
            "prefer this for real exploration, auto_bootstrap only for "
            "the 5-viewpoint box test."
        ),
    )
    initial_scan_iterations_arg = DeclareLaunchArgument(
        "initial_scan_iterations", default_value="1",
        description=(
            "How many times initial_scan should repeat its full phase loop "
            "back-to-back within a single node lifetime. Each iteration "
            "publishes /viz_scan_diag_trigger with data='iter_N' so any "
            "diag node can snapshot per-iteration state without needing a "
            "wrapper to re-run initial_scan."
        ),
    )
    auto_bootstrap_arg = DeclareLaunchArgument(
        "auto_bootstrap", default_value="false",
        description=(
            "If true, run bootstrap_scan (5-viewpoint box test). Kept for "
            "verifying viewpoint IK / OctoMap render; not part of the real "
            "exploration flow."
        ),
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui", default_value="true",
        description="If false, run only the Gazebo server and keep RViz available.",
    )
    world_file_arg = DeclareLaunchArgument(
        "world_file", default_value="exploration_scene_multi.sdf",
        description=(
            "SDF filename inside jaka_exploration_core/worlds/ to load. "
            "Default = exploration_scene_multi.sdf (box + sphere + cylinder). "
            "Pass world_file:=exploration_scene.sdf for the single-box case."
        ),
    )
    start_semantic_arg = DeclareLaunchArgument(
        "start_semantic", default_value="false",
        description=(
            "If true, start semantic_frontier_ros semantic_inference_node on "
            "the Gazebo RGB-D camera stream."
        ),
    )

    # ------------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH so package:// mesh URIs resolve in Gazebo.
    # ------------------------------------------------------------------
    jaka_share_parent = os.path.dirname(get_package_share_directory("jaka_a5_description"))
    desc_share_parent = os.path.dirname(get_package_share_directory("jaka_exploration_description"))
    core_share_parent = os.path.dirname(get_package_share_directory("jaka_exploration_core"))
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = ":".join(
        [p for p in [core_share_parent, desc_share_parent, jaka_share_parent, existing] if p]
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    # ------------------------------------------------------------------
    # MoveIt config (our URDF, sim_mode=gazebo, teammate's SRDF + OMPL)
    # ------------------------------------------------------------------
    our_xacro = os.path.join(
        get_package_share_directory("jaka_exploration_description"),
        "urdf", "jaka_a5_with_d435.urdf.xacro",
    )
    # Use our extended SRDF (teammate's + extra disable_collisions for
    # camera_link). Without this MoveIt reports a J6 - camera_link
    # self-collision at start, error code -10.
    our_srdf = os.path.join(
        get_package_share_directory("jaka_exploration_core"),
        "config", "jaka_a5_with_d435.srdf",
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .robot_description(file_path=our_xacro, mappings={"sim_mode": "gazebo"})
        .robot_description_semantic(file_path=our_srdf)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time, {"publish_frequency": 60.0}],
    )

    # ------------------------------------------------------------------
    # Gazebo Harmonic + custom world
    # ------------------------------------------------------------------
    # world_path is resolved at LAUNCH TIME by substituting the
    # world_file argument into the share dir. gz_args must be a single
    # string but launch_arguments accepts a list-of-substitutions which
    # is joined together at execution.
    worlds_dir = os.path.join(
        get_package_share_directory("jaka_exploration_core"), "worlds"
    )
    world_path = PathJoinSubstitution([worlds_dir, LaunchConfiguration("world_file")])
    gz_args_prefix = IfElseSubstitution(
        LaunchConfiguration("gazebo_gui"),
        if_value="-r -v 3 ",
        else_value="-s -r -v 3 ",
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [gz_args_prefix, world_path]}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "jaka_a5",
            "-topic", "/robot_description",
            # The URDF world_fixed joint already carries the floor offset.
            "-z", "0.0",
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
    # Controllers (spawn after the robot lands in Gazebo)
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
    # MoveIt 2 move_group
    # ------------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    # ★ MULTI-PLANNER POOL (2026-07-20): spawn N extra move_group processes
    #   for plan_only probes.  Each is a fully independent OMPL instance
    #   (own memory space) so they can plan in parallel without racing.
    #   Master move_group above still owns the real /move_action for
    #   executes (has controller access).  probe_planners only serve
    #   plan_only requests routed by motion_planner's round-robin.
    #
    #   Naming: name=probe_planner_{i}, node namespace=probe_planner_{i}.
    #   MoveIt's action name relative to node namespace is /move_action, so
    #   from motion_planner we address it as /probe_planner_{i}/move_action.
    #   Same for the apply_planning_scene service.
    #
    #   Verified working (2026-07-20): 2 planners cut cycle 60s→30-40s,
    #   but shelf world is IK-bound so more cycles ≠ more successes.
    #   Currently DISABLED (probe_planner_count=0); flip to 2+ when the
    #   scene is IK-friendly (open workspace with reachable targets).
    probe_planner_count = 0
    probe_planners = []
    probe_move_group_actions = []
    probe_apply_scene_services = []
    for i in range(probe_planner_count):
        planner_name = f"probe_planner_{i}"
        probe_planners.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name=planner_name,
            namespace=planner_name,
            output="screen",
            parameters=[moveit_config.to_dict(), use_sim_time,
                        # disable execution --- probes are plan_only
                        {"allow_trajectory_execution": False}],
            # ★ MULTI-PLANNER POOL: bring the state / TF topics back to the
            #   global scope.  Namespacing move_group as /probe_planner_i
            #   otherwise makes it look for /probe_planner_i/joint_states,
            #   which doesn't exist --- MoveIt then falls back to a default
            #   start state (all zeros) that is in collision with the
            #   OctoMap and every probe fails with CheckStartStateCollision.
            remappings=[
                ("joint_states", "/joint_states"),
                ("robot_description", "/robot_description"),
                ("robot_description_semantic", "/robot_description_semantic"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ))
        probe_move_group_actions.append(
            f"/{planner_name}/move_action"
        )
        probe_apply_scene_services.append(
            f"/{planner_name}/apply_planning_scene"
        )

    # ------------------------------------------------------------------
    # Teammate's motion-planning pipeline:
    # motion_planner + environment_bridge + decision_bridge
    # ------------------------------------------------------------------
    pipeline_share = get_package_share_directory("jaka_motion_pipeline")
    bridge_share = get_package_share_directory("mp_team_bridge")

    # ★ MULTI-PLANNER POOL: only inject pool params when count>0.
    #   ROS 2 parameter type inference can't handle an empty list `[]`;
    #   passing one throws "got '()' of type tuple" during launch and
    #   shuts down the entire graph (rviz included).
    motion_planner_params = [
        os.path.join(pipeline_share, "config", "pipeline.yaml"),
        use_sim_time,
    ]
    if probe_planner_count > 0:
        motion_planner_params.append({
            "probe_move_group_actions": probe_move_group_actions,
            "probe_apply_scene_services": probe_apply_scene_services,
            "max_concurrent_plan_only": probe_planner_count,
        })
    motion_planner = Node(
        package="jaka_motion_pipeline", executable="motion_planner",
        name="motion_planner", output="screen",
        parameters=motion_planner_params,
    )
    env_bridge = Node(
        package="mp_team_bridge", executable="environment_bridge",
        name="environment_bridge", output="screen",
        parameters=[
            os.path.join(bridge_share, "config", "bridge.yaml"),
            use_sim_time,
        ],
    )
    dec_bridge_params = [
        os.path.join(bridge_share, "config", "bridge.yaml"),
        use_sim_time,
    ]
    if probe_planner_count > 0:
        dec_bridge_params.append(
            {"max_parallel_probes": probe_planner_count}
        )
    dec_bridge = Node(
        package="mp_team_bridge", executable="decision_bridge",
        name="decision_bridge", output="screen",
        parameters=dec_bridge_params,
    )
    known_free_validator = Node(
        package="known_free_validator",
        executable="known_free_validator_node",
        name="known_free_validator",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "environment_topic": "/known_environment",
                "validation_service": "/validate_known_free_trajectory",
                "conservative_map_service": "/build_conservative_octomap",
                "joint_states_topic": "/joint_states",
            },
            use_sim_time,
        ],
    )
    metrics_reporter = Node(
        package="mp_team_bridge",
        executable="motion_metrics_reporter",
        name="motion_metrics_reporter",
        output="screen",
        parameters=[
            {
                "result_topic": "/team/motion_result_json",
                "report_interval_sec": 60.0,
                "report_directory":
                    "/root/ros2_ws/runtime_reports/motion_planning",
            },
            use_sim_time,
        ],
    )
    pipeline_after_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_ctrl,
            on_exit=[
                motion_planner,
                env_bridge,
                dec_bridge,
                known_free_validator,
                metrics_reporter,
            ],
        )
    )

    # ------------------------------------------------------------------
    # RViz 2 — BLACK background, only RobotModel + OctoMap
    # ------------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("jaka_exploration_core"),
        "config", "exploration_view.rviz",
    )
    rviz = Node(
        package="rviz2", executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            use_sim_time,
        ],
    )

    # ------------------------------------------------------------------
    # octomap_server (real-time mapping from /camera/points)
    # ------------------------------------------------------------------
    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {
                # 3 cm voxels (0.05 -> 0.03). Finer than 0.05 for
                # tighter geometry around obstacles but ~4.6x more
                # leaves per volume vs 0.05. Still much lighter than
                # the original 0.01. octomap_free_region's sample
                # spacing (0.025) is fine enough to cover every 0.03
                # voxel inside the injection cylinders.
                "resolution": 0.03,
                "frame_id": "world",
                "base_frame_id": "base_link",
                # max_range determines how far each ray marks cells as
                # free (rays longer than this are clipped; endpoints
                # beyond are NOT marked occupied). 3 m gives up-facing
                # scans enough vertical reach so the free-space cap
                # above the arm is visible (previously 1.5 m stopped
                # the free coverage right at the arm height).
                "sensor_model.max_range": 3.0,
                "sensor_model.hit": 0.7,
                "sensor_model.miss": 0.45,
                "sensor_model.min": 0.12,
                "sensor_model.max": 0.85,
                "occupancy_thres": 0.5,
                # Publish free-voxel MarkerArray on /free_cells_vis_array
                # so we can visualise the known-free volume alongside
                # occupied+frontier - lets us verify visually whether
                # a frontier point that "looks wrong" is actually at the
                # boundary of a real known/unknown transition, or is
                # inside a genuine known-free region.
                "publish_free_space": True,
                "filter_ground": False,
                "pointcloud_min_z": -1.0,
                "pointcloud_max_z": 3.0,
                # ros_gz_bridge publishes /camera/points BEST_EFFORT;
                # tell the subscription to match.
                "qos_overrides./camera/points.subscription.reliability": "best_effort",
                "qos_overrides./camera/points.subscription.durability": "volatile",
                "qos_overrides./camera/points.subscription.history": "keep_last",
                "qos_overrides./camera/points.subscription.depth": 5,
            },
            use_sim_time,
        ],
        remappings=[
            ("cloud_in", "/camera/points_filtered"),
            # Redirect the raw octomap so octomap_free_region can
            # post-process it into the FINAL /octomap_binary that
            # everyone else consumes.
            ("octomap_binary", "/octomap_binary_raw"),
            ("octomap_full", "/octomap_full_raw"),
        ],
    )

    # ------------------------------------------------------------------
    # octomap_free_region — post-process octomap_server's binary output:
    # inject FREE leaves into a configured cylinder (the structural
    # blind zone around the folded scanning arm) so downstream sees it
    # as known-free instead of unknown.
    # ------------------------------------------------------------------
    octomap_free_region = Node(
        package="octomap_free_region",
        executable="octomap_free_region_node",
        name="octomap_free_region",
        output="screen",
        parameters=[
            {
                "input_topic": "/octomap_binary_raw",
                "output_topic": "/octomap_binary",
                # One FREE cylinder + one OCC floor-seal disk (机械臂立柱):
                #   FREE 柱: r<=0.30 覆盖机械臂自身立柱;z=[0.05, 1.25] —— 起点
                #     从 0.02 抬到 0.05,不再碰 [0,0.03] 地板层 voxel,避免把基座
                #     正下方“遮挡导致的 unknown 地板”填成 free 而在地板打洞。
                #   ★ 柱1(head disk, r=0.95)已删除。
                "cylinder_x":     [0.0],
                "cylinder_y":     [0.0],
                "cylinder_r":     [0.30],
                "cylinder_z_min": [0.05],
                "cylinder_z_max": [1.25],
                # OCC 地板盘:把基座正下方(被本体遮挡、永远观测不到)的 unknown
                #   地板层填成 OCCUPIED,让 FREE 柱底部贴“已知地板”而非 unknown。
                #   r=0.35(略大于 FREE 柱,封住边缘);z=[-0.15, 0.04](地板层及
                #   其下,止于 FREE 柱起点 0.05 之下)。这样:
                #     free(z>=0.05) ↔ occupied(z<0.05) = free-occupied,不是 frontier;
                #     occupied ↔ 地下 unknown = occupied-unknown,也不是 frontier。
                #   → 彻底消除“地板洞 + 洞下高价值假 frontier”。
                "occ_cylinder_x":     [0.0],
                "occ_cylinder_y":     [0.0],
                "occ_cylinder_r":     [0.35],
                "occ_cylinder_z_min": [-0.15],
                "occ_cylinder_z_max": [0.04],
                "sample_spacing": 0.025,
                "enable": True,
            },
            use_sim_time,
        ],
    )

    # ------------------------------------------------------------------
    # self_filter — strips the robot's own arm from /camera/points before
    # OctoMap insertion. Without this, the camera sees its own links
    # during inter-viewpoint motion and OctoMap builds voxels around the
    # arm; those voxels then enter MoveIt's PlanningScene and cause
    # CheckStartStateCollision to reject every subsequent plan
    # (contacts: octomap - J1/J2/J3/base_link).
    # ------------------------------------------------------------------
    self_filter = Node(
        package="jaka_exploration_core",
        executable="self_filter",
        name="self_filter",
        output="screen",
        parameters=[use_sim_time],
    )

    # ------------------------------------------------------------------
    # initial_scan — 15 s after launch, publishes an empty environment
    # (so PlanningScene has no world obstacles) and sends 6 viewpoint
    # goals from the current tool0 position, sweeping the camera through
    # 6 world-frame directions to bootstrap the OctoMap. This is the
    # default startup path for exploration.
    # ------------------------------------------------------------------
    initial_scan = Node(
        package="jaka_exploration_core",
        executable="initial_scan",
        name="initial_scan",
        output="screen",
        parameters=[
            use_sim_time,
            {"iterations": LaunchConfiguration("initial_scan_iterations")},
        ],
        condition=IfCondition(LaunchConfiguration("auto_initial_scan")),
    )
    initial_scan_delayed = TimerAction(
        period=15.0,
        actions=[initial_scan],
    )

    # ------------------------------------------------------------------
    # bootstrap_scan — off by default. Kept for the box-render smoke
    # test (5 viewpoints aimed at the known box) but no longer part of
    # the real exploration flow.
    # ------------------------------------------------------------------
    bootstrap_scan = Node(
        package="jaka_exploration_core",
        executable="bootstrap_scan",
        name="bootstrap_scan",
        output="screen",
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("auto_bootstrap")),
    )
    bootstrap_scan_delayed = TimerAction(
        period=15.0,
        actions=[bootstrap_scan],
    )

    # ------------------------------------------------------------------
    # octomap_to_json_bridge — streams the real-time OctoMap from
    # octomap_server into /team/optimized_environment_json so that
    # motion-planning re-plans against the latest explored world.
    #
    # Wired to start when whichever startup scan finishes (initial_scan
    # or bootstrap_scan), so the scan's own env_version publishes stay
    # authoritative while it runs, and the bridge takes over the
    # /team topic immediately after. If neither scan is enabled, the
    # bridge starts right away.
    #
    # A previous version used a fixed 100 s TimerAction here; that left
    # env_status silent for ~85 s while initial_scan ran (initial_scan
    # publishes exactly one seed env then goes quiet), which starved
    # both the TSP node and decision_bridge and then flooded them at
    # once — triggering decision_bridge's strict env-version equality
    # check on every subsequent task.
    # ------------------------------------------------------------------
    octomap_bridge_parameters = [
        {
            "input_topic": "/octomap_binary",
            "output_topic": "/team/optimized_environment_json",
            "publish_rate_hz": 1.0,
            "min_data_bytes": 1,
        },
        use_sim_time,
    ]
    octomap_to_json_bridge_after_initial_scan = Node(
        package="jaka_exploration_core",
        executable="octomap_to_json_bridge",
        name="octomap_to_json_bridge",
        output="screen",
        parameters=octomap_bridge_parameters,
        condition=IfCondition(LaunchConfiguration("auto_initial_scan")),
    )
    octomap_bridge_after_initial_scan = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_scan,
            on_exit=[octomap_to_json_bridge_after_initial_scan],
        ),
        condition=IfCondition(LaunchConfiguration("auto_initial_scan")),
    )
    octomap_to_json_bridge_after_bootstrap = Node(
        package="jaka_exploration_core",
        executable="octomap_to_json_bridge",
        name="octomap_to_json_bridge",
        output="screen",
        parameters=octomap_bridge_parameters,
        condition=IfCondition(LaunchConfiguration("auto_bootstrap")),
    )
    octomap_bridge_after_bootstrap = RegisterEventHandler(
        OnProcessExit(
            target_action=bootstrap_scan,
            on_exit=[octomap_to_json_bridge_after_bootstrap],
        ),
        condition=IfCondition(LaunchConfiguration("auto_bootstrap")),
    )
    # When both scans are disabled, launch the bridge immediately.
    # Uses a NotSubstitution-free encoding: two UnlessConditions is
    # equivalent to (not auto_initial_scan and not auto_bootstrap) at
    # launch validation time.
    octomap_to_json_bridge_immediate_a = Node(
        package="jaka_exploration_core",
        executable="octomap_to_json_bridge",
        name="octomap_to_json_bridge",
        output="screen",
        parameters=octomap_bridge_parameters,
        condition=UnlessCondition(LaunchConfiguration("auto_initial_scan")),
    )
    octomap_to_json_bridge_immediate = TimerAction(
        period=0.0,
        actions=[octomap_to_json_bridge_immediate_a],
        condition=UnlessCondition(LaunchConfiguration("auto_bootstrap")),
    )

    # ------------------------------------------------------------------
    # image_relay — ros_gz_bridge publishes /camera/image with
    # BEST_EFFORT QoS, but rqt_image_view subscribes RELIABLE and has
    # no UI to switch. This relay rebroadcasts the same frames on
    # /camera/image_rel with RELIABLE QoS so rqt_image_view can see
    # them. Cheap; just a memcpy per frame.
    # ------------------------------------------------------------------
    image_relay = Node(
        package="jaka_exploration_core",
        executable="image_relay",
        name="image_relay",
        output="screen",
        parameters=[use_sim_time],
    )

    # ------------------------------------------------------------------
    # frontier_viz — parses /octomap_binary in Python each time it
    # updates, finds free voxels adjacent to unknown, publishes them as
    # red POINTS MarkerArray on /frontier_markers.  Visualization-only:
    # no planning, no goals, no filters. Refreshes automatically at
    # OctoMap update rate (~1 Hz) so the frontier is live as the camera
    # explores.
    # ------------------------------------------------------------------
    frontier_viz = Node(
        package="jaka_exploration_core",
        executable="frontier_viz",
        name="frontier_viz",
        output="screen",
        parameters=[use_sim_time],
    )

    semantic_inference = Node(
        package="semantic_frontier_ros",
        executable="semantic_inference_node",
        name="semantic_inference_node",
        output="screen",
        parameters=[
            use_sim_time,
            {
                "rgb_topic": "/camera/image_rel",
                "depth_topic": "/camera/depth_image",
                "camera_info_topic": "/camera/camera_info",
                "checkpoint_path": "/root/ros2_ws/model",
                "label_space": "ade150",
                "image_size": 512,
                "semantic_points_topic": "/semantic_frontier/semantic_points",
                "relevance_heatmap_topic": "/semantic_frontier/relevance_heatmap",
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_semantic")),
    )

    return LaunchDescription(
        [
            auto_initial_scan_arg,
            initial_scan_iterations_arg,
            auto_bootstrap_arg,
            gazebo_gui_arg,
            world_file_arg,
            start_semantic_arg,
            set_gz_resource_path,
            rsp,
            gz_sim,
            spawn_robot,
            bridge,
            seq_after_robot,
            seq_after_jsb,
            move_group,
            *probe_planners,   # ★ MULTI-PLANNER POOL: N extra move_group procs
            pipeline_after_ctrl,
            rviz,
            octomap_server,
            octomap_free_region,
            self_filter,
            initial_scan_delayed,
            bootstrap_scan_delayed,
            octomap_bridge_after_initial_scan,
            octomap_bridge_after_bootstrap,
            octomap_to_json_bridge_immediate,
            image_relay,
            frontier_viz,
            semantic_inference,
        ]
    )
