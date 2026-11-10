"""Full real-arm autonomous exploration.

Layers on top of real_arm_realsense_octomap.launch.py:
  base layer  : robot_state_publisher, RealSense driver, self_filter,
                octomap_server (world frame), RViz, jaka_scan_and_publish
                in run_mode="scan_then_explore" (does the 6-dir initial
                scan first, then keeps its FollowJointTrajectory action
                server up for MoveIt).
  this layer  : MoveIt move_group, motion_planner (jaka_motion_pipeline),
                environment_bridge + decision_bridge (mp_team_bridge),
                octomap_to_json_bridge, hppt_boundary_ros, frontier_viz.

Data flow after initial scan finishes:
  /octomap_binary
    -> octomap_to_json_bridge  ->  /team/optimized_environment_json
    -> environment_bridge      ->  /known_environment (EnvironmentModel)
    -> motion_planner (caches env; applies via /apply_planning_scene)

  hppt_boundary_ros (subscribes /octomap_binary + /known_environment)
    -> /team/viewpoint_task_json
    -> decision_bridge          ->  motion_planner.PlanMotion action
    -> motion_planner           ->  move_group.MoveGroup action
    -> move_group               ->  our FollowJointTrajectory server on
                                    /jaka_a5_controller/follow_joint_trajectory
    -> jaka_scan_and_publish    ->  jkrc.joint_move at SLOW_SPEED_DPS
    -> feedback loop closes; HPP-PT picks the next viewpoint.

All speeds are inherited from jaka_scan_and_publish: MoveIt plans at any
speed it likes, but our executor rate-limits every waypoint to
SLOW_SPEED_DPS (5 deg/s) and polls for arrival before advancing.

Typical usage:

  ros2 launch jaka_exploration_core real_arm_exploration.launch.py

That is all --- one terminal, one command.  The launch bundles the base
sensor stack via IncludeLaunchDescription so the two launches never drift.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    core_share = get_package_share_directory("jaka_exploration_core")
    desc_share = get_package_share_directory("jaka_exploration_description")
    pipeline_share = get_package_share_directory("jaka_motion_pipeline")
    bridge_share = get_package_share_directory("mp_team_bridge")

    # ---- Launch arguments -----------------------------------------------
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="10.5.5.100",
        description="JAKA A5 controller IP address.",
    )
    scan_delay_arg = DeclareLaunchArgument(
        "scan_delay", default_value="15.0",
        description=(
            "Seconds to wait after launch start before spawning "
            "jaka_scan_and_publish (so RealSense / MoveIt / TF are ready)."
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
        description="Start RViz.",
    )
    start_hppt_arg = DeclareLaunchArgument(
        "start_hppt", default_value="true",
        description=(
            "Whether to start hppt_boundary_ros (the actual exploration "
            "brain).  Turn off if you only want the plumbing up for "
            "manual PlanMotion calls."
        ),
    )
    env_pub_rate_arg = DeclareLaunchArgument(
        "environment_rate", default_value="1.0",
        description=(
            "Hz at which octomap_to_json_bridge republishes the OctoMap "
            "as a versioned JSON environment snapshot."
        ),
    )
    # These control the motion_planner's default speed/acceleration when
    # a viewpoint task does not override them.  MoveIt uses these to time-
    # parameterise the planned trajectory; our shim then ignores those
    # times and paces at SLOW_SPEED_DPS, but a low value keeps the
    # trajectory shape smooth (fewer high-velocity segments to chase).
    default_velocity_arg = DeclareLaunchArgument(
        "default_velocity_scaling", default_value="0.05",
        description=(
            "MoveIt velocity_scaling for exploration trajectories "
            "(0.05 = 5% of joint max velocity).  Kept low so the trajectory "
            "shape is smooth and our SLOW_SPEED_DPS executor doesn't have "
            "to chase implausible per-waypoint speeds."
        ),
    )
    default_accel_arg = DeclareLaunchArgument(
        "default_acceleration_scaling", default_value="0.05",
        description="MoveIt acceleration_scaling for exploration.",
    )
    report_directory_arg = DeclareLaunchArgument(
        "motion_report_directory",
        default_value="/root/ros2_ws/runtime_reports/motion_planning",
        description="Directory for one-minute motion-planning reports.",
    )

    # ---- Base sensor + arm stack (via include) --------------------------
    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                core_share, "launch",
                "real_arm_realsense_octomap.launch.py",
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "resolution": LaunchConfiguration("resolution"),
            "max_range": LaunchConfiguration("max_range"),
            "rviz": LaunchConfiguration("rviz"),
            # We do NOT want the base launch's read-only observer; the
            # scan-and-publish node owns the sole allowed JAKA connection.
            "start_jaka_reader": "false",
            # Auto-launch the scan+publish node inside the base launch,
            # in the mode that keeps the FollowJointTrajectory server up
            # after the scan finishes.
            "auto_scan": "true",
            "scan_run_mode": "scan_then_explore",
            "scan_delay": LaunchConfiguration("scan_delay"),
        }.items(),
    )

    # ---- MoveIt config --------------------------------------------------
    # Same URDF + our extended SRDF (adds camera_link disable_collisions).
    urdf_xacro = os.path.join(
        desc_share, "urdf", "jaka_a5_with_d435.urdf.xacro"
    )
    srdf = os.path.join(
        core_share, "config", "jaka_a5_with_d435.srdf"
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_a5", package_name="jaka_a5_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            mappings={"sim_mode": "mock"},  # avoid gazebo tags in URDF
        )
        .robot_description_semantic(file_path=srdf)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                # We do NOT run controller_manager on the real arm.  Tell
                # MoveIt not to try to query it for controller state;
                # trajectories flow directly to our FollowJointTrajectory
                # action server.
                "moveit_manage_controllers": False,
                # Very loose execution timing tolerances because our
                # executor deliberately paces every waypoint slower than
                # MoveIt's plan.
                "trajectory_execution.allowed_execution_duration_scaling":
                    50.0,
                "trajectory_execution.allowed_goal_duration_margin":
                    120.0,
                "trajectory_execution.execution_duration_monitoring":
                    False,
            },
        ],
    )

    # ---- motion_planner + bridges (verbatim from sim) -------------------
    motion_planner = Node(
        package="jaka_motion_pipeline",
        executable="motion_planner",
        name="motion_planner",
        output="screen",
        parameters=[
            os.path.join(pipeline_share, "config", "pipeline.yaml"),
            {
                "use_sim_time": False,
                # Override the defaults so trajectories are slow-and-smooth.
                "default_velocity_scaling": ParameterValue(
                    LaunchConfiguration("default_velocity_scaling"),
                    value_type=float,
                ),
                "default_acceleration_scaling": ParameterValue(
                    LaunchConfiguration("default_acceleration_scaling"),
                    value_type=float,
                ),
            },
        ],
    )
    env_bridge = Node(
        package="mp_team_bridge",
        executable="environment_bridge",
        name="environment_bridge",
        output="screen",
        parameters=[
            os.path.join(bridge_share, "config", "bridge.yaml"),
            {
                "use_sim_time": False,
            },
        ],
    )
    dec_bridge = Node(
        package="mp_team_bridge",
        executable="decision_bridge",
        name="decision_bridge",
        output="screen",
        parameters=[
            os.path.join(bridge_share, "config", "bridge.yaml"),
            {
                "use_sim_time": False,
                "default_velocity_scaling": ParameterValue(
                    LaunchConfiguration("default_velocity_scaling"),
                    value_type=float,
                ),
                "default_acceleration_scaling": ParameterValue(
                    LaunchConfiguration("default_acceleration_scaling"),
                    value_type=float,
                ),
            },
        ],
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
                "use_sim_time": False,
            },
        ],
    )
    metrics_reporter = Node(
        package="mp_team_bridge",
        executable="motion_metrics_reporter",
        name="motion_metrics_reporter",
        output="screen",
        parameters=[{
            "result_topic": "/team/motion_result_json",
            "report_interval_sec": 60.0,
            "report_directory": LaunchConfiguration("motion_report_directory"),
            "use_sim_time": False,
        }],
    )

    # ---- OctoMap free-region post-processor -----------------------------
    # Intercepts /octomap_binary_raw from octomap_server (see the
    # remapping in real_arm_realsense_octomap.launch.py), injects FREE
    # leaves into the structural-blind cylinders (arm J1 column + head
    # disk from the folded scan pose), then republishes on the original
    # /octomap_binary that everything downstream subscribes to.
    octomap_free_region = Node(
        package="octomap_free_region",
        executable="octomap_free_region_node",
        name="octomap_free_region",
        output="screen",
        parameters=[{
            "input_topic":    "/octomap_binary_raw",
            "output_topic":   "/octomap_binary",
            # cyl 0: arm J1-axis column (arm body + folded blind cylinder).
            # cyl 1: head disk (r=0.95, z 0.85..1.25) covering the ring
            #        around the camera + the cap above the folded head.
            "cylinder_x":     [0.0,  0.0],
            "cylinder_y":     [0.0,  0.0],
            "cylinder_r":     [0.30, 0.95],
            "cylinder_z_min": [0.02, 0.85],
            "cylinder_z_max": [1.25, 1.25],
            "sample_spacing": 0.025,
            "enable":         True,
            "use_sim_time":   False,
        }],
    )

    # ---- OctoMap -> JSON environment snapshots --------------------------
    octomap_to_json_bridge = Node(
        package="jaka_exploration_core",
        executable="octomap_to_json_bridge",
        name="octomap_to_json_bridge",
        output="screen",
        parameters=[{
            "input_topic": "/octomap_binary",
            "output_topic": "/team/optimized_environment_json",
            "publish_rate_hz": ParameterValue(
                LaunchConfiguration("environment_rate"), value_type=float
            ),
            "min_data_bytes": 1,
            "enable_topic": "/jaka_initial_scan_complete",
            "use_sim_time": False,
        }],
    )

    # ---- Frontier visualisation (RViz only, no planning) ----------------
    frontier_viz = Node(
        package="jaka_exploration_core",
        executable="frontier_viz",
        name="frontier_viz",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # ---- HPP-PT: the actual viewpoint / exploration brain ---------------
    #   frontier_max_range_factor 1.5 → 2.5 so shelf frontiers up to
    #     2.125m from base survive Stage 2 too_far filter.
    #   z_max 0.85 → 1.20 so cabinet-top frontiers survive Stage 2
    #     forbidden(z-oob) filter; still within workspace shell.
    hppt = Node(
        package="hppt_boundary_ros",
        executable="hppt_boundary_ros",
        name="hppt_boundary_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "frontier_max_range_factor": 2.5,
            "z_max": 1.20,
        }],
        condition=IfCondition(LaunchConfiguration("start_hppt")),
    )

    # ---- Timing ---------------------------------------------------------
    # move_group needs a few seconds to load OMPL / SRDF etc.  Start
    # motion_planner + bridges only after MoveIt is ready so their first
    # /apply_planning_scene call succeeds.  HPP-PT starts a bit later so
    # environment_bridge has already published a seed environment.
    pipeline_after_moveit = TimerAction(
        period=8.0,
        actions=[
            motion_planner,
            env_bridge,
            dec_bridge,
            known_free_validator,
            metrics_reporter,
            octomap_to_json_bridge,
        ],
    )
    hppt_after_pipeline = TimerAction(
        period=20.0,
        actions=[hppt],
    )

    # ★ 收敛 → 先复位回 home → 再停整个系统:
    #   hppt 收敛(ALL EXECUTE FAILED / B2 EMPTY)会 rclcpp::shutdown() 退出自己;
    #   此时 motion_planner / bridge / jaka_scan_and_publish 等还在(独立进程),
    #   所以先跑 go_home(MoveIt 规划避障回初始位姿),等它完成后再 emit 全局
    #   Shutdown,把整个 launch 的所有节点一起收掉。
    reset_before_shutdown = ExecuteProcess(
        cmd=["ros2", "run", "jaka_exploration_core", "go_home"],
        output="screen",
    )
    reset_on_hppt_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=hppt,
            on_exit=[reset_before_shutdown],   # hppt 退出 → 启动复位
        )
    )
    shutdown_after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_before_shutdown,   # 复位完成(不管成败)→ 关整个系统
            on_exit=[EmitEvent(event=Shutdown(
                reason="reset (go_home) done -> shutting down the whole launch"))],
        )
    )

    # ---- 旁路 YOLO 叠加节点(bypass overlay)------------------------------
    #   只订阅相机 RGB,发布画好检测框的 /yolo/annotated/compressed;完全旁路,
    #   不发布任何主系统消费的 topic,不参与探索决策。YOLO 推理跑 GPU,不抢
    #   CPU 控制回路。前端视频源默认指向它 → 视频流即带框画面。
    yolo_overlay_arg = DeclareLaunchArgument(
        "yolo_overlay", default_value="true",
        description="启动旁路 YOLO 叠加节点(相机 RGB → 画框 → /yolo/annotated/compressed)。",
    )
    web_image_topic_arg = DeclareLaunchArgument(
        "web_image_topic", default_value="/yolo/annotated/compressed",
        description="前端视频流订阅的图像 topic。默认=YOLO 标注图(带框);"
                    "关掉 YOLO 时改回 /camera/camera/color/image_raw/compressed。",
    )
    yolo_overlay = Node(
        package="jaka_exploration_core",
        executable="yolo_overlay",
        name="yolo_overlay",
        output="screen",
        # nice 降优先级 + 节点内限 torch 线程:GPU 不兼容时回退 CPU 推理,
        #   这样不与主系统的实时控制回路抢 CPU。
        #   ★ prefix 必须是单个字符串(内部再分词);传 list 会被拼成 "nice-n10"
        #     这个不存在的命令 → 节点启动 FileNotFoundError。
        prefix="nice -n 10",
        parameters=[{
            "input_topic": "/camera/camera/color/image_raw/compressed",
            "output_topic": "/yolo/annotated/compressed",
            "model_path": "/root/ros2_ws/yolov8n.pt",
            "device": "auto",            # 试 GPU,不兼容自动回退 CPU
            "confidence_threshold": 0.35,
            "cpu_threads": 4,            # CPU 推理线程上限,防吃满核
            "max_rate_hz": 15.0,         # 与相机帧率一致,够用又省 CPU
        }],
        condition=IfCondition(LaunchConfiguration("yolo_overlay")),
    )

    # ---- 语义分割节点(SegFormer → semantic_points,供 hppt 语义打分)------------
    #   与 YOLO 平级的独立旁路节点。订阅相机 RGB + 对齐 depth + camera_info,
    #   SegFormer 推理出每点语义相关度,发布 /semantic_frontier/semantic_points;
    #   hppt 默认订阅并纳入 frontier 打分(use_semantic_probability)。
    #   CPU 推理(nice 降优先级);depth 用对齐到 color 的,保证与 RGB 像素一一对齐。
    semantic_arg = DeclareLaunchArgument(
        "semantic", default_value="false",
        description="启动语义分割节点(SegFormer→semantic_points,供 hppt 语义打分)。"
                    "默认关:SegFormer 在 CPU 上每帧约 2.8s,会拖慢 YOLO/前端/hppt;"
                    "需要时 semantic:=true 开启,迁到 GPU 后可考虑默认开。",
    )
    semantic_node = Node(
        package="semantic_frontier_ros",
        executable="semantic_inference_node",
        name="semantic_inference_node",
        output="screen",
        prefix="nice -n 10",
        parameters=[{
            "checkpoint_path": "/root/ros2_ws/model",
            "image_size": 512,
            "label_space": "ade150",
            "device": "cpu",   # SegFormer 节点无 GPU 探测回退,直接 CPU 避开 sm_120 不兼容
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "semantic_points_topic": "/semantic_frontier/semantic_points",
            "relevance_heatmap_topic": "/semantic_frontier/relevance_heatmap",
            "stride": 4,
            "max_depth_m": 5.0,
            "sync_tolerance_ms": 100.0,
            "publish_heatmap": True,
        }],
        condition=IfCondition(LaunchConfiguration("semantic")),
    )

    # ---- RViz Web Viewer(FastAPI, 端口 8765;控制按钮接入 /exploration_pause)----
    web_viewer_arg = DeclareLaunchArgument(
        "web_viewer", default_value="true",
        description="启动 RViz Web Viewer 前端(FastAPI, 0.0.0.0:8765)。",
    )
    web_viewer_open_arg = DeclareLaunchArgument(
        "web_viewer_open", default_value="true",
        description="Web 服务起来后用 xdg-open 自动打开浏览器(仅 Linux 桌面)。",
    )
    web_source_arg = DeclareLaunchArgument(
        "web_source", default_value="topic",
        description="前端视频源:topic=订阅相机压缩图(不抓屏,不卡,默认);"
                    "x11=抓屏显示 RViz(会和 RViz 抢显示,可能卡)。",
    )
    # 第二步(隔离实时性):nice -n 15 降低采集进程优先级,让 ROS 实时节点优先拿 CPU。
    #   默认 RVIZ_WEB_SOURCE=topic(订阅相机压缩图,不抓屏),CPU 占用已很低;
    #   若切 x11 抓屏(RVIZ_WEB_SOURCE=x11)nice 也能减轻对控制的干扰。
    web_viewer = ExecuteProcess(
        cmd=["nice", "-n", "15", "python3",
             os.path.join(core_share, "rviz_web_viewer", "server.py")],
        output="screen",
        additional_env={
            "RVIZ_WEB_RVIZ_WINDOW_TITLE": "RViz",
            "RVIZ_WEB_SOURCE": LaunchConfiguration("web_source"),
            # 前端视频源 topic:默认指向 YOLO 标注图(带框)。
            "RVIZ_WEB_IMAGE_TOPIC": LaunchConfiguration("web_image_topic"),
            # 固定管理员:用户名 admin,密码 test_admin(scrypt hash,不落明文)。
            "RVIZ_WEB_ADMIN_USER": "admin",
            "RVIZ_WEB_ADMIN_HASH":
                "scrypt$9444d86767da7618dddcc5ca5e721472$"
                "2b6766dbb6f5d93c1229eb628571b24f57dd2ba699a4a35abd8acdb5b03abd1a",
        },
        condition=IfCondition(LaunchConfiguration("web_viewer")),
    )

    # 第二步(隔离实时性):jaka_scan_and_publish 起来后,给它 SCHED_FIFO 实时优先级
    #   + 绑到独立核(0-3),保证轨迹下发的周期不被采集/其它进程抢占 → 机械臂平滑。
    #   ★ 在节点起来后调整(不用启动 prefix),chrt/taskset 失败无害、不影响连臂节点。
    isolate_realtime_arg = DeclareLaunchArgument(
        "isolate_realtime", default_value="true",
        description="给 jaka_scan_and_publish 加实时优先级+绑核(需 CAP_SYS_NICE,容器已 privileged)。",
    )
    isolate_realtime = ExecuteProcess(
        cmd=["bash", "-c",
             "pid=''; for i in $(seq 1 90); do "
             "pid=$(pgrep -f jaka_scan_and_publish | head -1); "
             "[ -n \"$pid\" ] && break; sleep 2; done; "
             "if [ -n \"$pid\" ]; then "
             "chrt -f -p 80 $pid 2>/dev/null "
             "&& echo \"[isolate] jaka_scan_and_publish(pid=$pid) -> SCHED_FIFO 80\" "
             "|| echo \"[isolate] chrt 失败(权限?),跳过实时优先级\"; "
             "taskset -pc 0-3 $pid 2>/dev/null "
             "&& echo \"[isolate] jaka_scan_and_publish -> cores 0-3\"; "
             "else echo \"[isolate] 未找到 jaka_scan_and_publish,跳过\"; fi"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("isolate_realtime")),
    )
    # 延迟打开浏览器,给 uvicorn 起服务留时间。xdg-open 缺失/无桌面时失败无害。
    web_viewer_open = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=["xdg-open", "http://127.0.0.1:8765"],
            output="log",
            condition=IfCondition(LaunchConfiguration("web_viewer_open")),
        )],
        condition=IfCondition(LaunchConfiguration("web_viewer")),
    )

    return LaunchDescription([
        robot_ip_arg,
        scan_delay_arg,
        resolution_arg,
        max_range_arg,
        rviz_arg,
        start_hppt_arg,
        env_pub_rate_arg,
        default_velocity_arg,
        default_accel_arg,
        report_directory_arg,
        web_viewer_arg,
        web_viewer_open_arg,
        web_source_arg,
        yolo_overlay_arg,
        web_image_topic_arg,
        semantic_arg,
        isolate_realtime_arg,
        base_stack,
        move_group,
        octomap_free_region,
        frontier_viz,
        pipeline_after_moveit,
        hppt_after_pipeline,
        reset_on_hppt_exit,
        shutdown_after_reset,
        yolo_overlay,
        semantic_node,
        web_viewer,
        web_viewer_open,
        isolate_realtime,
    ])
