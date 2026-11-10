# robotics-capstone

本仓库将 JAKA A5 机械臂、Intel RealSense D435 RGB-D 相机、OctoMap、
MoveIt 2 和 HPP-PT 视点规划算法整合为一个 ROS 2 自主探索流程。

当前目标环境为 ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2 和 Docker。
仿真闭环已在 Gazebo 中验证；真实 JAKA A5 + 真实 D435 的 initial scan + 自主探索
链路也已跑通 (见 `real_arm_realsense_octomap.launch.py` /
`real_arm_exploration.launch.py`)。

当前分支提供从 RGB-D 建图、frontier 提取、HPP-PT 视点选择到 MoveIt 执行的
完整实验链路，并保留 5-viewpoint、真实 RealSense 和独立算法测试模式。

## new_method 变更（2026-07-25）：可验证 Greedy Baseline

`hppt_boundary_ros` 的探索决策默认从旧 direction-first（固定 standoff 单点 +
未归一化 raw_score 排序 + top-N MoveIt 探针）**替换为可验证的同步 Greedy Baseline**
（本阶段不做 RPT*/F-RPT*）。参数 `use_greedy_baseline`（默认 `true`）分派；
`false` 可切回 legacy 旧路径对照。主路径：

Frontier 聚类 → 每 frontier **多方向×多距离** viewpoint 生成 → **IK+碰撞预过滤**
（每 viewpoint 绑定 joint_state）→ **三项归一化打分**（unknown density + geometric
visibility + semantic prior，权重自动归一化、非法配置启动即报错）→ **per-frontier
Weighted Mean Shift**（选真实候选，非平均 Pose）→ **current→candidate 工作空间 A***
motion cost（3D 绕障路径长度，非 config-space）→ **Greedy utility 选择**
（`λ_score·p − λ_motion·ĉ`，min-max 归一化）→ MoveIt plan-only 验证 → execute；
失败标 **Infinity 边**并在剩余候选重选（不持久拉黑）。

实现要点：
- IK/碰撞走已运行 move_group 的 `/compute_ik`（`ik_link_name=tool0`、
  `avoid_collisions=true`、位姿严格复刻 `mp_team_bridge/conversions.py::viewpoint_quaternion`）
  与可选 `/check_state_validity`；节点新增 `moveit_msgs` 依赖 + `/joint_states` 订阅；
  执行器 3 线程、IK 客户端独立 ReentrantCallbackGroup（回调内阻塞不死锁）。
- 纯算法层 `src/hppt_baseline.hpp`（仅 octomap+Eigen，可独立单测
  `ros2 run hppt_boundary_ros test_baseline`）。
- 新参数（C++ 默认，未写进 launch）：`use_greedy_baseline`、`viewpoint_line_sample_step`、
  `viewpoint_max_distances_per_dir`、`max_ik_calls_per_cycle`、`ik_timeout_sec`、
  `score_w_{unknown,visibility,semantic}`、`score_{h,v}fov_deg`、`ms_*`（Mean Shift）、
  `greedy_lambda_{score,motion}`、`greedy_utility_mode`、`astar_{grid_res,inflation,max_expansions}` 等。
- 详细逻辑见 `docs/hppt_decision_logic.md`；已在 shelf `start_all` 仿真跑通（22 cycle、
  每 cycle gen→IK→meanshift→A*→greedy→execute、机械臂正常移动、0 崩溃/死锁）。

## 这个分支演示了什么

- ✅ Gazebo 中的 JAKA A5 和末端 D435 相机仿真。
- ✅ RGB-D 点云机械臂自滤波和增量 OctoMap 构建。
- ✅ 从 OctoMap free/unknown 边界提取 frontier。
- ✅ 使用 obstacle/unknown 局部密度计算候选视点概率。
- ✅ HPP-PT 图搜索和视点选择。
- ✅ 工作空间、动态黑名单、重复目标和向外观察过滤。
- ✅ 建图、决策和运动规划模块之间的 JSON topic 接口。
- ✅ MoveIt/OMPL 无碰撞规划和 Gazebo 轨迹执行。
- ✅ RViz 中显示机械臂、OctoMap、frontier 红点和目标黄球。
- ✅ Self Filter 后保持完整的机械臂与 OctoMap 碰撞检测，不使用 ACM 绕过。
- ✅ workspace 部署、启动、停止脚本以及 HPP-PT 基线测试。
- ✅ 真实 D435 点云生成 OctoMap，并由 motion planner 在任务开始时写入 PlanningScene。
- ✅ YOLO target detection 旁路观察节点：可从仿真 RGB-D 图像检测目标，
  发布 `/target/found`、`/target/pose`、`/target/marker` 和 `/target/status`。
  每帧全量 detection debug 输出默认关闭，可按需打开 `/target/detections`。
- ✅ **Stage A**: coarse-grid 下采样 + 26-邻接 BFS 聚类 + Gonzalez 最远点采样，
  把几万到几十万个 boundary voxel 汇聚成几十个 cluster，K 个候选空间均匀铺开。
- ✅ **Stage B1**: 视锥 raycast 信息增益 `IG_norm`，从相机实际位姿 `(goal, look_at)`
  出发在 FOV 视锥内均匀采样射线，估计"看一眼能揭开多少 unknown 体素"。
- ✅ **Plan-only cost probe**: 起点→候选边权用 MoveIt `planning_time +
  estimated_execution_time` 代替欧氏距离；失败/超时的边赋 `cost_probe_max_cost`。
- ✅ **decision_bridge 双队列**: `cost_probe_*` 前缀走独立并行队列，view 任务与
  probe 之间互不阻塞。
- ✅ **硬距离截断** `frontier_max_range_factor`: 超过 3× reach_max 的 frontier 直接
  过滤，对固定基座相当于永久拉黑。
- ✅ **永久黑名单** `permanent_block_count`: 12cm 直径簇内累计成功访问 N 次后，
  本次运行永久剔除；默认关闭。
- ✅ **retryable=true 也拉黑**: 防止 env_version drift 等瞬时失败的 goal 陷入
  "选中→失败→不入黑名单→又选中"死循环。
- ✅ **多方向 frontier direction 平均**: `unknown_dir` 用所有 unknown 邻居方向的
  平均值，避免只取第一个轴导致的 look_at 偏斜。
- ✅ **cluster 层 mean_unknown_dir**: BFS 聚类时累加成员方向，让大 cluster 观察
  方向 = 所有成员方向的加权平均。
- ✅ **自适应 standoff**: 按 cluster 尺寸 `extent = sqrt(fine_count) × res` 算出
  刚好把整片 frontier 纳入 FOV 的相机距离，clamp 到 `[standoff_min, standoff_max]`。
- ✅ **look_at 拉远到 standoff + 1m**: 让相机光轴指向 unknown 中央，Stage B1 IG
  raycast 视锥更贴合真实相机。
- ✅ **Line-of-Sight 过滤**: 相机→look_at 视线若被 known-occupied 遮挡则候选淘汰。
- ✅ **Adaptive observation_min_z**: 低 frontier 允许更低相机高度，避免"低平面被
  硬拉高到 0.25m 强制俯视"。
- ✅ `hppt_boundary_ros` perf tests: `test_stage_a_perf` / `test_stage_b_perf`
  独立测算聚类和 IG raycast 的 wall time。

## 架构数据流

```text
Gazebo RGB-D camera
  -> /camera/points
  -> self_filter
  -> /camera/points_filtered
  -> octomap_server
  -> /octomap_binary
       |-> RViz OctoMap display
       |-> octomap_to_json_bridge
           -> /team/optimized_environment_json
           -> environment_bridge
           -> MoveIt PlanningScene

/octomap_binary + TF(world -> tool0)
  -> hppt_boundary_ros
       -> frontier 提取
       -> 工作空间/黑名单/观察方向过滤
       -> 候选点概率计算
       -> HPP-PT 图搜索
       -> /team/viewpoint_task_json
  -> decision_bridge
  -> motion_planner
  -> MoveIt + OMPL
  -> gz_ros2_control
  -> Gazebo 机械臂执行
  -> /team/motion_result_json
  -> hppt_boundary_ros 结果处理和动态黑名单
```

目标检测是旁路观察链路，不参与 HPP-PT 概率、OctoMap 或 MoveIt 规划：

```text
/camera/image_rel + /camera/depth_image + /camera/camera_info
  -> target_detection_node
       -> /target/found
       -> /target/pose
       -> /target/marker
       -> /target/status
       -> /target/detections   # 仅 publish_all_detections=true 时发布
```

真实相机与虚拟机械臂的碰撞规划测试使用另一条数据链：

```text
RealSense D435
  -> /camera/camera/depth/color/points
  -> octomap_server (frame=world)
  -> /octomap_binary
       |-> octomap_to_json_bridge
            -> environment_bridge
            -> motion_planner (single PlanningScene writer)
            -> /apply_planning_scene
            -> MoveIt PlanningScene.world.octomap
```

## 关键文件

```text
robotics-capstone/
├── README.md
├── docker/
│   ├── Dockerfile
│   ├── build.sh
│   ├── entrypoint.sh
│   └── run.sh
└── ros2_ws/
    ├── scripts/
    │   ├── container_deploy.sh
    │   ├── container_start_all.sh
    │   ├── container_start_jaka.sh
    │   └── container_kill_all.sh
    ├── source/                  # cppraplab 源文件
    ├── include/                 # cppraplab 头文件
    └── src/
        ├── hppt_boundary_ros/   # frontier 提取和 HPP-PT 节点
        ├── jaka_a5_description/
        ├── jaka_a5_moveit_config/
        ├── jaka_exploration_core/
        ├── jaka_exploration_description/
        ├── jaka_motion_pipeline/
        ├── jaka_planning_interfaces/
        ├── semantic_frontier_ros/ # SegFormer semantic inference + YOLO target detection
        └── mp_team_bridge/
```

真实相机与 MoveIt 联调相关文件：

```text
ros2_ws/src/jaka_exploration_core/
├── launch/
│   ├── realsense_octomap.launch.py
│   ├── realsense_virtual_arm_octomap.launch.py
│   └── realsense_virtual_arm_moveit_octomap.launch.py
└── jaka_exploration_core/
    ├── octomap_to_json_bridge.py
    └── periodic_octomap_saver.py
```

真机 (真实 JAKA A5 + 真实 D435) 相关文件：

```text
ros2_ws/src/jaka_exploration_core/
├── launch/
│   ├── real_arm_realsense_octomap.launch.py    # 建图 + (可选) initial scan
│   └── real_arm_exploration.launch.py          # 完整自主探索一条命令
└── jaka_exploration_core/
    ├── jaka_state_publisher.py                 # 只读观察者 (login-only)
    └── jaka_scan_and_publish.py                # 独占连接: /joint_states +
                                                #  initial scan +
                                                #  FollowJointTrajectory 执行器
```

Docker 启动脚本将仓库中的 `ros2_ws/` 直接挂载到容器
`/root/ros2_ws`。workspace 脚本直接操作挂载后的代码，不再从其他仓库复制包。

## 前置要求

### 宿主机

- Docker 24 或更高版本。
- 使用 GPU 渲染时需要 NVIDIA Container Toolkit。
- Windows 使用 WSL2 + WSLg；原生 Linux 需要可用的 X11 图形环境。

### 容器

- Ubuntu 24.04。
- ROS 2 Jazzy。
- Gazebo Harmonic。
- MoveIt 2 和 OMPL。
- OctoMap 和 Eigen3。
- 语义/目标检测依赖：`transformers`、`torch`、`torchvision`、`ultralytics`
  等由 Dockerfile 安装；若使用旧镜像测试 target detection，可在容器内临时运行
  `python3 -m pip install --break-system-packages --no-deps ultralytics==8.4.96`。

`ros2_ws/scripts/container_deploy.sh` 会安装 `hppt_boundary_ros` 需要的
ROS、OctoMap 和 Eigen 依赖。

## 一键运行 HPP-PT 自主探索

### 1. 构建并启动容器

在宿主机仓库根目录执行：

```bash
bash docker/build.sh
bash docker/run.sh
```

容器启动后，workspace 位于 `/root/ros2_ws`。

WSL2 出现图形问题时，先在管理员 PowerShell 更新并重启 WSL：

```powershell
wsl --update
wsl --shutdown
```

容器内运行 `glxinfo -B` 时应显示硬件渲染器，不能是 `llvmpipe`。

### 2. 安装依赖并构建

首次进入容器后执行：

```bash
bash /root/ros2_ws/scripts/container_deploy.sh
source /root/ros2_ws/install/setup.bash
```

部署脚本会检查 workspace、安装 apt 依赖并构建全部 ROS 2 package。

如需干净的全量构建：

```bash
cd /root/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

### 3. 启动完整探索流程

推荐使用单终端启动：

```bash
bash /root/ros2_ws/scripts/container_start_all.sh
```

`container_start_all.sh` 默认使用 shelf 场景
`exploration_scene_shelf.sdf`。默认不打开 Gazebo GUI，以降低 WSLg/Remote Desktop
的渲染负载；RViz、仿真时钟、相机、控制器和建图仍正常运行。如确实需要 Gazebo
窗口：

```bash
GAZEBO_GUI=true bash /root/ros2_ws/scripts/container_start_all.sh
```

脚本依次执行：

1. 停止残留 ROS、Gazebo、RViz 和规划进程。
2. 使用 `auto_bootstrap:=false gazebo_gui:=false
   world_file:=exploration_scene_shelf.sdf` 启动 `exploration_stage2.launch.py`。
3. 等待 MoveIt 启动，保持完整 OctoMap 碰撞检测。
4. 在前台运行 `hppt_boundary_ros`。
5. 收到 `Ctrl+C` 后清理后台 launch 和相关节点。

可以直接向 HPP-PT 节点传入 ROS 参数：

```bash
bash /root/ros2_ws/scripts/container_start_all.sh --ros-args \
  -p reach_min:=0.55 \
  -p reach_max:=0.85 \
  -p moveit_allowed_planning_time:=5.0 \
  -p prob_w_obs:=0.8 \
  -p prob_w_unk:=0.2
```

### 4. 停止全部进程

单终端脚本收到 `Ctrl+C` 后会自动清理。如需强制清理：

```bash
bash /root/ros2_ws/scripts/container_kill_all.sh
```

## 其他运行与测试模式

### 两终端调试

终端 A 启动 JAKA、Gazebo、建图和 MoveIt：

```bash
bash /root/ros2_ws/scripts/container_start_jaka.sh
```

终端 B 只启动 TSP/HPP-PT 节点：

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run hppt_boundary_ros hppt_boundary_ros
```

### Stage 2 默认探索启动

默认启动 multi scene、Gazebo GUI、wrist-only initial scan 和 frontier 可视化：

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py
```

`exploration_stage2.launch.py` 自身默认仍是 multi scene；一键脚本
`container_start_all.sh` 默认覆盖为 shelf。手动 launch 时可切换到 shelf 场景：

```bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
  world_file:=exploration_scene_shelf.sdf
```

如果 Gazebo GUI 占用过高，可以使用 server-only 模式，RViz 仍会启动：

```bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
  gazebo_gui:=false
```

### YOLO 目标检测旁路节点

target detection 节点只观察相机图像，不修改 OctoMap、HPP-PT、MoveIt
PlanningScene 或机械臂任务。默认配置文件：

```text
ros2_ws/src/semantic_frontier_ros/config/target_detection.yaml
```

默认输入来自仿真相机：

```yaml
rgb_topic: "/camera/image_rel"
depth_topic: "/camera/depth_image"
camera_info_topic: "/camera/camera_info"
```

默认目标与模型：

```yaml
model_path: "/tmp/yolov8n.pt"
target_class_names: ["bottle"]
confidence_threshold: 0.5
```

`model_path` 放在 `/tmp` 是为了让 Ultralytics 自动下载 `yolov8n.pt` 时不要把权重
写进挂载的 `/root/ros2_ws`，避免污染 git 状态。

启动节点：

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch semantic_frontier_ros target_detection.launch.py
```

观察输出：

```bash
ros2 topic echo /target/status
ros2 topic echo /target/found
ros2 topic echo /target/pose
```

默认 `publish_all_detections=false`，因此不会发布 `/target/detections`，保持原来的
"只关心目标是否找到"逻辑。需要调试 YOLO 每帧看到了什么时，可手动打开：

```bash
ros2 run semantic_frontier_ros target_detection_node --ros-args \
  -p publish_all_detections:=true \
  -p use_tf:=false \
  -p output_frame:=camera
```

打开后会发布 JSON 字符串到：

```text
/target/detections
```

其中 `detections_min_confidence` 只影响 debug 输出，正式 `/target/found` 仍由
`confidence_threshold` 控制。

### 5-viewpoint bootstrap 扫描

不运行 wrist-only initial scan，只测试五点建图和 motion planning：

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
  auto_initial_scan:=false \
  auto_bootstrap:=true
```

shelf 场景下的五点测试：

```bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
  world_file:=exploration_scene_shelf.sdf \
  auto_initial_scan:=false \
  auto_bootstrap:=true
```

成功时终端会输出类似：

```text
Bootstrap scan complete: 5/5 OK. succeeded=[1, 2, 3, 4, 5] failed=[]
```

### 仅启动 JAKA，不自动发送任务

```bash
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
  auto_bootstrap:=false
```

### 真实 RealSense 固定姿态建图

该模式不启动 Gazebo，也不使用模拟 `/joint_states`：

```bash
ros2 launch jaka_exploration_core realsense_octomap.launch.py
```

调整地图分辨率和传感器范围：

```bash
ros2 launch jaka_exploration_core realsense_octomap.launch.py \
  resolution:=0.02 max_range:=1.5
```

定期保存 `.bt` 和 `.ot` 地图：

```bash
ros2 launch jaka_exploration_core realsense_octomap.launch.py \
  save_octomap:=true save_interval:=30.0 \
  save_directory:=/root/ros2_ws/maps
```

只测试数据链路、不启动 RViz：

```bash
ros2 launch jaka_exploration_core realsense_octomap.launch.py rviz:=false
```

如果没有匹配的机械臂 TF，相机或机械臂移动后，累积的 world-frame 地图会失真。

### 真实 D435 绑定虚拟 JAKA 末端

```bash
ros2 launch jaka_exploration_core realsense_virtual_arm_octomap.launch.py
```

该模式只用于 TF 和数据链路测试。真实相机必须复现虚拟机械臂的运动，才能保证
world-frame 地图有效。

### 真实 D435 OctoMap 参与虚拟机械臂碰撞规划

该模式同时启动 mock JAKA、MoveIt、RViz、RealSense 和 OctoMap。相机地图先转换为
带版本的团队环境输入；`motion_planner` 在每个任务开始时应用任务指定的精确版本。
RViz MotionPlanning 面板会绕过该版本化入口，因此此集成模式应通过
`/team/viewpoint_task_json` 发起规划：

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch jaka_exploration_core \
  realsense_virtual_arm_moveit_octomap.launch.py
```

同时启动 HPP-PT 自动生成视点任务：

```bash
ros2 launch jaka_exploration_core \
  realsense_virtual_arm_moveit_octomap.launch.py \
  start_hppt:=true
```

任务开始后成功应用环境时应看到：

```text
Applied OctoMap environment version ...
```

可通过环境状态确认 JSON 链路也已经接收到非空地图：

```bash
ros2 topic echo /team/environment_status_json --once
```

RViz 中显示蓝色 OctoMap Marker 只代表可视化 topic 正常；只有 motion planner 日志出现
`Applied OctoMap environment version`，才能证明地图已经通过 `/apply_planning_scene`
进入 MoveIt。若机械臂起始状态位于 occupied 体素中，下一次规划应报告
`START_STATE_IN_COLLISION`。

### 真实 JAKA A5 + 真实 D435 (真机测试)

本节覆盖将真实 D435 挂载在真实 JAKA A5 末端上、跑完整"建图 → initial scan → 自主
探索"链路的用法。三个真机 launch 分层递进，可按需要选一个跑。

#### 前置条件

1. **网络**：宿主机可以 ping 通 JAKA 控制器 IP，默认 `10.5.5.100`。
2. **JAKA SDK**：`docker/run.sh` 会自动挂载 SDK 到容器 `/opt/jaka_sdk`；默认从
   `<repo_root>/../JAKA-A5+UR5e/JAKA_A5_Docs/JAKA SDK v2.2.2` 找，用
   `JAKA_SDK_HOST=/path/to/sdk` 环境变量可覆盖。挂载成功容器启动会打印
   `[run.sh] Mounting JAKA SDK from: ...`。
3. **USB 直通**：D435 必须在**容器启动前**插好宿主机 USB。如果启动后再插，需要
   `docker rm -f jaka_exploration_ros2` 重启容器让新设备进 USB 命名空间。
4. **相机安装约定** (见 `jaka_a5_with_d435.urdf.xacro:231-235`): 相机以 `rpy="0 -π/2 0"`
   挂在 tool0，含义是 D435 顶部 (logo 面) 对齐 tool0 的 `-X` 方向。initial scan
   起点 `[0, π/2, -π/2, π/2, π/2, 0]` 时 tool0 朝下，此时相机拍到的 RGB 图中：
   - `base` 应出现在**图像左边**
   - 机械臂前方 (world `+X`) 出现在**图像右边**

   物理装偏了 90° / 180° 也可以在 launch 里加 static TF 覆盖 URDF 的
   `tool0 -> camera_link` 旋转来纠正，不必重新拧螺丝。

#### 单进程执行器 (`jaka_scan_and_publish`)

JAKA 控制器同一时刻**只允许一个 jkrc 客户端**连接。为避免"发 `/joint_states` 的观察
者"和"发运动命令的脚本"抢连接，所有真机 launch 都用一个统一节点
`jaka_scan_and_publish`：

- 独占 jkrc 连接 (login + power_on + enable)
- 后台 50 Hz 线程读关节角发 `/joint_states`
- 主线程按 `run_mode` 决定行为：
  - `scan_only`         : 跑 6 方向 initial scan 后 disable + logout 退出
  - `scan_then_explore` : scan 完成后不 disable，把 FollowJointTrajectory
                          action server 挂在 `/jaka_a5_controller/follow_joint_trajectory`
                          等 MoveIt 送轨迹
  - `explore_only`      : 跳过 scan，直接进 action server 模式
- 所有运动使用 `jkrc.joint_move(is_block=False)` + Python 侧轮询到位。**必须非阻塞**：
  阻塞调用会让 jkrc 的 C 扩展持有 Python GIL 几十秒，饿死 `/joint_states` 发布线程，
  导致 TF 停更、`octomap_server` 丢掉所有点云 (`Lookup would require extrapolation
  into the past`)。
- 速度硬编码 `SLOW_SPEED_DPS = 5.0` deg/s，与
  `real_robot_test_basic/initial_scan.py` 完全一致。

#### 层 1: 真机 initial scan + OctoMap 建图

只跑 6 方向 initial scan (不做后续 exploration)：

```bash
# 一条命令包含相机驱动 + robot_state_publisher + self_filter + octomap_server + RViz
# + 15s 后自动启动 jaka_scan_and_publish (scan_only 模式)
ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py \
  auto_scan:=true
```

不启动 scan 节点，只跑感知栈 (给示教器手动摆姿势时验证 TF / OctoMap)：

```bash
ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py
```

只加一个只读观察者 (login-only，不 enable 机械臂)，另开终端手动跑运动脚本：

```bash
ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py \
  start_jaka_reader:=true
# 另一个终端：
python3 /home/raplab/project/450_g12/real_robot_test_basic/initial_scan.py
```

关键参数：

- `robot_ip` (默认 `10.5.5.100`)
- `resolution` (默认 `0.03`)
- `max_range` (默认 `2.0`)
- `scan_delay` (默认 `15.0`) — auto_scan 模式下等基础栈就绪的秒数
- `scan_run_mode` (默认 `scan_only`) — 供 exploration launch 覆盖

self_filter 用 `jaka_exploration_core/self_filter.py` 中 base_link + J1..J6 的球形包
络剔除机械臂本体点云 (camera_link 故意不加，它就是传感器本体)。参数已按仿真调好，
真臂完全复用。

#### 层 2: 真机全自主探索 (initial scan → HPP-PT loop)

在层 1 基础上加 MoveIt + motion_planner + environment_bridge + decision_bridge +
octomap_to_json_bridge + HPP-PT，形成完整探索闭环：

```bash
ros2 launch jaka_exploration_core real_arm_exploration.launch.py
```

启动时间线：

| t (s) | 事件 |
|-------|------|
| 0     | robot_state_publisher / RealSense / self_filter / octomap_server / RViz 起来 |
| 8     | MoveIt、motion planner、bridges、known-free validator、metrics reporter 和 OctoMap bridge 起来 |
| 15    | `jaka_scan_and_publish` 起来 (`run_mode=scan_then_explore`)，login → power_on → enable |
| 20    | `hppt_boundary_node` 起来，但在 scan-complete gate 打开前不会收到 accepted environment |
| ~25   | scan 节点内部 5 秒倒数结束，开始 6 方向 initial scan |
| scan_time 后 | scan 完成，发布 `/jaka_initial_scan_complete=true`，环境 bridge 开始发布，随后允许探索任务 |
| 之后   | HPP-PT 循环: 选 viewpoint → decision_bridge → motion_planner → MoveIt 规划 → 我们的 action server 慢速执行 → HPP-PT 处理结果继续选下一个 |

真机执行链的关键 shim: **MoveIt 的 `moveit_simple_controller_manager` 通过
FollowJointTrajectory action 与"控制器"通信**，`jaka_a5_moveit_config` 已经配好指
向 `/jaka_a5_controller/follow_joint_trajectory`。真机没有 ros2_control 硬件接口，
`jaka_scan_and_publish` 直接实现这个 action server，把收到的轨迹按 `SLOW_SPEED_DPS`
逐 waypoint 慢速跟踪。MoveIt 的 `time_from_start` 被有意忽略——我们比 MoveIt 预算慢
一到两个数量级，所以 launch 里也关掉了 `execution_duration_monitoring`。

关键参数：

- `default_velocity_scaling` (默认 `0.05`) / `default_acceleration_scaling` (默认
  `0.05`) — MoveIt 规划出**形状**平滑的轨迹；实际速度仍由执行器锁在 5 deg/s
- `environment_rate` (默认 `1.0`) — OctoMap → JSON 环境快照频率
- `start_hppt` (默认 `true`) — 关掉可只留 plumbing 手动 `PlanMotion`
- `motion_report_directory` — 每分钟 planning 报告目录，默认
  `/root/ros2_ws/runtime_reports/motion_planning`

正式探索任务在实机上同样执行两层安全检查：RRTConnect 使用 unknown-as-occupied 的
conservative OctoMap 搜索，生成轨迹后再由 `known_free_validator` 逐段检查。validator
读取实机 `/joint_states` 和带 D435 的 URDF/SRDF，当前机械臂 collision geometry 会从
保守地图中 carve，真实 observed obstacle 不会被清除。该流程已完成代码和 launch
接入，但在连接真实机械臂前必须先以 `start_hppt:=false` 验证节点、服务和环境版本。

#### 数据链路 (真机 exploration)

```text
真 D435 (/camera/camera/depth/color/points)
  -> self_filter (剔除机械臂本体)
  -> /camera/points_filtered
  -> octomap_server (frame=world)
  -> /octomap_binary
       |-> octomap_to_json_bridge -> /team/optimized_environment_json
       |    -> environment_bridge -> /known_environment
       |    -> motion_planner -> /apply_planning_scene -> MoveIt PlanningScene
       |-> hppt_boundary_node
            -> /team/viewpoint_task_json
            -> decision_bridge
            -> motion_planner (PlanMotion action)
            -> MoveIt move_group (OMPL)
            -> FollowJointTrajectory
              @ /jaka_a5_controller/follow_joint_trajectory
            -> jaka_scan_and_publish -> jkrc.joint_move (5 deg/s)
            -> /team/motion_result_json
            -> hppt_boundary_node 更新黑名单，选下一个 viewpoint

TF 链:
  world -> base_link (static from launch)
        -> J1..J6 (from robot_state_publisher, driven by jaka_scan_and_publish
                   -> /joint_states @ 50 Hz)
        -> tool0
        -> camera_link (URDF: rpy 0 -π/2 0)
        -> camera_depth_optical_frame (发布自 realsense2_camera)
```

#### 常见问题

- **`login -> -3`**：JAKA 控制器不在同网段，或急停按下。先在容器里 `ping 10.5.5.100`。
- **`No RealSense devices were found`**：D435 掉了。销毁容器 → 拔插 USB → 重启容器。
- **RViz 里机械臂不跟真臂动**：几乎肯定是双客户端冲突。确认没有其他脚本
  (initial_scan.py、realsense-viewer 等) 也在连接 JAKA。
- **`Lookup would require extrapolation into the past/future`** (启动早期几秒或
  scan 节点结束后)：TF 追不上时间戳。启动早期正常；scan 节点关闭后 `/joint_states`
  停发，TF 会 stale，此时 launch 里的建图节点会开始 drop 点云 — 是设计如此，不是 bug。
- **MoveIt 一直 "waiting for controller"**：说明 action server 名字对不上。
  `ros2 action list` 应包含 `/jaka_a5_controller/follow_joint_trajectory`，
  没有的话看 `jaka_scan_and_publish` 日志里 `FollowJointTrajectory action server
  ready at ...` 有没有出现。

#### 与仿真的关系

**新增的所有代码都是"叠加"式的**，没有替换或修改任何仿真代码：`exploration_stage2.launch.py`
以及所有 `realsense_*.launch.py` 完全不受影响，可以原样跑。真机 launch 复用了仿真
里的 `motion_planner` / `environment_bridge` / `decision_bridge` /
`octomap_to_json_bridge` / `hppt_boundary_ros` / `self_filter` / URDF / SRDF /
MoveIt config，只在末端把执行层从 `gz_ros2_control` 换成了 `jaka_scan_and_publish`
的 FollowJointTrajectory shim。

### 查看相机图像

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_rel
```

应订阅使用 reliable QoS 转发的 `/camera/image_rel`。

## HPP-PT 处理流程

### 输入

- `/octomap_binary`：标准 `octomap_msgs/msg/Octomap` 环境。
- TF `world -> tool0`：当前末端位置。
- `/team/environment_status_json`：最新被接受的环境版本。
- `/team/motion_result_json`：motion planning 任务结果。

### Frontier 和候选点生成

节点遍历 free OcTree 叶节点。当一个 free 节点的六个轴向邻居中至少有一个未知节点
时，标记为 frontier。**所有** unknown 邻居方向的平均值作为该体素的
`unknown_dir`（P0-1：不再只取第一个方向；一个 frontier 若同时贴 2-3 个 unknown
面，look_at 会朝真正的"unknown 集合中心"看）。

原始 boundary voxel 数量通常在几万到几十万。选点流水线：

1. **Per-voxel 七重过滤**：`isTooFar` (`frontier_max_range_factor × reach_max` 外
   的硬距离截断) → `isForbidden` (贴基座 / z 越界) →
   `isUnreachable` (超出 `reach_max`；若 `observe_unreachable=true` 只记数不淘汰) →
   `isBlacklisted` (60s 失败半径) → `isPermanentlyBlocked` (永久黑名单，默认关) →
   `isVisited` (45s 成功半径) → `isInwardLook` (相机朝向基座)。
2. **Stage A: 聚类 + 采样**
   1. **粗网格下采样**：按 `octomap_res × coarse_grid_factor` (默认 6cm) 聚合，
      同 key 的细体素合并成一个 super voxel，累加成员方向。
   2. **26-邻接 BFS 连通分量**：把相邻 super voxel 聚成 cluster；用成员方向的
      fine_count 加权平均得到 `mean_unknown_dir`。
   3. **小 cluster 过滤**：super voxel 数低于 `cluster_min_voxels` 视作噪点丢弃。
   4. **centroid 层五重过滤**：对每个 cluster centroid，用 `mean_unknown_dir` 和
      自适应 standoff 生成实际会发布的 `(goal, look_at)`，再检查
      `isBlacklisted / isPermanentlyBlocked / isVisited / isInwardLook /
      isLineOfSightBlocked`。第五项从 `goal` 出发沿 `(look_at - goal)` 做
      `castRay(ignoreUnknown=true)`：若在到达 `look_at` 之前就撞到 known-occupied
      → 视线被墙挡 → 淘汰。
   5. **Gonzalez 最远点采样 (FPS)**：从当前末端位置起，反复选距已选集合最远的
      cluster centroid，直到取够 `boundary_samples` 个。若 cluster 数少于 K，全取。
3. **构造 BoundaryPoint**：`unknown_neighbor` 用 cluster `mean_unknown_dir` 编码，
   `standoff_override` 存自适应 standoff `clamp(extent / (2·tan(fov/2)),
   standoff_min, standoff_max)`，其中 `extent = sqrt(fine_count) × octomap_res`。

启用 `observe_unreachable` 时，`(reach_max, frontier_max_range_factor × reach_max]`
区间内的 frontier 会被投影到 `projection_scale × reach_max` 球面上做 SURROGATE
观察，`look_at` 仍指向原始 frontier 方向。

### 相机位姿细节 (`resolveViewpoint`)

给定 frontier `coord` 与 `unknown_neighbor`：

```text
unknown_dir = normalize(unknown_neighbor - coord)          # P0-1 已是平均方向
standoff    = clamp(standoff_override, standoff_min, standoff_max)  # P1-1
goal        = coord - unknown_dir × standoff
look_at     = coord + unknown_dir × (standoff + 1.0m)       # P1-2 拉远光轴

effective_min_z = observation_min_z                         # P3-3 低 frontier 松绑
if coord.z < observation_min_z:
    effective_min_z = max(observation_min_z_low,
                          min(observation_min_z, coord.z - 0.05))
goal.z = clamp(goal.z, effective_min_z, z_max)

if isUnreachable(goal):
    goal = projectToReachable(goal)                         # SURROGATE
    goal.z 再夹一次
```

### 概率模型

每个候选点的概率由四项加权和构成：

```text
probability = prob_w_obs * occupied_density
            + prob_w_unk * unknown_density
            + prob_w_sem * semantic_relevance
            + prob_w_ig  * IG_norm
```

前三项在候选点周围 `prob_density_radius` 球邻域内统计 occupied 和 unknown
体素占比，`semantic_relevance` 来自 `/semantic_frontier/semantic_points`：节点把
每个候选点附近 `semantic_radius` 内的语义点 `relevance_prob` 聚合成一个 0..1
的语义分数。没有语义点或语义点过期时该项为 0。

**Stage B1 `IG_norm`**：从 `resolveViewpoint` 得到的 `(goal, look_at)` 出发，
按分层 cos θ + Fibonacci 均匀在 `ig_fov_deg` 视锥内采样 `ig_num_rays` 条射线。
每条射线用 `tree.castRay(ignoreUnknown=true, max_range=ig_max_range_m)` 找第一
个 occupied 或走满 `ig_max_range_m`；沿射线以 `octomap_res` 步长采样，统计沿途
`tree.search()==nullptr` 的 unknown 体素数。归一化：

```text
IG_norm = unknown_total / (num_rays × max_steps_per_ray)   # ∈ [0, 1]
```

1 = 每条射线都走满 `ig_max_range_m` 且沿途全是 unknown。相机被墙挡住或视锥内
全是 known-free 时 IG_norm ≈ 0。四项加权后 clamp 到 `[0, 0.999]`（HPP-PT 要求
概率严格 < 1，否则期望代价立刻收敛）。

### 图搜索和任务发布

当前末端位置与 `K` 个候选点组成完全图，边权来源：

- **起点 → 每个候选** (Stage B core)：若 `use_planning_cost=true`（默认），
  节点为每个候选发一个 `cost_probe_<n>_<i>` 前缀的 plan_only 任务给
  decision_bridge，阻塞等待最多 `cost_probe_timeout_sec` 秒。probe 结果里的
  `planning_time + estimated_execution_time` 就是该边的权重。plan_only 失败或
  超时的边赋 `cost_probe_max_cost` (1e6)，等价于禁用；`use_planning_cost=false`
  时回落 Euclidean 距离。
- **候选 ↔ 候选**：一律 `cost_probe_max_cost`，等价于"从起点直选一个候选"。
  HPP-PT 图形式上完整，语义上退化为期望代价 argmin。

`raplab::RunAstarHPPPT` 结合边权和候选概率求最优路径。节点从 `path[1]` 开始顺序
检查，发布第一个仍然合法的目标到 `/team/viewpoint_task_json`。

**失败 goal 处理**：`success=false` 时**不管 `retryable`** 都加入短期黑名单
`blacklist_duration_sec`。这是必要的，否则 `env_version drift` 等 transient 失败
的 goal 会形成"选中→失败→不入黑名单→octomap 未变→又选中"的死循环。

**cost_probe queue 独立**：`decision_bridge` 用两个队列——`cost_probe_*` 前缀走
probe 队列（`max_parallel_probes` 控制并发度，当前 `1`——因为 `motion_planner_node`
用 `_busy=True` 会拒并发 goal；将来若允许 plan_only 并发可以调高），其他任务走
串行 view 队列。这样一批 probe 不会阻塞后续 view 任务，反之亦然。

任务中包含唯一 `task_id`、最新 `environment_version`、`world` 坐标系、相机位置和
`look_at` 点。`decision_bridge` 将 viewpoint 转换为末端位姿，再交给 MoveIt action。

## ROS 接口

| Topic | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/camera/points` | `sensor_msgs/PointCloud2` | Gazebo camera | self filter |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | RealSense D435 | OctoMap server |
| `/camera/points_filtered` | `sensor_msgs/PointCloud2` | self filter | OctoMap server |
| `/octomap_binary` | `octomap_msgs/Octomap` | OctoMap server | HPP-PT、JSON bridge |
| `/team/optimized_environment_json` | `std_msgs/String` | OctoMap JSON bridge | environment bridge |
| `/team/environment_status_json` | `std_msgs/String` | environment bridge | HPP-PT |
| `/team/viewpoint_task_json` | `std_msgs/String` | HPP-PT | decision bridge |
| `/team/motion_result_json` | `std_msgs/String` | decision bridge | HPP-PT |
| `/hppt_boundary/markers` | `visualization_msgs/MarkerArray` | HPP-PT | RViz |
| `/target/found` | `std_msgs/Bool` | target detection | 任务停止/监控节点 |
| `/target/pose` | `geometry_msgs/PoseStamped` | target detection | RViz/任务停止/监控节点 |
| `/target/marker` | `visualization_msgs/Marker` | target detection | RViz |
| `/target/status` | `std_msgs/String` | target detection | 调试/日志 |
| `/target/detections` | `std_msgs/String` | target detection | 调试；仅 `publish_all_detections=true` |

任务中的 `environment_version` 必须与最新被接受的环境版本完全一致。HPP-PT 节点会
订阅状态 topic 动态获取版本号，不应硬编码版本。环境版本必须严格递增且不可复用；
任务在队列等待期间发生版本变化时会返回可重试的
`environment_version_mismatch`，调用方应基于新版本重新生成任务。

`/team/motion_result_json` 包含 `retryable`、`error_type`、
`environment_applied`、`applied_environment_version`、`planning_time` 和
`estimated_execution_time`。`environment_applied=true` 时，
`applied_environment_version` 用于确认 MoveIt 实际应用的环境快照；
`planning_time` 是 MoveIt 实际规划耗时；
`estimated_execution_time` 取规划轨迹最后一个点的 `time_from_start`，表示按当前
速度和加速度限制执行该轨迹的预计运动时间。
轨迹为空时预计执行时间为 `0.0`。

MoveIt 默认使用 OMPL 的 RRTConnect (`RRTConnectkConfigDefault`)。RRT* 仍保留在
配置中，供后续算法对比。

## 关键参数

### HPP-PT 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `world_frame` | `world` | 共享世界坐标系 |
| `robot_frame` | `tool0` | 当前机械臂位置对应的 TF frame |
| `octomap_topic` | `/octomap_binary` | OctoMap 输入 topic |
| `env_status_topic` | `/team/environment_status_json` | 环境版本来源 |
| `task_topic` | `/team/viewpoint_task_json` | 视点任务输出 |
| `feedback_topic` | `/team/motion_feedback_json` | motion feedback 输入，用于确认任务被下游接收 |
| `result_topic` | `/team/motion_result_json` | motion result 输入 |
| `boundary_samples` | `5` | HPP-PT 候选采样数量 |
| `planning_time_limit` | `60.0` | HPP-PT 搜索时限，单位秒 |
| `moveit_allowed_planning_time` | `5.0` | 单个 MoveIt 目标的规划时限，单位秒 |
| `moveit_num_planning_attempts` | `10` | 单个 MoveIt 目标的规划尝试次数 |
| `moveit_position_tolerance` | `0.03` | HPPT viewpoint 任务的位置容差，单位米 |
| `moveit_orientation_tolerance` | `0.35` | HPPT viewpoint 任务的姿态容差，单位弧度 |
| `task_timeout_sec` | `300.0` | 等待下游任务结果的最长时间，应大于 decision bridge 超时 |
| `task_dispatch_timeout_sec` | `10.0` | 发布任务后等待下游 feedback 的最长时间 |
| `selection_interval_sec` | `1.0` | 两次 HPP-PT 选点计算之间的最小间隔 |
| `duplicate_goal_cooldown_sec` | `3.0` | 选到重复目标后暂停重新选点的时间 |
| `goal_dedup_tol` | `0.02` | 重复目标距离，单位米 |
| `blacklist_radius` | `0.20` | 失败目标排除半径，单位米 |
| `blacklist_duration_sec` | `60.0` | 失败目标黑名单持续时间 |
| `visited_radius` | `0.12` | 已完成目标短期排除半径，单位米 |
| `visited_duration_sec` | `45.0` | 已完成目标短期排除时间 |
| `base_x`, `base_y`, `base_z` | `0.0` | world 中的机械臂底座位置 |
| `reach_min` | `0.60` | 候选点到基座的最小距离 |
| `reach_max` | `0.85` | 候选点到基座的最大距离 |
| `z_min` | `0.05` | 候选点最低高度 |
| `z_max` | `1.20` | 候选点最高高度 |
| `frontier_max_range_factor` | `1.5` | ★ 硬距离截断：`dist(frontier, base) > factor × reach_max` 时直接淘汰（固定基座 = 永久拉黑）。`1.5 × 0.85 ≈ 1.28m` |
| `observation_standoff` | `0.35` | 观察位姿相对 frontier 向 free 侧后退的距离（未启用自适应时使用）|
| `observation_standoff_min` | `0.15` | ★ P1-1 自适应 standoff 下限 |
| `observation_standoff_max` | `0.55` | ★ P1-1 自适应 standoff 上限；实际 standoff = clamp(cluster_extent / (2·tan(fov/2)), min, max) |
| `observation_min_z` | `0.25` | 生成观察位姿的最低高度 |
| `observation_min_z_low` | `0.10` | ★ P3-3 低 frontier 时相机高度下限（frontier.z < 0.25 时线性放宽到该值）|
| `enable_los_check` | `true` | ★ P3-1 视线遮挡检查：goal → look_at 若被 known-occupied 挡则候选淘汰 |
| `require_outward_look` | `false` | 是否拒绝朝向基座的视点(2026-07-16 默认改 false,原 true 会误杀绕物观测,详见"死循环 + 背面盲区修复"章节) |
| `observe_unreachable` | `true` | 是否投影不可达候选点 |
| `projection_scale` | `0.82` | 不可达点投影比例 |
| `permanent_block_radius` | `0.06` | ★ 永久黑名单簇半径（12cm 直径），单位米 |
| `permanent_block_count` | `0` | ★ 永久黑名单成功访问次数阈值；0 = 关闭 |
| **Stage A** (聚类) | | |
| `enable_frontier_clustering` | `true` | 关掉回退到旧 index-uniform 采样 |
| `coarse_grid_factor` | `3` | ★ 粗网格因子；super voxel 边长 = `octomap_res × factor` |
| `cluster_min_voxels` | `5` | ★ super voxel 数低于该值的 cluster 视作噪点丢弃 |
| **概率打分权重** | | |
| `prob_density_radius` | `0.15` | 密度统计半径，单位米 |
| `prob_w_obs` | `0.7` | occupied 密度权重 |
| `prob_w_unk` | `0.2` | ★ unknown 密度权重（原 0.3）|
| `semantic_topic` | `/semantic_frontier/semantic_points` | 语义点云输入 |
| `use_semantic_probability` | `true` | 是否把语义 relevance 加入 HPP-PT 候选概率 |
| `semantic_radius` | `0.20` | 候选点周围语义点聚合半径，单位米 |
| `semantic_max_age_sec` | `3.0` | 语义点云最大有效时间 |
| `prob_w_sem` | `0.5` | semantic relevance 权重 |
| `semantic_min_relevance` | `0.0` | 忽略低于该 relevance 的语义点 |
| **Stage B1** (视锥 IG) | | |
| `prob_w_ig` | `0.5` | ★ 视锥 raycast IG 权重 |
| `ig_fov_deg` | `70.0` | ★ 相机水平视场，单位度 |
| `ig_max_range_m` | `2.0` | ★ IG raycast 最大距离，单位米 |
| `ig_num_rays` | `64` | ★ 每候选采样射线数 |
| **Cost probe** (Stage B core) | | |
| `use_planning_cost` | `true` | ★ 起点→候选边权用 plan_only 代价代替欧氏距离 |
| `cost_probe_timeout_sec` | `30.0` | ★ 阻塞等待所有 probe 完成的超时 |
| `cost_probe_max_cost` | `1e6` | ★ probe 失败/超时时该边的代价 |

### `decision_bridge` 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `probe_task_prefix` | `cost_probe_` | ★ 前缀匹配的任务走独立并行队列 |
| `max_parallel_probes` | `1` | ★ probe 并行度上限；`motion_planner_node._busy` 拒并发时保持 1 |

## RViz 可视化

在 RViz 中将 Fixed Frame 设置为 `world`，然后添加
`/hppt_boundary/markers` 的 `MarkerArray` display：

- 红色半透明点：全部 frontier 体素。
- 黄色球：当前选择的目标。
- OctoMap display：已累积的占据体素。

self-filter 日志应显示正数的 dropped points。如果始终为零，应先检查相机和机械臂
TF，再将 OctoMap 用于碰撞规划。

## 开发与测试

只修改 HPP-PT C++ 代码时，可以单独构建该 package：

```bash
cd /root/ros2_ws
colcon build --packages-select hppt_boundary_ros
source install/setup.bash
```

修改 URDF、SRDF、self-filter 或 launch 后，需要完整重启仿真。启动脚本会终止旧
`octomap_server`，防止历史自身点云残留在新测试中。

运行 HPP-PT 星形图基线测试：

```bash
ros2 run hppt_boundary_ros test_hppt_star
```

常用运行状态检查：

```bash
ros2 node list
ros2 topic hz /octomap_binary
ros2 topic echo /team/environment_status_json --once
ros2 topic echo /team/motion_result_json --once
ros2 run tf2_ros tf2_echo world tool0
```

## 关键设计说明

### 使用标准 OctoMap 输入

原 RAP-lab 实现使用带有 probability 字段的自定义 `MyOcTree`。当前集成直接读取
标准 `octomap::OcTree`，并在 HPP-PT 节点中计算候选概率，避免自定义地图序列化
依赖。

### 单一 PlanningScene writer

`octomap_to_json_bridge` 只发布带版本的完整环境快照，不直接修改 MoveIt。
`motion_planner` 是集成流程中唯一调用 `/apply_planning_scene` 的节点，并在任务开始时
应用任务指定版本。规划期间不会有第二个 bridge 替换场景。

空 OctoMap 也会携带非空 `id` 写入 PlanningScene。MoveIt 会先删除旧 OctoMap，再因
payload 为空而不创建新地图，从而实现明确清场。

Self Filter 负责在建图前移除机械臂自身点云。启动流程不再修改 `<octomap>` 的 ACM，
`base_link/J1/J2/J3` 与真实障碍物之间的碰撞保持启用。

### Unknown space 安全检查

正式 motion task 和 plan-only cost probe 在规划前先生成 conservative OctoMap：规划边界内
observed-free 保持 free，unknown 和 observed-occupied 都作为 occupied；当前机械臂实际
占据的 URDF collision geometry 会根据 `/joint_states` carve（默认 10 cm padding，另加半个体素对角线），避免初始状态与 unknown 自碰撞。RRTConnect 因此
会在搜索过程中主动绕开 unknown，而不是规划后才发现路径无效。

第一次正式规划还会固定记录扫描结束后的初始机械臂骨架，并沿各 link 中心线建立
默认半径 25 cm 的胶囊形 seed-free 包络。该包络只把其中尚未观测的 unknown 体素
设为 free，不覆盖任何 observed-occupied 障碍；它也不会随当前姿态移动，因此不能
通过连续任务逐步向 unknown 空间扩张。半径由
`conservative_initial_seed_radius` 配置。

MoveIt 产生路径后，`known_free_validator` 还会对关节轨迹插值，并沿机械臂 link 中心线和默认 6 cm 安全半径采样。
每个采样点必须在任务指定版本的 OctoMap 中存在且为 free；unknown 或 occupied 都会
以 `trajectory_not_known_free` 拒绝，轨迹不会发送到 `/execute_trajectory`。
validator 的 conservative map 在单个任务期间固定为该任务版本；实时 OctoMap 即使
继续发布新版本，也不会覆盖正在规划和验证的场景。下一任务会在开始时显式构建并
切换到其指定版本，避免“版本 A 规划、版本 B 验证”的竞态。

`initial_scan_*` 是唯一默认例外。系统启动时尚无 known-free 体积，严格检查会形成
“没有地图就不能移动、不能移动就无法建图”的死锁，因此初始 6 向原地扫描会明确
bypass validator。完成 bootstrap 后，所有 TSP 任务恢复严格检查。例外前缀配置在
`jaka_motion_pipeline/config/pipeline.yaml`，不应加入普通任务前缀。

### 每分钟 motion planning 报告

`motion_metrics_reporter` 每 60 秒写一份 JSON 和 Markdown 报告：

```text
/root/ros2_ws/runtime_reports/motion_planning/report_<UTC timestamp>.json
/root/ros2_ws/runtime_reports/motion_planning/report_<UTC timestamp>.md
/root/ros2_ws/runtime_reports/motion_planning/latest.json
/root/ros2_ws/runtime_reports/motion_planning/latest.md
```

报告分别统计正式任务和 `cost_probe_*`，包含累计及最近一分钟的成功率、planning time、
预计 execution time、均值、中位数、P95、失败类型、MoveIt error code 和环境版本；
`initial_scan_*` 单独统计，不计入正式任务成功率。
该目录已加入 `.gitignore`。实时查看：

```bash
cat /root/ros2_ws/runtime_reports/motion_planning/latest.md
```

`estimated_execution_time` 来自轨迹最后一点的 `time_from_start`，不是墙钟实测执行时间。

### 向外观察约束

观察方向需要满足：

```text
(look_at - goal) dot (goal - base) > 0
```

该约束减少相机朝向基座、机械臂手腕向内折叠的危险构型。

### 路径顺延

候选点可能在搜索完成后、任务发布前进入黑名单。节点会继续检查路径中的后续节点，
而不是立即重新运行 HPP-PT；整条路径都无效时才终止。

## 排错速查

### `Package 'hppt_boundary_ros' not found`

构建并 source 当前挂载的 workspace：

```bash
bash /root/ros2_ws/scripts/container_deploy.sh
source /root/ros2_ws/install/setup.bash
```

### 规划在几十毫秒内因 start-state collision 失败

先检查 `/camera/points_filtered` 和 Self Filter 日志，确认机械臂自身点云没有进入
OctoMap。不要通过 ACM 忽略 `base_link/J1/J2/J3`，否则真实障碍也会被忽略。

### `/move_action` 不可用

MoveIt 尚未启动完成，可以增加启动等待时间：

```bash
MOVE_GROUP_WAIT_SEC=90 bash /root/ros2_ws/scripts/container_start_all.sh
```

### 所有 boundary points 都进入黑名单

适当放宽工作空间或观察方向过滤：

```bash
ros2 run hppt_boundary_ros hppt_boundary_ros --ros-args \
  -p reach_min:=0.45 -p require_outward_look:=false
```

### `TF lookup world -> tool0 failed`

检查 `robot_state_publisher`、controller 和 TF：

```bash
ros2 node list | grep robot_state_publisher
ros2 run tf2_ros tf2_echo world tool0
```

### self-filter 没有过滤机械臂点云

```bash
ros2 node list | grep self_filter
grep -E "self_filter|dropped" /tmp/launch.log
```

如果第一帧点云到达时 TF 尚未就绪，应停止全部进程并重新启动完整流程。

### RViz 看不到 marker 或 OctoMap

- 将 Fixed Frame 设置为 `world`。
- 添加 `/hppt_boundary/markers` 的 `MarkerArray` display。
- 检查 OctoMap 和 Image display 的 QoS 是否与发布者一致。
- WSL 修改 WSLg/GPU 挂载后，需要重新创建容器。

### RViz 有蓝色体素，但 MoveIt 仍穿过 occupied 区域

蓝色体素来自 `/occupied_cells_vis_array`，只代表 RViz Marker 可见。发布一个引用
当前环境版本的任务，并检查 motion planner 是否成功调用 PlanningScene 服务：

```bash
ros2 topic hz /octomap_binary
ros2 service list | grep apply_planning_scene
```

启动日志必须出现：

```text
Applied OctoMap environment version ...
```

测试时不要让 Start State 和 Goal State 同时保持 `<current>`，否则请求可能只是
零运动轨迹。设置不同目标后，如果起始状态位于 occupied 体素中，应看到
`START_STATE_IN_COLLISION`。

## 验证运行结果

- OctoMap -> HPP-PT -> JSON task -> MoveIt -> Gazebo 数据链路。
- `environment_version` 精确匹配。
- Self Filter 生效且所有机械臂 link 仍参与 OctoMap 碰撞检测。
- PlanningScene 只有 motion planner 一个 writer。
- 超时任务会取消 MoveGroup goal，并向队列返回可重试错误。
- 空 OctoMap 会清除 MoveIt 中已有的 OctoMap。
- 失败目标进入临时动态黑名单。
- 工作空间和向外观察过滤能够剔除不安全候选点。
- RViz 能够显示 frontier 和当前目标。
- HPP-PT 基线测试在多种代价/概率配置下行为符合预期。
- 真实 D435 的非空 OctoMap 能通过版本化任务写入 MoveIt PlanningScene。
- MoveIt 能够将真实相机生成的 occupied 体素用于起始状态和路径碰撞检查。

## 视点选择改进 (2026-07-15)

针对 shelf 世界 3 分钟探索测试暴露的问题——所有 goal 都落在 -X 半球不朝
柜子、HPP-PT 有时把柜子候选排到 path 末位、MoveIt 秒拒率高——对候选生成、
评分、选择三层做了系统性重构。3-run mean 覆盖率从 +2%(baseline) 提升到
+4.17% (v9),100% 触发 IK 秒拒的死循环消失。

### 改动一览

**Frontier / cluster 过滤**
- LOS check 用 frontier 位置作射线终点(原来是 look_at),避免柜子后墙撞击
  误判把整簇 shelf-adjacent cluster 全杀掉。
- `los_min_hit_dist` (默认 0.10m):击中点距离 goal >= 该值时不算 blocked,
  允许"中距离撞物体"的有效观察。
- `computeAttractiveness` 排除地板 (`z <= z_min`) 和机械臂自身占据体素
  (`|q - base| <= reach_min`),防止贴地板的 -X 候选拿假 attract 分。
- `attract_search_radius` 0.30 → 0.50m,覆盖柜子两侧 42cm 侧壁。

**Size penalty**(压制 hemisphere 大团聚 cluster)
- 硬阈值 `fine_count = 5000`(内部常量,不作 param):5000 以下 sizepen=1.0,
  超过按 `5000/fc` 递减,下限 0.3。原自适应 `1.5×median` 会误伤中等 cluster
  (fc≈2000,如柜子顶层内部)。

**Probability 评分**
- `ig_hit_bonus` 0.5 → 3.0:hit_ratio(视锥内命中 known-occupied 的射线
  比例)是"看向物体"最强的信号,权重放大后柜子候选的 prob 从 0.44 → 0.999。

**候选选择**
- Cluster 层:直接按 weight 取 top-K(原 informed FPS 会因空间发散把柜子
  稀释)。
- Candidate 层:`use_greedy_argmin=true`(默认)绕过 raplab HPP-PT 的图搜索
  (启发式非 admissible,多 candidate prob=0.999 时排序抖动),直接按
  `probability / effective_cost` 选 top-1。原 HPP-PT 路径保留作 fallback。

**Alt viewpoint (pullback)**
- Primary IK 失败时,goal 沿 `(base → goal)` 反方向拉近 `alt_pullback_dist`
  (默认 20cm),look_at 完全不变。相机从更"卷起来"的姿势看同一 unknown
  区域,IK 通常更松。
- 只对 `probability >= alt_min_probability=0.4` 的 candidate 生成 alt probe。
- Fallback:pullback 触发 forbidden/inward 时试 4 方向 perpendicular offset。

**HPP-PT edge cost discount**(未用 greedy 时的兜底)
- start→candidate 边权按 `raw_cost / (1 + hppt_cost_attract_boost * attract)`
  打折,让高 attract 候选相对便宜。

**Probe blacklist + threading**
- `cost_probe` 结果 `success=false, planning_time < fast_reject_planning_time`
  视为 IK 秒拒,立刻加 fast-reject 黑名单(半径 `fast_reject_radius=0.15m`),
  不等 HPP-PT 图搜索完成。
- 加 `blacklist_mutex_` 修 MultiThreadedExecutor 下 result callback 与
  octomap callback 并发写 blacklist_ 的 race condition。

**Visited 逻辑**
- `visited_radius` 0.12 → 0.30m,已访问的 0.3m 邻域 45s 内不再选,
  避免反复扫柜子前同一小区域。

**边缘错误处理**
- `boundaries.empty()` 从 `rclcpp::shutdown()` 改成 soft return,不再因暂态
  filter 结果杀掉整个 node。
- `cost_probe_timeout_sec` 30 → 60s(alt 加 probe 后 K=5 变 10 个 probe)。

### 新参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `los_min_hit_dist` | `0.10` | LOS 击中点 >= 该距离不算 blocked |
| `attract_search_radius` | `0.50` | attract 搜索半径 (原 0.30) |
| `visited_radius` | `0.30` | 已访问排除半径 (原 0.12) |
| `ig_hit_bonus` | `3.0` | IG 击中 occupied 的加分权重 (原 0.5) |
| `prob_w_attract` | `0.4` | attract 项在 prob 里的权重 |
| `alt_pullback_dist` | `0.20` | primary IK 失败时 goal 沿 base 方向拉近距离 |
| `alt_min_probability` | `0.4` | 生成 alt probe 的 prob 门槛 |
| `use_greedy_argmin` | `true` | 绕过 HPP-PT,直接 prob/cost 排 top-1 |
| `hppt_cost_attract_boost` | `3.0` | HPP-PT 边权 attract 折扣 boost |
| `fast_reject_planning_time` | `0.05` | IK 秒拒判定阈值 (秒) |
| `fast_reject_radius` | `0.15` | IK 秒拒黑名单半径 |
| `cost_probe_timeout_sec` | `60.0` | probe 总超时 (原 30,alt 加 probe 后翻倍) |

内部常量(非 param):`SIZEPEN_HARD_CAP=5000`,`SIZEPEN_FLOOR=0.3`。

### 新工具

**Shelf 覆盖率测试**
- `ros2_ws/src/jaka_exploration_core/jaka_exploration_core/shelf_coverage_monitor.py`
  订阅 `/octomap_binary`,统计柜子 AABB 内 known/unknown 体积比,定时输出。
- `ros2_ws/scripts/test_shelf_coverage.sh` 一键测试驱动 (`MODE=hppt|random`,
  `HPPT_EXTRA_ARGS="--ros-args -p key:=val"` 传参)。
- `ros2_ws/scripts/test_shelf_coverage_compare.sh` HPP-PT vs random 对比。
- `ros2_ws/scripts/run_v9_batch3.sh` 3-run batch driver。

**理论上限**
- `ros2_ws/scripts/shelf_theoretical_coverage.py` 纯 numpy,采样 N 个相机
  位置 × M 个 look-at 方向,IG 打分算出柜子 AABB 的可观测理论上限
  (~98% for hollow shelf model with 3 internal panels)。

**Random baseline**
- `ros2_ws/src/jaka_exploration_core/jaka_exploration_core/random_frontier_selector.py`
  与 hppt_boundary_ros 相同 frontier 提取和 workspace 过滤,候选用
  `random.randrange` 均匀抽取(不做 HPP-PT)。对比 HPP-PT 相对 random 的
  性能增益。

### 关键诊断日志

hppt_boundary_ros 现在会打印:
- `★ SHELF-ADJACENT frontier voxels`:柜子 AABB 邻域(扩 30cm)内的
  frontier 数,pre-filter / post per-voxel 两阶段。
- `★ SHELF-ADJACENT: raw_shelf_clusters=X  dropped_by(vis bl perm in los)`
- `===== CLUSTER DIAGNOSTIC =====` 每 cluster 的 centroid/distance/fine/
  super/attract/sizepen/weight 完整 dump,`P` 标记被选中,`S` 标记
  shelf-adjacent。
- `HPP-PT edge costs (raw → eff / attract)` 每 candidate 的原始 / 折扣代价。
- `GREEDY-ARGMIN ranks` 和 `GREEDY picked` greedy 模式排序和选中。
- `ALT used (alt_goal=... cost=...)` primary IK 失败切换到 alt 的事件。

### 测试结果 (shelf world, 3-min windows, N=3)

| 版本 | Mean Δcov | Best | Worst | Std | 备注 |
|------|---:|---:|---:|:-:|-----|
| v1 baseline (无优化) | +2.0 | +2.0 | +2.0 | – | 100% IK 秒拒 |
| v6b (LOS/sizepen 初版) | +1.79 | +8.0 | 0.00 | ±3.5 | sizepen 误伤柜子中等 cluster |
| v7 (prob boost + floor exclude) | +3.01 | +8.81 | 0.00 | ±5.05 | shelf 进 top-5,HPP-PT 排序抖动 |
| Greedy (v8) | +3.60 | +6.99 | +1.48 | ±3.05 | +X 命中率 77% |
| v9 (sizepen=5000 + visited=0.30) | +4.17 | +8.37 | +1.85 | ±3.62 | 高 z shelf-top 候选首次进 top-5 |
| **v11 (raw score + sky penalty + z-cap + z-preserve + visited=0.15)** | **+8.29** | **+8.71** | **+7.99** | **±0.38** | **稳定,方差 10× 缩小** |

**v11 均值达到 +8.29% 且 std 极小(±0.38)**,是所有版本里唯一"每次都稳定 +8%"
的配置。相比 v9 均值翻倍,方差缩小一个数量级。见下面的 2026-07-16 改动
细节。

理论上限 (shelf_theoretical_coverage.py, hollow_shelves × 3 model,
res=0.02, 300 cameras × 6 orientations): **~98.5%**。
当前 3-min 达到 ~70% 是"选择层"能到的最高;要进一步需要:
- 更长时间窗口(6-10 min → 更多 task 累加)
- 或者提高 arm velocity_scaling / 缩短 transit path
- 或者 IK-first 采样避免 SURROGATE(camera 反推得不到"手腕伸开口内")

## 视点选择改进 v11 (2026-07-16)

v9 之后针对**"看天分数假高、shelf 候选被 cost 便宜的非 shelf 候选打败、
高 z SURROGATE 相机反而看向天花板"**三个问题联合修复。3-run mean 覆盖率
从 v9 的 +4.17% 提到 **+8.29%**(std ±0.38,几乎每次都 +8%)。

### 三个联合 fix

**Fix 1: Raw score for greedy(替代 clamped probability)**  
`computeViewpointProbability` 加 `out double* raw_out` 输出未 clamp 的加权和。  
HPP-PT 语义要求 `prob < 1` (clamp 到 0.999),多个候选(shelf-facing vs
sky-looking)都会 clamp 到 0.999 → Greedy `probability / cost` 只能靠 cost
决定,便宜的 sky-looking 假分击败 shelf-facing。  
Greedy 改用 `raw_score / cost` 排序,让 clamp 前的真实差距(sky raw≈0.94,
shelf raw≈1.87)反映到排序里。

**Fix 2: Sky penalty(look-dir +Z 分量惩罚 IG)**  
`sky_factor = 1 - 0.7 × max(0, look_dir.z / |look_dir|)`。相机上仰时
IG 项被 × sky_factor(直上 → × 0.3)。原因:`ignoreUnknown=true` 的 IG
raycast 让"射线走满 2m 全 unknown"的"看天"拿到 IG_norm=1.0 假高分。
sky_factor 从**评分层**惩罚这种情况。

**Fix 3: `unknown_dir.z` cap 在 `resolveViewpoint`**  
```cpp
if (unknown_dir.z() > 0.3 && xy_mag > 0.1) {
    unknown_dir.z() = 0.3;
    unknown_dir.normalize();
}
```
柜顶 frontier 的 unknown 邻居多在 +Z 方向,原始 `unknown_dir.z` 常常 0.7-0.9,
`look_at = frontier + unknown_dir × (standoff+1)` 冲到 z=2m 天花板。cap 到
0.3 后 look_at 大部分水平指向柜内。相机不再是 "站柜前看天花板",而是
"站柜前平视柜内"。**这是最关键的一个 fix** —— 一个 SURROGATE task 的
覆盖率贡献从 0% 提到 +5-7%。

### Z-preserving 投影

`projectToReachable` 从纯 radial 缩放改成**z 优先保留**:
- 原:目标点沿 base→p 方向按比例缩到 `projection_scale × reach_max` 球面,
  高 z frontier 的 z 会被 proportional 压到低 z (0.9 → 0.5)。
- 现:优先保留 `target_z = min(frontier.z, target_r × 0.95)`,只在 xy 方向
  缩到 `sqrt(target_r² - target_z²)` 半径。柜顶 (z=0.7-0.85) frontier 的
  camera 保持在原 z 高度水平看柜内,而不是站低处斜看上去。
- 无解退回原 radial(z 需求超出 reach 球时)。

Param: `projection_z_preserve` (默认 `true`,可关掉对照)

### Visited radius 微调

`visited_radius` 参数经历 0.12 → 0.30 → **0.15**:
- 0.12 (原):太小,反复扫柜子同一小区域,coverage 不涨
- 0.30 (v9):太大,把柜子中层 3-7 个相邻 cluster 一次淘汰,下一 cycle 无候选
- 0.15 (v11):中间;每 cycle 淘汰 0-1 个柜子候选,让 shelf-adjacent cluster
  持续存在,greedy 有的选

### 新参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `projection_z_preserve` | `true` | z-preserving SURROGATE 投影;false 回退纯 radial |
| `visited_radius` | `0.15` | 已访问排除半径(0.30 太屠杀柜子候选,回调) |

新加 out-param(不是 ROS param): `computeViewpointProbability(bp, double* raw_out)`
返回未 clamp 的加权和。

### 关键诊断日志更新

- `Sampled probs (raw):` 现在同时打印 `clamped(raw)` 两个值,方便看
  clamp 抹掉了多大差异。
- `GREEDY-ARGMIN ranks (score=raw/eff_cost):` 排序基于 raw 而非 clamped prob。
- `GREEDY picked ... raw=X.XXX prob=Y.YYY`:同时显示,方便追溯为什么某
  raw>1 但便宜的候选没赢便宜非柜子候选。

### v11 单次运行诊断示例

Task 4 goal `[+0.211, +0.055, +0.662]` SURROGATE,单次贡献 +7.26%。这个位置
在 v9 时期是 "看天" 0% 覆盖率:
- unknown_dir cap 前:`unknown_dir.z ≈ 0.87` → look_at.z = 0.86 + 1.35×0.87 = 2.03m(天花板方向)
- Cap 后:`unknown_dir.z = 0.3` → look_at.z = 0.86 + 1.35×0.30 = 1.27m(斜上 25°)
- Sky_factor:1 - 0.7×0.3 = 0.79(轻微惩罚)
- IG raycast 撞到柜后墙 → hit_ratio 高,IG_norm 中等 → raw ≈ 1.87(真高分)
- Greedy pick i=1 raw=1.87 cost=20.63,score=0.091 击败 i=3 raw=0.92 cost=15.3
  score=0.06(如果用 clamped prob=0.999 两者一样,便宜的会赢)
- Arm 执行 24s 长距离移动,camera 从 (0.21, 0.06, 0.66) 平视柜顶内部 → **+7.26%**

### 剩余限制 (v11 时期)

- **单次 task 覆盖率上限 ~7%**,累积到 70% 需要 10 个 task,3-min 只跑 4-6 个
- Arm velocity_scaling=0.2 保守,单个 SURROGATE task exec 可达 24s(70% 时间在移动)
- Cycle 5-6 位置若干 shelf 候选被 visited 淘汰,arm 偶尔跑到 -X 后侧(3-min
  batch 里每 run 1-2 个这种浪费 task)

### 内存泄漏 fix

`test_shelf_coverage.sh` cleanup 的 pkill 名单原来缺 `frontier_viz`、
`initial_scan`、`bootstrap_scan`、`periodic_octomap_saver`,多轮测试后累积
~11 个 frontier_viz 进程(每个 ~2GB VSZ)吃满 22GB 容器内存。已加入
cleanup 名单。

推荐每 10-20 次测试后 `docker rm -f jaka_exploration_ros2` 重建容器,
彻底清 zombie(容器 PID 1 是 `sleep infinity` 不 reap 子进程)。

## Multi world 和 Close cube 场景 (2026-07-16)

除 shelf world 外,增加两个新场景 + 对应 coverage monitor,验证 v11 选择层
在**多物体**和**背面遮挡**场景下的行为。

### Multi world (原有场景 exploration_scene_multi.sdf)

三个物体的 AABB(见文件顶注释)。coverage monitor 统计各 AABB 内的
known/unknown 体积比 + 总覆盖率。

- 新节点: `jaka_exploration_core/multi_coverage_monitor.py`
  entry: `ros2 run jaka_exploration_core multi_coverage_monitor`
- 测试脚本: `ros2_ws/scripts/test_multi_coverage.sh`(基于
  `test_shelf_coverage.sh` 改的驱动,`MODE=hppt|random`,`HPPT_EXTRA_ARGS`)
- 对比脚本: `ros2_ws/scripts/run_multi_compare.sh`(hppt vs random 各 1 轮)

**对比测试结果** (单次 3-min run):

| Mode | Total FINAL | Δ | box Δ | sphere Δ | cyl Δ | 覆盖物体数 |
|------|:-:|:-:|:-:|:-:|:-:|:-:|
| **HPP-PT v11** | **17.20%** | +5.76% | +4.13% | +12.59% | **+8.89%** | **3/3** |
| Random | 15.53% | +6.14% | +6.09% | +12.30% | **0.00%** | **2/3** |

**关键差异**:HPP-PT v11 扫遍 3 个物体,random 完全漏扫红圆柱(coverage
一直 18.93%)。HPP-PT 的 `attract` (排除 floor/base 后) + `hit_bonus × 3.0`
让"物体附近"候选拿高 prob,random 的均匀抽样没有这个 prior。

### Close cube world (新加场景)

单个 20 cm 立方体距 base_link **30 cm 前**并**贴地**,专门测**背面探测能力**:
初始 6 视角看不到 back face,arm 必须绕到 cube 后面(SURROGATE 投影)。

**2026-07-16 更新**: cube 已移得更低更近 —— 底面贴地 (z=0.00),前面距 base 从
45cm 缩到 30cm。目的是让 back face 落进 arm reach 中段,减轻 SURROGATE 依赖。

- 新场景: `ros2_ws/src/jaka_exploration_core/worlds/exploration_scene_close_cube.sdf`
  ```
  cube center: (0.40, 0.00, 0.10)  size 0.20 x 0.20 x 0.20
  AABB: x=[0.30, 0.50]  y=[-0.10, 0.10]  z=[0.00, 0.20]
  front face x=0.30  back face x=0.50  bottom z=0.00 (贴地)
  ```
- 新节点: `jaka_exploration_core/close_cube_coverage_monitor.py`
  除 total cube coverage 外,还统计 **6 个面的 2cm 厚 slab 覆盖率**,
  单独看 back face 探测程度。注意 bottom slab 与地面重叠,理论上永远不可见,
  保留只是对称性。
- 测试脚本: `ros2_ws/scripts/test_close_cube_coverage.sh`

**距离几何(新)**:
| 面 | 相机位置(standoff=0.35) | 距 base | 状态 |
|:-:|:-|:-:|:-:|
| back 头对头 | (0.85, 0, 0.10) | ~0.86 m | 接近 reach 边界,可能 SURROGATE |
| 侧面 | (0.40, 0.42, 0.15) | ~0.60 m | ✓ 可达(reach 中段) |
| 顶-back 角 | (0.50, 0.15, 0.45) | ~0.68 m | ✓ 可达 |

**单次 v11 测试结果(旧几何,cube center=(0.55,0,0.20) — 需以新几何重跑)**:

```
面           Initial  FINAL   Δ       备注
front (-X)    9.8%    13.2%   +3.4%
back  (+X)    7.4%    11.0%   +3.6%   核心目标,略涨
left  (-Y)    5.0%     9.9%   +4.9%
right (+Y)   51.8%    52.6%   +0.8%   已高
top   (+Z)   49.5%    49.5%   0       已高
bottom(-Z)    5.0%    45.9%  +40.9%   意外爆涨(arm 从下-后仰视)
CUBE total    9.45%   13.54%  +4.09%
```

**观察**:
- Task 3 goal `(0.655, 0.182, 0.104)` (cube 后-右-下角,exec 20s) 是全部
  coverage 增益的来源
- 相机 z=0.10 从后-右-下方仰视 cube,同时扫到 bottom + back-right 边缘
- Back face 只 +3.6%,因为 wrist orientation 挑到"仰视 bottom"而不是"平视 back"
- **z-preserving 投影**把 SURROGATE camera 保持在 z=0.10 附近(target_z 保留
  frontier z 而不是被 radial 压到低 z),这对 back face 是正确方向,但恰好让
  wrist 挑到 bottom-forward orientation

**残余挑战(旧几何)**:cube back face 中心 (0.65, 0, 0.20) 距 base 0.68m,与
reach_min=0.60 只差 8cm。attract 半径 0.50m 里刚好**不含 cube 表面**(base 到
cube 距离 0.58m),attract 分数拉不到 -X 半球候选。放大到 0.80m 或改用"距离最近
物体"的其他先验可能改善。**新几何**下 back face 中心 (0.50, 0, 0.10) 距 base
仅 0.51m,落入 attract 半径内,预期能显著改善 back face 选点覆盖。

### 新参数汇总(v11 之后新增)

无新增 ROS param。新增内部常量 `SIZEPEN_HARD_CAP=5000, SIZEPEN_FLOOR=0.3`
(已在 v11 章节列出),以及 close_cube 场景 face_slab 厚度 `FACE_T=0.02m`。

### 新脚本

| 脚本 | 用途 |
|------|------|
| `run_multi_compare.sh` | multi world hppt vs random 各一轮 |
| `test_multi_coverage.sh` | multi world 一键测试驱动 |
| `test_close_cube_coverage.sh` | close_cube 一键测试驱动 |
| `run_v11_batch3.sh` | shelf world v11 3-run batch(mean +8.29%) |

## 死循环 + 背面盲区修复 (2026-07-16)

close_cube 场景测试暴露出 HPP-PT 三个耦合缺陷,导致 cube back face 长时间
无法探测,且部分场景下 arm 卡死不动。三处修复叠加后:back face 从 9.0% → 56.5%,
left face 从 5.0% → 29.0%,shelf 场景 +8.55% 无回归 (v11 batch3 均值 +8.29 ±0.38)。

### 缺陷 1: `require_outward_look=true` 误杀绕物观测

原逻辑 `outward · look_dir ≤ 0 → 判为向内看,过滤`。cube back face 观测**几何上
天生**满足这个条件(相机站在 cube 后方朝 -X 看,视线指回基座),导致所有 back
face 候选被在 voxel 层直接丢弃(初始 5 min 测试中 `inward=36238` 全被杀)。

**修复**: `require_outward_look_ = declare_parameter("require_outward_look", false)`
默认改 false。真正的自碰撞由 MoveIt IK precheck + collision planner 负责,不需要
这个几何硬截断。5min close_cube 测试:back face `9.0% → 52.6%` (`+42.8%`)。

### 缺陷 2: 几何 forbidden 的 cluster 不被上游过滤 → 死循环

**症状**: close_cube 5 min 测试第 3 个任务完成后 (t≈99s),arm 静止,剩下 200s
没发任何任务。日志显示同 3 个 goal 反复被 GREEDY 挑中又 skip: `[-0.146, 0.389,
0.100]` (r=0.427m), `[0.209, 0.192, 0.250]` (r=0.378m), `[0.438, -0.266, 0.142]`
(r=0.532m),全部 < `reach_min=0.60m`,几何 forbidden。

**根因**:
1. **Cluster centroid filter** (`hppt_boundary_ros.cpp:1509-1533`) 只检查 blacklist /
   visited / perm_blocked / inward / LOS,**没有** `isOutOfWorkspace / isForbidden`
   检查。→ forbidden goal 的 cluster 每 cycle 都存活,派生同一 goal。
2. **GREEDY 层** (`hppt_boundary_ros.cpp:1990-2010`) `isGoalBad` 分支只 skip 不入
   黑名单(设计假设"几何 forbidden 是确定性的,加黑名单是冗余")。
3. arm 不动 → octomap 不更新 → 同 cluster 再来 → 死循环。

**修复 A**(上游堵漏): cluster centroid filter 加 `isOutOfWorkspace(goal)` 检查:
```cpp
if (isOutOfWorkspace(goal)) { ++cforbid; return true; }
```
新计数器 `cforbid` 加入 fallback WARN 日志。

**修复 B**(下游保底): GREEDY skip_bad 分支加 `blacklist.push_back`,防止上游因
浮点/投影漂移漏掉的 forbidden cluster 在 GREEDY 再触发:
```cpp
{
    std::lock_guard<std::mutex> lk(blacklist_mutex_);
    blacklist_.push_back({p, this->now(), fast_reject_radius_});
}
continue;
```

**修复效果** (5min close_cube, 同参数):

| 指标 | Fix 前 | Fix 后 |
|:-|:-:|:-:|
| 5min 内 Published tasks | 3 | **11** (+266%) |
| Deadlock 时长 | ~200s | **0s** |
| back face | 52.6% | **56.5%** (+3.9%) |
| left face | 9.6% | **29.0%** (+19.4%) |
| CUBE total | 14.04% | **16.15%** (+2.1%) |

### 缺陷 3: 未修复 - right face 从未进候选池

**症状**: right face 从初始 bootstrap `right` 姿态一次到 52.5% 后,剩余 47%
在 5 min 内没有增加。cluster diagnostic 显示 cube 正右方 (x∈[0.30, 0.50],
y∈[+0.10, +0.30]) 从头到尾**没有任何 cluster centroid**。

**尝试**: 把 `cluster_min_voxels` 从 5 降到 2,cluster 池从 3-5 涨到 5-10,
右后侧角落 cluster 出现 (`(+0.504, +0.079, +0.036)`),但 right face 只 +2.5%。

**结论**: 不是 cluster 阈值问题。right face 剩余 frontier 是薄壳(y=+0.11 一层),
在 super-voxel 聚合时被**吸并进右侧向外延伸的大空气 cluster**,不能自成
cluster。降 `cluster_min_voxels` 只让**孤立小 cluster** 存活,不能拆开被大
cluster 吸并的 voxel。

**下一步方案(未做)**: 降 `coarse_grid_factor` 3 → 2 让右侧薄壳自成 cluster;
或加"物体邻域独立聚类"策略;或改初始 scan 姿态减少初始高覆盖导致的"cluster 断层"。

### Close cube 场景新几何 + 挡板

原几何 cube center `(0.55, 0, 0.20)`,back face 中心距 base 0.68 m(接近
reach_max),需要 SURROGATE 投影。为让 back face 落入实际可达空间,更新为:
```
cube center: (0.40, 0.00, 0.10)   size 0.20³
AABB:        x=[0.30, 0.50]  y=[-0.10, 0.10]  z=[0.00, 0.20]
front face:  x=0.30  back face: x=0.50  bottom: z=0.00 (贴地)
```
back face 中心 `(0.50, 0, 0.10)` 距 base 仅 0.51 m,落入 attract 0.50m 半径内。

**背景挡板** (2026-07-16 加): `x=0.90, y∈[-0.70, 0.70], z∈[0, 1.20]`,
厚 0.02 m 深蓝色。目的是**尝试**用挡板截断 cube 后方的大空气 cluster,让 back
face frontier 变小 cluster → 顺着 HPP-PT size penalty 偏好被选中。**实测挡板
效果有限** (未被初始扫描观测,一直保持 unknown-占据,不能有效截断),真正解决
back face 问题的是缺陷 1 + 2 的修复。

### 运行时性能剖析 (2026-07-16 分析,改动已回滚)

从 shelf 5min 日志抽出各阶段耗时中位数:

| 阶段 | 时间 | 占比 |
|:-|:-:|:-:|
| boundary filter | ~0.4 s | 决策 |
| cluster + FPS | ~0.4 s | 决策 |
| **cost probe (IK 串行)** | **~4 s (max 12.5s)** | **决策瓶颈** |
| GREEDY 选点 | 负毫秒 | 决策 |
| **MoveIt planning + arm 执行** | **~35s (max 67s)** | **执行主导** |

**总时间账**: 5min 内决策 ~45s (15%),arm 执行/等待 ~235s (78%)。

**试验性优化 (A+B+C,未合入)**:
- **A**: `moveit_allowed_planning_time` 5s → 2s + `moveit_num_planning_attempts`
  10 → 3。原 5s×10=50s 最坏 planning timeout,规划失败通常前 2 次能看出。
- **B**: cost_probe 全 cycle 至多 2 个 alt (原每 candidate 都可能造 alt)。
- **C**: `cost_probe_timeout_sec` 60s → 5s。个别 probe 卡壳不再拖垮整 cycle。

**A+B+C 测试结果 (3min shelf)**:
- Task 吞吐 1.4 → 2.0 tasks/min (+43%)
- Coverage rate 1.71 → 1.99 %/min (+16%)
- 决策中位 4s → 2.5s (-38%),最大 13.8s → 5s (-64%)

**为什么回滚**: 3min 测试样本小,`cost_probe_timeout=5s` 已经出现 `1/2 succeeded`
(50% probe 被截断),担心稀有但难的 goal (SURROGATE 大动作) 被误判,导致长期
稳定性下降;需要更多轮 batch 才能确认收益/风险比。**这些参数保留在 param
定义处的注释中**,可以通过 CLI 覆盖复现: `--ros-args -p
moveit_allowed_planning_time:=2.0 -p moveit_num_planning_attempts:=3 -p
cost_probe_timeout_sec:=5.0`。

## 已知限制

- 当前 HPP-PT 启发函数不满足 admissible 条件，因此只能得到近似结果，不能保证
  全局最优。
- self-filter 在 TF 未就绪时会丢弃该帧点云，可能暂时降低建图帧率。
- conservative map 只覆盖 `pipeline.yaml` 配置的 planning bounds；边界设置必须覆盖
  实机完整工作空间，否则边界外空间不受 unknown-as-occupied 保证。
- Planning Scene 更新只影响下一次规划，不会自动重新检查正在执行的轨迹。
- known-free 实机 launch 已完成代码接入，但尚未完成真实机械臂闭环验证。

## 后续工作

1. 修正 HPP-PT 启发函数并评估解的质量。
2. 对比 RRTConnect、RRT*、PRM、EST 等多种 motion planning 算法。
3. 在 HPP-PT 选择视点前增加 IK 和 state validity 预检。
4. 使用空间随机或加权采样替代索引均匀采样。
5. 增加执行期间碰撞监控和自动重规划。
6. 持久化 explored map，避免重复扫描已完成区域。
7. 接入真实 JAKA controller，验证真实 RGB-D 自主探索闭环。

## 投影/alt 重写 + FREE 盲柱注入 + 关节限位收窄 (2026-07-24)

一批相互关联的改动:HPP-PT 视点算法整体重构、结构性盲区改为直接注 FREE
到 octomap、URDF 关节限位收窄、初始扫描支持多轮 + 诊断事件。

### 视点投影:沿 look_at 线交可达球

`projectToReachable(p, unknown_dir)` 从旧的 "base→goal 径向缩到球面"
改成 **"沿 frontier→look_at 直线延长到可达球"**:

- 每个 frontier 有一条自然观察轴 `ud = look_at − frontier`(camera 应该在
  这条线的 free 侧,看向 unknown 侧)
- 若 primary goal 落在 reach shell 外(`isUnreachable`),沿 `cam_dir = -ud`
  从 frontier 出发解二次方程,和 `reach_max × projection_scale` 球面求交
- 取近交点(`t_near > 0`),camera 落在 shell 上,朝向 frontier 的观察角度
  与 primary 完全一致 —— 只是滑到 reach shell 内的最接近点
- 退化情形(线不与球相交,或指向背离 base)才 fallback 到径向缩放

**旧径向投影**保留了半径但**丢了观察角度**;新投影保留观察角度,IG raycast
命中 unknown 更准。5min shelf 实测 octomap 增长率提升 ~7%。

调用点(`resolveViewpoint` + `generateAlternativeViewpoints` 内 5 处)
全部把 `unknown_dir` 传下去。同时移除了曾经的"z-preserving" 分支和 P1-Z
sky-look cap,frontier 朝上时相机就朝上看,不再截断 `unknown_dir.z`。

### Alt viewpoints:8 个 angular deviation

原 `generateAlternativeViewpoints` = pullback 20cm + 4 个垂直偏移
(±up/±right,0.20-0.30m)。重写为 **8 个角度偏离**:

- 在 `right_axis = normalize(ud × helper)` 和 `up_axis = normalize(right × ud)`
  两个正交轴上,把 `ud` 用 `Eigen::AngleAxisd` 旋转 ±30° 和 ±60°
- 得到 8 个新方向 `new_ud`,每个生成完整的 `(alt_goal, alt_look_at)` 对
  (**alt 有自己的 look_at**,不再和 primary 共用)
- 顺序:先 30° 的 UP/DOWN/LEFT/RIGHT,再 60° 的 UP/DOWN/LEFT/RIGHT
- `alt_max_count` 默认 2 → **8**,把 8 个全部投 probe

STAGE 4 里 primary 失败就挑 cost 最低的 alt 顶上,逻辑不变。

### Permanent blacklist(within session)

`blacklist_duration_sec` 彻底删掉:regular blacklist 一次进就**本次运行永不
释放**。`sticky_blacklist_` 仍在 STAGE 5 EMPTY 时统一清空。目的:防止
"失败 goal 60s 到期后被反复选中反复失败"的循环。

### Cluster size penalty 移除

TIER-5 `SIZEPEN_HARD_CAP` / `SIZEPEN_FLOOR` 完全删除;`weights[ci]` 只算
`(0.1 + fc_norm) × (1 + attracts)`。大 cluster 不再被反比降权,STAGE 4 raw
和 probability 也不再乘 size_penalty。相应的 `last_picked_size_penalties_`
class 成员和所有 plumbing 一并清掉。

### `octomap_free_region` 新包(C++)

新独立 ROS 2 包,拦截 `octomap_server` 输出并往 UNKNOWN 位置注入 FREE
叶子后再发布:

```
octomap_server → /octomap_binary_raw
                   ↓
              octomap_free_region  (往盲柱内 UNKNOWN cell 塞 FREE)
                   ↓
              /octomap_binary   (下游全部用这个)
```

参数为平行数组,支持多个圆柱区域:

```yaml
cylinder_x:     [0.0,  0.0]
cylinder_y:     [0.0,  0.0]
cylinder_r:     [0.30, 0.95]
cylinder_z_min: [0.02, 0.85]
cylinder_z_max: [1.25, 1.25]
sample_spacing: 0.025
```

- **cyl 0**:整根 J1 竖直轴(r≤0.30, z 0.02~1.25)—— 覆盖手臂 base + J1
  本体所在竖直柱,以及折叠扫描姿态下相机附近的结构性盲区
- **cyl 1**:头顶盘(r≤0.95, z 0.85~1.25)—— 覆盖折叠姿态下相机水平方向
  0.95m 半径的头顶盘,包含机械臂头顶那圈"看不见"的 unknown 环

每个位置只在 octomap `tree.search(p) == nullptr`(即真的 UNKNOWN)时
才 `updateNode(p, false)`,不覆盖任何 real observation(FREE/OCC 叶子
维持原样)。real observation 永远优先。

sim (`exploration_stage2.launch.py`) 和真机 (`real_arm_exploration.launch.py`)
都接入了 `octomap_free_region`,`octomap_server` 输出 remap 到 `_raw`。

### URDF 关节限位收窄

`jaka_a5_description/urdf/jaka_a5.urdf.xacro` +
`jaka_exploration_description/urdf/jaka_a5_with_d435.urdf.xacro`:

| 关节 | 旧 (rad) | 新 (rad) | 新 (度) |
|---|---:|---:|---|
| joint_1 | ±6.28 | **-1.5708 ~ +4.7124** | **-90° ~ +270°** |
| joint_5 | ±6.28 | **-3.14 ~ +3.14** | **±180°** |
| joint_6 | ±6.28 | **-3.14 ~ +3.14** | **±180°** |

J1 上限 +270° 与真机 initial scan 的 J1 sweep(-90° → +270°,360° 一圈)
完美对齐,不再顶到旧的 ±360° 边界。J5/J6 收到 ±180° 减小 IK 搜索空间。

### Initial scan:多轮 + 诊断触发

`initial_scan_node` 增加两个参数:

- `iterations`(默认 1):把 5-phase 扫描 loop N 次(iter>1 时每次
  自动 home 到 phase-1 起点)
- `between_iteration_pause_sec`(默认 3.0):iter 之间的等待,方便
  octomap 融合

每次 iter 结束后 publish `std_msgs/String` 到
`/viz_scan_diag_trigger`,data = `"iter_N"`。搭配 `viz_scan_diag` 节点
可以按 iter 分别 dump 诊断 JSON。

`exploration_stage2.launch.py` 加了 `initial_scan_iterations` launch arg,
自动透传给 initial_scan Node。

### Frontier viz 重写:原生 emit octomap UNKNOWN

`frontier_viz.py` 完全重写:

- 不再自己扫 workspace 网格找 unknown;直接遍历 octomap 二叉树,
  每个 `0b00` UNKNOWN 子节点原样 emit 成一个 CUBE_LIST cube(大小 =
  该节点在 tree 里剪枝到的自然深度)。彻底摆脱之前 coarse-sampling
  bug(取一个 fine cell 代表 512 个)。
- CUBE_LIST 强制每 marker 一个 scale,按 size 分桶,每 size 一个
  marker,`marker_id = int(size × 1000)` 稳定 ID(不同 size bucket
  之间不会 shuffle)
- 默认参数:
  - `max_display_size = 0.5m`(过滤 root 级别几十米的大 UNKNOWN)
  - `workspace_r_max = 2.125m`(和 HPPT `frontier_max_range_factor
     × reach_max` = 2.5 × 0.85 对齐)
  - `workspace_z_max = 2.00m`
- 每 30 帧打一次 leaves 计数 log

### `viz_scan_diag` 新诊断节点

`jaka_exploration_core/viz_scan_diag.py`:

- 订阅 `/frontier_markers` + `/octomap_binary` + `/viz_scan_diag_trigger`
- 累积 per-fine-cell 的 `first_known_ts` 和 `updates_count`
- 收到 `String` trigger(data = 任意 label)后 dump `/tmp/viz_scan_diag_<label>.json`,包含:
  - viz 黄方块 vs octomap UNKNOWN 立方的 cube-level 对比(pos + size)
  - fine-cell 级别 coverage(viz_says_unknown_but_octomap_knows,
    octomap_unknown_but_viz_missed 等)
  - 每个黄方块的 scan_pressure:`fine_cells_ever_known`,`update_total`,
    `first_known_min/max_sec`
  - **跨 snapshot delta**:上次 dump 到本次 dump 之间 viz/octomap
    unknown 的 stayed/resolved/newly-yellow 计数
- 支持一次 launch 中按 trigger 多次 snapshot(常用于 `initial_scan
  iterations=2` 的场景)

### Launch 参数变更

**`exploration_stage2.launch.py`**(sim):

- `octomap_server` resolution `0.01 → 0.03`(sim 中更接近真机)
- 加 launch arg `initial_scan_iterations`(默认 1)
- octomap 输出 remap 到 `/octomap_binary_raw`,`octomap_free_region`
  接手输出 `/octomap_binary`

**`real_arm_exploration.launch.py`** + **`real_arm_realsense_octomap.launch.py`**(真机):

- 同 remap 链:`octomap_server → /octomap_binary_raw → octomap_free_region
  → /octomap_binary`
- HPPT Node 参数覆盖:
  - `frontier_max_range_factor: 2.5`(too_far 阈值 1.275m → **2.125m**,
    让 shelf 深处 frontier 进入候选池)
  - `z_max: 1.20`(允许柜顶 frontier 进入 candidate)

### 真机 scan defaults

`jaka_scan_and_publish.py`:

- `SCAN_SPEED_DPS = 10.0`(scan-only,execute 保持 `SLOW_SPEED_DPS = 5.0`)
- 5 phase 包含 `camera_up`(J4=180°),真机扫顶
- `DEFAULT_J1_START_DEG = -90`, `DEFAULT_J1_RANGE_DEG = 360`
  与 URDF J1 新限位 [-90°, +270°] 完全对齐

### 移除

- `save_scan_snapshot.py` / `replay_scan_snapshot.py`:snapshot 保存重放
  链路不再使用
- HPPT 里所有 TIER-5 size penalty 代码 + SKY-LOOKING FIX(unknown_dir.z
  截断到 0.3)

### 保留但未启用的实验分支

- `generateAlternativeViewpoint` singular 版本(pullback 老逻辑)代码
  留着,当前无调用
- self_filter 里合成 free 光线注入方案 —— 已回滚,当前只做纯 self-point 过滤

### 已知 shelf 场景遗留问题(需要后续调)

- 真机 shelf 场景下 HPPT 每次 STAGE 5.5 cost probe 都返回 `cost=max`,
  MoveIt 报 OMPL FAILURE,推测是电缆被 octomap 标为 OCC 阻塞 planner。
  下一步:扩大 self_filter 的 LINK_SPHERES 半径或加 "cable" sphere。
- Permanent blacklist 加上后,shelf cluster 反复失败会快速把 primary
  goal 全部拉黑,STAGE 3c EMPTY 频繁触发;需要 alt 层面也能救回被
  blacklisted 的 cluster(目前只在 LoS blocked 时能救 —— 该逻辑已
  回滚,只保留了 8-alt STAGE 4 层面)。

---

## tsp_real_0724 变更（2026-07-24）

> 相对上一次 push（分支 `tsp_real_0723` @ `1f99d8e`）的全部改动。
> 主要文件：`hppt_boundary_ros.cpp`、`motion_planner_node.py`、
> `exploration_scene_shelf.sdf`、新增 `exploration_scene_complex.sdf`、
> `exploration_view.rviz`。

### 1. 视点选取重构为「方向优先」（direction-first）

`hppt_boundary_ros.cpp`。旧流程「每个 frontier 算单一视点 → 打分 → 事后
`generateAlternativeViewpoints` 补救 IK」改为：

1. **STAGE 3**：26-邻接聚类 + min_voxels + centroid 过滤，**去掉 weight top-K
   预选**，所有存活 cluster 作为 seed（`max_frontier_seeds` 安全上限）。
2. **STAGE 4** `generateViewpointCandidates`：每个 seed 绕 unknown 主方向
   **扇形展开 N 条 look_at 方向**（`viewpoint_fan_angles_deg` 默认 {25,50}，
   `viewpoint_max_directions` 默认 9）；每条方向沿视线/延长线用**几何法**
   （reach 球面投影 + workspace + `isLineOfSightBlocked` 碰撞 raycast）找可达
   相机位，撞障碍/不可达/向内看/已访问/黑名单则拒绝。
3. **STAGE 5** `scoreViewpoint`：对每条存活方向独立打分（obs/unk/occl/sem/
   ig/peek/attract + IG-gate + LOS-gate），按 raw 分降序。
4. **STAGE 6**：取 top-N（`motion_planning_top_n` 默认 5）依次送 plan-only
   MoveIt，**第一个规划成功即发布 execute**。

### 2. HPPT 决策日志（decision logger）

- 打完分后 `logScoredDirections` 按分数降序记录每条方向
  【位置 goal、方向、各项评分 obs/unk/occl/sem/ig/peek/attract + los/igate +
  RAW】。
- 所有拉黑统一走唯一入口 `blacklistPoint(pos,radius,type,reason,permanent)`，
  记录【类型 + 原因 + 永久/临时】。
- 统一出口 `hpptLog` 镜像 rclcpp INFO + 落盘（`hppt_log_file`，默认
  `$HOME/.ros/hppt_decision.log`）。

### 3. 非保守兜底 + 分级拉黑

`motion_planner_node.py` + `hppt_boundary_ros.cpp`。探索目标**保守规划失败**
时（unknown=障碍），自动用 `allow_unknown=True`（unknown=free）**再规划一次**
（plan-only，`_classify_with_nonconservative`）分类失败原因：

| 非保守结果 | error_type | 拉黑等级 |
|---|---|---|
| 成功 | `blocked_by_unknown` | 临时 |
| 超时/没找到（TIMED_OUT/PLANNING_FAILED） | `nonconservative_timeout` | 临时 |
| 直接失败（IK/碰撞/无效计划/违反约束） | `hard_unreachable` | **永久** |

- HPPT 侧 `classifyBlock` / `blacklistByErrorType` 按 error_type 分级：
  `hard_unreachable`→永久（regular，本 session 不释放）；其余→临时（sticky，
  STAGE EMPTY 释放）；基础设施 retryable→不拉黑。
- **拉黑半径统一 `block_radius_small`=0.06m**（只封失败落点附近一小块，不覆盖
  整个 cluster）。
- **env 快照修复**：`_execute_callback` 开头 deepcopy 一份 `task_environment`，
  两遍规划复用同一张 octomap（`_apply_environment` 新增 `environment` 参数、
  跳过版本校验），避免规划期间地图漂移导致第二遍 `environment_version_mismatch`。
- 参数 `enable_nonconservative_fallback`（默认 true）。
- 注：拉黑的是**机械臂落点（相机 goal 位置）**，不是 frontier / look_at 目标。

### 4. 探索收敛自动停机

- 连续 `convergence_empty_cycles`（**默认 20**）次「未产出目标」的选点周期
  **且 octomap 无实质增长**（节点增量 < `convergence_growth_leaves`=100）→
  `declareConverged`：打印 `exploration converged`、发 latched
  `/exploration_complete`（std_msgs/Bool）、按 `convergence_shutdown`（默认
  true）调 `rclcpp::shutdown()`。
- 接线于全部 EMPTY 出口（STAGE 1/2/3/4/6 EMPTY）；发布目标或地图增长时重置。
- 修复了之前「探完可达区后无限 STAGE 2 EMPTY 空转到 timeout」的问题（此前唯一
  `rclcpp::shutdown()` 只在 `main()`，只响应外部 SIGINT）。

### 5. 最低信息增益门槛（MIN-IG gate）

- STAGE 6 里候选按 raw 降序；最优/剩余最高分 `raw < min_publish_raw_score`
  （**默认 0.05**）→ 判「未产出有效目标」，**不发布** + 计入收敛（`STAGE 6
  LOW-IG`）。
- 治两个病：机械臂后期不再被派去「正对可见障碍面做零价值观察」；这种「只剩
  垃圾候选」状态计入收敛 → 正常自停（实测同场景从 144 周期空转到 timeout →
  28 周期自然收敛）。

### 6. 选点 / 拉黑可视化（RViz）

- 新 topic `/hppt_ranked_viz`（MarkerArray）；`exploration_view.rviz` 加了默认
  启用的「HPPT Ranked」显示项。
- 每 cycle top-5 视角：**#0 最高分=绿（大球）**、**#1–4=青**，各带视线 ARROW；
  **拉黑区域 SPHERE**（直径=2×半径）：**永久=红 / 临时=橙**（半透明）。
- 全部 `lifetime=0`（永久留存，同 id 覆盖）、`enable_ranked_viz` 默认 true。

### 7. 仿真场景

- `exploration_scene_shelf.sdf`：加了 **1.5m 天花板**（6×6 薄板，box 碰撞）。
- **新增 `exploration_scene_complex.sdf`**：基于 shelf 的更复杂观察场景——
  货架+多物块、可穿过看的**格栅**、正对管口才能看的**方形粗管**、越过前壁
  反向向里看的**凹室**（各带醒目目标块，全 box 碰撞）。运行：
  `ros2 launch ... exploration_stage2.launch.py world_file:=exploration_scene_complex.sdf`

### 新增参数汇总（HPPT node）

`viewpoint_fan_angles_deg` {25,50} · `viewpoint_max_directions` 9 ·
`viewpoint_dedup_tol` 0.08 · `motion_planning_top_n` 5 · `max_frontier_seeds` 50 ·
`block_radius_small` 0.06 · `hppt_log_file` · `enable_convergence_detection` true ·
`convergence_empty_cycles` 20 · `convergence_growth_leaves` 100 ·
`convergence_shutdown` true · `exploration_complete_topic` /exploration_complete ·
`min_publish_raw_score` 0.05 · `enable_ranked_viz` true。
motion planner：`enable_nonconservative_fallback` true。

### 已知问题 / 待办

- `hard_unreachable`（永久拉黑）路径已接线但尚未在真实场景触发（复杂场景的
  管内/凹室目标表现为 `blocked_by_unknown` 或被 reach/几何过滤，未升级到永久）。
- 拉黑仅拉「落点」，不拉「被观察的 look_at 目标区域」；同一片够不到/被遮的
  unknown 仍可从别的站位反复生成视点（已由 MIN-IG 门槛大幅缓解）。
- 旧的 `generateAlternativeViewpoints` / HPP-PT greedy 图搜索分支保留未删。
