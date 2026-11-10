# HPPT 探索决策逻辑（frontier 选取 + 视点生成/打分/选择 + 任务编排）

> 范围：`hppt_boundary_ros` 节点（`ros2_ws/src/hppt_boundary_ros/src/hppt_boundary_ros.cpp`）的**全部决策逻辑**。
> **不含**：initial scan、感知、建图（octomap_server）、语义概率节点、路径规划模块（motion_planner_node.py / MoveIt / 保守-非保守规划）。
> 本节点是整个系统的"大脑"：读 octomap → 找 frontier → 生成/打分/选视点 → 编排 MoveIt 探针 → 发布 execute 目标 → 处理结果 → 维护黑名单/收敛。
> 文档同步于 2026-07-25 的代码状态（含本会话所有改动）。

> ⚠️ **new_method 分支更新（2026-07-25）**：`hppt_boundary_ros` 的**默认**决策路径已从
> 下文描述的 direction-first 流水线**替换为可验证的 Greedy Baseline**
> （`use_greedy_baseline=true`）。本文档以下章节（STAGE 0–6、direction-first 打分/选择）
> 现在描述的是 **legacy 路径**（`use_greedy_baseline=false` 时才走）。Greedy Baseline 主路径为：
> Frontier 聚类 → 多方向×多距离 viewpoint 生成 → IK+碰撞预过滤（MoveIt `/compute_ik`，
> 每 viewpoint 绑定 joint_state）→ 三项归一化打分（unknown density + geometric visibility
> + semantic prior）→ per-frontier Weighted Mean Shift → current→candidate **工作空间 A***
> motion cost → Greedy utility（`λ_score·p − λ_motion·ĉ`）→ plan-only 验证 → execute，
> 失败标 Infinity 边并重选。纯算法在 `src/hppt_baseline.hpp`，实现说明见 `README.md`
> 的「new_method 变更」段。以下 STAGE 0（解析/TF/收敛/markers）、STAGE 1（frontier 提取）、
> 黑名单/visited/收敛/RViz/pending 状态机/plan-only 探针/execute 派发等**基础设施两条路径共用**。

---

## 0. 架构与数据流

**完全回调驱动，没有主定时器。** 主循环 = 每收到一帧 OctoMap 触发一次 `octomapCallback`。

订阅 / 发布：

| 方向 | topic | 用途 |
|---|---|---|
| sub | `/octomap_binary`（octomap_msgs/Octomap） | **主循环触发**；每帧跑一次选点 |
| sub | env_status（String JSON） | 下游是否接受了某个 octomap 版本 → 门控 |
| sub | `/team/motion_result_json`（String JSON） | MoveIt 探针 / execute 的结果 |
| sub | feedback（String JSON） | 下游是否已接手 pending 任务 |
| sub | `/semantic_frontier/semantic_points` | 语义点（*语义模块，本文档不展开*） |
| pub | task_topic（String JSON） | 给 motion planner 发 plan-only 探针 / execute 任务 |
| pub | `/hppt_boundary/markers` | frontier / goal marker |
| pub | `/hppt_ranked_viz` | top-5 视点 + 拉黑区域可视化 |
| pub | `/exploration_complete`（Bool, latched） | 收敛信号 |

线程：`MultiThreadedExecutor(2)`。`octomapCallback` 与 `resultCallback` 可能并发 → 共享状态用 `blacklist_mutex_` / `cost_probes_mutex_` / `hppt_log_mutex_` 保护。

---

## 1. 任务生命周期与状态机

### 关键状态
- `latest_env_version_`（optional int64）：下游**已接受**的最新 octomap 版本（纳秒时间戳）。没有它就不跑任何 cycle。
- `pending_task_id_` / `pending_goal_` / `pending_task_sent_at_` / `pending_task_env_version_` / `pending_seen_feedback_`：当前**已发布、等结果**的 execute 任务。
- `cost_probes_`（map<id, CostProbe>）：在飞的 **plan-only 探针**（与 pending 任务分开追踪）。
- `has_last_goal_` / `last_goal_`：上一个发布的 goal（去重用）。
- `last_selection_attempt_at_` / `duplicate_goal_pause_until_`：选点节流。
- 黑名单：`blacklist_`（永久/regular）、`sticky_blacklist_`（临时）、`visited_`、`visit_history_`。

### env_status 门控（`envStatusCallback`）
只有在 `accepted=true` **且** `payload_bytes>0`（非空图）时才把 `latest_env_version_` 前移。忽略 initial_scan 启动时的 0 字节握手环境（否则会对着空图乱发探针）。

### 结果分流（`resultCallback`）
按 task_id 前缀分两条路：
1. **`cost_probe_*`**（plan-only 探针）：把 `cost = success ? planning_time+execution_time : max_cost` 写回 `cost_probes_` 表（`runPlanOnlyProbe` 的等待循环靠它解阻塞）。若失败，按 `error_type` 走**分级拉黑**（见 §6）。
2. **`hppt_view_*`**（execute 任务，匹配 `pending_task_id_`）：
   - 记录 stale-plan 诊断（env 从探针到执行漂移了多少秒）。
   - **失败** → 按 error_type 分级拉黑（仅 `hard_unreachable` 永久，其余至少临时；见 §6），`has_last_goal_=false`。
   - **成功** → 把 goal 记入 `visited_`（`visited_duration_sec=45s`）+ `bump_visit_history`，`has_last_goal_=false`。
   - 清空 pending 状态。

### `readyForNewTask()` / `readyForSelection()`
`octomapCallback` 进入实际工作前的两道闸：
- `readyForNewTask()`：无 pending 任务；或 pending 超时（`task_timeout_sec=300s`，或无 feedback 的 dispatch 超时）→ 放弃 pending，允许新任务。
- `readyForSelection()`：`now >= duplicate_goal_pause_until_` 且距上次尝试 `>= selection_interval_sec`。

---

## 2. 主决策周期 `octomapCallback`（STAGE 0–6）

```
每帧 OctoMap:
  闸: msg.binary? / 有 latest_env_version_? / readyForNewTask()? / readyForSelection()?  否→return
  STAGE 0: 解析 octomap→tree; TF 读 current_position_(EE 位)
           CONVERGENCE: 算 octomap 增长(tree.size delta); 增长→重置 consecutive_empty_
           BL-DIAG: 记录黑名单快照 + 刷新拉黑可视化
  STAGE 1: extractFreeUnknownBoundary(tree) → boundaries(free/unknown 边界体素+平均 unknown 方向)
           空→ handleUnproductiveCycle("STAGE 1 EMPTY"); return
  STAGE 2: 逐体素过滤(见 §5); 空→ handleUnproductiveCycle("STAGE 2 EMPTY"); return
  STAGE 3: 聚类→ seeds(direction-first, 无 top-K 预选; 见 §3); 空→ handleUnproductiveCycle; return
  STAGE 4: generateViewpointCandidates(seeds) → candidates(见 §3)
           空→ releaseSticky + handleUnproductiveCycle("STAGE 4 EMPTY"); return
  STAGE 5: 对每个候选 scoreViewpoint→raw_score; 按 raw 降序; 记录排名表 + 刷 top-5 可视化
  STAGE 6: 选择 + 编排(见 §4)
```

---

## 3. Frontier → seed → 视点候选

### STAGE 1 提取（`extractFreeUnknownBoundary`）
遍历 free 叶子，若有 unknown 面邻居 → 是 frontier。**累加所有 unknown 面方向再平均**得到 `unknown_neighbor`（一个贴多面 unknown 的角落 frontier 会朝"unknown 集合中心"看，而非某根轴）。

### STAGE 3 聚类 → seed（direction-first）
- `clusterFrontier`：粗网格 super-voxel + 26-邻接 BFS 连通分量；每 cluster 有 centroid、`mean_unknown_dir`（fine_count 加权平均）、fine_count。
- `cluster_min_voxels` 过滤碎 cluster。
- **centroid 层再过一遍**：对每 cluster 用 `resolveViewpoint(centroid, mean_dir, adaptive_standoff)` 得 goal，检查 `isOutOfWorkspace / isBlacklisted / isPermanentlyBlocked / isVisited / isInwardLook / isLineOfSightBlocked`（这里 LOS 用宽松版做粗筛）。
- **不做 weight top-K 预选**：所有存活 cluster 都成为 seed；仅 `max_frontier_seeds=50` 做安全上限（超限按 fine_count 降序保留最大的，并打日志，不静默丢）。
- `FrontierSeed = {centroid, ud(unknown 主方向), standoff(自适应), fine_count}`。
- 自适应 standoff = `clamp( sqrt(fine_count)·res / (2·tan(fov/2)), standoff_min, standoff_max )`（刚好把整块 frontier 面纳入 FOV）。
- fallback（无 cluster / 聚类关）：`uniformSample(boundaries, boundary_samples=5)`。

### `resolveViewpoint(frontier, unknown_neighbor, standoff)` → (goal, look_at)
- `ud = normalize(unknown_neighbor − frontier)`；退化则 `base→frontier`，再退化 `+X`。
- `goal = frontier − ud·standoff`（相机站在 free 一侧后退 standoff），`goal.z` clamp 到 `[effective_min_z, z_max]`（frontier 越低 effective_min_z 越贴 low 阈值）。
- `look_at = frontier + ud·(standoff+1)`（光轴指向 unknown 后方约 1m）。
- 若 `observe_unreachable` 且 goal 超 reach → `projectToReachable`（投影回 `projection_scale·reach_max` 球面，z-preserving）。

### STAGE 4 视点生成（`generateViewpointCandidates`）
对每个 seed：
1. 主方向 `ud0` + **锥环方位密采样**：对每个半锥角 θ∈`viewpoint_fan_angles_deg={15,30,45,60}`，在环上取 `viewpoint_fan_azimuths=8` 个方位 → `cosθ·ud0 + sinθ·(cosφ·right + sinφ·up)`。上限 `viewpoint_max_directions=33`（1 主 + 4×8）。
   - *（本会话把原"只取上下左右 4 个"改成整圈方位密采样，为了采到"沿轴正对管口/深腔开口"的窄窗口。）*
2. 对每条方向 ud：
   - `goal = F − ud·standoff`；z clamp。
   - **几何可达**：`r=|goal−base|`；`r>reach_max` 且不 observe_unreachable → 拒(unreach)；否则超 band → `projectToReachable`。
   - `isOutOfWorkspace || isForbidden` → 拒(forbid)。
   - `look_at = F + ud·(standoff+1)`；`isInwardLook` → 拒(inward)。
   - **`!losClear(goal, F)` → 拒(LOS_blocked)**。*（本会话统一为严格 `losClear`，和打分同一套，遮挡方向在此剔除，不再漏进打分形成退化循环。）*
   - `isBlacklisted || isPermanentlyBlocked` → 拒(blacklist)。
   - `isVisited` → 拒(visited)。
   - 与已接受候选相机位 `< viewpoint_dedup_tol=0.08` → 拒(dup)。
   - 存活 → `ViewpointCandidate{goal, look_at, frontier, unknown_neighbor, is_surrogate}`。

---

## 4. STAGE 5 打分 + STAGE 6 选择/编排

### 打分（`scoreViewpoint`，§7 详述公式）
每候选 → `raw_score`（未归一化加权和）+ `ScoreBreakdown`。按 `raw_score` 降序。

### STAGE 6 选择 + motion-plan 编排
```
low_ig_gated=false; published=false
for vc in candidates(降序):
    # ★ MIN-IG 门槛: 候选已按 raw 降序, 最优/剩余最高分 < 阈值 → 无有效视点
    if vc.raw_score < min_publish_raw_score(0.05):  low_ig_gated=true; break
    if tried >= motion_planning_top_n(5): break
    if isGoalBad(vc.goal, vc.look_at):  # 出工作空间/已黑名单/向内看
        blacklistPoint(vc.goal, 0.06, "geometry","goal_bad", permanent=true); continue
    if is_dup(vc.goal, last_goal, goal_dedup_tol=0.02): continue
    ++tried
    if not runPlanOnlyProbe(vc.goal, vc.look_at):   # plan-only MoveIt 探针
        continue        # 拉黑由 resultCallback 依 error_type 处理, 这里只跳过
    published = publishExecuteTask(vc.goal, vc.look_at, is_surrogate)
    if published: consecutive_empty_=0; converged_announced_=false   # 有进展→重置收敛
    break
if not published:
    releaseStickyBlacklist(stage)      # 逃生: 清空 sticky, 下轮重试
    handleUnproductiveCycle(stage)     # 计入收敛(STAGE 6 EMPTY 或 LOW-IG)
```

- `runPlanOnlyProbe`：发 `cost_probe_mp_*` plan-only 任务，**阻塞**等 `cost_probes_` 被 resultCallback 写回（超时 `cost_probe_timeout_sec=90s`）；`cost < max` = 成功。`use_planning_cost=false`（默认）→ 直接返回 true（无 MoveIt 时信任打分、送 top-1）。
- `publishExecuteTask`：发 `hppt_view_*`（plan_only=false）任务，设 pending 状态 + goal marker。
- **注意**：选择主键是 **viewpoint 分数 raw**，不是 motion cost；motion cost 只作"能不能规划到"的二值 gate。欧氏距离 motion_cost 仅存在于**已弃用的老 HPP-PT/GREEDY 路径**。

---

## 5. 过滤器 / 几何判据

STAGE 2 逐体素（用 `resolveViewpoint` 的 goal 判）+ STAGE 3 centroid 层 + STAGE 4 逐方向，都复用这套：

| 判据 | 含义 |
|---|---|
| `isTooFar(p)` | `dist(p,base) > frontier_max_range_factor(1.5)·reach_max(0.85)=1.275m` → 丢 |
| `isUnreachable(p)` | `dist > reach_max(0.85)`；`observe_unreachable=true` 时保留(投影观察) |
| `isForbidden / isOutOfWorkspace` | reach band `[reach_min0.60, reach_max0.85]` 外 / z∉`[z_min0.05, z_max0.85]` |
| `isInwardLook(goal,look_at)` | 光轴反向指基座（`require_outward_look`，默认 false 只挡极端） |
| `isBlacklisted(p)` | 在 `blacklist_`(regular) 或 `sticky_blacklist_` 半径内 |
| `isPermanentlyBlocked(p)` | `visit_history` 中同点访问次数 ≥ `permanent_block_count`（默认 0=关） |
| `isVisited(p)` | `visited_` 中 `< visited_radius(0.15)` 且未过期(`visited_duration_sec=45s`) |
| LOS | `losClear(goal, frontier)`：相机→frontier 射线撞 known-occupied 即 blocked（严格） |
| `isGoalBad(p,look_at)` | `isOutOfWorkspace || isBlacklisted`（+ inward 变体） |

---

## 6. 黑名单系统（拉黑的是**机械臂落点 goal**，不是 frontier）

### 两张表
- **`blacklist_`（regular = 永久-within-session）**：本 session 从不释放（连 `releaseStickyBlacklist` 也不动）。
- **`sticky_blacklist_`（临时）**：`releaseStickyBlacklist` 一次清空全部；触发点 = STAGE 4 EMPTY / STAGE 6 EMPTY/LOW-IG（"卡到没候选，放开重试"的逃生舱）。
- 唯一写入口 `blacklistPoint(pos, radius, type, reason, permanent)`：`permanent→blacklist_`，否则 `→sticky_blacklist_`；半径统一用 `block_radius_small=0.06m`（只封落点附近一小块，不覆盖整簇）。
- `isBlacklisted` 查两张表；`permanent_block`（visit-count，默认关）是独立机制。

### 按 motion planner 的 error_type 分级（`classifyBlock` / `blacklistByErrorType`）
| error_type | 等级 |
|---|---|
| `hard_unreachable`（保守失败 + 非保守也 IK/碰撞硬失败） | **永久** |
| `blocked_by_unknown` / `nonconservative_timeout` / `nonconservative_probe_failed` / `trajectory_not_known_free` / `execution_failure` / `moveit_planning_failure` | 临时 |
| 基础设施 retryable（env 版本不匹配 / move_group 超时/不可用） | 不拉黑（下轮重试） |

- **cost_probe 分支**：None→不拉；否则按等级。
- **execute 分支**：失败一律**至少临时**拉一小块（保留"防同一 goal 反复选中→反复失败"的 P0-A 死循环保护），仅 `hard_unreachable` 升永久。
- 诊断日志：`BLACKLIST-ADD [type|reason|PERMANENT/temporary] -> now regular=/sticky=`、`BL-REMOVE [trigger=...]`（逐条移除）、`BL-SNAPSHOT`（每 cycle 全量）、`BL-VIZ`（viz vs actual 对照）。

---

## 7. 打分（"概率"）计算

```
raw = 0.7·obs_density        # prob_w_obs=0.0  → 已去除(书架过拟合)
    + 0.2·unk_density        # prob_w_unk=0.2  (视线盲, 候选删减)
    + 0.7·occlusion_score    # prob_w_occl=0.0 → 已去除
    + 0.5·semantic_relevance # prob_w_sem=0.5  (语义节点关时恒 0)
    + 1.0·(ig + 0·hit_ratio) # prob_w_ig=1.0   (可见性感知, 主导)
    + 1.0·peek_score         # prob_w_peek=0.0 → 已去除
    + 0.1·attraction         # prob_w_attract=0.0 → 已去除
# 现行有效: raw = 0.2·unk + 0.5·sem + 1.0·ig
门控:
  if ig < 0.02:      raw=0   # IG-gate: 只看 ig(本会话去掉 attraction 豁免)
  if not los_clear:  raw=0   # LOS-gate
  raw = max(raw,0)
```

- **各分项**：`obs/unk_density`=frontier 邻域(`prob_density_radius=0.15`)内 occupied/unknown 比例；`occlusion_score`=frontier 背/侧被墙包围度；`semantic_relevance`=语义点距离加权(见语义模块文档)；`ig`=从(goal,look_at)视锥打 `ig_num_rays=64` 条射线(FOV 70°, range 2.0m)看到的 unknown 比例；`peek`=blocked 射线透 unknown 命中墙的贡献；`attraction`=directional，视线擦过已知物边缘的 band-pass。
- **归一化说明**：`scoreViewpoint` 末尾把返回值 clamp 到 [0,0.999)，但那是**老 HPP-PT 图搜索**用的；**direction-first 管线丢弃返回值、全程用未归一化的 `raw`**。所以实际**没有概率归一化**，`raw` 是加权和(可 >1)。`min_publish_raw_score=0.05` 等阈值都在此原始尺度上。

---

## 8. 收敛自动停机

- `handleUnproductiveCycle(stage)`：在每个 EMPTY 出口(STAGE 1/2/3/4/6 EMPTY + LOW-IG)调用。若本 cycle octomap **有实质增长**(`tree.size` 增量 ≥ `convergence_growth_leaves=100`) → 重置计数；否则 `++consecutive_empty_`。
- 达 `convergence_empty_cycles=20` → `declareConverged`：打印 `exploration converged`、发 latched `/exploration_complete`、若 `convergence_shutdown=true` 调 `rclcpp::shutdown()`。
- 发布目标(有进展)或地图增长时重置计数；`pending` 存在的 cycle 在 `readyForNewTask` 早退、不计数。
- 唯一由内部触发的停机路径（否则节点只响应外部 SIGINT）。

---

## 9. 可视化（`/hppt_ranked_viz`）

- **top-5 视点**：`publishRankedViewpointMarkers`——SPHERE(相机位)+ ARROW(视线)。**#0 最高分=绿(大)**，#1-4=青。固定 id 0-4，不足发 DELETE。
- **拉黑区域**：`publishBlacklistMarkers`——SPHERE(直径=2×半径)。**永久=红 / 临时=橙**，半透明。id 连续，数量减少发 DELETE。
- 全部 `lifetime=0`（不更新则永久留存，同 id 覆盖）。`enable_ranked_viz=true`。`exploration_view.rviz` 有默认启用的 "HPPT Ranked" display。

---

## 10. 参数速查（现行默认）

| 类别 | 参数=默认 |
|---|---|
| 工作空间 | `reach_min=0.60` `reach_max=0.85` `z_min=0.05` `z_max=0.85` `base_x/y/z=0` |
| frontier | `frontier_max_range_factor=1.5` `observe_unreachable=true` `projection_scale=0.82` `coarse_grid_factor` `cluster_min_voxels` `boundary_samples=5` `max_frontier_seeds=50` |
| 视点生成 | `viewpoint_fan_angles_deg={15,30,45,60}` `viewpoint_fan_azimuths=8` `viewpoint_max_directions=33` `viewpoint_dedup_tol=0.08` `enable_los_check=true` `los_min_hit_dist=0.10`(仅粗筛用) 观察 standoff/min_z 系列 |
| 打分 | `prob_w_obs=0` `prob_w_unk=0.2` `prob_w_occl=0` `prob_w_sem=0.5` `prob_w_ig=1.0` `prob_w_peek=0` `prob_w_attract=0` `ig_fov_deg=70` `ig_max_range_m=2.0` `ig_num_rays=64` IG-gate `ig<0.02` |
| 选择/编排 | `motion_planning_top_n=5` `min_publish_raw_score=0.05` `use_planning_cost=false` `cost_probe_timeout_sec=90` `cost_probe_max_cost=1e6` moveit_* 容差 |
| 黑名单 | `block_radius_small=0.06` `blacklist_radius=0.20` `visited_radius=0.15` `visited_duration_sec=45` `permanent_block_count=0` `goal_dedup_tol=0.02` |
| 收敛 | `enable_convergence_detection=true` `convergence_empty_cycles=20` `convergence_growth_leaves=100` `convergence_shutdown=true` `exploration_complete_topic=/exploration_complete` |
| 节流 | `selection_interval_sec` `task_timeout_sec=300` `duplicate_goal_cooldown_sec` |
| 可视化/日志 | `enable_ranked_viz=true` `hppt_log_file`(默认 `$HOME/.ros/hppt_decision.log`) |

---

## 11. 死代码 / 已知过拟合（本会话标注）

- **老 HPP-PT 图搜索路径**（`RunAstarHPPPT` / greedy-argmin / `start_to_cand_costs` 欧氏 cost / `cost_probe_max_cost` MAX-edge / clamp-to-0.999 "prob"）：direction-first 重构后基本 dead，只 `runPlanOnlyProbe` 复用了 cost_probe 表做二值 gate。
- `hppt_cost_attract_boost=3.0`、`enable_informed_fps` + `farthestPointSample/informedFarthestPointSample`：**无调用**（老 FPS/cost-discount 遗留）。
- 硬编码 **SHELF-ADJACENT 诊断 AABB**（`x[0.80,1.19] y[-0.42,0.42] z[0,1.11]`，两处）：**仅日志**，但写死了 shelf SDF 几何，在 complex/真机里无意义。
- `unk_density`（0.2）：视线盲，和 ig 冗余（代码注释自称病态）。
- `computeAttractiveness` / directional attraction：权重已 0，实际不参与。

---

## 12. 本会话对逻辑的关键改动（相对上次 push tsp_real_0723）

1. **视点选取重构为 direction-first**：generate-then-score，扇形/锥环方向展开，几何可达 + 严格 LOS 剪枝，top-N 第一个规划成功即执行（去掉 weight top-K 预选）。
2. **决策日志**：SCORED DIRECTIONS 排名表 + BLACKLIST-ADD 分级记录 + BL-SNAPSHOT/REMOVE/VIZ 诊断。
3. **非保守兜底 + 分级拉黑**（motion planner 侧 2-pass + env 快照修复；HPPT 侧按 error_type 分级、小半径 0.06）。
4. **收敛自动停机**（连续 20 空 cycle + 地图稳定 → shutdown + /exploration_complete）。
5. **MIN-IG 门槛**（raw<0.05 不发布、计入收敛）。
6. **选点/拉黑 RViz 可视化**（/hppt_ranked_viz，绿/青/红/橙，常驻）。
7. **LOS 统一**（候选生成改用严格 losClear）+ **密集扇形**（4 锥角×8 方位）解决管状/深腔视点漏采。
8. **评分去过拟合**：`prob_w_ig 0.5→1.0`、`prob_w_attract→0`、IG-gate 只看 ig、去掉 obs/occl/peek/attract 四项。
