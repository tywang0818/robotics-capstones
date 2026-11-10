#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/joint_state.hpp>          // ★ baseline: IK 种子/当前关节
#include <moveit_msgs/srv/get_position_ik.hpp>      // ★ baseline: /compute_ik
#include <moveit_msgs/srv/get_state_validity.hpp>   // ★ baseline: /check_state_validity
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>   // ★ CONVERGENCE: /exploration_complete 信号
#include <atomic>                  // ★ PAUSE: paused_ 标志线程安全
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <fstream>       // ★ LOGGER: decision log 落盘
#include <cstdlib>       // ★ LOGGER: getenv("HOME")
#include <iomanip>
#include <cmath>
#include <memory>
#include <cstdint>
#include <optional>
#include <algorithm>
#include <mutex>
#include <limits>       // ★ std::numeric_limits (bump_visit_history)
#include <thread>       // ★ sleep_for (cost probe wait loop)
#include <chrono>       // ★ ms 时长
#include <map>          // ★ cost_probes_ 用 map
#include <unordered_map>  // ★ Stage A: frontier 聚类的体素 hash
#include <tuple>          // ★ Stage A: 体素索引 key
#include <array>          // ★ baseline: group 关节名固定数组

#include "search_hpp_pt.hpp"
#include "hppt_stage_algos.hpp"   // ★ Stage A/B 算法的纯函数实现（与 tests 共用）
#include "hppt_baseline.hpp"      // ★ new_method: Greedy Baseline 纯算法层

// ========== 边界点结构 ==========
// BoundaryPoint / BoundaryCluster 定义在 hppt_stage_algos.hpp，这里 using 使原调用点不变
using BoundaryPoint    = hppt_stage::BoundaryPoint;
using BoundaryCluster  = hppt_stage::BoundaryCluster;
using hppt_stage::clusterFrontier;
using hppt_stage::farthestPointSample;
using hppt_stage::informedFarthestPointSample;

struct SemanticPointWorld {
    Eigen::Vector3d position;
    double relevance_prob;
};

struct SemanticVoxelKey {
    int x;
    int y;
    int z;

    bool operator==(const SemanticVoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SemanticVoxelKeyHash {
    std::size_t operator()(const SemanticVoxelKey& key) const {
        const std::size_t hx = std::hash<int>{}(key.x);
        const std::size_t hy = std::hash<int>{}(key.y);
        const std::size_t hz = std::hash<int>{}(key.z);
        return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2))
                  ^ (hz + 0x9e3779b9 + (hy << 6) + (hy >> 2));
    }
};

struct SemanticVoxelEvidence {
    Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
    double relevance_sum = 0.0;
    double max_relevance = 0.0;
    uint32_t count = 0;
    double last_seen_sec = 0.0;
};

std::vector<BoundaryPoint> extractFreeUnknownBoundary(octomap::OcTree& tree) {
    std::vector<BoundaryPoint> boundaries;
    double res = tree.getResolution();
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (it->getOccupancy() >= 0.5) continue;
        octomap::point3d center = it.getCoordinate();
        double size = it.getSize();
        double half = size / 2.0 + res * 0.5;
        std::vector<octomap::point3d> dirs = {
            { static_cast<float>(half), 0.0f, 0.0f},
            { static_cast<float>(-half), 0.0f, 0.0f},
            { 0.0f, static_cast<float>(half), 0.0f},
            { 0.0f, static_cast<float>(-half), 0.0f},
            { 0.0f, 0.0f, static_cast<float>(half)},
            { 0.0f, 0.0f, static_cast<float>(-half)}
        };
        // ★ P0-1: 累加 **所有** unknown 邻居方向再平均。
        // 一个 frontier 若同时贴 2-3 个 unknown 面（比如货架顶角），
        // 之前只取第一个方向会让 look_at 偏向某个轴，现在朝真正的
        // "unknown 集合的中心"看。
        octomap::point3d unknown_sum(0.0f, 0.0f, 0.0f);
        int n_unknown = 0;
        for (const auto& d : dirs) {
            octomap::point3d neighbor = center + d;
            if (tree.search(neighbor) == nullptr) {
                unknown_sum += d;   // d 已是 (±half, 0, 0) 之类的向量
                ++n_unknown;
            }
        }
        if (n_unknown > 0) {
            // 平均方向 = sum / n；再乘 half 让长度跟 "一格 offset" 同量级。
            // resolveViewpoint 会做 normalize(unknown_neighbor - coord)，所以
            // 只要方向对，长度不重要。若所有方向抵消（如 ±x 都 unknown），
            // 长度为 0，resolveViewpoint 会走 base→coord 的兜底。
            unknown_sum *= (1.0f / static_cast<float>(n_unknown));
            octomap::point3d unknown_coord = center + unknown_sum;
            boundaries.push_back({center, 0.0, unknown_coord, -1.0});
        }
    }
    return boundaries;
}

std::vector<BoundaryPoint> uniformSample(const std::vector<BoundaryPoint>& all, int k) {
    if (all.size() <= static_cast<size_t>(k)) return all;
    std::vector<BoundaryPoint> sampled;
    sampled.reserve(k);
    double step = static_cast<double>(all.size()) / k;
    for (int i = 0; i < k; ++i)
        sampled.push_back(all[static_cast<int>(i * step)]);
    return sampled;
}

// ========== 极简 JSON 解析 ==========
static std::optional<bool> jsonGetBool(const std::string& s, const std::string& key) {
    auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return std::nullopt;
    auto colon = s.find(':', k);
    if (colon == std::string::npos) return std::nullopt;
    size_t i = colon + 1;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    if (s.compare(i, 4, "true") == 0)  return true;
    if (s.compare(i, 5, "false") == 0) return false;
    return std::nullopt;
}

static std::optional<int64_t> jsonGetInt64(const std::string& s, const std::string& key) {
    auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return std::nullopt;
    auto colon = s.find(':', k);
    if (colon == std::string::npos) return std::nullopt;
    size_t i = colon + 1;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    size_t j = i;
    if (j < s.size() && (s[j] == '+' || s[j] == '-')) ++j;
    while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j]))) ++j;
    if (j == i) return std::nullopt;
    try {
        return static_cast<int64_t>(std::stoll(s.substr(i, j - i)));
    } catch (...) { return std::nullopt; }
}

static std::optional<double> jsonGetDouble(const std::string& s, const std::string& key) {
    auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return std::nullopt;
    auto colon = s.find(':', k);
    if (colon == std::string::npos) return std::nullopt;
    size_t i = colon + 1;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    try {
        size_t parsed = 0;
        double value = std::stod(s.substr(i), &parsed);
        if (parsed == 0) return std::nullopt;
        return value;
    } catch (...) { return std::nullopt; }
}

static std::optional<std::string> jsonGetString(const std::string& s, const std::string& key) {
    auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return std::nullopt;
    auto colon = s.find(':', k);
    if (colon == std::string::npos) return std::nullopt;
    auto q1 = s.find('"', colon + 1);
    if (q1 == std::string::npos) return std::nullopt;
    auto q2 = s.find('"', q1 + 1);
    if (q2 == std::string::npos) return std::nullopt;
    return s.substr(q1 + 1, q2 - q1 - 1);
}

// ========== JSON 生成 ==========
std::string makeTaskJson(const std::string& task_id,
                         int64_t environment_version,
                         const Eigen::Vector3d& position,
                         const Eigen::Vector3d& look_at,
                         const std::string& frame_id,
                         bool plan_only,
                         double allowed_planning_time,
                         int num_planning_attempts,
                         double position_tolerance,
                         double orientation_tolerance) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{\n"
        << "  \"task_id\": \"" << task_id << "\",\n"
        << "  \"environment_version\": " << environment_version << ",\n"
        << "  \"goal\": {\n"
        << "    \"type\": \"viewpoint\",\n"
        << "    \"frame_id\": \"" << frame_id << "\",\n"
        << "    \"position\": [" << position.x() << ", " << position.y() << ", " << position.z() << "],\n"
        << "    \"look_at\": [" << look_at.x() << ", " << look_at.y() << ", " << look_at.z() << "]\n"
        << "  },\n"
        << "  \"plan_only\": " << (plan_only ? "true" : "false") << ",\n"
        << "  \"allowed_planning_time\": " << allowed_planning_time << ",\n"
        << "  \"num_planning_attempts\": " << num_planning_attempts << ",\n"
        << "  \"position_tolerance\": " << position_tolerance << ",\n"
        << "  \"orientation_tolerance\": " << orientation_tolerance << "\n"
        << "}";
    return oss.str();
}

// ========== ROS2 节点 ==========
class HPPTBoundaryNode : public rclcpp::Node {
public:
    HPPTBoundaryNode() : Node("hppt_boundary_node") {
        world_frame_   = this->declare_parameter<std::string>("world_frame", "world");
        robot_frame_   = this->declare_parameter<std::string>("robot_frame", "tool0");
        octomap_topic_ = this->declare_parameter<std::string>(
            "octomap_topic", "/octomap_binary");
        env_status_topic_ = this->declare_parameter<std::string>(
            "env_status_topic", "/team/environment_status_json");
        task_topic_ = this->declare_parameter<std::string>(
            "task_topic", "/team/viewpoint_task_json");
        feedback_topic_ = this->declare_parameter<std::string>(
            "feedback_topic", "/team/motion_feedback_json");
        result_topic_ = this->declare_parameter<std::string>(
            "result_topic", "/team/motion_result_json");
        semantic_topic_ = this->declare_parameter<std::string>(
            "semantic_topic", "/semantic_frontier/semantic_points");
        // ★ REVERTED to "100% success rate" config from the earlier
        //   working test: K=5 primary + 2 alt each = 15 probes serial.
        //   ~4s * 15 = 60s STAGE 4 + 5s confirm + 20s execute ≈ 90-120s
        //   cycle, but stale-plan issues are handled by STAGE 5.5 confirm.
        boundary_samples_ = this->declare_parameter<int>("boundary_samples", 5);

        // ★★★ DIRECTION-FIRST 流水线参数（新方案）
        //   旧流程: 每个 frontier 只算一个 (goal,look_at) → 打分 → 事后
        //     generateAlternativeViewpoints 补救 IK。
        //   新流程: 每个 frontier 先绕 unknown 方向扇形展开 N 个 look_at 方向,
        //     沿每条方向/延长线用几何法(reach 球面+workspace+碰撞 raycast)
        //     找可达相机位,撞障碍/不可达则拒绝该方向,再对所有存活方向打分,
        //     取 top-N 依次送 motion planning,第一个规划成功即执行。
        //   viewpoint_fan_angles_deg: 相对 unknown 主方向的半锥角(度)。每个角
        //     构成一个锥环, 在环上按 viewpoint_fan_azimuths 个方位角均匀取方向
        //     (而非只取上/下/左/右 4 个), 加密+加宽以采到"沿轴正对开口"这类
        //     窄窗口(管状/深腔结构)。默认 {15,30,45,60}。
        viewpoint_fan_angles_deg_ = this->declare_parameter<std::vector<double>>(
            "viewpoint_fan_angles_deg",
            std::vector<double>{15.0, 30.0, 45.0, 60.0});
        //   每个锥环上均匀采样的方位角数(默认 8, 即每 45° 一个方向)。
        viewpoint_fan_azimuths_ = this->declare_parameter<int>(
            "viewpoint_fan_azimuths", 8);
        //   每个 frontier 最多保留多少条方向(含主方向)。
        //   上限 = 1 + azimuths*len(angles) = 1 + 8*4 = 33。
        viewpoint_max_directions_ = this->declare_parameter<int>(
            "viewpoint_max_directions", 33);
        //   两个候选相机位距离 < 该值视为重复,只留其一。
        viewpoint_dedup_tol_ = this->declare_parameter<double>(
            "viewpoint_dedup_tol", 0.08);
        //   打分后按分数降序取 top-N 依次送 motion planning。
        motion_planning_top_n_ = this->declare_parameter<int>(
            "motion_planning_top_n", 5);
        //   安全上限: 参与扇形展开的 frontier seed 数(跳过 top-K 后防爆炸)。
        //   超过则按 fine_count 降序保留最大的若干个并打印截断日志。
        max_frontier_seeds_ = this->declare_parameter<int>(
            "max_frontier_seeds", 50);

        // ★★★ LOGGER: decision log 落盘路径。空字符串 → 自动落到
        //   $HOME/.ros/hppt_decision.log(HOME 缺失则 /tmp)。无论是否落盘,
        //   排名表与黑名单记录都会镜像到 rclcpp INFO。
        hppt_log_file_ = this->declare_parameter<std::string>(
            "hppt_log_file", "");
        if (hppt_log_file_.empty()) {
            const char* home = std::getenv("HOME");
            hppt_log_file_ = std::string(home ? home : "/tmp")
                             + "/.ros/hppt_decision.log";
        }
        hppt_log_ofs_.open(hppt_log_file_, std::ios::app);
        if (!hppt_log_ofs_.is_open()) {
            // 目录可能不存在 → 退回 /tmp 再试一次
            hppt_log_file_ = "/tmp/hppt_decision.log";
            hppt_log_ofs_.open(hppt_log_file_, std::ios::app);
        }
        hppt_log_to_file_ = hppt_log_ofs_.is_open();
        if (hppt_log_to_file_) {
            RCLCPP_INFO(this->get_logger(),
                "HPPT decision logger writing to: %s", hppt_log_file_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(),
                "HPPT decision logger: could not open '%s'; console-only.",
                hppt_log_file_.c_str());
        }

        planning_time_limit_ = this->declare_parameter<double>("planning_time_limit", 60.0);
        moveit_allowed_planning_time_ = this->declare_parameter<double>(
            "moveit_allowed_planning_time", 5.0);
        moveit_num_planning_attempts_ = this->declare_parameter<int>(
            "moveit_num_planning_attempts", 10);
        // 到达点 vs goal 允许的差距:位置 0.015m(半体素,保证到达体素==目标体素,
        //   即"体素差为0");角度 0.1396rad(8°)。
        moveit_position_tolerance_ = this->declare_parameter<double>(
            "moveit_position_tolerance", 0.015);
        moveit_orientation_tolerance_ = this->declare_parameter<double>(
            "moveit_orientation_tolerance", 0.1396);
        task_timeout_sec_ = this->declare_parameter<double>("task_timeout_sec", 300.0);
        task_dispatch_timeout_sec_ = this->declare_parameter<double>(
            "task_dispatch_timeout_sec", 10.0);
        selection_interval_sec_ = this->declare_parameter<double>(
            "selection_interval_sec", 1.0);
        duplicate_goal_cooldown_sec_ = this->declare_parameter<double>(
            "duplicate_goal_cooldown_sec", 3.0);
        goal_dedup_tol_ = this->declare_parameter<double>("goal_dedup_tol", 0.02);
        blacklist_radius_ = this->declare_parameter<double>("blacklist_radius", 0.20);
        // blacklist_duration_sec removed: regular blacklist entries are
        // now permanent-within-session. See isBlacklisted().
        // ★★★ VISITED-RADIUS 历经 0.12 → 0.30 → 0.15:
        //   0.12 (原) 太小,反复扫同一柜子小区域,coverage 不涨;
        //   0.30 太大,把柜子中层 5-7 个相邻 cluster 一次淘汰,后续 cycle 无候选;
        //   0.15 (当前) 中间:允许探索柜子不同角落,但不至于一片全屠。
        visited_radius_ = this->declare_parameter<double>("visited_radius", 0.15);
        visited_duration_sec_ = this->declare_parameter<double>("visited_duration_sec", 45.0);
        // ★ 永久黑名单：投影面 12cm 直径 = 6cm 半径的区域被成功访问
        // permanent_block_count 次以上后，本次运行永久拉黑。
        // 默认关闭：permanent_block_count=0（任何 count 都不会 >=0 触发，因为检查是 count>0 且 count>=threshold）
        permanent_block_radius_ = this->declare_parameter<double>(
            "permanent_block_radius", 0.06);
        permanent_block_count_ = this->declare_parameter<int>(
            "permanent_block_count", 0);   // ★ 默认 0 = 关闭；置正数才启用

        // ★★★ HPP-PT cost：起点→候选的排序权重
        //   - true  = MoveIt plan-only cost (真实规划+执行时间估计, 慢)
        //   - false = Euclidean 距离 (瞬时, 但没有 IK 可达性预筛)
        //
        // ★ CHANGED to false (2026-07-20): STAGE 4 从 60-90s 掉到 <0.1s,
        //   cycle 时间 100s → 25-40s。丢掉的 IK 预筛信息由 STAGE 5.5
        //   confirm probe 兜底（拒绝就 blacklist + 下一 cycle 再选）。
        //   失败 cycle 也从 100s 缩到 15s，即使 confirm 通过率下降，
        //   总吞吐仍是 3-4× 提升。
        use_planning_cost_ = this->declare_parameter<bool>(
            "use_planning_cost", false);
        // ★ 60s: primary + alt probe pair, K=5 → up to 10 probes serial.
        //   每个 probe ≤ moveit_allowed_planning_time_=5s,10 × 5 + 排队余量 ≈ 60s。
        //   决策 bridge 串行 (max_parallel_probes=1); WAIT_INITIAL_SCAN=true 更稳。
        // ★ REVERTED: K=5 alt=2 → 15 probes × ~4s = 60s；90s 缓冲充分。
        cost_probe_timeout_sec_ = this->declare_parameter<double>(
            "cost_probe_timeout_sec", 90.0);
        cost_probe_max_cost_ = this->declare_parameter<double>(
            "cost_probe_max_cost", 1.0e6);
        // ★ EVENT-DRIVEN:greedy 已改为事件驱动等真实结果,不再有 hppt 侧的
        //   execute 等待超时(删除 execute_wait_timeout_sec)。max_execute_attempts
        //   仅保留为兼容参数(实际 limit = 本 cycle 的 A* reachable 候选数)。
        max_execute_attempts_ = this->declare_parameter<int>(
            "max_execute_attempts", 20);

        // ★ APPROACH D (2026-07-20 stale-plan fix): between STAGE 5 (select)
        //   and STAGE 6 (publish), run one final probe against the CURRENT
        //   env_version to catch stale-plan failures before we commit.
        //   Cost probes are validated against env V1 (from ~90s ago);
        //   env may have grown a lot since.  A ~5s fresh probe closes
        //   most of the drift window.
        enable_confirm_probe_ = this->declare_parameter<bool>(
            "enable_confirm_probe", true);
        confirm_probe_timeout_sec_ = this->declare_parameter<double>(
            "confirm_probe_timeout_sec", 8.0);

        // ★★★ Stage A: frontier 聚类 + 空间最远点采样
        //   - enable_frontier_clustering: false 回落到原 index-uniform 采样，便于对比
        //   - cluster_min_voxels: 小于该体素数的 cluster 视作噪点丢弃（1 = 保留所有）
        enable_frontier_clustering_ = this->declare_parameter<bool>(
            "enable_frontier_clustering", true);
        // ★ FIX B (2026-07-20): 把 cluster 切细，让 raw cluster 从 ~5 提高到
        //   ~15-30，配合 Fix C 的多 alt_viewpoint，让每个 cycle 能真正探索到
        //   多个可达 goal。原值 5 是为了压制小 cluster，但配合 3x 粗网格
        //   ~5 个 raw cluster 太少，probe 池太窄。
        //   2 与 coarse_grid_factor=2 (~4cm super voxel) 配合，等价体积门槛 ≈
        //   2 × (0.04m)³ = 1.3e-4 m³ (~一个玻璃球)。
        cluster_min_voxels_ = this->declare_parameter<int>(
            "cluster_min_voxels", 2);
        // ★ FIX B (2026-07-20): 3 → 2，让 BFS 在更细网格上做，大 blob 会分成
        //   多个小 blob，产生更多独立 centroid。Trade-off：cluster 数增加
        //   ~3-5x，但每 cluster 内部结构分辨率也变好。1 会产生太多小 cluster
        //   （每 cycle 上千），2 是平衡点。
        coarse_grid_factor_ = this->declare_parameter<int>(
            "coarse_grid_factor", 2);

        // Reachable workspace constraints (JAKA A5 nominal reach ~0.857 m).
        // Goal candidates outside this sphere/z-range are dropped before HPP-PT.
        base_x_ = this->declare_parameter<double>("base_x", 0.0);
        base_y_ = this->declare_parameter<double>("base_y", 0.0);
        base_z_ = this->declare_parameter<double>("base_z", 0.0);
        reach_min_ = this->declare_parameter<double>("reach_min", 0.60);
        reach_max_ = this->declare_parameter<double>("reach_max", 0.85);
        // ★ 硬距离截断：dist(frontier, base) > factor * reach_max 的 frontier 直接淘汰，
        //   不再产生 surrogate 观察（远超 reach_max 的 surrogate 是切线视角，无用）。
        //   对固定基座 = 永久黑名单效果（该位置每一 cycle 都被重新过滤掉）。
        //   1.5 = 1.5 × 0.85 ≈ 1.28m 硬上限。保守值，减少远处 SURROGATE 浪费。
        frontier_max_range_factor_ = this->declare_parameter<double>(
            "frontier_max_range_factor", 1.5);
        z_min_ = this->declare_parameter<double>("z_min", 0.05);
        // ★ z_max 1.20 → 0.85m: 匹配 reach_max。太松导致天花板 (z=0.9-1.2m)
        //   的 frontier voxel 全部通过过滤 → 相机被推到 workspace 顶
        //   (z=0.66) 仰视，触发一堆"看天"pick。改到 0.85m 后 arm 够不到
        //   的高处 frontier 直接淘汰;而 0.66-0.85 之间留给 IG-gate 处理
        //   (不可看的斜仰视图 ig 会很低 → 被 igate 剔除)。
        z_max_ = this->declare_parameter<double>("z_max", 0.85);
        observation_standoff_ = this->declare_parameter<double>(
            "observation_standoff", 0.35);
        // ★ P1-1: 按 cluster 尺寸自适应 standoff。
        //   desired = cluster_extent / (2·tan(fov/2))
        //   clamp 到 [min, max]。cluster_extent = sqrt(fine_count) · octomap_res
        observation_standoff_min_ = this->declare_parameter<double>(
            "observation_standoff_min", 0.15);
        observation_standoff_max_ = this->declare_parameter<double>(
            "observation_standoff_max", 0.55);
        observation_min_z_ = this->declare_parameter<double>(
            "observation_min_z", 0.25);
        // ★ P3-3: 低 frontier 时允许更低相机高度（否则 goal.z clamp 到 0.25 → 俯视太陡）
        observation_min_z_low_ = this->declare_parameter<double>(
            "observation_min_z_low", 0.10);
        // ★ P3-1: 相机→look_at 视线穿过 known-occupied → 视线被墙挡，直接淘汰
        enable_los_check_ = this->declare_parameter<bool>(
            "enable_los_check", true);
        // ★★★ FIX-2: LOS 击中点距离 goal >= 此值时不算 blocked。
        //   典型:goal 到柜子后墙 ~0.9m,但也是有效观察方向(能看到 0.9m 的
        //   free/unknown)。默认 0.10m: 只有真的贴脸(<10cm)才判 blocked。
        los_min_hit_dist_ = this->declare_parameter<double>(
            "los_min_hit_dist", 0.10);
        // 观察方向必须有"远离基座"的正分量；否则相机看向基座，
        // 手腕反折，近端 link 会贴到基座 octomap 上撞出 collision。
        // 2026-07-16 默认改 false: 原 true 会把"绕到物体背后向 -X 看"这类
        // 合法 SURROGATE 姿态一并否掉(look_dir · outward 天生为负),导致
        // close_cube back face 永远拿不到候选。真正的自碰撞由 MoveIt IK
        // precheck + collision planner 负责,不需要这个几何硬截断。
        require_outward_look_ = this->declare_parameter<bool>("require_outward_look", false);

        // 观察不可达 frontier：把相机沿 base→frontier 方向投影到
        // projection_scale * reach_max 的球面上，look_at 保留原 frontier 位置。
        // 用于"想看那一边但机械臂够不到"的情况。
        observe_unreachable_ = this->declare_parameter<bool>("observe_unreachable", true);
        projection_scale_ = this->declare_parameter<double>("projection_scale", 0.82);
        // ★★★ Z-PRESERVING PROJECTION: 原 projectToReachable 用纯 radial 缩放
        //   把 frontier 投到 reach 球面 → 高 z frontier(如柜顶 z=0.9)会被
        //   proportional 压到 z=0.5,camera 站低处斜看向柜顶,视野斜。
        //   z-preserving 保留 frontier 的 z(在 reach 允许范围内),只缩 xy,
        //   → camera 站在柜顶高度平视,视野正对目标。
        projection_z_preserve_ = this->declare_parameter<bool>(
            "projection_z_preserve", true);

        // 概率模型：viewpoint 周围球形邻域内
        //   prob = w_obs * (occupied 密度) + w_unk * (unknown 密度)
        // 主要：obstacle 密度大 → 概率高
        // 次要：unknown  密度大 → 概率高
        prob_density_radius_ = this->declare_parameter<double>(
            "prob_density_radius", 0.15);
        // ★★★ 去除(默认 0): obs_density 奖励"frontier 靠近 occupied(障碍物表面)",
        //   是对书架/密集障碍结构的过拟合(强化"贴着货架钻")。设 0 移出评分。
        prob_w_obs_ = this->declare_parameter<double>("prob_w_obs", 0.0);
        prob_w_unk_ = this->declare_parameter<double>("prob_w_unk", 0.2);
        use_semantic_probability_ = this->declare_parameter<bool>(
            "use_semantic_probability", true);
        semantic_radius_ = this->declare_parameter<double>("semantic_radius", 0.20);
        semantic_max_age_sec_ = this->declare_parameter<double>(
            "semantic_max_age_sec", 3.0);
        semantic_fusion_voxel_size_ = this->declare_parameter<double>(
            "semantic_fusion_voxel_size", 0.05);
        semantic_fusion_max_age_sec_ = this->declare_parameter<double>(
            "semantic_fusion_max_age_sec", 0.0);
        prob_w_sem_ = this->declare_parameter<double>("prob_w_sem", 0.5);
        semantic_min_relevance_ = this->declare_parameter<double>(
            "semantic_min_relevance", 0.0);

        // ★★★ Stage B1: 视锥 raycast 信息增益 IG，作为概率的新一项
        //   prob += prob_w_ig * IG_norm
        // IG_norm ∈ [0,1]，1 = 每条射线都走满 max_range 且全程 unknown
        //   - prob_w_ig=0 关闭该项（等价于 Stage A 之前的公式）
        //   - fov 用相机水平视场，num_rays 越多越准但越慢
        prob_w_ig_ = this->declare_parameter<double>("prob_w_ig", 1.0);
        ig_fov_deg_ = this->declare_parameter<double>("ig_fov_deg", 70.0);
        ig_max_range_m_ = this->declare_parameter<double>("ig_max_range_m", 2.0);
        ig_num_rays_ = this->declare_parameter<int>("ig_num_rays", 64);

        // ★★★ TIER-2.4 hit-bonus: IG raycast 击中 known-occupied 的射线比例作为
        //   独立奖励项。原 IG(ignoreUnknown=true)偏向"射线走满全程 unknown"→
        //   奖励看向大片空气；hit-bonus 相反,奖励"看到实体物体"。两者加权和
        //   让候选既覆盖 unknown 又不会飘到空中。
        //   最终 prob += prob_w_ig * (IG_norm + ig_hit_bonus * hit_ratio)
        //   3.0 (原 0.5):hit_ratio 是"看向物体"最强的信号 (柜子候选 hit~0.5,
        //   -X 空气候选 hit~0.05),3.0 让差异被概率评分主导,推柜子进 path[1]。
        // ★★★ TIER-3: 默认改为 0.0。理由 = 当探索目标是"被遮挡的 unknown"
        //   (货架内部/立方体侧面),直接奖励"看到实体墙"会让相机偏向盯正面
        //   遮挡物。改用 peek_score(下)代替:奖励"穿过 unknown 后才碰到墙"。
        ig_hit_bonus_ = this->declare_parameter<double>("ig_hit_bonus", 0.0);

        // ★★★ TIER-3 (B) peek_score: 每条 blocked raycast 贡献
        //   (unknown_length_before_hit / max_range)。0 credit 给未命中的射线,
        //   0 credit 给贴墙短射线,满 credit 给"看 2m unknown 之后打到墙"。
        //   这才是"从缝隙里看进去"的真正信号。
        // ★★★ 去除(默认 0): peek_score 奖励"透过 unknown 命中墙的窥视"——针对
        //   书架/格栅"从缝里看"的过拟合。设 0 移出评分。
        prob_w_peek_ = this->declare_parameter<double>("prob_w_peek", 0.0);

        // ★★★ TIER-3 (A) occlusion_score: frontier 体素"背面 + 侧面"的
        //   occupied 密度(以 d_unk 反向半球统计)。frontier 后方/两侧有墙
        //   → unknown 是"被藏起来"的,值得优先看。开放空间 frontier → 0。
        // ★★★ 去除(默认 0): occlusion_score 奖励"frontier 背/侧被墙包围"——正是
        //   "钻进书架格子/被三面挡的凹腔"的过拟合信号。设 0 移出评分(计算也跳过)。
        prob_w_occl_ = this->declare_parameter<double>("prob_w_occl", 0.0);
        occl_radius_ = this->declare_parameter<double>("occl_radius", 0.15);

        // ★★★ TIER-3 (C) directional attraction: 只算 look_at 附近的
        //   occupied 且落在相机→look_at 视线的 lateral_tol 半径内。取代
        //   老"以相机为圆心"attraction 的方向性缺失。开关关掉可回落到旧公式。
        directional_attract_ = this->declare_parameter<bool>(
            "directional_attract", true);
        attract_lateral_tol_ = this->declare_parameter<double>(
            "attract_lateral_tol", 0.10);

        // ★★★ TIER-2.5 attraction: 候选周围球邻域内 "最近 known-occupied 体素"
        //   的距离衰减,鼓励候选站在物体附近。物体外壁 5cm → 得高分;远离物体
        //   50cm → 得低分。加入 prob = ... + prob_w_attract * attraction。
        //   attract_search_radius 定义搜索范围;超出该范围就当"没有物体"→ 0 分。
        // ★★★ 去除(默认 0): attraction(视线擦过已知物体边缘)是"擦着货架边缘
        //   窥视"的过拟合、且与信息增益脱钩(曾占低-ig 高分候选 RAW 的 ~53%)。
        //   设 0 移出评分(计算也跳过)。
        prob_w_attract_ = this->declare_parameter<double>("prob_w_attract", 0.0);
        // ★★★ FIX-3: 0.30 → 0.50。柜子这类"开口面"结构,frontier centroid 到最近
        //   已知墙(左右侧壁,距开口 42cm)>0.30 → attract 拿不到分。放大到 0.50m
        //   保证柜子候选能感知侧壁并获得 attraction 加分。
        attract_search_radius_ = this->declare_parameter<double>(
            "attract_search_radius", 0.50);
        attract_decay_ = this->declare_parameter<double>("attract_decay", 0.15);

        // ★★★ TIER-3.7 informed FPS: 用 cluster.fine_count * attract_factor 作为
        //   FPS 权重,取代纯几何 min_dist 排序。开关默认打开;关掉回落到原 FPS。
        enable_informed_fps_ = this->declare_parameter<bool>(
            "enable_informed_fps", true);

        // ★★★ TIER-1.3 fast-reject cluster blacklist: 当结果返回 success=false
        //   且 planning_time < fast_reject_planning_time_ 时,几乎肯定是 IK/state
        //   validity 秒拒(而不是真的规划失败),用更大半径 fast_reject_radius_
        //   一次把该 cluster 覆盖,避免"选中→秒拒→选邻居→秒拒"的循环。
        fast_reject_planning_time_ = this->declare_parameter<double>(
            "fast_reject_planning_time", 0.05);
        fast_reject_radius_ = this->declare_parameter<double>(
            "fast_reject_radius", 0.15);
        // ★★★ NON-CONSERVATIVE FALLBACK: 所有基于规划结果的拉黑都用这个"一小块"
        //   半径(默认 0.06m ≈ 单体素级),只封失败点本身附近,绝不覆盖整个 cluster。
        block_radius_small_ = this->declare_parameter<double>(
            "block_radius_small", 0.06);

        // ★★★ ALT-PULLBACK: 高概率 frontier 的 primary IK 失败时,把 goal 沿
        //   base→goal 反向拉近 alt_pullback_dist(默认 20cm),look_at 保持不变
        //   → 相机从更"卷起来"的姿态看同一 unknown 区域,IK 通常更松。
        //   alt_min_probability 过滤:probability 低于该值的 candidate 不生成 alt
        //   probe (避免浪费双探带宽在不重要的 candidate 上)。
        alt_pullback_dist_ = this->declare_parameter<double>(
            "alt_pullback_dist", 0.20);
        // ★ FIX C (2026-07-20): 0.4 → 0.0，让所有 candidate 都尝试 alt viewpoint。
        //   之前的门槛是为了省 probe 带宽，但配合 alt_max_count 多 alt 后，即使
        //   低概率 candidate 也可能有一个视角 IK 通过 → 大幅提高 reachable 命中率。
        alt_min_probability_ = this->declare_parameter<double>(
            "alt_min_probability", 0.0);
        // ★ FIX C (2026-07-20): 每个 cluster 最多生成 N 个 alt viewpoint
        //   (pullback + 4 个 perpendicular offset)。之前 generateAlternativeViewpoint
        //   只返回第一个能过 workspace/inward 检查的 alt，其他备胎被丢掉。
        //   现在返回 up to N 个，全部送去 probe，任意一个 IK 通过就用。
        //
        // ★ REVERTED to K=5 alt=2 for high success rate.
        // Default 8 = full sweep of the new angular alt set
        // (4 directions × 2 angles). Override if probe bandwidth is
        // an issue.
        alt_max_count_ = this->declare_parameter<int>("alt_max_count", 8);

        // ★★★ GREEDY-ARGMIN: 绕过 raplab HPP-PT (启发式非 admissible,当多个
        //   candidate prob 都被 clamp 到 0.999 时排序变随机),直接从 K 个
        //   candidate 里按 score = probability / effective_cost 排序,取 top-1。
        //   跳过 bad-goal 和 probe-rejected 候选。这是"path[1]-only" 用例的
        //   直白实现,避免 HPP-PT 的哈密顿路径复杂度和启发式偏差。
        use_greedy_argmin_ = this->declare_parameter<bool>(
            "use_greedy_argmin", true);

        // ★★★ HPP-PT COST-DISCOUNT: 图边权 start→candidate 用 attraction 打折。
        //   eff_cost = raw_cost / (1 + boost × attraction)。
        //   boost=3.0: attraction=1 → cost 变 25%; attraction=0.5 → cost 变 40%。
        //   目的:HPP-PT 图搜索按 expected cost 最小化,柜子候选虽然 raw_cost
        //   高(reach 边界规划慢),但 attraction 高,打折后与外围候选竞争,
        //   让 path[1] 能选到柜子。max_cost (probe fail) 不打折,继续被 skip。
        hppt_cost_attract_boost_ = this->declare_parameter<double>(
            "hppt_cost_attract_boost", 3.0);

        sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, 10,
            std::bind(&HPPTBoundaryNode::octomapCallback, this, std::placeholders::_1));

        auto retained_qos = rclcpp::QoS(1).reliable().transient_local();
        sub_env_status_ = this->create_subscription<std_msgs::msg::String>(
            env_status_topic_, retained_qos,
            std::bind(&HPPTBoundaryNode::envStatusCallback, this, std::placeholders::_1));
        result_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions result_sub_opts;
        result_sub_opts.callback_group = result_cb_group_;
        sub_result_ = this->create_subscription<std_msgs::msg::String>(
            result_topic_, retained_qos,
            std::bind(&HPPTBoundaryNode::resultCallback, this, std::placeholders::_1),
            result_sub_opts);
        sub_feedback_ = this->create_subscription<std_msgs::msg::String>(
            feedback_topic_, 10,
            std::bind(&HPPTBoundaryNode::feedbackCallback, this, std::placeholders::_1));
        // ★ PAUSE: 订阅 /exploration_pause(latched)。true=暂停(当前 execute 跑完后
        //   不再选/派发新任务,机械臂停在原地),false=继续。octomap 由 self_filter 侧
        //   同步暂停(暂停期间丢弃点云 → octomap_server 不更新)。
        // ★ transient_local(latched)订阅:与 server 的 latched pause publisher 匹配,
        //   无论 discovery 早晚都能拿到最新 pause 状态(避免丢消息导致状态不一致)。
        sub_pause_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration_pause", rclcpp::QoS(1).transient_local(),
            std::bind(&HPPTBoundaryNode::pauseCallback, this, std::placeholders::_1));
        sub_semantic_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            semantic_topic_, rclcpp::SensorDataQoS(),
            std::bind(&HPPTBoundaryNode::semanticCallback, this, std::placeholders::_1));

        pub_json_ = this->create_publisher<std_msgs::msg::String>(task_topic_, 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/hppt_boundary/markers", 10);

        // ★★★ CONVERGENCE: 连续 N 次未产出目标的选点周期(且 octomap 无实质增长)
        //   → 判定探索收敛,发 /exploration_complete 信号,并按开关 rclcpp::shutdown()。
        enable_convergence_detection_ = this->declare_parameter<bool>(
            "enable_convergence_detection", true);
        convergence_empty_cycles_ = this->declare_parameter<int>(
            "convergence_empty_cycles", 20);
        // octomap 节点数增量 < 该阈值 → 视为"无实质增长"(地图稳定)。
        convergence_growth_leaves_ = this->declare_parameter<int>(
            "convergence_growth_leaves", 100);
        // true = 收敛时 rclcpp::shutdown() 停机;false = 只发信号, 让上层决定。
        convergence_shutdown_ = this->declare_parameter<bool>(
            "convergence_shutdown", true);
        exploration_complete_topic_ = this->declare_parameter<std::string>(
            "exploration_complete_topic", "/exploration_complete");
        // ★★★ MIN-IG GATE: 本 cycle 最优候选的 raw 分低于此值 → 不发布,
        //   判"未产出有效目标"(计入收敛)。避免相机被派去正对障碍面零价值观察。
        min_publish_raw_score_ = this->declare_parameter<double>(
            "min_publish_raw_score", 0.05);
        // latched(transient_local),晚订阅者也能收到收敛信号。
        pub_exploration_complete_ = this->create_publisher<std_msgs::msg::Bool>(
            exploration_complete_topic_, rclcpp::QoS(1).transient_local());

        // ★★★ RANKED VIZ: 每 cycle top-5 视角(+方向箭头) + 拉黑区域可视化。
        //   最高分=绿、其余=青、永久拉黑=红、临时拉黑=橙;lifetime=0 永久留存,
        //   下一 cycle 覆盖同 id。默认开。RViz 显示 topic /hppt_ranked_viz。
        enable_ranked_viz_ = this->declare_parameter<bool>(
            "enable_ranked_viz", true);
        pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/hppt_ranked_viz", rclcpp::QoS(10));

        // ============================================================
        // ★★★ NEW_METHOD: Greedy Baseline 参数 + IK 服务 + /joint_states
        // ============================================================
        use_greedy_baseline_ = this->declare_parameter<bool>(
            "use_greedy_baseline", true);
        // ★ 沿线采样步长:与聚合 merge_distance(0.15m)对齐,避免生成会被聚合
        //   合并的冗余点;长 free 段多取、短 free 段少取(变步长)。
        viewpoint_line_sample_step_ = this->declare_parameter<double>(
            "viewpoint_line_sample_step", 0.15);
        // ★ 沿线 occupied 截断检测步长:只用于判断视线是否穿过障碍物,
        //   不用于生成候选点。候选仍按 viewpoint_line_sample_step_ 生成。
        viewpoint_line_collision_step_ = this->declare_parameter<double>(
            "viewpoint_line_collision_step", 0.03);
        // ★ 相机到 frontier 的观测距离上限(m):沿采样线选点时,candidate_t(=相机
        //   到 frontier F 的距离)不得超过此值。匹配 octomap sensor max_range(3m)——
        //   超过这个距离相机物理上看不到该 frontier 的 unknown(U 采到但 V/max_range
        //   够不着),会产生"远观够不到的幽灵候选"卡死机械臂。默认 3.0m。
        viewpoint_max_observe_distance_ = this->declare_parameter<double>(
            "viewpoint_max_observe_distance", 3.0);
        // ★ 选点距 free 段两端(unknown/occupied 边界或 reach 区间端点)的最小间隙(m):
        //   选中的点沿采样线前后 endpoint_clearance 内必须都是 known-free,否则丢弃。
        //   避免在被 unknown 夹住的短 free 段(如 0.16m)里选出紧贴 unknown 的观察点。
        viewpoint_endpoint_clearance_ = this->declare_parameter<double>(
            "viewpoint_endpoint_clearance", 0.10);
        viewpoint_max_distances_per_dir_ = this->declare_parameter<int>(
            "viewpoint_max_distances_per_dir", 4);
        // ★ frontier 最大距离(m,离基座):建 seed 时超出即丢;<=0 关闭限制。
        frontier_max_distance_ = this->declare_parameter<double>(
            "frontier_max_distance", 3.0);
        // ★ 全局聚合(位置+方向)参数 + 聚合后进 A* 的 top-N + 生成安全上限
        aggregate_merge_distance_ = this->declare_parameter<double>(
            "aggregate_merge_distance", 0.15);
        aggregate_merge_angle_deg_ = this->declare_parameter<double>(
            "aggregate_merge_angle_deg", 30.0);
        motion_cost_top_n_ = this->declare_parameter<int>("motion_cost_top_n", 50);
        max_raw_candidates_ = this->declare_parameter<int>(
            "max_raw_candidates", 4000000);   // 防 OOM 的候选生成上限
        max_ik_calls_per_cycle_ = this->declare_parameter<int>(
            "max_ik_calls_per_cycle", 120);
        ik_timeout_sec_ = this->declare_parameter<double>("ik_timeout_sec", 0.05);
        planning_group_ = this->declare_parameter<std::string>(
            "planning_group", "jaka_a5");
        ik_link_name_ = this->declare_parameter<std::string>("ik_link_name", "tool0");
        compute_ik_service_ = this->declare_parameter<std::string>(
            "compute_ik_service", "/compute_ik");
        state_validity_service_ = this->declare_parameter<std::string>(
            "state_validity_service", "/check_state_validity");
        // 默认 false:/compute_ik(avoid_collisions=true)已做 IK+关节限位+自碰撞+
        //   环境碰撞校验,单独再调 /check_state_validity 只是冗余往返、翻倍延迟。
        //   置 true 可按 prompt Phase 5 显式再验一次。
        enable_state_validity_check_ = this->declare_parameter<bool>(
            "enable_state_validity_check", false);
        // 三项打分权重(自动归一化,非法配置在此报错)
        score_w_unknown_    = this->declare_parameter<double>("score_w_unknown", 0.3);
        score_w_visibility_ = this->declare_parameter<double>("score_w_visibility", 0.5);
        score_w_semantic_   = this->declare_parameter<double>("score_w_semantic", 0.2);
        score_hfov_deg_ = this->declare_parameter<double>("score_hfov_deg", 69.0);
        score_vfov_deg_ = this->declare_parameter<double>("score_vfov_deg", 42.0);
        // Mean Shift 参数
        ms_sigma_position_    = this->declare_parameter<double>("ms_sigma_position", 0.10);
        ms_sigma_orientation_ = this->declare_parameter<double>("ms_sigma_orientation", 0.52);
        ms_lambda_orientation_= this->declare_parameter<double>("ms_lambda_orientation", 1.0);
        ms_merge_distance_    = this->declare_parameter<double>("ms_merge_distance", 0.05);
        ms_merge_angle_deg_   = this->declare_parameter<double>("ms_merge_angle_deg", 30.0);
        ms_max_iter_          = this->declare_parameter<int>("ms_max_iter", 20);
        // Greedy 参数(λ 自动归一化,非法报错)
        greedy_lambda_score_  = this->declare_parameter<double>("greedy_lambda_score", 0.7);
        greedy_lambda_motion_ = this->declare_parameter<double>("greedy_lambda_motion", 0.3);
        greedy_utility_mode_  = this->declare_parameter<std::string>(
            "greedy_utility_mode", "weighted_sum");
        // 工作空间 A* 参数
        astar_grid_res_       = this->declare_parameter<double>("astar_grid_res", 0.05);
        astar_inflation_      = this->declare_parameter<double>("astar_inflation", 0.05);
        astar_max_expansions_ = this->declare_parameter<int>("astar_max_expansions", 200000);

        // 启动即校验权重合法性(非法配置明确报错,prompt Phase 4/7 要求)
        {
            hppt_baseline::ScoreWeights sw{score_w_unknown_, score_w_visibility_,
                                           score_w_semantic_};
            sw.normalizeOrThrow();   // 抛异常 = 启动失败,不静默
            hppt_baseline::GreedyParams gp;
            gp.lambda_score = greedy_lambda_score_;
            gp.lambda_motion = greedy_lambda_motion_;
            gp.normalizeOrThrow();
        }

        // /joint_states —— IK 种子 + 当前关节(默认 QoS 与 robot_state_publisher 一致)
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&HPPTBoundaryNode::jointStateCallback, this,
                      std::placeholders::_1));

        // IK / state-validity 服务客户端置独立 ReentrantCallbackGroup,
        // 使 octomapCallback 阻塞等待服务响应时,另一线程可处理响应(不死锁)。
        ik_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        compute_ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>(
            compute_ik_service_, rclcpp::ServicesQoS(), ik_cb_group_);
        state_validity_client_ =
            this->create_client<moveit_msgs::srv::GetStateValidity>(
                state_validity_service_, rclcpp::ServicesQoS(), ik_cb_group_);
        RCLCPP_INFO(this->get_logger(),
            "NEW_METHOD Greedy Baseline: use_greedy_baseline=%s, IK via %s / %s "
            "(group=%s, link=%s), max_ik/cycle=%d, score_w=[U%.2f V%.2f S%.2f], "
            "greedy_lambda=[score%.2f motion%.2f, mode=%s], astar_res=%.3f.",
            use_greedy_baseline_ ? "true" : "false",
            compute_ik_service_.c_str(), state_validity_service_.c_str(),
            planning_group_.c_str(), ik_link_name_.c_str(),
            max_ik_calls_per_cycle_, score_w_unknown_, score_w_visibility_,
            score_w_semantic_, greedy_lambda_score_, greedy_lambda_motion_,
            greedy_utility_mode_.c_str(), astar_grid_res_);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        current_position_ = Eigen::Vector3d::Zero();
        last_goal_ = Eigen::Vector3d::Zero();
        last_selection_attempt_at_ = this->now();
        duplicate_goal_pause_until_ = this->now();
        has_last_goal_ = false;
        pending_seen_feedback_ = false;
        task_counter_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "HPPTBoundaryNode ready. octomap=%s, env_status=%s, feedback=%s, "
            "result=%s, semantic=%s, world=%s, robot=%s, timeout=%.1fs",
            octomap_topic_.c_str(), env_status_topic_.c_str(), feedback_topic_.c_str(),
            result_topic_.c_str(), semantic_topic_.c_str(),
            world_frame_.c_str(), robot_frame_.c_str(), task_timeout_sec_);
    }

private:
    void semanticCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!use_semantic_probability_) return;
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Semantic cloud has empty frame_id; ignoring.");
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                world_frame_, msg->header.frame_id, tf2::TimePointZero,
                tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Semantic TF lookup %s->%s failed: %s",
                world_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
            return;
        }

        const auto& tr = tf.transform.translation;
        const auto& qr = tf.transform.rotation;
        Eigen::Vector3d t(tr.x, tr.y, tr.z);
        Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);

        std::vector<SemanticPointWorld> points;
        points.reserve(msg->width * msg->height);
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_rel(
                *msg, "relevance_prob");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rel) {
                double rel = static_cast<double>(*iter_rel);
                if (!std::isfinite(rel) || rel < semantic_min_relevance_) continue;
                Eigen::Vector3d local(*iter_x, *iter_y, *iter_z);
                if (!local.allFinite()) continue;
                points.push_back({q * local + t, std::clamp(rel, 0.0, 1.0)});
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Failed to parse semantic PointCloud2 fields: %s", ex.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(semantic_mutex_);
            const double now_sec = this->now().seconds();
            const double voxel = std::max(1e-3, semantic_fusion_voxel_size_);
            for (const auto& sp : points) {
                SemanticVoxelKey key{
                    static_cast<int>(std::floor(sp.position.x() / voxel)),
                    static_cast<int>(std::floor(sp.position.y() / voxel)),
                    static_cast<int>(std::floor(sp.position.z() / voxel))};
                auto& ev = semantic_voxels_[key];
                ev.position_sum += sp.position;
                ev.relevance_sum += sp.relevance_prob;
                ev.max_relevance = std::max(ev.max_relevance, sp.relevance_prob);
                ++ev.count;
                ev.last_seen_sec = now_sec;
            }
            if (semantic_fusion_max_age_sec_ > 0.0) {
                for (auto it = semantic_voxels_.begin(); it != semantic_voxels_.end();) {
                    if (now_sec - it->second.last_seen_sec > semantic_fusion_max_age_sec_) {
                        it = semantic_voxels_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            latest_semantic_stamp_ = this->now();
            have_semantic_points_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Semantic fusion update: frame_points=%zu fused_voxels=%zu "
                "voxel_size=%.3fm max_age=%.1fs.",
                points.size(), semantic_voxels_.size(), voxel,
                semantic_fusion_max_age_sec_);
        }
    }

    void envStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        auto accepted = jsonGetBool(msg->data, "accepted");
        auto version  = jsonGetInt64(msg->data, "version");
        auto payload_bytes = jsonGetInt64(msg->data, "payload_bytes");
        if (!accepted.has_value()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "envStatusCallback: message missing 'accepted' field; "
                "cannot advance env_version. Payload prefix: '%.80s'",
                msg->data.c_str());
            return;
        }
        if (!accepted.value()) {
            RCLCPP_WARN(this->get_logger(),
                "envStatusCallback: env update REJECTED by downstream "
                "(version=%s). HPP-PT will keep using previous "
                "env_version=%s.",
                version.has_value() ? std::to_string(*version).c_str() : "?",
                latest_env_version_.has_value()
                    ? std::to_string(*latest_env_version_).c_str() : "none");
            return;
        }
        if (!version.has_value()) {
            RCLCPP_WARN(this->get_logger(),
                "envStatusCallback: accepted=true but no 'version' field; "
                "cannot advance env_version.");
            return;
        }
        // ★ FIX PROBLEM 2: ignore the 0-byte handshake environment that
        // initial_scan publishes at startup. If we latch that version,
        // HPP-PT will start firing cost probes at motion_planner while
        // initial_scan is still executing its 6 moves --- every probe
        // gets rejected as "planner busy" and previously that led to
        // fast-blacklisting the frontier. We wait until env_bridge
        // announces a real, non-empty OctoMap before enabling cycles.
        // (payload_bytes absent => old env_bridge => accept for
        // backward compatibility.)
        if (payload_bytes.has_value() && payload_bytes.value() <= 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                5000,
                "envStatusCallback: ignoring empty environment (version=%ld, "
                "payload_bytes=0). Waiting for a real OctoMap snapshot "
                "before starting HPP-PT cycles.",
                static_cast<long>(version.value()));
            return;
        }
        if (!latest_env_version_.has_value() ||
            latest_env_version_.value() != version.value()) {
            RCLCPP_INFO(this->get_logger(),
                "envStatusCallback: env_version %s -> %ld (accepted by "
                "downstream, payload_bytes=%s).",
                latest_env_version_.has_value()
                    ? std::to_string(*latest_env_version_).c_str() : "none",
                static_cast<long>(version.value()),
                payload_bytes.has_value()
                    ? std::to_string(*payload_bytes).c_str() : "n/a");
            latest_env_version_ = version.value();
        }
    }

    void resultCallback(const std_msgs::msg::String::SharedPtr msg) {
        auto tid = jsonGetString(msg->data, "task_id");
        if (!tid.has_value()) return;

        // ★★★ Cost probe 分流：cost_probe_* 前缀走独立的 cost_probes_ 表，
        // 不占用 pending_task_id_，避免干扰正常任务追踪。
        if (tid->rfind("cost_probe_", 0) == 0) {
            auto success = jsonGetBool(msg->data, "success").value_or(false);
            auto retryable = jsonGetBool(msg->data, "retryable").value_or(false);
            auto pt = jsonGetDouble(msg->data, "planning_time").value_or(0.0);
            auto et = jsonGetDouble(msg->data, "estimated_execution_time").value_or(0.0);
            std::string error_type = jsonGetString(msg->data, "error_type").value_or("");
            Eigen::Vector3d probe_goal;
            bool have_goal = false;
            {
                std::lock_guard<std::mutex> lk(cost_probes_mutex_);
                auto it = cost_probes_.find(*tid);
                if (it != cost_probes_.end()) {
                    it->second.succeeded = success;
                    // Always assign a numeric cost so the wait loop in
                    // octomapCallback marks this probe "done".  Failures
                    // (any reason) get cost_probe_max_cost so downstream's
                    // MAX-edge sink still applies for this cycle.  What
                    // changes below (FIX PROBLEM 1) is only the blacklist
                    // decision --- retryable failures no longer poison the
                    // frontier for 60s; they simply have no useful cost
                    // this cycle and get re-probed next cycle.
                    it->second.cost = success
                        ? (pt + et)
                        : cost_probe_max_cost_;
                    probe_goal = it->second.goal;
                    have_goal = true;
                    RCLCPP_DEBUG(this->get_logger(),
                        "cost_probe '%s' -> success=%s retryable=%s "
                        "error_type='%s' pt=%.3f et=%.3f",
                        tid->c_str(),
                        success ? "true" : "false",
                        retryable ? "true" : "false",
                        error_type.c_str(), pt, et);
                }
            }
            // ★★★ NON-CONSERVATIVE FALLBACK: 按 motion planner 回报的 error_type
            //   分级拉黑到一小块范围。
            //     hard_unreachable(非保守也直接失败) → 永久(regular, 永不释放)
            //     blocked_by_unknown / *_timeout / not_known_free / ...→ 临时(sticky)
            //     基础设施类 retryable(env 版本 / move_group 超时) → 不拉黑, 下轮重试
            if (have_goal && !success) {
                BlockKind k = classifyBlock(error_type, success);
                if (k == BlockKind::None) {
                    RCLCPP_INFO(this->get_logger(),
                        "cost_probe '%s' failure error_type='%s' (retryable=%s) "
                        "-> NOT blacklisted (infra/retryable, retry next cycle).",
                        tid->c_str(), error_type.c_str(),
                        retryable ? "true" : "false");
                } else if (!use_greedy_baseline_) {
                    blacklistByErrorType(probe_goal, error_type, success,
                                         "cost_probe");
                } else {
                    // ★ GREEDY BASELINE(prompt §十):plan-only 失败只表示
                    //   current→该候选的"边无效",在本 cycle 的重选循环里标 Infinity 边,
                    //   不做跨 cycle 持久拉黑(每个 env_version 重新推导)。
                    RCLCPP_INFO(this->get_logger(),
                        "cost_probe '%s' failure (error_type='%s') -> greedy "
                        "baseline: edge=Infinity only (no persistent blacklist).",
                        tid->c_str(), error_type.c_str());
                }
            }
            return;  // 不进入正常任务处理逻辑
        }

        if (pending_task_id_.has_value() && pending_task_id_.value() == tid.value()) {
            auto success = jsonGetBool(msg->data, "success").value_or(false);
            auto retryable = jsonGetBool(msg->data, "retryable").value_or(false);
            auto planning_time = jsonGetDouble(msg->data, "planning_time").value_or(0.0);
            auto execution_time = jsonGetDouble(
                msg->data, "estimated_execution_time").value_or(0.0);
            // ★ Stale-plan diagnostic: how much wall-clock passed and how
            //   many env_version bumps the OctoMap has taken since we picked
            //   this goal.  Large drift is the classic reason a probe-picked
            //   viewpoint fails at real execute.
            double age_sec = (this->now() - pending_task_sent_at_).seconds();
            int64_t cur_ev = latest_env_version_.value_or(0);
            int64_t probe_ev = pending_task_env_version_;
            double env_drift_sec = 0.0;
            if (cur_ev > 0 && probe_ev > 0) {
                // env_version 是 nanosecond timestamp
                env_drift_sec =
                    static_cast<double>(cur_ev - probe_ev) / 1.0e9;
            }
            RCLCPP_INFO(this->get_logger(),
                "Result for pending task '%s': success=%s, retryable=%s, "
                "planning_time=%.3fs, estimated_execution_time=%.3fs, "
                "task_age=%.1fs, probe_env_v=%ld, current_env_v=%ld, "
                "env_drift=%.1fs -- ready for next task",
                tid.value().c_str(), success ? "true" : "false",
                retryable ? "true" : "false", planning_time, execution_time,
                age_sec, static_cast<long>(probe_ev),
                static_cast<long>(cur_ev), env_drift_sec);
            if (!success && env_drift_sec > 30.0) {
                RCLCPP_WARN(this->get_logger(),
                    "STALE-PLAN LIKELY: task '%s' failed with env_drift="
                    "%.1fs (probe validated against env_v=%ld, "
                    "execute ran against env_v=%ld). "
                    "Cost probes are slow enough that OctoMap grew "
                    "substantially between probe and execute; goals that "
                    "passed probe may now be blocked by new occupied "
                    "voxels. Consider: fewer probes per cycle, faster "
                    "MoveIt planner, or accepting probe results within a "
                    "narrower env_version window.",
                    tid.value().c_str(), env_drift_sec,
                    static_cast<long>(probe_ev),
                    static_cast<long>(cur_ev));
            }
            // ★ P0-A: 无论 retryable 与否，只要 success=false 就拉黑；
            //   否则 retryable=true 的失败 goal 会在 octomap 没变化时被反复选中，
            //   形成"同一 goal → 失败 → 不入黑名单 → 又选同一 goal"的闭环。
            //   偶发 transient 失败被拉 60s 可接受（blacklist 有超时衰减）。
            if (!success && pending_goal_.has_value()) {
                std::string error_type =
                    jsonGetString(msg->data, "error_type").value_or("");
                if (!use_greedy_baseline_) {
                    // ★★★ LEGACY: execute 失败按 error_type 分级拉黑一小块。
                    //   仅 hard_unreachable → 永久;其余 → 临时 sticky。
                    bool permanent =
                        (classifyBlock(error_type, success) == BlockKind::Permanent);
                    blacklistPoint(pending_goal_.value(), block_radius_small_,
                                   "execute",
                                   error_type.empty() ? "execute_fail"
                                                      : error_type.c_str(),
                                   permanent);
                    RCLCPP_WARN(this->get_logger(),
                        "Execute task '%s' failed (error_type='%s', retryable=%s) -> "
                        "%s blacklist r=%.2fm at [%.3f,%.3f,%.3f].",
                        tid.value().c_str(), error_type.c_str(),
                        retryable ? "true" : "false",
                        permanent ? "PERMANENT" : "temporary", block_radius_small_,
                        pending_goal_->x(), pending_goal_->y(), pending_goal_->z());
                } else {
                    // ★ GREEDY BASELINE: 不拉黑。每周期从零重新选候选,失败点后续
                    //   unknown 变少可能就可达;within-cycle 的 Infinity 边重选已防
                    //   坏候选卡住机械臂。
                    RCLCPP_WARN(this->get_logger(),
                        "Execute task '%s' failed (error_type='%s') -> greedy "
                        "baseline: no blacklist, re-select fresh next cycle.",
                        tid.value().c_str(), error_type.c_str());
                }
                // 允许下一帧立刻重选别的目标
                has_last_goal_ = false;
            }
            if (success && pending_goal_.has_value()) {
                {
                    std::lock_guard<std::mutex> lk(blacklist_mutex_);
                    visited_.push_back({pending_goal_.value(), this->now()});
                }
                RCLCPP_INFO(this->get_logger(),
                    "Marking completed goal [%.3f, %.3f, %.3f] as visited for %.1fs.",
                    pending_goal_->x(), pending_goal_->y(), pending_goal_->z(),
                    visited_duration_sec_);
                // ★ 永久访问计数（默认关闭：permanent_block_count_=0 时不生效）
                bump_visit_history(pending_goal_.value());
                // ★ FIX PROBLEM 4 (from 7-min run): also clear last_goal_
                // on SUCCESS.  Otherwise the tight 2 cm duplicate check
                // (is_duplicate_goal) blocks that exact spot forever, and
                // once visited_ expires at visited_duration_sec (default
                // 45 s) the arm re-considers the same cluster but the
                // duplicate gate never lifts --- we saw 12+ consecutive
                // STAGE 5 EMPTY cycles with "duplicates last goal within
                // 0.020m" until the run ended.  visited_ (0.15 m / 45 s)
                // still prevents immediate re-selection; after it expires
                // it's fine to revisit if the frontier still calls for it.
                has_last_goal_ = false;
            }
            // 清 pending(本任务已收到最终结果)。
            pending_task_id_.reset();
            pending_goal_.reset();
            pending_seen_feedback_ = false;
            // ★★★ EVENT-DRIVEN: 用 motion_planner 的真实结果驱动 greedy 状态机。
            //   成功(execute success,只在机械臂真实到达后返回)→ 结束本 cycle,
            //     下个 OctoMap 触发新 cycle(从生成 frontier 重新开始);
            //   失败(规划失败 moveit_planning_failure / 执行失败 execution_failure /
            //     trajectory_not_known_free 等)→ 本 cycle 把该点 motion_cost 标
            //     Infinity、重排、派发下一个候选(不跨 cycle,不拉黑)。
            if (use_greedy_baseline_) {
                if (success) {
                    onGreedyExecuteSucceeded();
                } else {
                    onGreedyExecuteFailed(
                        jsonGetString(msg->data, "error_type").value_or(""));
                }
            }
        }
    }

    void feedbackCallback(const std_msgs::msg::String::SharedPtr msg) {
        auto tid = jsonGetString(msg->data, "task_id");
        if (!tid.has_value()) return;
        if (pending_task_id_.has_value() && pending_task_id_.value() == tid.value()) {
            if (!pending_seen_feedback_) {
                RCLCPP_INFO(this->get_logger(),
                    "feedbackCallback: downstream picked up pending task '%s'",
                    tid->c_str());
            }
            pending_seen_feedback_ = true;
        }
    }

    // ★ PAUSE: /exploration_pause 回调。暂停时,当前正在执行的 execute 会正常跑完
    //   (resultCallback 处理到达),之后 octomapCallback 不开新 cycle、
    //   dispatchNextGreedyCandidate 不派发下一个 → 决策暂停,机械臂停在原地;
    //   octomap 由 self_filter 侧同步停更新。继续时恢复正常。
    void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        const bool was = paused_.exchange(msg->data);
        if (was != msg->data) {
            RCLCPP_WARN(this->get_logger(), "EXPLORATION %s (via /exploration_pause).",
                        msg->data
                            ? "PAUSED -- will stop after the current task finishes"
                            : "RESUMED");
        }
    }

    bool isBlacklisted(const Eigen::Vector3d& p) {
        std::lock_guard<std::mutex> lk(blacklist_mutex_);
        // Regular blacklist is now permanent-within-session (no time-based
        // expiry). Entries can still be cleared via releaseStickyBlacklist
        // on STAGE 5 EMPTY (which also clears sticky_blacklist_) — see
        // releaseStickyBlacklist below; regular blacklist is untouched by
        // that path so failed goals stay blocked for the whole run.
        for (const auto& e : blacklist_) {
            // ★ TIER-1.3: 每 entry 可用自己更大的 fast-reject 半径
            double r = (e.radius > 0.0) ? e.radius : blacklist_radius_;
            if ((p - e.pos).norm() < r) return true;
        }
        // ★ TIER-4: sticky blacklist (no expiration; released via
        //   releaseStickyBlacklist() only when STAGE 5 exhausts all
        //   candidates).
        for (const auto& e : sticky_blacklist_) {
            double r = (e.radius > 0.0) ? e.radius : blacklist_radius_;
            if ((p - e.pos).norm() < r) return true;
        }
        return false;
    }

    // ★ TIER-4: add to sticky blacklist AND regular (regular's timed
    //   auto-expiry is still useful for logging; sticky handles the
    //   real "don't re-select" enforcement).
    void addToStickyBlacklist(const Eigen::Vector3d& pos, double radius) {
        // Caller holds blacklist_mutex_.
        sticky_blacklist_.push_back({pos, this->now(), radius});
    }

    // ★ TIER-4: release all sticky entries. Called when HPP-PT reports
    //   STAGE 5 EMPTY (no candidate survived all filters), which means
    //   the sticky blacklist itself is blocking every option. Log so we
    //   can trace how often this happens.
    void releaseStickyBlacklist(const char* reason) {
        std::string diag;
        size_t reg_after = 0;
        {
            std::lock_guard<std::mutex> lk(blacklist_mutex_);
            if (sticky_blacklist_.empty()) return;
            // ★★★ BL-DIAG: 逐条记录被移除的 sticky 条目 + 触发原因 + 存活时长。
            //   这是 sticky(橙球)唯一的移除路径; 橙球消失时来这里对照。
            auto now_t = this->now();
            std::ostringstream oss;
            oss << "BL-REMOVE [trigger=" << reason << "] releasing "
                << sticky_blacklist_.size() << " sticky entries:";
            for (const auto& e : sticky_blacklist_) {
                char b[110];
                std::snprintf(b, sizeof(b),
                    " (%.3f,%.3f,%.3f;r=%.2f;age=%.1fs)",
                    e.pos.x(), e.pos.y(), e.pos.z(),
                    (e.radius > 0.0 ? e.radius : blacklist_radius_),
                    (now_t - e.added_at).seconds());
                oss << b;
            }
            diag = oss.str();
            sticky_blacklist_.clear();
            reg_after = blacklist_.size();
        }
        hpptLog(diag);
        RCLCPP_WARN(this->get_logger(),
            "STICKY-RELEASE: cleared sticky-blacklist because %s "
            "(regular kept=%zu). Failed goals will be re-considered next cycle.",
            reason, reg_after);
    }

    bool isVisited(const Eigen::Vector3d& p) {
        auto now_t = this->now();
        std::lock_guard<std::mutex> lk(blacklist_mutex_);  // reuse as state mutex
        visited_.erase(std::remove_if(visited_.begin(), visited_.end(),
            [&](const VisitEntry& e){
                return (now_t - e.added_at).seconds() > visited_duration_sec_;
            }), visited_.end());
        for (const auto& e : visited_) {
            if ((p - e.pos).norm() < visited_radius_) return true;
        }
        return false;
    }

    // ★ 永久黑名单检查：仅当 permanent_block_count_ > 0 时启用
    // 12cm 直径 (6cm 半径) 内累计成功访问 >= permanent_block_count_ 的簇被禁用
    bool isPermanentlyBlocked(const Eigen::Vector3d& p) const {
        if (permanent_block_count_ <= 0) return false;   // ★ 默认关闭
        for (const auto& c : visit_history_) {
            if (c.count >= permanent_block_count_ &&
                (p - c.pos).norm() < permanent_block_radius_) {
                return true;
            }
        }
        return false;
    }

    // 每次成功任务后调用：仅当 permanent_block_count_ > 0 时启用
    void bump_visit_history(const Eigen::Vector3d& p) {
        if (permanent_block_count_ <= 0) return;   // ★ 默认关闭，什么都不做

        int nearest_idx = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < visit_history_.size(); ++i) {
            double d = (p - visit_history_[i].pos).norm();
            if (d < permanent_block_radius_ && d < best_d) {
                nearest_idx = static_cast<int>(i);
                best_d = d;
            }
        }
        if (nearest_idx >= 0) {
            auto& c = visit_history_[nearest_idx];
            c.count++;
            c.pos = (c.pos * (c.count - 1) + p) / c.count;
            RCLCPP_INFO(this->get_logger(),
                "Permanent visit cluster #%d at [%.3f, %.3f, %.3f] count=%d/%d",
                nearest_idx, c.pos.x(), c.pos.y(), c.pos.z(),
                c.count, permanent_block_count_);
            if (c.count == permanent_block_count_) {
                RCLCPP_WARN(this->get_logger(),
                    "★ Cluster #%d at [%.3f, %.3f, %.3f] hit count %d "
                    "→ PERMANENTLY BLOCKED (radius %.2fm) for this session.",
                    nearest_idx, c.pos.x(), c.pos.y(), c.pos.z(),
                    permanent_block_count_, permanent_block_radius_);
            }
        } else {
            visit_history_.push_back({p, 1});
            RCLCPP_INFO(this->get_logger(),
                "New permanent visit cluster #%zu at [%.3f, %.3f, %.3f] count=1/%d",
                visit_history_.size() - 1, p.x(), p.y(), p.z(),
                permanent_block_count_);
        }
    }

    // 完全无法救回的点：贴基座（r < reach_min）或 z 超范围
    // 这类点即使做”远远观察”投影，也会让相机反折朝内、姿态不合理
    bool isForbidden(const Eigen::Vector3d& p) {
        double dx = p.x() - base_x_;
        double dy = p.y() - base_y_;
        double dz = p.z() - base_z_;
        double r = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (r < reach_min_) return true;
        if (p.z() < z_min_ || p.z() > z_max_) return true;
        return false;
    }

    // 仅”距离太远”：可通过投影到 reach 内的球面来间接观察
    bool isUnreachable(const Eigen::Vector3d& p) {
        double dx = p.x() - base_x_;
        double dy = p.y() - base_y_;
        double dz = p.z() - base_z_;
        double r = std::sqrt(dx*dx + dy*dy + dz*dz);
        return r > reach_max_;
    }

    // ★ 硬距离截断：> frontier_max_range_factor × reach_max 的 frontier 直接丢。
    //   典型 = 3.0 × 0.85 = 2.55m。超出后 surrogate 观察为切线视角，几乎无信息。
    //   对固定基座 = 永久拉黑（stateless，每 cycle 重新过滤一次即可）。
    bool isTooFar(const Eigen::Vector3d& p) const {
        double dx = p.x() - base_x_;
        double dy = p.y() - base_y_;
        double dz = p.z() - base_z_;
        double r = std::sqrt(dx*dx + dy*dy + dz*dz);
        return r > frontier_max_range_factor_ * reach_max_;
    }

    // 兼容旧调用：out_of_workspace = forbidden OR (unreachable AND 不允许观察)
    bool isOutOfWorkspace(const Eigen::Vector3d& p) {
        if (isForbidden(p)) return true;
        if (!observe_unreachable_ && isUnreachable(p)) return true;
        return false;
    }

    // Project a frontier onto the reachable sphere.
    //
    // Preferred logic ("look-at projection"): if the caller provides
    // `unknown_dir` (unit vector from frontier INTO unknown), extend the
    // same line through the frontier IN THE OPPOSITE DIRECTION (i.e.
    // into the free-space side) until it hits the reach sphere.
    // The camera-proxy position is that intersection; the camera keeps
    // looking along `unknown_dir` at the original frontier. This keeps
    // the observation geometry the caller intended -- the surrogate
    // just slides along the look-at line, no radial re-aim.
    //
    // Fallback ("radial"): if no look direction is given, or if the
    // look-at line misses the sphere / points away from base entirely,
    // fall back to the original radial scale-to-sphere projection so
    // the caller always gets *some* reachable point.
    Eigen::Vector3d projectToReachable(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& unknown_dir = Eigen::Vector3d::Zero())
    {
        Eigen::Vector3d base(base_x_, base_y_, base_z_);
        Eigen::Vector3d rel = p - base;
        double r = rel.norm();
        if (r < 1e-6) return p;
        double target_r = projection_scale_ * reach_max_;
        target_r = std::max(target_r, reach_min_ + 0.01);
        double min_observation_z = std::max(z_min_, observation_min_z_);

        // ---- Primary: look-at line intersection with reach sphere ----
        // Line: Q(t) = p + t * cam_dir, cam_dir = -unknown_dir (toward free).
        // Solve |Q(t) - base|² = target_r² for t ≥ 0.
        //   quadratic: t² + 2t (d·cam_dir) + (d·d - R²) = 0
        //   where d = p - base = rel, R = target_r.
        if (unknown_dir.squaredNorm() > 1e-12) {
            Eigen::Vector3d cam_dir = -unknown_dir.normalized();
            double b_half = rel.dot(cam_dir);           // d · cam_dir
            double c = rel.squaredNorm() - target_r * target_r;
            double disc = b_half * b_half - c;
            if (disc >= 0.0) {
                double sd = std::sqrt(disc);
                double t_near = -b_half - sd;   // smaller root
                double t_far  = -b_half + sd;   // larger root
                double t = -1.0;
                // frontier outside sphere (c > 0): both roots same sign.
                //   positive → line points toward base, take t_near
                //     (camera at first intersection = closest to frontier)
                //   negative → line points away from base, no valid hit
                // frontier inside sphere (c < 0): t_near < 0 < t_far,
                //   take t_far (camera at exit on free side)
                if (t_near > 1e-6) t = t_near;
                else if (t_far > 1e-6) t = t_far;
                if (t > 0.0) {
                    Eigen::Vector3d q = p + t * cam_dir;
                    // clamp z into observation range; if clamp had to
                    // move the point noticeably we'll still return it,
                    // it's better than the radial fallback for the
                    // usual case where z drift is small.
                    if (q.z() < min_observation_z) q.z() = min_observation_z;
                    if (q.z() > z_max_) q.z() = z_max_;
                    return q;
                }
            }
            // fell through → cam_dir points wrong way or line misses
            // sphere; use radial as safety net.
        }

        // ---- Radial fallback ----
        Eigen::Vector3d q = base + rel * (target_r / r);
        if (q.z() < min_observation_z) q.z() = min_observation_z;
        if (q.z() > z_max_) q.z() = z_max_;
        return q;
    }

    // 给定候选 frontier，返回实际要发布的 (goal 位置, look_at 位置)。
    // Frontier 点本身是地图边界，不适合作为 tool0 目标。相机应该站在
    // 已知/free 一侧，隔一段安全距离看向 frontier/unknown 方向。
    //
    // standoff_override < 0 → 用节点默认 observation_standoff_
    // otherwise             → 使用传入值（clamp 到 [min, max]）
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    resolveViewpoint(const octomap::point3d& coord,
                     const octomap::point3d& unknown_neighbor,
                     double standoff_override = -1.0) {
        Eigen::Vector3d orig_pos(coord.x(), coord.y(), coord.z());
        Eigen::Vector3d orig_look(
            unknown_neighbor.x(), unknown_neighbor.y(), unknown_neighbor.z());
        Eigen::Vector3d unknown_dir = orig_look - orig_pos;
        if (unknown_dir.norm() < 1e-6) {
            // Symptom: the unknown-neighbor encoding cancelled out (all six
            // face directions had unknown neighbours). Fall back to
            // "look away from base" so we at least pick a sane orientation.
            Eigen::Vector3d base(base_x_, base_y_, base_z_);
            unknown_dir = orig_pos - base;
            RCLCPP_DEBUG(this->get_logger(),
                "resolveViewpoint: unknown_dir was ~0 at coord "
                "[%.3f,%.3f,%.3f]; using base->coord fallback.",
                orig_pos.x(), orig_pos.y(), orig_pos.z());
        }
        if (unknown_dir.norm() < 1e-6) {
            // Even the base->coord fallback failed (frontier is essentially
            // on top of the base). Default +X. Should be very rare.
            unknown_dir = Eigen::Vector3d(1.0, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "resolveViewpoint: both unknown_dir and base->coord "
                "collapsed at [%.3f,%.3f,%.3f]; defaulting look_dir to +X.",
                orig_pos.x(), orig_pos.y(), orig_pos.z());
        }
        unknown_dir.normalize();

        // SKY-LOOKING FIX removed per user request: camera can now look
        // straight up when a frontier's unknown neighbour is +Z-dominant
        // (e.g. cabinet top). The look-at projection change already keeps
        // the goal geometrically sensible, and IG rays going into open
        // sky are handled elsewhere.

        // ★ P1-1: standoff 按传入值 clamp；调用方按 cluster 尺寸算好
        double standoff = (standoff_override > 0.0)
            ? std::clamp(standoff_override,
                         observation_standoff_min_, observation_standoff_max_)
            : observation_standoff_;

        Eigen::Vector3d goal = orig_pos - unknown_dir * standoff;

        // ★ P3-3: 若 frontier 本身低于 observation_min_z，允许 goal 更低
        //   effective_min_z ∈ [observation_min_z_low, observation_min_z]
        //   frontier 越低 → 越贴 low 阈值，避免"低 frontier 被强推到 0.25 高"的俯视
        double effective_min_z = observation_min_z_;
        if (orig_pos.z() < observation_min_z_) {
            effective_min_z = std::max(observation_min_z_low_,
                                        std::min(observation_min_z_,
                                                 orig_pos.z() - 0.05));
        }
        goal.z() = std::max(goal.z(), effective_min_z);
        goal.z() = std::min(goal.z(), z_max_);

        // ★ P1-2: look_at 定在 frontier 后 (standoff + 1m) —— 让相机光轴
        //   指向 "unknown 区域中心" 而不是刚过 frontier 5cm 的点。
        //   IG raycast 用同一 (goal, look_at) 算方向，这里改善后 IG 更准。
        double look_ahead = standoff + 1.0;
        Eigen::Vector3d look_at = orig_pos + unknown_dir * look_ahead;

        if (observe_unreachable_ && isUnreachable(goal)) {
            goal = projectToReachable(goal, unknown_dir);
            goal.z() = std::max(goal.z(), effective_min_z);
        }
        return {goal, look_at};
    }


    // ★★★ ALT-PULLBACK: 生成备用视点 —— 把 goal 沿 (base → goal) 反方向拉近
    //   alt_pullback_dist_ (默认 20cm),look_at 完全不变。
    //
    //   直觉:primary goal 常常贴到 reach shell 外围 (frontier 远端 + standoff 反推),
    //   手腕需要极端伸展 → IK 无解。往 base 方向拉近一点,机械臂更"卷起来",
    //   IK 通常更松。因为 look_at 不动,相机看的还是原来那片 unknown 区域,
    //   只是从稍近的距离看。
    //
    //   拉近距离 clamp 到 [reach_min + 2cm, 原距离],避免落进 forbidden 内球。
    //   z 走 z_min/z_max 兜底,不改变角度。若 alt 与 primary 距离 <5cm 视为
    //   "没差别" → 返回 nullopt。若 alt 触发 forbidden/inward/unreachable →
    //   fallback 到 perpendicular offset (up/down/right/left)。
    // Generate up to `max_count` alternative viewpoints by ANGULAR
    // deviation from the frontier→look_at axis (`ud`). Eight variants
    // total: rotate `ud` by ±30° and ±60° around the two axes
    // perpendicular to it (UP/DOWN around 'right', LEFT/RIGHT around
    // 'up'). Order: all four 30° variants first (UP, DOWN, LEFT,
    // RIGHT), then the same four at 60° — so a small deviation is
    // tried before a big one.
    //
    // Each alt gets its OWN look_at (F + rotated_ud * (standoff+1))
    // because we've tilted the observation axis. The camera keeps
    // seeing the frontier from a different angle, not just a
    // sideways-shifted primary.
    //
    // Filters (per try_add):
    //   - z clamp, isForbidden, isUnreachable → projectToReachable
    //     along the rotated axis, isInwardLook, dedup vs primary/alts.
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    generateAlternativeViewpoints(const octomap::point3d& coord,
                                   const octomap::point3d& unknown_neighbor,
                                   double standoff_override,
                                   int max_count) {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> alts;
        if (max_count <= 0) return alts;

        Eigen::Vector3d F(coord.x(), coord.y(), coord.z());
        Eigen::Vector3d L(unknown_neighbor.x(), unknown_neighbor.y(),
                          unknown_neighbor.z());
        Eigen::Vector3d ud = L - F;
        if (ud.norm() < 1e-6) return alts;
        ud.normalize();

        double standoff = (standoff_override > 0.0)
            ? std::clamp(standoff_override,
                         observation_standoff_min_, observation_standoff_max_)
            : observation_standoff_;
        double effective_min_z = observation_min_z_;
        if (F.z() < observation_min_z_) {
            effective_min_z = std::max(observation_min_z_low_,
                std::min(observation_min_z_, F.z() - 0.05));
        }

        Eigen::Vector3d prim_goal = F - ud * standoff;
        prim_goal.z() = std::max(prim_goal.z(), effective_min_z);
        prim_goal.z() = std::min(prim_goal.z(), z_max_);
        if (observe_unreachable_ && isUnreachable(prim_goal)) {
            prim_goal = projectToReachable(prim_goal, ud);
            prim_goal.z() = std::max(prim_goal.z(), effective_min_z);
        }
        Eigen::Vector3d look_at = F + ud * (standoff + 1.0);
        Eigen::Vector3d base(base_x_, base_y_, base_z_);

        // Each alt has its OWN look_at because the frontier→unknown
        // direction (ud) is rotated: the camera keeps looking through
        // the frontier along the rotated axis. try_add accepts (goal,
        // look_at, ud_for_projection) so projectToReachable can slide
        // along the rotated axis when unreachable.
        auto try_add = [&](Eigen::Vector3d alt_goal,
                           const Eigen::Vector3d& alt_look_at,
                           const Eigen::Vector3d& alt_ud) {
            if (static_cast<int>(alts.size()) >= max_count) return;
            alt_goal.z() = std::max(alt_goal.z(), z_min_);
            alt_goal.z() = std::min(alt_goal.z(), z_max_);
            if (isForbidden(alt_goal)) return;
            if (isUnreachable(alt_goal)) {
                if (!observe_unreachable_) return;
                alt_goal = projectToReachable(alt_goal, alt_ud);
            }
            if (isInwardLook(alt_goal, alt_look_at)) return;
            if ((alt_goal - prim_goal).norm() < 0.05) return;
            for (const auto& existing : alts) {
                if ((existing.first - alt_goal).norm() < 0.05) return;
            }
            alts.emplace_back(alt_goal, alt_look_at);
        };

        // ---- Angular deviation from the frontier→look_at axis ----
        // Rotate `ud` by ±θ around two axes (right, up) that span the
        // plane perpendicular to `ud`. Four directional variants per
        // angle: UP (+θ around right), DOWN (−θ around right),
        // LEFT (+θ around up), RIGHT (−θ around up).
        // Order per user request: all 30° first, then all 60° — so a
        // small deviation gets tried before a big one.
        Eigen::Vector3d helper = (std::abs(ud.z()) < 0.9)
            ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
        Eigen::Vector3d right_axis = ud.cross(helper).normalized();
        Eigen::Vector3d up_axis    = right_axis.cross(ud).normalized();
        struct Variant { const char* name; Eigen::Vector3d axis; double sign; };
        const std::vector<Variant> variants = {
            {"UP",    right_axis, +1.0},
            {"DOWN",  right_axis, -1.0},
            {"LEFT",  up_axis,    +1.0},
            {"RIGHT", up_axis,    -1.0},
        };
        const std::vector<double> angles_deg = {30.0, 60.0};
        for (double deg : angles_deg) {
            const double theta = deg * M_PI / 180.0;
            for (const auto& v : variants) {
                Eigen::AngleAxisd aa(v.sign * theta, v.axis);
                Eigen::Vector3d new_ud = (aa * ud).normalized();
                Eigen::Vector3d alt_goal = F - new_ud * standoff;
                alt_goal.z() = std::max(alt_goal.z(), effective_min_z);
                alt_goal.z() = std::min(alt_goal.z(), z_max_);
                Eigen::Vector3d alt_look_at =
                    F + new_ud * (standoff + 1.0);
                try_add(alt_goal, alt_look_at, new_ud);
                if (static_cast<int>(alts.size()) >= max_count) return alts;
            }
        }
        return alts;
    }

    // Backward-compatible singular wrapper — first alt only.  Left in case
    // any other caller relies on the old signature.
    std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    generateAlternativeViewpoint(const octomap::point3d& coord,
                                  const octomap::point3d& unknown_neighbor,
                                  double standoff_override) {
        Eigen::Vector3d F(coord.x(), coord.y(), coord.z());
        Eigen::Vector3d L(unknown_neighbor.x(), unknown_neighbor.y(),
                          unknown_neighbor.z());
        Eigen::Vector3d ud = L - F;
        if (ud.norm() < 1e-6) return std::nullopt;
        ud.normalize();

        double standoff = (standoff_override > 0.0)
            ? std::clamp(standoff_override,
                         observation_standoff_min_, observation_standoff_max_)
            : observation_standoff_;
        double effective_min_z = observation_min_z_;
        if (F.z() < observation_min_z_) {
            effective_min_z = std::max(observation_min_z_low_,
                std::min(observation_min_z_, F.z() - 0.05));
        }

        // Primary goal + look_at (same math as resolveViewpoint)
        Eigen::Vector3d prim_goal = F - ud * standoff;
        prim_goal.z() = std::max(prim_goal.z(), effective_min_z);
        prim_goal.z() = std::min(prim_goal.z(), z_max_);
        if (observe_unreachable_ && isUnreachable(prim_goal)) {
            prim_goal = projectToReachable(prim_goal, ud);
            prim_goal.z() = std::max(prim_goal.z(), effective_min_z);
        }
        Eigen::Vector3d look_at = F + ud * (standoff + 1.0);

        Eigen::Vector3d base(base_x_, base_y_, base_z_);

        // ---- Strategy 1: PULLBACK toward base ----
        // 沿 (base → prim_goal) 反方向缩短距离 alt_pullback_dist_,但不能小于
        // reach_min + 2cm(否则 goal 掉入 forbidden 内球)。
        {
            Eigen::Vector3d radial = prim_goal - base;
            double d_orig = radial.norm();
            if (d_orig > 1e-6) {
                double d_new = std::max(reach_min_ + 0.02,
                                        d_orig - alt_pullback_dist_);
                if (std::abs(d_new - d_orig) > 0.05) {  // meaningful shift
                    Eigen::Vector3d alt_goal =
                        base + radial * (d_new / d_orig);
                    alt_goal.z() = std::max(alt_goal.z(), z_min_);
                    alt_goal.z() = std::min(alt_goal.z(), z_max_);
                    // look_at 不变;alt_goal 已在 reach shell 内,不必再投影
                    if (!isForbidden(alt_goal) &&
                        !isInwardLook(alt_goal, look_at)) {
                        return std::make_pair(alt_goal, look_at);
                    }
                }
            }
        }

        // ---- Strategy 2 (fallback): perpendicular offset ----
        // pullback 触发 forbidden/inward 或 shift 太小(<5cm) 时兜底,
        // 保留原来 perpendicular offset 方案。
        Eigen::Vector3d look_dir = (look_at - prim_goal).normalized();
        Eigen::Vector3d helper = (std::abs(look_dir.z()) < 0.9)
            ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
        Eigen::Vector3d right = look_dir.cross(helper).normalized();
        Eigen::Vector3d up    = right.cross(look_dir).normalized();
        std::vector<Eigen::Vector3d> offsets = {
            +0.25 * up, -0.20 * up, +0.30 * right, -0.30 * right,
        };
        for (const auto& off : offsets) {
            Eigen::Vector3d alt_goal = prim_goal + off;
            alt_goal.z() = std::max(alt_goal.z(), z_min_);
            alt_goal.z() = std::min(alt_goal.z(), z_max_);
            if (isForbidden(alt_goal)) continue;
            if (isUnreachable(alt_goal)) {
                if (!observe_unreachable_) continue;
                alt_goal = projectToReachable(alt_goal, ud);
            }
            if (isInwardLook(alt_goal, look_at)) continue;
            if ((alt_goal - prim_goal).norm() < 0.05) continue;
            return std::make_pair(alt_goal, look_at);
        }
        return std::nullopt;
    }

    // ★ P3-1 + FIX-1/2: 相机→FRONTIER 视线是否被 known-occupied 挡住。
    //   FIX-1: 用 frontier(不是 look_at)作射线终点。look_at 在 frontier 后
    //     (standoff+1m),射线沿路很容易撞上 frontier 后方的 known-occupied
    //     (例如柜子后墙),但那些墙正是我们想 approach 的对象,不该淘汰候选。
    //     用 frontier 作终点后,射线只走 standoff 距离 ~35cm,只检查
    //     "goal 站的位置到 frontier 有没有墙挡在中间"—— 这才是原始意图。
    //   FIX-2: 允许 hit_dist >= los_min_hit_dist_ 的击中通过。中距离撞上物体
    //     意味着相机能看到 goal→撞点之间的一段 free/unknown 空间,是有用的观察。
    //     只有极近距离(<10cm)撞上才真的意味着"没什么可看"。
    bool isLineOfSightBlocked(octomap::OcTree& tree,
                              const Eigen::Vector3d& goal,
                              const Eigen::Vector3d& frontier_pos) {
        if (!enable_los_check_) return false;
        Eigen::Vector3d v = frontier_pos - goal;
        double dist = v.norm();
        if (dist < 1e-4) return false;
        v /= dist;
        octomap::point3d origin(
            static_cast<float>(goal.x()),
            static_cast<float>(goal.y()),
            static_cast<float>(goal.z()));
        octomap::point3d dir(
            static_cast<float>(v.x()),
            static_cast<float>(v.y()),
            static_cast<float>(v.z()));
        octomap::point3d hit;
        bool blocked = tree.castRay(origin, dir, hit,
                                    /*ignoreUnknown=*/true, dist);
        if (!blocked) return false;
        // FIX-2: 击中点距离 goal 越远,相机能看到的 free/unknown 越多。
        // 只有近到 los_min_hit_dist_ 以内的撞击才算真正 blocked。
        octomap::point3d dv = hit - origin;
        double hit_dist = std::sqrt(
            dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z());
        return hit_dist < los_min_hit_dist_;
    }

    // 向内看 = 相机的观察方向指向基座（相机会反折靠近基座）
    bool isInwardLook(const Eigen::Vector3d& goal, const Eigen::Vector3d& look_at) {
        if (!require_outward_look_) return false;
        Eigen::Vector3d base(base_x_, base_y_, base_z_);
        Eigen::Vector3d outward = goal - base;              // 从基座指向相机
        Eigen::Vector3d look_dir = look_at - goal;          // 相机的观察方向
        // 观察方向必须至少与"向外"有正内积；否则视为向内看
        return outward.dot(look_dir) <= 0.0;
    }

    // 组合判定：永久（工作空间外）+ 动态（近期失败）+ 向内看
    bool isGoalBad(const Eigen::Vector3d& p) {
        return isOutOfWorkspace(p) || isBlacklisted(p);
    }
    bool isGoalBad(const Eigen::Vector3d& p, const Eigen::Vector3d& look_at) {
        return isGoalBad(p) || isInwardLook(p, look_at);
    }

    double computeSemanticRelevance(const octomap::point3d& p) {
        if (!use_semantic_probability_ || semantic_radius_ <= 0.0) return 0.0;

        Eigen::Vector3d center(p.x(), p.y(), p.z());
        double radius2 = semantic_radius_ * semantic_radius_;
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        double max_relevance = 0.0;
        const double now_sec = this->now().seconds();
        std::lock_guard<std::mutex> lock(semantic_mutex_);
        if (!have_semantic_points_ || semantic_voxels_.empty()) return 0.0;
        for (const auto& kv : semantic_voxels_) {
            const auto& ev = kv.second;
            if (ev.count == 0) continue;
            if (semantic_fusion_max_age_sec_ > 0.0 &&
                now_sec - ev.last_seen_sec > semantic_fusion_max_age_sec_) {
                continue;
            }
            Eigen::Vector3d pos = ev.position_sum / static_cast<double>(ev.count);
            double d2 = (pos - center).squaredNorm();
            if (d2 > radius2) continue;
            // Nearby semantic points should matter more, but keep a max term
            // so a sparse confident detection still boosts the frontier.
            double w = 1.0 / (std::sqrt(d2) + 0.02);
            double mean_relevance = ev.relevance_sum / static_cast<double>(ev.count);
            double fused_relevance = std::clamp(
                0.7 * mean_relevance + 0.3 * ev.max_relevance, 0.0, 1.0);
            weighted_sum += w * fused_relevance;
            weight_sum += w;
            max_relevance = std::max(max_relevance, fused_relevance);
        }
        if (weight_sum <= 0.0) return 0.0;
        double mean_relevance = weighted_sum / weight_sum;
        return std::clamp(0.7 * mean_relevance + 0.3 * max_relevance, 0.0, 1.0);
    }

    // 在 point 周围球形邻域内统计 occupied / unknown / total 体素数
    // 返回归一化概率：
    //   w_obs * occupied_density + w_unk * unknown_density
    //   + w_sem * semantic_relevance
    // 用 tree.search() 采样：radius 内的每个网格点 (间隔 = tree.getResolution())
    //   - search() 返回 nullptr => unknown
    //   - occupancy >= 0.5      => occupied
    //   - 其余                   => free（忽略）
    // ★★★ raw_out (可选): 返回 clamp 前的原始加权和,供 Greedy 打分使用。
    //   HPP-PT 语义要求 prob < 1 (clamp 到 0.999),多个候选在 0.999 处会
    //   并列,Greedy 只能靠 cost 决定,导致"看天"和"看柜"都是 0.999 时,
    //   便宜的看天赢。用 raw_out 让 Greedy 排序反映真实差距。
    double computeViewpointProbability(octomap::OcTree& tree,
                                       const BoundaryPoint& bp,
                                       double* raw_out = nullptr) {
        auto [g0, l0] = resolveViewpoint(bp.coord, bp.unknown_neighbor,
                                         bp.standoff_override);
        return scoreViewpoint(tree, bp.coord, bp.unknown_neighbor,
                              g0, l0, raw_out);
    }

    // ★★★ DIRECTION-FIRST: 给定已生成好的 (goal_view, look_at_view) 直接打分。
    //   与旧路径的区别: 不再内部 resolveViewpoint 反推单一方向, 而是对调用方
    //   扇形展开出来的每条 (goal, look_at) 独立评估。密度/语义/occlusion 用
    //   frontier 体素 p 及其 unknown 方向; IG / directional attraction / LOS 用
    //   传入的 goal_view/look_at_view。
    // ★★★ LOGGER: 单个视点的分项打分明细，供 decision logger 记录。
    struct ScoreBreakdown {
        double obs = 0.0, unk = 0.0, occl = 0.0, sem = 0.0;
        double ig = 0.0, hit_ratio = 0.0, peek = 0.0, attract = 0.0;
        double raw = 0.0;
        bool   los_clear = true;
        bool   ig_gate_killed = false;
    };

    double scoreViewpoint(octomap::OcTree& tree,
                          const octomap::point3d& p,
                          const octomap::point3d& unknown_neighbor,
                          const Eigen::Vector3d& goal_view,
                          const Eigen::Vector3d& look_at_view,
                          double* raw_out = nullptr,
                          ScoreBreakdown* bd = nullptr) {
        double res = tree.getResolution();
        int steps = std::max(1, static_cast<int>(std::ceil(prob_density_radius_ / res)));
        int total = 0, occ = 0, unk = 0;
        for (int dx = -steps; dx <= steps; ++dx) {
            for (int dy = -steps; dy <= steps; ++dy) {
                for (int dz = -steps; dz <= steps; ++dz) {
                    octomap::point3d q(
                        p.x() + dx * res,
                        p.y() + dy * res,
                        p.z() + dz * res);
                    double dist2 = (q - p).norm();
                    if (dist2 > prob_density_radius_) continue;
                    ++total;
                    auto* node = tree.search(q);
                    if (node == nullptr) {
                        ++unk;
                    } else if (node->getOccupancy() >= 0.5) {
                        ++occ;
                    }
                }
            }
        }
        if (total == 0) return 0.0;
        double obs_density = static_cast<double>(occ) / total;
        double unk_density = static_cast<double>(unk) / total;
        double semantic_relevance = computeSemanticRelevance(p);
        // ★ Stage B1: 视锥 raycast IG（用 goal / look_at，不是 frontier 体素本身）
        // ★★★ TIER-2.4: 同时取回 hit_ratio(击中 known-occupied 的射线比例)
        // ★★★ TIER-3 (B): 取回 peek_score(透过 unknown 命中墙的贡献)
        double ig = 0.0;
        double hit_ratio = 0.0;
        double peek_score = 0.0;
        if (prob_w_ig_ > 0.0 || prob_w_peek_ > 0.0) {
            ig = hppt_stage::computeInformationGain(
                tree, goal_view, look_at_view,
                ig_fov_deg_, ig_max_range_m_, ig_num_rays_,
                &hit_ratio, &peek_score);
        }
        // ★★★ TIER-2.5 / TIER-3 (C): attraction
        double attraction = 0.0;
        if (prob_w_attract_ > 0.0) {
            if (directional_attract_) {
                attraction = hppt_stage::computeDirectionalAttraction(
                    tree, goal_view, look_at_view,
                    attract_search_radius_, attract_lateral_tol_,
                    attract_decay_, z_min_);
            } else {
                attraction = computeAttractiveness(tree, p);
            }
        }
        // ★★★ TIER-3 (A): frontier 背面/侧面被墙包围度
        double occlusion_score = 0.0;
        if (prob_w_occl_ > 0.0) {
            occlusion_score = hppt_stage::computeOcclusionScore(
                tree, p, unknown_neighbor, occl_radius_);
        }

        // ★★★ TIER-3 (LOS gate): 相机→frontier 视线如果被墙挡断,候选
        //   毫无观察价值。用 frontier 本身作 endpoint,从 free 侧的 camera
        //   直射到它应通畅;不通畅就是相机贴墙看墙背的病态。
        Eigen::Vector3d frontier_pt(p.x(), p.y(), p.z());
        bool los_clear = hppt_stage::losClear(tree, goal_view, frontier_pt);

        // ★★★ 原始加权和(未 clamp)。TIER-5: sky_factor 彻底移除。
        //   原意"look_at 上仰 → 相机在看天"太粗糙,连"斜看 45° shelf 顶"
        //   这种合法 pick 也一起打折。IG-gate + LOS gate + z_max 已经
        //   把真正"看天"病态拦掉了,sky_factor 是重复且过火的惩罚。
        double raw = (
            prob_w_obs_ * obs_density
            + prob_w_unk_ * unk_density
            + prob_w_occl_ * occlusion_score
            + prob_w_sem_ * semantic_relevance
            + prob_w_ig_  * (ig + ig_hit_bonus_ * hit_ratio)
            + prob_w_peek_ * peek_score
            + prob_w_attract_ * attraction
        );

        // ★★★ TIER-4 (IG-gate): 如果 raycast 几乎看不到 unknown (ig<0.02)
        //   且没有 attract 拉救(<0.05),说明相机根本无法观察到未知区域 ——
        //   典型"正对障碍物"病态: unk_density 从 frontier 邻域拿到高分,
        //   但 look_at 方向的 IG raycast 全部撞墙即停。归零阻止此类 pick。
        //   0.05 → 0.02 (更宽松): 保留"窄缝斜看"型 pick,只 kill 真"对墙
        //   ig=0"的极端情况。
        bool ig_gate_killed = false;
        // ★★★ IG-gate 只看 ig: 去掉原 "&& attraction < 0.05" 豁免。attraction 高
        //   但 ig≈0 的候选(相机靠近已知物体、却看不到新 unknown)之前借 attraction
        //   逃过门控、以高 RAW 被发布 → 对障碍物空看。现在 ig<0.02 一律归零。
        if (ig < 0.02) {
            raw = 0.0;
            ig_gate_killed = true;
        }

        if (!los_clear) {
            raw = 0.0;
        }

        // ★★★ TIER-3: INFO 打印每候选的分项分解,便于日志里验证行为。K=5 →
        //   每 cycle ~5 行,与 GREEDY-ARGMIN 行同量级,不算过量。
        // 逐候选分项明细降级为 DEBUG（排名后的 decision logger 会汇总打印）。
        RCLCPP_DEBUG(this->get_logger(),
            "PROB [p=(%.2f,%.2f,%.2f)]: obs=%.3f unk=%.3f occl=%.3f "
            "ig=%.3f peek=%.3f attract=%.3f sem=%.3f los=%d "
            "igate=%d raw=%.3f",
            p.x(), p.y(), p.z(),
            obs_density, unk_density, occlusion_score,
            ig, peek_score, attraction, semantic_relevance,
            los_clear ? 1 : 0, ig_gate_killed ? 1 : 0, raw);
        if (raw < 0.0) raw = 0.0;
        if (bd) {
            bd->obs = obs_density;   bd->unk = unk_density;
            bd->occl = occlusion_score; bd->sem = semantic_relevance;
            bd->ig = ig;             bd->hit_ratio = hit_ratio;
            bd->peek = peek_score;   bd->attract = attraction;
            bd->raw = raw;
            bd->los_clear = los_clear;
            bd->ig_gate_killed = ig_gate_killed;
        }
        if (raw_out) *raw_out = raw;
        // clamp 到 [0, 1)(HPP-PT 要求 <1;概率=1 让期望代价立刻收敛)
        double prob = raw;
        if (prob > 0.999) prob = 0.999;
        return prob;
    }

    // ★★★ TIER-2.5 attraction: 在候选点周围 attract_search_radius_ 范围内搜索,
    //   找最近的 known-occupied 体素;attraction = exp(-min_dist / attract_decay_)。
    //   没找到 → 0(候选悬在空气里,与任何物体都无关);找到且距离 0 → 1。
    //
    // ★★★ FLOOR-EXCLUSION: 跳过 q.z <= z_min_ 的 occupied voxel。地板 (z=0)
    //   总是 known-occupied,所有低 z 的候选(甚至根本没看柜子的)都会拿到
    //   attract=0.8+ 假信号 → 概率评分被地板"污染"。
    //   同理跳过 (q - base) 距离 <= reach_min_ 的 voxel(机械臂自身)。
    double computeAttractiveness(octomap::OcTree& tree,
                                  const octomap::point3d& p) {
        if (attract_search_radius_ <= 0.0 || attract_decay_ <= 0.0) return 0.0;
        double res = tree.getResolution();
        int steps = std::max(1,
            static_cast<int>(std::ceil(attract_search_radius_ / res)));
        double min_d = attract_search_radius_ + 1.0;   // 初值:未找到
        double r2 = attract_search_radius_ * attract_search_radius_;
        double reach_min_sq = reach_min_ * reach_min_;
        for (int dx = -steps; dx <= steps; ++dx) {
            for (int dy = -steps; dy <= steps; ++dy) {
                for (int dz = -steps; dz <= steps; ++dz) {
                    octomap::point3d q(
                        p.x() + dx * res,
                        p.y() + dy * res,
                        p.z() + dz * res);
                    // FLOOR-EXCLUSION: 地板不算"物体"
                    if (q.z() <= z_min_) continue;
                    // BASE-EXCLUSION: 机械臂自身占据体素不算"物体"
                    double qbx = q.x() - base_x_;
                    double qby = q.y() - base_y_;
                    double qbz = q.z() - base_z_;
                    if (qbx*qbx + qby*qby + qbz*qbz < reach_min_sq) continue;
                    octomap::point3d dv = q - p;
                    double d2 = dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z();
                    if (d2 > r2) continue;
                    auto* node = tree.search(q);
                    if (node && node->getOccupancy() >= 0.5) {
                        double d = std::sqrt(d2);
                        if (d < min_d) min_d = d;
                    }
                }
            }
        }
        if (min_d > attract_search_radius_) return 0.0;   // 没找到
        return std::exp(-min_d / attract_decay_);
    }

    // 发布 frontier 点云可视化（半透明红色 POINTS）
    void publishBoundaryMarker(const std::vector<BoundaryPoint>& boundaries) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp = this->now();
        m.ns = "frontier";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.03;   // point width (m)
        m.scale.y = 0.03;   // point height (m)
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 0.5f;   // 半透明
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        m.points.reserve(boundaries.size());
        for (const auto& bp : boundaries) {
            geometry_msgs::msg::Point p;
            p.x = bp.coord.x();
            p.y = bp.coord.y();
            p.z = bp.coord.z();
            m.points.push_back(p);
        }
        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(m);
        pub_markers_->publish(arr);
    }

    // 发布当前目标可视化：普通目标 → 黄色 SPHERE
    //                    surrogate 观察目标 → 橙色 SPHERE
    // 传入 nullopt 表示删除。
    void publishGoalMarker(const std::optional<Eigen::Vector3d>& goal,
                           bool is_surrogate = false) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp = this->now();
        m.ns = "goal";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.orientation.w = 1.0;
        if (goal.has_value()) {
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = goal->x();
            m.pose.position.y = goal->y();
            m.pose.position.z = goal->z();
            m.scale.x = 0.10;
            m.scale.y = 0.10;
            m.scale.z = 0.10;
            if (is_surrogate) {
                m.color.r = 1.0f;
                m.color.g = 0.5f;   // 橙色
                m.color.b = 0.0f;
            } else {
                m.color.r = 1.0f;
                m.color.g = 1.0f;   // 黄色
                m.color.b = 0.0f;
            }
            m.color.a = 1.0f;
            m.lifetime = rclcpp::Duration::from_seconds(0.0);
        } else {
            m.action = visualization_msgs::msg::Marker::DELETE;
        }
        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(m);
        pub_markers_->publish(arr);
    }

    bool updateRobotPositionFromTF() {
        try {
            auto t = tf_buffer_->lookupTransform(
                world_frame_, robot_frame_, tf2::TimePointZero,
                tf2::durationFromSec(0.2));
            current_position_.x() = t.transform.translation.x;
            current_position_.y() = t.transform.translation.y;
            current_position_.z() = t.transform.translation.z;
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup %s->%s failed: %s",
                world_frame_.c_str(), robot_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool readyForNewTask() {
        if (!pending_task_id_.has_value()) return true;
        double age = (this->now() - pending_task_sent_at_).seconds();
        if (!pending_seen_feedback_ && age > task_dispatch_timeout_sec_) {
            RCLCPP_WARN(this->get_logger(),
                "Pending task '%s' got no downstream feedback after %.1fs; "
                "assuming it was not accepted and allowing a new task.",
                pending_task_id_.value().c_str(), age);
            pending_task_id_.reset();
            pending_goal_.reset();
            pending_seen_feedback_ = false;
            has_last_goal_ = false;
            return true;
        }
        if (age > task_timeout_sec_) {
            RCLCPP_WARN(this->get_logger(),
                "Pending task '%s' timed out after %.1fs; allowing new task.",
                pending_task_id_.value().c_str(), age);
            pending_task_id_.reset();
            pending_goal_.reset();
            pending_seen_feedback_ = false;
            return true;
        }
        RCLCPP_DEBUG(this->get_logger(),
            "Waiting on '%s' (%.1fs/%.1fs)",
            pending_task_id_.value().c_str(), age, task_timeout_sec_);
        return false;
    }

    bool readyForSelection() {
        auto now_t = this->now();
        if (now_t < duplicate_goal_pause_until_) return false;
        double since_last = (now_t - last_selection_attempt_at_).seconds();
        return since_last >= selection_interval_sec_;
    }

    // ★ APPROACH D (stale-plan fix): after the greedy/HPP-PT selection picks
    //   a candidate but BEFORE we publish the real execute task, run one
    //   more probe against the *current* env_version.  If it fails, the
    //   original probe's OK verdict is stale; skip publish, blacklist the
    //   goal, and let the next cycle pick something else.
    //
    // Returns:
    //   true  = fresh probe succeeded (or feature disabled/no env) → OK to publish
    //   false = fresh probe failed → do NOT publish (caller should skip cycle)
    bool runConfirmProbe(const Eigen::Vector3d& goal,
                          const Eigen::Vector3d& look_at) {
        if (!enable_confirm_probe_) return true;
        if (!latest_env_version_.has_value()) return true;
        int64_t env_v = latest_env_version_.value();

        std::ostringstream pid;
        pid << "cost_probe_" << (++cost_probe_counter_) << "_confirm";
        std::string probe_id = pid.str();
        {
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            CostProbe cp;
            cp.candidate_idx = -1;
            cp.sent_at = this->now();
            cp.cost = std::nullopt;
            cp.succeeded = false;
            cp.goal = goal;
            cost_probes_[probe_id] = cp;
        }
        std::string json = makeTaskJson(
            probe_id, env_v, goal, look_at, world_frame_,
            /*plan_only=*/true,
            moveit_allowed_planning_time_,
            moveit_num_planning_attempts_,
            moveit_position_tolerance_,
            moveit_orientation_tolerance_);
        std_msgs::msg::String probe_msg;
        probe_msg.data = json;
        pub_json_->publish(probe_msg);

        RCLCPP_INFO(this->get_logger(),
            "STAGE 5.5 (confirm probe): sending fresh probe '%s' for "
            "goal=[%.3f,%.3f,%.3f] against env_v=%ld (drift 0s, "
            "timeout %.1fs)",
            probe_id.c_str(), goal.x(), goal.y(), goal.z(),
            static_cast<long>(env_v), confirm_probe_timeout_sec_);

        auto deadline = this->now() +
            rclcpp::Duration::from_seconds(confirm_probe_timeout_sec_);
        while (this->now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            auto it = cost_probes_.find(probe_id);
            if (it != cost_probes_.end() && it->second.cost.has_value()) {
                double c = it->second.cost.value();
                cost_probes_.erase(it);
                bool ok = c < cost_probe_max_cost_ * 0.99;
                RCLCPP_INFO(this->get_logger(),
                    "STAGE 5.5 (confirm probe): '%s' returned cost=%.3f "
                    "-> %s (max=%.3f)",
                    probe_id.c_str(), c,
                    ok ? "OK, publishing" : "FAIL, aborting cycle",
                    cost_probe_max_cost_);
                return ok;
            }
        }
        // timed out (probe result never arrived) → be conservative,
        // treat as failure so we don't publish a task that we no longer
        // have any confidence in.
        {
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            cost_probes_.erase(probe_id);
        }
        RCLCPP_WARN(this->get_logger(),
            "STAGE 5.5 (confirm probe): '%s' timed out after %.1fs; "
            "treating as failure and aborting cycle.",
            probe_id.c_str(), confirm_probe_timeout_sec_);
        return false;
    }

    // ★★★ DIRECTION-FIRST 数据结构 -----------------------------------------
    struct FrontierSeed {
        octomap::point3d centroid;    // frontier 代表点
        Eigen::Vector3d  ud;          // 单位 unknown 主方向
        double           standoff;    // 自适应 standoff
        int              fine_count;  // 覆盖细体素数(仅用于 seed 截断排序)
    };
    struct ViewpointCandidate {
        Eigen::Vector3d  goal;             // 相机位置(已确保几何可达)
        Eigen::Vector3d  look_at;          // 光轴指向
        octomap::point3d frontier;         // 来源 frontier
        octomap::point3d unknown_neighbor; // occlusion 打分用的 unknown 编码点
        bool             is_surrogate = false;
        double           raw_score = 0.0;  // 视点打分(未 clamp)
        ScoreBreakdown   score;            // ★ LOGGER: 分项明细
    };

    // 对一批 frontier seed 扇形展开 look_at 方向, 沿每条方向/延长线用几何法
    // (reach 球面 + workspace + 碰撞 raycast)找可达相机位; 撞障碍/不可达/
    // 向内看/已访问/黑名单则拒绝该方向, 去重后返回全部存活候选。
    std::vector<ViewpointCandidate>
    generateViewpointCandidates(octomap::OcTree& tree,
                                const std::vector<FrontierSeed>& seeds) {
        std::vector<ViewpointCandidate> out;
        Eigen::Vector3d base(base_x_, base_y_, base_z_);
        const double res = tree.getResolution();
        size_t n_dirs = 0, rej_reach = 0, rej_forbid = 0, rej_inward = 0,
               rej_los = 0, rej_visited = 0, rej_blacklist = 0, rej_dup = 0;

        for (const auto& seed : seeds) {
            Eigen::Vector3d F(seed.centroid.x(), seed.centroid.y(),
                              seed.centroid.z());
            Eigen::Vector3d ud0 = seed.ud;
            if (ud0.norm() < 1e-6) {
                ud0 = F - base;                    // 兜底: 朝外看
                if (ud0.norm() < 1e-6) ud0 = Eigen::Vector3d(1, 0, 0);
            }
            ud0.normalize();
            octomap::point3d unknown_enc(
                seed.centroid.x() + static_cast<float>(ud0.x() * res),
                seed.centroid.y() + static_cast<float>(ud0.y() * res),
                seed.centroid.z() + static_cast<float>(ud0.z() * res));
            double standoff = std::clamp(seed.standoff,
                observation_standoff_min_, observation_standoff_max_);
            double effective_min_z = observation_min_z_;
            if (F.z() < observation_min_z_) {
                effective_min_z = std::max(observation_min_z_low_,
                    std::min(observation_min_z_, F.z() - 0.05));
            }

            // ---- 扇形方向集: 主方向 + 绕 right/up 轴的 ±角度偏转 ----
            std::vector<Eigen::Vector3d> dirs;
            dirs.reserve(1 + std::max(1, viewpoint_fan_azimuths_) *
                                 viewpoint_fan_angles_deg_.size());
            dirs.push_back(ud0);
            {
                Eigen::Vector3d helper = (std::abs(ud0.z()) < 0.9)
                    ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
                Eigen::Vector3d right_axis = ud0.cross(helper).normalized();
                Eigen::Vector3d up_axis    = right_axis.cross(ud0).normalized();
                // ★★★ 锥环方位密采样: 每个半锥角 θ 上按 viewpoint_fan_azimuths_
                //   个方位角均匀取方向(cos θ·ud0 + sin θ·(cosφ·right + sinφ·up))。
                //   取代原"只取上/下/左/右 4 个"——覆盖整圈, 能采到"沿轴正对
                //   管口/深腔开口"这类窄窗口, 而不是漏掉。
                const int n_az = std::max(1, viewpoint_fan_azimuths_);
                for (double deg : viewpoint_fan_angles_deg_) {
                    double th = deg * M_PI / 180.0;
                    double ct = std::cos(th), st = std::sin(th);
                    for (int a = 0; a < n_az; ++a) {
                        double phi = 2.0 * M_PI * static_cast<double>(a) / n_az;
                        Eigen::Vector3d d =
                            ct * ud0 +
                            st * (std::cos(phi) * right_axis +
                                  std::sin(phi) * up_axis);
                        dirs.push_back(d.normalized());
                    }
                }
            }
            if (viewpoint_max_directions_ > 0 &&
                static_cast<int>(dirs.size()) > viewpoint_max_directions_) {
                dirs.resize(viewpoint_max_directions_);
            }

            for (const auto& ud : dirs) {
                ++n_dirs;
                // 沿该方向反推相机位: goal = frontier - ud*standoff
                Eigen::Vector3d goal = F - ud * standoff;
                goal.z() = std::clamp(goal.z(), effective_min_z, z_max_);
                // 几何可达: 落在 reach band [reach_min, reach_max] 之外时,
                // 沿视线/延长线投影回 reach 球面。
                double r = (goal - base).norm();
                if (r > reach_max_ || r < reach_min_) {
                    if (r > reach_max_ && !observe_unreachable_) {
                        ++rej_reach; continue;
                    }
                    goal = projectToReachable(goal, ud);
                    goal.z() = std::clamp(goal.z(), effective_min_z, z_max_);
                }
                if (isOutOfWorkspace(goal) || isForbidden(goal)) {
                    ++rej_forbid; continue;
                }
                Eigen::Vector3d look_at = F + ud * (standoff + 1.0);
                if (isInwardLook(goal, look_at)) { ++rej_inward; continue; }
                // ★ 障碍剪枝: 相机→frontier 视线撞 known-occupied → 拒绝该方向
                // ★★★ LOS 统一: 候选生成改用与打分相同的严格 losClear(而非宽松的
                //   isLineOfSightBlocked, 后者 los_min_hit_dist=0.10 只把"贴脸"算挡)。
                //   遮挡后的方向(如格栅后)在生成阶段就剔除, 不再漏进打分 → 消除
                //   "RAW=0 候选每 3 cycle 重复生成 + 触发 sticky 释放"的退化循环。
                if (enable_los_check_ && !hppt_stage::losClear(tree, goal, F)) {
                    ++rej_los; continue;
                }
                if (isBlacklisted(goal) || isPermanentlyBlocked(goal)) {
                    ++rej_blacklist; continue;
                }
                if (isVisited(goal)) { ++rej_visited; continue; }
                // 去重(与已接受候选的相机位过近)
                bool dup = false;
                for (const auto& c : out) {
                    if ((c.goal - goal).norm() < viewpoint_dedup_tol_) {
                        dup = true; break;
                    }
                }
                if (dup) { ++rej_dup; continue; }

                ViewpointCandidate vc;
                vc.goal = goal;
                vc.look_at = look_at;
                vc.frontier = seed.centroid;
                vc.unknown_neighbor = unknown_enc;
                vc.is_surrogate = observe_unreachable_ && isUnreachable(F);
                out.push_back(vc);
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "VIEWPOINT-GEN: %zu seeds -> %zu directions expanded -> %zu "
            "candidates survived (rejected: unreach=%zu forbid=%zu inward=%zu "
            "LOS_blocked=%zu blacklist=%zu visited=%zu dup=%zu).",
            seeds.size(), n_dirs, out.size(),
            rej_reach, rej_forbid, rej_inward, rej_los,
            rej_blacklist, rej_visited, rej_dup);
        return out;
    }

    // ★★★ MOTION-PLAN 探针: 对一个 (goal, look_at) 发一个 plan-only MoveIt 请求,
    //   阻塞等待结果; 返回 true = 规划成功(代价 < max)。用于 top-N 依次尝试、
    //   第一个成功即执行。use_planning_cost 关或无 env → 直接返回 true(无 MoveIt
    //   时信任打分, 送 top-1)。
    bool runPlanOnlyProbe(const Eigen::Vector3d& goal,
                          const Eigen::Vector3d& look_at) {
        if (!use_planning_cost_) return true;
        if (!latest_env_version_.has_value()) return true;
        int64_t env_v = latest_env_version_.value();

        // 前缀必须以 "cost_probe_" 开头, resultCallback 才会把结果路由进
        // cost_probes_ 表(否则被当普通任务结果处理, 探针永远等不到 cost)。
        std::ostringstream pid;
        pid << "cost_probe_mp_" << (++cost_probe_counter_);
        std::string probe_id = pid.str();
        {
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            CostProbe cp;
            cp.candidate_idx = -1;
            cp.sent_at = this->now();
            cp.cost = std::nullopt;
            cp.succeeded = false;
            cp.goal = goal;
            cost_probes_[probe_id] = cp;
        }
        std::string json = makeTaskJson(
            probe_id, env_v, goal, look_at, world_frame_,
            /*plan_only=*/true,
            moveit_allowed_planning_time_,
            moveit_num_planning_attempts_,
            moveit_position_tolerance_,
            moveit_orientation_tolerance_);
        std_msgs::msg::String probe_msg;
        probe_msg.data = json;
        pub_json_->publish(probe_msg);

        auto deadline = this->now() +
            rclcpp::Duration::from_seconds(cost_probe_timeout_sec_);
        while (this->now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            auto it = cost_probes_.find(probe_id);
            if (it != cost_probes_.end() && it->second.cost.has_value()) {
                double c = it->second.cost.value();
                cost_probes_.erase(it);
                return c < cost_probe_max_cost_ * 0.99;
            }
        }
        {
            std::lock_guard<std::mutex> lk(cost_probes_mutex_);
            cost_probes_.erase(probe_id);
        }
        return false;   // 超时: 保守当失败
    }

    // 发布 execute 任务(plan_only=false), 设置 pending 状态与可视化。
    //   返回 true = 已发布。
    bool publishExecuteTask(const Eigen::Vector3d& goal,
                            const Eigen::Vector3d& look_at,
                            bool is_surrogate) {
        if (pub_json_->get_subscription_count() == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "No subscribers on %s; skip task publish for now.",
                task_topic_.c_str());
            return false;
        }
        std::ostringstream task_id_ss;
        task_id_ss << "hppt_view_" << (++task_counter_)
                   << "_" << this->now().nanoseconds();
        std::string task_id = task_id_ss.str();

        std::string json = makeTaskJson(
            task_id, latest_env_version_.value(),
            goal, look_at, world_frame_, false,
            moveit_allowed_planning_time_, moveit_num_planning_attempts_,
            moveit_position_tolerance_, moveit_orientation_tolerance_);
        std_msgs::msg::String msg_out;
        msg_out.data = json;
        pub_json_->publish(msg_out);

        pending_task_id_ = task_id;
        pending_task_sent_at_ = this->now();
        pending_goal_ = goal;
        pending_seen_feedback_ = false;
        last_goal_ = goal;
        has_last_goal_ = true;
        pending_task_env_version_ = latest_env_version_.value();

        publishGoalMarker(goal, is_surrogate);
        RCLCPP_INFO(this->get_logger(),
            "STAGE 6 (publish): task '%s' -> goal [%.3f, %.3f, %.3f]%s "
            "(MoveIt allow=%.1fs x %d attempts, subscribers=%zu). Waiting "
            "for /team/motion_result_json.",
            task_id.c_str(), goal.x(), goal.y(), goal.z(),
            is_surrogate ? "  (SURROGATE - observing unreachable frontier)" : "",
            moveit_allowed_planning_time_, moveit_num_planning_attempts_,
            pub_json_->get_subscription_count());
        return true;
    }

    // ★★★ EVENT-DRIVEN: 只派发一个真实 execute 任务(plan_only=false)后立即返回,
    //   绝不阻塞等待。真实结果由 resultCallback 异步处理(成功→下一cycle;失败→
    //   dispatchNextGreedyCandidate 标 inf 重排)。返回 true=已派发,false=无订阅者。
    bool dispatchExecuteOnly(const Eigen::Vector3d& goal,
                             const Eigen::Vector3d& look_at) {
        if (pub_json_->get_subscription_count() == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "No subscribers on %s; cannot dispatch execute.",
                task_topic_.c_str());
            return false;
        }
        std::ostringstream task_id_ss;
        task_id_ss << "hppt_view_" << (++task_counter_)
                   << "_" << this->now().nanoseconds();
        std::string task_id = task_id_ss.str();

        // pending 状态:供 resultCallback 匹配 + readyForNewTask 门控。
        pending_task_id_ = task_id;
        pending_task_sent_at_ = this->now();
        pending_goal_ = goal;
        pending_seen_feedback_ = false;
        last_goal_ = goal;
        has_last_goal_ = true;
        pending_task_env_version_ = latest_env_version_.value_or(0);

        std::string json = makeTaskJson(
            task_id, latest_env_version_.value_or(0),
            goal, look_at, world_frame_, /*plan_only=*/false,
            moveit_allowed_planning_time_, moveit_num_planning_attempts_,
            moveit_position_tolerance_, moveit_orientation_tolerance_);
        std_msgs::msg::String msg_out;
        msg_out.data = json;
        pub_json_->publish(msg_out);
        publishGoalMarker(goal, false);
        {
            std::ostringstream o;
            o << "STAGE 6 (dispatch): task '" << task_id << "' -> goal ["
              << std::fixed << std::setprecision(3) << goal.x() << ", "
              << goal.y() << ", " << goal.z() << "]  event-driven, waiting REAL "
              << "result (no timeout; motion_planner reports success/failure).";
            hpptLog(o.str());
        }
        return true;
    }

    // ★★★ EVENT-DRIVEN: 选 utility 最高的未失败候选并派发真实 execute(不阻塞)。
    //   由 runGreedyBaselinePipeline(cycle 开始)和 resultCallback(失败重排)调用。
    //   没有可选候选(全失败/不可达/达到 limit)→ 结束本 cycle,declareConverged。
    void dispatchNextGreedyCandidate() {
        using namespace hppt_baseline;
        Eigen::Vector3d goal, look_at;
        bool converge = false;
        bool have_candidate = false;
        {
            std::lock_guard<std::mutex> lk(greedy_mutex_);
            if (!greedy_cycle_active_) return;
            // ★ PAUSE: 暂停时不派发下一个候选。当前正在执行的 execute 已由
            //   resultCallback 处理完(到达),此处结束本 cycle,机械臂停在原地。
            //   继续后由下一帧 OctoMap 触发新 cycle。
            if (paused_.load()) {
                greedy_cycle_active_ = false;
                greedy_current_idx_ = -1;
                RCLCPP_INFO(this->get_logger(),
                    "EXPLORATION PAUSED: current task finished, not dispatching next.");
                return;
            }
            int best = -1;
            if (greedy_attempt_ < greedy_limit_) {
                normalizeMotionCosts(greedy_reps_, greedy_gp_.epsilon);
                best = selectBestByUtility(greedy_reps_, greedy_failed_, greedy_gp_);
            }
            if (best < 0) {
                greedy_cycle_active_ = false;
                greedy_current_idx_ = -1;
                converge = true;
            } else {
                publishBaselineRankedViz(greedy_reps_);
                ++greedy_attempt_;
                greedy_current_idx_ = best;
                logGreedyRanking(greedy_reps_, greedy_failed_, greedy_attempt_);
                goal = greedy_reps_[best].camera_position;
                look_at = greedy_reps_[best].look_at;
                have_candidate = true;
                // ★ TEMP SELECT-DIAG: 打印选中候选的真实几何(聚合后 frontier_point
                //   = 成员 frontier 均值),用于诊断"高U零V反复选"的成因。
                {
                    const auto& rr = greedy_reps_[best];
                    std::ostringstream od;
                    od << std::fixed << std::setprecision(3)
                       << "SELECT-DIAG: node=" << rr.node_id << " f=" << rr.frontier_id
                       << " cam[" << rr.camera_position.x() << "," << rr.camera_position.y()
                       << "," << rr.camera_position.z()
                       << "] Fmean[" << rr.frontier_point.x() << "," << rr.frontier_point.y()
                       << "," << rr.frontier_point.z()
                       << "] look[" << rr.look_at.x() << "," << rr.look_at.y()
                       << "," << rr.look_at.z()
                       << "] vdir[" << rr.view_direction.x() << "," << rr.view_direction.y()
                       << "," << rr.view_direction.z()
                       << "] U=" << rr.unknown_density << " V=" << rr.geometric_visibility;
                    hpptLog(od.str());
                }
            }
        }
        if (converge) {
            hpptLog("BASELINE: no executable candidate this cycle "
                    "(all execute-failed / unreachable).");
            releaseStickyBlacklist("BASELINE EMPTY");
            declareConverged("BASELINE ALL EXECUTE FAILED");
            return;
        }
        if (have_candidate) {
            if (!dispatchExecuteOnly(goal, look_at)) {
                // 无订阅者:当作本次失败,标 inf 并继续重排。
                onGreedyExecuteFailed("no_subscriber");
            }
        }
    }

    // ★★★ EVENT-DRIVEN: 当前派发的候选执行/规划失败 → 本 cycle 把它 motion_cost
    //   标 Infinity、加入 failed,然后派发下一个候选(同 cycle 重排,不跨 cycle)。
    void onGreedyExecuteFailed(const std::string& error_type) {
        int node_id = -1;
        int attempt = 0;
        {
            std::lock_guard<std::mutex> lk(greedy_mutex_);
            if (!greedy_cycle_active_ || greedy_current_idx_ < 0 ||
                greedy_current_idx_ >= static_cast<int>(greedy_reps_.size())) {
                return;
            }
            auto& r = greedy_reps_[greedy_current_idx_];
            node_id = r.node_id;
            attempt = greedy_attempt_;
            greedy_failed_.push_back(r.node_id);
            r.edge_feasible = false;
            r.motion_cost = hppt_baseline::kInf;
        }
        {
            std::ostringstream o;
            o << "BASELINE B8: node " << node_id << " EXECUTE failed (attempt "
              << attempt << ", error_type='" << error_type
              << "') -> edge=Infinity this cycle, reselect.";
            hpptLog(o.str());
        }
        dispatchNextGreedyCandidate();
    }

    // ★★★ EVENT-DRIVEN: 当前派发的候选真实到达(execute success,只在到达后返回)
    //   → 结束本 cycle,下个 OctoMap 触发新 cycle(从生成 frontier 重新开始)。
    void onGreedyExecuteSucceeded() {
        int node_id = -1;
        int attempt = 0;
        {
            std::lock_guard<std::mutex> lk(greedy_mutex_);
            if (greedy_current_idx_ >= 0 &&
                greedy_current_idx_ < static_cast<int>(greedy_reps_.size())) {
                node_id = greedy_reps_[greedy_current_idx_].node_id;
            }
            attempt = greedy_attempt_;
            greedy_cycle_active_ = false;
            greedy_current_idx_ = -1;
        }
        consecutive_empty_ = 0;
        converged_announced_ = false;
        std::ostringstream o;
        o << "BASELINE B8: node " << node_id << " EXECUTE success (attempt "
          << attempt << ") -> arm arrived, cycle done.";
        hpptLog(o.str());
    }

    // ★★★ LOGGER: 统一写日志。始终镜像到 rclcpp INFO;若 decision-log 文件
    //   打开成功则同时追加落盘(带 ROS 时间戳)。多行文本(如排名表)整块传入。
    //   线程安全: 文件写用 hppt_log_mutex_ 保护(octomapCallback 与 resultCallback
    //   在不同 callback group, 可能并发)。
    void hpptLog(const std::string& text) {
        RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
        if (!hppt_log_to_file_) return;
        std::lock_guard<std::mutex> lk(hppt_log_mutex_);
        if (!hppt_log_ofs_.is_open()) return;
        hppt_log_ofs_ << "[" << std::fixed << std::setprecision(3)
                      << this->now().seconds() << "] " << text << "\n";
        hppt_log_ofs_.flush();
    }

    // ★★★ LOGGER: 唯一的黑名单写入口。记录到 blacklist_(+可选 sticky), 并把
    //   【类型 + 原因 + 位置 + 半径】写进 decision log。所有拉黑动作都应走这里,
    //   保证"所有被拉进黑名单的点"都被记录。
    //   permanent=true  → 写 regular blacklist_（本 session 永不释放，连
    //     releaseStickyBlacklist 也不动）——用于"非保守也直接失败"的死点。
    //   permanent=false → 写 sticky_blacklist_（STAGE EMPTY 时释放、可重试）
    //     ——用于"仅被 unknown 挡住 / 超时未找到"的临时点。
    //   两者半径都应传一小块（block_radius_small_），只封失败点本身附近，
    //   不覆盖整个 cluster。
    void blacklistPoint(const Eigen::Vector3d& p, double radius,
                        const char* type, const char* reason,
                        bool permanent) {
        size_t reg_n = 0, stk_n = 0;
        {
            std::lock_guard<std::mutex> lk(blacklist_mutex_);
            if (permanent) blacklist_.push_back({p, this->now(), radius});
            else           sticky_blacklist_.push_back({p, this->now(), radius});
            reg_n = blacklist_.size();
            stk_n = sticky_blacklist_.size();
        }
        // ★★★ BL-DIAG: 记录加入事件 + 加入后两个列表的实际计数(供橙球消失诊断)。
        char buf[300];
        std::snprintf(buf, sizeof(buf),
            "BLACKLIST-ADD [type=%s | reason=%s | %s] pos=(%.3f,%.3f,%.3f) r=%.2f "
            "-> now regular=%zu sticky=%zu",
            type, reason, permanent ? "PERMANENT" : "temporary",
            p.x(), p.y(), p.z(), radius, reg_n, stk_n);
        hpptLog(std::string(buf));
    }

    // ★★★ NON-CONSERVATIVE FALLBACK: 由 motion planner 回报的 error_type 决定
    //   拉黑等级。hard_unreachable = 非保守也直接失败(IK/碰撞/无效计划) → 永久;
    //   blocked_by_unknown / *_timeout / *_probe_failed / not_known_free /
    //   execution_failure / moveit_planning_failure = 临时; 其余(基础设施类
    //   retryable: env 版本不匹配 / move_group 超时不可用等) → 不拉黑, 下轮重试。
    enum class BlockKind { None, Temporary, Permanent };
    BlockKind classifyBlock(const std::string& et, bool success) {
        if (success) return BlockKind::None;
        if (et == "hard_unreachable") return BlockKind::Permanent;
        if (et == "blocked_by_unknown" || et == "nonconservative_timeout"
            || et == "nonconservative_probe_failed"
            || et == "trajectory_not_known_free"
            || et == "execution_failure"
            || et == "moveit_planning_failure") {
            return BlockKind::Temporary;
        }
        return BlockKind::None;
    }
    // 便捷封装: 按 error_type 分级拉黑到一小块范围。
    void blacklistByErrorType(const Eigen::Vector3d& p,
                              const std::string& error_type,
                              bool success, const char* where) {
        BlockKind k = classifyBlock(error_type, success);
        if (k == BlockKind::None) return;
        blacklistPoint(p, block_radius_small_,
                       where,
                       error_type.empty() ? "unspecified" : error_type.c_str(),
                       /*permanent=*/(k == BlockKind::Permanent));
    }

    // ★★★ LOGGER: 打完分、按分数降序排序后, 逐条记录方向。
    //   每行格式:【位置(相机 goal), 方向(光轴单位向量), 各项评分】。
    //   各项评分 = obs/unk/occl/sem/ig/peek/attract + los/igate 门控标志 + RAW 总分。
    void logScoredDirections(const std::vector<ViewpointCandidate>& cands) {
        std::ostringstream oss;
        oss << "==== HPPT SCORED DIRECTIONS (" << cands.size()
            << " 条, 按 RAW 分数降序) ====\n";
        oss << "  rank |        位置 goal(x,y,z)        |       方向 dir(x,y,z)"
               "        |   obs   unk  occl   sem    ig  peek  attr | los igate |"
               "   RAW\n";
        for (size_t i = 0; i < cands.size(); ++i) {
            const auto& vc = cands[i];
            Eigen::Vector3d dir = vc.look_at - vc.goal;
            if (dir.norm() > 1e-9) dir.normalize();
            const auto& s = vc.score;
            char line[360];
            std::snprintf(line, sizeof(line),
                "  #%2zu | (%+6.2f,%+6.2f,%+6.2f) | (%+5.2f,%+5.2f,%+5.2f) | "
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f |  %d    %d  | %.3f\n",
                i, vc.goal.x(), vc.goal.y(), vc.goal.z(),
                dir.x(), dir.y(), dir.z(),
                s.obs, s.unk, s.occl, s.sem, s.ig, s.peek, s.attract,
                s.los_clear ? 1 : 0, s.ig_gate_killed ? 1 : 0, s.raw);
            oss << line;
        }
        hpptLog(oss.str());
    }

    // ★★★ CONVERGENCE: 一个"未产出目标"的选点周期结束时调用。若本 cycle octomap
    //   有实质增长(相机还在发现)→ 重置计数; 否则累加。达到阈值 → 判定收敛。
    void handleUnproductiveCycle(const char* stage) {
        if (!enable_convergence_detection_) return;
        if (converged_announced_) return;              // 本轮已宣告过
        if (octomap_grew_this_cycle_) {                // 地图仍在长 → 不算收敛
            consecutive_empty_ = 0;
            return;
        }
        ++consecutive_empty_;
        RCLCPP_INFO(this->get_logger(),
            "CONVERGENCE watch: unproductive cycle %d/%d at %s "
            "(octomap stable at %zu nodes).",
            consecutive_empty_, convergence_empty_cycles_, stage,
            last_octomap_size_);
        if (consecutive_empty_ >= convergence_empty_cycles_) {
            declareConverged(stage);
        }
    }

    void declareConverged(const char* stage) {
        converged_announced_ = true;
        RCLCPP_WARN(this->get_logger(),
            "=========== exploration converged: %d consecutive unproductive "
            "cycles with a stable OctoMap (last stage: %s, %zu nodes) — no "
            "reachable frontier left to explore. ===========",
            consecutive_empty_, stage, last_octomap_size_);
        std_msgs::msg::Bool done;
        done.data = true;
        pub_exploration_complete_->publish(done);
        RCLCPP_INFO(this->get_logger(),
            "Published exploration-complete on %s.",
            exploration_complete_topic_.c_str());
        if (convergence_shutdown_) {
            RCLCPP_WARN(this->get_logger(),
                "convergence_shutdown=true -> requesting rclcpp::shutdown().");
            rclcpp::shutdown();
        }
    }

    // ★★★ RANKED VIZ: 发布本 cycle 排名前 5 的视角(相机位 SPHERE + 视线 ARROW)。
    //   #0(最高分)=绿且更大, #1-4=青。固定 id 0-4, 不足 5 个的槽位发 DELETE。
    //   lifetime=0 → 不更新则永久留存。sorted 需已按分数降序。
    void publishRankedViewpointMarkers(
            const std::vector<ViewpointCandidate>& sorted) {
        if (!enable_ranked_viz_) return;
        visualization_msgs::msg::MarkerArray arr;
        const int take = std::min<int>(5, static_cast<int>(sorted.size()));
        auto stamp = this->now();
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker s;   // camera-position sphere
            s.header.frame_id = world_frame_; s.header.stamp = stamp;
            s.ns = "hppt_top_vp"; s.id = i;
            s.type = visualization_msgs::msg::Marker::SPHERE;
            s.pose.orientation.w = 1.0;
            visualization_msgs::msg::Marker a;   // look-direction arrow
            a.header = s.header; a.ns = "hppt_top_arrow"; a.id = i;
            a.type = visualization_msgs::msg::Marker::ARROW;
            a.pose.orientation.w = 1.0;
            if (i < take) {
                const auto& vc = sorted[i];
                const bool best = (i == 0);
                s.action = visualization_msgs::msg::Marker::ADD;
                s.pose.position.x = vc.goal.x();
                s.pose.position.y = vc.goal.y();
                s.pose.position.z = vc.goal.z();
                double d = best ? 0.07 : 0.045;
                s.scale.x = s.scale.y = s.scale.z = d;
                // #0 绿, 其余青
                s.color.r = best ? 0.10f : 0.10f;
                s.color.g = best ? 1.00f : 0.75f;
                s.color.b = best ? 0.20f : 1.00f;
                s.color.a = best ? 1.00f : 0.90f;
                a.action = visualization_msgs::msg::Marker::ADD;
                Eigen::Vector3d dir = vc.look_at - vc.goal;
                if (dir.norm() > 1e-6) dir.normalize();
                Eigen::Vector3d end = vc.goal + dir * 0.22;
                geometry_msgs::msg::Point p0, p1;
                p0.x = vc.goal.x(); p0.y = vc.goal.y(); p0.z = vc.goal.z();
                p1.x = end.x();     p1.y = end.y();     p1.z = end.z();
                a.points.push_back(p0); a.points.push_back(p1);
                a.scale.x = 0.012;  // shaft diameter
                a.scale.y = 0.025;  // head diameter
                a.scale.z = 0.03;   // head length
                a.color = s.color;
            } else {
                s.action = visualization_msgs::msg::Marker::DELETE;
                a.action = visualization_msgs::msg::Marker::DELETE;
            }
            arr.markers.push_back(s);
            arr.markers.push_back(a);
        }
        pub_viz_->publish(arr);
    }

    // ★★★ RANKED VIZ: 发布所有被拉黑的区域(SPHERE, 直径=2×半径)。
    //   永久(regular blacklist_)=红, 临时(sticky_blacklist_)=橙, 半透明。
    //   id 连续编号; 数量减少时对多余 id 发 DELETE。lifetime=0 永久留存。
    void publishBlacklistMarkers() {
        if (!enable_ranked_viz_) return;
        std::vector<BlacklistEntry> reg, stk;
        {
            std::lock_guard<std::mutex> lk(blacklist_mutex_);
            reg = blacklist_;
            stk = sticky_blacklist_;
        }
        visualization_msgs::msg::MarkerArray arr;
        auto stamp = this->now();
        int id = 0;
        auto add = [&](const BlacklistEntry& e, bool permanent) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = world_frame_; m.header.stamp = stamp;
            m.ns = "hppt_blacklist"; m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.pose.position.x = e.pos.x();
            m.pose.position.y = e.pos.y();
            m.pose.position.z = e.pos.z();
            double r = (e.radius > 0.0) ? e.radius : blacklist_radius_;
            m.scale.x = m.scale.y = m.scale.z = 2.0 * r;
            if (permanent) { m.color.r = 0.90f; m.color.g = 0.10f; m.color.b = 0.10f; }
            else           { m.color.r = 1.00f; m.color.g = 0.60f; m.color.b = 0.00f; }
            m.color.a = 0.35f;
            arr.markers.push_back(m);
        };
        for (const auto& e : reg) add(e, true);
        for (const auto& e : stk) add(e, false);
        int add_markers = id;   // ADD marker 数 = 实际 regular+sticky 条目数
        // 数量减少 → 对多余的旧 id 发 DELETE
        int del_markers = 0;
        for (int i = id; i < last_bl_marker_count_; ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = world_frame_; m.header.stamp = stamp;
            m.ns = "hppt_blacklist"; m.id = i;
            m.action = visualization_msgs::msg::Marker::DELETE;
            arr.markers.push_back(m);
            ++del_markers;
        }
        int prev_count = last_bl_marker_count_;
        last_bl_marker_count_ = id;
        if (!arr.markers.empty()) pub_viz_->publish(arr);
        // ★★★ BL-DIAG: viz 与 actual 对照。add_markers 应恒等于 regular+sticky;
        //   若 RViz 里橙球比这少 → case 1(viz/传输不一致); 若这里 sticky 本身
        //   变少 → case 2(实际 sticky 被移除, 见 BL-REMOVE)。
        char buf[200];
        std::snprintf(buf, sizeof(buf),
            "BL-VIZ actual regular=%zu sticky=%zu (total=%zu) | markers ADD=%d "
            "DELETE=%d prev=%d",
            reg.size(), stk.size(), reg.size() + stk.size(),
            add_markers, del_markers, prev_count);
        hpptLog(std::string(buf));
    }

    // ★★★ BL-DIAG: 每 cycle 记录黑名单实际全量列表(regular=永久/红, sticky=临时/橙),
    //   供离线判断橙球消失是"实际被移除"(case 2, 对照 BL-REMOVE)还是"viz 不一致"
    //   (case 1, 对照 BL-VIZ)。
    void logBlacklistSnapshot(const char* where) {
        std::vector<BlacklistEntry> reg, stk;
        {
            std::lock_guard<std::mutex> lk(blacklist_mutex_);
            reg = blacklist_;
            stk = sticky_blacklist_;
        }
        std::ostringstream oss;
        oss << "BL-SNAPSHOT [" << where << "] regular=" << reg.size()
            << " sticky=" << stk.size() << " | reg:";
        for (const auto& e : reg) {
            char b[48];
            std::snprintf(b, sizeof(b), " (%.2f,%.2f,%.2f)",
                e.pos.x(), e.pos.y(), e.pos.z());
            oss << b;
        }
        oss << " | sticky:";
        for (const auto& e : stk) {
            char b[48];
            std::snprintf(b, sizeof(b), " (%.2f,%.2f,%.2f)",
                e.pos.x(), e.pos.y(), e.pos.z());
            oss << b;
        }
        hpptLog(oss.str());
    }

    // ========================================================================
    // ★★★ NEW_METHOD: Greedy Baseline —— IK 服务 / 打分 / 选择 / 派发
    // ========================================================================

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(joint_state_mutex_);
        latest_joint_state_ = *msg;
        have_joint_state_ = true;
    }

    // 从 IK 解里按 planning group 关节名(joint_1..joint_6)顺序抽取关节角。
    std::vector<double> extractGroupJoints(const sensor_msgs::msg::JointState& js) {
        static const std::array<std::string, 6> kNames = {
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        std::vector<double> out;
        for (const auto& want : kNames) {
            bool found = false;
            for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
                if (js.name[i] == want) { out.push_back(js.position[i]); found = true; break; }
            }
            if (!found) return {};   // 缺关节 → 无效
        }
        return out;
    }

    // ★ Phase 3: 对一个相机位姿求 IK(avoid_collisions=true → 同时做自碰撞/环境碰撞/
    //   关节限位校验)。以当前 /joint_states 为种子。成功返回 true 并填 out_joints。
    //   在 octomapCallback(default group)内阻塞轮询 future;响应由 ik_cb_group_ 的
    //   另一执行器线程处理,不死锁。
    bool computeIK(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q,
                   const sensor_msgs::msg::JointState& seed,
                   std::vector<double>& out_joints) {
        if (!compute_ik_client_->service_is_ready()) {
            if (!compute_ik_client_->wait_for_service(std::chrono::milliseconds(200))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "/compute_ik service not available; IK filter degraded.");
                return false;
            }
        }
        auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        auto& ik = req->ik_request;
        ik.group_name = planning_group_;
        ik.ik_link_name = ik_link_name_;
        ik.avoid_collisions = true;
        ik.pose_stamped.header.frame_id = world_frame_;
        ik.pose_stamped.pose.position.x = pos.x();
        ik.pose_stamped.pose.position.y = pos.y();
        ik.pose_stamped.pose.position.z = pos.z();
        ik.pose_stamped.pose.orientation.x = q.x();
        ik.pose_stamped.pose.orientation.y = q.y();
        ik.pose_stamped.pose.orientation.z = q.z();
        ik.pose_stamped.pose.orientation.w = q.w();
        // 种子:当前关节(is_diff=true → 缺失关节用 move_group 当前状态补全)
        ik.robot_state.joint_state = seed;
        ik.robot_state.is_diff = true;
        ik.timeout = rclcpp::Duration::from_seconds(ik_timeout_sec_);

        auto future = compute_ik_client_->async_send_request(req);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(ik_timeout_sec_ + 0.5));
        while (std::chrono::steady_clock::now() < deadline) {
            if (future.wait_for(std::chrono::milliseconds(2)) ==
                std::future_status::ready) {
                auto resp = future.get();
                if (resp->error_code.val !=
                    moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    return false;
                }
                out_joints = extractGroupJoints(resp->solution.joint_state);
                return !out_joints.empty();
            }
        }
        // 超时:移除挂起请求防泄漏
        compute_ik_client_->remove_pending_request(future);
        return false;
    }

    // ★ Phase 5(可选):显式 target-state validity 校验。默认关(compute_ik
    //   avoid_collisions 已覆盖)。
    bool checkStateValid(const std::vector<double>& joints) {
        if (!state_validity_client_->service_is_ready()) {
            if (!state_validity_client_->wait_for_service(std::chrono::milliseconds(200)))
                return true;   // 服务不可用 → 不因此丢弃(交给 compute_ik 保证)
        }
        auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        req->group_name = planning_group_;
        req->robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3",
                                             "joint_4", "joint_5", "joint_6"};
        req->robot_state.joint_state.position = joints;
        req->robot_state.is_diff = false;
        auto future = state_validity_client_->async_send_request(req);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(300);
        while (std::chrono::steady_clock::now() < deadline) {
            if (future.wait_for(std::chrono::milliseconds(2)) ==
                std::future_status::ready) {
                return future.get()->valid;
            }
        }
        state_validity_client_->remove_pending_request(future);
        return true;   // 超时不误杀
    }

    // ★ Phase 7 日志:代表视点排名表。
    void logGreedyRanking(const std::vector<hppt_baseline::RepresentativeViewpoint>& reps,
                          const std::vector<int>& failed, int attempt) {
        std::vector<int> order(reps.size());
        for (size_t i = 0; i < reps.size(); ++i) order[i] = (int)i;
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return reps[a].greedy_utility > reps[b].greedy_utility;
        });
        auto is_failed = [&](int id) {
            return std::find(failed.begin(), failed.end(), id) != failed.end();
        };
        std::ostringstream o;
        o << "BASELINE B7 GREEDY RANKING (attempt " << attempt << ", "
          << reps.size() << " reps):\n";
        int rank = 0;
        for (int idx : order) {
            const auto& r = reps[idx];
            o << "  #" << rank++ << " node=" << r.node_id << " f=" << r.frontier_id
              << " pos[" << std::fixed << std::setprecision(3)
              << r.camera_position.x() << "," << r.camera_position.y() << ","
              << r.camera_position.z() << "]"
              << " score=" << std::setprecision(3) << r.viewpoint_score
              // ★ 分数三分量(归一/加权前的原始值,各∈[0,1]):
              //   score = w_U·U + w_V·V + w_S·S
              << "(U=" << r.unknown_density
              << " V=" << r.geometric_visibility
              << " S=" << r.semantic_prior << ")"
              // ★ Acost=归一前 A* 原始代价(m);ncost=min-max 归一后∈[0,1]
              << " Acost=" << (std::isfinite(r.motion_cost)
                               ? std::to_string(r.motion_cost) : std::string("INF"))
              << " ncost=" << std::setprecision(3) << r.normalized_motion_cost
              << " util=" << std::setprecision(4) << r.greedy_utility
              << " reach=" << (r.edge_feasible ? "Y" : "N")
              << (is_failed(r.node_id) ? " [FAILED]" : "") << "\n";
        }
        hpptLog(o.str());
    }

    // 复用 ranked marker:用代表视点(★按 greedy_utility 降序 = 实际选择/尝试顺序)
    //   构造 ViewpointCandidate 代理。#0(绿)= 真正会被优先选中的候选,与决策一致。
    //   前置条件:调用前 reps[*].greedy_utility 已被 selectBestByUtility 填充。
    void publishBaselineRankedViz(
            const std::vector<hppt_baseline::RepresentativeViewpoint>& reps) {
        if (!enable_ranked_viz_) return;
        std::vector<int> order(reps.size());
        for (size_t i = 0; i < reps.size(); ++i) order[i] = static_cast<int>(i);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return reps[a].greedy_utility > reps[b].greedy_utility;
        });
        std::vector<ViewpointCandidate> proxy;
        proxy.reserve(reps.size());
        for (int i : order) {
            const auto& r = reps[i];
            ViewpointCandidate vc;
            vc.goal = r.camera_position;
            vc.look_at = r.look_at;
            vc.frontier = octomap::point3d(r.camera_position.x(), r.camera_position.y(),
                                           r.camera_position.z());
            vc.is_surrogate = false;
            vc.raw_score = r.greedy_utility;   // 记录 utility(marker 只按顺序着色)
            proxy.push_back(vc);
        }
        publishRankedViewpointMarkers(proxy);
    }

    // ★★★ Phase 2/4/5/6/7/8: Greedy Baseline 主流水线。
    void runGreedyBaselinePipeline(octomap::OcTree& tree,
                                   const std::vector<BoundaryPoint>& boundaries) {
        using namespace hppt_baseline;
        const int64_t env_v = latest_env_version_.value();
        const Eigen::Vector3d base(base_x_, base_y_, base_z_);

        // ---------- B1: seeds —— 直接用每个 frontier 体素(★不聚类、★无降数)----------
        //   每个 frontier 体素 = 一个 seed(coord + 它自己那面的 unknown 方向)。
        //   若没有 frontier → STAGE 1 已提前返回(不行动)。
        //   ⚠️ 目前无降数:seed 数 = frontier 体素数(可能上万)→ 候选/打分会爆炸。
        //      降数机制待定(见后续讨论/流程讲解中标注的位置)。
        struct Seed { Eigen::Vector3d F, ud; int frontier_id; };
        std::vector<Seed> seeds;
        int fid = 0;
        for (const auto& bp : boundaries) {
            Eigen::Vector3d F(bp.coord.x(), bp.coord.y(), bp.coord.z());
            Eigen::Vector3d ud(bp.unknown_neighbor.x() - bp.coord.x(),
                               bp.unknown_neighbor.y() - bp.coord.y(),
                               bp.unknown_neighbor.z() - bp.coord.z());
            if (ud.norm() < 1e-6) ud = F - base;   // 兜底方向:朝外
            seeds.push_back({F, ud, fid++});
        }
        if (seeds.empty()) { handleUnproductiveCycle("BASELINE B1 EMPTY"); return; }

        // ---------- B2: 多方向×多距离生成(仅几何预剪,不在此 cap IK 预算)----------
        WorkspaceLimits wl; wl.base = base; wl.reach_min = reach_min_;
        wl.reach_max = reach_max_; wl.z_min = z_min_; wl.z_max = z_max_;
        SphericalShellWorkspace ws(wl);
        std::vector<RawViewpointCandidate> raws;
        int cand_id = 0;
        size_t gen_total = 0, occ_checks = 0;
        size_t rej_geo = 0, rej_unknown = 0;
        size_t rej_occupied_trunc = 0, capped = 0;
        size_t rej_near_edge = 0;
        const size_t raw_cap = (max_raw_candidates_ > 0)
            ? static_cast<size_t>(max_raw_candidates_) : SIZE_MAX;
        bool cap_hit = false;
        for (auto& s : seeds) {
            if (cap_hit) break;
            auto dirs = generateViewDirections(s.ud, viewpoint_fan_angles_deg_,
                                               viewpoint_fan_azimuths_);
            if (viewpoint_max_directions_ > 0 &&
                (int)dirs.size() > viewpoint_max_directions_)
                dirs.resize(viewpoint_max_directions_);
            for (auto& d : dirs) {
                if (cap_hit) break;
                // ★ 双步长射线采样:
                //   射线定义为 P(t)=F-t*dir, t>=0, 即从原始 frontier F 朝
                //   free-space / 相机侧往外走。先和 reach shell 求交得到
                //   [t0,t1],只在该区间内按 sample_step(默认 0.15m)生成
                //   candidate；但 occupied 检查必须从 t=0 开始按
                //   collision_step(默认 0.03m)沿整条射线 march。
                //   一旦 F->candidate 的任意位置遇到 occupied,立即截断
                //   当前方向，后续更远 candidate 全部不生成。
                Eigen::Vector3d dir = d;
                if (dir.norm() < 1e-9) continue;
                dir.normalize();
                auto seg = ws.intersectViewLine(s.F, dir);
                if (!seg) continue;
                const double sample_step = std::max(1e-6, viewpoint_line_sample_step_);
                const double collision_step = std::max(1e-6, viewpoint_line_collision_step_);
                const double t0 = seg->first;
                const double t1 = seg->second;
                bool truncated = false;
                double checked_until = 0.0;
                for (double candidate_t = t0 + sample_step;
                     candidate_t <= t1 + 1e-9;
                     candidate_t += sample_step) {
                    // ★ 改动A: candidate_t = 相机到 frontier F 的距离。超过观测距离
                    //   上限(默认 3m, 匹配 sensor max_range)相机物理上看不到该 frontier
                    //   的 unknown → 停止该方向后续更远的采样点(candidate_t 递增)。
                    if (candidate_t > viewpoint_max_observe_distance_ + 1e-9) break;
                    // 先用更密的步长检查 frontier F 到当前候选点之间是否穿过 occupied。
                    // 注意：这里不能用 ws.contains(check_pos) 跳过 workspace 外的段；
                    // 书柜背板可能正位于 frontier 到 reach shell 入口之间。
                    for (double check_t = checked_until + collision_step;
                         check_t <= candidate_t + 1e-9;
                         check_t += collision_step) {
                        Eigen::Vector3d check_pos = s.F - check_t * dir;
                        ++occ_checks;
                        octomap::OcTreeNode* nd_check = tree.search(
                            octomap::point3d(static_cast<float>(check_pos.x()),
                                             static_cast<float>(check_pos.y()),
                                             static_cast<float>(check_pos.z())));
                        if (nd_check != nullptr && tree.isNodeOccupied(nd_check)) {
                            ++rej_occupied_trunc;
                            truncated = true;
                            break;
                        }
                    }
                    if (truncated) break;
                    checked_until = candidate_t;

                    Eigen::Vector3d pos = s.F - candidate_t * dir;
                    ++gen_total;
                    // 纯几何工作空间过滤 isForbidden / isOutOfWorkspace。
                    if (isForbidden(pos) || isOutOfWorkspace(pos)) {
                        ++rej_geo; continue;
                    }
                    // ★ 只从 unknown 块外部观察:相机位必须是 known-free。
                    //   unknown=search 返回 null → 丢弃该点但继续沿线搜索；
                    //   occupied → 截断整条采样线，避免障碍物后方继续产生候选。
                    octomap::OcTreeNode* nd = tree.search(
                        octomap::point3d(static_cast<float>(pos.x()),
                                         static_cast<float>(pos.y()),
                                         static_cast<float>(pos.z())));
                    if (nd == nullptr) {
                        ++rej_unknown; continue;
                    }
                    if (tree.isNodeOccupied(nd)) {
                        ++rej_occupied_trunc; break;
                    }
                    // ★ 改动B: 选点须距 free 段两端(unknown/occupied 边界或 reach 区间
                    //   端点)>= endpoint_clearance(默认 0.1m),避免在被 unknown 夹住的
                    //   短 free 段里选出紧贴 unknown 的观察点。
                    {
                        bool near_edge = false;
                        const double clr = viewpoint_endpoint_clearance_;
                        for (double sgn : {-1.0, 1.0}) {
                            const double et = candidate_t + sgn * clr;
                            if (et < t0 - 1e-9 || et > t1 + 1e-9) { near_edge = true; break; }
                            Eigen::Vector3d epos = s.F - et * dir;
                            octomap::OcTreeNode* en = tree.search(
                                octomap::point3d(static_cast<float>(epos.x()),
                                                 static_cast<float>(epos.y()),
                                                 static_cast<float>(epos.z())));
                            if (en == nullptr || tree.isNodeOccupied(en)) {
                                near_edge = true; break;
                            }
                        }
                        if (near_edge) { ++rej_near_edge; continue; }
                    }
                    if (raws.size() >= raw_cap) { ++capped; cap_hit = true; break; }
                    RawViewpointCandidate rc;
                    rc.candidate_id = cand_id++;
                    rc.frontier_id = s.frontier_id;
                    rc.camera_position = pos;
                    rc.view_direction = dir;
                    rc.frontier_point = s.F;
                    rc.look_at = pos + dir;                       // forward == dir
                    rc.camera_orientation = cameraOrientationLookAt(pos, rc.look_at);
                    rc.environment_version = env_v;
                    raws.push_back(rc);
                }
            }
        }
        {
            std::ostringstream o; o << "BASELINE B2 (gen): seeds=" << seeds.size()
              << " gen=" << gen_total << " -> raw=" << raws.size()
              << " (occ_checks=" << occ_checks
              << " collision_step=" << std::fixed << std::setprecision(3)
              << viewpoint_line_collision_step_ << "m sample_step="
              << viewpoint_line_sample_step_ << "m rej_geo=" << rej_geo
              << " rej_unknown=" << rej_unknown
              << " rej_occupied_trunc=" << rej_occupied_trunc
              << " rej_near_edge=" << rej_near_edge << ")";
            if (cap_hit) o << " [★CAPPED at " << raw_cap << "]";
            o << ".";
            hpptLog(o.str());
        }
        if (raws.empty()) {
            releaseStickyBlacklist("BASELINE B2 EMPTY");
            handleUnproductiveCycle("BASELINE B2 EMPTY"); return;
        }

        // ---------- AGG: 位置+方向 全局聚合(跨 frontier)→ 造新点 ----------
        //   ★ 新方法:把"位置相近 + 方向相似"的候选跨 frontier 合成一个新点
        //   (平均位置 + 平均方向,不落在任何真实 frontier 上)。O(n) 网格分桶降数。
        //   IK 已删除(暂不做);打分挪到聚合之后(只打这些新点)。
        auto reps = aggregateByPositionDirection(
            raws, aggregate_merge_distance_,
            aggregate_merge_angle_deg_ * M_PI / 180.0);
        for (auto& r : reps) r.environment_version = env_v;
        {
            std::ostringstream o; o << "BASELINE AGG (pos+dir): " << raws.size()
              << " candidates -> " << reps.size() << " aggregated points "
              << "(merge_dist=" << std::fixed << std::setprecision(2)
              << aggregate_merge_distance_ << "m, merge_angle="
              << aggregate_merge_angle_deg_ << "deg).";
            hpptLog(o.str());
        }
        if (reps.empty()) {
            releaseStickyBlacklist("BASELINE AGG EMPTY");
            handleUnproductiveCycle("BASELINE AGG EMPTY"); return;
        }

        // ---------- SCORE: 只对聚合后的新点打分(unknown/visibility/semantic)----------
        ScoreWeights sw{score_w_unknown_, score_w_visibility_, score_w_semantic_};
        sw.normalizeOrThrow();
        for (auto& r : reps) {
            octomap::point3d Fp(r.frontier_point.x(), r.frontier_point.y(),
                                r.frontier_point.z());
            const double U = unknownDensity(tree, r.frontier_point, prob_density_radius_);
            const double V = geometricVisibility(tree, r.camera_position,
                r.camera_orientation, score_hfov_deg_, score_vfov_deg_,
                ig_max_range_m_, ig_num_rays_);
            const double S = use_semantic_probability_ ? computeSemanticRelevance(Fp) : 0.0;
            r.unknown_density = U; r.geometric_visibility = V; r.semantic_prior = S;
            r.viewpoint_score = viewpointScore(U, V, S, sw);
        }

        // ---------- TOP-N: 按 score 取前 motion_cost_top_n 个进 A* ----------
        std::sort(reps.begin(), reps.end(),
                  [](const hppt_baseline::RepresentativeViewpoint& a,
                     const hppt_baseline::RepresentativeViewpoint& b) {
                      return a.viewpoint_score > b.viewpoint_score;
                  });
        if (motion_cost_top_n_ > 0 && (int)reps.size() > motion_cost_top_n_)
            reps.resize(motion_cost_top_n_);
        {
            std::ostringstream o; o << "BASELINE TOP-N: -> top-" << reps.size()
              << " by score (best=" << std::fixed << std::setprecision(3)
              << (reps.empty() ? 0.0 : reps.front().viewpoint_score) << ").";
            hpptLog(o.str());
        }

        // ---------- B6: current→candidate 工作空间 A* motion cost(Phase 6)----------
        const double margin = reach_max_ + 0.30;
        AStarParams ap; ap.grid_res = astar_grid_res_; ap.inflation = astar_inflation_;
        ap.max_expansions = astar_max_expansions_;
        ap.aabb_min = Eigen::Vector3d(base.x() - margin, base.y() - margin,
            std::min(z_min_, current_position_.z()) - 0.15);
        ap.aabb_max = Eigen::Vector3d(base.x() + margin, base.y() + margin,
            std::max(z_max_, current_position_.z()) + 0.15);
        WorkspaceAStarCostEstimator est(ap);
        size_t reachable = 0;
        for (auto& r : reps) {
            auto mc = est.estimate(current_position_, r.camera_position, tree);
            r.motion_cost = mc.reachable ? mc.cost : hppt_baseline::kInf;
            r.edge_feasible = mc.reachable;
            if (mc.reachable) ++reachable;
        }
        {
            std::ostringstream o; o << "BASELINE B6 (A* motion cost): " << reps.size()
              << " reps, " << reachable << " reachable from current EE ["
              << std::fixed << std::setprecision(3) << current_position_.x() << ","
              << current_position_.y() << "," << current_position_.z() << "].";
            hpptLog(o.str());
        }

        // ---------- B7/B8: Greedy utility + Infinity 边重选 + plan-only 验证 + 派发 ----------
        GreedyParams gp; gp.lambda_score = greedy_lambda_score_;
        gp.lambda_motion = greedy_lambda_motion_;
        gp.mode = (greedy_utility_mode_ == "score_cost_ratio")
            ? GreedyUtilityMode::ScoreCostRatio : GreedyUtilityMode::WeightedSum;
        gp.normalizeOrThrow();

        // ★★★ SYNC-EXECUTE 重选循环:选 utility 最高的点 → 直接派发真实 execute
        //   并阻塞等真实结果(ground truth 判据,不再用 plan-only 探针)。
        //     成功 → 机械臂已真实移动,结束本 cycle;
        //     失败 → 本 cycle 内把该点 motion_cost 标 Infinity(局部 failed,
        //            不跨 cycle 保存)→ 重新归一化排序 → 选下一个点再派发;
        //     直到某点真实执行成功,或本 cycle 内所有 A* reachable 候选都试完。
        // ★★★ EVENT-DRIVEN(取代旧的同步 while + 30s 阻塞轮询):把本 cycle 的
        //   候选集保存为成员,派发第一个真实 execute 后立即返回。之后由
        //   resultCallback 驱动:失败 → 标 Infinity、重排、派发下一个(同 cycle);
        //   成功(只在机械臂真实到达后返回)→ 结束本 cycle,下个 OctoMap 开新 cycle。
        {
            std::lock_guard<std::mutex> lk(greedy_mutex_);
            greedy_reps_ = std::move(reps);
            greedy_failed_.clear();
            greedy_gp_ = gp;
            greedy_attempt_ = 0;
            greedy_limit_ = static_cast<int>(reachable);
            greedy_current_idx_ = -1;
            greedy_cycle_active_ = true;
        }
        {
            std::ostringstream o;
            o << "BASELINE B8: event-driven execute, attempt limit=" << reachable
              << " (= reachable candidates this cycle). Dispatch one, wait REAL "
              << "result (no 30s timeout).";
            hpptLog(o.str());
        }
        dispatchNextGreedyCandidate();
    }

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        // ★ LOG: cycle entry --- prints only when we actually start doing work
        //   (i.e. after the readyForNewTask / readyForSelection gates below).
        //   Use DEBUG for the very first checks so we don't spam every OctoMap.
        RCLCPP_DEBUG(this->get_logger(),
            "octomapCallback: enter (binary=%d, env_version=%s, "
            "pending_task=%s)",
            msg->binary ? 1 : 0,
            latest_env_version_.has_value()
                ? std::to_string(latest_env_version_.value()).c_str()
                : "none",
            pending_task_id_.has_value() ? pending_task_id_->c_str() : "none");
        if (!msg->binary) {
            RCLCPP_WARN(this->get_logger(), "Expecting binary OctoMap; got full.");
            return;
        }
        if (!latest_env_version_.has_value()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "STAGE-GATE: skip cycle -- no accepted environment_version yet "
                "on %s (env_status has not confirmed any snapshot).",
                env_status_topic_.c_str());
            return;
        }
        // ★ PAUSE: 暂停时不开新 cycle。若当前 cycle 正在进行(已派发 execute),
        //   由 resultCallback / dispatchNextGreedyCandidate 收尾(当前 execute 跑完
        //   后停),此处只保证不再启动新的 cycle。
        if (paused_.load()) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "STAGE-GATE: skip cycle -- exploration PAUSED.");
            return;
        }
        // ★★★ EVENT-DRIVEN: 本 cycle 正在进行(已派发 execute,正等真实结果或
        //   在失败重排中)→ 不重入新 cycle。覆盖"失败重排时 pending 短暂为空"的
        //   竞态窗口(此时 readyForNewTask 可能瞬时为 true)。cycle 由 execute
        //   success 或 all-execute-failed 结束时清 greedy_cycle_active_。
        {
            std::lock_guard<std::mutex> lk(greedy_mutex_);
            if (greedy_cycle_active_) {
                RCLCPP_DEBUG(this->get_logger(),
                    "STAGE-GATE: skip cycle -- greedy cycle in progress "
                    "(attempt %d/%d, awaiting real execute result).",
                    greedy_attempt_, greedy_limit_);
                return;
            }
        }
        if (!readyForNewTask()) {
            // readyForNewTask() already logs why (waiting on pending task /
            // dispatch timeout / task timeout) at DEBUG or WARN level.
            return;
        }
        if (!readyForSelection()) {
            RCLCPP_DEBUG(this->get_logger(),
                "STAGE-GATE: skip cycle -- selection cooldown "
                "(paused_until=%.3fs, last_attempt=%.3fs, interval=%.1fs)",
                duplicate_goal_pause_until_.seconds(),
                last_selection_attempt_at_.seconds(),
                selection_interval_sec_);
            return;
        }
        last_selection_attempt_at_ = this->now();
        RCLCPP_INFO(this->get_logger(),
            "=========== HPP-PT cycle START (env_version=%ld) ===========",
            latest_env_version_.value());

        std::unique_ptr<octomap::AbstractOcTree> abs_tree(
            octomap_msgs::binaryMsgToMap(*msg));
        if (!abs_tree) {
            RCLCPP_ERROR(this->get_logger(),
                "STAGE 0 (parse): binaryMsgToMap failed --- corrupt OctoMap "
                "message on %s; skip cycle.", octomap_topic_.c_str());
            return;
        }
        auto* tree_ptr = dynamic_cast<octomap::OcTree*>(abs_tree.get());
        if (!tree_ptr) {
            RCLCPP_ERROR(this->get_logger(),
                "STAGE 0 (parse): tree is not octomap::OcTree (type=%s); "
                "skip cycle.", abs_tree->getTreeType().c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(),
            "STAGE 0 (parse): OctoMap decoded ok, resolution=%.3fm, "
            "leaf count reported by tree.size()=%zu",
            tree_ptr->getResolution(),
            static_cast<size_t>(tree_ptr->size()));

        // ★★★ CONVERGENCE: 判断相比上次 octomap 是否有实质增长。增量 <
        //   convergence_growth_leaves_ → 地图稳定; 有增长 → 相机还在发现新空间,
        //   重置连续空转计数。
        {
            size_t cur_size = static_cast<size_t>(tree_ptr->size());
            octomap_grew_this_cycle_ =
                (cur_size > last_octomap_size_ +
                    static_cast<size_t>(std::max(0, convergence_growth_leaves_)));
            last_octomap_size_ = cur_size;
            if (octomap_grew_this_cycle_) {
                consecutive_empty_ = 0;
                converged_announced_ = false;
            }
        }

        // ★★★ BL-DIAG: 每 cycle 记录黑名单实际列表(regular+sticky)全量快照,
        //   再刷可视化(内部会记录 viz 与 actual 计数对照)。橙球消失时用这两条
        //   日志判断是"实际 sticky 被清空(case 2)"还是"viz 与 actual 不一致(case 1)"。
        logBlacklistSnapshot("cycle");
        publishBlacklistMarkers();

        if (!updateRobotPositionFromTF()) {
            RCLCPP_WARN(this->get_logger(),
                "STAGE 0 (TF): failed to read %s->%s; cannot proceed. "
                "See TF lookup warning above for cause.",
                world_frame_.c_str(), robot_frame_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(),
            "STAGE 0 (TF): robot at [%.3f, %.3f, %.3f] in %s",
            current_position_.x(), current_position_.y(), current_position_.z(),
            world_frame_.c_str());

        auto boundaries = extractFreeUnknownBoundary(*tree_ptr);
        // ★ frontier 最大距离限制:提取后立即裁掉离基座 > frontier_max_distance 的
        //   free-unknown 边界体素——最开始就只考虑 3m 内的 unknown/free 边界。
        //   (候选视点本就在可达球壳 ≤reach_max 内,故只需在此处一次性限距。)
        //   frontier_max_distance <= 0 关闭限制。
        if (frontier_max_distance_ > 0.0) {
            const Eigen::Vector3d base_pt(base_x_, base_y_, base_z_);
            const size_t before_far = boundaries.size();
            boundaries.erase(std::remove_if(boundaries.begin(), boundaries.end(),
                [&](const BoundaryPoint& bp) {
                    Eigen::Vector3d F(bp.coord.x(), bp.coord.y(), bp.coord.z());
                    return (F - base_pt).norm() > frontier_max_distance_;
                }), boundaries.end());
            if (boundaries.size() != before_far) {
                RCLCPP_INFO(this->get_logger(),
                    "STAGE 1 (frontier dist filter): %zu -> %zu (dropped %zu "
                    "beyond %.2fm from base).",
                    before_far, boundaries.size(), before_far - boundaries.size(),
                    frontier_max_distance_);
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "STAGE 1 (frontier extract): %zu free-unknown boundary voxels "
            "found in OctoMap (within %.2fm of base).",
            boundaries.size(), frontier_max_distance_);
        // 可视化未过滤的 frontier 全集（红色半透明）
        publishBoundaryMarker(boundaries);
        // ★★★ DIAGNOSTIC: RAW shelf-adjacent frontier voxel count (before per-voxel filter)
        {
            int shelf_adj_raw = 0;
            for (const auto& bp : boundaries) {
                double x = bp.coord.x(), y = bp.coord.y(), z = bp.coord.z();
                if (x >= 0.80-0.30 && x <= 1.19+0.30 &&
                    y >= -0.42-0.30 && y <= 0.42+0.30 &&
                    z >= 0.00-0.30 && z <= 1.11+0.30) {
                    ++shelf_adj_raw;
                }
            }
            RCLCPP_INFO(this->get_logger(),
                "★ SHELF-ADJACENT frontier voxels (RAW pre-filter): %d / %zu",
                shelf_adj_raw, boundaries.size());
        }
        if (boundaries.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "STAGE 1 EMPTY: no free-unknown boundary leaves in OctoMap. "
                "Likely causes: (a) OctoMap has no free voxels adjacent to "
                "unknown yet (map still building), (b) all free-unknown "
                "borders were smoothed away by hit/miss params, "
                "(c) the whole map is either fully known or fully unknown. "
                "Waiting for a richer OctoMap update.");
            handleUnproductiveCycle("STAGE 1 EMPTY");
            return;
        }

        // ★★★ NEW_METHOD 分派:默认走 Greedy Baseline;false 时落到下方 legacy
        //   direction-first(STAGE 2-6,固定 standoff + raw_score 排序 + top-N 探针)。
        if (use_greedy_baseline_) {
            runGreedyBaselinePipeline(*tree_ptr, boundaries);
            return;
        }

        // ========================================================================
        // ↓↓↓ LEGACY direction-first pipeline(STAGE 2-6),默认关闭,仅供对照 ↓↓↓
        // ========================================================================
        // 组合过滤：按"发布时会用的实际 (goal, look_at)"评估
        //   - isForbidden：真的救不回来（贴基座 / z 越界）
        //   - unreach_only：仅"太远"但可以投影观察（只是记数）
        //   - 失败黑名单 / 已访问目标
        //   - look_at 指向基座
        size_t before = boundaries.size();
        size_t fb_hits = 0, ur_hits = 0, bl_hits = 0;
        size_t perm_hits = 0, vis_hits = 0, in_hits = 0;
        size_t far_hits = 0;
        boundaries.erase(std::remove_if(boundaries.begin(), boundaries.end(),
            [&](const BoundaryPoint& bp){
                // ★ 硬距离截断：优先于其它检查，先用原始 frontier 坐标判定，
                //   不走 resolveViewpoint（因为 resolveViewpoint 会把远点投影回球面）。
                Eigen::Vector3d orig(bp.coord.x(), bp.coord.y(), bp.coord.z());
                if (isTooFar(orig)) { ++far_hits; return true; }
                // ★ TIER-3 fix: 用 orig 坐标查 z 越界，不能等到 resolveViewpoint
                //   之后。resolveViewpoint 会把 goal.z clamp 到 z_max，之后
                //   isForbidden(goal) 永远读不出 z 越界的 frontier —— 结果
                //   是 z=0.9-1.2 的天花板 frontier 全部漏网，产生"看天"pick。
                if (orig.z() < z_min_ || orig.z() > z_max_) {
                    ++fb_hits;
                    return true;
                }

                auto [goal, la] = resolveViewpoint(bp.coord, bp.unknown_neighbor);
                if (isForbidden(goal))          { ++fb_hits;   return true; }
                if (isUnreachable(goal)) {
                    ++ur_hits;
                    if (!observe_unreachable_) return true;
                }
                if (isBlacklisted(goal))         { ++bl_hits;   return true; }
                // ★ 永久黑名单：默认关闭时 isPermanentlyBlocked 直接返回 false
                if (isPermanentlyBlocked(goal))  { ++perm_hits; return true; }
                if (isVisited(goal))             { ++vis_hits;  return true; }
                if (isInwardLook(goal, la))      { ++in_hits;   return true; }
                return false;
            }), boundaries.end());
        if (boundaries.size() < before) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "STAGE 2 (per-voxel filter): %zu -> %zu (dropped %zu). "
                "reasons: too_far=%zu, forbidden(r<reach_min or z-oob)=%zu, "
                "unreach_but_observed_flag_off=%zu, blacklisted=%zu, "
                "perm_blocked=%zu, visited=%zu, inward_look_at=%zu.",
                before, boundaries.size(), (before - boundaries.size()),
                far_hits, fb_hits, ur_hits, bl_hits, perm_hits, vis_hits, in_hits);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "STAGE 2 (per-voxel filter): %zu voxels all passed.",
                boundaries.size());
        }
        // ★★★ DIAGNOSTIC: 统计过滤后 shelf-adjacent frontier VOXEL 数,配合 cluster 诊断
        //   一起看:如果这里 shelf_adj_v 很大但后面 shelf-adj cluster=0,说明是聚类
        //   步骤把它们并到非 shelf cluster 里 (超大 cluster centroid 飘出 shelf 邻域)。
        {
            int shelf_adj_v = 0;
            for (const auto& bp : boundaries) {
                double x = bp.coord.x(), y = bp.coord.y(), z = bp.coord.z();
                if (x >= 0.80-0.30 && x <= 1.19+0.30 &&
                    y >= -0.42-0.30 && y <= 0.42+0.30 &&
                    z >= 0.00-0.30 && z <= 1.11+0.30) {
                    ++shelf_adj_v;
                }
            }
            RCLCPP_INFO(this->get_logger(),
                "★ SHELF-ADJACENT frontier voxels (post per-voxel filter): %d / %zu",
                shelf_adj_v, boundaries.size());
        }
        if (boundaries.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "STAGE 2 EMPTY: all %zu boundary voxels dropped by per-voxel "
                "filter (too_far=%zu forbidden=%zu unreach=%zu blacklist=%zu "
                "perm=%zu visited=%zu inward=%zu). Most common causes: "
                "workspace too small (reach_min/reach_max), everything already "
                "visited, or all frontiers behind the base. Waiting for next "
                "octomap update.",
                before, far_hits, fb_hits, ur_hits, bl_hits,
                perm_hits, vis_hits, in_hits);
            // ★ TIER-5 收紧: STAGE 2 EMPTY 不释放 sticky。理由: STAGE 2
            //   dropped 主要是 too_far / forbidden / visited 之类几何硬约束,
            //   跟 sticky_blacklist 没直接关系;释放了反而让上一 cycle 拒过
            //   的 failed goal 立刻复活 → view_2 → view_4 型死循环。
            //   仅 STAGE 5 EMPTY (GREEDY 已试 K 个 candidate 全废) 才释放。
            handleUnproductiveCycle("STAGE 2 EMPTY");
            return;
        }
        // ★★★ STAGE 3 (DIRECTION-FIRST): 把存活 frontier 聚成 seed。
        //   不再做 weight top-K 预选; 所有存活 cluster(或 fallback 体素)都作为
        //   seed 进入下一步的扇形方向展开。仅用 max_frontier_seeds 做安全上限。
        const double fov_rad = ig_fov_deg_ * M_PI / 180.0;
        const double tan_half_fov = std::tan(0.5 * fov_rad);
        const double fine_res = tree_ptr->getResolution();
        auto compute_standoff = [&](int fine_count) {
            double extent = std::sqrt(static_cast<double>(std::max(1, fine_count)))
                            * fine_res;
            return std::clamp(extent / (2.0 * std::max(0.01, tan_half_fov)),
                              observation_standoff_min_, observation_standoff_max_);
        };
        auto cluster_dir = [&](const BoundaryCluster& c) -> Eigen::Vector3d {
            if (c.mean_unknown_dir.norm() > 1e-6)
                return c.mean_unknown_dir.normalized();
            Eigen::Vector3d d(c.centroid.x() - base_x_,
                              c.centroid.y() - base_y_,
                              c.centroid.z() - base_z_);
            return d;
        };
        auto encode_unknown = [&](const BoundaryCluster& c) -> octomap::point3d {
            Eigen::Vector3d d = cluster_dir(c);
            if (d.norm() < 1e-6) return c.unknown_representative;
            d.normalize();
            return octomap::point3d(
                c.centroid_oct.x() + static_cast<float>(d.x() * fine_res),
                c.centroid_oct.y() + static_cast<float>(d.y() * fine_res),
                c.centroid_oct.z() + static_cast<float>(d.z() * fine_res));
        };
        auto seed_from_bp = [&](const BoundaryPoint& bp) -> FrontierSeed {
            FrontierSeed s;
            s.centroid = bp.coord;
            s.ud = Eigen::Vector3d(
                bp.unknown_neighbor.x() - bp.coord.x(),
                bp.unknown_neighbor.y() - bp.coord.y(),
                bp.unknown_neighbor.z() - bp.coord.z());
            s.standoff = (bp.standoff_override > 0.0)
                ? bp.standoff_override : observation_standoff_;
            s.fine_count = 1;
            return s;
        };

        std::vector<FrontierSeed> seeds;
        if (enable_frontier_clustering_) {
            RCLCPP_INFO(this->get_logger(),
                "STAGE 3 (cluster): clustering ON "
                "(coarse_grid_factor=%d, cluster_min_voxels=%d).",
                coarse_grid_factor_, cluster_min_voxels_);
            auto clusters = clusterFrontier(
                boundaries, tree_ptr->getResolution(), coarse_grid_factor_);
            size_t clusters_raw = clusters.size();
            if (cluster_min_voxels_ > 1) {
                clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                    [this](const BoundaryCluster& c){
                        return c.voxel_count < cluster_min_voxels_;
                    }), clusters.end());
            }
            RCLCPP_INFO(this->get_logger(),
                "STAGE 3a (cluster): %zu voxels -> %zu raw -> %zu after "
                "min_voxels>=%d.",
                boundaries.size(), clusters_raw, clusters.size(),
                cluster_min_voxels_);

            // cluster centroid 层再过一次 workspace/visited/blacklist/inward/LOS,
            // 与后面扇形展开用的几何一致(主方向 + 自适应 standoff)。
            size_t before_c = clusters.size();
            size_t cvis=0, cbl=0, cperm=0, cin=0, clos=0, cforbid=0;
            clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                [&](const BoundaryCluster& c) {
                    octomap::point3d uk = encode_unknown(c);
                    double s = compute_standoff(c.fine_count);
                    auto [goal, la] = resolveViewpoint(c.centroid_oct, uk, s);
                    if (isOutOfWorkspace(goal))     { ++cforbid; return true; }
                    if (isBlacklisted(goal))        { ++cbl;    return true; }
                    if (isPermanentlyBlocked(goal)) { ++cperm;  return true; }
                    if (isVisited(goal))            { ++cvis;   return true; }
                    if (isInwardLook(goal, la))     { ++cin;    return true; }
                    Eigen::Vector3d fp(c.centroid_oct.x(), c.centroid_oct.y(),
                                       c.centroid_oct.z());
                    if (isLineOfSightBlocked(*tree_ptr, goal, fp)) {
                        ++clos; return true;
                    }
                    return false;
                }), clusters.end());
            RCLCPP_INFO(this->get_logger(),
                "STAGE 3b (centroid filter): %zu -> %zu (workspace=%zu "
                "blacklist=%zu perm=%zu visited=%zu inward=%zu LOS=%zu).",
                before_c, clusters.size(),
                cforbid, cbl, cperm, cvis, cin, clos);

            if (clusters.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "STAGE 3 FALLBACK: no cluster survived; uniformSample on "
                    "%zu raw boundary voxels.", boundaries.size());
                auto sampled = uniformSample(boundaries, boundary_samples_);
                for (const auto& bp : sampled) seeds.push_back(seed_from_bp(bp));
            } else {
                // ★ 安全上限: 跳过 top-K, 但避免 seed 爆炸。超限则按 fine_count
                //   降序保留最大的若干个并打印截断日志(不静默丢弃)。
                if (max_frontier_seeds_ > 0 &&
                    static_cast<int>(clusters.size()) > max_frontier_seeds_) {
                    std::sort(clusters.begin(), clusters.end(),
                        [](const BoundaryCluster& a, const BoundaryCluster& b){
                            return a.fine_count > b.fine_count;
                        });
                    RCLCPP_WARN(this->get_logger(),
                        "STAGE 3 CAP: %zu surviving clusters > "
                        "max_frontier_seeds=%d; keeping the %d largest by "
                        "fine_count (rest dropped this cycle).",
                        clusters.size(), max_frontier_seeds_,
                        max_frontier_seeds_);
                    clusters.resize(max_frontier_seeds_);
                }
                for (const auto& c : clusters) {
                    FrontierSeed s;
                    s.centroid   = c.centroid_oct;
                    s.ud         = cluster_dir(c);
                    s.standoff   = compute_standoff(c.fine_count);
                    s.fine_count = c.fine_count;
                    seeds.push_back(s);
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(),
                "STAGE 3 (uniform-sample): clustering OFF, uniformSample on "
                "%zu boundary voxels (boundary_samples=%d).",
                boundaries.size(), boundary_samples_);
            auto sampled = uniformSample(boundaries, boundary_samples_);
            for (const auto& bp : sampled) seeds.push_back(seed_from_bp(bp));
        }

        if (seeds.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "STAGE 3 EMPTY: no frontier seed produced; skip cycle.");
            handleUnproductiveCycle("STAGE 3 EMPTY");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
            "STAGE 3 done: %zu frontier seeds ready for direction expansion.",
            seeds.size());

        // ★★★ STAGE 4 (viewpoint generation): 每个 seed 扇形展开 look_at 方向,
        //   几何法找可达相机位 + 障碍剪枝, 得到全部存活候选。
        auto candidates = generateViewpointCandidates(*tree_ptr, seeds);
        if (candidates.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "STAGE 4 EMPTY: every direction was rejected (unreachable / "
                "blocked by obstacle / inward / visited / blacklisted). "
                "Releasing sticky blacklist and waiting for next octomap.");
            releaseStickyBlacklist("STAGE 4 EMPTY (all directions rejected)");
            handleUnproductiveCycle("STAGE 4 EMPTY");
            return;
        }

        // ★★★ STAGE 5 (score): 对每条存活方向独立打分(纯视点分, 不含运动代价)。
        for (auto& vc : candidates) {
            double raw = 0.0;
            scoreViewpoint(*tree_ptr, vc.frontier, vc.unknown_neighbor,
                           vc.goal, vc.look_at, &raw, &vc.score);
            vc.raw_score = raw;
        }
        std::sort(candidates.begin(), candidates.end(),
            [](const ViewpointCandidate& a, const ViewpointCandidate& b){
                return a.raw_score > b.raw_score;
            });
        // ★★★ LOGGER: 打完分后按分数降序记录每一条方向【位置, 方向, 各项评分】。
        logScoredDirections(candidates);
        // ★★★ RANKED VIZ: 发布 top-5 视角(#0 绿 / 其余青 + 视线箭头)。
        publishRankedViewpointMarkers(candidates);

        // ★★★ STAGE 6 (select + motion planning): 按分数降序取 top-N, 依次送
        //   plan-only MoveIt; 第一个规划成功的立即发布 execute。
        const auto is_dup = [&](const Eigen::Vector3d& p){
            return has_last_goal_ && (p - last_goal_).norm() < goal_dedup_tol_;
        };
        int tried = 0;
        bool published = false;
        bool low_ig_gated = false;
        size_t skip_bad = 0, skip_dup = 0, plan_fail = 0;
        for (const auto& vc : candidates) {
            // ★★★ MIN-IG GATE: candidates 已按 raw 降序。一旦当前(=剩余最高分)
            //   候选 raw < 门槛, 说明没有值得观察的视点(相机看过去几乎无信息
            //   增益) → 不发布, 整个 cycle 判"未产出有效目标"(计入收敛),
            //   不再把机械臂派去正对障碍面空看。
            if (vc.raw_score < min_publish_raw_score_) {
                low_ig_gated = true;
                RCLCPP_INFO(this->get_logger(),
                    "STAGE 6 MIN-IG: best remaining candidate raw=%.3f < "
                    "min_publish_raw_score=%.3f -> no worthwhile viewpoint; "
                    "skip publish this cycle.",
                    vc.raw_score, min_publish_raw_score_);
                break;
            }
            if (tried >= motion_planning_top_n_) break;
            if (isGoalBad(vc.goal, vc.look_at)) {
                ++skip_bad;
                // 确定性几何坏点(出工作空间/已黑名单/向内看) → 永久, 一小块。
                blacklistPoint(vc.goal, block_radius_small_,
                               "geometry", "goal_bad(oow/blacklisted/inward)",
                               /*permanent=*/true);
                continue;
            }
            if (is_dup(vc.goal)) { ++skip_dup; continue; }
            ++tried;
            RCLCPP_INFO(this->get_logger(),
                "STAGE 6: try #%d (score=%.3f) goal=[%.3f,%.3f,%.3f] -> "
                "motion planning.",
                tried, vc.raw_score, vc.goal.x(), vc.goal.y(), vc.goal.z());
            if (!runPlanOnlyProbe(vc.goal, vc.look_at)) {
                ++plan_fail;
                // 拉黑由 resultCallback 依 motion planner 回报的 error_type 分级
                //   处理(hard_unreachable→永久 / 其余→临时, 均一小块)。这里不再
                //   重复拉黑, 只跳过本候选。
                RCLCPP_INFO(this->get_logger(),
                    "STAGE 6: try #%d plan probe failed; blacklist handled by "
                    "resultCallback (error_type-based). Next.",
                    tried);
                continue;
            }
            RCLCPP_INFO(this->get_logger(),
                "STAGE 6: try #%d planning OK; publishing execute.", tried);
            published = publishExecuteTask(vc.goal, vc.look_at, vc.is_surrogate);
            if (published) {
                // ★ CONVERGENCE: 有实质进展(发布了目标)→ 重置收敛计数。
                consecutive_empty_ = 0;
                converged_announced_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "=========== HPP-PT cycle END (published) ===========");
            }
            break;
        }

        if (!published) {
            if (skip_dup > 0) {
                duplicate_goal_pause_until_ = this->now() +
                    rclcpp::Duration::from_seconds(duplicate_goal_cooldown_sec_);
            }
            const char* empty_stage =
                low_ig_gated ? "STAGE 6 LOW-IG" : "STAGE 6 EMPTY";
            RCLCPP_WARN(this->get_logger(),
                "%s: no candidate published (tried=%d, bad=%zu, duplicate=%zu, "
                "plan_fail=%zu, low_ig=%d of %zu candidates, best_raw=%.3f). "
                "Releasing sticky blacklist; waiting for next octomap.",
                empty_stage, tried, skip_bad, skip_dup, plan_fail,
                low_ig_gated ? 1 : 0, candidates.size(),
                candidates.empty() ? 0.0 : candidates.front().raw_score);
            releaseStickyBlacklist(empty_stage);
            handleUnproductiveCycle(empty_stage);
            return;
        }
    }

    struct BlacklistEntry {
        Eigen::Vector3d pos;
        rclcpp::Time added_at;
        double radius = -1.0;   // ★ TIER-1.3: 负数表示用节点默认 blacklist_radius_
    };
    // Sticky blacklist parallel to the regular blacklist_. Duplicated
    // entries let STAGE-5-EMPTY release path (releaseStickyBlacklist)
    // clear just the sticky copy while the regular blacklist stays
    // permanent-within-session, giving two escape hatches:
    //   - sticky is released when the arm truly runs out of moves;
    //   - regular is never released.
    std::vector<BlacklistEntry> sticky_blacklist_;

    struct VisitEntry {
        Eigen::Vector3d pos;
        rclcpp::Time added_at;
    };
    struct VisitHistoryEntry {
        Eigen::Vector3d pos;
        int count;
    };
    struct CostProbe {
        int candidate_idx;              // 对应的 sampled 候选索引 0..K-1
        rclcpp::Time sent_at;
        std::optional<double> cost;     // 结果回来后填 planning_time+execution_time
        bool succeeded = false;
        // ★★★ FIX-BLACKLIST-PROBE: 保存 goal 位置,便于 IK 秒拒时 result callback
        //   直接把该点加入 fast-reject 黑名单。之前只有被 HPP-PT 选进 path 的
        //   候选才会通过 Fix 1.1 拉黑,其他失败点在下一 cycle 会重新聚出来又试。
        Eigen::Vector3d goal;
    };

    // parameters
    std::string world_frame_, robot_frame_;
    std::string octomap_topic_, env_status_topic_, task_topic_;
    std::string feedback_topic_, result_topic_;
    std::string semantic_topic_;
    int boundary_samples_;
    // ★★★ DIRECTION-FIRST 流水线参数
    std::vector<double> viewpoint_fan_angles_deg_;
    int    viewpoint_fan_azimuths_;
    int    viewpoint_max_directions_;
    double viewpoint_dedup_tol_;
    int    motion_planning_top_n_;
    int    max_frontier_seeds_;
    // ★★★ LOGGER: decision log 落盘
    std::string   hppt_log_file_;
    std::ofstream hppt_log_ofs_;
    bool          hppt_log_to_file_ = false;
    std::mutex    hppt_log_mutex_;
    int moveit_num_planning_attempts_;
    double planning_time_limit_, moveit_allowed_planning_time_;
    double moveit_position_tolerance_, moveit_orientation_tolerance_;
    double task_timeout_sec_, task_dispatch_timeout_sec_, selection_interval_sec_;
    double duplicate_goal_cooldown_sec_, goal_dedup_tol_;
    double blacklist_radius_;
    double visited_radius_, visited_duration_sec_;
    double permanent_block_radius_;
    int permanent_block_count_;
    bool use_planning_cost_;
    double cost_probe_timeout_sec_;
    double cost_probe_max_cost_;
    // ★ APPROACH D: confirm-probe against current env before publish.
    bool   enable_confirm_probe_;
    double confirm_probe_timeout_sec_;
    bool enable_frontier_clustering_;
    int cluster_min_voxels_;
    int coarse_grid_factor_;
    double base_x_, base_y_, base_z_;
    double reach_min_, reach_max_;
    double frontier_max_range_factor_;
    double z_min_, z_max_;
    double observation_standoff_, observation_min_z_;
    double observation_standoff_min_, observation_standoff_max_;
    double observation_min_z_low_;
    bool   enable_los_check_;
    double los_min_hit_dist_;          // ★ FIX-2
    bool require_outward_look_;
    bool observe_unreachable_;
    double projection_scale_;
    bool   projection_z_preserve_;   // ★ Z-PRESERVING
    double prob_density_radius_, prob_w_obs_, prob_w_unk_;
    bool use_semantic_probability_;
    double semantic_radius_, semantic_max_age_sec_;
    double semantic_fusion_voxel_size_ = 0.05;
    double semantic_fusion_max_age_sec_ = 0.0;
    double prob_w_sem_, semantic_min_relevance_;
    double prob_w_ig_;
    double ig_fov_deg_;
    double ig_max_range_m_;
    int    ig_num_rays_;
    // ★★★ Tier 1/2/3 新增参数
    double ig_hit_bonus_;              // Tier 2.4
    double prob_w_attract_;            // Tier 2.5
    double attract_search_radius_;     // Tier 2.5
    double attract_decay_;             // Tier 2.5
    // ★ Tier 3 (occlusion-aware): 探索被遮挡 unknown
    double prob_w_peek_;               // Tier 3 (B)
    double prob_w_occl_;               // Tier 3 (A)
    double occl_radius_;               // Tier 3 (A)
    bool   directional_attract_;       // Tier 3 (C)
    double attract_lateral_tol_;       // Tier 3 (C)
    bool   enable_informed_fps_;       // Tier 3.7
    double fast_reject_planning_time_; // Tier 1.3
    double fast_reject_radius_;        // Tier 1.3
    double block_radius_small_;         // NON-CONSERVATIVE FALLBACK: 一小块拉黑半径
    double hppt_cost_attract_boost_;   // HPP-PT COST-DISCOUNT
    double alt_pullback_dist_;         // ALT-PULLBACK
    double alt_min_probability_;       // ALT-PULLBACK
    int    alt_max_count_;             // Fix C: multi-alt viewpoints per cluster
    bool   use_greedy_argmin_;         // GREEDY-ARGMIN

    // state
    uint64_t task_counter_;
    std::optional<int64_t> latest_env_version_;
    std::optional<std::string> pending_task_id_;
    std::optional<Eigen::Vector3d> pending_goal_;
    rclcpp::Time pending_task_sent_at_;
    // ★ Stale-plan diagnostic: which env_version the cost probe validated
    //   this pending task against.  Compared against latest_env_version_
    //   at result time to detect probe-vs-execute drift.
    int64_t pending_task_env_version_ = 0;
    bool pending_seen_feedback_;
    rclcpp::Time last_selection_attempt_at_;
    rclcpp::Time duplicate_goal_pause_until_;
    Eigen::Vector3d last_goal_;
    bool has_last_goal_;
    Eigen::Vector3d current_position_;
    std::vector<BlacklistEntry> blacklist_;
    std::vector<VisitEntry> visited_;
    std::vector<VisitHistoryEntry> visit_history_;
    // ★★★ FIX-BLACKLIST-PROBE: MultiThreadedExecutor 让 result callback 与
    //   octomap callback 并发运行,两者都会读写 blacklist_ (append vs remove_if
    //   erase),必须加锁,否则并发场景下是 UB。visited_ 只在 result callback
    //   写、octomap callback 读,同样加锁保底。
    std::mutex blacklist_mutex_;
    std::mutex cost_probes_mutex_;
    std::map<std::string, CostProbe> cost_probes_;
    uint64_t cost_probe_counter_ = 0;
    // ★★★ EVENT-DRIVEN EXECUTE(greedy 新流水线,取代旧的 30s 阻塞轮询):
    //   派发一个真实 execute 任务后 **不阻塞**;由 resultCallback 依 task_id
    //   驱动:失败 → 本 cycle 把该点 motion_cost 标 Infinity、重排、派发下一个;
    //   成功(只在机械臂真实到达后返回)→ 结束本 cycle,下个 OctoMap 开新 cycle。
    //   下面这组成员持久化"本 cycle 的候选集 + 已失败列表 + 当前派发索引",
    //   让 resultCallback 能在同一 cycle 内继续重选。greedy_mutex_ 保护它们
    //   (octomapCallback 与 resultCallback 在不同 callback group,可能并发)。
    std::mutex greedy_mutex_;
    std::vector<hppt_baseline::RepresentativeViewpoint> greedy_reps_;
    std::vector<int> greedy_failed_;
    hppt_baseline::GreedyParams greedy_gp_;
    int  greedy_attempt_ = 0;
    int  greedy_limit_ = 0;
    int  greedy_current_idx_ = -1;
    bool greedy_cycle_active_ = false;
    int  max_execute_attempts_ = 20;
    rclcpp::CallbackGroup::SharedPtr result_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_semantic_;
    std::mutex semantic_mutex_;
    std::unordered_map<SemanticVoxelKey, SemanticVoxelEvidence, SemanticVoxelKeyHash>
        semantic_voxels_;
    rclcpp::Time latest_semantic_stamp_;
    bool have_semantic_points_ = false;

    // ============================================================
    // ★★★ NEW_METHOD: Greedy Baseline 成员
    // ============================================================
    bool        use_greedy_baseline_ = true;
    double      viewpoint_line_sample_step_ = 0.08;
    double      viewpoint_line_collision_step_ = 0.03;
    double      viewpoint_max_observe_distance_ = 3.0;
    double      viewpoint_endpoint_clearance_ = 0.10;
    int         viewpoint_max_distances_per_dir_ = 4;
    double      frontier_max_distance_ = 3.0;
    double      aggregate_merge_distance_ = 0.15;
    double      aggregate_merge_angle_deg_ = 30.0;
    int         motion_cost_top_n_ = 50;
    int         max_raw_candidates_ = 4000000;
    int         max_ik_calls_per_cycle_ = 120;
    double      ik_timeout_sec_ = 0.05;
    std::string planning_group_ = "jaka_a5";
    std::string ik_link_name_ = "tool0";
    std::string compute_ik_service_ = "/compute_ik";
    std::string state_validity_service_ = "/check_state_validity";
    bool        enable_state_validity_check_ = false;
    double      score_w_unknown_ = 0.3, score_w_visibility_ = 0.5, score_w_semantic_ = 0.2;
    double      score_hfov_deg_ = 69.0, score_vfov_deg_ = 42.0;
    double      ms_sigma_position_ = 0.10, ms_sigma_orientation_ = 0.52;
    double      ms_lambda_orientation_ = 1.0, ms_merge_distance_ = 0.05;
    double      ms_merge_angle_deg_ = 30.0;
    int         ms_max_iter_ = 20;
    double      greedy_lambda_score_ = 0.7, greedy_lambda_motion_ = 0.3;
    std::string greedy_utility_mode_ = "weighted_sum";
    double      astar_grid_res_ = 0.05, astar_inflation_ = 0.05;
    int         astar_max_expansions_ = 200000;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    std::mutex joint_state_mutex_;
    sensor_msgs::msg::JointState latest_joint_state_;
    bool have_joint_state_ = false;

    rclcpp::CallbackGroup::SharedPtr ik_cb_group_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;

    // ROS i/o
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_env_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_feedback_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    std::atomic<bool> paused_{false};   // ★ PAUSE: true=暂停决策与派发
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_result_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_json_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    // ★★★ CONVERGENCE detection
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_exploration_complete_;
    bool        enable_convergence_detection_ = true;
    int         convergence_empty_cycles_ = 20;
    int         convergence_growth_leaves_ = 100;
    bool        convergence_shutdown_ = true;
    std::string exploration_complete_topic_;
    size_t      last_octomap_size_ = 0;
    int         consecutive_empty_ = 0;
    bool        octomap_grew_this_cycle_ = false;
    bool        converged_announced_ = false;
    double      min_publish_raw_score_ = 0.05;   // ★ MIN-IG GATE
    // ★★★ RANKED VIZ
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    bool        enable_ranked_viz_ = true;
    int         last_bl_marker_count_ = 0;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HPPTBoundaryNode>();
    // 3 threads: octomapCallback(default group) 阻塞时,另有线程服务
    // result_cb_group_(plan-only 结果)与 ik_cb_group_(/compute_ik、
    // /check_state_validity 服务响应)。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
