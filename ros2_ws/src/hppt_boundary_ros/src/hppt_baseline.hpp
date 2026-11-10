// hppt_baseline.hpp — new_method 可验证 Greedy Baseline 的纯算法与接口层。
//
// 设计原则(见 prompt.md Phase 1 / 记忆 new-method-greedy-baseline):
//   * 本头文件只依赖 octomap + Eigen,**不依赖 ROS / MoveIt**,因此可独立单测。
//   * ROS 节点(hppt_boundary_ros.cpp)负责:调 MoveIt 服务做 IK+碰撞(Phase 3)、
//     订阅 octomap/joint_states、octomapCallback 编排、发 execute。
//   * 本层负责纯算法:多方向×多距离采样(Phase 2)、3 项归一化打分(Phase 4)、
//     per-frontier Weighted Mean Shift(Phase 5)、工作空间 A* motion cost(Phase 6)、
//     Greedy utility 选择(Phase 7)。
//
// ⚠️ Phase 6 的 A* 按用户澄清覆盖 prompt.md 第八节:是 **3D 工作空间**绕障(octomap
//    occupied)路径搜索,**几何路径长度**作为 motion cost,不是配置空间/关节空间,
//    也不调 motion planning 出轨迹。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <octomap/octomap.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace hppt_baseline {

constexpr double kInf = std::numeric_limits<double>::infinity();

// ============================================================================
// Phase 1: 数据结构
// ============================================================================

// 原始候选视点(IK+碰撞过滤前/后都用它;joint_state 由节点侧 MoveIt 服务填充)。
struct RawViewpointCandidate {
    int    candidate_id = -1;
    int    frontier_id  = -1;

    Eigen::Vector3d    camera_position;      // 相机(=tool0)位置
    Eigen::Quaterniond camera_orientation{1, 0, 0, 0};  // 光轴指向 look_at 的姿态
    Eigen::Vector3d    view_direction;       // 观察方向 d(camera → frontier/unknown 归一化)
    Eigen::Vector3d    frontier_point;       // 该视点观察的 frontier 点 F(打分用)
    Eigen::Vector3d    look_at;              // = camera_position + view_direction(派发/IK 用)

    // IK 结果(Phase 3 由节点侧填充);为空表示尚未求解 / 求解失败。
    std::vector<double> ik_solution;         // 关节角(顺序同 planning group)
    bool   ik_valid = false;                 // IK 成功且 target-state 无碰撞

    // 打分分项(Phase 4);均在 [0,1]。
    double unknown_density     = 0.0;        // U_i
    double geometric_visibility= 0.0;        // V_i
    double semantic_prior      = 0.0;        // S_i
    double viewpoint_score     = 0.0;        // p_i ∈ [0,1]

    int64_t environment_version = 0;
};

// 代表视点(Mean Shift 后每 cluster 选一个真实候选)+ motion cost / greedy 字段。
struct RepresentativeViewpoint {
    int    node_id      = -1;
    int    frontier_id  = -1;

    Eigen::Vector3d    camera_position;
    Eigen::Quaterniond camera_orientation{1, 0, 0, 0};
    Eigen::Vector3d    view_direction;
    Eigen::Vector3d    look_at;
    Eigen::Vector3d    frontier_point;          // 聚合后:成员 frontier 均值(打分用)
    std::vector<double> joint_state;

    double unknown_density      = 0.0;
    double geometric_visibility = 0.0;
    double semantic_prior       = 0.0;
    double viewpoint_score      = 0.0;       // p_i ∈ [0,1]

    // Phase 6/7
    double motion_cost            = kInf;    // c_i(工作空间 A* 路径长度)
    double normalized_motion_cost = 0.0;     // ĉ_i ∈ [0,1]
    double greedy_utility         = -kInf;   // utility_i
    bool   edge_feasible          = false;   // current→node A* 可达

    int64_t environment_version   = 0;
};

// ============================================================================
// Phase 1: WorkspaceModel 接口(第一版球壳近似,但封成接口,禁止硬编码几何进采样代码)
// ============================================================================

struct WorkspaceLimits {
    Eigen::Vector3d base{0, 0, 0};
    double reach_min = 0.60;
    double reach_max = 0.85;
    double z_min     = 0.05;
    double z_max     = 0.85;
    double cam_min_view_distance = 0.0;   // 相机到 frontier 的最小/最大观察距离(0=不约束)
    double cam_max_view_distance = 0.0;
};

class WorkspaceModel {
public:
    virtual ~WorkspaceModel() = default;

    // p 是否落在(相机可放置的)工作空间内。
    virtual bool contains(const Eigen::Vector3d& p) const = 0;

    // 求相机位线 P(t) = F - t·d(t>0)与工作空间的交区间 [t_min, t_max]。
    // 无交集返回 nullopt。d 需已归一化。
    virtual std::optional<std::pair<double, double>>
    intersectViewLine(const Eigen::Vector3d& F,
                      const Eigen::Vector3d& d) const = 0;
};

// 球壳工作空间:base 为球心,半径 [reach_min, reach_max],再叠加 z 与观察距离约束。
class SphericalShellWorkspace : public WorkspaceModel {
public:
    explicit SphericalShellWorkspace(const WorkspaceLimits& lim) : lim_(lim) {}

    bool contains(const Eigen::Vector3d& p) const override {
        const double r = (p - lim_.base).norm();
        if (r < lim_.reach_min - 1e-9 || r > lim_.reach_max + 1e-9) return false;
        if (p.z() < lim_.z_min - 1e-9 || p.z() > lim_.z_max + 1e-9) return false;
        return true;
    }

    // 沿 P(t)=F - t·d 求与球壳 [reach_min,reach_max] 的相交 t 区间,再夹到 z 约束
    // 与观察距离 [cam_min_view, cam_max_view]。返回 t>0 的可行区间。
    std::optional<std::pair<double, double>>
    intersectViewLine(const Eigen::Vector3d& F,
                      const Eigen::Vector3d& d) const override {
        // |F - t d - base|^2 = r^2  →  t^2 - 2 t (d·(F-base)) + (|F-base|^2 - r^2)=0
        const Eigen::Vector3d fb = F - lim_.base;
        const double b = d.dot(fb);              // 注意符号:方程里是 -2t(d·fb)
        const double base2 = fb.squaredNorm();

        auto ring = [&](double R, double& lo, double& hi) -> bool {
            // t^2 - 2 b t + (base2 - R^2) = 0
            const double disc = b * b - (base2 - R * R);
            if (disc < 0) return false;
            const double s = std::sqrt(disc);
            lo = b - s;
            hi = b + s;
            return true;
        };
        double outer_lo, outer_hi;
        if (!ring(lim_.reach_max, outer_lo, outer_hi)) return std::nullopt;  // 线不进外球
        double inner_lo, inner_hi;
        const bool has_inner = ring(lim_.reach_min, inner_lo, inner_hi);

        // 壳层 = 外球内部 减去 内球内部。外球给 [outer_lo, outer_hi];
        // 若存在内球相交,壳层是 [outer_lo, inner_lo] ∪ [inner_hi, outer_hi]。
        // 取"离 frontier 更近的可行段"作为采样区间(优先靠 frontier 的一侧)。
        std::vector<std::pair<double, double>> segs;
        if (!has_inner) {
            segs.push_back({outer_lo, outer_hi});
        } else {
            segs.push_back({outer_lo, inner_lo});
            segs.push_back({inner_hi, outer_hi});
        }

        double best_lo = kInf, best_hi = -kInf;
        for (auto& s : segs) {
            double lo = std::max(s.first, 0.0);   // t>0
            double hi = s.second;
            if (lim_.cam_max_view_distance > 0.0) hi = std::min(hi, lim_.cam_max_view_distance);
            if (lim_.cam_min_view_distance > 0.0) lo = std::max(lo, lim_.cam_min_view_distance);
            if (hi <= lo) continue;
            // z 约束需逐点检验,这里先粗取整段,细采样时再逐点 contains() 过滤。
            if (lo < best_lo) { best_lo = lo; best_hi = hi; }  // 取更靠 frontier(小 t)的段
        }
        if (best_hi <= best_lo) return std::nullopt;
        return std::make_pair(best_lo, best_hi);
    }

private:
    WorkspaceLimits lim_;
};

// ============================================================================
// Phase 2: 多方向 × 多距离视点生成(方向集 + 沿线多距离采样)
// ============================================================================

// 绕 unknown 主方向 ud0 做锥环方位密采样:主方向 + 每个半锥角 θ 上 n_az 个方位。
inline std::vector<Eigen::Vector3d>
generateViewDirections(const Eigen::Vector3d& ud0_in,
                       const std::vector<double>& fan_angles_deg,
                       int n_azimuth) {
    std::vector<Eigen::Vector3d> dirs;
    Eigen::Vector3d ud0 = ud0_in;
    if (ud0.norm() < 1e-9) ud0 = Eigen::Vector3d(1, 0, 0);
    ud0.normalize();
    dirs.push_back(ud0);

    const Eigen::Vector3d helper = (std::abs(ud0.z()) < 0.9)
        ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d right = ud0.cross(helper).normalized();
    const Eigen::Vector3d up    = right.cross(ud0).normalized();
    const int n_az = std::max(1, n_azimuth);
    for (double deg : fan_angles_deg) {
        const double th = deg * M_PI / 180.0;
        const double ct = std::cos(th), st = std::sin(th);
        for (int a = 0; a < n_az; ++a) {
            const double phi = 2.0 * M_PI * static_cast<double>(a) / n_az;
            Eigen::Vector3d dv = ct * ud0 + st * (std::cos(phi) * right + std::sin(phi) * up);
            dirs.push_back(dv.normalized());
        }
    }
    return dirs;
}

// 沿观察方向 d 从 frontier F 反推的相机位线 P(t)=F - t·d,在工作空间交区间内
// 按步长 step 采样多个相机位(⚠️ 取代旧的"每方向只用一个固定 standoff 点")。
// 逐点再用 workspace.contains() 过滤 z 等约束。
inline std::vector<Eigen::Vector3d>
sampleCameraPositionsAlongLine(const Eigen::Vector3d& F,
                               const Eigen::Vector3d& d_in,
                               const WorkspaceModel& ws,
                               double step) {
    std::vector<Eigen::Vector3d> out;
    Eigen::Vector3d d = d_in;
    if (d.norm() < 1e-9) return out;
    d.normalize();
    auto seg = ws.intersectViewLine(F, d);
    if (!seg) return out;
    const double t0 = seg->first, t1 = seg->second;
    if (step <= 1e-6) step = 0.05;
    // ★ 跳过近端端点 t0(常落在 reach 外边界 / 太贴 frontier 边缘的位置):
    //   第一个点至少取在 t0+step 处。
    for (double t = t0 + step; t <= t1 + 1e-9; t += step) {
        Eigen::Vector3d p = F - t * d;
        if (ws.contains(p)) out.push_back(p);
    }
    // 兜底:若交区间短于一个 step 导致上面一个点都没采到,取区间中点(避免整条
    //   方向丢失候选)。
    if (out.empty()) {
        Eigen::Vector3d p = F - 0.5 * (t0 + t1) * d;
        if (ws.contains(p)) out.push_back(p);
    }
    return out;
}

// 相机(=tool0)姿态:local +Z = forward = (look_at - cam)。
// ⚠️ 必须与下游 mp_team_bridge/conversions.py::viewpoint_quaternion 完全一致
//    (否则 in-node IK 预过滤与真实 execute 姿态不符):
//      forward = normalize(look_at - position)
//      right   = normalize(reference_up × forward)   (注意叉乘顺序)
//      up      = forward × right
//      reference_up = world +Z,|forward·+Z|>0.98 时换成 world +Y
inline Eigen::Quaterniond
cameraOrientationLookAt(const Eigen::Vector3d& cam,
                        const Eigen::Vector3d& target) {
    Eigen::Vector3d fwd = target - cam;
    if (fwd.norm() < 1e-9) return Eigen::Quaterniond::Identity();
    fwd.normalize();
    Eigen::Vector3d ref_up(0, 0, 1);
    if (std::abs(fwd.dot(ref_up)) > 0.98) ref_up = Eigen::Vector3d(0, 1, 0);
    const Eigen::Vector3d right = ref_up.cross(fwd).normalized();  // reference_up × forward
    const Eigen::Vector3d up    = fwd.cross(right);                // forward × right
    Eigen::Matrix3d R;
    R.col(0) = right;   // x
    R.col(1) = up;      // y
    R.col(2) = fwd;     // z = optical axis (view direction)
    return Eigen::Quaterniond(R);
}

// ============================================================================
// Phase 4: 三项归一化打分
// ============================================================================

// U_i:frontier 邻域(半径 radius 的立方采样)内 unknown 体素占比 ∈ [0,1]。
inline double unknownDensity(const octomap::OcTree& tree,
                             const Eigen::Vector3d& frontier,
                             double radius) {
    if (radius <= 0.0) return 0.0;
    const double res = tree.getResolution();
    int n_unknown = 0, n_total = 0;
    for (double dx = -radius; dx <= radius + 1e-9; dx += res)
      for (double dy = -radius; dy <= radius + 1e-9; dy += res)
        for (double dz = -radius; dz <= radius + 1e-9; dz += res) {
            if (dx * dx + dy * dy + dz * dz > radius * radius) continue;
            ++n_total;
            octomap::point3d q(frontier.x() + dx, frontier.y() + dy, frontier.z() + dz);
            if (tree.search(q) == nullptr) ++n_unknown;   // null = unknown
        }
    if (n_total == 0) return 0.0;
    return static_cast<double>(n_unknown) / static_cast<double>(n_total);
}

// V_i:从相机位/姿态构造视锥,打 n_rays 条射线,统计"先到 unknown(未撞 occupied)"
// 的射线占比 ∈ [0,1]。考虑 hfov/vfov/max_range/朝向/occupied 遮挡。
inline double geometricVisibility(const octomap::OcTree& tree,
                                  const Eigen::Vector3d& cam,
                                  const Eigen::Quaterniond& orient,
                                  double hfov_deg, double vfov_deg,
                                  double max_range, int n_rays) {
    if (n_rays <= 0 || max_range <= 0.0) return 0.0;
    const Eigen::Matrix3d R = orient.toRotationMatrix();
    const Eigen::Vector3d fwd = R.col(2);   // 光轴 = +Z
    const Eigen::Vector3d rgt = R.col(0);
    const Eigen::Vector3d up  = R.col(1);
    const double hh = 0.5 * hfov_deg * M_PI / 180.0;
    const double hv = 0.5 * vfov_deg * M_PI / 180.0;

    // 在 hfov×vfov 上近似均匀取样(方格网格,数量≈n_rays)。
    const int grid = std::max(1, static_cast<int>(std::round(std::sqrt((double)n_rays))));
    int valid = 0, reached_unknown = 0;
    for (int iy = 0; iy < grid; ++iy) {
        const double fy = (grid == 1) ? 0.0 : (2.0 * iy / (grid - 1) - 1.0);   // [-1,1]
        for (int ix = 0; ix < grid; ++ix) {
            const double fx = (grid == 1) ? 0.0 : (2.0 * ix / (grid - 1) - 1.0);
            const double ay = fx * hh;   // 水平偏角
            const double ax = fy * hv;   // 垂直偏角
            Eigen::Vector3d dir = (fwd + std::tan(ay) * rgt + std::tan(ax) * up).normalized();
            ++valid;
            // ★ 用 octomap castRay 沿八叉树高效投射,取代"逐 res 步进 + 每步从根查":
            //   ignoreUnknownCells=false → 撞 occupied 返回 true(遮挡,不计);
            //   停在 unknown 或走满 maxRange 返回 false,end=停止点。
            //   停止点是 unknown(search==nullptr)→ 先到未知,计数。
            //   语义与原手动步进一致,只是一条射线一次 castRay(快 ~10-50×)。
            octomap::point3d end;
            const bool hit_occ = tree.castRay(
                octomap::point3d(static_cast<float>(cam.x()),
                                 static_cast<float>(cam.y()),
                                 static_cast<float>(cam.z())),
                octomap::point3d(static_cast<float>(dir.x()),
                                 static_cast<float>(dir.y()),
                                 static_cast<float>(dir.z())),
                end, /*ignoreUnknownCells=*/false, max_range);
            if (!hit_occ && tree.search(end) == nullptr) {
                ++reached_unknown;   // 先到 unknown(未被 occupied 挡)
            }
        }
    }
    if (valid == 0) return 0.0;
    return static_cast<double>(reached_unknown) / static_cast<double>(valid);
}

struct ScoreWeights {
    double w_unknown    = 0.3;
    double w_visibility = 0.5;
    double w_semantic   = 0.2;

    // 权重自动归一化;非法(全负/全零)抛异常(Phase 4 要求"非法配置明确报错")。
    void normalizeOrThrow() {
        if (!(w_unknown >= 0.0 && w_visibility >= 0.0 && w_semantic >= 0.0))
            throw std::invalid_argument("ScoreWeights: 权重必须非负");
        const double s = w_unknown + w_visibility + w_semantic;
        if (s <= 1e-9)
            throw std::invalid_argument("ScoreWeights: 权重之和为 0,非法");
        w_unknown /= s; w_visibility /= s; w_semantic /= s;
    }
};

// p_i = clamp(w_u·U + w_v·V + w_s·S, 0, 1)。要求各项与结果都在 [0,1]。
inline double viewpointScore(double U, double V, double S, const ScoreWeights& w) {
    auto c01 = [](double x) { return std::clamp(x, 0.0, 1.0); };
    const double p = w.w_unknown * c01(U) + w.w_visibility * c01(V) + w.w_semantic * c01(S);
    return std::clamp(p, 0.0, 1.0);
}

// ============================================================================
// Phase 5: per-frontier Weighted Mean Shift(视点级),每 cluster 选一个真实候选
// ============================================================================

struct MeanShiftParams {
    double sigma_position    = 0.10;   // 位置尺度
    double sigma_orientation = 0.52;   // 姿态尺度(rad,≈30°)
    double lambda_orientation= 1.0;    // 姿态项权重(=0 → 退化为纯位置聚合)
    int    max_iterations    = 20;
    double convergence_eps   = 1e-3;   // 中心收敛阈
    double merge_distance    = 0.05;   // mode 位置合并阈(m)
    double merge_angle       = 0.52;   // mode 朝向合并阈(rad,≈30°;≥π → 忽略朝向)
};

// D²(i,j) = ||xi-xj||²/σp² + λ·angle(Ri,Rj)²/σo²
inline double meanShiftDistanceSq(const RawViewpointCandidate& a,
                                  const RawViewpointCandidate& b,
                                  const MeanShiftParams& mp) {
    const double dp = (a.camera_position - b.camera_position).squaredNorm()
                    / (mp.sigma_position * mp.sigma_position);
    double ang = a.camera_orientation.angularDistance(b.camera_orientation);  // [0,π]
    const double dorient = mp.lambda_orientation * (ang * ang)
                    / (mp.sigma_orientation * mp.sigma_orientation);
    return dp + dorient;
}

// 对同一 frontier_id 的候选做 Mean Shift:每点漂移到 (w_ij=p_j·exp(-0.5 D²)) 加权均值,
// 收敛得到若干 mode;每个 mode 选"最靠 mode 中心 & viewpoint_score 高 & ik_valid"的
// 真实候选作为 RepresentativeViewpoint。**不直接输出平均 Pose**。
inline std::vector<RepresentativeViewpoint>
meanShiftRepresentatives(const std::vector<RawViewpointCandidate>& cands_in,
                         const MeanShiftParams& mp) {
    std::vector<RepresentativeViewpoint> reps;
    // 只对 ik_valid 的候选做(Phase 5 要求:进入 Mean Shift 的候选须 IK 有效)。
    std::vector<RawViewpointCandidate> cands;
    for (const auto& c : cands_in) if (c.ik_valid) cands.push_back(c);
    if (cands.empty()) return reps;

    // 按 frontier_id 分组(禁止跨 frontier 混聚)。
    std::unordered_map<int, std::vector<int>> by_frontier;
    for (int i = 0; i < (int)cands.size(); ++i) by_frontier[cands[i].frontier_id].push_back(i);

    const double sig_o = std::max(1e-6, mp.sigma_orientation);
    int node_counter = 0;
    for (auto& kv : by_frontier) {
        const std::vector<int>& idx = kv.second;
        const int n = (int)idx.size();
        // ★★★ 方案 B:方向门控核 + 联合空间合并。
        //   每个漂移点固定一个参考朝向 R_a(= 起点候选姿态,迭代中不变);
        //   核用完整 D² = |Δx|²/σp² + λ·angle(R_a,R_j)²/σo²,只漂位置。
        //   mode = (收敛位置, R_a);合并需位置<merge_distance 且朝向<merge_angle。
        std::vector<Eigen::Vector3d>    mode_pos(n);
        std::vector<Eigen::Quaterniond> mode_rot(n);
        for (int a = 0; a < n; ++a) {
            Eigen::Vector3d    x   = cands[idx[a]].camera_position;
            const Eigen::Quaterniond Ra = cands[idx[a]].camera_orientation;   // 固定参考朝向
            for (int it = 0; it < mp.max_iterations; ++it) {
                Eigen::Vector3d num = Eigen::Vector3d::Zero();
                double den = 0.0;
                for (int b = 0; b < n; ++b) {
                    const RawViewpointCandidate& cb = cands[idx[b]];
                    const double dp = (x - cb.camera_position).squaredNorm()
                                    / (mp.sigma_position * mp.sigma_position);
                    const double ang = Ra.angularDistance(cb.camera_orientation);  // [0,π]
                    const double d2  = dp + mp.lambda_orientation * (ang * ang)
                                                / (sig_o * sig_o);
                    const double w = cb.viewpoint_score * std::exp(-0.5 * d2);
                    num += w * cb.camera_position;
                    den += w;
                }
                if (den <= 1e-12) break;
                Eigen::Vector3d x_new = num / den;
                if ((x_new - x).norm() < mp.convergence_eps) { x = x_new; break; }
                x = x_new;
            }
            mode_pos[a] = x;
            mode_rot[a] = Ra;
        }
        // 联合空间合并:位置<merge_distance 且 朝向夹角<merge_angle 才并簇。
        std::vector<int> cluster_of(n, -1);
        std::vector<Eigen::Vector3d>    cluster_pos;   // 簇参考位置(首个 mode)
        std::vector<Eigen::Quaterniond> cluster_rot;   // 簇参考朝向(首个 mode)
        for (int a = 0; a < n; ++a) {
            int found = -1;
            for (int c = 0; c < (int)cluster_pos.size(); ++c) {
                if ((mode_pos[a] - cluster_pos[c]).norm() >= mp.merge_distance) continue;
                if (mode_rot[a].angularDistance(cluster_rot[c]) >= mp.merge_angle) continue;
                found = c; break;
            }
            if (found < 0) {
                found = (int)cluster_pos.size();
                cluster_pos.push_back(mode_pos[a]);
                cluster_rot.push_back(mode_rot[a]);
            }
            cluster_of[a] = found;
        }
        // 每 cluster 选真实候选:分数高、离簇中心近(位置)、朝向与簇参考接近。
        for (int c = 0; c < (int)cluster_pos.size(); ++c) {
            int best = -1;
            double best_key = -kInf;
            for (int a = 0; a < n; ++a) {
                if (cluster_of[a] != c) continue;
                const RawViewpointCandidate& ca = cands[idx[a]];
                const double pdist = (ca.camera_position - cluster_pos[c]).norm()
                                     / mp.sigma_position;
                const double adist = ca.camera_orientation.angularDistance(cluster_rot[c])
                                     / sig_o;
                const double key = ca.viewpoint_score - 0.5 * pdist
                                   - 0.5 * mp.lambda_orientation * adist;
                if (key > best_key) { best_key = key; best = a; }
            }
            if (best < 0) continue;
            const RawViewpointCandidate& cb = cands[idx[best]];
            RepresentativeViewpoint rv;
            rv.node_id = node_counter++;
            rv.frontier_id = cb.frontier_id;
            rv.camera_position = cb.camera_position;
            rv.camera_orientation = cb.camera_orientation;
            rv.view_direction = cb.view_direction;
            rv.look_at = cb.look_at;
            rv.joint_state = cb.ik_solution;
            rv.unknown_density = cb.unknown_density;
            rv.geometric_visibility = cb.geometric_visibility;
            rv.semantic_prior = cb.semantic_prior;
            rv.viewpoint_score = cb.viewpoint_score;
            rv.environment_version = cb.environment_version;
            reps.push_back(rv);
        }
    }
    return reps;
}

// ============================================================================
// ★★★ 全局聚合(跨 frontier):把"位置相近 + 方向相似"的候选合成一个新点
//   (平均位置 + 平均方向,不落在任何真实 frontier 上)。用位置×方向网格分桶
//   O(n) 降数;每桶输出一个 RepresentativeViewpoint(未打分)。
//   - merge_distance: 位置网格边长(m);merge_angle_rad: 方向 az/el 分箱边长(rad)。
//   ⚠️ v1 用 azimuth/elevation 分箱近似"方向角距离<merge_angle";极点/绕回处有
//      轻微伪影,但对降数足够;后续可换球面网格。
// ============================================================================
inline std::vector<RepresentativeViewpoint>
aggregateByPositionDirection(const std::vector<RawViewpointCandidate>& cands,
                             double merge_distance, double merge_angle_rad) {
    struct Key {
        int px, py, pz, iaz, iel;
        bool operator==(const Key& o) const {
            return px==o.px && py==o.py && pz==o.pz && iaz==o.iaz && iel==o.iel;
        }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            size_t h = 1469598103934665603ULL;
            for (int v : {k.px, k.py, k.pz, k.iaz, k.iel}) {
                h ^= static_cast<size_t>(static_cast<int64_t>(v));
                h *= 1099511628211ULL;
            }
            return h;
        }
    };
    struct Accum {
        Eigen::Vector3d pos_sum{0,0,0}, dir_sum{0,0,0}, f_sum{0,0,0};
        int n = 0; int frontier_id = -1;
    };
    const double inv_md  = 1.0 / std::max(1e-6, merge_distance);
    const double inv_ang = 1.0 / std::max(1e-6, merge_angle_rad);
    std::unordered_map<Key, Accum, KeyHash> buckets;
    buckets.reserve(cands.size() / 8 + 16);
    for (const auto& c : cands) {
        Eigen::Vector3d d = c.view_direction;
        if (d.norm() < 1e-9) continue;
        d.normalize();
        const double az = std::atan2(d.y(), d.x());                 // [-pi, pi]
        const double el = std::asin(std::clamp(d.z(), -1.0, 1.0));  // [-pi/2, pi/2]
        Key k{ (int)std::floor(c.camera_position.x() * inv_md),
               (int)std::floor(c.camera_position.y() * inv_md),
               (int)std::floor(c.camera_position.z() * inv_md),
               (int)std::floor(az * inv_ang),
               (int)std::floor(el * inv_ang) };
        Accum& a = buckets[k];
        if (a.n == 0) a.frontier_id = c.frontier_id;
        a.pos_sum += c.camera_position;
        a.dir_sum += d;
        a.f_sum   += c.frontier_point;
        ++a.n;
    }
    std::vector<RepresentativeViewpoint> out;
    out.reserve(buckets.size());
    int id = 0;
    for (auto& kv : buckets) {
        const Accum& a = kv.second;
        RepresentativeViewpoint r;
        r.node_id = id++;
        r.frontier_id = a.frontier_id;
        r.camera_position = a.pos_sum / a.n;
        Eigen::Vector3d md = a.dir_sum;
        if (md.norm() < 1e-9) md = Eigen::Vector3d(1, 0, 0);
        md.normalize();
        r.view_direction  = md;
        r.frontier_point  = a.f_sum / a.n;
        r.look_at         = r.camera_position + md;
        r.camera_orientation = cameraOrientationLookAt(r.camera_position, r.look_at);
        out.push_back(r);
    }
    return out;
}

// ============================================================================
// Phase 6: 工作空间 3D A* motion cost(绕 octomap occupied,路径长度=cost)
// ============================================================================

struct MotionCostResult {
    bool   reachable = false;
    double cost = kInf;
    std::vector<Eigen::Vector3d> path;
    std::string failure_reason;
};

class MotionCostEstimator {
public:
    virtual ~MotionCostEstimator() = default;
    // 从 start 到 goal 的绕障路径代价(工作空间)。
    virtual MotionCostResult estimate(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& goal,
                                      const octomap::OcTree& tree) const = 0;
};

struct AStarParams {
    double grid_res = 0.05;          // A* 栅格分辨率(可粗于 octomap)
    Eigen::Vector3d aabb_min{-1, -1, -0.2};   // 搜索包围盒(工作空间外扩)
    Eigen::Vector3d aabb_max{ 1,  1,  1.2};
    double inflation = 0.0;          // 障碍膨胀半径(m),0=不膨胀
    int    max_expansions = 200000;  // 展开上限,防爆
};

// 真实工作空间 A*:26 连通栅格,occupied(可选膨胀)不可通行,unknown/free 可通行,
// 边代价=欧氏,启发式=欧氏。路径长度即 motion cost。
class WorkspaceAStarCostEstimator : public MotionCostEstimator {
public:
    explicit WorkspaceAStarCostEstimator(const AStarParams& p) : p_(p) {}

    MotionCostResult estimate(const Eigen::Vector3d& start,
                              const Eigen::Vector3d& goal,
                              const octomap::OcTree& tree) const override {
        MotionCostResult res;
        const double gr = std::max(1e-3, p_.grid_res);
        auto toCell = [&](const Eigen::Vector3d& w) {
            return Cell{(int)std::floor(w.x() / gr), (int)std::floor(w.y() / gr),
                        (int)std::floor(w.z() / gr)};
        };
        auto toWorld = [&](const Cell& c) {
            return Eigen::Vector3d((c.x + 0.5) * gr, (c.y + 0.5) * gr, (c.z + 0.5) * gr);
        };
        auto inAabb = [&](const Eigen::Vector3d& w) {
            return w.x() >= p_.aabb_min.x() && w.x() <= p_.aabb_max.x() &&
                   w.y() >= p_.aabb_min.y() && w.y() <= p_.aabb_max.y() &&
                   w.z() >= p_.aabb_min.z() && w.z() <= p_.aabb_max.z();
        };
        auto occupied = [&](const Eigen::Vector3d& w) -> bool {
            if (p_.inflation <= 0.0) {
                octomap::OcTreeNode* n = tree.search(octomap::point3d(w.x(), w.y(), w.z()));
                return n != nullptr && tree.isNodeOccupied(n);
            }
            const double step = std::max(gr, tree.getResolution());
            for (double dx = -p_.inflation; dx <= p_.inflation + 1e-9; dx += step)
              for (double dy = -p_.inflation; dy <= p_.inflation + 1e-9; dy += step)
                for (double dz = -p_.inflation; dz <= p_.inflation + 1e-9; dz += step) {
                    octomap::OcTreeNode* n = tree.search(
                        octomap::point3d(w.x() + dx, w.y() + dy, w.z() + dz));
                    if (n != nullptr && tree.isNodeOccupied(n)) return true;
                }
            return false;
        };

        const Cell cs = toCell(start), cg = toCell(goal);
        if (!inAabb(start) || !inAabb(goal)) { res.failure_reason = "endpoint_out_of_aabb"; return res; }
        if (occupied(goal)) { res.failure_reason = "goal_in_obstacle"; return res; }

        std::unordered_map<Cell, double, CellHash> g;
        std::unordered_map<Cell, Cell, CellHash> parent;
        auto heur = [&](const Cell& c) { return (toWorld(c) - goal).norm(); };

        using PQItem = std::pair<double, Cell>;   // f, cell
        // 只按 f 排序(min-heap);f 相等时不比较 Cell(Cell 无序关系)。
        auto pq_cmp = [](const PQItem& a, const PQItem& b) { return a.first > b.first; };
        std::priority_queue<PQItem, std::vector<PQItem>, decltype(pq_cmp)> open(pq_cmp);
        g[cs] = 0.0;
        open.push({heur(cs), cs});
        int expansions = 0;

        while (!open.empty()) {
            auto [f, c] = open.top(); open.pop();
            (void)f;
            if (c == cg) {
                res.reachable = true;
                res.cost = g[cg];
                // 回溯路径
                Cell cur = cg;
                while (!(cur == cs)) {
                    res.path.push_back(toWorld(cur));
                    auto it = parent.find(cur);
                    if (it == parent.end()) break;
                    cur = it->second;
                }
                res.path.push_back(toWorld(cs));
                std::reverse(res.path.begin(), res.path.end());
                return res;
            }
            if (++expansions > p_.max_expansions) { res.failure_reason = "expansion_limit"; return res; }
            const double gc = g[c];
            for (int dx = -1; dx <= 1; ++dx)
              for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    Cell nb{c.x + dx, c.y + dy, c.z + dz};
                    Eigen::Vector3d nw = toWorld(nb);
                    if (!inAabb(nw)) continue;
                    if (occupied(nw)) continue;
                    const double step_cost = std::sqrt((double)(dx * dx + dy * dy + dz * dz)) * gr;
                    const double ng = gc + step_cost;
                    auto it = g.find(nb);
                    if (it == g.end() || ng < it->second) {
                        g[nb] = ng;
                        parent[nb] = c;
                        open.push({ng + heur(nb), nb});
                    }
                }
        }
        res.failure_reason = "no_path";
        return res;
    }

private:
    struct Cell { int x, y, z; bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; } };
    struct CellHash {
        size_t operator()(const Cell& c) const noexcept {
            size_t h = (size_t)(c.x * 73856093) ^ (size_t)(c.y * 19349663) ^ (size_t)(c.z * 83492791);
            return h;
        }
    };
    AStarParams p_;
};

// ============================================================================
// Phase 7: Greedy utility 选择(current→candidate 星形图)
// ============================================================================

enum class GreedyUtilityMode { WeightedSum, ScoreCostRatio };

struct GreedyParams {
    double lambda_score  = 0.7;
    double lambda_motion = 0.3;
    double epsilon = 1e-6;
    GreedyUtilityMode mode = GreedyUtilityMode::WeightedSum;

    // λ_score + λ_motion = 1;自动归一化,非法明确报错。
    void normalizeOrThrow() {
        if (!(lambda_score >= 0.0 && lambda_motion >= 0.0))
            throw std::invalid_argument("GreedyParams: λ 必须非负");
        const double s = lambda_score + lambda_motion;
        if (s <= 1e-9)
            throw std::invalid_argument("GreedyParams: λ_score+λ_motion=0,非法");
        lambda_score /= s; lambda_motion /= s;
    }
};

// 对可达候选的 motion cost 做 min-max 归一化;只有一个可达 → ĉ=0。
// 写回 reps[*].normalized_motion_cost(仅 edge_feasible 者)。
inline void normalizeMotionCosts(std::vector<RepresentativeViewpoint>& reps,
                                 double epsilon) {
    double cmin = kInf, cmax = -kInf;
    int n_feasible = 0;
    for (auto& r : reps) if (r.edge_feasible && std::isfinite(r.motion_cost)) {
        cmin = std::min(cmin, r.motion_cost);
        cmax = std::max(cmax, r.motion_cost);
        ++n_feasible;
    }
    for (auto& r : reps) {
        if (!r.edge_feasible || !std::isfinite(r.motion_cost)) { r.normalized_motion_cost = 0.0; continue; }
        if (n_feasible <= 1) { r.normalized_motion_cost = 0.0; continue; }
        r.normalized_motion_cost = (r.motion_cost - cmin) / std::max(cmax - cmin, epsilon);
    }
}

inline double greedyUtility(double p_i, double chat_i, double raw_cost,
                            const GreedyParams& gp) {
    if (gp.mode == GreedyUtilityMode::ScoreCostRatio)
        return p_i / (raw_cost + gp.epsilon);
    return gp.lambda_score * p_i - gp.lambda_motion * chat_i;   // weighted_sum
}

// 计算所有 reps 的 greedy_utility(需先 normalizeMotionCosts),返回 utility 最高且
// 满足(edge_feasible & 未在 failed 集合)的下标;无可选返回 -1。
// failed:本 cycle 已验证失败(Infinity 边)的 node_id 集合。
inline int selectBestByUtility(std::vector<RepresentativeViewpoint>& reps,
                               const std::vector<int>& failed_node_ids,
                               const GreedyParams& gp) {
    auto is_failed = [&](int id) {
        return std::find(failed_node_ids.begin(), failed_node_ids.end(), id) != failed_node_ids.end();
    };
    int best = -1;
    double best_u = -kInf;
    for (auto& r : reps) {
        if (!r.edge_feasible || !std::isfinite(r.motion_cost)) { r.greedy_utility = -kInf; continue; }
        r.greedy_utility = greedyUtility(r.viewpoint_score, r.normalized_motion_cost,
                                         r.motion_cost, gp);
        if (is_failed(r.node_id)) continue;
        if (r.greedy_utility > best_u) { best_u = r.greedy_utility; best = &r - &reps[0]; }
    }
    return best;
}

}  // namespace hppt_baseline
