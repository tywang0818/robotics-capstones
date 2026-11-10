// hppt_stage_algos.hpp
// Stage A (frontier 聚类 + Gonzalez FPS) 与 Stage B1 (视锥 raycast 信息增益)
// 的纯函数实现，供主节点与性能测试共同复用。
// 依赖：octomap / Eigen / STL；不含 ROS 依赖。

#pragma once

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace hppt_stage {

// --- BoundaryPoint 与 hppt_boundary_ros.cpp 中同名结构保持二进制兼容
struct BoundaryPoint {
    octomap::point3d coord;
    double           probability;
    octomap::point3d unknown_neighbor;
    // ★ P1-1: 每候选可覆盖 standoff（<0 表示用节点默认 observation_standoff_）
    double           standoff_override = -1.0;
};

struct BoundaryCluster {
    Eigen::Vector3d  centroid;
    octomap::point3d centroid_oct;
    octomap::point3d unknown_representative;
    Eigen::Vector3d  mean_unknown_dir = Eigen::Vector3d::Zero();  // ★ P0-2
    int              voxel_count = 0;    // super voxel 数
    int              fine_count  = 0;    // ★ P1-1: 原始细体素数，用于估算尺寸
};

// ========== Stage A: 26-邻接连通分量聚类（可选粗网格下采样）==========
// 两级：先把细体素按 (res * coarse_grid_factor) 的粗网格 hash 去重成 super voxel，
// 再对 super voxel 做 26-邻接 BFS。coarse_grid_factor=1 时退化为原来的行为。
//
// 每个 BoundaryCluster.voxel_count 记录的是"该 cluster 包含的 super voxel 数"，
// 而不是原始细体素数。因此配合 cluster_min_voxels 使用时要按 coarse 的粒度调。
inline std::vector<BoundaryCluster>
clusterFrontier(const std::vector<BoundaryPoint>& boundaries, double res,
                int coarse_grid_factor = 1) {
    using Key = std::tuple<int,int,int>;
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            auto h = std::hash<int64_t>{};
            return  h(std::get<0>(k) * 73856093LL)
                  ^ h(std::get<1>(k) * 19349663LL)
                  ^ h(std::get<2>(k) * 83492791LL);
        }
    };
    const int df = std::max(1, coarse_grid_factor);
    const double cell = res * static_cast<double>(df);
    auto make_key = [cell](const octomap::point3d& p) -> Key {
        return {
            static_cast<int>(std::lround(p.x() / cell)),
            static_cast<int>(std::lround(p.y() / cell)),
            static_cast<int>(std::lround(p.z() / cell)),
        };
    };

    // Step-1: 把细体素聚合到粗网格 super voxel 上。
    // 每个 super voxel 记录：成员 sum（供 centroid 均值）、成员数、
    //   累加的 unknown_dir（P0-2：per-cluster mean direction）、一个 rep。
    struct Super {
        Eigen::Vector3d      sum;
        Eigen::Vector3d      dir_sum;
        octomap::point3d     unknown_rep;
        int                  count;
    };
    std::unordered_map<Key, Super, KeyHash> supers;
    supers.reserve(boundaries.size() / std::max(1, df * df * df) * 2 + 16);
    for (const auto& bp : boundaries) {
        Key k = make_key(bp.coord);
        Eigen::Vector3d dir(
            static_cast<double>(bp.unknown_neighbor.x() - bp.coord.x()),
            static_cast<double>(bp.unknown_neighbor.y() - bp.coord.y()),
            static_cast<double>(bp.unknown_neighbor.z() - bp.coord.z()));
        if (dir.norm() > 1e-9) dir.normalize();
        auto it = supers.find(k);
        if (it == supers.end()) {
            supers.emplace(k, Super{
                Eigen::Vector3d(bp.coord.x(), bp.coord.y(), bp.coord.z()),
                dir, bp.unknown_neighbor, 1});
        } else {
            it->second.sum += Eigen::Vector3d(bp.coord.x(), bp.coord.y(), bp.coord.z());
            it->second.dir_sum += dir;
            ++it->second.count;
        }
    }

    // 把 super voxel 拉成数组便于 BFS 索引
    struct SuperNode {
        Key                  key;
        Eigen::Vector3d      centroid;         // super voxel 内细体素中心均值
        Eigen::Vector3d      mean_dir;         // ★ P0-2: 归一化平均 unknown 方向
        octomap::point3d     unknown_rep;
        int                  fine_count;       // 聚合的细体素数
    };
    std::vector<SuperNode> nodes;
    nodes.reserve(supers.size());
    std::unordered_map<Key, int, KeyHash> key_to_idx;
    key_to_idx.reserve(supers.size() * 2);
    for (auto& kv : supers) {
        int idx = static_cast<int>(nodes.size());
        Eigen::Vector3d mean = kv.second.sum / static_cast<double>(kv.second.count);
        Eigen::Vector3d dir  = kv.second.dir_sum;
        if (dir.norm() > 1e-9) dir.normalize();
        nodes.push_back(SuperNode{
            kv.first, mean, dir, kv.second.unknown_rep, kv.second.count});
        key_to_idx[kv.first] = idx;
    }

    // Step-2: 26-邻接 BFS on super voxels.
    std::vector<int> comp(nodes.size(), -1);
    std::vector<BoundaryCluster> clusters;
    clusters.reserve(64);
    std::vector<int> stack;
    stack.reserve(256);

    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (comp[i] >= 0) continue;
        int cid = static_cast<int>(clusters.size());
        BoundaryCluster c{};
        Eigen::Vector3d sum(0, 0, 0);
        Eigen::Vector3d dir_sum(0, 0, 0);   // ★ P0-2

        stack.clear();
        stack.push_back(i);
        comp[i] = cid;
        while (!stack.empty()) {
            int u = stack.back(); stack.pop_back();
            const SuperNode& sv = nodes[u];
            sum += sv.centroid;
            // 用 fine_count 加权，避免单细胞 super 主导 direction
            dir_sum += sv.mean_dir * static_cast<double>(std::max(1, sv.fine_count));
            c.fine_count += sv.fine_count;
            if (c.voxel_count == 0) c.unknown_representative = sv.unknown_rep;
            ++c.voxel_count;  // ★ 该 cluster 的 super voxel 数

            const Key& k = sv.key;
            for (int dx = -1; dx <= 1; ++dx)
              for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    Key nk{ std::get<0>(k) + dx,
                            std::get<1>(k) + dy,
                            std::get<2>(k) + dz };
                    auto it = key_to_idx.find(nk);
                    if (it == key_to_idx.end()) continue;
                    if (comp[it->second] >= 0) continue;
                    comp[it->second] = cid;
                    stack.push_back(it->second);
                }
        }
        Eigen::Vector3d mean = sum / static_cast<double>(c.voxel_count);
        c.centroid = mean;
        c.centroid_oct = octomap::point3d(
            static_cast<float>(mean.x()),
            static_cast<float>(mean.y()),
            static_cast<float>(mean.z()));
        if (dir_sum.norm() > 1e-9) c.mean_unknown_dir = dir_sum.normalized();
        clusters.push_back(std::move(c));
    }
    return clusters;
}

// ========== Stage A: Gonzalez 最远点采样 ==========
inline std::vector<BoundaryCluster>
farthestPointSample(const std::vector<BoundaryCluster>& clusters,
                    const Eigen::Vector3d& anchor, int K) {
    const int N = static_cast<int>(clusters.size());
    if (N == 0 || K <= 0) return {};
    if (N <= K) return clusters;

    std::vector<double> mind2(N, std::numeric_limits<double>::infinity());
    for (int i = 0; i < N; ++i) {
        mind2[i] = (clusters[i].centroid - anchor).squaredNorm();
    }
    std::vector<BoundaryCluster> picked;
    picked.reserve(K);
    for (int step = 0; step < K; ++step) {
        int best = -1;
        double best_d = -1.0;
        for (int i = 0; i < N; ++i) {
            if (mind2[i] > best_d) { best_d = mind2[i]; best = i; }
        }
        if (best < 0) break;
        picked.push_back(clusters[best]);
        mind2[best] = -1.0;
        for (int i = 0; i < N; ++i) {
            if (mind2[i] < 0.0) continue;
            double d2 = (clusters[i].centroid - clusters[best].centroid).squaredNorm();
            if (d2 < mind2[i]) mind2[i] = d2;
        }
    }
    return picked;
}

// ★★★ TIER-3 Informed FPS: Gonzalez FPS 用 (min_dist × weight) 作为 score,
// 而不是仅仅 min_dist。weight 反映每个 cluster 的先验价值(例如 fine_count
// 归一化后再乘一个 "靠近已知物体" 的 attraction 因子),让 K 个候选既保持
// 空间发散,又向 "大且靠近物体" 的 cluster 倾斜 —— 修正原 FPS 把柜面这种
// "小且集中" cluster 稀释掉的偏差。
//
// weights.size() 必须 == clusters.size(),值应 >= 0。全 1 时退化到普通 FPS。
inline std::vector<BoundaryCluster>
informedFarthestPointSample(const std::vector<BoundaryCluster>& clusters,
                             const std::vector<double>& weights,
                             const Eigen::Vector3d& anchor, int K) {
    const int N = static_cast<int>(clusters.size());
    if (N == 0 || K <= 0) return {};
    if (N <= K) return clusters;
    // 无权重或权重长度不匹配 → 回落到普通 FPS
    if (static_cast<int>(weights.size()) != N) {
        return farthestPointSample(clusters, anchor, K);
    }

    std::vector<double> mind2(N, std::numeric_limits<double>::infinity());
    std::vector<char>   taken(N, 0);
    for (int i = 0; i < N; ++i) {
        mind2[i] = (clusters[i].centroid - anchor).squaredNorm();
    }
    std::vector<BoundaryCluster> picked;
    picked.reserve(K);
    for (int step = 0; step < K; ++step) {
        int best = -1;
        double best_score = -1.0;
        for (int i = 0; i < N; ++i) {
            if (taken[i]) continue;
            // score = min_dist × weight。sqrt(mind2) 只用其单调关系不影响排序,
            // 但要保证 weight=0 的 cluster 被绝对排到最后。
            double w = weights[i];
            if (w < 0.0) w = 0.0;
            double d = std::sqrt(mind2[i]);
            double s = d * w;
            if (s > best_score) { best_score = s; best = i; }
        }
        if (best < 0) break;
        picked.push_back(clusters[best]);
        taken[best] = 1;
        for (int i = 0; i < N; ++i) {
            if (taken[i]) continue;
            double d2 = (clusters[i].centroid - clusters[best].centroid).squaredNorm();
            if (d2 < mind2[i]) mind2[i] = d2;
        }
    }
    return picked;
}

// ========== Stage B1: 视锥 raycast 信息增益 ==========
// 从相机位姿 (goal, look_at) 出发在视锥内均匀采样 num_rays 条射线，
// 每条 castRay(ignoreUnknown=true) 找第一个 occupied 或走到 max_range，
// 沿途以 tree.resolution() 为步长采样 unknown 体素。返回 IG_norm ∈ [0,1]，
// 1 = 每条射线都走满 max_range 且全程 unknown。
// ★★★ TIER-2 修改:hit_ratio 通过 out_hit_ratio 指针返回(可选,nullptr = 不需要)。
// 老调用点(测试文件)不传该参数即可保持二进制/源码兼容。
// hit_ratio = 击中 known-occupied 的射线数 / 总射线数。它是"看到实体物体"
// 的强信号,与原 IG(全程 unknown 越多越好)互补。
// ★★★ TIER-3 (occlusion-aware): out_peek_score = "透过遮挡看到 unknown"
// 的强信号。每条 blocked 射线贡献 (unknown_length_before_hit / max_range)。
// 未命中的射线贡献 0(它们只是空穿 unknown/free,不算"从缝隙里窥视")。
// 综合起来:peek_score 高 ⇔ 相机确实透过障碍缝隙揭示了 unknown+物体边界。
inline double computeInformationGain(
    octomap::OcTree& tree,
    const Eigen::Vector3d& goal,
    const Eigen::Vector3d& look_at,
    double fov_deg,
    double max_range_m,
    int    num_rays,
    double* out_hit_ratio = nullptr,
    double* out_peek_score = nullptr)
{
    if (out_hit_ratio) *out_hit_ratio = 0.0;
    if (out_peek_score) *out_peek_score = 0.0;
    if (num_rays <= 0 || max_range_m <= 0.0 || fov_deg <= 0.0) return 0.0;
    Eigen::Vector3d axis = look_at - goal;
    if (axis.norm() < 1e-6) return 0.0;
    axis.normalize();
    Eigen::Vector3d helper = (std::abs(axis.z()) < 0.9)
        ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    Eigen::Vector3d u_basis = axis.cross(helper).normalized();
    Eigen::Vector3d w_basis = axis.cross(u_basis);

    const double res = tree.getResolution();
    const int max_steps = std::max(
        1, static_cast<int>(std::round(max_range_m / res)));
    const double half_angle = 0.5 * fov_deg * M_PI / 180.0;
    const double cos_half = std::cos(half_angle);
    const double golden = 0.6180339887498949;

    long unknown_total = 0;
    long hit_count = 0;   // ★ TIER-2: 击中 known-occupied 的射线计数
    double peek_sum = 0.0;   // ★ TIER-3: blocked 射线的 unknown_length/max_range 累加
    for (int i = 0; i < num_rays; ++i) {
        double u1 = (static_cast<double>(i) + 0.5) /
                    static_cast<double>(num_rays);
        double u2 = std::fmod(golden * i, 1.0);
        double cos_theta = 1.0 - u1 * (1.0 - cos_half);
        double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
        double phi = 2.0 * M_PI * u2;
        Eigen::Vector3d local(sin_theta * std::cos(phi),
                              sin_theta * std::sin(phi),
                              cos_theta);
        Eigen::Vector3d dir_w =
            local.x() * u_basis + local.y() * w_basis + local.z() * axis;
        dir_w.normalize();

        octomap::point3d origin(
            static_cast<float>(goal.x()),
            static_cast<float>(goal.y()),
            static_cast<float>(goal.z()));
        octomap::point3d ray_dir(
            static_cast<float>(dir_w.x()),
            static_cast<float>(dir_w.y()),
            static_cast<float>(dir_w.z()));
        octomap::point3d hit;
        double dist_limit = max_range_m;
        bool blocked = tree.castRay(
            origin, ray_dir, hit, /*ignoreUnknown=*/true, max_range_m);
        if (blocked) {
            octomap::point3d d = hit - origin;
            double hit_dist = std::sqrt(
                d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
            if (hit_dist < dist_limit) dist_limit = hit_dist;
            ++hit_count;   // ★ TIER-2: 命中 known-occupied 计数
        }
        int steps_this_ray = std::max(
            1, static_cast<int>(std::round(dist_limit / res)));
        long unknown_this_ray = 0;
        for (int s = 1; s <= steps_this_ray; ++s) {
            double dist_s = s * res;
            octomap::point3d q(
                static_cast<float>(goal.x() + dir_w.x() * dist_s),
                static_cast<float>(goal.y() + dir_w.y() * dist_s),
                static_cast<float>(goal.z() + dir_w.z() * dist_s));
            if (tree.search(q) == nullptr) {
                ++unknown_total;
                ++unknown_this_ray;
            }
        }
        if (blocked) {
            // Ray reached a known-occupied hit -> we "peeked" through
            // whatever unknown/free was in the way. Contribute the
            // unknown-fraction of that traversal.
            peek_sum += static_cast<double>(unknown_this_ray) / static_cast<double>(max_steps);
        }
    }
    double denom = static_cast<double>(num_rays) *
                   static_cast<double>(max_steps);
    double v = static_cast<double>(unknown_total) / denom;
    if (out_hit_ratio) {
        *out_hit_ratio = static_cast<double>(hit_count) /
                         static_cast<double>(num_rays);
    }
    if (out_peek_score) {
        *out_peek_score = std::clamp(
            peek_sum / static_cast<double>(num_rays), 0.0, 1.0);
    }
    return std::clamp(v, 0.0, 1.0);
}


// ★★★ TIER-3 (A): occlusion_score = "unknown 被遮挡的程度"。
// 度量方式:frontier 体素 p 周围 radius 球内,只统计 "d_unk 反向半球"
// (即 unknown 的背面 + 侧面)里的 occupied 体素比例。
// - 开放空间的 frontier → 0
// - 货架格子里/被三面挡的 frontier → 高
// 语义:unknown 后面有墙 = "值得看"。
inline double computeOcclusionScore(
    octomap::OcTree& tree,
    const octomap::point3d& p,
    const octomap::point3d& unknown_neighbor,
    double radius)
{
    if (radius <= 0.0) return 0.0;
    octomap::point3d d = unknown_neighbor - p;
    double dn = std::sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    if (dn < 1e-6) return 0.0;
    double dux = d.x() / dn, duy = d.y() / dn, duz = d.z() / dn;
    double res = tree.getResolution();
    int steps = std::max(1, static_cast<int>(std::ceil(radius / res)));
    double r2 = radius * radius;
    int hemi_total = 0, hemi_occ = 0;
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                double qx = p.x() + dx * res;
                double qy = p.y() + dy * res;
                double qz = p.z() + dz * res;
                double vx = qx - p.x();
                double vy = qy - p.y();
                double vz = qz - p.z();
                double dist2 = vx*vx + vy*vy + vz*vz;
                if (dist2 > r2) continue;
                // Only count the hemisphere opposite the unknown direction
                double dot = vx*dux + vy*duy + vz*duz;
                if (dot >= 0.0) continue;
                ++hemi_total;
                auto* node = tree.search(octomap::point3d(
                    static_cast<float>(qx),
                    static_cast<float>(qy),
                    static_cast<float>(qz)));
                if (node && node->getOccupancy() >= 0.5) ++hemi_occ;
            }
        }
    }
    if (hemi_total == 0) return 0.0;
    return static_cast<double>(hemi_occ) / static_cast<double>(hemi_total);
}


// ★★★ TIER-3 (C): directional attraction = "视线**擦过**遮挡物边缘"。
// 意图:相机从缝隙里窥视时,sight-line 会 grazingly 掠过隔板/立柱边缘,
// 而不是"贴脸打在墙上"。所以奖励是 band-pass:
//   * perp_dist ≈ 0.5 * lateral_tol : peak 奖励(真擦过)
//   * perp_dist → 0                 : 视线打在墙上, camera 看不到墙背, 反奖励
//   * perp_dist > lateral_tol       : 视线远离物体, 中性 (=0)
// 老版本 exp(-perp/decay) 有 bug: perp=0 时给最高分, 结果是把"相机对着
// 遮挡物、镜头指向墙背 1m 处"当作最好视点 —— 对应用户观察到的
// "面向障碍物、背后虚空的 pick 分很高"问题。
inline double computeDirectionalAttraction(
    octomap::OcTree& tree,
    const Eigen::Vector3d& camera,
    const Eigen::Vector3d& look_at,
    double search_radius,
    double lateral_tol,
    double decay,
    double z_min)
{
    if (search_radius <= 0.0 || lateral_tol <= 0.0) return 0.0;
    Eigen::Vector3d axis = look_at - camera;
    double axis_len = axis.norm();
    if (axis_len < 1e-6) return 0.0;
    axis /= axis_len;
    double res = tree.getResolution();
    int steps = std::max(1, static_cast<int>(std::ceil(search_radius / res)));
    double min_perp = lateral_tol + 1.0;
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                double qx = look_at.x() + dx * res;
                double qy = look_at.y() + dy * res;
                double qz = look_at.z() + dz * res;
                if (qz <= z_min) continue;  // floor exclusion
                Eigen::Vector3d q(qx, qy, qz);
                Eigen::Vector3d r = q - camera;
                double along = r.dot(axis);
                if (along < 0.5 * axis_len) continue;  // must be in far half of sight line
                if (along > axis_len + lateral_tol) continue;
                double perp2 = r.squaredNorm() - along * along;
                if (perp2 < 0.0) perp2 = 0.0;
                double perp = std::sqrt(perp2);
                if (perp > lateral_tol) continue;
                auto* node = tree.search(octomap::point3d(
                    static_cast<float>(qx),
                    static_cast<float>(qy),
                    static_cast<float>(qz)));
                if (node && node->getOccupancy() >= 0.5 && perp < min_perp) {
                    min_perp = perp;
                }
            }
        }
    }
    if (min_perp > lateral_tol) return 0.0;
    // Band-pass shape: gaussian centered at ~0.5*lateral_tol with
    // std-dev ~0.3*lateral_tol. Peaks around 5cm for lateral_tol=10cm.
    // decay param used to normalise sharpness (compatibility, but the
    // primary shape is now the gaussian above).
    double optimal = 0.5 * lateral_tol;
    double sigma = std::max(1e-3, 0.3 * lateral_tol);
    double delta = min_perp - optimal;
    double band = std::exp(-(delta * delta) / (2.0 * sigma * sigma));
    // Extra hard-zero right at perp=0 in case the gaussian tail is not
    // tight enough for the current lateral_tol tuning.
    if (min_perp < 0.02) return 0.0;
    return band;
}


// ★★★ TIER-3 (LOS): 检查从 camera 到 look_at 的射线是否被 known-occupied
// 打断。返回 true = 视线通畅可看到 look_at 附近; false = 中途被墙挡。
// 用途:在 computeViewpointProbability 里当作 hard gate,把"贴脸看墙"、
// "视线穿过 shelf 隔板到墙后"这类 pathologic 候选整体归零 —— 无论
// attract/ig/occl 打成多少分,如果相机根本看不见 look_at 所在的 unknown,
// 该候选就毫无意义。
inline bool losClear(
    octomap::OcTree& tree,
    const Eigen::Vector3d& camera,
    const Eigen::Vector3d& look_at,
    double slack = 0.05)
{
    Eigen::Vector3d axis = look_at - camera;
    double axis_len = axis.norm();
    if (axis_len < 1e-6) return true;
    axis /= axis_len;
    octomap::point3d origin(
        static_cast<float>(camera.x()),
        static_cast<float>(camera.y()),
        static_cast<float>(camera.z()));
    octomap::point3d dir(
        static_cast<float>(axis.x()),
        static_cast<float>(axis.y()),
        static_cast<float>(axis.z()));
    octomap::point3d hit;
    // ignoreUnknown=true so hitting unknown space does NOT count as
    // blocked; we only care about hitting a known-occupied voxel before
    // reaching look_at.
    bool blocked = tree.castRay(origin, dir, hit, /*ignoreUnknown=*/true,
                                axis_len + slack);
    if (!blocked) return true;   // sight line reached look_at unobstructed
    // If hit is essentially AT look_at (within slack), still count as clear.
    double hx = hit.x() - camera.x();
    double hy = hit.y() - camera.y();
    double hz = hit.z() - camera.z();
    double hit_dist = std::sqrt(hx * hx + hy * hy + hz * hz);
    return hit_dist >= axis_len - slack;
}

}  // namespace hppt_stage
