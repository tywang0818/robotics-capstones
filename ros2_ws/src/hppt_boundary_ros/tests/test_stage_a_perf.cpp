// test_stage_a_perf.cpp
//
// Standalone C++ perf test for Stage A：
//   1) clusterFrontier()  — 26-邻接 BFS 聚类
//   2) farthestPointSample() — Gonzalez FPS
//
// 生成三种合成 frontier 分布：wall / blobs / noise，
// 在 5 档 N 下测端到端时延，并把 FPS 的空间散布度
// 与旧的 index-uniform sample 做对比。
//
// Build via ros2 pkg CMakeLists:
//   ros2 run hppt_boundary_ros test_stage_a_perf
// or direct:
//   /root/ros2_ws/install/hppt_boundary_ros/lib/hppt_boundary_ros/test_stage_a_perf

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "hppt_stage_algos.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using hppt_stage::BoundaryCluster;
using hppt_stage::BoundaryPoint;

double ms_since(Clock::time_point t0) {
    auto dt = Clock::now() - t0;
    return std::chrono::duration<double, std::milli>(dt).count();
}

// index-uniform sample —— tests/ 内部再现，用于跟 FPS 的空间散布对比。
std::vector<BoundaryPoint>
uniformSample(const std::vector<BoundaryPoint>& all, int k) {
    if (all.size() <= static_cast<size_t>(k)) return all;
    std::vector<BoundaryPoint> sampled;
    sampled.reserve(k);
    double step = static_cast<double>(all.size()) / k;
    for (int i = 0; i < k; ++i)
        sampled.push_back(all[static_cast<int>(i * step)]);
    return sampled;
}

// ====== 合成 frontier 生成器 ======
struct Gen {
    double res;
    std::mt19937 rng;
    Gen(uint32_t seed, double r) : res(r), rng(seed) {}

    // "wall"：单个 z=const 平面上密铺 N 个体素（模拟大片 frontier 面）
    std::vector<BoundaryPoint> wall(int N) {
        std::vector<BoundaryPoint> out;
        out.reserve(N);
        int side = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(N))));
        for (int i = 0; i < side && (int)out.size() < N; ++i) {
            for (int j = 0; j < side && (int)out.size() < N; ++j) {
                float x = 0.0f;
                float y = static_cast<float>(i * res);
                float z = static_cast<float>(j * res);
                BoundaryPoint bp;
                bp.coord = octomap::point3d(x, y, z);
                bp.probability = 0.0;
                bp.unknown_neighbor = octomap::point3d(x - static_cast<float>(res), y, z);
                out.push_back(bp);
            }
        }
        return out;
    }

    // "blobs"：M 个中心随机置在 [-1,1]^3，每个球心附近实心填 ~N/M 体素
    std::vector<BoundaryPoint> blobs(int N, int M = 5) {
        std::vector<BoundaryPoint> out;
        out.reserve(N);
        std::uniform_real_distribution<double> pos(-1.0, 1.0);
        int per_blob = std::max(1, N / M);
        int side = static_cast<int>(std::ceil(std::cbrt(
            static_cast<double>(per_blob))));
        for (int b = 0; b < M && (int)out.size() < N; ++b) {
            double cx = std::round(pos(rng) / res) * res;
            double cy = std::round(pos(rng) / res) * res;
            double cz = std::round(pos(rng) / res) * res;
            for (int i = 0; i < side && (int)out.size() < N; ++i) {
              for (int j = 0; j < side && (int)out.size() < N; ++j) {
                for (int k = 0; k < side && (int)out.size() < N; ++k) {
                    BoundaryPoint bp;
                    bp.coord = octomap::point3d(
                        static_cast<float>(cx + i * res),
                        static_cast<float>(cy + j * res),
                        static_cast<float>(cz + k * res));
                    bp.probability = 0.0;
                    bp.unknown_neighbor = octomap::point3d(
                        bp.coord.x() + static_cast<float>(res), bp.coord.y(), bp.coord.z());
                    out.push_back(bp);
                }
              }
            }
        }
        return out;
    }

    // "noise"：完全随机的孤立体素（几乎每个体素自成一 cluster）
    std::vector<BoundaryPoint> noise(int N) {
        std::vector<BoundaryPoint> out;
        out.reserve(N);
        std::uniform_real_distribution<double> pos(-2.0, 2.0);
        for (int i = 0; i < N; ++i) {
            double x = std::round(pos(rng) / res) * res;
            double y = std::round(pos(rng) / res) * res;
            double z = std::round(pos(rng) / res) * res;
            BoundaryPoint bp;
            bp.coord = octomap::point3d(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
            bp.probability = 0.0;
            bp.unknown_neighbor = octomap::point3d(
                bp.coord.x() + static_cast<float>(res), bp.coord.y(), bp.coord.z());
            out.push_back(bp);
        }
        return out;
    }
};

// 最近点对距离 —— O(K^2)，K 一般 ≤ 30，够用
double minPairDist(const std::vector<Eigen::Vector3d>& pts) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < pts.size(); ++i)
        for (size_t j = i + 1; j < pts.size(); ++j) {
            double d = (pts[i] - pts[j]).norm();
            if (d < best) best = d;
        }
    return std::isfinite(best) ? best : 0.0;
}

// 汇总统计
struct ClusterStats {
    size_t clusters;
    int    max_voxels;
    double avg_voxels;
};
ClusterStats stats(const std::vector<BoundaryCluster>& cs) {
    ClusterStats s{ cs.size(), 0, 0.0 };
    long long sum = 0;
    for (const auto& c : cs) {
        if (c.voxel_count > s.max_voxels) s.max_voxels = c.voxel_count;
        sum += c.voxel_count;
    }
    s.avg_voxels = cs.empty() ? 0.0 : static_cast<double>(sum) / cs.size();
    return s;
}

void runOne(const std::string& pattern,
            const std::vector<BoundaryPoint>& b,
            double res, int K) {
    auto t0 = Clock::now();
    auto clusters = hppt_stage::clusterFrontier(b, res);
    double t_cluster = ms_since(t0);
    auto cs = stats(clusters);

    Eigen::Vector3d anchor(0, 0, 0);
    auto t1 = Clock::now();
    auto picked = hppt_stage::farthestPointSample(clusters, anchor, K);
    double t_fps = ms_since(t1);

    std::vector<Eigen::Vector3d> pick_pts;
    pick_pts.reserve(picked.size());
    for (const auto& c : picked) pick_pts.push_back(c.centroid);
    double fps_min = minPairDist(pick_pts);

    // baseline：index-uniform sample，把每个 boundary 都当候选，取 K 个体素坐标
    auto uni = uniformSample(b, K);
    std::vector<Eigen::Vector3d> uni_pts;
    uni_pts.reserve(uni.size());
    for (const auto& bp : uni)
        uni_pts.emplace_back(bp.coord.x(), bp.coord.y(), bp.coord.z());
    double uni_min = minPairDist(uni_pts);

    std::printf(
        "  %-6s N=%-6zu | cluster: %5zu clusters, max_vox=%-5d avg_vox=%6.1f | "
        "t_cluster=%7.3f ms | K=%2d picked=%2zu t_fps=%7.3f ms | "
        "min_pair FPS=%6.3f m  uniform=%6.3f m  (FPS/uni=%.2fx)\n",
        pattern.c_str(), b.size(),
        cs.clusters, cs.max_voxels, cs.avg_voxels,
        t_cluster,
        K, picked.size(), t_fps,
        fps_min, uni_min,
        uni_min > 1e-9 ? (fps_min / uni_min) : 0.0);
}

}  // namespace

int main() {
    const double res = 0.02;
    const std::vector<int> Ns = {100, 500, 2000, 10000, 50000};
    const std::vector<int> Ks = {10, 30};

    std::cout <<
        "\n========================================================================\n"
        " Stage A perf test — clusterFrontier() + farthestPointSample()\n"
        "   resolution = " << res << " m ; anchor at origin ; "
        "min_pair = min pairwise 3D distance among K picks\n"
        "========================================================================\n";

    for (int K : Ks) {
        std::cout << "\n---------------- K = " << K << " -----------------\n";
        Gen gen(42, res);
        for (int N : Ns) {
            auto wall  = gen.wall(N);
            auto blobs = gen.blobs(N, /*M=*/5);
            auto noise = gen.noise(N);
            runOne("wall",  wall,  res, K);
            runOne("blobs", blobs, res, K);
            runOne("noise", noise, res, K);
        }
    }
    std::cout << "\nDone.\n";
    return 0;
}
