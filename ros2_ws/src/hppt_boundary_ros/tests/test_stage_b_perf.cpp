// test_stage_b_perf.cpp
//
// Standalone C++ perf test for Stage B1：computeInformationGain()
// 视锥 raycast 信息增益估计。
//
// 构造 3 个合成 OcTree scenario：
//   1) open       — 全 unknown → IG ≈ 1（每条射线走满 max_range 都算 unknown）
//   2) wall_blind — max_range/2 处一堵墙，前面 unknown → IG ≈ 0.5
//                    （射线在 unknown 中走到墙前停，前半段全计 unknown）
//   3) box        — 视锥被 known-free 完全包住 → IG ≈ 0（没有任何 unknown）
//
// 每个场景扫 ray count ∈ {8, 16, 32, 64, 128, 256, 512}，
// 报告 IG 值 + 每次调用的 wall time；同时看 IG 是否随 rays 收敛。
//
// Build via ros2 pkg CMakeLists:
//   ros2 run hppt_boundary_ros test_stage_b_perf

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include "hppt_stage_algos.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0) {
    auto dt = Clock::now() - t0;
    return std::chrono::duration<double, std::milli>(dt).count();
}

// ---------- Scenario 生成 ----------
// scenario 布置在 x ∈ [0, +front_range] 上，相机站在原点，看向 +x。

// 1) open：什么都不插入 → 整个空间 unknown。IG 应逼近 1.0。
octomap::OcTree makeOpen(double res) {
    octomap::OcTree tree(res);
    // 不插任何 known 节点 —— tree.search() 处处返回 nullptr → 全 unknown
    return tree;
}

// 用整数索引循环避免浮点漂移。★ 插入必须落在 leaf **中心**(i+0.5)*res，
// 否则 float(i*res) 精度损失会让 coordToKey 掉到相邻 key（实测 key=i-1）。
// setNodeValue 直接钉住 log-odds，不受累加更新影响。
static void fillFree(octomap::OcTree& tree, double res,
                     int i_min, int i_max, int j_min, int j_max, int k_min, int k_max) {
    const float free_log = -2.0f;  // 强 free（clamping_min = -2.0 by default）
    const double h = 0.5 * res;
    for (int i = i_min; i <= i_max; ++i)
      for (int j = j_min; j <= j_max; ++j)
        for (int k = k_min; k <= k_max; ++k) {
            octomap::point3d p(static_cast<float>(i * res + h),
                               static_cast<float>(j * res + h),
                               static_cast<float>(k * res + h));
            tree.setNodeValue(p, free_log, /*lazy_eval=*/false);
        }
}
static void fillOccupied(octomap::OcTree& tree, double res,
                         int i, int j_min, int j_max, int k_min, int k_max) {
    const float occ_log = 3.5f;   // 强 occupied（clamping_max ≈ 3.5）
    const double h = 0.5 * res;
    for (int j = j_min; j <= j_max; ++j)
      for (int k = k_min; k <= k_max; ++k) {
          octomap::point3d p(static_cast<float>(i * res + h),
                             static_cast<float>(j * res + h),
                             static_cast<float>(k * res + h));
          tree.setNodeValue(p, occ_log, /*lazy_eval=*/false);
      }
}

// 2) wall_blind：只在 x = wall_x 处插一堵 occupied 墙（YZ 平面），
//    墙前面保持 unknown（不插入任何 free），模拟"相机看向前方 unknown，
//    途中有障碍打断"→ IG ≈ wall_x / max_range = 0.5。
octomap::OcTree makeWall(double res, double wall_x,
                         double half_yz = 1.5) {
    octomap::OcTree tree(res);
    int wall_i = static_cast<int>(std::round(wall_x / res));
    int hy = static_cast<int>(std::round(half_yz / res));
    fillOccupied(tree, res, wall_i, -hy, hy, -hy, hy);
    return tree;
}

// 3) box：整个视锥体积都插成 known-free → IG 应逼近 0。
octomap::OcTree makeBox(double res, double front_range,
                        double half_yz = 1.5) {
    octomap::OcTree tree(res);
    int xi = static_cast<int>(std::round(front_range / res));
    int hy = static_cast<int>(std::round(half_yz / res));
    fillFree(tree, res, 0, xi, -hy, hy, -hy, hy);
    return tree;
}

struct RunResult {
    int    rays;
    double ig;
    double ms;
};

// 复用同一棵 tree 多次调用，报告平均时长（每场景重复 5 次取均值）
RunResult sweepIG(octomap::OcTree& tree,
                  const Eigen::Vector3d& goal,
                  const Eigen::Vector3d& look_at,
                  double fov_deg, double max_range, int rays,
                  int repeats = 5) {
    double ig_last = 0.0;
    auto t0 = Clock::now();
    for (int r = 0; r < repeats; ++r) {
        ig_last = hppt_stage::computeInformationGain(
            tree, goal, look_at, fov_deg, max_range, rays);
    }
    double total_ms = ms_since(t0);
    return { rays, ig_last, total_ms / repeats };
}

void runScenario(const std::string& name,
                 octomap::OcTree& tree,
                 double fov_deg, double max_range) {
    Eigen::Vector3d goal(0, 0, 0);
    Eigen::Vector3d look_at(1, 0, 0);
    std::cout << "\n[" << name << "] fov=" << fov_deg
              << "° max_range=" << max_range << " m,  goal=(0,0,0) → +x\n";
    std::cout << "     rays |     IG    |  avg call (ms)  |  per-ray (µs)\n";
    std::cout << "  -------+-----------+-----------------+---------------\n";
    for (int rays : { 8, 16, 32, 64, 128, 256, 512 }) {
        auto r = sweepIG(tree, goal, look_at, fov_deg, max_range, rays);
        std::printf("  %6d |  %7.4f  |     %9.3f    |    %8.2f\n",
                    r.rays, r.ig, r.ms, r.ms * 1000.0 / r.rays);
    }
}

}  // namespace

int main() {
    const double res = 0.02;
    const double fov_deg = 70.0;
    const double max_range = 2.0;

    std::cout <<
        "\n========================================================================\n"
        " Stage B1 perf test — computeInformationGain()\n"
        "   resolution=" << res << " m, fov=" << fov_deg
        << "°, max_range=" << max_range << " m, 5x repeat/measurement\n"
        "   Expected: open→IG≈1, wall_blind(1m)→IG≈0.5, box→IG≈0\n"
        "========================================================================\n";

    {
        auto tree = makeOpen(res);
        runScenario("open (all unknown)", tree, fov_deg, max_range);
    }
    {
        auto tree = makeWall(res, /*wall_x=*/max_range * 0.5);
        runScenario("wall_blind at max_range/2 (unknown before wall)",
                    tree, fov_deg, max_range);
    }
    {
        // box 必须完全包住相机视锥，否则边缘 ray 会从盒子逸出而计入 unknown
        // 视锥基底半径 = max_range * sin(fov/2) = 2.0 * sin(35°) ≈ 1.147
        double half = max_range * std::sin(0.5 * fov_deg * M_PI / 180.0) + 0.2;
        auto tree = makeBox(res, /*front_range=*/max_range + 0.2, /*half_yz=*/half);
        runScenario("box (all known-free)", tree, fov_deg, max_range);
    }

    // 视场角敏感度：固定 rays=64，扫 FOV
    std::cout << "\n---- FOV sensitivity (open, rays=64) ----\n";
    std::cout << "  fov_deg |    IG\n";
    std::cout << "  --------+---------\n";
    {
        auto tree = makeOpen(res);
        Eigen::Vector3d goal(0, 0, 0), look_at(1, 0, 0);
        for (double fov : { 15.0, 30.0, 60.0, 90.0, 120.0, 160.0 }) {
            double ig = hppt_stage::computeInformationGain(
                tree, goal, look_at, fov, max_range, 64);
            std::printf("   %6.1f  |  %6.4f\n", fov, ig);
        }
    }

    // max_range 敏感度：固定 rays=64、fov=70，扫 range
    std::cout << "\n---- max_range sensitivity (open, rays=64, fov=70°) ----\n";
    std::cout << "  range(m) |    IG    |  ms/call\n";
    std::cout << "  ---------+----------+---------\n";
    {
        auto tree = makeOpen(res);
        Eigen::Vector3d goal(0, 0, 0), look_at(1, 0, 0);
        for (double r_m : { 0.5, 1.0, 2.0, 3.0, 4.0 }) {
            auto t0 = Clock::now();
            double ig = 0.0;
            for (int i = 0; i < 5; ++i) {
                ig = hppt_stage::computeInformationGain(
                    tree, goal, look_at, 70.0, r_m, 64);
            }
            double avg_ms = ms_since(t0) / 5.0;
            std::printf("   %5.2f   |  %6.4f  |   %6.3f\n", r_m, ig, avg_ms);
        }
    }

    std::cout << "\nDone.\n";
    return 0;
}
