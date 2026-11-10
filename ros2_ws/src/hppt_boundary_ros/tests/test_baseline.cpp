// test_baseline.cpp — new_method Greedy Baseline 纯算法单元测试(Phase 1/2/4/5/6/7)。
//   只依赖 octomap + Eigen,不依赖 ROS/MoveIt。
//   运行: ros2 run hppt_boundary_ros test_baseline   (返回码 0 = 全通过)
#include "hppt_baseline.hpp"

#include <cassert>
#include <cstdio>
#include <cmath>

using namespace hppt_baseline;

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("  [FAIL] %s\n", msg); ++g_fail; } \
                              else { std::printf("  [ ok ] %s\n", msg); } } while (0)

// 构造一个简单 octree:x∈[0,0.45] free,x∈{0.5,0.55} 一堵 2-voxel occupied 墙,
// 墙后(x>=0.6)不标 = unknown。⚠️ 用整数索引造景,避免 for(double) 累加到 y=0 时
// 的 ~1e-17 漂移把体素写到相邻 key(那会让墙/查询点错位成 unknown)。
static octomap::OcTree makeScene(double res = 0.05) {
    octomap::OcTree tree(res);
    auto C = [&](int i) { return i * res; };                     // 精确坐标
    for (int ix = 0; ix <= 9; ++ix)                              // x 0..0.45 free
      for (int iy = -6; iy <= 6; ++iy)                           // y -0.3..0.3
        for (int iz = 0; iz <= 12; ++iz)                         // z 0..0.6
            tree.updateNode(octomap::point3d(C(ix), C(iy), C(iz)), false);
    for (int ix = 10; ix <= 11; ++ix)                            // x 0.5,0.55 wall
      for (int iy = -6; iy <= 6; ++iy)
        for (int iz = 0; iz <= 12; ++iz)
            tree.updateNode(octomap::point3d(C(ix), C(iy), C(iz)), true);
    tree.updateInnerOccupancy();
    return tree;
}

static void testWorkspace() {
    std::printf("[Phase1] WorkspaceModel 球壳交线\n");
    WorkspaceLimits lim; lim.base = {0, 0, 0}; lim.reach_min = 0.3; lim.reach_max = 0.9;
    lim.z_min = 0.0; lim.z_max = 1.0;
    SphericalShellWorkspace ws(lim);
    CHECK(ws.contains(Eigen::Vector3d(0.5, 0, 0.3)), "壳内点 contains=true");
    CHECK(!ws.contains(Eigen::Vector3d(0.1, 0, 0.05)), "壳内球太近 contains=false");
    // frontier 在 x=0.6,朝 +x 看(unknown 方向 +x),相机在 -x 一侧
    auto seg = ws.intersectViewLine(Eigen::Vector3d(0.6, 0, 0.3), Eigen::Vector3d(1, 0, 0));
    CHECK(seg.has_value(), "视线与壳有交");
    if (seg) CHECK(seg->first >= 0.0 && seg->second > seg->first, "交区间 t>0 且非空");
}

static void testSampling() {
    std::printf("[Phase2] 方向集 + 沿线多距离采样\n");
    auto dirs = generateViewDirections(Eigen::Vector3d(1, 0, 0), {15, 30, 45}, 8);
    CHECK(dirs.size() == 1u + 3u * 8u, "方向数 = 1 + 3*8 = 25");
    for (auto& d : dirs) CHECK(std::abs(d.norm() - 1.0) < 1e-6, "方向已归一化");

    WorkspaceLimits lim; lim.base = {0, 0, 0}; lim.reach_min = 0.3; lim.reach_max = 0.9;
    lim.z_min = 0.0; lim.z_max = 1.0;
    SphericalShellWorkspace ws(lim);
    auto pts = sampleCameraPositionsAlongLine(Eigen::Vector3d(0.7, 0, 0.3),
                                              Eigen::Vector3d(1, 0, 0), ws, 0.05);
    CHECK(pts.size() >= 2, "同一方向采到多个不同距离相机位(>=2)");
    // 单调:沿 -x 方向 t 增大,x 减小
    bool multi_dist = pts.size() >= 2 && std::abs(pts.front().x() - pts.back().x()) > 0.05;
    CHECK(multi_dist, "多距离(相机位 x 明显不同)");
}

static void testScoring() {
    std::printf("[Phase4] 三项归一化打分\n");
    auto tree = makeScene();
    // frontier 就在墙前 free/unknown 交界附近
    double U = unknownDensity(tree, Eigen::Vector3d(0.45, 0, 0.3), 0.15);
    CHECK(U >= 0.0 && U <= 1.0, "unknownDensity ∈ [0,1]");
    // 相机在墙前朝 +x(看向墙后 unknown),部分射线应先到 unknown
    Eigen::Vector3d cam(0.2, 0, 0.3);
    auto orient = cameraOrientationLookAt(cam, Eigen::Vector3d(0.45, 0, 0.3));
    double V = geometricVisibility(tree, cam, orient, 70, 50, 1.0, 64);
    CHECK(V >= 0.0 && V <= 1.0, "geometricVisibility ∈ [0,1]");

    ScoreWeights w; w.w_unknown = 3; w.w_visibility = 5; w.w_semantic = 2;
    w.normalizeOrThrow();
    CHECK(std::abs(w.w_unknown + w.w_visibility + w.w_semantic - 1.0) < 1e-9, "权重归一化和=1");
    double p = viewpointScore(0.8, 0.6, 0.5, w);
    CHECK(p >= 0.0 && p <= 1.0, "viewpointScore ∈ [0,1]");
    bool threw = false;
    try { ScoreWeights bad; bad.w_unknown = 0; bad.w_visibility = 0; bad.w_semantic = 0;
          bad.normalizeOrThrow(); } catch (...) { threw = true; }
    CHECK(threw, "非法权重(全0)抛异常");
}

static void testMeanShift() {
    std::printf("[Phase5] per-frontier Weighted Mean Shift\n");
    std::vector<RawViewpointCandidate> cands;
    // frontier 0:两簇(x≈0.2 一簇 3 个、x≈0.6 一簇 2 个);frontier 1:一簇。
    auto mk = [&](int id, int fid, double x, double score) {
        RawViewpointCandidate c; c.candidate_id = id; c.frontier_id = fid;
        c.camera_position = {x, 0, 0.3}; c.viewpoint_score = score; c.ik_valid = true;
        c.ik_solution = {0.0}; return c;
    };
    cands.push_back(mk(0, 0, 0.20, 0.5));
    cands.push_back(mk(1, 0, 0.21, 0.9));   // 该簇最高分
    cands.push_back(mk(2, 0, 0.19, 0.4));
    cands.push_back(mk(3, 0, 0.60, 0.7));
    cands.push_back(mk(4, 0, 0.61, 0.6));
    cands.push_back(mk(5, 1, 0.40, 0.8));
    // 一个 ik 无效的应被排除
    auto bad = mk(6, 0, 0.20, 1.0); bad.ik_valid = false; cands.push_back(bad);

    MeanShiftParams mp; mp.sigma_position = 0.05; mp.merge_distance = 0.08;
    auto reps = meanShiftRepresentatives(cands, mp);
    // frontier 0 应聚成 2 个 mode,frontier 1 应 1 个 → 共 3 个代表
    CHECK(reps.size() == 3, "3 个代表视点(f0:2簇 + f1:1簇)");
    bool picked_high = false;
    for (auto& r : reps)
        if (r.frontier_id == 0 && std::abs(r.camera_position.x() - 0.21) < 0.02
            && std::abs(r.viewpoint_score - 0.9) < 1e-6) picked_high = true;
    CHECK(picked_high, "簇内选到最高分真实候选(非平均 Pose)");
    for (auto& r : reps) CHECK(!r.joint_state.empty(), "代表视点带 joint_state");
}

static void testMeanShiftOrientation() {
    std::printf("[Phase5] 方向门控(方案B):同位置反向朝向应分成两个代表\n");
    auto mk = [&](int id, int fid, Eigen::Vector3d pos, Eigen::Vector3d viewdir, double score) {
        RawViewpointCandidate c; c.candidate_id = id; c.frontier_id = fid;
        c.camera_position = pos; c.view_direction = viewdir.normalized();
        c.look_at = pos + viewdir.normalized();
        c.camera_orientation = cameraOrientationLookAt(pos, c.look_at);
        c.viewpoint_score = score; c.ik_valid = true; c.ik_solution = {0.0};
        return c;
    };
    Eigen::Vector3d P(0.4, 0.0, 0.3);
    std::vector<RawViewpointCandidate> cands;
    cands.push_back(mk(0, 5, P, Eigen::Vector3d(1, 0, 0), 0.7));    // 朝 +x
    cands.push_back(mk(1, 5, P, Eigen::Vector3d(-1, 0, 0), 0.6));   // 同位置,朝 -x(相反)

    MeanShiftParams mp; mp.sigma_position = 0.05; mp.merge_distance = 0.10;
    mp.merge_angle = 30.0 * M_PI / 180.0; mp.lambda_orientation = 1.0;
    mp.sigma_orientation = 0.52;
    auto reps = meanShiftRepresentatives(cands, mp);
    CHECK(reps.size() == 2, "同位置反向朝向 -> 2 个代表(方向门控生效)");

    // 关掉朝向(λ=0 且 merge_angle=π)→ 退化为纯位置聚合,应合并为 1 个
    MeanShiftParams mp0 = mp; mp0.lambda_orientation = 0.0; mp0.merge_angle = M_PI + 1.0;
    auto reps0 = meanShiftRepresentatives(cands, mp0);
    CHECK(reps0.size() == 1, "λ=0 且 merge_angle=π -> 退化纯位置聚合(1 个代表)");
}

static void testAggregation() {
    std::printf("[AGG] 位置+方向 全局聚合(跨 frontier 造新点)\n");
    auto mk = [&](Eigen::Vector3d pos, Eigen::Vector3d dir, int fid) {
        RawViewpointCandidate c; c.camera_position = pos;
        c.view_direction = dir.normalized();
        c.frontier_point = pos + dir.normalized() * 0.3; c.frontier_id = fid;
        return c;
    };
    std::vector<RawViewpointCandidate> cands;
    // 3 个几乎同位置、同方向(应合成 1 个);用 bin 中部坐标避免边界抖动
    cands.push_back(mk({0.55, 0.05, 0.35}, {1, 0, 0}, 0));
    cands.push_back(mk({0.57, 0.05, 0.37}, {1, 0, 0}, 1));
    cands.push_back(mk({0.55, 0.07, 0.35}, {1, 0, 0}, 2));
    // 同位置但反方向(应单独)
    cands.push_back(mk({0.55, 0.05, 0.35}, {-1, 0, 0}, 3));
    // 远位置、同方向(应单独)
    cands.push_back(mk({0.85, 0.05, 0.35}, {1, 0, 0}, 4));

    auto reps = aggregateByPositionDirection(cands, 0.10, 30.0 * M_PI / 180.0);
    CHECK(reps.size() == 3, "近3同向合1 + 反向1 + 远1 = 3 个聚合点");

    bool found_mean = false;
    for (const auto& r : reps) {
        Eigen::Vector3d expect(0.5567, 0.0567, 0.3567);   // 近3个的位置均值
        if ((r.camera_position - expect).norm() < 0.02 && r.view_direction.x() > 0.9)
            found_mean = true;
    }
    CHECK(found_mean, "近3同向聚合点 = 位置均值、方向 +x(造的新点)");
    for (const auto& r : reps) CHECK(r.joint_state.empty(), "聚合点无 IK(已删除)");
}

static void testAStar() {
    std::printf("[Phase6] 工作空间 A* motion cost(绕障)\n");
    auto tree = makeScene();
    AStarParams ap; ap.grid_res = 0.05; ap.aabb_min = {-0.1, -0.5, -0.1};
    ap.aabb_max = {1.1, 0.5, 0.8};
    WorkspaceAStarCostEstimator est(ap);
    // start 墙前,goal 墙前另一点(无需绕),直线可达
    auto r1 = est.estimate({0.2, -0.2, 0.3}, {0.2, 0.2, 0.3}, tree);
    CHECK(r1.reachable, "同侧点可达");
    CHECK(r1.cost >= 0.4 - 1e-6, "路径长度 >= 直线下界");
    // goal 在墙里 → 不可达
    auto r2 = est.estimate({0.2, 0, 0.3}, {0.5, 0, 0.3}, tree);
    CHECK(!r2.reachable, "目标在障碍内不可达");
    // 绕障:start 墙前,goal 墙后(墙有缺口?这里墙满,y 范围外可绕)
    auto r3 = est.estimate({0.3, 0, 0.3}, {0.3, 0.45, 0.3}, tree);
    CHECK(r3.reachable, "绕到墙侧点可达");
}

static void testQuaternionConvention() {
    std::printf("[Phase2] 相机姿态与 conversions.py::viewpoint_quaternion 一致性\n");
    // 参考实现(conversions.py): forward=norm(look_at-pos);
    //   right=norm(ref_up×forward)(|f·+Z|>0.98 时 ref_up=+Y);up=forward×right。
    auto ref = [](const Eigen::Vector3d& cam, const Eigen::Vector3d& look,
                  Eigen::Vector3d& R0, Eigen::Vector3d& R1, Eigen::Vector3d& R2) {
        Eigen::Vector3d f = (look - cam).normalized();
        Eigen::Vector3d up(0, 0, 1);
        if (std::abs(f.dot(up)) > 0.98) up = Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d right = up.cross(f).normalized();
        Eigen::Vector3d u = f.cross(right);
        R0 = right; R1 = u; R2 = f;
    };
    struct Case { Eigen::Vector3d cam, look; };
    Case cases[] = {
        {{0.2, 0.0, 0.3}, {0.5, 0.1, 0.3}},
        {{0.3, -0.2, 0.4}, {0.3, 0.4, 0.2}},
        {{0.0, 0.0, 0.2}, {0.0, 0.0, 0.9}},   // 近垂直 → 触发 ref_up 翻转
    };
    for (auto& c : cases) {
        Eigen::Matrix3d Rq = cameraOrientationLookAt(c.cam, c.look).toRotationMatrix();
        Eigen::Vector3d r0, r1, r2; ref(c.cam, c.look, r0, r1, r2);
        bool ok = (Rq.col(0) - r0).norm() < 1e-6 &&
                  (Rq.col(1) - r1).norm() < 1e-6 &&
                  (Rq.col(2) - r2).norm() < 1e-6;
        CHECK(ok, "四元数与 conversions.py 约定逐轴一致(+Z=forward, right=ref_up×f)");
    }
}

static void testGreedy() {
    std::printf("[Phase7] Greedy utility 选择 + 归一化 + Infinity 重选\n");
    std::vector<RepresentativeViewpoint> reps(3);
    for (int i = 0; i < 3; ++i) { reps[i].node_id = i; reps[i].edge_feasible = true; }
    reps[0].viewpoint_score = 0.9; reps[0].motion_cost = 1.0;   // 高分近
    reps[1].viewpoint_score = 0.5; reps[1].motion_cost = 0.2;   // 低分很近
    reps[2].viewpoint_score = 0.95; reps[2].motion_cost = 5.0;  // 最高分但很远

    GreedyParams gp; gp.lambda_score = 7; gp.lambda_motion = 3; gp.normalizeOrThrow();
    CHECK(std::abs(gp.lambda_score + gp.lambda_motion - 1.0) < 1e-9, "λ 归一化和=1");

    normalizeMotionCosts(reps, gp.epsilon);
    int best = selectBestByUtility(reps, {}, gp);
    CHECK(best >= 0, "选出最优候选");
    // node0: 0.7*0.9 - 0.3*((1-0.2)/(5-0.2)) ≈ 0.63-0.05=0.58
    // node2: 0.7*0.95 - 0.3*1 = 0.665-0.3=0.365  → node0 应胜
    CHECK(reps[best].node_id == 0, "utility 最高是 node0(分数/代价平衡)");

    // 模拟 node0 验证失败 → Infinity 重选,应回退到剩余里 utility 最高
    int best2 = selectBestByUtility(reps, {0}, gp);
    CHECK(best2 >= 0 && reps[best2].node_id != 0, "标记失败后重选到其它候选");

    // 单一可达 → ĉ=0
    std::vector<RepresentativeViewpoint> one(1);
    one[0].node_id = 0; one[0].edge_feasible = true; one[0].viewpoint_score = 0.5;
    one[0].motion_cost = 3.3;
    normalizeMotionCosts(one, gp.epsilon);
    CHECK(one[0].normalized_motion_cost == 0.0, "只有一个可达候选 ĉ=0");

    // ratio 模式
    GreedyParams gr; gr.mode = GreedyUtilityMode::ScoreCostRatio;
    double u = greedyUtility(0.8, 0.0, 0.2, gr);
    CHECK(std::abs(u - 0.8 / (0.2 + gr.epsilon)) < 1e-6, "ratio 模式 utility=p/(c+eps)");
}

int main() {
    std::printf("=== hppt_baseline 单元测试 (new_method) ===\n");
    testWorkspace();
    testSampling();
    testQuaternionConvention();
    testScoring();
    testMeanShift();
    testMeanShiftOrientation();
    testAggregation();
    testAStar();
    testGreedy();
    if (g_fail == 0) { std::printf("\n[ALL PASS]\n"); return 0; }
    std::printf("\n[%d CHECK(s) FAILED]\n", g_fail);
    return 1;
}
