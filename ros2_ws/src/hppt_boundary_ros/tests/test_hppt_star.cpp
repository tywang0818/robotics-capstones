// test_hppt_star.cpp
//
// Standalone C++ test for the HPP-PT core (raplab::RunAstarHPPPT)
// using a star graph: node 1 is the start, nodes 2..6 are five leaves.
//
// Test case 1  — same target probability, different edge costs.
//                HPP-PT should prefer visiting the cheapest reachable
//                leaf first (cost dominates).
//
// Test case 2  — same edge cost, different target probabilities.
//                HPP-PT should prefer visiting the highest-probability
//                leaf first (probability dominates).
//
// Build (via the hppt_boundary_ros package's CMakeLists.txt) then:
//    ros2 run hppt_boundary_ros test_hppt_star
// or from the install directory:
//    /root/ros2_ws/install/hppt_boundary_ros/lib/hppt_boundary_ros/test_hppt_star

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "graph.hpp"
#include "search_hpp_pt.hpp"

namespace {

void printHeader(const std::string& title) {
    std::cout << "\n=========================================================\n";
    std::cout << " " << title << "\n";
    std::cout << "=========================================================\n";
}

// Build a graph with STAR cost pattern but COMPLETE topology.
//
// Why not a pure star (only center<->leaf edges)?
//   RunAstarHPPPT requires visiting every node exactly once (its termination
//   condition is visited.size() == NumVertex()-1). In a pure star you have
//   to route back through the center, but 'visited' is a set — the center
//   would then be re-entered as an already-visited node, and the search
//   never terminates → _res.id stays uninitialized → segfault in
//   _BuildPath. See search_hpp_pt.cpp:276.
//
// Encoding:
//   edge (1, leaf_i) cost = edge_costs[i]                (the "spoke")
//   edge (leaf_i, leaf_j) cost = edge_costs[i] + edge_costs[j]
//       — equivalent to "leaf_i → center → leaf_j" via the star spokes.
//
// This preserves star semantics (leaf-to-leaf cost is the true detour
// cost through the center) while giving HPP-PT the completeness it needs.
raplab::SparseGraph buildStarGraph(const std::vector<double>& edge_costs) {
    raplab::SparseGraph g;
    raplab::CostVec cv(1);
    const size_t n = edge_costs.size();
    // spokes: center 1 <-> each leaf
    for (size_t i = 0; i < n; ++i) {
        long leaf = static_cast<long>(i + 2);
        cv[0] = edge_costs[i];
        g.AddEdge(1L,  leaf, cv);
        g.AddEdge(leaf, 1L,  cv);
    }
    // synthetic leaf-to-leaf via center (equivalent to star traversal cost)
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            long a = static_cast<long>(i + 2);
            long b = static_cast<long>(j + 2);
            cv[0] = edge_costs[i] + edge_costs[j];
            g.AddEdge(a, b, cv);
            g.AddEdge(b, a, cv);
        }
    }
    return g;
}

void runCase(const std::string& title,
             const std::vector<double>& edge_costs,
             const std::vector<double>& leaf_probs,
             double start_prob = 0.5)
{
    printHeader(title);

    // ---- pretty-print inputs ----
    std::cout << "Graph: star (complete topology, star-cost pattern)\n";
    std::cout << "       center=1, leaves=2.." << 1 + edge_costs.size() << "\n";
    std::cout << "Spoke costs (1<->leaf):\n";
    for (size_t i = 0; i < edge_costs.size(); ++i)
        std::cout << "    (1 <-> " << (i + 2) << ")  cost = "
                  << std::fixed << std::setprecision(3) << edge_costs[i] << "\n";
    std::cout << "Leaf-to-leaf cost = spoke_i + spoke_j (via center)\n";
    std::cout << "Target (leaf) probabilities:\n";
    for (size_t i = 0; i < leaf_probs.size(); ++i)
        std::cout << "    p[" << (i + 2) << "] = "
                  << std::fixed << std::setprecision(3) << leaf_probs[i] << "\n";
    std::cout << "Start prob p[1] = " << start_prob << "\n\n";

    // ---- build graph ----
    auto g = buildStarGraph(edge_costs);

    // HPP-PT probability vector layout: probs[node_id] for node_id 1..N.
    // Size must be large enough (>= N + 1) so probs[N] is valid indexing.
    std::vector<double> probs(2 + edge_costs.size(), 0.0);
    probs[1] = start_prob;
    for (size_t i = 0; i < leaf_probs.size(); ++i)
        probs[i + 2] = leaf_probs[i];

    // ---- run ----
    long vo = 1;
    double time_limit = 10.0;
    raplab::HPPPTResult res;
    int rc = raplab::RunAstarHPPPT(&g, vo, probs, time_limit, &res);

    // ---- print result ----
    std::cout << "\n---- HPP-PT result ----\n";
    std::cout << "return code       : " << rc << "\n";
    std::cout << "path (node order) : ";
    for (auto v : res.path) std::cout << v << " ";
    std::cout << "\n";
    std::cout << "path cost         : ";
    for (auto c : res.cost) std::cout << std::fixed << std::setprecision(3) << c << " ";
    std::cout << "\n";
    std::cout << "nodes generated   : " << res.n_generated << "\n";
    std::cout << "nodes expanded    : " << res.n_expanded << "\n";
    std::cout << "search time (s)   : " << res.rt_search << "\n";
    std::cout << "timeout           : " << (res.timeout ? "yes" : "no") << "\n";

    // ---- interpretation hint ----
    if (res.path.size() >= 2) {
        long first_visited = res.path[1];
        int idx = static_cast<int>(first_visited) - 2;   // leaf index 0..N-1
        if (idx >= 0 && idx < static_cast<int>(edge_costs.size())) {
            std::cout << "\nfirst visited leaf: " << first_visited
                      << "  (edge_cost=" << edge_costs[idx]
                      << ", prob="       << leaf_probs[idx] << ")\n";
        }
    }
}

}  // namespace

int main() {
    std::cout << "HPP-PT star-graph sanity test\n";

    // ------------------------------------------------------------------
    // Case 1: uniform probability, varying edge costs.
    // Every leaf is equally likely; algorithm should prefer the leaf
    // reachable through the cheapest edge (leaf 2, cost 1.0) first.
    // ------------------------------------------------------------------
    {
        std::vector<double> edge_costs = {1.0, 2.0, 3.0, 4.0, 5.0};
        std::vector<double> leaf_probs = {0.2, 0.2, 0.2, 0.2, 0.2};
        runCase("Case 1: uniform probability, varying edge costs",
                edge_costs, leaf_probs, /*start_prob=*/0.5);
    }

    // ------------------------------------------------------------------
    // Case 2: uniform edge cost, varying probabilities.
    // All edges cost 1.0; algorithm should prefer the highest-probability
    // leaf first (leaf 3 with p=0.5).
    // ------------------------------------------------------------------
    {
        std::vector<double> edge_costs = {1.0, 1.0, 1.0, 1.0, 1.0};
        std::vector<double> leaf_probs = {0.1, 0.5, 0.2, 0.3, 0.4};
        //                                p2   p3   p4   p5   p6
        runCase("Case 2: uniform edge cost, varying probabilities",
                edge_costs, leaf_probs, /*start_prob=*/0.5);
    }

    // ------------------------------------------------------------------
    // Bonus Case 3: mixed — high-prob leaf is also the farthest.
    // Useful to see the algorithm's trade-off between prob & cost.
    // ------------------------------------------------------------------
    {
        std::vector<double> edge_costs = {1.0, 2.0, 3.0, 4.0, 5.0};
        std::vector<double> leaf_probs = {0.1, 0.1, 0.1, 0.1, 0.6};
        //                                p2   p3   p4   p5   p6
        //   -> leaf 6 has both largest cost AND largest prob;
        //      does the search still go there first?
        runCase("Case 3 (bonus): high-prob leaf is farthest",
                edge_costs, leaf_probs, /*start_prob=*/0.5);
    }

    std::cout << "\nDone.\n";
    return 0;
}
