# HPP-T Frontier-to-Viewpoint Pipeline

This document describes the current `new_method` frontier pipeline from the first OctoMap frontier extraction to the final selected execution target.

## Summary

Current flow:

```text
OctoMap
  -> raw frontier extraction
  -> 33-direction ray sampling per frontier
  -> occupied truncation / known-free camera filtering
  -> position+direction aggregation
  -> U/V/S viewpoint scoring
  -> top-50 by score
  -> A* motion cost
  -> greedy utility ranking
  -> real execute
  -> failed candidates are set to INF within this cycle
  -> if all reachable candidates fail, stop exploration
```

## 1. Input and cycle gate

The node receives OctoMap data from `/octomap_binary`.

HPP-T does not immediately select points. A cycle only starts after a valid non-empty environment has been accepted by the downstream environment bridge. This avoids selecting goals while the initial scan or map setup is still incomplete.

Cycle start log:

```text
=========== HPP-PT cycle START (env_version=...) ===========
```

## 2. Raw frontier extraction

Function:

```cpp
extractFreeUnknownBoundary(tree)
```

Logic:

```text
For every free OctoMap leaf:
  if at least one of its 6-connected neighbors is unknown:
    mark this free voxel as a frontier
```

Each frontier stores:

```cpp
coord             // free voxel center
unknown_neighbor  // encoded direction toward unknown space
```

The unknown direction is computed from all unknown neighbor directions, not just the first one:

```text
unknown_dir = mean(all unknown neighbor offsets)
```

This makes the frontier direction point toward the center of the adjacent unknown region when multiple unknown faces exist.

## 3. Frontier seed construction

In the current greedy baseline path, raw frontiers are not pre-clustered. Each extracted boundary point becomes one seed:

```cpp
F  = bp.coord
ud = bp.unknown_neighbor - bp.coord
```

Where:

```text
F  = original frontier point
ud = direction from frontier toward unknown
```

If `ud` is degenerate, the fallback direction is:

```text
ud = F - base
```

## 4. 33-direction fan generation

For each frontier seed, the system generates a fan of view directions:

```cpp
generateViewDirections(s.ud, viewpoint_fan_angles_deg_, viewpoint_fan_azimuths_)
```

Conceptually:

```text
1 main direction
+ cone-angle / azimuth samples around that direction
```

These are the “33 rays” discussed in testing.

Each generated direction is called:

```text
dir
```

The candidate ray is defined as:

```cpp
P(t) = F - t * dir, t >= 0
```

Meaning:

```text
F is the original frontier.
dir is the camera forward direction.
F - t * dir walks from the frontier toward the free-space/camera side.
```

## 5. Reach shell intersection

Each ray is intersected with the robot reachable spherical shell:

```cpp
ws.intersectViewLine(F, dir)
```

The workspace constraints include:

```text
reach_min <= distance(base, camera_position) <= reach_max
z_min <= camera_position.z <= z_max
```

The result is a valid interval:

```text
[t0, t1]
```

Only positions inside this interval are considered as camera candidates.

## 6. Ray sampling and occupied truncation

The current implementation uses two step sizes:

```text
viewpoint_line_sample_step    = 0.15 m
viewpoint_line_collision_step = 0.03 m
```

Candidate positions are generated every `0.15 m`.

However, occupied collision checks are performed every `0.03 m` from the frontier to the candidate.

For each candidate `candidate_t`, the system checks:

```text
F -> candidate_t
```

If any occupied voxel is found along that segment:

```text
truncate this ray immediately
do not generate any farther candidate on this same ray
```

This is intended to prevent candidates from being generated behind an occupied obstacle such as the back panel of a shelf.

Relevant log fields:

```text
occ_checks
collision_step
sample_step
rej_occupied_trunc
```

## 7. Candidate camera-position filtering

For each candidate position:

```cpp
pos = F - candidate_t * dir
```

The system applies these filters.

### 7.1 Geometry and workspace

Reject if:

```cpp
isForbidden(pos) || isOutOfWorkspace(pos)
```

Logged as:

```text
rej_geo
```

### 7.2 Known-free camera position

The camera position must be known-free in the OctoMap.

If:

```cpp
tree.search(pos) == nullptr
```

then `pos` is unknown. The candidate is rejected, but the ray continues.

Logged as:

```text
rej_unknown
```

If `pos` is occupied, the ray is truncated:

```text
rej_occupied_trunc
```

### 7.3 Raw candidate creation

If the candidate survives filtering:

```cpp
RawViewpointCandidate rc;
rc.camera_position = pos;
rc.view_direction = dir;
rc.frontier_point = F;
rc.look_at = pos + dir;
rc.camera_orientation = cameraOrientationLookAt(pos, rc.look_at);
```

Important detail:

```text
look_at = pos + dir
```

It is not directly set to:

```text
look_at = F
```

So the raw candidate orientation follows the sampled direction, not necessarily the exact individual frontier point.

## 8. B2 generation log

Generation log:

```text
BASELINE B2 (gen): seeds=... gen=... -> raw=...
```

Meaning:

```text
seeds = number of raw frontier seeds
gen   = number of sampled positions attempted on rays
raw   = number of raw candidates surviving filtering
```

Example:

```text
BASELINE B2 (gen): seeds=10946 gen=37105 -> raw=33599
```

## 9. Position+direction aggregation

Current aggregation function:

```cpp
aggregateByPositionDirection(raws, merge_distance, merge_angle_rad)
```

Default parameters:

```text
aggregate_merge_distance = 0.15 m
aggregate_merge_angle    = 30 deg
```

This is not mean-shift. It is an O(n) grid-bucket aggregation over camera position and view direction.

### 9.1 Aggregation key

For each raw candidate:

```text
camera_position = p_i
view_direction  = d_i
```

Position bins:

```text
bucket_x = floor(p_i.x / 0.15)
bucket_y = floor(p_i.y / 0.15)
bucket_z = floor(p_i.z / 0.15)
```

Direction bins:

```text
azimuth   = atan2(d_i.y, d_i.x)
elevation = asin(d_i.z)

bucket_az = floor(azimuth / 30deg)
bucket_el = floor(elevation / 30deg)
```

Candidates in the same 5D bucket are merged.

### 9.2 Representative formula

For all candidates inside one bucket:

```text
p_rep = mean(p_i)
d_rep = normalize(sum(d_i))
F_rep = mean(F_i)
look_at_rep = p_rep + d_rep
orientation_rep = cameraOrientationLookAt(p_rep, look_at_rep)
```

Code equivalent:

```cpp
r.camera_position = a.pos_sum / a.n;
r.view_direction  = normalize(a.dir_sum);
r.frontier_point  = a.f_sum / a.n;
r.look_at         = r.camera_position + r.view_direction;
```

Important consequence:

```text
Scoring uses frontier_point = F_rep.
Execution orientation uses look_at = p_rep + d_rep.
```

Therefore, the RViz arrow may not point exactly to one original frontier voxel.

Aggregation log:

```text
BASELINE AGG (pos+dir): raw candidates -> aggregated points
```

## 10. U/V/S viewpoint scoring

Each aggregated representative is scored after aggregation:

```cpp
U = unknownDensity(tree, r.frontier_point, prob_density_radius_)
V = geometricVisibility(tree, r.camera_position, r.camera_orientation, ...)
S = computeSemanticRelevance(r.frontier_point)
```

Total score:

```text
viewpoint_score = 0.3 * U + 0.5 * V + 0.2 * S
```

Default weights:

```text
score_w_unknown    = 0.3
score_w_visibility = 0.5
score_w_semantic   = 0.2
```

Interpretation:

```text
U = local unknown density around the representative frontier point
V = geometric visibility / expected visible unknown from this camera pose
S = semantic relevance around the representative frontier point
```

This score is local per representative. A globally larger unknown region does not automatically get a higher score.

## 11. Top-50 score filter

After scoring:

```cpp
sort(reps by viewpoint_score descending)
reps.resize(motion_cost_top_n_)
```

Default:

```text
motion_cost_top_n = 50
```

Current behavior:

```text
Only top-50 by score are kept for A* and execution selection.
There is no current 50-100 or 100-150 fallback batch.
```

Log:

```text
BASELINE TOP-N: -> top-50 by score (best=...)
```

## 12. A* motion cost

For each top-50 representative:

```cpp
auto mc = est.estimate(current_position_, r.camera_position, tree);
r.motion_cost = mc.reachable ? mc.cost : INF;
r.edge_feasible = mc.reachable;
```

A* does not delete points from `reps`.

It marks unreachable points as:

```text
motion_cost = INF
edge_feasible = false
```

These points remain in the vector but are not selectable later.

Log:

```text
BASELINE B6 (A* motion cost): 50 reps, N reachable from current EE [...]
```

## 13. Greedy utility

For reachable candidates, motion costs are min-max normalized:

```text
normalized_motion_cost in [0, 1]
```

Default utility mode:

```text
weighted_sum
```

Formula:

```text
utility = 0.7 * viewpoint_score - 0.3 * normalized_motion_cost
```

Default weights:

```text
greedy_lambda_score  = 0.7
greedy_lambda_motion = 0.3
```

Unreachable candidates are effectively skipped:

```cpp
if (!r.edge_feasible || !isfinite(r.motion_cost)) {
    r.greedy_utility = -INF;
    continue;
}
```

## 14. B7 ranking log

Each execution attempt prints a ranking table:

```text
BASELINE B7 GREEDY RANKING
```

Each row contains:

```text
node=...
f=...
pos[x,y,z]
score=...(U=... V=... S=...)
Acost=...
ncost=...
util=...
reach=Y/N
```

This is the most useful log for explaining why a candidate was selected.

## 15. RViz top-5 visualization

Before dispatching each attempt, the node publishes the current top-5 ranked viewpoints.

Meaning:

```text
green = current best candidate that will be dispatched
cyan  = next ranked candidates
```

The visualization uses:

```text
camera_position
look_at
```

It does not directly visualize the original frontier voxel.

## 16. Real execute

The selected candidate is sent as a real execution task:

```cpp
dispatchExecuteAndWait(r.camera_position, r.look_at)
```

This waits for:

```text
/team/motion_result_json
```

Timeout:

```text
execute_wait_timeout_sec = 30 s
```

Success log:

```text
BASELINE B8: node ... EXECUTE success (attempt N) -> arm moved, cycle done.
```

## 17. Failure and within-cycle reselection

If execution fails:

```cpp
failed.push_back(r.node_id);
r.edge_feasible = false;
r.motion_cost = INF;
```

The failed node is only disabled for the current cycle. It is not permanently blacklisted across cycles in the greedy baseline path.

The system then recomputes normalized motion costs and selects the next best utility candidate.

Common failure types:

```text
blocked_by_unknown
nonconservative_timeout
trajectory_not_known_free
execute_wait_timeout
```

## 18. Attempt limit

The current attempt limit is dynamic:

```cpp
execute_attempt_limit = reachable
```

Where:

```text
reachable = number of top-50 candidates marked reachable by A*
```

Example:

```text
BASELINE B8: dynamic execute attempt limit=37
```

This means the cycle may try up to 37 candidates.

`max_execute_attempts=20` is currently ignored in this path.

## 19. Stop condition on all-candidate failure

Current added stop logic:

```cpp
if (!moved) {
    declareConverged("BASELINE ALL EXECUTE FAILED");
}
```

Meaning:

```text
If every A*-reachable candidate in the current top-50 fails execution,
stop immediately instead of waiting for repeated unproductive cycles.
```

Effects:

```text
publish /exploration_complete
rclcpp::shutdown()
container_start_all.sh cleans up the launch
```

## Key implementation notes

1. Aggregation is currently position+direction grid aggregation, not mean-shift.

2. A* does not physically remove points. It sets unreachable points to `INF`, making them unselectable.

3. The current pipeline only evaluates top-50 by viewpoint score. There is no implemented fallback to candidates 50-100 or 100-150.

4. Aggregated execution orientation uses:

```text
look_at = camera_position + mean_direction
```

not:

```text
look_at = frontier_point
```

5. A larger unknown region does not automatically win. The selected point depends on local U/V/S score and motion cost.
