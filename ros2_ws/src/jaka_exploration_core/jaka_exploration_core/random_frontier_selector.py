#!/usr/bin/env python3
"""Random frontier selector — baseline for HPP-PT comparison.

Same input/output contract as hppt_boundary_ros, same frontier extraction
(free voxel with >=1 unknown 6-neighbour), same workspace filter, same
standoff/look_at formula from resolveViewpoint. The ONLY difference: the
target frontier is chosen uniformly at random instead of via clustering,
IG scoring, and HPP-PT graph search.

This is the "no-brain" baseline for the shelf coverage comparison test:
if the smart planner cannot beat random selection on coverage, that is a
signal the smart cost is not paying off.

Contract (matches hppt_boundary_ros exactly):
  in:  /octomap_binary                     octomap_msgs/Octomap
       /team/environment_status_json       std_msgs/String  (latest env_version)
       /team/motion_result_json            std_msgs/String  (task done)
  out: /team/viewpoint_task_json           std_msgs/String

Task JSON schema (same as hppt_boundary_ros):
  {
    "task_id": "random_view_<counter>_<ns>",
    "environment_version": <int>,
    "goal": {
      "type": "viewpoint",
      "frame_id": "world",
      "position": [x, y, z],
      "look_at":  [x, y, z]
    },
    "plan_only": false,
    "allowed_planning_time": <float>,
    "num_planning_attempts": <int>,
    "position_tolerance": <float>,
    "orientation_tolerance": <float>,
    "velocity_scaling": <float>,
    "acceleration_scaling": <float>
  }
"""
import json
import math
import random
import time

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


MAX_DEPTH = 16


class RandomFrontierSelector(Node):
    def __init__(self):
        super().__init__("random_frontier_selector")

        # ---- Same defaults as hppt_boundary_ros (see README parameter table). ----
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("env_status_topic", "/team/environment_status_json")
        self.declare_parameter("task_topic", "/team/viewpoint_task_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")

        self.declare_parameter("base_x", 0.0)
        self.declare_parameter("base_y", 0.0)
        self.declare_parameter("base_z", 0.0)
        self.declare_parameter("reach_min", 0.60)
        self.declare_parameter("reach_max", 0.85)
        self.declare_parameter("z_min", 0.05)
        self.declare_parameter("z_max", 1.20)
        self.declare_parameter("observation_standoff", 0.35)
        self.declare_parameter("observation_min_z", 0.25)
        self.declare_parameter("observation_min_z_low", 0.10)
        self.declare_parameter("require_outward_look", True)
        self.declare_parameter("frontier_max_range_factor", 1.5)

        # ---- Task publishing ----
        self.declare_parameter("moveit_allowed_planning_time", 5.0)
        self.declare_parameter("moveit_num_planning_attempts", 10)
        self.declare_parameter("moveit_position_tolerance", 0.03)
        self.declare_parameter("moveit_orientation_tolerance", 0.35)
        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)

        # ---- Loop pacing ----
        self.declare_parameter("task_timeout_sec", 60.0)
        self.declare_parameter("post_result_delay_sec", 0.5)
        self.declare_parameter("blacklist_radius", 0.20)
        self.declare_parameter("blacklist_duration_sec", 60.0)
        self.declare_parameter("visited_radius", 0.12)
        self.declare_parameter("visited_duration_sec", 45.0)
        self.declare_parameter("random_seed", -1)

        gp = self.get_parameter
        self._world_frame = gp("world_frame").value
        self._base = (gp("base_x").value, gp("base_y").value, gp("base_z").value)
        self._reach_min = float(gp("reach_min").value)
        self._reach_max = float(gp("reach_max").value)
        self._z_min = float(gp("z_min").value)
        self._z_max = float(gp("z_max").value)
        self._standoff = float(gp("observation_standoff").value)
        self._obs_min_z = float(gp("observation_min_z").value)
        self._obs_min_z_low = float(gp("observation_min_z_low").value)
        self._outward = bool(gp("require_outward_look").value)
        self._far_factor = float(gp("frontier_max_range_factor").value)

        self._plan_time = float(gp("moveit_allowed_planning_time").value)
        self._plan_attempts = int(gp("moveit_num_planning_attempts").value)
        self._pos_tol = float(gp("moveit_position_tolerance").value)
        self._ori_tol = float(gp("moveit_orientation_tolerance").value)
        self._vs = float(gp("velocity_scaling").value)
        self._as = float(gp("acceleration_scaling").value)

        self._task_timeout = float(gp("task_timeout_sec").value)
        self._post_result_delay = float(gp("post_result_delay_sec").value)
        self._bl_radius = float(gp("blacklist_radius").value)
        self._bl_dur = float(gp("blacklist_duration_sec").value)
        self._vis_radius = float(gp("visited_radius").value)
        self._vis_dur = float(gp("visited_duration_sec").value)

        seed = int(gp("random_seed").value)
        if seed >= 0:
            random.seed(seed)
        # Otherwise system entropy; result varies run to run.

        # ---- State ----
        self._latest_leaves = None
        self._latest_res = None
        self._env_version = None
        self._active_task_id = None
        self._active_deadline = None
        self._task_counter = 0
        # blacklist / visited: [(x, y, z, expire_wall_time)]
        self._blacklist = []
        self._visited = []

        # ---- Pub/Sub ----
        pub_qos = 10
        env_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        oct_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._task_pub = self.create_publisher(
            String, gp("task_topic").value, pub_qos
        )
        self.create_subscription(
            Octomap, gp("octomap_topic").value, self._on_octomap, oct_qos
        )
        self.create_subscription(
            String, gp("env_status_topic").value, self._on_env_status, env_status_qos
        )
        self.create_subscription(
            String, gp("result_topic").value, self._on_result, 10
        )
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "random_frontier_selector started. "
            f"reach=[{self._reach_min:.2f},{self._reach_max:.2f}] "
            f"z=[{self._z_min:.2f},{self._z_max:.2f}] "
            f"standoff={self._standoff:.2f} "
            f"outward={self._outward} "
            f"seed={'auto' if seed < 0 else seed}"
        )

    # ================= Callbacks =================
    def _on_octomap(self, msg):
        if not msg.binary:
            return
        res = float(msg.resolution)
        if res <= 0:
            return
        data = bytes(b & 0xFF for b in msg.data)
        if not data:
            self._latest_leaves = []
            self._latest_res = res
            return
        self._latest_leaves = self._parse_binary(data, res)
        self._latest_res = res

    def _on_env_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"env_status parse error: {e}")
            return
        if payload.get("accepted") is True and "version" in payload:
            v = int(payload["version"])
            if self._env_version is None or v != self._env_version:
                self.get_logger().info(f"env_version updated: {v}")
                self._env_version = v

    def _on_result(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"result parse error: {e}")
            return
        task_id = payload.get("task_id")
        if task_id != self._active_task_id:
            return  # not ours

        success = bool(payload.get("success", False))
        err = payload.get("error_type", "")
        goal = self._active_goal
        self._active_task_id = None
        self._active_deadline = None

        if goal is not None:
            now = time.monotonic()
            if success:
                self._visited.append((goal[0], goal[1], goal[2], now + self._vis_dur))
            else:
                # Blacklist retryable-or-not (mirror hppt_boundary_ros default).
                self._blacklist.append(
                    (goal[0], goal[1], goal[2], now + self._bl_dur)
                )
        self.get_logger().info(
            f"result task_id={task_id} success={success} error={err}"
        )
        # Short pause before next pick to let octomap incorporate new obs.
        self._next_pick_earliest = time.monotonic() + self._post_result_delay

    # ================= Main loop =================
    def _tick(self):
        now = time.monotonic()
        # Expire stale blacklist/visited entries.
        self._blacklist = [e for e in self._blacklist if e[3] > now]
        self._visited = [e for e in self._visited if e[3] > now]

        # Handle task timeout.
        if (self._active_task_id is not None
                and self._active_deadline is not None
                and now > self._active_deadline):
            self.get_logger().warn(
                f"task {self._active_task_id} timed out; giving up"
            )
            if self._active_goal is not None:
                self._blacklist.append(
                    (self._active_goal[0], self._active_goal[1],
                     self._active_goal[2], now + self._bl_dur)
                )
            self._active_task_id = None
            self._active_deadline = None
            self._active_goal = None

        if self._active_task_id is not None:
            return  # waiting on current task
        if getattr(self, "_next_pick_earliest", 0.0) > now:
            return
        if self._env_version is None:
            return
        if self._latest_leaves is None:
            return

        # Extract frontiers and filter.
        candidates = self._collect_candidates(
            self._latest_leaves, self._latest_res
        )
        if not candidates:
            return

        # Uniform random pick.
        idx = random.randrange(len(candidates))
        coord, unknown_dir = candidates[idx]
        goal, look_at = self._resolve_viewpoint(coord, unknown_dir)

        self._task_counter += 1
        task_id = f"random_view_{self._task_counter}_{time.time_ns()}"
        task = {
            "task_id": task_id,
            "environment_version": self._env_version,
            "goal": {
                "type": "viewpoint",
                "frame_id": self._world_frame,
                "position": [float(goal[0]), float(goal[1]), float(goal[2])],
                "look_at":  [float(look_at[0]), float(look_at[1]), float(look_at[2])],
            },
            "plan_only": False,
            "allowed_planning_time": self._plan_time,
            "num_planning_attempts": self._plan_attempts,
            "position_tolerance": self._pos_tol,
            "orientation_tolerance": self._ori_tol,
            "velocity_scaling": self._vs,
            "acceleration_scaling": self._as,
        }
        msg = String()
        msg.data = json.dumps(task, separators=(",", ":"))
        self._task_pub.publish(msg)

        self._active_task_id = task_id
        self._active_deadline = now + self._task_timeout
        self._active_goal = (float(goal[0]), float(goal[1]), float(goal[2]))
        self.get_logger().info(
            f"published {task_id}: "
            f"pos=({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f}) "
            f"look_at=({look_at[0]:.3f},{look_at[1]:.3f},{look_at[2]:.3f}) "
            f"|candidates|={len(candidates)}"
        )

    # ================= Candidate collection =================
    def _collect_candidates(self, leaves, res):
        """Return list of (frontier_coord, unknown_dir) for every leaf that
        passes the workspace/outward filters. Same semantics as
        hppt_boundary_ros extractFreeUnknownBoundary + per-voxel filters,
        minus clustering / IG / HPP-PT scoring.
        """
        inv_res = 1.0 / res
        known = self._build_known_set(leaves, res)
        # 6-neighbour direction vectors.
        neigh_dirs = (
            (+1, 0, 0), (-1, 0, 0),
            (0, +1, 0), (0, -1, 0),
            (0, 0, +1), (0, 0, -1),
        )
        base_x, base_y, base_z = self._base
        far_limit = self._far_factor * self._reach_max
        far_limit_sq = far_limit * far_limit
        reach_min_sq = self._reach_min * self._reach_min
        reach_max_sq = self._reach_max * self._reach_max

        cand = []
        for cx, cy, cz, size, is_occ in leaves:
            if is_occ:
                continue
            # Frontier: >=1 face-neighbour unknown.
            half = size * 0.5 + res * 0.5
            unknown_sum = [0.0, 0.0, 0.0]
            n_unknown = 0
            for dx, dy, dz in neigh_dirs:
                nx = cx + dx * half
                ny = cy + dy * half
                nz = cz + dz * half
                key = (
                    math.floor(nx * inv_res),
                    math.floor(ny * inv_res),
                    math.floor(nz * inv_res),
                )
                if key not in known:
                    unknown_sum[0] += dx
                    unknown_sum[1] += dy
                    unknown_sum[2] += dz
                    n_unknown += 1
            if n_unknown == 0:
                continue

            # Hard distance cull.
            dxb = cx - base_x
            dyb = cy - base_y
            dzb = cz - base_z
            d2 = dxb * dxb + dyb * dyb + dzb * dzb
            if d2 > far_limit_sq:
                continue
            # z bounds.
            if cz < self._z_min or cz > self._z_max:
                continue
            # Reach range.
            if d2 < reach_min_sq or d2 > reach_max_sq:
                continue
            # Blacklist / visited.
            if self._near_any(cx, cy, cz, self._blacklist, self._bl_radius):
                continue
            if self._near_any(cx, cy, cz, self._visited, self._vis_radius):
                continue

            # Averaged unknown direction (matches P0-1 in README).
            ud = self._normalize(unknown_sum)
            if ud is None:
                continue

            # Outward-look check on frontier coord itself.
            if self._outward:
                # look_at direction goes ALONG unknown_dir; base direction
                # is (coord - base). Require (unknown_dir) . (coord - base) > 0.
                dot = ud[0] * dxb + ud[1] * dyb + ud[2] * dzb
                if dot <= 0.0:
                    continue

            cand.append(((cx, cy, cz), ud))
        return cand

    def _resolve_viewpoint(self, coord, unknown_dir):
        cx, cy, cz = coord
        ux, uy, uz = unknown_dir
        # Camera placement: standoff back from frontier along unknown_dir.
        gx = cx - ux * self._standoff
        gy = cy - uy * self._standoff
        gz = cz - uz * self._standoff
        # Adaptive lower z clamp.
        eff_min_z = self._obs_min_z
        if cz < self._obs_min_z:
            eff_min_z = max(self._obs_min_z_low,
                            min(self._obs_min_z, cz - 0.05))
        gz = min(max(gz, eff_min_z), self._z_max)
        # look_at: past frontier by (standoff + 1m).
        look_ahead = self._standoff + 1.0
        lx = cx + ux * look_ahead
        ly = cy + uy * look_ahead
        lz = cz + uz * look_ahead
        return (gx, gy, gz), (lx, ly, lz)

    # ================= OctoMap binary parser =================
    def _parse_binary(self, data, res):
        leaves = []
        idx = [0]
        data_len = len(data)
        root_size = res * (2 ** MAX_DEPTH)

        def recurse(cx, cy, cz, size):
            if idx[0] + 2 > data_len:
                return
            b1 = data[idx[0]]
            b2 = data[idx[0] + 1]
            idx[0] += 2
            bits = b1 | (b2 << 8)
            child_size = size * 0.5
            offset = child_size * 0.5
            for octant in range(8):
                pair = (bits >> (octant * 2)) & 0b11
                if pair == 0b00:
                    continue
                ox = offset if (octant & 1) else -offset
                oy = offset if (octant & 2) else -offset
                oz = offset if (octant & 4) else -offset
                ccx = cx + ox
                ccy = cy + oy
                ccz = cz + oz
                if pair == 0b01:
                    leaves.append((ccx, ccy, ccz, child_size, False))
                elif pair == 0b10:
                    leaves.append((ccx, ccy, ccz, child_size, True))
                else:
                    recurse(ccx, ccy, ccz, child_size)

        recurse(0.0, 0.0, 0.0, root_size)
        return leaves

    def _build_known_set(self, leaves, res):
        known = set()
        inv_res = 1.0 / res
        for cx, cy, cz, size, _ in leaves:
            k = max(1, int(round(size * inv_res)))
            base_x = math.floor((cx - size * 0.5) * inv_res)
            base_y = math.floor((cy - size * 0.5) * inv_res)
            base_z = math.floor((cz - size * 0.5) * inv_res)
            for dx in range(k):
                for dy in range(k):
                    for dz in range(k):
                        known.add((base_x + dx, base_y + dy, base_z + dz))
        return known

    # ================= Utilities =================
    @staticmethod
    def _normalize(v):
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n < 1e-9:
            return None
        return (v[0] / n, v[1] / n, v[2] / n)

    @staticmethod
    def _near_any(x, y, z, entries, radius):
        r2 = radius * radius
        for ex, ey, ez, _ in entries:
            dx = x - ex
            dy = y - ey
            dz = z - ez
            if dx * dx + dy * dy + dz * dz < r2:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = RandomFrontierSelector()
    node._active_goal = None
    node._next_pick_earliest = 0.0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
