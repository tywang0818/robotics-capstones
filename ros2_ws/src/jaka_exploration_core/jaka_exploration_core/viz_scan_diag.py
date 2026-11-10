#!/usr/bin/env python3
"""Diagnostic node for the 'yellow blocks persist after scanning' bug.

Runs alongside a normal shelf test, exits when
/jaka_initial_scan_complete flips true. Captures:

  Test 1 — viz vs octomap agreement:
    - Latest /frontier_markers snapshot. New frontier_viz emits ONE
      Marker per size bucket (all under ns="unknown"), so we accumulate
      cube centres AND sizes across every marker with that ns.
    - Latest /octomap_binary snapshot parsed to native-depth UNKNOWN
      leaves (state=0b00). Same workspace + size filter as
      frontier_viz. This is what viz SHOULD emit.
    - Compare viz-yellow set vs octomap-unknown set at fine-cell
      resolution: any fine cell covered by a viz cube goes into
      viz_coverage; likewise octomap. viz_only = "viz says yellow but
      octomap knows something there" (the bug we're hunting).

  Test 2 — per-fine-cell scan pressure:
    - For every fine cell that ever entered the octomap known set
      during the scan, record first_known_ts and updates_count (how
      many /octomap_binary messages touched it).
    - For every viz-yellow cube dumped at end, iterate ALL fine cells
      the cube covers (depends on its size) and report:
        * fine_cells_ever_known (how many were in known set at any
          point during scan)
        * fine_cells_known_at_end
        * update_total (sum over covered fine cells)
    - A cube with high update_total but 0 fine_cells_ever_known means
      raycasts passed near it MANY times but octomap could not resolve
      it — either occlusion or occupancy prob stuck sub-threshold.

Output: /tmp/viz_scan_diag.json (schema self-documented inside).
"""
import collections
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from octomap_msgs.msg import Octomap
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray


TREE_DEPTH = 16


def parse_binary_octomap(data_bytes, res):
    """Return list of (cx, cy, cz, size, state) tuples for every leaf.

    state = 'U' (unknown / no subtree, 0b00),
            'F' (free leaf,               0b01),
            'O' (occupied leaf,           0b10).
    Inner nodes (0b11) are recursed, not emitted.

    UNKNOWN leaves come out at whatever depth octomap pruned them to:
    a big unobserved region is one big U cube (shallow depth); a small
    pocket is a small U cube (deeper).
    """
    leaves = []
    idx = [0]
    root_size = res * (2 ** TREE_DEPTH)

    def _recurse(cx, cy, cz, size):
        if idx[0] + 2 > len(data_bytes):
            return
        b1 = data_bytes[idx[0]]
        b2 = data_bytes[idx[0] + 1]
        idx[0] += 2
        bits = b1 | (b2 << 8)
        half = size * 0.5
        for k in range(8):
            state = (bits >> (2 * k)) & 0b11
            dx = half * (0.5 if (k & 1) else -0.5)
            dy = half * (0.5 if (k & 2) else -0.5)
            dz = half * (0.5 if (k & 4) else -0.5)
            ccx = cx + dx
            ccy = cy + dy
            ccz = cz + dz
            child_size = half
            if state == 0:
                leaves.append((ccx, ccy, ccz, child_size, 'U'))
            elif state == 1:
                leaves.append((ccx, ccy, ccz, child_size, 'F'))
            elif state == 2:
                leaves.append((ccx, ccy, ccz, child_size, 'O'))
            else:  # 3 = INNER
                _recurse(ccx, ccy, ccz, child_size)

    _recurse(0.0, 0.0, 0.0, root_size)
    return leaves


def leaf_fine_cells(cx, cy, cz, size, res):
    """Yield (i, j, k) max-depth cell indices covered by a leaf of the
    given centre+size. Uses lower-corner floor to avoid banker's-rounding
    collisions between adjacent 0.005-boundary voxel centres."""
    inv_res = 1.0 / res
    n = max(1, int(round(size * inv_res)))
    base_x = int(math.floor((cx - size * 0.5) * inv_res + 0.5))
    base_y = int(math.floor((cy - size * 0.5) * inv_res + 0.5))
    base_z = int(math.floor((cz - size * 0.5) * inv_res + 0.5))
    for di in range(n):
        for dj in range(n):
            for dk in range(n):
                yield (base_x + di, base_y + dj, base_z + dk)


def build_known_set(leaves, res):
    """Set of max-depth (i,j,k) cells covered by any FREE or OCCUPIED
    leaf. UNKNOWN leaves are excluded — those are the ones viz shows."""
    known = set()
    for cx, cy, cz, size, state in leaves:
        if state == 'U':
            continue
        for cell in leaf_fine_cells(cx, cy, cz, size, res):
            known.add(cell)
    return known


class VizScanDiag(Node):
    def __init__(self):
        super().__init__("viz_scan_diag")
        self.declare_parameter("output_dir", "/tmp")
        self.declare_parameter("workspace_r_min", 0.15)
        self.declare_parameter("workspace_r_max", 2.125)
        self.declare_parameter("workspace_z_min", 0.02)
        self.declare_parameter("workspace_z_max", 2.00)
        # Should match frontier_viz's max_display_size so the "what viz
        # should emit" comparison uses the same filter.
        self.declare_parameter("max_display_size", 0.5)

        self._out_dir = self.get_parameter("output_dir").value
        self._r_min = float(self.get_parameter("workspace_r_min").value)
        self._r_max = float(self.get_parameter("workspace_r_max").value)
        self._z_min = float(self.get_parameter("workspace_z_min").value)
        self._z_max = float(self.get_parameter("workspace_z_max").value)
        self._max_size = float(self.get_parameter("max_display_size").value)

        # State
        # Latest viz cubes: list of (cx, cy, cz, size)
        self._latest_viz_cubes = []
        # Latest octomap parse
        self._latest_leaves = []
        self._latest_res = None
        # Per-fine-cell tracking (accumulates across ALL scans in this run)
        self._per_cell_first_known_ts = {}   # (i,j,k) → sec since arm
        self._per_cell_updates = collections.defaultdict(int)
        self._prev_known = set()
        self._octomap_msg_count = 0
        self._marker_msg_count = 0
        self._scan_start_ts = time.time()
        # Cache last dump payload so we can cross-compare snapshots.
        self._last_snapshot = None
        self._snapshot_index = 0

        self.create_subscription(
            MarkerArray, "/frontier_markers", self._marker_cb, 10)
        self.create_subscription(
            Octomap, "/octomap_binary", self._octomap_cb, 10)
        # Manual trigger: publish a String (e.g. "1") to dump into
        # /tmp/viz_scan_diag_<data>.json. Best-effort so a slow wrapper
        # publish still lands even without a matched-durability sub.
        self.create_subscription(
            String, "/viz_scan_diag_trigger",
            self._trigger_cb, 10)

        self.get_logger().info(
            f"viz_scan_diag armed. publish std_msgs/String to "
            f"/viz_scan_diag_trigger with data=<label> to dump "
            f"{self._out_dir}/viz_scan_diag_<label>.json")

    # ------------------------ callbacks ---------------------------------
    def _marker_cb(self, msg: MarkerArray):
        """Accumulate cubes across ALL 'unknown' ns markers. Each such
        marker has its own scale; a marker with action=DELETE has no
        points and is skipped."""
        cubes = []
        for m in msg.markers:
            if m.ns != "unknown":
                continue
            if m.action != 0:              # 0 = ADD; skip DELETE/DELETEALL
                continue
            size = float(m.scale.x)
            for p in m.points:
                cubes.append((round(p.x, 5),
                              round(p.y, 5),
                              round(p.z, 5),
                              size))
        self._latest_viz_cubes = cubes
        self._marker_msg_count += 1

    def _octomap_cb(self, msg: Octomap):
        if not msg.binary or not msg.data:
            return
        res = float(msg.resolution)
        if res <= 0:
            return
        data = bytes(b & 0xFF for b in msg.data)
        try:
            leaves = parse_binary_octomap(data, res)
        except Exception as exc:              # noqa: BLE001
            self.get_logger().warn(f"octomap parse failed: {exc}")
            return
        known = build_known_set(leaves, res)
        ts = time.time() - self._scan_start_ts
        # Per-fine-cell first_known_ts + update count
        new_known = known - self._prev_known
        for cell in new_known:
            if cell not in self._per_cell_first_known_ts:
                self._per_cell_first_known_ts[cell] = ts
            self._per_cell_updates[cell] += 1
        for cell in known & self._prev_known:
            self._per_cell_updates[cell] += 1
        self._prev_known = known
        self._latest_leaves = leaves
        self._latest_res = res
        self._octomap_msg_count += 1

    def _trigger_cb(self, msg: String):
        label = (msg.data or "").strip() or str(self._snapshot_index + 1)
        self._snapshot_index += 1
        out_path = f"{self._out_dir}/viz_scan_diag_{label}.json"
        # Give the last octomap+viz pair 1s to settle (viz processes
        # octomap asynchronously so a "right now" dump can miss the tail).
        self.get_logger().info(
            f"trigger label={label} received; dumping in 1.5s -> {out_path}")

        def _delayed():
            timer.cancel()
            self._dump(out_path)

        timer = self.create_timer(1.5, _delayed)

    # ------------------------ dump --------------------------------------
    def _dump(self, out_path):
        if self._latest_res is None:
            self.get_logger().error(f"no octomap seen yet; skipping {out_path}")
            self._write(out_path, {"error": "no octomap seen"})
            return

        res = self._latest_res
        known = self._prev_known               # fine cells known at scan end

        # ---------- What OCTOMAP considers UNKNOWN in workspace ----------
        # Apply exactly the same filter frontier_viz applies:
        #   state == 'U' and size <= max_display_size and centre in shell.
        octomap_unknown_cubes = []
        for (cx, cy, cz, size, state) in self._latest_leaves:
            if state != 'U':
                continue
            if size > self._max_size:
                continue
            if not self._in_workspace(cx, cy, cz):
                continue
            octomap_unknown_cubes.append((round(cx, 5),
                                          round(cy, 5),
                                          round(cz, 5),
                                          size))

        # ---------- Fine-cell coverage sets ----------
        viz_coverage = set()
        for (cx, cy, cz, size) in self._latest_viz_cubes:
            for cell in leaf_fine_cells(cx, cy, cz, size, res):
                viz_coverage.add(cell)
        octomap_unknown_coverage = set()
        for (cx, cy, cz, size) in octomap_unknown_cubes:
            for cell in leaf_fine_cells(cx, cy, cz, size, res):
                octomap_unknown_coverage.add(cell)

        # Cells viz calls unknown that octomap actually knows.
        viz_false_positive_cells = viz_coverage & known
        # Cells octomap does not know but viz missed.
        viz_missed_cells = octomap_unknown_coverage - viz_coverage
        # Cells both agree are unknown.
        agreement_cells = viz_coverage & octomap_unknown_coverage

        # ---------- Cube-level set diff (position + size identity) ----------
        viz_set = set(self._latest_viz_cubes)
        oct_set = set(octomap_unknown_cubes)
        viz_only_cubes = list(viz_set - oct_set)
        oct_only_cubes = list(oct_set - viz_set)
        common_cubes = list(viz_set & oct_set)

        # ---------- Per-cube scan-pressure stats (Test 2) ----------
        per_cube_stats = []
        for (cx, cy, cz, size) in self._latest_viz_cubes:
            fine_cells = list(leaf_fine_cells(cx, cy, cz, size, res))
            n = len(fine_cells)
            n_known_end = sum(1 for c in fine_cells if c in known)
            n_ever_known = sum(1 for c in fine_cells
                               if c in self._per_cell_first_known_ts)
            update_total = sum(self._per_cell_updates.get(c, 0)
                               for c in fine_cells)
            first_known_min = None
            first_known_max = None
            for c in fine_cells:
                t = self._per_cell_first_known_ts.get(c)
                if t is None:
                    continue
                if first_known_min is None or t < first_known_min:
                    first_known_min = t
                if first_known_max is None or t > first_known_max:
                    first_known_max = t
            per_cube_stats.append({
                "pos": (cx, cy, cz),
                "size": size,
                "fine_cells_total": n,
                "fine_cells_known_at_end": n_known_end,
                "fine_cells_ever_known": n_ever_known,
                "first_known_min_sec": first_known_min,
                "first_known_max_sec": first_known_max,
                "update_total": update_total,
            })

        # Sort worst-first for readability
        per_cube_stats.sort(
            key=lambda s: (-s["update_total"], -s["fine_cells_ever_known"]))

        # ---------- Global summary ----------
        payload = {
            "octomap_res": res,
            "max_display_size": self._max_size,
            "octomap_msg_count_so_far": self._octomap_msg_count,
            "marker_msg_count_so_far": self._marker_msg_count,
            "wall_time_since_diag_start_sec": time.time() - self._scan_start_ts,
            "snapshot_index": self._snapshot_index,
            "cubes": {
                "viz_yellow_count": len(self._latest_viz_cubes),
                "octomap_unknown_count": len(octomap_unknown_cubes),
                "cubes_agreement_exact_posSize": len(common_cubes),
                "cubes_viz_only": len(viz_only_cubes),
                "cubes_octomap_only": len(oct_only_cubes),
            },
            "fine_cells": {
                "viz_coverage": len(viz_coverage),
                "octomap_unknown_coverage": len(octomap_unknown_coverage),
                "agreement_unknown": len(agreement_cells),
                "viz_says_unknown_but_octomap_knows": len(viz_false_positive_cells),
                "octomap_unknown_but_viz_missed": len(viz_missed_cells),
                "octomap_known_at_now": len(known),
            },
            "scan_pressure_summary": self._pressure_summary(per_cube_stats),
            "viz_yellow_cubes": sorted(list(self._latest_viz_cubes)),
            "octomap_unknown_cubes": sorted(list(octomap_unknown_cubes)),
            "viz_only_cubes_sample": viz_only_cubes[:200],
            "oct_only_cubes_sample": oct_only_cubes[:200],
            "per_cube_stats_top50_by_pressure": per_cube_stats[:50],
            "per_cube_stats_count": len(per_cube_stats),
        }

        # ---------- Cross-snapshot delta vs previous dump ----------
        if self._last_snapshot is not None:
            prev_viz = set(tuple(x) for x in self._last_snapshot["viz_yellow_cubes"])
            prev_oct = set(tuple(x) for x in self._last_snapshot["octomap_unknown_cubes"])
            cur_viz = set(tuple(x) for x in payload["viz_yellow_cubes"])
            cur_oct = set(tuple(x) for x in payload["octomap_unknown_cubes"])
            payload["delta_vs_prev_snapshot"] = {
                "prev_snapshot_index": self._last_snapshot["snapshot_index"],
                # viz: what stayed yellow, what resolved, what became yellow
                "viz_stayed_yellow": len(prev_viz & cur_viz),
                "viz_resolved": len(prev_viz - cur_viz),
                "viz_newly_yellow": len(cur_viz - prev_viz),
                # octomap: what stayed unknown, what got scanned, new unknown appeared
                "oct_stayed_unknown": len(prev_oct & cur_oct),
                "oct_resolved": len(prev_oct - cur_oct),
                "oct_newly_unknown": len(cur_oct - prev_oct),
                # Persistently-yellow-but-octomap-known-now cubes: viz's
                # residual false positives after another scan pass.
                "viz_stayed_but_oct_moved_sample":
                    list((prev_viz & cur_viz) - cur_oct)[:100],
            }

        self._last_snapshot = payload
        self._write(out_path, payload)
        self.get_logger().info(
            f"dumped {out_path} — "
            f"viz_cubes={payload['cubes']['viz_yellow_count']} "
            f"oct_cubes={payload['cubes']['octomap_unknown_count']} "
            f"fine_agree={payload['fine_cells']['agreement_unknown']} "
            f"fine_viz_only={payload['fine_cells']['viz_says_unknown_but_octomap_knows']}")

    def _pressure_summary(self, stats):
        if not stats:
            return {}
        n = len(stats)
        updates = [s["update_total"] for s in stats]
        ever = [s["fine_cells_ever_known"] for s in stats]
        ever_ratio = [(s["fine_cells_ever_known"] / max(1, s["fine_cells_total"]))
                      for s in stats]
        never = sum(1 for e in ever if e == 0)
        return {
            "cubes_total": n,
            "cubes_with_zero_fine_ever_known": never,
            "update_total_min": min(updates),
            "update_total_p25": _pctile(sorted(updates), 25),
            "update_total_p50": _pctile(sorted(updates), 50),
            "update_total_p75": _pctile(sorted(updates), 75),
            "update_total_max": max(updates),
            "ever_known_ratio_p50": _pctile(sorted(ever_ratio), 50),
            "ever_known_ratio_p90": _pctile(sorted(ever_ratio), 90),
        }

    def _in_workspace(self, x, y, z):
        r = math.sqrt(x * x + y * y + z * z)
        if r < self._r_min or r > self._r_max:
            return False
        if z < self._z_min or z > self._z_max:
            return False
        return True

    def _write(self, out_path, obj):
        tmp = out_path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(obj, fh, indent=2, default=str)
        os.replace(tmp, out_path)


def _pctile(sorted_vals, pct):
    if not sorted_vals:
        return None
    i = int(round((pct / 100.0) * (len(sorted_vals) - 1)))
    return sorted_vals[max(0, min(len(sorted_vals) - 1, i))]


def main(args=None):
    rclpy.init(args=args)
    node = VizScanDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                        # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
