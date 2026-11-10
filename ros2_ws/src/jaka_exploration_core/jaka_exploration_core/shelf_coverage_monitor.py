#!/usr/bin/env python3
"""Shelf exploration coverage monitor.

Subscribes to /octomap_binary and measures how much of an axis-aligned
bounding box (default: the RealShelf collision box in world frame) has
been classified as known (free OR occupied) vs unknown, for a fixed
duration. Prints periodic samples and a final summary, then shuts down.

Coverage is volume-based: for each OctoMap leaf we compute the overlap
volume with the target bbox (leaves may be pruned to larger sizes than
the max-depth voxel, so cell-counting would be biased). Unknown volume
is inferred as bbox_volume - known_volume_in_bbox, because the OctoMap
binary format only carries known leaves.

Default bbox is the shelf's world-frame AABB derived from
exploration_scene_shelf.sdf:
  model pose (0.80, 0.0, 0.0), yaw = -pi/2
  local collision box: offset (0, 0.195, 0.555), size (0.84, 0.39, 1.11)
  -> world AABB: x in [0.80, 1.19], y in [-0.42, 0.42], z in [0, 1.11].

Note: the RealShelf collision is a SOLID box (see the SDF comment). A
depth camera can only see the outer surface, so the box interior stays
unknown forever. The maximum achievable known-ratio on the strict shelf
AABB is therefore a thin surface shell (~10-20%). Interpret coverage
trends over the test duration, not absolute values.
"""
import math
import time

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


MAX_DEPTH = 16


class ShelfCoverageMonitor(Node):
    def __init__(self):
        super().__init__("shelf_coverage_monitor")

        # Default bbox: RealShelf world-frame AABB.
        self.declare_parameter("bbox_min", [0.80, -0.42, 0.0])
        self.declare_parameter("bbox_max", [1.19, 0.42, 1.11])
        self.declare_parameter("duration_sec", 180.0)
        self.declare_parameter("sample_period_sec", 10.0)
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("exit_when_done", True)

        bb_min = list(self.get_parameter("bbox_min").value)
        bb_max = list(self.get_parameter("bbox_max").value)
        if len(bb_min) != 3 or len(bb_max) != 3:
            raise ValueError("bbox_min/bbox_max must have length 3")
        for i in range(3):
            if bb_max[i] <= bb_min[i]:
                raise ValueError(
                    f"bbox_max[{i}]={bb_max[i]} must be > bbox_min[{i}]={bb_min[i]}"
                )
        self._bb_min = [float(v) for v in bb_min]
        self._bb_max = [float(v) for v in bb_max]
        self._bb_vol = (
            (self._bb_max[0] - self._bb_min[0])
            * (self._bb_max[1] - self._bb_min[1])
            * (self._bb_max[2] - self._bb_min[2])
        )

        self._duration = float(self.get_parameter("duration_sec").value)
        self._sample_period = float(self.get_parameter("sample_period_sec").value)
        self._exit_when_done = bool(self.get_parameter("exit_when_done").value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        topic = self.get_parameter("octomap_topic").value
        self.create_subscription(Octomap, topic, self._on_octomap, qos)

        self._start_wall = time.monotonic()
        self._last_sample_wall = self._start_wall
        self._latest = None  # (occupied_vol, free_vol, msg_count)
        self._msg_count = 0
        # Ensure we still terminate even if no octomap ever arrives.
        self._tick_timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "shelf_coverage_monitor started: "
            f"bbox x=[{self._bb_min[0]:.3f}, {self._bb_max[0]:.3f}] "
            f"y=[{self._bb_min[1]:.3f}, {self._bb_max[1]:.3f}] "
            f"z=[{self._bb_min[2]:.3f}, {self._bb_max[2]:.3f}] "
            f"vol={self._bb_vol:.4f} m^3, "
            f"duration={self._duration:.1f}s, "
            f"sample_period={self._sample_period:.1f}s, "
            f"topic={topic}"
        )
        self.get_logger().info(
            "Note: RealShelf collision is a solid box; camera cannot see "
            "interior. Absolute coverage is capped by surface shell."
        )

    # ---------------- Octomap callback ----------------
    def _on_octomap(self, msg):
        if not msg.binary:
            self.get_logger().warn(
                "expected binary OctoMap; got full", throttle_duration_sec=5
            )
            return
        res = float(msg.resolution)
        if res <= 0:
            return
        data = bytes(b & 0xFF for b in msg.data)
        if not data:
            self._latest = (0.0, 0.0, res)
            self._msg_count += 1
            return

        occ_vol, free_vol = self._integrate(data, res)
        self._latest = (occ_vol, free_vol, res)
        self._msg_count += 1

    # ---------------- Volume integration ----------------
    def _integrate(self, data, res):
        """Recursively traverse the binary OctoMap tree and accumulate the
        overlap volume between each leaf and the target bbox, split by
        occupied vs free.
        """
        occ_vol = 0.0
        free_vol = 0.0
        root_size = res * (2 ** MAX_DEPTH)
        idx = [0]
        data_len = len(data)
        bb_min = self._bb_min
        bb_max = self._bb_max

        def overlap_vol(cx, cy, cz, size):
            half = size * 0.5
            ox = min(cx + half, bb_max[0]) - max(cx - half, bb_min[0])
            if ox <= 0.0:
                return 0.0
            oy = min(cy + half, bb_max[1]) - max(cy - half, bb_min[1])
            if oy <= 0.0:
                return 0.0
            oz = min(cz + half, bb_max[2]) - max(cz - half, bb_min[2])
            if oz <= 0.0:
                return 0.0
            return ox * oy * oz

        def leaf_fully_outside(cx, cy, cz, size):
            half = size * 0.5
            return (
                cx + half <= bb_min[0]
                or cx - half >= bb_max[0]
                or cy + half <= bb_min[1]
                or cy - half >= bb_max[1]
                or cz + half <= bb_min[2]
                or cz - half >= bb_max[2]
            )

        def recurse(cx, cy, cz, size):
            nonlocal occ_vol, free_vol
            if idx[0] + 2 > data_len:
                return
            # Prune: if the whole subtree cube misses the bbox, we still
            # need to advance the byte index because the tree layout is
            # depth-first, so we cannot skip bytes. Just recurse without
            # accumulating volume.
            b1 = data[idx[0]]
            b2 = data[idx[0] + 1]
            idx[0] += 2
            bits = b1 | (b2 << 8)

            child_size = size * 0.5
            offset = child_size * 0.5

            for octant in range(8):
                pair = (bits >> (octant * 2)) & 0b11
                if pair == 0b00:
                    continue  # unknown
                ox = offset if (octant & 1) else -offset
                oy = offset if (octant & 2) else -offset
                oz = offset if (octant & 4) else -offset
                ccx = cx + ox
                ccy = cy + oy
                ccz = cz + oz
                if pair == 0b11:
                    recurse(ccx, ccy, ccz, child_size)
                else:
                    if leaf_fully_outside(ccx, ccy, ccz, child_size):
                        continue
                    v = overlap_vol(ccx, ccy, ccz, child_size)
                    if v <= 0.0:
                        continue
                    if pair == 0b10:
                        occ_vol += v
                    else:  # 0b01
                        free_vol += v

        recurse(0.0, 0.0, 0.0, root_size)
        return occ_vol, free_vol

    # ---------------- Periodic tick ----------------
    def _tick(self):
        now = time.monotonic()
        elapsed = now - self._start_wall

        if now - self._last_sample_wall >= self._sample_period:
            self._last_sample_wall = now
            self._log_sample(elapsed, label="sample")

        if elapsed >= self._duration:
            self._log_sample(elapsed, label="FINAL")
            self.get_logger().info(
                f"Duration {self._duration:.1f}s reached; "
                f"{self._msg_count} octomap messages processed."
            )
            if self._exit_when_done:
                self._tick_timer.cancel()
                rclpy.shutdown()

    def _log_sample(self, elapsed, label):
        if self._latest is None:
            self.get_logger().info(
                f"[{label} t={elapsed:6.1f}s] no octomap received yet"
            )
            return
        occ_vol, free_vol, res = self._latest
        known_vol = occ_vol + free_vol
        # Clamp against floating point overshoot (leaves at the bbox
        # boundary can produce numerically slightly > bbox volume).
        if known_vol > self._bb_vol:
            known_vol = self._bb_vol
            free_vol = max(0.0, known_vol - occ_vol)
        unknown_vol = max(0.0, self._bb_vol - known_vol)
        coverage = known_vol / self._bb_vol if self._bb_vol > 0 else 0.0
        occ_ratio = occ_vol / self._bb_vol if self._bb_vol > 0 else 0.0
        free_ratio = free_vol / self._bb_vol if self._bb_vol > 0 else 0.0
        # known / unknown, guard against div-by-zero when nothing observed.
        if unknown_vol > 1e-9:
            ku_ratio = known_vol / unknown_vol
            ku_str = f"{ku_ratio:.3f}"
        else:
            ku_str = "inf"
        self.get_logger().info(
            f"[{label} t={elapsed:6.1f}s] "
            f"res={res:.3f} msgs={self._msg_count} | "
            f"coverage={coverage*100:5.2f}%  "
            f"(occ={occ_ratio*100:5.2f}%, free={free_ratio*100:5.2f}%, "
            f"unknown={(1-coverage)*100:5.2f}%)  "
            f"known/unknown={ku_str}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ShelfCoverageMonitor()
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
