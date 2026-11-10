#!/usr/bin/env python3
"""Multi-object exploration coverage monitor.

Subscribes to /octomap_binary and reports known/unknown volume within
the AABBs of the 3 exploration_scene_multi objects (box + sphere + cyl).

Same volume-integration logic as shelf_coverage_monitor; per-object plus
combined coverage numbers.

Object AABBs (world frame, from exploration_scene_multi.sdf):
  box_target  (brown box):   center (0.85, 0.00, 0.20)  size 0.40x0.30x0.40
                             AABB x[0.65,1.05] y[-0.15,0.15] z[0.00,0.40]
  sphere_left (green sphere):center (0.50, 0.60, 0.15)  radius 0.10
                             AABB x[0.40,0.60] y[0.50,0.70] z[0.05,0.25]
  cyl_right   (red cylinder):center (0.50,-0.60, 0.15)  radius 0.08 height 0.30
                             AABB x[0.42,0.58] y[-0.68,-0.52] z[0.00,0.30]

Note: box_target and cyl_right are SOLID; interior always unknown; the
metric essentially measures how well the surface shell + immediate
surroundings are observed. sphere_left same. Interpret trends across
duration, not absolute values.
"""
import math
import time

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


MAX_DEPTH = 16


# Object AABBs (world frame)
OBJECTS = [
    ("box",    [0.65, -0.15, 0.00], [1.05, 0.15, 0.40]),
    ("sphere", [0.40,  0.50, 0.05], [0.60, 0.70, 0.25]),
    ("cyl",    [0.42, -0.68, 0.00], [0.58,-0.52, 0.30]),
]


class MultiCoverageMonitor(Node):
    def __init__(self):
        super().__init__("multi_coverage_monitor")

        self.declare_parameter("duration_sec", 180.0)
        self.declare_parameter("sample_period_sec", 15.0)
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("exit_when_done", True)

        self._duration = float(self.get_parameter("duration_sec").value)
        self._sample_period = float(self.get_parameter("sample_period_sec").value)
        self._exit_when_done = bool(self.get_parameter("exit_when_done").value)

        self._boxes = []
        for name, bmin, bmax in OBJECTS:
            v = (bmax[0]-bmin[0]) * (bmax[1]-bmin[1]) * (bmax[2]-bmin[2])
            self._boxes.append({"name": name, "min": bmin, "max": bmax, "vol": v})

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        topic = self.get_parameter("octomap_topic").value
        self.create_subscription(Octomap, topic, self._on_octomap, qos)

        self._start_wall = time.monotonic()
        self._last_sample_wall = self._start_wall
        self._latest = None
        self._msg_count = 0
        self._tick_timer = self.create_timer(1.0, self._tick)

        total_vol = sum(b["vol"] for b in self._boxes)
        self.get_logger().info(
            "multi_coverage_monitor started: 3 objects, "
            f"total_vol={total_vol:.4f} m^3, "
            f"duration={self._duration:.1f}s, "
            f"sample_period={self._sample_period:.1f}s, "
            f"topic={topic}"
        )
        for b in self._boxes:
            self.get_logger().info(
                f"  {b['name']:6s} bbox x=[{b['min'][0]:.2f},{b['max'][0]:.2f}] "
                f"y=[{b['min'][1]:.2f},{b['max'][1]:.2f}] "
                f"z=[{b['min'][2]:.2f},{b['max'][2]:.2f}] vol={b['vol']:.4f}"
            )

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
            self._latest = ([(0.0, 0.0) for _ in self._boxes], res)
            self._msg_count += 1
            return
        per_box = self._integrate(data, res)
        self._latest = (per_box, res)
        self._msg_count += 1

    def _integrate(self, data, res):
        """Return list of (occ_vol, free_vol) per object."""
        results = [(0.0, 0.0)] * len(self._boxes)
        results = [[0.0, 0.0] for _ in self._boxes]
        root_size = res * (2 ** MAX_DEPTH)
        idx = [0]
        data_len = len(data)
        boxes = self._boxes

        def overlap_vol(cx, cy, cz, size, bb_min, bb_max):
            half = size * 0.5
            ox = min(cx + half, bb_max[0]) - max(cx - half, bb_min[0])
            if ox <= 0.0: return 0.0
            oy = min(cy + half, bb_max[1]) - max(cy - half, bb_min[1])
            if oy <= 0.0: return 0.0
            oz = min(cz + half, bb_max[2]) - max(cz - half, bb_min[2])
            if oz <= 0.0: return 0.0
            return ox * oy * oz

        def leaf_fully_outside_all(cx, cy, cz, size):
            half = size * 0.5
            for b in boxes:
                bb_min, bb_max = b["min"], b["max"]
                if not (cx + half <= bb_min[0] or cx - half >= bb_max[0] or
                        cy + half <= bb_min[1] or cy - half >= bb_max[1] or
                        cz + half <= bb_min[2] or cz - half >= bb_max[2]):
                    return False
            return True

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
                if pair == 0b00: continue
                ox = offset if (octant & 1) else -offset
                oy = offset if (octant & 2) else -offset
                oz = offset if (octant & 4) else -offset
                ccx = cx + ox; ccy = cy + oy; ccz = cz + oz
                if pair == 0b11:
                    recurse(ccx, ccy, ccz, child_size)
                else:
                    if leaf_fully_outside_all(ccx, ccy, ccz, child_size):
                        continue
                    is_occ = (pair == 0b10)
                    for i, b in enumerate(boxes):
                        v = overlap_vol(ccx, ccy, ccz, child_size,
                                        b["min"], b["max"])
                        if v > 0.0:
                            if is_occ:
                                results[i][0] += v
                            else:
                                results[i][1] += v

        recurse(0.0, 0.0, 0.0, root_size)
        return [tuple(r) for r in results]

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self._start_wall
        if now - self._last_sample_wall >= self._sample_period:
            self._last_sample_wall = now
            self._log_sample(elapsed, "sample")
        if elapsed >= self._duration:
            self._log_sample(elapsed, "FINAL")
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
        per_box, res = self._latest
        total_occ = total_free = total_vol = 0.0
        parts = []
        for i, b in enumerate(self._boxes):
            occ_vol, free_vol = per_box[i]
            known = occ_vol + free_vol
            if known > b["vol"]:
                known = b["vol"]
                free_vol = max(0.0, known - occ_vol)
            unk = max(0.0, b["vol"] - known)
            cov = 100.0 * known / b["vol"] if b["vol"] > 0 else 0.0
            parts.append(f"{b['name']}={cov:5.2f}%")
            total_occ += occ_vol
            total_free += free_vol
            total_vol += b["vol"]
        total_known = total_occ + total_free
        total_unk = max(0.0, total_vol - total_known)
        total_cov = 100.0 * total_known / total_vol if total_vol > 0 else 0.0
        occ_pct = 100.0 * total_occ / total_vol if total_vol > 0 else 0.0
        free_pct = 100.0 * total_free / total_vol if total_vol > 0 else 0.0
        ku = (total_known / total_unk) if total_unk > 1e-9 else float("inf")
        self.get_logger().info(
            f"[{label} t={elapsed:6.1f}s] res={res:.3f} msgs={self._msg_count} | "
            f"TOTAL={total_cov:5.2f}% (occ={occ_pct:5.2f}%, free={free_pct:5.2f}%) "
            f"known/unknown={ku:.3f} | per-obj: {' '.join(parts)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiCoverageMonitor()
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
