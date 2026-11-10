#!/usr/bin/env python3
"""Close-cube exploration coverage monitor.

Subscribes to /octomap_binary and reports:
  - Total cube AABB coverage (known / total volume) — analogous to
    shelf_coverage_monitor.
  - Per-face zone coverage for the 6 faces (2 cm thick slabs on each
    side of the cube). Highlights how well the BACK face (the whole
    point of this test) has been observed.

Cube geometry (from exploration_scene_close_cube.sdf):
    center pose (0.40, 0.00, 0.10), size 0.20 x 0.20 x 0.20
    AABB x=[0.30, 0.50]  y=[-0.10, 0.10]  z=[0.00, 0.20]
    Front face -X at x=0.30.  Back face +X at x=0.50.
    Bottom face -Z at z=0.00 (touching ground plane).
"""
import time

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


MAX_DEPTH = 16

# Cube AABB (world frame)
CUBE_MIN = [0.30, -0.10, 0.00]
CUBE_MAX = [0.50,  0.10, 0.20]

# Face-slab thickness (2 cm) — 2 layers of 1cm voxels around each face.
FACE_T = 0.02

# Slabs: name -> (xmin, ymin, zmin, xmax, ymax, zmax)
FACE_SLABS = {
    "front": (CUBE_MIN[0],       CUBE_MIN[1], CUBE_MIN[2],
              CUBE_MIN[0]+FACE_T, CUBE_MAX[1], CUBE_MAX[2]),
    "back":  (CUBE_MAX[0]-FACE_T, CUBE_MIN[1], CUBE_MIN[2],
              CUBE_MAX[0],        CUBE_MAX[1], CUBE_MAX[2]),
    "left":  (CUBE_MIN[0], CUBE_MIN[1],       CUBE_MIN[2],
              CUBE_MAX[0], CUBE_MIN[1]+FACE_T, CUBE_MAX[2]),
    "right": (CUBE_MIN[0], CUBE_MAX[1]-FACE_T, CUBE_MIN[2],
              CUBE_MAX[0], CUBE_MAX[1],        CUBE_MAX[2]),
    "top":   (CUBE_MIN[0], CUBE_MIN[1], CUBE_MAX[2]-FACE_T,
              CUBE_MAX[0], CUBE_MAX[1], CUBE_MAX[2]),
    "bottom":(CUBE_MIN[0], CUBE_MIN[1], CUBE_MIN[2],
              CUBE_MAX[0], CUBE_MAX[1], CUBE_MIN[2]+FACE_T),
}


def bbox_volume(bmin, bmax):
    return ((bmax[0]-bmin[0]) *
            (bmax[1]-bmin[1]) *
            (bmax[2]-bmin[2]))


class CloseCubeCoverageMonitor(Node):
    def __init__(self):
        super().__init__("close_cube_coverage_monitor")

        self.declare_parameter("duration_sec", 180.0)
        self.declare_parameter("sample_period_sec", 15.0)
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("exit_when_done", True)

        self._duration = float(self.get_parameter("duration_sec").value)
        self._sample_period = float(self.get_parameter("sample_period_sec").value)
        self._exit_when_done = bool(self.get_parameter("exit_when_done").value)

        self._cube_vol = bbox_volume(CUBE_MIN, CUBE_MAX)
        # Face slab volumes
        self._face_vols = {}
        for name, s in FACE_SLABS.items():
            self._face_vols[name] = ((s[3]-s[0]) * (s[4]-s[1]) * (s[5]-s[2]))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        topic = self.get_parameter("octomap_topic").value
        self.create_subscription(Octomap, topic, self._on_octomap, qos)

        self._start_wall = time.monotonic()
        self._last_sample_wall = self._start_wall
        self._latest = None  # (cube_occ, cube_free, face_dict, res)
        self._msg_count = 0
        self._tick_timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"close_cube_coverage_monitor started: "
            f"cube AABB x=[{CUBE_MIN[0]:.2f},{CUBE_MAX[0]:.2f}] "
            f"y=[{CUBE_MIN[1]:.2f},{CUBE_MAX[1]:.2f}] "
            f"z=[{CUBE_MIN[2]:.2f},{CUBE_MAX[2]:.2f}] "
            f"vol={self._cube_vol:.4f} m^3, "
            f"duration={self._duration:.1f}s"
        )
        self.get_logger().info(
            "Note: cube is SOLID; interior always unknown; max coverage "
            "is essentially the outer surface shell. Back-face zone is "
            "the target this scene was designed to test."
        )

    def _on_octomap(self, msg):
        if not msg.binary:
            return
        res = float(msg.resolution)
        if res <= 0:
            return
        data = bytes(b & 0xFF for b in msg.data)
        if not data:
            self._latest = (0.0, 0.0,
                            {n: (0.0, 0.0) for n in FACE_SLABS}, res)
            self._msg_count += 1
            return
        cube_occ, cube_free, face_vols = self._integrate(data, res)
        self._latest = (cube_occ, cube_free, face_vols, res)
        self._msg_count += 1

    def _integrate(self, data, res):
        cube_occ = 0.0
        cube_free = 0.0
        face_vols = {n: [0.0, 0.0] for n in FACE_SLABS}
        root_size = res * (2 ** MAX_DEPTH)
        idx = [0]
        data_len = len(data)

        def overlap_vol(cx, cy, cz, size, bmin, bmax):
            half = size * 0.5
            ox = min(cx+half, bmax[0]) - max(cx-half, bmin[0])
            if ox <= 0.0: return 0.0
            oy = min(cy+half, bmax[1]) - max(cy-half, bmin[1])
            if oy <= 0.0: return 0.0
            oz = min(cz+half, bmax[2]) - max(cz-half, bmin[2])
            if oz <= 0.0: return 0.0
            return ox * oy * oz

        def leaf_fully_outside(cx, cy, cz, size):
            half = size * 0.5
            return (cx+half <= CUBE_MIN[0] or cx-half >= CUBE_MAX[0] or
                    cy+half <= CUBE_MIN[1] or cy-half >= CUBE_MAX[1] or
                    cz+half <= CUBE_MIN[2] or cz-half >= CUBE_MAX[2])

        def recurse(cx, cy, cz, size):
            nonlocal cube_occ, cube_free
            if idx[0] + 2 > data_len:
                return
            b1 = data[idx[0]]
            b2 = data[idx[0]+1]
            idx[0] += 2
            bits = b1 | (b2 << 8)
            child_size = size * 0.5
            offset = child_size * 0.5
            for octant in range(8):
                pair = (bits >> (octant*2)) & 0b11
                if pair == 0b00: continue
                ox = offset if (octant & 1) else -offset
                oy = offset if (octant & 2) else -offset
                oz = offset if (octant & 4) else -offset
                ccx = cx+ox; ccy = cy+oy; ccz = cz+oz
                if pair == 0b11:
                    recurse(ccx, ccy, ccz, child_size)
                else:
                    if leaf_fully_outside(ccx, ccy, ccz, child_size):
                        continue
                    is_occ = (pair == 0b10)
                    # Cube AABB overlap
                    v = overlap_vol(ccx, ccy, ccz, child_size,
                                    CUBE_MIN, CUBE_MAX)
                    if v > 0.0:
                        if is_occ: cube_occ += v
                        else: cube_free += v
                    # Face slab overlaps
                    for name, s in FACE_SLABS.items():
                        vf = overlap_vol(ccx, ccy, ccz, child_size,
                                         s[:3], s[3:])
                        if vf > 0.0:
                            if is_occ: face_vols[name][0] += vf
                            else: face_vols[name][1] += vf

        recurse(0.0, 0.0, 0.0, root_size)
        return cube_occ, cube_free, {n: tuple(v) for n, v in face_vols.items()}

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self._start_wall
        if now - self._last_sample_wall >= self._sample_period:
            self._last_sample_wall = now
            self._log_sample(elapsed, "sample")
        if elapsed >= self._duration:
            self._log_sample(elapsed, "FINAL")
            self.get_logger().info(
                f"Duration {self._duration:.1f}s; {self._msg_count} msgs."
            )
            if self._exit_when_done:
                self._tick_timer.cancel()
                rclpy.shutdown()

    def _log_sample(self, elapsed, label):
        if self._latest is None:
            self.get_logger().info(
                f"[{label} t={elapsed:6.1f}s] no octomap yet"
            )
            return
        cube_occ, cube_free, face_vols, res = self._latest
        known = min(cube_occ + cube_free, self._cube_vol)
        unk = max(0.0, self._cube_vol - known)
        cov = 100.0 * known / self._cube_vol if self._cube_vol > 0 else 0.0
        occ_pct = 100.0 * cube_occ / self._cube_vol
        free_pct = 100.0 * cube_free / self._cube_vol
        ku = (known/unk) if unk > 1e-9 else float("inf")
        # Per-face
        face_parts = []
        for name in ["front", "back", "left", "right", "top", "bottom"]:
            occ, free = face_vols[name]
            fv = self._face_vols[name]
            k = min(occ + free, fv)
            fc = 100.0 * k / fv if fv > 0 else 0.0
            fo = 100.0 * occ / fv if fv > 0 else 0.0
            face_parts.append(f"{name}={fc:5.1f}%(occ{fo:4.1f})")
        self.get_logger().info(
            f"[{label} t={elapsed:6.1f}s] res={res:.3f} msgs={self._msg_count} | "
            f"CUBE={cov:5.2f}% (occ={occ_pct:5.2f}%, free={free_pct:5.2f}%) "
            f"k/u={ku:.3f} | faces: " + " ".join(face_parts)
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloseCubeCoverageMonitor()
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
