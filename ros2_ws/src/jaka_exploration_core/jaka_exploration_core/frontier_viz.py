#!/usr/bin/env python3
"""Real-time UNKNOWN voxel visualization from OctoMap.

Subscribes to /octomap_binary, walks the pruned octree, and publishes
every UNKNOWN subtree (child bit-pair 0b00) as a semi-transparent yellow
CUBE_LIST on /frontier_markers. NO aggregation, NO workspace sampling —
the display matches octomap's native storage: large unobserved regions
get one big cube (a shallow-depth UNKNOWN node); small unobserved
pockets get small cubes (a deep UNKNOWN node).

Because CUBE_LIST forces one scale per marker, unknowns are bucketed by
their native size and one Marker is emitted per size bucket.

Filters applied to the raw UNKNOWN leaves:
  1. size ≤ max_display_size (default 0.5m) — skip the root-level
     UNKNOWN cubes covering 100+m of space the arm cannot reach.
  2. cube centre inside the workspace shell (r_min..r_max, z_min..z_max)
     — same as the arm's reachable region.

OctoMap binary format decoded in place:
  Each internal node = 2 bytes = 16 bits = 8 octants x 2 bits each.
  Per octant:
      0b00 = child does not exist  (unknown region — EMITTED here)
      0b01 = child is a free leaf
      0b10 = child is an occupied leaf
      0b11 = child has children    (recurse)
  Octant index bits: bit 0 = x, bit 1 = y, bit 2 = z
      (0 = negative half of the parent, 1 = positive half)
  Byte 1 covers octants 0-3, byte 2 covers octants 4-7 (LSB first).
"""
import math

import rclpy
from geometry_msgs.msg import Point
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


# OctoMap default tree depth. Root spans (2**MAX_DEPTH * resolution).
MAX_DEPTH = 16


class FrontierViz(Node):
    def __init__(self):
        super().__init__("frontier_viz")
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("marker_topic", "/frontier_markers")
        self.declare_parameter("frame_id", "world")
        # ★ 不再聚合。octomap 用自适应分辨率(pruned tree)存 unknown:
        #   大片纯 unknown 用一个大 subtree 表示,小 pocket 用小的。
        #   viz 直接照 octomap 存的样子画,每个 unknown subtree 一个 cube,
        #   size = subtree 边长。多种 size 分组到多个 Marker 里(CUBE_LIST
        #   要求同一 marker 里所有 cube 同 size)。
        #   marker_scale 现在只是"最大允许显示尺寸"上限,避免画超出
        #   workspace 的巨大 root-level unknowns。
        self.declare_parameter("max_display_size", 0.5)
        self.declare_parameter("marker_lifetime_sec", 0.0)
        # Workspace filter: only visualise frontiers within this sphere
        # around workspace_center. Mirrors the teammate's isOutOfWorkspace()
        # check but applied to visualisation so we do not drown RViz in
        # 50k+ boundary voxels from a 3 m-radius observed volume that we
        # cannot actually reach with a 0.86 m-reach arm.
        self.declare_parameter("workspace_center", [0.0, 0.0, 0.0])
        self.declare_parameter("workspace_r_min", 0.15)
        # Aligned with HPPT's isTooFar threshold =
        # frontier_max_range_factor(2.5) * reach_max(0.85) = 2.125 m.
        # Anything HPPT is willing to chase, viz shows.
        self.declare_parameter("workspace_r_max", 2.125)
        self.declare_parameter("workspace_z_min", 0.02)
        self.declare_parameter("workspace_z_max", 2.00)

        self._frame = self.get_parameter("frame_id").value
        self._max_size = float(self.get_parameter("max_display_size").value)
        self._lifetime = float(self.get_parameter("marker_lifetime_sec").value)
        self._ws_center = list(self.get_parameter("workspace_center").value)
        self._ws_r_min = float(self.get_parameter("workspace_r_min").value)
        self._ws_r_max = float(self.get_parameter("workspace_r_max").value)
        self._ws_z_min = float(self.get_parameter("workspace_z_min").value)
        self._ws_z_max = float(self.get_parameter("workspace_z_max").value)

        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10
        )
        self.create_subscription(
            Octomap, self.get_parameter("octomap_topic").value, self._cb, 10
        )

        self._prev_frontier_count = 0
        self._msg_count = 0
        self.get_logger().info(
            f"frontier_viz ready: "
            f"{self.get_parameter('octomap_topic').value} -> "
            f"{self.get_parameter('marker_topic').value}"
        )

    def _cb(self, msg):
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
            self._publish_unknown([])
            return

        # Parse tree; get ALL leaves including UNKNOWN ones (parser emits
        # (cx, cy, cz, size, state) with state ∈ {'U','F','O'}).
        leaves = self._parse_binary(data, res)

        # Filter: keep UNKNOWN leaves whose bounding cube is inside the
        # workspace shell AND whose size ≤ max_display_size (root-level
        # unknowns can be 100+m across; we skip those since they cover
        # regions the arm could never reach).
        unknowns = [
            (cx, cy, cz, size)
            for (cx, cy, cz, size, state) in leaves
            if state == 'U'
            and size <= self._max_size
            and self._in_workspace((cx, cy, cz))
        ]
        self._publish_unknown(unknowns)

        self._msg_count += 1
        if self._msg_count == 1 or self._msg_count % 30 == 0:
            n_free = sum(1 for L in leaves if L[4] == 'F')
            n_occ = sum(1 for L in leaves if L[4] == 'O')
            n_unk_all = sum(1 for L in leaves if L[4] == 'U')
            self.get_logger().info(
                f"leaves={len(leaves)} (free={n_free}, occ={n_occ}, "
                f"unknown={n_unk_all}), unknown_ws_kept={len(unknowns)}"
            )

    def _in_workspace(self, p):
        x, y, z = p
        dx = x - self._ws_center[0]
        dy = y - self._ws_center[1]
        dz = z - self._ws_center[2]
        r = math.sqrt(dx * dx + dy * dy + dz * dz)
        return (
            self._ws_r_min <= r <= self._ws_r_max
            and self._ws_z_min <= z <= self._ws_z_max
        )

    # -------- OctoMap binary parser --------
    def _parse_binary(self, data, res):
        """Return list of (cx, cy, cz, size, state) for every leaf, where
        state ∈ {'U','F','O'} for UNKNOWN / FREE / OCCUPIED.

        UNKNOWN leaves come out at *whatever depth octomap pruned them to*:
        a big region entirely unobserved is one big UNKNOWN cube at a
        shallow depth; a small pocket is a small cube deeper down. This is
        exactly how octomap natively stores unknowns (as absence of a
        subtree = a 0b00 child at some inner node).
        """
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
                ox = offset if (octant & 1) else -offset
                oy = offset if (octant & 2) else -offset
                oz = offset if (octant & 4) else -offset
                ccx = cx + ox
                ccy = cy + oy
                ccz = cz + oz
                if pair == 0b00:
                    leaves.append((ccx, ccy, ccz, child_size, 'U'))
                elif pair == 0b01:
                    leaves.append((ccx, ccy, ccz, child_size, 'F'))
                elif pair == 0b10:
                    leaves.append((ccx, ccy, ccz, child_size, 'O'))
                else:                                     # 0b11 - recurse
                    recurse(ccx, ccy, ccz, child_size)

        # Root center at (0, 0, 0) per OctoMap convention.
        recurse(0.0, 0.0, 0.0, root_size)
        return leaves

    # -------- Publish --------
    def _publish_unknown(self, unknowns):
        """`unknowns` is a list of (cx, cy, cz, size). Group by size and
        emit one CUBE_LIST marker per unique size (CUBE_LIST uses a single
        scale for every cube in it, so we need one marker per size bucket).
        """
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Bucket unknowns by their native size.
        by_size = {}
        for (cx, cy, cz, size) in unknowns:
            by_size.setdefault(size, []).append((cx, cy, cz))

        # Stable id per size (id = size in mm) so RViz never sees id=k
        # switch from scale=0.16 to scale=0.32 across publishes. Without
        # this, sequential ids would shuffle when a new size bucket
        # appears/disappears and cubes would visibly flicker.
        used_ids = set()
        for size in sorted(by_size.keys()):
            positions = by_size[size]
            marker_id = int(round(size * 1000.0))
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = stamp
            m.ns = "unknown"
            m.id = marker_id
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = size
            m.scale.y = size
            m.scale.z = size
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.5     # 半透明黄
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            for x, y, z in positions:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = float(z)
                m.points.append(p)
            arr.markers.append(m)
            used_ids.add(marker_id)

        # DELETE any marker ids that were used last publish but aren't
        # this time (e.g. a size bucket disappeared as scan progressed).
        # Otherwise stale cubes at those sizes linger forever.
        for old_id in getattr(self, "_prev_ids", set()):
            if old_id in used_ids:
                continue
            m_del = Marker()
            m_del.header.frame_id = self._frame
            m_del.header.stamp = stamp
            m_del.ns = "unknown"
            m_del.id = old_id
            m_del.action = Marker.DELETE
            arr.markers.append(m_del)
        self._prev_ids = used_ids

        # Also DELETE any leftover markers from the old namespaces used
        # by earlier builds. Costs nothing if they never existed.
        for old_ns, old_id in (("frontier", 0), ("free", 1)):
            m_del = Marker()
            m_del.header.frame_id = self._frame
            m_del.header.stamp = stamp
            m_del.ns = old_ns
            m_del.id = old_id
            m_del.action = Marker.DELETE
            arr.markers.append(m_del)

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
