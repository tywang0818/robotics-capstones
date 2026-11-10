#!/usr/bin/env python3
"""Self-filter: drop point cloud points that hit the robot's own body
so OctoMap does not falsely map the arm as an obstacle.

For every link in LINK_SPHERES, looks up the link's current pose via TF,
projects the link's bounding sphere(s) into the cloud's own frame, then
drops any incoming point inside any sphere. Remaining points are
republished with the original frame_id and timestamp so octomap_server's
raycasting still sees the correct sensor origin.

LINK_SPHERES are conservative bounding spheres for each JAKA A5 link,
hand-tuned from the URDF mesh dimensions. Inflate radii by 2-3 cm so
grazing rays near the arm are also dropped. If genuine box voxels get
clipped because the arm grazes the obstacle, shrink the relevant sphere.
"""
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


# (cx, cy, cz, radius) in the link's LOCAL frame, in meters.
# Multiple spheres per link approximate non-spherical shapes.
LINK_SPHERES = {
    "base_link": [(0.00, 0.00, 0.06, 0.16)],
    "J1":        [(0.00, 0.00, 0.00, 0.13)],
    "J2": [
        # J2's mesh extends along +x around z=-0.143 in its local frame.
        (0.00, 0.00, -0.143, 0.125),
        (0.12, 0.00, -0.143, 0.125),
        (0.24, 0.00, -0.143, 0.125),
        (0.36, 0.00, -0.143, 0.125),
        (0.48, 0.00, -0.143, 0.125),
    ],
    "J3": [
        # J3's mesh is nearly level along +x in its local frame.
        (0.00, 0.00, -0.007, 0.11),
        (0.13, 0.00, -0.007, 0.11),
        (0.26, 0.00, -0.007, 0.11),
        (0.39, 0.00, -0.007, 0.11),
    ],
    "J4": [(0.00, 0.000, 0.00, 0.100)],
    "J5": [(0.00, -0.004, 0.00, 0.115)],
    "J6": [(0.00, 0.00, -0.02, 0.07)],
    # camera_link itself is intentionally excluded - it's the sensor body.
}


def _transform_to_matrix(t):
    x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return M


class SelfFilter(Node):
    def __init__(self):
        super().__init__("self_filter")
        self.declare_parameter("input_topic", "/camera/points")
        self.declare_parameter("output_topic", "/camera/points_filtered")
        self.declare_parameter("tf_timeout_sec", 0.2)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self._cb, sub_qos
        )

        # ★ PAUSE: 暂停期间丢弃点云 → octomap_server 收不到输入 → octomap 停止更新。
        #   与 hppt 的 /exploration_pause 同一指令 topic:true=暂停,false=继续。
        #   ★ transient_local(latched)订阅:与 server 的 latched publisher 匹配,
        #     无论 discovery 早晚都拿到最新 pause 状态(避免丢 pause=false 卡死)。
        pause_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.paused = False
        self.pause_sub = self.create_subscription(
            Bool, "/exploration_pause", self._pause_cb, pause_qos
        )

        n_spheres = sum(len(v) for v in LINK_SPHERES.values())
        self.get_logger().info(
            f"self_filter: {in_topic} -> {out_topic} "
            f"({n_spheres} spheres over {len(LINK_SPHERES)} links)"
        )
        self._first_log = True
        self._last_warning_at = 0.0

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self._last_warning_at >= 2.0:
            self.get_logger().warning(message)
            self._last_warning_at = now

    def _pause_cb(self, msg):
        # ★ PAUSE: true=暂停(丢弃点云,octomap 停更新),false=继续。
        if msg.data != self.paused:
            self.paused = msg.data
            self.get_logger().warning(
                "self_filter %s: octomap updates %s."
                % ("PAUSED" if msg.data else "RESUMED",
                   "stopped" if msg.data else "resumed")
            )

    def _cb(self, msg):
        # ★ PAUSE: 暂停期间不转发点云 → octomap_server 无输入 → octomap 停止更新。
        if self.paused:
            return
        cloud_frame = msg.header.frame_id

        # 1) Look up TF for each link and project its spheres into the
        # cloud frame.
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            self._warn_throttled(
                "Dropping point cloud because its timestamp is zero."
            )
            return

        timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_sec").value)
        )
        centers, radii_sq = [], []
        missing_links = []
        for link, spheres in LINK_SPHERES.items():
            try:
                tfst = self.tf_buf.lookup_transform(
                    cloud_frame, link, stamp, timeout=timeout
                )
            except Exception as error:
                missing_links.append(f"{link}: {error}")
                continue
            M = _transform_to_matrix(tfst.transform)
            for cx, cy, cz, r in spheres:
                p = M @ np.array([cx, cy, cz, 1.0])
                centers.append(p[:3])
                radii_sq.append(r * r)
        if missing_links:
            self._warn_throttled(
                "Dropping point cloud because robot TF is incomplete: "
                + "; ".join(missing_links)
            )
            return
        if not centers:
            self._warn_throttled(
                "Dropping point cloud because no robot-link TF was available."
            )
            return
        centers = np.asarray(centers, dtype=np.float32)
        radii_sq = np.asarray(radii_sq, dtype=np.float32)

        # 2) Read incoming points (already in cloud_frame).
        try:
            pts_struct = pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        except Exception as error:
            self._warn_throttled(
                f"Dropping point cloud because it could not be decoded: {error}"
            )
            return
        if pts_struct.size == 0:
            self.pub.publish(msg)
            return
        pts = np.stack(
            [pts_struct["x"], pts_struct["y"], pts_struct["z"]], axis=-1
        ).astype(np.float32)

        # 3) Vectorized point-in-sphere check.
        diff = pts[:, None, :] - centers[None, :, :]
        dist_sq = np.sum(diff * diff, axis=2)        # (P, N)
        inside = dist_sq < radii_sq[None, :]          # (P, N)
        drop = np.any(inside, axis=1)                 # (P,)
        kept = pts[~drop]

        # 4) Republish with the SAME header so octomap_server's TF lookup
        # of sensor origin still works.
        out_msg = pc2.create_cloud_xyz32(msg.header, kept)
        self.pub.publish(out_msg)

        if self._first_log:
            self.get_logger().info(
                f"first cloud: {pts.shape[0]} in -> {kept.shape[0]} out "
                f"({pts.shape[0] - kept.shape[0]} self-points dropped)"
            )
            self._first_log = False


def main(args=None):
    rclpy.init(args=args)
    node = SelfFilter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
