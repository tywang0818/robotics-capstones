#!/usr/bin/env python3
"""Bridge: stream the real-time OctoMap from octomap_server into the
motion-planning team's JSON environment topic.

Pipeline:
  octomap_server --> /octomap_binary (octomap_msgs/Octomap)
                |
                | this node
                v
       /team/optimized_environment_json (std_msgs/String, JSON with
                                          base64-encoded OctoMap payload)
                |
                v
  environment_bridge --> motion_planner --> MoveIt PlanningScene

Each publication bumps the environment version. Downstream task requests
must reference the latest version, so the exploration FSM should always
read the most recent /team/environment_status_json after planning.

To avoid racing with bootstrap_scan (which seeds an initial environment
from a file), start this bridge AFTER bootstrap_scan finishes.
"""
import base64
import json
import time

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class OctomapToJsonBridge(Node):
    def __init__(self):
        super().__init__("octomap_to_json_bridge")

        self.declare_parameter("input_topic", "/octomap_binary")
        self.declare_parameter("output_topic", "/team/optimized_environment_json")
        # Throttle publication rate to keep the planning scene churn low.
        self.declare_parameter("publish_rate_hz", 1.0)
        # Skip OctoMaps with fewer than this many encoded bytes so we
        # don't push a "world is empty" message to motion-planning before
        # the camera has seen anything.
        self.declare_parameter("min_data_bytes", 1)
        # Optional safety gate. When configured, no environment is published
        # until a transient-local Bool true arrives on this topic.
        self.declare_parameter("enable_topic", "")

        self._publish_interval = 1.0 / max(
            0.1, float(self.get_parameter("publish_rate_hz").value)
        )
        self._min_data_bytes = int(self.get_parameter("min_data_bytes").value)
        enable_topic = str(self.get_parameter("enable_topic").value)
        self._publishing_enabled = not bool(enable_topic)

        # octomap_server publishes /octomap_binary RELIABLE by default,
        # so we match. The /team/* publisher uses default reliability;
        # environment_bridge subscribes with depth=10 default and
        # tolerates either.
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher_ = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            Octomap,
            self.get_parameter("input_topic").value,
            self._octomap_cb,
            sub_qos,
        )
        if enable_topic:
            gate_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(Bool, enable_topic, self._enable_cb, gate_qos)

        self._latest_octomap = None
        self._last_publish_time = 0.0
        self._published_count = 0

        self.create_timer(0.1, self._maybe_publish)
        self.get_logger().info(
            f"Bridge ready: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} "
            f"(rate {self.get_parameter('publish_rate_hz').value} Hz, "
            f"min payload {self._min_data_bytes} bytes)"
        )
        if enable_topic:
            self.get_logger().info(
                f"Environment publication is gated by {enable_topic}."
            )

    def _enable_cb(self, message):
        was_enabled = self._publishing_enabled
        self._publishing_enabled = bool(message.data)
        if self._publishing_enabled and not was_enabled:
            self.get_logger().info("Environment publication gate opened.")

    def _octomap_cb(self, msg):
        self._latest_octomap = msg

    def _maybe_publish(self):
        if not self._publishing_enabled or self._latest_octomap is None:
            return

        now = time.monotonic()
        if now - self._last_publish_time < self._publish_interval:
            return

        msg = self._latest_octomap
        # octomap_msgs/Octomap.data is int8[]; treat each entry as its
        # signed-byte representation, convert back to unsigned bytes for
        # base64 transport. The team's environment_bridge reverses this
        # exactly the same way.
        data_bytes = bytes((b & 0xFF) for b in msg.data)
        if len(data_bytes) < self._min_data_bytes:
            return

        encoded = base64.b64encode(data_bytes).decode("ascii")

        payload = {
            "frame_id": msg.header.frame_id or "world",
            "version": int(time.time_ns()),
            "replace": True,
            "octomap": {
                "id": msg.id or "OcTree",
                "resolution": float(msg.resolution),
                "binary": bool(msg.binary),
                "origin": {
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                },
                "data_base64": encoded,
            },
        }

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.publisher_.publish(out)
        self._last_publish_time = now
        self._published_count += 1
        if self._published_count <= 3 or self._published_count % 10 == 0:
            # Avoid spamming the log every second forever.
            self.get_logger().info(
                f"#{self._published_count} published "
                f"version={payload['version']} resolution={msg.resolution} "
                f"bytes={len(data_bytes)} frame_id={payload['frame_id']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OctomapToJsonBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
