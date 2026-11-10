#!/usr/bin/env python3
"""Bootstrap scan via the motion-planning team's JSON API.

What it does, end-to-end:
  1. Publishes an initial "environment" JSON to
     /team/optimized_environment_json. This is needed because the team's
     decision_bridge requires an exact environment version match,
     so a task can only be accepted after some environment version has
     been registered. We reuse the teammate's example environment file
     (which already contains a valid OctoMap payload) as the initial
     stand-in, just to get past the version handshake.
  2. Waits for /team/environment_status_json to come back with
     accepted=true and remembers the accepted version.
  3. For every joint configuration in SCAN_POSES, publishes a
     `joint` goal task JSON to /team/viewpoint_task_json. The team's
     motion_planner uses MoveIt to plan + execute through the
     gz_ros2_control controller, so the arm physically moves in Gazebo.
     Each task waits for /team/motion_result_json before sending the next.
  4. octomap_server keeps fusing new point clouds the whole time, so the
     OctoMap visualization in RViz grows during the scan.

Edit SCAN_POSES to taste once you can see what the camera actually looks at.
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# Cartesian viewpoints for the bootstrap scan. Each entry is
#   (position [x,y,z], look_at [x,y,z])  in the WORLD frame.
#
# How this works end-to-end:
#   decision_bridge.viewpoint_quaternion(position, look_at) builds a
#   rotation whose +Z axis is (look_at - position).normalized. That
#   rotation is applied to end_effector_link, which defaults to `tool0`.
#   Our URDF mounts the camera so camera_link +X = tool0 +Z (because
#   camera_mount_joint has rpy "0 -pi/2 0"), so when MoveIt aligns
#   tool0 +Z with the look-at direction, the CAMERA also points at
#   look_at. The ~1.25 cm physical offset between tool0 and the camera
#   optical center is negligible for our viewing distances (~30-50 cm).
#
# Box at world (0.85, 0, 0.20), 0.40 x 0.30 x 0.40 m:
#   top center  = (0.85, 0,    0.40)
#   front face  = (0.65, 0,    0.20)
#
# 5 viewpoints: 3-pose top-down sweep + 2-pose front-face sweep. With
# self_filter dropping the robot's own arm from /camera/points, these 5
# views are enough to render box top + front + the upper part of both
# side faces in OctoMap.
SCAN_VIEWPOINTS = [
    # ---- Top-down sweep ----
    ([0.50,  0.00, 0.55], [0.85, 0.00, 0.40]),  # 1. top center
    ([0.50,  0.20, 0.55], [0.85, 0.00, 0.40]),  # 2. top from +Y
    ([0.50, -0.20, 0.55], [0.85, 0.00, 0.40]),  # 3. top from -Y

    # ---- Front face from two diagonals ----
    ([0.40,  0.20, 0.35], [0.85, 0.00, 0.20]),  # 4. front from +Y diagonal
    ([0.40, -0.20, 0.35], [0.85, 0.00, 0.20]),  # 5. front from -Y diagonal
]

# Default fallback path to the teammate's seed environment JSON. The
# bridge will only consume a valid OctoMap payload, so we point at the
# example file the integration_test_client already uses.
DEFAULT_SEED_ENV_PATH = (
    "/root/ros2_ws/examples/environment_v1.json"
)


class BootstrapScan(Node):
    def __init__(self):
        super().__init__("bootstrap_scan")

        self.declare_parameter("env_topic", "/team/optimized_environment_json")
        self.declare_parameter("env_status_topic", "/team/environment_status_json")
        self.declare_parameter("task_topic", "/team/viewpoint_task_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("seed_env_path", DEFAULT_SEED_ENV_PATH)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("task_timeout_sec", 60.0)
        self.declare_parameter("env_accept_timeout_sec", 15.0)
        # If True, keep going through SCAN_POSES even when one fails
        # (e.g. IK no-solution, in-collision). Useful for orbit scans
        # where some hand-picked poses are out of reach but most work.
        self.declare_parameter("continue_on_failure", True)

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.env_pub = self.create_publisher(
            String, self.get_parameter("env_topic").value, 10
        )
        self.task_pub = self.create_publisher(
            String, self.get_parameter("task_topic").value, 10
        )
        self.create_subscription(
            String,
            self.get_parameter("env_status_topic").value,
            self._env_status_cb,
            retained_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("result_topic").value,
            self._result_cb,
            retained_qos,
        )

        self._env_accepted_version = None
        self._latest_result = {}  # task_id -> {success, message}

        self.get_logger().info("Bootstrap scan node ready.")

    # ----- callbacks -----
    def _env_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("accepted") and payload.get("version") is not None:
            self._env_accepted_version = payload["version"]
            self.get_logger().info(
                f"Environment version {payload['version']} accepted."
            )

    def _result_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "task_id" in payload:
            self._latest_result[payload["task_id"]] = payload

    # ----- helpers -----
    def _wait_until(self, predicate, timeout_sec, message):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f"Timed out: {message}")
        return False

    def _publish_string(self, publisher, data):
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        publisher.publish(msg)

    # ----- main flow -----
    def publish_seed_environment(self):
        seed_path = self.get_parameter("seed_env_path").value
        if not os.path.isfile(seed_path):
            self.get_logger().error(
                f"Seed environment file not found: {seed_path}. "
                "Set the seed_env_path parameter, or copy the teammate's "
                "examples/environment_v1.json into the container."
            )
            return False

        # Wait for environment_bridge AND decision_bridge to subscribe
        # before publishing. With default VOLATILE QoS, messages sent
        # before the subscriber connects are silently dropped, which is
        # exactly the failure mode the integration_test_client guards
        # against.
        self.get_logger().info(
            "Waiting for environment_bridge and decision_bridge to subscribe..."
        )
        if not self._wait_until(
            lambda: (
                self.env_pub.get_subscription_count() > 0
                and self.task_pub.get_subscription_count() > 0
            ),
            timeout_sec=30.0,
            message="bridges never subscribed to /team/* topics "
                    "(environment_bridge or decision_bridge not running?)",
        ):
            return False
        self.get_logger().info("Bridges are subscribed. Publishing seed env.")

        with open(seed_path, "r") as fh:
            env_payload = json.load(fh)
        env_payload["version"] = int(time.time_ns())
        env_payload["replace"] = True

        msg = String()
        msg.data = json.dumps(env_payload, separators=(",", ":"))
        self.env_pub.publish(msg)
        self.get_logger().info(
            f"Seed environment published (version={env_payload['version']}); "
            "waiting for environment_bridge to accept it."
        )
        return self._wait_until(
            lambda: self._env_accepted_version == env_payload["version"],
            timeout_sec=float(self.get_parameter("env_accept_timeout_sec").value),
            message="environment_bridge did not accept the seed environment",
        )

    def send_viewpoint_goal(self, index, position, look_at):
        env_version = self._env_accepted_version
        task_id = f"bootstrap_view_{index}_{time.time_ns()}"
        task = {
            "task_id": task_id,
            "environment_version": env_version,
            "goal": {
                "type": "viewpoint",
                "frame_id": "world",
                "position": list(position),
                "look_at": list(look_at),
            },
            "plan_only": bool(self.get_parameter("plan_only").value),
            "allowed_planning_time": 10.0,
            "num_planning_attempts": 20,
            "velocity_scaling": 0.2,
            "acceleration_scaling": 0.2,
        }
        self._publish_string(self.task_pub, task)
        self.get_logger().info(
            f"[{index + 1}/{len(SCAN_VIEWPOINTS)}] sent '{task_id}': "
            f"pos={position} look_at={look_at}"
        )

        timeout = float(self.get_parameter("task_timeout_sec").value)
        if not self._wait_until(
            lambda: task_id in self._latest_result,
            timeout_sec=timeout,
            message=f"no motion_result for task '{task_id}'",
        ):
            return False

        result = self._latest_result[task_id]
        success = bool(result.get("success"))
        self.get_logger().info(
            f"[{index + 1}/{len(SCAN_VIEWPOINTS)}] success={success}, "
            f"planning_time={result.get('planning_time')}, "
            f"estimated_execution_time="
            f"{result.get('estimated_execution_time')}, "
            f"trajectory_points={result.get('trajectory_points')}, "
            f"message={result.get('message', '')[:100]}"
        )
        return success

    def run(self):
        if not self.publish_seed_environment():
            return False
        continue_on_fail = bool(self.get_parameter("continue_on_failure").value)
        successes = []
        failures = []
        for index, (position, look_at) in enumerate(SCAN_VIEWPOINTS):
            ok = self.send_viewpoint_goal(index, position, look_at)
            (successes if ok else failures).append(index + 1)
            if not ok and not continue_on_fail:
                self.get_logger().error(
                    f"Aborting bootstrap scan at viewpoint {index + 1}."
                )
                return False
        self.get_logger().info(
            f"Bootstrap scan complete: {len(successes)}/{len(SCAN_VIEWPOINTS)} OK. "
            f"succeeded={successes} failed={failures}"
        )
        return len(failures) == 0


def main(args=None):
    rclpy.init(args=args)
    node = BootstrapScan()
    success = False
    try:
        # Brief settle so subscribers are wired up before publishing.
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        success = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
