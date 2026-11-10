#!/usr/bin/env python3
"""Sim initial scan: 4-phase continuous J1 sweep for map bootstrap.

Mirrors `jaka_scan_and_publish.py` (real arm) so that sim and hardware
produce comparable coverage. The arm is held fully vertical from a
taught pose (J2=90°, J3=0°, J5=90°, J6=0°) and only J1 (base rotation)
and J4 (camera pitch) change. Each phase is a single joint-space goal;
MoveIt plans and executes it as one continuous trajectory.

Per phase:
  * J4 stays at one pitch (0° / 45° / 90° / 135°)
  * J1 sweeps a full 360° with sweep direction alternating per phase

An explicit HOME task drives the arm to the phase-1 start pose first
so the scan starts from a known configuration regardless of where the
sim spawns the arm.

An empty OctoMap is published up front so MoveIt sees no world obstacles
during the scan; only SRDF self-collision is enforced.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# --- Fixed base joint values (taught pose, applied to every waypoint). ---
VERTICAL_J2_DEG = 90.0
VERTICAL_J3_DEG = 0.0
VERTICAL_J5_DEG = 90.0
VERTICAL_J6_DEG = 0.0

# Five camera-pitch bands. J4 = 0° camera straight down, 180° camera
# straight up. camera_up phase actively scans the airspace directly
# above the arm so it becomes known-free; without it, HPP-PT sees a
# free-unknown boundary layer at z≈0.7-0.85m and picks "surrogate"
# viewpoints that force the camera to look up.
DEFAULT_PHASE_NAMES = [
    "camera_down",
    "camera_45deg_down",
    "camera_horizontal",
    "camera_45deg_up",
    "camera_up",
]
DEFAULT_PHASE_J4_DEG = [0.0, 45.0, 90.0, 135.0, 180.0]

# J1 sweep endpoints (degrees). One full turn per phase; direction
# alternates. J1 limit is [-90°, +270°], so -90° → +270° covers 360°.
DEFAULT_J1_START_DEG = -90.0
DEFAULT_J1_RANGE_DEG = 360.0

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class InitialScan(Node):
    def __init__(self):
        super().__init__("initial_scan")
        self.declare_parameter("env_topic", "/team/optimized_environment_json")
        self.declare_parameter("env_status_topic", "/team/environment_status_json")
        self.declare_parameter("task_topic", "/team/viewpoint_task_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("task_timeout_sec", 120.0)
        self.declare_parameter("env_accept_timeout_sec", 15.0)
        self.declare_parameter("continue_on_failure", False)
        # 4-phase continuous scan config (mirror of real-arm scan).
        self.declare_parameter("phase_names", DEFAULT_PHASE_NAMES)
        self.declare_parameter("phase_j4_deg", DEFAULT_PHASE_J4_DEG)
        self.declare_parameter("vertical_j2_deg", VERTICAL_J2_DEG)
        self.declare_parameter("vertical_j3_deg", VERTICAL_J3_DEG)
        self.declare_parameter("vertical_j5_deg", VERTICAL_J5_DEG)
        self.declare_parameter("vertical_j6_deg", VERTICAL_J6_DEG)
        self.declare_parameter("j1_start_deg", DEFAULT_J1_START_DEG)
        self.declare_parameter("j1_range_deg", DEFAULT_J1_RANGE_DEG)
        # Velocity/acceleration scaling on the MoveIt tasks. Slower = more
        # camera frames per unit J1 rotation, but longer scan.
        # ★ 0.2 → 0.4: 双倍速度,scan 总时长 ~2:54 → ~1:27。MoveIt 内部
        #   梯形加减速处理平滑;OctoMap 融合速率 ~1Hz,depth camera 30Hz,
        #   足够跟上加速后的角速度。
        self.declare_parameter("sweep_velocity_scaling", 0.4)
        self.declare_parameter("sweep_acceleration_scaling", 0.4)
        # ★ iterations: run the whole phase loop N times back-to-back,
        #   emitting a /viz_scan_diag_trigger String "iter_1" .. "iter_N"
        #   after each iteration so downstream diag nodes dump per-scan
        #   snapshots without needing a wrapper to coordinate re-runs.
        self.declare_parameter("iterations", 1)
        self.declare_parameter("between_iteration_pause_sec", 3.0)

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
        # Trigger for viz_scan_diag: publish a String label per iteration
        # so the diag node dumps /tmp/viz_scan_diag_<label>.json.
        self.diag_trigger_pub = self.create_publisher(
            String, "/viz_scan_diag_trigger", 10
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
        self._latest_result = {}
        self.get_logger().info(
            "initial_scan node ready (4-phase continuous J1 sweep)."
        )

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

    def _wait_until(self, predicate, timeout_sec, message):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f"Timed out: {message}")
        return False

    def publish_empty_environment(self):
        self.get_logger().info(
            "Waiting for environment_bridge and decision_bridge to subscribe..."
        )
        if not self._wait_until(
            lambda: (
                self.env_pub.get_subscription_count() > 0
                and self.task_pub.get_subscription_count() > 0
            ),
            timeout_sec=30.0,
            message="bridges never subscribed to /team/* topics",
        ):
            return False
        env_payload = {
            "frame_id": self.get_parameter("world_frame").value,
            "version": int(time.time_ns()),
            "replace": True,
            "octomap": {
                "id": "OcTree",
                "resolution": 0.05,
                "binary": True,
                "origin": {
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                },
                "data_base64": "",
            },
        }
        msg = String()
        msg.data = json.dumps(env_payload, separators=(",", ":"))
        self.env_pub.publish(msg)
        self.get_logger().info(
            f"Empty environment published (version={env_payload['version']})."
        )
        return self._wait_until(
            lambda: self._env_accepted_version == env_payload["version"],
            timeout_sec=float(self.get_parameter("env_accept_timeout_sec").value),
            message="environment_bridge did not accept the empty seed",
        )

    def send_joint_goal(self, label, joints_full):
        task_id = f"initial_scan_{label}_{time.time_ns()}".replace(" ", "_")
        vel = float(self.get_parameter("sweep_velocity_scaling").value)
        acc = float(self.get_parameter("sweep_acceleration_scaling").value)
        task = {
            "task_id": task_id,
            "environment_version": self._env_accepted_version,
            "goal": {
                "type": "joint",
                "names": list(JOINT_NAMES),
                "positions": [float(v) for v in joints_full],
            },
            "allowed_planning_time": 10.0,
            "num_planning_attempts": 20,
            "velocity_scaling": vel,
            "acceleration_scaling": acc,
        }
        msg = String()
        msg.data = json.dumps(task, separators=(",", ":"))
        self.task_pub.publish(msg)
        self.get_logger().info(
            f"[{label}] joints={[round(math.degrees(v), 1) for v in joints_full]}°"
        )
        if not self._wait_until(
            lambda: task_id in self._latest_result,
            timeout_sec=float(self.get_parameter("task_timeout_sec").value),
            message=f"no motion_result for '{task_id}'",
        ):
            return False
        result = self._latest_result[task_id]
        success = bool(result.get("success"))
        self.get_logger().info(
            f"[{label}] success={success}, "
            f"trajectory_points={result.get('trajectory_points')}, "
            f"message={result.get('message', '')[:80]}"
        )
        return success

    def _run_one_pass(self, iter_idx, iters):
        """Do one full sweep of all phases. Reused across iterations."""
        phase_names = list(self.get_parameter("phase_names").value)
        phase_j4_deg = list(self.get_parameter("phase_j4_deg").value)
        j2 = math.radians(float(self.get_parameter("vertical_j2_deg").value))
        j3 = math.radians(float(self.get_parameter("vertical_j3_deg").value))
        j5 = math.radians(float(self.get_parameter("vertical_j5_deg").value))
        j6 = math.radians(float(self.get_parameter("vertical_j6_deg").value))
        j1_start_deg = float(self.get_parameter("j1_start_deg").value)
        j1_range_deg = float(self.get_parameter("j1_range_deg").value)
        continue_on_fail = bool(self.get_parameter("continue_on_failure").value)

        if len(phase_names) != len(phase_j4_deg):
            self.get_logger().error(
                f"phase_names ({len(phase_names)}) and phase_j4_deg "
                f"({len(phase_j4_deg)}) must have equal length"
            )
            return False

        self.get_logger().info(
            f"==> Iter {iter_idx}/{iters}: {len(phase_names)} phases, "
            f"J1 range {j1_range_deg:+.1f}° per phase"
        )

        successes, failures = [], []
        # After iteration 1 the arm is not at j1_start_deg any more --
        # it's at the end pose of the last phase. We could re-home it,
        # but sending the phase-1 pitch/sweep targets from wherever we
        # are lets MoveIt figure out the re-approach as part of the
        # first task in the new iteration. So we always start each
        # iteration by explicitly commanding the phase-1 START pose.
        current_j1_deg = j1_start_deg
        if iter_idx > 1:
            home_target = [
                math.radians(j1_start_deg), j2, j3,
                math.radians(phase_j4_deg[0]),
                j5, j6,
            ]
            if not self.send_joint_goal(f"iter{iter_idx}_home", home_target):
                self.get_logger().error(
                    f"iter{iter_idx}_home failed; aborting iteration")
                return False

        for phase_idx, (name, j4_deg) in enumerate(zip(phase_names, phase_j4_deg)):
            j4 = math.radians(j4_deg)
            direction = 1.0 if phase_idx % 2 == 0 else -1.0
            j1_end_deg = current_j1_deg + direction * j1_range_deg

            if phase_idx > 0:
                pitch_label = f"iter{iter_idx}_{name}_pitch"
                pitch_target = [
                    math.radians(current_j1_deg), j2, j3, j4, j5, j6,
                ]
                if not self.send_joint_goal(pitch_label, pitch_target):
                    self.get_logger().error(
                        f"{pitch_label} failed; aborting"
                    )
                    failures.append(pitch_label)
                    if not continue_on_fail:
                        return False
                else:
                    successes.append(pitch_label)

            sweep_label = f"iter{iter_idx}_{name}_sweep"
            sweep_target = [math.radians(j1_end_deg), j2, j3, j4, j5, j6]
            self.get_logger().info(
                f"==> ITER {iter_idx} PHASE {phase_idx + 1}/{len(phase_names)}: "
                f"{name} J1 {current_j1_deg:+.1f}° -> {j1_end_deg:+.1f}° "
                f"({'CCW' if direction > 0 else 'CW'}), J4={j4_deg:+.1f}°"
            )
            if not self.send_joint_goal(sweep_label, sweep_target):
                self.get_logger().error(f"{sweep_label} failed; aborting")
                failures.append(sweep_label)
                if not continue_on_fail:
                    return False
            else:
                successes.append(sweep_label)
                current_j1_deg = j1_end_deg

        total = len(successes) + len(failures)
        self.get_logger().info(
            f"Iter {iter_idx} scan complete: {len(successes)}/{total} OK. "
            f"succeeded={successes} failed={failures}"
        )
        return len(failures) == 0

    def run(self):
        if not self.publish_empty_environment():
            return False
        iters = int(self.get_parameter("iterations").value)
        pause = float(self.get_parameter("between_iteration_pause_sec").value)
        all_ok = True
        for i in range(1, iters + 1):
            ok = self._run_one_pass(i, iters)
            if not ok:
                all_ok = False
                # Even on failure, publish the trigger so viz_scan_diag
                # can snapshot the partial state -- helps see how far the
                # scan got before the fault.
            # Emit the diag snapshot trigger for this iteration.
            trig = String()
            trig.data = f"iter_{i}"
            self.diag_trigger_pub.publish(trig)
            self.get_logger().info(
                f"published /viz_scan_diag_trigger data=iter_{i}"
            )
            if i < iters:
                self.get_logger().info(
                    f"pausing {pause:.1f}s before iter {i + 1}...")
                # Sleep while spinning so subscribers keep processing.
                deadline = time.monotonic() + pause
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
        # Also emit legacy "Initial scan complete" line for anything
        # grepping launch logs.
        self.get_logger().info(
            f"Initial scan complete: total_iterations={iters}, "
            f"all_ok={all_ok}"
        )
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = InitialScan()
    success = False
    try:
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
