"""Single-process JAKA connection: owns jkrc, publishes /joint_states at
50 Hz in a background thread, AND executes the 6-direction initial scan
trajectory in the main thread.

Solves the "JAKA rejects the second login" problem: only one client owns
the arm, but ROS 2 (RViz / robot_state_publisher / octomap_server) still
gets live joint states, so TF updates and the map accumulates correctly
while the arm moves.

Trajectory + wrist IK are copied verbatim from
real_robot_test_basic/initial_scan.py so behaviour matches what the user
has already validated on hardware.

Run inside the docker container:

    ros2 run jaka_exploration_core jaka_scan_and_publish

or with launch:

    ros2 launch jaka_exploration_core real_arm_realsense_octomap.launch.py \\
        start_jaka_reader:=false
    ros2 run jaka_exploration_core jaka_scan_and_publish
"""
import ctypes
import math
import os
import platform
import sys
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# SDK bootstrap (matches jaka_state_publisher.py)
# ---------------------------------------------------------------------------
def _init_sdk_env(sdk_root_override: str = "") -> None:
    if platform.system() != "Linux":
        raise RuntimeError("Linux only")
    candidates = []
    if sdk_root_override:
        candidates.append(sdk_root_override)
    env_root = os.environ.get("JAKA_SDK_ROOT", "").strip()
    if env_root:
        candidates.append(env_root)
    candidates.extend([
        "/opt/jaka_sdk",
        os.path.expanduser(
            "~/project/450_g12/JAKA-A5+UR5e/JAKA_A5_Docs/JAKA SDK v2.2.2"
        ),
    ])
    sdk_root = None
    for cand in candidates:
        so = os.path.join(
            cand, "04 Linux", "python3", "x86_64-linux-gnu", "libjakaAPI.so"
        )
        if os.path.isfile(so):
            sdk_root = cand
            break
    if sdk_root is None:
        raise FileNotFoundError(
            f"Could not find JAKA SDK.  Tried: {candidates}."
        )
    so_path = os.path.join(
        sdk_root, "04 Linux", "python3", "x86_64-linux-gnu", "libjakaAPI.so"
    )
    py_path = os.path.join(
        sdk_root, "04 Linux", "python3", "x86_64-linux-gnu"
    )
    ctypes.CDLL(so_path)
    if py_path not in sys.path:
        sys.path.insert(0, py_path)


_init_sdk_env()
import jkrc  # noqa: E402


# ---------------------------------------------------------------------------
# Trajectory constants (verbatim from real_robot_test_basic/initial_scan.py)
# ---------------------------------------------------------------------------
ROBOT_IP = "10.5.5.100"

FIXED_J1 = 0.0
FIXED_J2 = np.pi / 2
FIXED_J3 = -np.pi / 2
INITIAL_J4 = np.pi / 2
INITIAL_J5 = np.pi / 2
INITIAL_J6 = 0.0
INITIAL_JOINTS = [FIXED_J1, FIXED_J2, FIXED_J3, INITIAL_J4, INITIAL_J5, INITIAL_J6]

J4_LIMITS = (-1.48, 4.62)
J5_LIMITS = (-6.28, 6.28)
J6_LIMITS = (-6.28, 6.28)

DIRECTIONS = [
    ("down",     (0.000,  0.000, -1.000)),
    ("forward",  (1.000,  0.000,  0.000)),
    ("right",    (0.000, -1.000,  0.000)),
    ("backward", (-1.000, 0.000,  0.000)),
    ("left",     (0.000,  1.000,  0.000)),
    ("up",       (0.000,  0.000,  1.000)),
]

R_WORLD_J3 = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
)

SLOW_SPEED_DPS = 5.0
# ★ Scan-only speed override. Doubles the wrist J1 sweep speed (5→10 °/s)
#   so a 360° phase drops from ~72s to ~36s, cutting scan total from
#   ~4 min to ~2 min. JAKA A5 rated joint velocity is ~180°/s, so 10°/s
#   is still far below any safety limit. JAKA SDK joint_move handles
#   trapezoidal accel/decel internally → no jerk between waypoints.
#   Trajectory-server execution (motion_planner tasks) keeps
#   SLOW_SPEED_DPS unchanged so exploration behavior is identical.
SCAN_SPEED_DPS = 10.0
DWELL_AT_POSE_S = 2.5
INTER_MOVE_PAUSE_S = 0.5
POS_TOLERANCE_DEG = 1.5
BASE_JOINT_DRIFT_LIMIT_DEG = 2.0

# ---------------------------------------------------------------------------
# New real-arm scan pattern (3 latitude bands × J1 sweep).
#
# The arm folds fully vertical using the following fixed joints (measured
# from the JAKA pendant with the arm taught to the "camera-down" pose):
#
#   J2 = +90°  (shoulder pitches so upper arm stands vertical)
#   J3 =   0°  (elbow straight)
#   J5 = +90°  (wrist bent so the D435 hangs off the flange)
#   J6 =   0°  (no tool roll)
#
# J1 (base rotation) sweeps 360° per phase, and J4 (wrist roll on the
# forearm axis) sets the camera pitch per phase:
#
#   J4 =  0°  -> camera straight down
#   J4 = 45°  -> camera tilted 45° toward horizontal
#   J4 = 90°  -> camera horizontal
#
# All values are launch-tuneable via ROS parameters below.
# ---------------------------------------------------------------------------
VERTICAL_J2_DEG = 90.0
VERTICAL_J3_DEG = 0.0
VERTICAL_J5_DEG = 90.0
VERTICAL_J6_DEG = 0.0

DEFAULT_PHASE_NAMES = [
    "camera_down",
    "camera_45deg_down",
    "camera_horizontal",
    "camera_45deg_up",
    "camera_up",
]
# ★ camera_up (J4=180°) makes the airspace above the arm known-free so
#   HPP-PT does not later pick "sky" surrogate viewpoints.
DEFAULT_PHASE_J4_DEG = [0.0, 45.0, 90.0, 135.0, 180.0]

# J1 sweep: full 360° starting at the taught initial J1. Continuous motion
# per phase (no intermediate stops); direction alternates each phase to
# unwind cables.
DEFAULT_J1_START_DEG = -90.0
DEFAULT_J1_RANGE_DEG = 360.0


# ---------------------------------------------------------------------------
# Wrist IK (verbatim from initial_scan.py)
# ---------------------------------------------------------------------------
def rotation_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _fit_in_limits(joint_val, lo, hi):
    for k in range(-2, 3):
        shifted = joint_val + k * 2 * np.pi
        if lo <= shifted <= hi:
            return shifted
    return None


def _wrist_ik_options(direction_world, up_hint):
    d = np.asarray(direction_world, dtype=float)
    d = d / np.linalg.norm(d)
    up = np.asarray(up_hint, dtype=float)
    up = up / np.linalg.norm(up)
    if abs(np.dot(d, up)) > 0.99:
        return []
    cam_y = np.cross(up, d)
    cam_y = cam_y / np.linalg.norm(cam_y)
    cam_z = np.cross(d, cam_y)
    R_world_cam = np.column_stack([d, cam_y, cam_z])
    R_J3_cam = R_WORLD_J3.T @ R_world_cam
    M = R_J3_cam @ rotation_y(np.pi / 2)
    b_pos = np.arctan2(np.sqrt(M[0, 2] ** 2 + M[1, 2] ** 2), M[2, 2])
    if abs(np.sin(b_pos)) > 1e-6:
        a_pos = np.arctan2(M[1, 2], M[0, 2])
        g_pos = np.arctan2(M[2, 1], -M[2, 0])
    else:
        a_pos = np.arctan2(M[1, 0], M[0, 0])
        g_pos = 0.0
    return [
        (a_pos, -b_pos, g_pos),
        (a_pos + np.pi, b_pos, g_pos + np.pi),
    ]


def solve_wrist_for_direction(
    direction, initial=(INITIAL_J4, INITIAL_J5, INITIAL_J6)
):
    ups = []
    for x in (-1, 0, 1):
        for y in (-1, 0, 1):
            for z in (-1, 0, 1):
                if x == 0 and y == 0 and z == 0:
                    continue
                ups.append(np.array([x, y, z], dtype=float))
    ups += [
        np.array([1.0, 1.0, 0.5]), np.array([1.0, -1.0, 0.5]),
        np.array([-1.0, 1.0, 0.5]), np.array([-1.0, -1.0, 0.5]),
    ]
    best = None
    best_cost = float("inf")
    for up in ups:
        for opt in _wrist_ik_options(direction, up):
            j4, j5, j6 = opt
            j4c = _fit_in_limits(j4, *J4_LIMITS)
            j5c = _fit_in_limits(j5, *J5_LIMITS)
            j6c = _fit_in_limits(j6, *J6_LIMITS)
            if j4c is None or j5c is None or j6c is None:
                continue
            cost = (
                (j4c - initial[0]) ** 2
                + (j5c - initial[1]) ** 2
                + (j6c - initial[2]) ** 2
            )
            if cost < best_cost:
                best_cost = cost
                best = (j4c, j5c, j6c)
    return best


# ---------------------------------------------------------------------------
# ROS 2 node: publish joint_states AND run the scan
# ---------------------------------------------------------------------------
class JakaScanAndPublish(Node):
    def __init__(self) -> None:
        super().__init__("jaka_scan_and_publish")

        self.declare_parameter("robot_ip", ROBOT_IP)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("auto_confirm", False)
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3",
             "joint_4", "joint_5", "joint_6"],
        )
        # scan_only         : run the 6-direction initial scan and exit
        # scan_then_explore : run the scan, then keep running so the
        #                     FollowJointTrajectory action server can
        #                     accept exploration trajectories from MoveIt
        # explore_only      : skip the scan; go straight to serving
        #                     trajectory goals (useful when a scan was
        #                     already done in a previous session)
        self.declare_parameter("run_mode", "scan_only")
        # action server name used by MoveIt's moveit_simple_controller_manager.
        # Must match jaka_a5_moveit_config/config/moveit_controllers.yaml.
        self.declare_parameter(
            "trajectory_action_name",
            "/jaka_a5_controller/follow_joint_trajectory",
        )
        # Per-waypoint tolerance when tracking a MoveIt trajectory.
        self.declare_parameter("exec_pos_tolerance_deg", POS_TOLERANCE_DEG)
        # Skip waypoints that are within this radius of the current pose,
        # to avoid stalling on IK-quantised no-op points.
        self.declare_parameter("waypoint_skip_deg", 0.5)
        # 3-phase × J1-sweep initial scan: arm held vertical, J4 = camera
        # pitch, J1 rotates the whole assembly.
        self.declare_parameter("phase_names", DEFAULT_PHASE_NAMES)
        self.declare_parameter("phase_j4_deg", DEFAULT_PHASE_J4_DEG)
        self.declare_parameter("vertical_j2_deg", VERTICAL_J2_DEG)
        self.declare_parameter("vertical_j3_deg", VERTICAL_J3_DEG)
        self.declare_parameter("vertical_j5_deg", VERTICAL_J5_DEG)
        self.declare_parameter("vertical_j6_deg", VERTICAL_J6_DEG)
        self.declare_parameter("j1_start_deg", DEFAULT_J1_START_DEG)
        self.declare_parameter("j1_range_deg", DEFAULT_J1_RANGE_DEG)

        ip = self.get_parameter("robot_ip").value
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._auto_confirm = bool(self.get_parameter("auto_confirm").value)
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._run_mode = str(self.get_parameter("run_mode").value)
        if self._run_mode not in (
            "scan_only", "scan_then_explore", "explore_only"
        ):
            raise ValueError(
                f"invalid run_mode {self._run_mode!r}; expected one of "
                "scan_only / scan_then_explore / explore_only"
            )
        self._exec_pos_tol_rad = math.radians(
            float(self.get_parameter("exec_pos_tolerance_deg").value)
        )
        self._waypoint_skip_rad = math.radians(
            float(self.get_parameter("waypoint_skip_deg").value)
        )

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._scan_complete_pub = self.create_publisher(
            Bool, "/jaka_initial_scan_complete", scan_qos
        )
        self.publish_scan_complete(False)
        # NOTE: no lock around jkrc calls.  The JAKA SDK allows
        # get_joint_position to be polled concurrently with a blocking
        # joint_move, and serialising them (as an earlier version did)
        # starves the joint-state publisher for the entire duration of
        # every slow move (tens of seconds), which in turn freezes the
        # TF chain and makes octomap_server drop every point cloud with
        # "Lookup would require extrapolation into the past".
        self._stop = threading.Event()
        # Coarse mutual exclusion between the initial-scan main thread and
        # the FollowJointTrajectory action executor thread.  They must
        # never both be commanding motion at the same time.
        self._motion_owner_lock = threading.Lock()
        self._cancel_requested = threading.Event()

        self.get_logger().info(
            f"Connecting to JAKA at {ip} (run_mode={self._run_mode})"
        )
        self._rc = jkrc.RC(ip)
        self.get_logger().info(f"login    -> {self._rc.login()}")
        self.get_logger().info(f"power_on -> {self._rc.power_on()}")
        self.get_logger().info(f"enable   -> {self._rc.enable_robot()}")
        time.sleep(1.0)

        # Sanity: one read after enable must succeed, otherwise abort early.
        ret = self._safe_read()
        if ret is None:
            raise RuntimeError(
                "get_joint_position failed right after enable; check WiFi "
                "and E-stop status."
            )
        self.get_logger().info(
            "initial joints (deg): "
            + ", ".join(f"{math.degrees(v):+.2f}" for v in ret)
        )

        # Start background publisher.
        self._pub_thread = threading.Thread(
            target=self._pub_loop, name="jaka_joint_state_pub", daemon=True
        )
        self._pub_thread.start()

        # FollowJointTrajectory action server (used in scan_then_explore
        # and explore_only modes).  Registered unconditionally so that
        # even in scan_only mode a curious client can see the server
        # (it just rejects goals until the scan is done, which is fine
        # because scan_only exits after the scan anyway).
        action_name = str(self.get_parameter("trajectory_action_name").value)
        self._action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_traj_action,
            goal_callback=self._traj_goal_callback,
            cancel_callback=self._traj_cancel_callback,
            callback_group=self._action_cb_group,
        )
        self._active_goal = None
        self._goal_lock = threading.Lock()
        self.get_logger().info(
            f"FollowJointTrajectory action server ready at {action_name}"
        )

    def publish_scan_complete(self, complete: bool) -> None:
        message = Bool()
        message.data = complete
        self._scan_complete_pub.publish(message)

    # ---- Low-level SDK access, all guarded by _lock -----------------------

    def _safe_read(self):
        """Return the 6-element joint vector, or None on failure."""
        ret = self._rc.get_joint_position()
        if not isinstance(ret, (tuple, list)) or len(ret) < 2:
            return None
        if ret[0] != 0 or ret[1] is None:
            return None
        q = list(ret[1])
        if len(q) != 6:
            return None
        return [float(x) for x in q]

    def _pub_loop(self) -> None:
        period = 1.0 / max(1.0, self._rate)
        misses = 0
        while not self._stop.is_set():
            q = self._safe_read()
            if q is None:
                misses += 1
                if misses % 100 == 1:
                    self.get_logger().warn(
                        f"joint read failing ({misses} times)"
                    )
            else:
                misses = 0
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self._joint_names
                msg.position = q
                self._pub.publish(msg)
            self._stop.wait(period)

    # ---- Motion helpers (mirror real_robot_test_basic/initial_scan.py) ---

    # Reasons _execute_single_target may return:
    #   "ok"        arrived within tolerance
    #   "cancel"    _cancel_requested was set during the poll
    #   "stalled"   error stopped decreasing (soft limit / protective stop)
    #   "timeout"   did not arrive within max_wait_s
    #   "ret_bad"   jkrc.joint_move returned non-zero
    #   "read_bad"  get_joint_position failed
    def _execute_single_target(
        self,
        target_joints_rad,
        label="",
        tolerance_rad=None,
        stall_timeout_ticks=50,
        on_progress=None,
        speed_dps=None,
    ):
        """Move to a single joint target, non-blocking, polling for arrival.
        Uses SLOW_SPEED_DPS by default; caller can override via `speed_dps`
        (used by the initial scan to run at SCAN_SPEED_DPS).  Optionally
        invoke on_progress(actual) at every poll tick so the caller can
        publish feedback.
        """
        if tolerance_rad is None:
            tolerance_rad = math.radians(POS_TOLERANCE_DEG)
        if speed_dps is None:
            speed_dps = SLOW_SPEED_DPS

        current = self._safe_read()
        if current is None:
            self.get_logger().error(f"[{label}] pre-move read failed")
            return "read_bad"
        deltas_deg = [
            math.degrees(target_joints_rad[i] - current[i]) for i in range(6)
        ]
        max_delta = max(abs(d) for d in deltas_deg)
        # If we're already there, skip.  This is important for MoveIt
        # trajectories that contain many nearly-identical points.
        if max_delta < math.degrees(tolerance_rad):
            if on_progress is not None:
                on_progress(current)
            return "ok"

        speed_rad = math.radians(speed_dps)
        est_time = (max_delta / speed_dps) if speed_dps > 0 else 0.0
        self.get_logger().info(
            f"[{label}] moving max Δ={max_delta:.2f}° at {speed_dps}°/s "
            f"(est {est_time:.1f}s)"
        )

        # Issue the motion NON-blocking (is_block=False).  A blocking call
        # keeps the Python GIL held inside the jkrc C extension for the
        # whole travel duration (tens of seconds at 5 deg/s), starving the
        # /joint_states publisher thread.  TF then freezes, RViz shows the
        # arm jumping between waypoints, and octomap_server drops every
        # cloud with "Lookup would require extrapolation into the past".
        # Non-blocking returns immediately; we poll for arrival ourselves
        # via a plain time.sleep loop, which does release the GIL.
        res = self._rc.joint_move(
            list(target_joints_rad), 0, False, speed_rad
        )
        code = res[0] if isinstance(res, tuple) else res
        if code != 0:
            self.get_logger().error(f"[{label}] joint_move issue ret={res}")
            return "ret_bad"

        # Generous timeout: 1.5x the theoretical time + 5s buffer, floor 30s.
        max_wait_s = max(30.0, est_time * 1.5 + 5.0)
        poll_period = 0.1  # 10 Hz status polling
        start = time.time()
        prev_err_rad = float("inf")
        stalled_ticks = 0
        while (time.time() - start) < max_wait_s:
            if self._stop.is_set() or self._cancel_requested.is_set():
                return "cancel"
            time.sleep(poll_period)
            actual = self._safe_read()
            if actual is None:
                continue
            if on_progress is not None:
                on_progress(actual)
            max_err_rad = max(
                abs(actual[i] - target_joints_rad[i]) for i in range(6)
            )
            if max_err_rad < tolerance_rad:
                self.get_logger().info(
                    f"[{label}] reached target in "
                    f"{(time.time() - start):.1f}s"
                )
                return "ok"
            # Stall detection: if the error stops decreasing for ~5s the
            # arm probably tripped a soft limit / protective stop.
            if abs(max_err_rad - prev_err_rad) < math.radians(0.01):
                stalled_ticks += 1
                if stalled_ticks > stall_timeout_ticks:
                    self.get_logger().error(
                        f"[{label}] motion stalled at "
                        f"err={math.degrees(max_err_rad):.2f}°"
                    )
                    return "stalled"
            else:
                stalled_ticks = 0
            prev_err_rad = max_err_rad

        self.get_logger().error(
            f"[{label}] timeout after {max_wait_s:.1f}s "
            f"(final err={math.degrees(prev_err_rad):.2f}°)"
        )
        return "timeout"

    def slow_move_abs(self, target_joints_rad, label="", speed_dps=None):
        # Legacy wrapper used by the initial-scan trajectory. `speed_dps`
        # None → SLOW_SPEED_DPS; scan loop passes SCAN_SPEED_DPS to run
        # twice as fast during initial scan.
        return self._execute_single_target(
            target_joints_rad, label, speed_dps=speed_dps) == "ok"

    def check_base_not_drifted(self):
        q = self._safe_read()
        if q is None:
            return False
        expected = [FIXED_J1, FIXED_J2, FIXED_J3]
        for i, ex in enumerate(expected):
            if abs(math.degrees(q[i] - ex)) > BASE_JOINT_DRIFT_LIMIT_DEG:
                self.get_logger().error(
                    f"base J{i+1} drifted: "
                    f"{math.degrees(q[i]):.2f}° vs {math.degrees(ex):.2f}°"
                )
                return False
        return True

    # ---- FollowJointTrajectory action server -----------------------------
    #
    # This shim makes JAKA look like a standard ros2_control trajectory
    # controller to MoveIt, without actually running controller_manager.
    # MoveIt's moveit_simple_controller_manager (configured in
    # jaka_a5_moveit_config/config/moveit_controllers.yaml) will send
    # planned trajectories here via FollowJointTrajectory.  We ignore the
    # trajectory's time_from_start (MoveIt's timing assumes JAKA's
    # ~200 deg/s controller loop, which is unsafe for us) and instead
    # execute each waypoint at SLOW_SPEED_DPS, polling for arrival.

    def _traj_goal_callback(self, goal_request):
        """Called when a new FollowJointTrajectory goal arrives."""
        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warn(
                    "rejecting trajectory: another goal is still active"
                )
                return GoalResponse.REJECT
            if not self._motion_owner_lock.acquire(blocking=False):
                # Initial scan (or something) is still moving the arm.
                self.get_logger().warn(
                    "rejecting trajectory: motion owner is busy "
                    "(scan probably still running)"
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _traj_cancel_callback(self, goal_handle):
        self.get_logger().info("trajectory cancel requested")
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def _remap_trajectory_point(self, incoming_names, point_positions):
        """Return positions in our canonical joint order (joint_1..joint_6)."""
        try:
            idx = [incoming_names.index(name) for name in self._joint_names]
        except ValueError as exc:
            raise ValueError(
                f"trajectory joint names {incoming_names!r} do not cover "
                f"expected {self._joint_names!r}: {exc}"
            )
        return [float(point_positions[i]) for i in idx]

    def _execute_traj_action(self, goal_handle):
        """Main action-server body: execute the trajectory waypoint by
        waypoint at SLOW_SPEED_DPS.  Publishes feedback with current
        joint state.  Returns a FollowJointTrajectory.Result.
        """
        # _motion_owner_lock was already acquired in _traj_goal_callback.
        self._active_goal = goal_handle
        self._cancel_requested.clear()

        result = FollowJointTrajectory.Result()
        try:
            traj = goal_handle.request.trajectory
            incoming_names = list(traj.joint_names)
            n_points = len(traj.points)
            self.get_logger().info(
                f"trajectory received: {n_points} points, "
                f"names={incoming_names}"
            )
            if n_points == 0:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "empty trajectory"
                goal_handle.abort()
                return result

            # Publish feedback at each poll tick.
            def _publish_feedback(actual, target_positions):
                fb = FollowJointTrajectory.Feedback()
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.joint_names = list(self._joint_names)
                fb.actual.positions = [float(v) for v in actual]
                fb.desired.positions = list(target_positions)
                fb.error.positions = [
                    float(target_positions[i] - actual[i]) for i in range(6)
                ]
                try:
                    goal_handle.publish_feedback(fb)
                except Exception:  # noqa: BLE001
                    pass

            for i, point in enumerate(traj.points):
                if self._cancel_requested.is_set():
                    result.error_code = -1  # generic failure code
                    result.error_string = "cancelled by client"
                    goal_handle.canceled()
                    return result
                try:
                    target = self._remap_trajectory_point(
                        incoming_names, point.positions
                    )
                except ValueError as exc:
                    result.error_code = (
                        FollowJointTrajectory.Result.INVALID_JOINTS
                    )
                    result.error_string = str(exc)
                    goal_handle.abort()
                    return result

                # Skip waypoints that are essentially the current pose to
                # avoid stall-timeouts on noise-level moves.
                current = self._safe_read()
                if current is not None:
                    max_delta_rad = max(
                        abs(target[j] - current[j]) for j in range(6)
                    )
                    if max_delta_rad < self._waypoint_skip_rad:
                        continue

                outcome = self._execute_single_target(
                    target,
                    label=f"traj[{i+1}/{n_points}]",
                    tolerance_rad=self._exec_pos_tol_rad,
                    on_progress=lambda act, t=target: _publish_feedback(
                        act, t
                    ),
                )
                if outcome == "cancel":
                    result.error_code = -1
                    result.error_string = "cancelled during motion"
                    goal_handle.canceled()
                    return result
                if outcome != "ok":
                    result.error_code = (
                        FollowJointTrajectory
                        .Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = (
                        f"waypoint {i+1}/{n_points} failed: {outcome}"
                    )
                    goal_handle.abort()
                    return result

            # All waypoints reached.
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = f"executed {n_points} waypoints"
            self.get_logger().info(
                f"trajectory complete ({n_points} points)"
            )
            goal_handle.succeed()
            return result
        finally:
            self._active_goal = None
            self._cancel_requested.clear()
            # Always release the owner lock; goal_callback grabbed it.
            try:
                self._motion_owner_lock.release()
            except RuntimeError:
                pass

    # ---- Main scan trajectory --------------------------------------------

    def run_scan(self) -> bool:
        # Take the motion owner lock for the whole scan so any incoming
        # action goal is rejected until the scan is done.
        acquired = self._motion_owner_lock.acquire(blocking=False)
        if not acquired:
            self.get_logger().error(
                "cannot start scan: motion owner already held "
                "(a trajectory is executing)"
            )
            return False
        try:
            return self._run_scan_locked()
        finally:
            self._motion_owner_lock.release()

    def _run_scan_locked(self) -> bool:
        # 4-phase continuous scan. Arm is held vertical (J2/J3/J5/J6 fixed
        # from the taught pose); J1 (base rotation) sweeps 360° per phase
        # with no intermediate stops, and J4 (camera pitch) steps between
        # phases. Sweep direction alternates each phase so cables don't
        # accumulate wind and the arm can flow into the next phase without
        # snapping back.
        phase_names = list(self.get_parameter("phase_names").value)
        phase_j4_deg = list(self.get_parameter("phase_j4_deg").value)
        j2 = math.radians(float(self.get_parameter("vertical_j2_deg").value))
        j3 = math.radians(float(self.get_parameter("vertical_j3_deg").value))
        j5 = math.radians(float(self.get_parameter("vertical_j5_deg").value))
        j6 = math.radians(float(self.get_parameter("vertical_j6_deg").value))
        j1_start_deg = float(self.get_parameter("j1_start_deg").value)
        j1_range_deg = float(self.get_parameter("j1_range_deg").value)

        if len(phase_names) != len(phase_j4_deg):
            self.get_logger().error(
                f"phase_names ({len(phase_names)}) and phase_j4_deg "
                f"({len(phase_j4_deg)}) must have equal length"
            )
            return False

        self.get_logger().info(
            f"==> Real-arm scan plan: {len(phase_names)} continuous phases, "
            f"J1 range {j1_range_deg:+.1f}° per phase"
        )
        self.get_logger().info(
            f"    fixed: J2={math.degrees(j2):+.1f}° J3={math.degrees(j3):+.1f}° "
            f"J5={math.degrees(j5):+.1f}° J6={math.degrees(j6):+.1f}°"
        )
        for name, j4 in zip(phase_names, phase_j4_deg):
            self.get_logger().info(f"    {name}: J4={j4:+.1f}°")

        if not self._auto_confirm:
            self.get_logger().info(
                "==> Press Enter to start scan, Ctrl+C to abort"
            )
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                return False
        else:
            for i in range(5, 0, -1):
                self.get_logger().warn(
                    f"auto_confirm=true --- starting motion in {i}s "
                    f"(Ctrl+C to abort)"
                )
                time.sleep(1.0)

        # Explicit home to the taught phase-1 starting pose. The arm may be
        # anywhere (T-pose from a MoveIt session, mid-air from an interrupted
        # scan, etc.), so we always drive it to a known safe pose before the
        # first sweep. A single joint_move interpolates ALL joints linearly,
        # which is fine for a one-shot repositioning but not what we want
        # inside a phase sweep -- hence this separate step.
        home_target = [
            math.radians(j1_start_deg), j2, j3,
            math.radians(phase_j4_deg[0]), j5, j6,
        ]
        self.get_logger().info(
            f"==> HOME: J1={j1_start_deg:+.1f}° J2={math.degrees(j2):+.1f}° "
            f"J3={math.degrees(j3):+.1f}° J4={phase_j4_deg[0]:+.1f}° "
            f"J5={math.degrees(j5):+.1f}° J6={math.degrees(j6):+.1f}°"
        )
        if not self.slow_move_abs(
                home_target, label="HOME", speed_dps=SCAN_SPEED_DPS):
            self.get_logger().error("home move failed; aborting scan")
            return False
        time.sleep(INTER_MOVE_PAUSE_S)

        # Sweep-endpoint bookkeeping. Phase 0 sweeps j1_start_deg -> +range,
        # phase 1 comes back to j1_start_deg, phase 2 goes out again, etc.
        current_j1_deg = j1_start_deg
        for phase_idx, (name, j4_deg) in enumerate(zip(phase_names, phase_j4_deg)):
            j4 = math.radians(j4_deg)
            direction = 1.0 if phase_idx % 2 == 0 else -1.0
            j1_end_deg = current_j1_deg + direction * j1_range_deg

            # Pitch-only step to reposition J4 for this phase (skip phase 1
            # because the HOME step above already parks J4 at phase_j4_deg[0]).
            if phase_idx > 0:
                pitch_target = [
                    math.radians(current_j1_deg), j2, j3, j4, j5, j6,
                ]
                pitch_label = f"{name}_pitch"
                self.get_logger().info(
                    f"==> PHASE {phase_idx + 1}/{len(phase_names)} pitch: "
                    f"J4 -> {j4_deg:+.1f}°"
                )
                if not self.slow_move_abs(
                        pitch_target, label=pitch_label,
                        speed_dps=SCAN_SPEED_DPS):
                    self.get_logger().error(
                        f"pitch move to {pitch_label} failed; aborting scan"
                    )
                    return False

            # Continuous J1 sweep for this phase.
            sweep_target = [math.radians(j1_end_deg), j2, j3, j4, j5, j6]
            sweep_label = f"{name}_sweep"
            self.get_logger().info(
                f"==> PHASE {phase_idx + 1}/{len(phase_names)} sweep: "
                f"J1 {current_j1_deg:+.1f}° -> {j1_end_deg:+.1f}° "
                f"({'CCW' if direction > 0 else 'CW'}), J4={j4_deg:+.1f}°"
            )
            if not self.slow_move_abs(
                    sweep_target, label=sweep_label,
                    speed_dps=SCAN_SPEED_DPS):
                self.get_logger().error(
                    f"sweep {sweep_label} failed; aborting scan"
                )
                return False
            current_j1_deg = j1_end_deg

        self.get_logger().info(
            f"==> Scan complete ({len(phase_names)} phases, "
            f"final J1={current_j1_deg:+.1f}°)"
        )
        return True

    # ---- Shutdown --------------------------------------------------------

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self._pub_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.get_logger().info(
                f"disable  -> {self._rc.disable_robot()}"
            )
            self.get_logger().info(
                f"power_off-> {self._rc.power_off()}"
            )
            self.get_logger().info(f"logout   -> {self._rc.logout()}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"shutdown error: {exc}")


def main() -> None:
    rclpy.init()
    node = JakaScanAndPublish()

    # A MultiThreadedExecutor lets the FollowJointTrajectory action
    # callback run in parallel with the joint-state publisher timer and
    # any other subscription: the executor holds a thread pool and
    # dispatches callbacks concurrently.  The initial scan runs in the
    # main thread; the executor spins in a background thread.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if node._run_mode == "scan_only":
            node.run_scan()
        elif node._run_mode == "scan_then_explore":
            ok = node.run_scan()
            if ok:
                node.publish_scan_complete(True)
                node.get_logger().info(
                    "==> Scan finished.  Now serving trajectory goals via "
                    "FollowJointTrajectory action.  Ctrl+C to exit."
                )
            else:
                node.get_logger().warn(
                    "==> Scan failed but staying up to serve trajectory "
                    "goals anyway.  Ctrl+C to exit."
                )
            # Idle wait so main thread doesn't return; action server runs
            # in the executor thread and will drive motion on demand.
            while not node._stop.is_set():
                time.sleep(0.5)
        elif node._run_mode == "explore_only":
            node.publish_scan_complete(True)
            node.get_logger().info(
                "==> Skipping initial scan (run_mode=explore_only).  "
                "Ready for FollowJointTrajectory goals.  Ctrl+C to exit."
            )
            while not node._stop.is_set():
                time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted by user")
    finally:
        node.shutdown()
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
