"""Publish real JAKA A5 joint_states from the vendor jkrc SDK.

Runs alongside a standalone motion script (e.g. real_robot_test_basic/
initial_scan.py) that owns the arm connection for MOTION.  This node only
POLLS joint positions and rebroadcasts them so that ROS 2's
robot_state_publisher can maintain a live TF tree base_link -> ... -> tool0
for OctoMap accumulation with a real end-effector-mounted RealSense.

Two connections to the same arm are usually not allowed by the JAKA
controller.  Therefore this node has two modes:

  * standalone=True  : owns the SDK connection (login + power_on + enable)
                        and polls get_joint_position at rate_hz.  Use this
                        when NO other process is talking to the arm.
  * standalone=False : does login only (no power_on/enable) and only polls
                        get_joint_position.  Use this when a separate
                        motion script owns power+enable.

If even a read-only login is rejected while another script is connected,
fall back to running this node BEFORE the motion script and let the motion
script attach later --- most JAKA firmware permits multiple observers as
long as only one client sends motion commands.
"""

import ctypes
import os
import platform
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ---------------------------------------------------------------------------
# SDK bootstrap.  Mirrors real_robot_test_basic/initial_scan.py::init_sdk_env
# but takes the SDK root from an env var so this works both on the host and
# inside the docker container.
# ---------------------------------------------------------------------------
def _init_sdk_env(sdk_root_override: str = "") -> None:
    if platform.system() != "Linux":
        raise RuntimeError("jaka_state_publisher currently supports Linux only")

    candidates = []
    if sdk_root_override:
        candidates.append(sdk_root_override)
    env_root = os.environ.get("JAKA_SDK_ROOT", "").strip()
    if env_root:
        candidates.append(env_root)
    # Default fallback: assume the SDK is mounted at /opt/jaka_sdk inside the
    # container (see docker/run.sh) or lives next to the ROS workspace on the
    # host.
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
            "Could not find JAKA SDK.  Tried: {}.  Set JAKA_SDK_ROOT or the "
            "'sdk_root' parameter to the folder that contains "
            "'04 Linux/python3/x86_64-linux-gnu/libjakaAPI.so'.".format(
                candidates
            )
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


class JakaStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("jaka_state_publisher")

        self.declare_parameter("robot_ip", "10.5.5.100")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("standalone", True)
        self.declare_parameter("sdk_root", "")
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3",
             "joint_4", "joint_5", "joint_6"],
        )

        ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value
        standalone = self.get_parameter("standalone").get_parameter_value().bool_value
        sdk_root = self.get_parameter("sdk_root").get_parameter_value().string_value
        self._joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        if len(self._joint_names) != 6:
            raise ValueError(
                "joint_names must have exactly 6 entries, got {}".format(
                    self._joint_names
                )
            )

        _init_sdk_env(sdk_root)
        import jkrc  # noqa: E402  # imported after SDK path setup

        self.get_logger().info(
            f"Connecting to JAKA at {ip} (standalone={standalone})"
        )
        self._rc = jkrc.RC(ip)
        err = self._rc.login()
        self.get_logger().info(f"login  -> {err}")
        if standalone:
            self.get_logger().info(f"power_on -> {self._rc.power_on()}")
            self.get_logger().info(f"enable   -> {self._rc.enable_robot()}")
        self._standalone = standalone
        self._lock = threading.Lock()  # jkrc calls should be serialized

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Publishing /joint_states at {rate:.1f} Hz (period {period:.3f}s)"
        )

    def _tick(self) -> None:
        with self._lock:
            ret = self._rc.get_joint_position()
        # jkrc returns (errcode, [j0..j5]).  Skip publish on transient errors
        # rather than crashing the node --- the arm may be re-enabling or
        # streaming a driver-side error.
        if not isinstance(ret, (tuple, list)) or len(ret) < 2:
            self.get_logger().warn(f"unexpected get_joint_position ret: {ret!r}")
            return
        err, q = ret[0], ret[1]
        if err != 0 or q is None:
            self.get_logger().warn(f"get_joint_position err={err}")
            return
        q_list = [float(x) for x in q]
        if len(q_list) != 6:
            self.get_logger().warn(f"expected 6 joints, got {len(q_list)}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = q_list
        self._pub.publish(msg)

    def shutdown(self) -> None:
        with self._lock:
            try:
                if self._standalone:
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
    node = JakaStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
