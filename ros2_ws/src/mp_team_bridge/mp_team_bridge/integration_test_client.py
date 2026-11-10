#!/usr/bin/env python3

import json
from pathlib import Path
from time import monotonic, time_ns

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class IntegrationTestClient(Node):
    def __init__(self):
        super().__init__("integration_test_client")

        self.declare_parameter("environment_file", "")
        self.declare_parameter("task_file", "")
        self.declare_parameter(
            "environment_topic", "/team/optimized_environment_json"
        )
        self.declare_parameter(
            "environment_status_topic", "/team/environment_status_json"
        )
        self.declare_parameter("task_topic", "/team/viewpoint_task_json")
        self.declare_parameter("feedback_topic", "/team/motion_feedback_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("task_delay_sec", 0.5)
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("use_unique_run_ids", True)

        share_dir = Path(get_package_share_directory("mp_team_bridge"))
        environment_file = self._input_path(
            "environment_file", share_dir / "examples" / "environment_v1.json"
        )
        task_file = self._input_path(
            "task_file", share_dir / "examples" / "viewpoint_task.json"
        )
        self.environment = self._load_json(environment_file)
        self.task = self._load_json(task_file)
        self._prepare_run_identifiers()
        self.task["plan_only"] = self.get_parameter("plan_only").value

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.environment_publisher = self.create_publisher(
            String,
            self.get_parameter("environment_topic").value,
            10,
        )
        self.task_publisher = self.create_publisher(
            String,
            self.get_parameter("task_topic").value,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("environment_status_topic").value,
            self.environment_status_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("feedback_topic").value,
            self.feedback_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("result_topic").value,
            self.result_callback,
            retained_qos,
        )
        self.create_timer(0.1, self.tick)

        self.started_at = monotonic()
        self.environment_published = False
        self.environment_accepted_at = None
        self.task_published = False
        self.finished = False
        self.success = False
        self.state = "waiting_for_bridges"

        self.get_logger().info(
            "Code-driven integration test ready: environment_version=%s, task_id=%s, "
            "plan_only=%s."
            % (
                self.environment["version"],
                self.task["task_id"],
                self.task["plan_only"],
            )
        )

    def _input_path(self, parameter_name, default_path):
        configured = self.get_parameter(parameter_name).value
        return Path(configured).expanduser() if configured else default_path

    @staticmethod
    def _load_json(path):
        try:
            return json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Cannot load JSON file '{path}': {error}") from error

    def _prepare_run_identifiers(self):
        if self.get_parameter("use_unique_run_ids").value:
            run_id = time_ns()
            base_task_id = str(self.task.get("task_id", "integration_test"))
            self.environment["version"] = run_id
            self.task["environment_version"] = run_id
            self.task["task_id"] = f"{base_task_id}_{run_id}"
            return

        if "version" not in self.environment:
            raise RuntimeError("Environment JSON requires 'version'.")
        if "task_id" not in self.task:
            raise RuntimeError("Task JSON requires 'task_id'.")
        self.task["environment_version"] = self.environment["version"]

    @staticmethod
    def _message(data):
        message = String()
        message.data = json.dumps(data, separators=(",", ":"))
        return message

    @staticmethod
    def _parse_message(message, description):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError as error:
            raise ValueError(f"Invalid {description} JSON: {error}") from error
        if not isinstance(data, dict):
            raise ValueError(f"{description} must be a JSON object.")
        return data

    def publish_environment(self):
        self.environment_publisher.publish(self._message(self.environment))
        self.environment_published = True
        self.state = "waiting_for_environment_acceptance"
        self.get_logger().info(
            f"Published environment version {self.environment['version']}; "
            "waiting for acceptance."
        )

    def publish_task(self):
        self.task_publisher.publish(self._message(self.task))
        self.task_published = True
        self.state = "waiting_for_motion_result"
        self.get_logger().info(
            f"Published task '{self.task['task_id']}'; waiting for motion result."
        )

    def environment_status_callback(self, message):
        try:
            status = self._parse_message(message, "environment status")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        if not self.environment_published:
            return
        if status.get("version") != self.environment["version"]:
            return
        if not status.get("accepted", False):
            self.finish(False, f"Environment rejected: {status.get('message', '')}")
            return

        if self.environment_accepted_at is None:
            self.environment_accepted_at = monotonic()
            self.state = "waiting_before_task"
            self.get_logger().info(
                f"Environment version {self.environment['version']} accepted."
            )

    def feedback_callback(self, message):
        try:
            feedback = self._parse_message(message, "motion feedback")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        if feedback.get("task_id") == self.task["task_id"]:
            self.get_logger().info(
                f"Task feedback: {feedback.get('state', 'unknown')}."
            )

    def result_callback(self, message):
        try:
            result = self._parse_message(message, "motion result")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        if not self.task_published or result.get("task_id") != self.task["task_id"]:
            return

        summary = (
            f"success={result.get('success')}, "
            f"moveit_error_code={result.get('moveit_error_code')}, "
            f"planning_time={result.get('planning_time')}, "
            f"estimated_execution_time="
            f"{result.get('estimated_execution_time')}, "
            f"trajectory_points={result.get('trajectory_points')}. "
            f"{result.get('message', '')}"
        )
        self.finish(bool(result.get("success")), summary)

    def tick(self):
        if self.finished:
            return

        if monotonic() - self.started_at > self.get_parameter("timeout_sec").value:
            self.finish(False, f"Timed out while {self.state}.")
            return

        if not self.environment_published:
            bridges_ready = (
                self.environment_publisher.get_subscription_count() > 0
                and self.task_publisher.get_subscription_count() > 0
            )
            if bridges_ready:
                self.publish_environment()
            return

        if self.environment_accepted_at is not None and not self.task_published:
            delay = self.get_parameter("task_delay_sec").value
            if monotonic() - self.environment_accepted_at >= delay:
                self.publish_task()

    def finish(self, success, message):
        if self.finished:
            return
        self.finished = True
        self.success = success
        if success:
            self.get_logger().info(f"INTEGRATION TEST PASSED: {message}")
        else:
            self.get_logger().error(f"INTEGRATION TEST FAILED: {message}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IntegrationTestClient()
    except Exception as error:
        rclpy.shutdown()
        raise SystemExit(f"Cannot start integration test: {error}") from error

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.finish(False, "Interrupted by user.")
    finally:
        success = node.success
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
