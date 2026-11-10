#!/usr/bin/env python3

import json
from collections import deque
from time import monotonic

import rclpy
from jaka_planning_interfaces.action import PlanMotion
from jaka_planning_interfaces.msg import EnvironmentModel
from mp_team_bridge.conversions import (
    normalize_quaternion,
    require_bool,
    require_dict,
    require_integer,
    require_number,
    require_string,
    require_vector,
    viewpoint_quaternion,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class DecisionBridge(Node):
    def __init__(self):
        super().__init__("decision_bridge")
        self.declare_parameter("input_topic", "/team/viewpoint_task_json")
        self.declare_parameter("known_environment_topic", "/known_environment")
        self.declare_parameter("action_name", "/plan_motion")
        self.declare_parameter("feedback_topic", "/team/motion_feedback_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("default_planning_group", "jaka_a5")
        self.declare_parameter("default_end_effector_link", "tool0")
        self.declare_parameter("default_position_tolerance", 0.02)
        self.declare_parameter("default_orientation_tolerance", 0.15)
        self.declare_parameter("default_planning_time", 10.0)
        self.declare_parameter("default_planning_attempts", 20)
        self.declare_parameter("default_velocity_scaling", 0.2)
        self.declare_parameter("default_acceleration_scaling", 0.2)
        self.declare_parameter("max_queue_size", 100)
        self.declare_parameter("active_task_timeout_sec", 240.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        # ★★★ P3: cost_probe_* prefix tasks are plan_only queries with no execution;
        # they can safely run in parallel. Non-probe tasks still serialize.
        self.declare_parameter("probe_task_prefix", "cost_probe_")
        # ★ 5-min test 教训:MoveGroup 的 OMPL 底层非线程安全,concurrent
        #   plan_only goal 会互相污染状态导致 32/33 probe 报 error 99999。
        #   默认回到 1(串行)。motion_planner 的 concurrency 基础设施保留,
        #   等 MoveIt 提供真正的 planner pool 后再提高。
        self.declare_parameter("max_parallel_probes", 1)
        # ★ FIX PROBLEM 3: motion_planner rejects incoming goals while it's
        # already handling one ("planner is busy"). Rather than forwarding
        # that as a probe failure (which HPP-PT then treated as an IK
        # rejection and blacklisted the goal), we requeue the probe/task and
        # retry after a short cooldown. Cap the number of retries to avoid
        # infinite loops when motion_planner is stuck.
        self.declare_parameter("busy_retry_max_attempts", 8)
        self.declare_parameter("busy_retry_cooldown_sec", 0.2)

        self.client = ActionClient(
            self, PlanMotion, self.get_parameter("action_name").value
        )
        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.feedback_publisher = self.create_publisher(
            String, self.get_parameter("feedback_topic").value, 10
        )
        self.result_publisher = self.create_publisher(
            String,
            self.get_parameter("result_topic").value,
            retained_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self.task_callback,
            10,
        )
        self.create_subscription(
            EnvironmentModel,
            self.get_parameter("known_environment_topic").value,
            self.environment_callback,
            retained_qos,
        )
        self.create_timer(0.2, self.tick)
        self.queue = deque()
        self.pending_task_ids = set()
        self.active_task = None
        self.active_goal_handle = None
        self.active_send_future = None
        self.active_result_future = None
        self.active_cancel_future = None
        self.active_started_at = None
        self.cancel_started_at = None
        self.active_timeout_message = None
        # ★★★ P3: parallel probe pool
        self.probe_queue = deque()
        self.active_probes = {}   # task_id -> state dict
        self.latest_environment_version = None
        # ★ FIX PROBLEM 3: per-task retry accounting for "planner busy"
        # rejections.  key = task_id, value = (attempts_so_far,
        # earliest_next_retry_monotonic).  Cleared when the task is
        # ultimately started or finally failed.
        self.busy_retry_state = {}
        self.busy_retry_pending_queue = deque()  # tuples (goal, task_id, is_probe)

        self.get_logger().info(
            f"Decision bridge ready on {self.get_parameter('input_topic').value}."
        )

    def environment_callback(self, message):
        self.latest_environment_version = message.version

    @staticmethod
    def json_message(data):
        message = String()
        message.data = json.dumps(data, separators=(",", ":"))
        return message

    def publish_feedback(self, task_id, state, queue_position=None):
        data = {"task_id": task_id, "state": state}
        if queue_position is not None:
            data["queue_position"] = queue_position
        self.feedback_publisher.publish(self.json_message(data))

    def publish_result(
        self,
        task_id,
        success,
        message,
        moveit_error_code=0,
        planning_time=0.0,
        estimated_execution_time=0.0,
        trajectory_points=0,
        environment_version=None,
        environment_applied=None,
        applied_environment_version=None,
        retryable=False,
        error_type="",
    ):
        data = {
            "task_id": task_id,
            "success": success,
            "retryable": retryable,
            "error_type": error_type,
            "moveit_error_code": moveit_error_code,
            "planning_time": planning_time,
            "estimated_execution_time": estimated_execution_time,
            "trajectory_points": trajectory_points,
            "message": message,
        }
        if environment_version is not None:
            data["environment_version"] = environment_version
        if environment_applied is not None:
            data["environment_applied"] = environment_applied
        if environment_applied and applied_environment_version is not None:
            data["applied_environment_version"] = applied_environment_version
        self.result_publisher.publish(self.json_message(data))

    def parameter_or_value(self, data, key, parameter_name, minimum=None, maximum=None):
        value = data.get(key, self.get_parameter(parameter_name).value)
        return require_number(value, key, minimum=minimum, maximum=maximum)

    @staticmethod
    def parse_joint_state(data, field):
        data = require_dict(data, field)
        names = data.get("names")
        positions = data.get("positions")
        if not isinstance(names, list) or not names:
            raise ValueError(f"'{field}.names' must be a non-empty array.")
        names = [
            require_string(name, f"{field}.names[{index}]")
            for index, name in enumerate(names)
        ]
        positions = require_vector(positions, f"{field}.positions", len(names))
        return names, positions

    def convert_task(self, data):
        data = require_dict(data, "root")
        task_id = require_string(data.get("task_id"), "task_id")
        goal_data = require_dict(data.get("goal"), "goal")
        goal_type = require_string(goal_data.get("type"), "goal.type").lower()

        goal = PlanMotion.Goal()
        goal.task_id = task_id
        goal.environment_version = require_integer(
            data.get("environment_version"),
            "environment_version",
            minimum=0,
        )
        goal.planning_group = require_string(
            data.get(
                "planning_group",
                self.get_parameter("default_planning_group").value,
            ),
            "planning_group",
        )
        goal.end_effector_link = require_string(
            data.get(
                "end_effector_link",
                self.get_parameter("default_end_effector_link").value,
            ),
            "end_effector_link",
        )

        goal.use_current_start_state = require_bool(
            data.get("use_current_start_state", True),
            "use_current_start_state",
        )
        if not goal.use_current_start_state:
            names, positions = self.parse_joint_state(
                data.get("start_joint_state"), "start_joint_state"
            )
            goal.start_state.name = names
            goal.start_state.position = positions

        if goal_type == "joint":
            names, positions = self.parse_joint_state(goal_data, "goal")
            goal.goal_type = PlanMotion.Goal.JOINT_GOAL
            goal.joint_goal.name = names
            goal.joint_goal.position = positions
        elif goal_type in {"pose", "viewpoint"}:
            position = require_vector(goal_data.get("position"), "goal.position", 3)
            if goal_type == "viewpoint":
                look_at = require_vector(goal_data.get("look_at"), "goal.look_at", 3)
                orientation = viewpoint_quaternion(position, look_at)
            else:
                orientation = normalize_quaternion(
                    goal_data.get("orientation"), "goal.orientation"
                )

            goal.goal_type = PlanMotion.Goal.POSE_GOAL
            goal.pose_goal.header.frame_id = require_string(
                goal_data.get("frame_id", "world"), "goal.frame_id"
            )
            (
                goal.pose_goal.pose.position.x,
                goal.pose_goal.pose.position.y,
                goal.pose_goal.pose.position.z,
            ) = position
            (
                goal.pose_goal.pose.orientation.x,
                goal.pose_goal.pose.orientation.y,
                goal.pose_goal.pose.orientation.z,
                goal.pose_goal.pose.orientation.w,
            ) = orientation
        else:
            raise ValueError("'goal.type' must be viewpoint, pose, or joint.")

        goal.position_tolerance = self.parameter_or_value(
            data,
            "position_tolerance",
            "default_position_tolerance",
            minimum=0.000001,
        )
        goal.orientation_tolerance = self.parameter_or_value(
            data,
            "orientation_tolerance",
            "default_orientation_tolerance",
            minimum=0.000001,
        )
        goal.plan_only = require_bool(data.get("plan_only", False), "plan_only")
        goal.allowed_planning_time = self.parameter_or_value(
            data, "allowed_planning_time", "default_planning_time", minimum=0.01
        )
        goal.num_planning_attempts = require_integer(
            data.get(
                "num_planning_attempts",
                self.get_parameter("default_planning_attempts").value,
            ),
            "num_planning_attempts",
            minimum=1,
        )
        goal.velocity_scaling = self.parameter_or_value(
            data,
            "velocity_scaling",
            "default_velocity_scaling",
            minimum=0.000001,
            maximum=1.0,
        )
        goal.acceleration_scaling = self.parameter_or_value(
            data,
            "acceleration_scaling",
            "default_acceleration_scaling",
            minimum=0.000001,
            maximum=1.0,
        )
        return task_id, goal

    def task_callback(self, message):
        task_id = "unknown"
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and isinstance(data.get("task_id"), str):
                task_id = data["task_id"]
            task_id, goal = self.convert_task(data)
        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().error(f"Rejected task '{task_id}': {error}")
            self.publish_result(
                task_id, False, str(error), error_type="invalid_task"
            )
            return

        max_queue_size = self.get_parameter("max_queue_size").value
        if self.latest_environment_version is None:
            error = "No known environment has been received."
            self.get_logger().error(f"Rejected task '{task_id}': {error}")
            self.publish_result(
                task_id,
                False,
                error,
                environment_version=goal.environment_version,
                retryable=True,
                error_type="environment_unavailable",
            )
            return
        if goal.environment_version != self.latest_environment_version:
            # A strict equality check here used to reject tasks whose
            # stamped env_version had drifted vs. our current, but that
            # created a permanent failure loop: env updates arrive at
            # ~1 Hz, while a publisher (e.g. the TSP node) may spend
            # hundreds of ms in planning between reading env and
            # publishing the task — so the stamp is stale by the time
            # we see it. Since the motion planner already uses the
            # latest cached OctoMap for planning, silently retagging
            # the goal to our current version is behaviorally
            # equivalent and avoids the loop.
            self.get_logger().warning(
                f"Task '{task_id}' stamped env_version "
                f"{goal.environment_version}; upgrading to current "
                f"{self.latest_environment_version} before dispatch."
            )
            goal.environment_version = self.latest_environment_version
        if task_id in self.pending_task_ids:
            error = f"Task ID '{task_id}' is already queued or active."
            self.get_logger().error(f"Rejected task '{task_id}': {error}")
            self.publish_result(
                task_id, False, error, error_type="duplicate_task_id"
            )
            return
        # ★★★ P3: cost_probe_* 走独立的并行队列，不占 view 任务的串行队列
        probe_prefix = self.get_parameter("probe_task_prefix").value
        is_probe = bool(probe_prefix) and task_id.startswith(probe_prefix)
        target_queue = self.probe_queue if is_probe else self.queue
        if len(target_queue) >= max_queue_size:
            error = f"Task queue is full ({max_queue_size})."
            self.get_logger().error(f"Rejected task '{task_id}': {error}")
            self.publish_result(
                task_id,
                False,
                error,
                retryable=True,
                error_type="queue_full",
            )
            return

        target_queue.append((task_id, goal))
        self.pending_task_ids.add(task_id)
        self.publish_feedback(task_id, "queued", len(target_queue))
        if is_probe:
            self.get_logger().info(
                f"Queued probe '{task_id}' (probe_queue={len(self.probe_queue)}, "
                f"active_probes={len(self.active_probes)})."
            )
            self.try_start_next_probe()
        else:
            self.get_logger().info(
                f"Queued task '{task_id}' at position {len(self.queue)}."
            )
            self.try_start_next_task()

    def try_start_next_task(self):
        if self.active_task is not None or not self.queue:
            return
        if not self.client.server_is_ready():
            return

        task_id, goal = self.queue[0]
        # Silently upgrade to the current env_version if it drifted
        # while queued. See the matching comment in handle_task.
        if self.latest_environment_version != goal.environment_version:
            goal.environment_version = self.latest_environment_version

        self.active_task = self.queue.popleft()
        task_id, goal = self.active_task
        self.active_started_at = monotonic()
        self.cancel_started_at = None
        self.active_timeout_message = None
        self.publish_feedback(task_id, "sending_to_motion_planner")
        self.active_send_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.handle_action_feedback(
                task_id, feedback
            ),
        )
        self.active_send_future.add_done_callback(
            lambda future, expected_task_id=task_id: self.handle_goal_response(
                expected_task_id, future
            )
        )

    def handle_action_feedback(self, task_id, feedback):
        self.publish_feedback(task_id, feedback.feedback.state)

    def handle_goal_response(self, task_id, future):
        if self.active_task is None or self.active_task[0] != task_id:
            return
        try:
            self.active_goal_handle = future.result()
        except Exception as error:
            self.finish_with_bridge_error(
                task_id,
                f"Action request failed: {error}",
                retryable=True,
                error_type="action_request_failed",
            )
            return

        if self.active_goal_handle is None or not self.active_goal_handle.accepted:
            # ★ FIX PROBLEM 3: motion_planner rejects as "planner busy" while
            # it's handling another task.  Requeue this task with a short
            # cooldown so it retries instead of surfacing a false failure.
            goal = self.active_task[1] if self.active_task else None
            requeued = (
                goal is not None
                and self.try_requeue_on_busy(task_id, goal, is_probe=False)
            )
            if requeued:
                # Clear active slot without publishing a result so try_start
                # can pick the retried task on a later tick.
                self.pending_task_ids.discard(task_id)
                self.active_task = None
                self.active_goal_handle = None
                self.active_send_future = None
                self.active_result_future = None
                self.active_cancel_future = None
                self.active_started_at = None
                self.cancel_started_at = None
                self.active_timeout_message = None
                # Re-insert into pending_task_ids so the requeued entry
                # doesn't get rejected as duplicate.
                self.pending_task_ids.add(task_id)
                return
            self.finish_with_bridge_error(
                task_id,
                "Motion planner rejected the task.",
                retryable=True,
                error_type="action_rejected",
            )
            return

        self.active_result_future = self.active_goal_handle.get_result_async()
        self.active_result_future.add_done_callback(
            lambda completed, expected_task_id=task_id: self.handle_action_result(
                expected_task_id, completed
            )
        )

    def handle_action_result(self, task_id, future):
        if self.active_task is None or self.active_task[0] != task_id:
            return
        if self.active_timeout_message is not None:
            self.finish_with_bridge_error(
                task_id,
                self.active_timeout_message,
                retryable=True,
                error_type="active_task_timeout",
            )
            return
        task_id, goal = self.active_task
        try:
            result = future.result().result
            trajectory_points = len(
                result.planned_trajectory.joint_trajectory.points
            )
            self.publish_result(
                task_id,
                result.success,
                result.message,
                result.moveit_error_code,
                result.planning_time,
                result.estimated_execution_time,
                trajectory_points,
                goal.environment_version,
                result.environment_applied,
                result.applied_environment_version,
                result.retryable,
                result.error_type,
            )
            log_message = (
                f"Task '{task_id}' completed: success={result.success}, "
                f"MoveIt error={result.moveit_error_code}, "
                f"planning_time={result.planning_time:.3f}s, "
                f"estimated_execution_time="
                f"{result.estimated_execution_time:.3f}s."
            )
            if result.success:
                self.get_logger().info(log_message)
            else:
                self.get_logger().error(log_message)
        except Exception as error:
            self.publish_result(
                task_id,
                False,
                f"Result handling failed: {error}",
                retryable=True,
                error_type="result_handling_failed",
            )
        finally:
            if self.active_task is not None and self.active_task[0] == task_id:
                self.clear_active_task(task_id)

    def finish_with_bridge_error(
        self, task_id, message, retryable=True, error_type="bridge_error"
    ):
        if self.active_task is None or self.active_task[0] != task_id:
            return
        environment_version = None
        if self.active_task is not None:
            _, goal = self.active_task
            environment_version = goal.environment_version
        self.get_logger().error(f"Task '{task_id}' failed: {message}")
        self.publish_result(
            task_id,
            False,
            message,
            environment_version=environment_version,
            retryable=retryable,
            error_type=error_type,
        )
        self.clear_active_task(task_id)

    def clear_active_task(self, task_id):
        self.pending_task_ids.discard(task_id)
        self.busy_retry_state.pop(task_id, None)
        self.active_task = None
        self.active_goal_handle = None
        self.active_send_future = None
        self.active_result_future = None
        self.active_cancel_future = None
        self.active_started_at = None
        self.cancel_started_at = None
        self.active_timeout_message = None
        self.try_start_next_task()

    # ────────────────── FIX PROBLEM 3: busy-rejection retry helper ──────────
    def try_requeue_on_busy(self, task_id, goal, is_probe):
        """Called when motion_planner returns action_rejected (planner busy).

        Instead of failing the task, put it in a per-task retry queue and
        re-enqueue it after busy_retry_cooldown_sec.  Cap total attempts
        at busy_retry_max_attempts so a stuck motion_planner cannot loop
        forever.  Returns True if the task was requeued (caller must not
        emit a failure), False if attempts were exhausted (caller should
        finalize the error).
        """
        max_attempts = int(self.get_parameter("busy_retry_max_attempts").value)
        cooldown = float(self.get_parameter("busy_retry_cooldown_sec").value)
        attempts = self.busy_retry_state.get(task_id, 0) + 1
        if attempts > max_attempts:
            self.busy_retry_state.pop(task_id, None)
            self.get_logger().warning(
                f"Task '{task_id}' still rejected by motion_planner as busy "
                f"after {max_attempts} attempts; giving up."
            )
            return False
        self.busy_retry_state[task_id] = attempts
        self.busy_retry_pending_queue.append(
            (goal, task_id, is_probe, monotonic() + cooldown, attempts)
        )
        self.get_logger().info(
            f"Task '{task_id}' rejected as busy; requeued for retry "
            f"{attempts}/{max_attempts} in {cooldown:.2f}s."
        )
        return True

    def drain_busy_retry_queue(self):
        """Called from tick(): move due retries back into the appropriate
        primary queue so try_start_next_{task,probe} can pick them up.
        """
        if not self.busy_retry_pending_queue:
            return
        now = monotonic()
        remaining = deque()
        for entry in self.busy_retry_pending_queue:
            goal, task_id, is_probe, ready_at, _attempts = entry
            if now >= ready_at:
                # pending_task_ids already contains this id; do not re-add.
                if is_probe:
                    self.probe_queue.append((task_id, goal))
                else:
                    self.queue.append((task_id, goal))
            else:
                remaining.append(entry)
        self.busy_retry_pending_queue = remaining

    # ────────────────────────────── P3: probe pipeline ──────────────────────────────
    def try_start_next_probe(self):
        max_par = int(self.get_parameter("max_parallel_probes").value)
        while (len(self.active_probes) < max_par and self.probe_queue
               and self.client.server_is_ready()):
            task_id, goal = self.probe_queue.popleft()
            if self.latest_environment_version != goal.environment_version:
                goal.environment_version = self.latest_environment_version
            state = {
                "goal": goal,
                "started_at": monotonic(),
                "send_future": None,
                "goal_handle": None,
                "result_future": None,
                "cancel_future": None,
                "cancel_started_at": None,
                "timeout_message": None,
            }
            self.active_probes[task_id] = state
            self.publish_feedback(task_id, "sending_to_motion_planner")
            state["send_future"] = self.client.send_goal_async(
                goal,
                feedback_callback=lambda fb, tid=task_id:
                    self.publish_feedback(tid, fb.feedback.state),
            )
            state["send_future"].add_done_callback(
                lambda fut, tid=task_id: self.handle_probe_goal_response(tid, fut)
            )

    def handle_probe_goal_response(self, task_id, future):
        state = self.active_probes.get(task_id)
        if state is None:
            return
        try:
            state["goal_handle"] = future.result()
        except Exception as error:
            self.finish_probe_with_error(
                task_id, f"Action request failed: {error}",
                retryable=True, error_type="action_request_failed"
            )
            return
        if state["goal_handle"] is None or not state["goal_handle"].accepted:
            # ★ FIX PROBLEM 3: same busy-retry treatment as main queue.
            goal = state.get("goal")
            requeued = (
                goal is not None
                and self.try_requeue_on_busy(task_id, goal, is_probe=True)
            )
            if requeued:
                # Clear active_probes slot silently; retried entry will
                # come back through try_start_next_probe on a later tick.
                self.active_probes.pop(task_id, None)
                return
            self.finish_probe_with_error(
                task_id, "Motion planner rejected the task.",
                retryable=True, error_type="action_rejected"
            )
            return
        state["result_future"] = state["goal_handle"].get_result_async()
        state["result_future"].add_done_callback(
            lambda fut, tid=task_id: self.handle_probe_action_result(tid, fut)
        )

    def handle_probe_action_result(self, task_id, future):
        state = self.active_probes.get(task_id)
        if state is None:
            return
        if state["timeout_message"] is not None:
            self.finish_probe_with_error(
                task_id, state["timeout_message"],
                retryable=True, error_type="active_task_timeout"
            )
            return
        goal = state["goal"]
        try:
            result = future.result().result
            trajectory_points = len(
                result.planned_trajectory.joint_trajectory.points
            )
            self.publish_result(
                task_id, result.success, result.message, result.moveit_error_code,
                result.planning_time, result.estimated_execution_time,
                trajectory_points, goal.environment_version,
                result.environment_applied, result.applied_environment_version,
                result.retryable, result.error_type,
            )
            log_message = (
                f"Task '{task_id}' completed: success={result.success}, "
                f"MoveIt error={result.moveit_error_code}, "
                f"planning_time={result.planning_time:.3f}s, "
                f"estimated_execution_time="
                f"{result.estimated_execution_time:.3f}s."
            )
            if result.success:
                self.get_logger().info(log_message)
            else:
                self.get_logger().error(log_message)
        except Exception as error:
            self.publish_result(
                task_id, False, f"Result handling failed: {error}",
                retryable=True, error_type="result_handling_failed"
            )
        finally:
            self.clear_probe(task_id)

    def finish_probe_with_error(self, task_id, message,
                                 retryable=True, error_type="bridge_error"):
        state = self.active_probes.get(task_id)
        if state is None:
            return
        env_v = state["goal"].environment_version if state.get("goal") else None
        self.get_logger().error(f"Task '{task_id}' failed: {message}")
        self.publish_result(
            task_id, False, message, environment_version=env_v,
            retryable=retryable, error_type=error_type,
        )
        self.clear_probe(task_id)

    def clear_probe(self, task_id):
        self.pending_task_ids.discard(task_id)
        self.active_probes.pop(task_id, None)
        self.busy_retry_state.pop(task_id, None)
        self.try_start_next_probe()

    def check_active_probe_timeouts(self):
        timeout = float(self.get_parameter("active_task_timeout_sec").value)
        cancel_timeout = float(self.get_parameter("cancel_timeout_sec").value)
        now = monotonic()
        # First pass: identify probes that just hit their timeout (need cancel dispatch)
        just_timed_out = []
        for tid, state in self.active_probes.items():
            if state.get("timeout_message") is not None:
                continue
            if state.get("started_at") is None:
                continue
            if now - state["started_at"] > timeout:
                state["timeout_message"] = (
                    f"Task '{tid}' timed out in decision bridge after "
                    f"{now - state['started_at']:.1f}s."
                )
                just_timed_out.append(tid)
        for tid in just_timed_out:
            state = self.active_probes.get(tid)
            if state is None:
                continue
            self.publish_feedback(tid, "canceling_timed_out_task")
            if state.get("goal_handle") is None:
                if state.get("send_future") is not None:
                    state["send_future"].cancel()
                self.finish_probe_with_error(
                    tid, state["timeout_message"],
                    retryable=True, error_type="active_task_timeout"
                )
                continue
            state["cancel_started_at"] = now
            state["cancel_future"] = state["goal_handle"].cancel_goal_async()
            state["cancel_future"].add_done_callback(
                lambda fut, expected_tid=tid:
                    self.handle_probe_timeout_cancel(expected_tid, fut)
            )
        # Second pass: probes whose cancel itself timed out
        stuck = []
        for tid, state in self.active_probes.items():
            cst = state.get("cancel_started_at")
            if cst is not None and now - cst > cancel_timeout:
                stuck.append(tid)
        for tid in stuck:
            state = self.active_probes.get(tid)
            if state is None:
                continue
            self.finish_probe_with_error(
                tid, (state.get("timeout_message") or f"Task '{tid}' timed out.")
                     + " Cancellation acknowledgement timed out.",
                retryable=True, error_type="active_task_timeout"
            )

    def handle_probe_timeout_cancel(self, task_id, future):
        state = self.active_probes.get(task_id)
        if state is None:
            return
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(
                f"Cancel request for task '{task_id}' failed: {error}"
            )
        self.finish_probe_with_error(
            task_id,
            state.get("timeout_message") or f"Task '{task_id}' timed out.",
            retryable=True, error_type="active_task_timeout"
        )

    # ─────────────────────────────────── tick ───────────────────────────────────
    def tick(self):
        self.check_active_task_timeout()
        self.check_active_probe_timeouts()
        # ★ FIX PROBLEM 3: promote due retries back to the primary queues
        # BEFORE try_start_* runs, so requeued tasks fire on the same tick
        # as any brand-new incoming ones.
        self.drain_busy_retry_queue()
        self.try_start_next_task()
        self.try_start_next_probe()

    def check_active_task_timeout(self):
        if self.active_task is None or self.active_started_at is None:
            return

        now = monotonic()
        task_id, _ = self.active_task
        timeout = float(self.get_parameter("active_task_timeout_sec").value)
        if self.active_timeout_message is None:
            elapsed = now - self.active_started_at
            if elapsed <= timeout:
                return
            self.active_timeout_message = (
                f"Task '{task_id}' timed out in decision bridge after "
                f"{elapsed:.1f}s."
            )
            self.publish_feedback(task_id, "canceling_timed_out_task")
            if self.active_goal_handle is None:
                if self.active_send_future is not None:
                    self.active_send_future.cancel()
                self.finish_with_bridge_error(
                    task_id,
                    self.active_timeout_message,
                    retryable=True,
                    error_type="active_task_timeout",
                )
                return

            self.cancel_started_at = now
            self.active_cancel_future = self.active_goal_handle.cancel_goal_async()
            self.active_cancel_future.add_done_callback(
                lambda completed, expected_task_id=task_id:
                self.handle_timeout_cancel(expected_task_id, completed)
            )
            return

        cancel_timeout = float(self.get_parameter("cancel_timeout_sec").value)
        if self.cancel_started_at is not None and now - self.cancel_started_at > cancel_timeout:
            self.finish_with_bridge_error(
                task_id,
                self.active_timeout_message + " Cancellation acknowledgement timed out.",
                retryable=True,
                error_type="active_task_timeout",
            )

    def handle_timeout_cancel(self, task_id, future):
        if self.active_task is None or self.active_task[0] != task_id:
            return
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(
                f"Cancel request for task '{task_id}' failed: {error}"
            )
        self.finish_with_bridge_error(
            task_id,
            self.active_timeout_message or f"Task '{task_id}' timed out.",
            retryable=True,
            error_type="active_task_timeout",
        )


def main(args=None):
    rclpy.init(args=args)
    node = DecisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
