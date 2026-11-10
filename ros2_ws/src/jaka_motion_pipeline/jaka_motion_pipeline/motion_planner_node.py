#!/usr/bin/env python3

from copy import deepcopy
from threading import Lock

import rclpy
from geometry_msgs.msg import Pose
from jaka_planning_interfaces.action import PlanMotion
from jaka_planning_interfaces.msg import EnvironmentModel
from jaka_planning_interfaces.srv import (
    BuildConservativeOctomap,
    ValidateKnownFreeTrajectory,
)
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from shape_msgs.msg import SolidPrimitive


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner")

        self.declare_parameter("environment_topic", "/known_environment")
        self.declare_parameter("planning_action", "/plan_motion")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("apply_scene_service", "/apply_planning_scene")
        self.declare_parameter("scene_service_timeout_sec", 10.0)
        self.declare_parameter("move_group_server_timeout_sec", 10.0)
        self.declare_parameter("move_group_goal_response_timeout_sec", 10.0)
        self.declare_parameter("move_group_result_timeout_sec", 180.0)
        self.declare_parameter("move_group_cancel_timeout_sec", 5.0)
        self.declare_parameter("execute_trajectory_action", "/execute_trajectory")
        self.declare_parameter("execute_result_timeout_sec", 180.0)
        self.declare_parameter(
            "known_free_validation_service", "/validate_known_free_trajectory"
        )
        self.declare_parameter("known_free_validation_timeout_sec", 20.0)
        self.declare_parameter("known_free_interpolation_step", 0.08)
        self.declare_parameter("known_free_link_sample_step", 0.02)
        self.declare_parameter("known_free_clearance", 0.06)
        self.declare_parameter(
            "conservative_map_service", "/build_conservative_octomap"
        )
        self.declare_parameter("conservative_map_timeout_sec", 30.0)
        self.declare_parameter("conservative_map_resolution", 0.05)
        self.declare_parameter("conservative_robot_clearance", 0.10)
        self.declare_parameter("conservative_initial_seed_radius", 0.25)

        # Conservative-only planning: a non-exempt task always plans against the
        # CONSERVATIVE map (unknown=obstacle).  If that plan fails we report
        # "moveit_planning_failure" directly -- there is no non-conservative
        # fallback pass any more.
        self.declare_parameter("planning_bounds", [-0.8, 1.2, -1.0, 1.0, 0.0, 1.6])
        self.declare_parameter(
            "known_free_exempt_task_prefixes", ["initial_scan_"]
        )
        self.declare_parameter("default_planning_group", "jaka_a5")
        self.declare_parameter("default_end_effector_link", "tool0")
        self.declare_parameter("default_planning_time", 5.0)
        self.declare_parameter("default_planning_attempts", 10)
        self.declare_parameter("default_velocity_scaling", 0.2)
        self.declare_parameter("default_acceleration_scaling", 0.2)
        # ★ PLAN-ONLY CONCURRENCY (2026-07-20): allow up to N plan_only tasks
        #   to run in parallel.  Real execute tasks stay strictly serial and
        #   also block plan_only tasks (to avoid PlanningScene races with a
        #   moving arm).  0 or 1 restores the old strictly-serial behaviour.
        #
        # NOTE: within a SINGLE move_group process OMPL is not thread-safe
        # (concurrency > 1 caused 32/33 probes to fail with error 99999).
        # HOWEVER, if probe_move_group_actions[] is populated with separate
        # move_group instances (spawned in the launch), each has its own OMPL
        # memory space and true parallelism becomes safe.  In that mode set
        # max_concurrent_plan_only >= len(probe_move_group_actions).
        self.declare_parameter("max_concurrent_plan_only", 1)
        # ★ MULTI-PLANNER POOL (2026-07-20): list of extra move_group action
        #   servers (each in its own process) that motion_planner will
        #   round-robin plan_only probes to.  Empty list => fall back to the
        #   single master move_group (default behaviour).  Each entry is the
        #   full action name, e.g. "/probe_planner_0/move_action".  Must
        #   pair one-to-one with probe_apply_scene_services[] so scene
        #   updates land on the right planner.
        self.declare_parameter("probe_move_group_actions", [""])
        self.declare_parameter("probe_apply_scene_services", [""])

        self._callback_group = ReentrantCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        environment_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._environment_lock = Lock()
        self._latest_environment = None
        self._task_lock = Lock()
        # ★ PLAN-ONLY CONCURRENCY: split the single _busy flag into two.
        #   _execute_busy       : true while a non-plan_only task is running.
        #                          Blocks BOTH new plan_only and new execute
        #                          tasks (real motion is happening; scene
        #                          shouldn't be swapped underneath it).
        #   _active_plan_only   : count of currently-running plan_only probes.
        #                          Bounded by max_concurrent_plan_only.
        #                          Blocks incoming EXECUTE tasks (no point
        #                          racing a scene update against pending
        #                          probes), but not other plan_only probes.
        self._execute_busy = False
        self._active_plan_only = 0
        self._active_move_group_goal = None

        self.create_subscription(
            EnvironmentModel,
            self.get_parameter("environment_topic").value,
            self._environment_callback,
            environment_qos,
            callback_group=self._callback_group,
        )
        self._scene_client = self.create_client(
            ApplyPlanningScene,
            self.get_parameter("apply_scene_service").value,
            callback_group=self._callback_group,
        )
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            self.get_parameter("move_group_action").value,
            callback_group=self._callback_group,
        )
        # ★ MULTI-PLANNER POOL: build per-probe clients (empty strings mean
        #   "no probe planner configured" --- we filter those out and fall
        #   back to the single master for probes too).
        raw_actions = list(
            self.get_parameter("probe_move_group_actions").value
        )
        raw_services = list(
            self.get_parameter("probe_apply_scene_services").value
        )
        probe_action_names = [s for s in raw_actions if s]
        probe_service_names = [s for s in raw_services if s]
        if len(probe_action_names) != len(probe_service_names):
            raise ValueError(
                "probe_move_group_actions and probe_apply_scene_services "
                "must have the same non-empty length (got "
                f"{probe_action_names} vs {probe_service_names})."
            )
        self._probe_move_group_clients = [
            ActionClient(self, MoveGroup, name,
                         callback_group=self._callback_group)
            for name in probe_action_names
        ]
        self._probe_scene_clients = [
            self.create_client(ApplyPlanningScene, name,
                               callback_group=self._callback_group)
            for name in probe_service_names
        ]
        self._probe_planner_names = probe_action_names
        self._probe_rr_index = 0
        self._probe_rr_lock = Lock()
        if self._probe_move_group_clients:
            self.get_logger().info(
                f"probe planner pool: {len(self._probe_move_group_clients)} "
                f"instance(s) -> {probe_action_names}"
            )
        else:
            self.get_logger().info(
                "probe planner pool: (empty) -- all probes go through master "
                "move_group"
            )
        self._execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            self.get_parameter("execute_trajectory_action").value,
            callback_group=self._callback_group,
        )
        self._known_free_client = self.create_client(
            ValidateKnownFreeTrajectory,
            self.get_parameter("known_free_validation_service").value,
            callback_group=self._callback_group,
        )
        self._conservative_map_client = self.create_client(
            BuildConservativeOctomap,
            self.get_parameter("conservative_map_service").value,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            PlanMotion,
            self.get_parameter("planning_action").value,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Ready: environment=%s, planning_action=%s"
            % (
                self.get_parameter("environment_topic").value,
                self.get_parameter("planning_action").value,
            )
        )

    def _environment_callback(self, message):
        with self._environment_lock:
            if (
                self._latest_environment is not None
                and message.version <= self._latest_environment.version
            ):
                self.get_logger().warning(
                    f"Ignoring non-increasing environment version {message.version}; "
                    f"latest immutable version is {self._latest_environment.version}."
                )
                return
            self._latest_environment = deepcopy(message)

        self.get_logger().info(
            f"Cached environment version {message.version} "
            f"with OctoMap payload size {len(message.octomap.octomap.data)} byte(s)."
        )

    def _goal_callback(self, goal_request):
        # ★ PLAN-ONLY CONCURRENCY: admit incoming goals per policy below.
        #   - execute task (plan_only=False):
        #       reject if _execute_busy or _active_plan_only > 0.
        #   - plan_only task (plan_only=True):
        #       reject if _execute_busy;
        #       accept up to max_concurrent_plan_only, else reject.
        max_par = int(self.get_parameter("max_concurrent_plan_only").value)
        is_plan_only = bool(goal_request.plan_only)
        with self._task_lock:
            if self._execute_busy:
                self.get_logger().warning(
                    f"Rejecting task '{goal_request.task_id}': "
                    f"execute in progress (plan_only={is_plan_only})."
                )
                return GoalResponse.REJECT
            if is_plan_only:
                if self._active_plan_only >= max_par:
                    self.get_logger().warning(
                        f"Rejecting task '{goal_request.task_id}': "
                        f"plan_only slots full ({self._active_plan_only}/"
                        f"{max_par})."
                    )
                    return GoalResponse.REJECT
                self._active_plan_only += 1
                return GoalResponse.ACCEPT
            # Non plan_only (execute) --- must be exclusive.
            if self._active_plan_only > 0:
                self.get_logger().warning(
                    f"Rejecting execute task '{goal_request.task_id}': "
                    f"{self._active_plan_only} plan_only probes still in flight."
                )
                return GoalResponse.REJECT
            self._execute_busy = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        if self._active_move_group_goal is not None:
            self._active_move_group_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _await_with_timeout(self, operation, timeout_sec, description):
        completion = Future(executor=self.executor)

        def operation_done(completed):
            if not completion.done():
                completion.set_result(completed)

        def operation_timed_out():
            if not completion.done():
                completion.set_exception(
                    TimeoutError(f"{description} timed out after {timeout_sec:.1f}s.")
                )

        operation.add_done_callback(operation_done)
        timer = self.create_timer(
            timeout_sec,
            operation_timed_out,
            callback_group=self._callback_group,
            clock=self._steady_clock,
        )
        try:
            completed = await completion
            return completed.result()
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def _fail(result, message, error_type, retryable, moveit_error_code=0):
        result.success = False
        result.retryable = retryable
        result.error_type = error_type
        result.moveit_error_code = moveit_error_code
        result.message = message

    async def _cancel_move_group_goal(self):
        if self._active_move_group_goal is None:
            return
        timeout = float(
            self.get_parameter("move_group_cancel_timeout_sec").value
        )
        cancel_future = self._active_move_group_goal.cancel_goal_async()
        try:
            await self._await_with_timeout(
                cancel_future, timeout, "MoveGroup cancellation"
            )
        except Exception as error:
            self.get_logger().warning(f"MoveGroup cancellation did not complete: {error}")

    @staticmethod
    def _feedback(goal_handle, state):
        feedback = PlanMotion.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    async def _build_conservative_environment(self, environment, planning_group):
        timeout = float(self.get_parameter("conservative_map_timeout_sec").value)
        if not self._conservative_map_client.wait_for_service(timeout_sec=timeout):
            return False, None, "Conservative OctoMap builder is unavailable."
        bounds = list(self.get_parameter("planning_bounds").value)
        if len(bounds) != 6:
            return False, None, "planning_bounds must contain six values."
        request = BuildConservativeOctomap.Request()
        request.observed = environment.octomap
        request.environment_version = environment.version
        request.planning_group = planning_group
        (
            request.min_x, request.max_x, request.min_y,
            request.max_y, request.min_z, request.max_z,
        ) = [float(value) for value in bounds]
        request.resolution = float(
            self.get_parameter("conservative_map_resolution").value
        )
        request.robot_clearance = float(
            self.get_parameter("conservative_robot_clearance").value
        )
        request.initial_seed_radius = float(
            self.get_parameter("conservative_initial_seed_radius").value
        )
        future = self._conservative_map_client.call_async(request)
        try:
            response = await self._await_with_timeout(
                future, timeout, "Conservative OctoMap generation"
            )
        except TimeoutError as error:
            future.cancel()
            return False, None, str(error)
        if response is None or not response.success:
            message = response.message if response is not None else "No response."
            return False, None, message
        details = (
            f"Conservative map: free={response.free_cells}, "
            f"unknown_as_occupied={response.unknown_as_occupied_cells}, "
            f"observed_occupied={response.observed_occupied_cells}, "
            f"robot_carved={response.robot_carved_cells}, "
            f"initial_seed_carved={response.initial_seed_carved_cells}."
        )
        self.get_logger().info(details)
        return True, response.conservative, details

    def _pick_probe_client_pair(self):
        """Round-robin pick a (move_group_client, scene_client) pair from
        the probe planner pool.  Returns (None, None) when the pool is
        empty --- callers then fall back to master."""
        if not self._probe_move_group_clients:
            return None, None, None
        with self._probe_rr_lock:
            idx = self._probe_rr_index
            self._probe_rr_index = (
                (self._probe_rr_index + 1)
                % len(self._probe_move_group_clients)
            )
        return (
            self._probe_move_group_clients[idx],
            self._probe_scene_clients[idx],
            self._probe_planner_names[idx],
        )

    async def _apply_environment(
        self, required_version, planning_group, allow_unknown=False,
        scene_client=None, environment=None,
    ):
        # ★ NON-CONSERVATIVE FALLBACK FIX: when `environment` is provided, reuse
        #   that EXACT snapshot (skip the latest-fetch and the version check).
        #   The conservative pass and the non-conservative fallback pass must
        #   plan against the identical map; re-fetching latest for the 2nd pass
        #   version-mismatched because the OctoMap drifted during the 1st plan.
        #   When None (legacy path), fetch latest and validate the version.
        if environment is None:
            with self._environment_lock:
                environment = deepcopy(self._latest_environment)
            if environment is None:
                return (
                    False,
                    "No known environment has been received.",
                    "environment_unavailable",
                    True,
                )
            if environment.version != required_version:
                return (
                    False,
                    f"Required environment version {required_version} does not "
                    f"match cached version {environment.version}.",
                    "environment_version_mismatch",
                    True,
                )
        if not environment.replace:
            return (
                False,
                "OctoMap environment updates must be complete snapshots.",
                "invalid_environment",
                False,
            )
        if not environment.octomap.octomap.id:
            return (
                False,
                "OctoMap ID must be non-empty.",
                "invalid_environment",
                False,
            )
        if environment.octomap.octomap.resolution <= 0.0:
            return (
                False,
                "OctoMap resolution must be positive.",
                "invalid_environment",
                False,
            )

        map_message = environment.octomap
        conservative_message = ""
        if not allow_unknown:
            map_ok, map_message, conservative_message = (
                await self._build_conservative_environment(
                    environment, planning_group
                )
            )
            if not map_ok:
                return False, conservative_message, "conservative_map_failed", True

        scene = PlanningScene()
        scene.is_diff = True
        octomap_size = len(environment.octomap.octomap.data)
        # A non-empty OctoMap ID makes MoveIt process this field even when the
        # payload is empty. processOctomapMsg() then removes the previous map.
        scene.world.octomap = map_message

        timeout = float(self.get_parameter("scene_service_timeout_sec").value)
        # ★ MULTI-PLANNER POOL: use the caller-provided scene_client if any
        #   (routes the scene update to the specific probe planner we're
        #   about to send the goal to).  Otherwise use the master client.
        active_scene_client = scene_client or self._scene_client
        if not active_scene_client.wait_for_service(timeout_sec=timeout):
            return (
                False,
                "apply_planning_scene service unavailable.",
                "planning_scene_unavailable",
                True,
            )

        request = ApplyPlanningScene.Request()
        request.scene = scene
        scene_future = active_scene_client.call_async(request)
        try:
            response = await self._await_with_timeout(
                scene_future, timeout, "PlanningScene update"
            )
        except TimeoutError as error:
            scene_future.cancel()
            try:
                active_scene_client.remove_pending_request(scene_future)
            except Exception:
                pass
            return False, str(error), "planning_scene_timeout", True
        if response is None or not response.success:
            return (
                False,
                "MoveIt rejected the planning scene update.",
                "planning_scene_rejected",
                True,
            )

        return (
            True,
            f"Applied OctoMap environment version {environment.version} "
            f"({octomap_size} byte payload). {conservative_message}",
            "",
            False,
        )

    def _request_value(self, value, parameter_name):
        if value > 0:
            return value
        return self.get_parameter(parameter_name).value

    @staticmethod
    def _make_joint_constraints(joint_state):
        if not joint_state.name or len(joint_state.name) != len(joint_state.position):
            raise ValueError("Joint goal names and positions must have equal lengths.")

        constraints = Constraints()
        constraints.name = "joint_goal"
        for name, position in zip(joint_state.name, joint_state.position):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return constraints

    @staticmethod
    def _make_pose_constraints(
        goal_pose, link_name, position_tolerance, orientation_tolerance
    ):
        frame_id = goal_pose.header.frame_id or "world"

        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [position_tolerance]
        region_pose = Pose()
        region_pose.position = goal_pose.pose.position
        region_pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = link_name
        position.constraint_region.primitives.append(region)
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame_id
        orientation.link_name = link_name
        orientation.orientation = goal_pose.pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = "pose_goal"
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _make_move_group_goal(self, request):
        goal = MoveGroup.Goal()
        motion_request = goal.request
        motion_request.group_name = (
            request.planning_group
            or self.get_parameter("default_planning_group").value
        )
        bounds = list(self.get_parameter("planning_bounds").value)
        if len(bounds) == 6:
            workspace = motion_request.workspace_parameters
            workspace.header.frame_id = "world"
            workspace.min_corner.x, workspace.max_corner.x = bounds[0], bounds[1]
            workspace.min_corner.y, workspace.max_corner.y = bounds[2], bounds[3]
            workspace.min_corner.z, workspace.max_corner.z = bounds[4], bounds[5]
        motion_request.num_planning_attempts = int(
            self._request_value(
                request.num_planning_attempts, "default_planning_attempts"
            )
        )
        motion_request.allowed_planning_time = float(
            self._request_value(
                request.allowed_planning_time, "default_planning_time"
            )
        )
        motion_request.max_velocity_scaling_factor = float(
            self._request_value(
                request.velocity_scaling, "default_velocity_scaling"
            )
        )
        motion_request.max_acceleration_scaling_factor = float(
            self._request_value(
                request.acceleration_scaling, "default_acceleration_scaling"
            )
        )

        if request.use_current_start_state:
            motion_request.start_state.is_diff = True
        else:
            if not request.start_state.name:
                raise ValueError("An explicit start state requires joint names.")
            motion_request.start_state.joint_state = request.start_state

        if request.goal_type == PlanMotion.Goal.JOINT_GOAL:
            constraints = self._make_joint_constraints(request.joint_goal)
        elif request.goal_type == PlanMotion.Goal.POSE_GOAL:
            link_name = (
                request.end_effector_link
                or self.get_parameter("default_end_effector_link").value
            )
            position_tolerance = request.position_tolerance or 0.01
            orientation_tolerance = request.orientation_tolerance or 0.05
            constraints = self._make_pose_constraints(
                request.pose_goal,
                link_name,
                position_tolerance,
                orientation_tolerance,
            )
        else:
            raise ValueError(f"Unsupported goal type {request.goal_type}.")

        motion_request.goal_constraints.append(constraints)
        # Normal tasks are deliberately separated into plan, validate, and
        # execute. Bootstrap scans are the only exception: they run against an
        # intentionally empty map and retain MoveGroup's stable combined
        # Plan+Execute path to establish the first known-free volume.
        direct_bootstrap_execution = (
            self._known_free_exempt(request.task_id) and not request.plan_only
        )
        goal.planning_options.plan_only = not direct_bootstrap_execution
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.replan = False
        return goal

    def _known_free_exempt(self, task_id):
        prefixes = self.get_parameter("known_free_exempt_task_prefixes").value
        return any(task_id.startswith(prefix) for prefix in prefixes)

    async def _validate_known_free(self, request, trajectory, planning_group):
        timeout = float(
            self.get_parameter("known_free_validation_timeout_sec").value
        )
        if not self._known_free_client.wait_for_service(timeout_sec=timeout):
            return False, "Known-free trajectory validator is unavailable."

        validation = ValidateKnownFreeTrajectory.Request()
        validation.environment_version = request.environment_version
        validation.planning_group = planning_group
        validation.trajectory = trajectory
        validation.interpolation_step = float(
            self.get_parameter("known_free_interpolation_step").value
        )
        validation.link_sample_step = float(
            self.get_parameter("known_free_link_sample_step").value
        )
        validation.clearance = float(
            self.get_parameter("known_free_clearance").value
        )
        future = self._known_free_client.call_async(validation)
        try:
            response = await self._await_with_timeout(
                future, timeout, "Known-free trajectory validation"
            )
        except TimeoutError as error:
            future.cancel()
            return False, str(error)
        if response is None:
            return False, "Known-free trajectory validator returned no response."
        details = (
            f"{response.reason} checked={response.checked_samples}, "
            f"unknown={response.unknown_samples}, "
            f"occupied={response.occupied_samples}."
        )
        return bool(response.valid), details

    async def _execute_validated_trajectory(self, goal_handle, trajectory):
        timeout = float(self.get_parameter("execute_result_timeout_sec").value)
        if not self._execute_trajectory_client.wait_for_server(timeout_sec=timeout):
            return False, 0, "/execute_trajectory is unavailable."

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory
        send_future = self._execute_trajectory_client.send_goal_async(execute_goal)
        try:
            self._active_move_group_goal = await self._await_with_timeout(
                send_future, timeout, "ExecuteTrajectory goal response"
            )
        except TimeoutError as error:
            send_future.cancel()
            return False, MoveItErrorCodes.TIMED_OUT, str(error)
        if not self._active_move_group_goal.accepted:
            return False, 0, "MoveIt rejected the validated trajectory."

        self._feedback(goal_handle, "executing_validated_trajectory")
        try:
            wrapped = await self._await_with_timeout(
                self._active_move_group_goal.get_result_async(),
                timeout,
                "ExecuteTrajectory result",
            )
        except TimeoutError as error:
            await self._cancel_move_group_goal()
            return False, MoveItErrorCodes.TIMED_OUT, str(error)
        error_code = wrapped.result.error_code.val
        return (
            error_code == MoveItErrorCodes.SUCCESS,
            error_code,
            "Validated trajectory execution completed."
            if error_code == MoveItErrorCodes.SUCCESS
            else f"Validated trajectory execution failed with error {error_code}.",
        )

    async def _execute_callback(self, goal_handle):
        request = goal_handle.request
        result = PlanMotion.Result()

        try:
            planning_group = (
                request.planning_group
                or self.get_parameter("default_planning_group").value
            )

            # ★ NON-CONSERVATIVE FALLBACK FIX: capture ONE environment snapshot
            #   for this whole task. Both the conservative pass and the
            #   non-conservative fallback pass plan against THIS exact map, so
            #   OctoMap drift during the 1st plan can no longer version-mismatch
            #   the 2nd pass. Version is validated once here (was previously
            #   done inside _apply_environment on a re-fetched snapshot).
            with self._environment_lock:
                task_environment = deepcopy(self._latest_environment)
            if task_environment is None:
                self._fail(result, "No known environment has been received.",
                           "environment_unavailable", True)
                goal_handle.abort()
                return result
            if task_environment.version != request.environment_version:
                self._fail(
                    result,
                    f"Required environment version {request.environment_version} "
                    f"does not match cached version {task_environment.version}.",
                    "environment_version_mismatch", True)
                goal_handle.abort()
                return result

            # ★ MULTI-PLANNER POOL: for plan_only requests, round-robin one
            #   of the probe planners; for real execute requests, always use
            #   the master (has controller access).  All follow-up calls
            #   (apply_scene, move_group send) use the same pair.
            use_probe_pool = (
                request.plan_only
                and bool(self._probe_move_group_clients)
                and not self._known_free_exempt(request.task_id)
            )
            active_mg_client = self._move_group_client
            active_scene_client = None  # None => master
            active_planner_label = "master"
            if use_probe_pool:
                mg, sc, label = self._pick_probe_client_pair()
                if mg is not None:
                    active_mg_client = mg
                    active_scene_client = sc
                    active_planner_label = label

            self._feedback(goal_handle, "applying_environment")
            (
                scene_ok,
                scene_message,
                scene_error_type,
                scene_retryable,
            ) = await self._apply_environment(
                request.environment_version,
                planning_group,
                allow_unknown=self._known_free_exempt(request.task_id),
                scene_client=active_scene_client,
                environment=task_environment,
            )
            if not scene_ok:
                self._fail(
                    result,
                    scene_message,
                    scene_error_type,
                    scene_retryable,
                )
                goal_handle.abort()
                return result
            result.environment_applied = True
            result.applied_environment_version = request.environment_version

            if goal_handle.is_cancel_requested:
                self._fail(result, "Task canceled before planning.", "canceled", False)
                goal_handle.canceled()
                return result

            try:
                move_group_goal = self._make_move_group_goal(request)
            except ValueError as error:
                self._fail(result, str(error), "invalid_request", False)
                goal_handle.abort()
                return result

            self._feedback(goal_handle, "waiting_for_move_group")
            server_timeout = float(
                self.get_parameter("move_group_server_timeout_sec").value
            )
            if not active_mg_client.wait_for_server(
                timeout_sec=server_timeout
            ):
                self._fail(
                    result,
                    f"move_group action unavailable ({active_planner_label}).",
                    "move_group_unavailable",
                    True,
                )
                goal_handle.abort()
                return result
            if use_probe_pool:
                self.get_logger().debug(
                    f"routing plan_only task '{request.task_id}' to "
                    f"probe planner '{active_planner_label}'"
                )

            self._feedback(
                goal_handle,
                "planning_and_executing_bootstrap"
                if self._known_free_exempt(request.task_id) and not request.plan_only
                else "planning_for_known_free_validation",
            )
            send_future = active_mg_client.send_goal_async(
                move_group_goal,
                feedback_callback=lambda feedback: self._feedback(
                    goal_handle, feedback.feedback.state
                ),
            )
            goal_response_timeout = float(
                self.get_parameter("move_group_goal_response_timeout_sec").value
            )
            try:
                self._active_move_group_goal = await self._await_with_timeout(
                    send_future,
                    goal_response_timeout,
                    "MoveGroup goal response",
                )
            except TimeoutError as error:
                send_future.cancel()
                self._fail(
                    result,
                    str(error),
                    "move_group_goal_timeout",
                    True,
                    MoveItErrorCodes.TIMED_OUT,
                )
                goal_handle.abort()
                return result
            if not self._active_move_group_goal.accepted:
                self._fail(
                    result,
                    "MoveIt rejected the motion request.",
                    "move_group_rejected",
                    True,
                )
                goal_handle.abort()
                return result

            result_timeout = float(
                self.get_parameter("move_group_result_timeout_sec").value
            )
            try:
                wrapped_result = await self._await_with_timeout(
                    self._active_move_group_goal.get_result_async(),
                    result_timeout,
                    "MoveGroup result",
                )
            except TimeoutError as error:
                await self._cancel_move_group_goal()
                self._fail(
                    result,
                    str(error),
                    "move_group_result_timeout",
                    True,
                    MoveItErrorCodes.TIMED_OUT,
                )
                goal_handle.abort()
                return result
            move_group_result = wrapped_result.result
            result.moveit_error_code = move_group_result.error_code.val
            result.planning_time = move_group_result.planning_time
            result.planned_trajectory = move_group_result.planned_trajectory
            trajectory_points = result.planned_trajectory.joint_trajectory.points
            if trajectory_points:
                duration = trajectory_points[-1].time_from_start
                result.estimated_execution_time = (
                    float(duration.sec) + float(duration.nanosec) / 1e9
                )
            else:
                result.estimated_execution_time = 0.0

            plan_succeeded = (
                move_group_result.error_code.val == MoveItErrorCodes.SUCCESS
            )
            if not plan_succeeded:
                # Conservative-only planning: no non-conservative fallback.
                # A failed conservative plan is reported directly as a planning
                # failure.  retryable=True because the conservative map grows as
                # the arm keeps scanning, so a goal that cannot be planned this
                # cycle may become plannable in a later one.
                conservative_code = move_group_result.error_code.val
                result.success = False
                result.retryable = True
                result.error_type = "moveit_planning_failure"
                result.moveit_error_code = conservative_code
                result.message = (
                    f"MoveIt failed to plan task '{request.task_id}' with error "
                    f"{conservative_code}."
                )
            else:
                direct_bootstrap_execution = (
                    self._known_free_exempt(request.task_id)
                    and not request.plan_only
                )
                if direct_bootstrap_execution:
                    result.success = True
                    result.retryable = False
                    result.error_type = ""
                    result.message = (
                        f"{scene_message} MoveIt completed bootstrap task "
                        f"'{request.task_id}' using combined Plan+Execute; "
                        "known-free validation was explicitly bypassed."
                    )
                    validation_ok = None
                else:
                    validation_ok = False
                if direct_bootstrap_execution:
                    pass
                elif self._known_free_exempt(request.task_id):
                    validation_message = (
                        "Known-free validation bypassed for explicit bootstrap "
                        f"task prefix: '{request.task_id}'."
                    )
                    self.get_logger().warning(validation_message)
                    validation_ok = True
                else:
                    self._feedback(goal_handle, "validating_known_free_trajectory")
                    validation_ok, validation_message = (
                        await self._validate_known_free(
                            request, result.planned_trajectory, planning_group
                        )
                    )

                if direct_bootstrap_execution:
                    pass
                elif not validation_ok:
                    result.success = False
                    result.retryable = True
                    result.error_type = (
                        "validation_environment_mismatch"
                        if "environment version" in validation_message.lower()
                        else "trajectory_not_known_free"
                    )
                    result.message = (
                        "Trajectory rejected before execution: "
                        f"{validation_message}"
                    )
                    self.get_logger().warning(result.message)
                elif request.plan_only:
                    result.success = True
                    result.retryable = False
                    result.error_type = ""
                    result.message = (
                        f"{scene_message} MoveIt planned and validated task "
                        f"'{request.task_id}'. {validation_message}"
                    )
                else:
                    execution_ok, execution_code, execution_message = (
                        await self._execute_validated_trajectory(
                            goal_handle, result.planned_trajectory
                        )
                    )
                    result.success = execution_ok
                    result.moveit_error_code = execution_code
                    result.retryable = not execution_ok
                    result.error_type = "" if execution_ok else "execution_failure"
                    result.message = (
                        f"{scene_message} {validation_message} {execution_message}"
                    )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as error:  # Keep the action server alive on integration errors.
            self.get_logger().error(f"Task '{request.task_id}' failed: {error}")
            self._fail(result, str(error), "internal_error", True)
            goal_handle.abort()
            return result
        finally:
            self._active_move_group_goal = None
            # ★ PLAN-ONLY CONCURRENCY: release the correct slot based on
            #   whether this task was plan_only or an execute.
            is_plan_only = bool(request.plan_only)
            with self._task_lock:
                if is_plan_only:
                    if self._active_plan_only > 0:
                        self._active_plan_only -= 1
                else:
                    self._execute_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
