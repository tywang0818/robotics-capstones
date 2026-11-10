#!/usr/bin/env python3

from time import sleep

import rclpy
from jaka_motion_pipeline.example_viewpoint_task import view_quaternion
from jaka_planning_interfaces.action import PlanMotion
from rclpy.action import ActionClient
from rclpy.node import Node


BOARD_CENTER = [0.35, 0.0, 0.72]
ABOVE_BOARD_CANDIDATES = [
    ("above_board", [0.35, 0.0, 0.86], BOARD_CENTER),
    ("above_board", [0.25, 0.0, 0.84], BOARD_CENTER),
    ("above_board", [0.45, 0.0, 0.84], BOARD_CENTER),
]

# Directly reaching the center under a horizontal board can be impossible
# because the arm links also need to fit below it. Try lower positions near
# different board edges and execute the first reachable candidate.
BELOW_BOARD_CANDIDATES = [
    ("below_board", [0.35, -0.32, 0.57], [0.35, -0.32, 0.30]),
    ("below_board", [0.35, 0.32, 0.57], [0.35, 0.32, 0.30]),
    ("below_board", [0.58, 0.0, 0.57], [0.58, 0.0, 0.30]),
]


class ExampleBoardSequenceClient(Node):
    def __init__(self):
        super().__init__("example_board_sequence")
        self.declare_parameter("pause_seconds", 2.0)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("end_effector_link", "tool0")
        self.declare_parameter("environment_version", 5)
        self.client = ActionClient(self, PlanMotion, "/plan_motion")

    def make_goal(self, task_id, position, look_at, plan_only=False):
        quaternion = view_quaternion(position, look_at)

        goal = PlanMotion.Goal()
        goal.task_id = task_id
        goal.environment_version = self.get_parameter("environment_version").value
        goal.planning_group = "jaka_a5"
        goal.end_effector_link = self.get_parameter("end_effector_link").value
        goal.use_current_start_state = True
        goal.goal_type = PlanMotion.Goal.POSE_GOAL
        goal.pose_goal.header.frame_id = "world"
        goal.pose_goal.header.stamp = self.get_clock().now().to_msg()
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
        ) = quaternion
        goal.position_tolerance = 0.02
        # This test focuses on position planning around the board. Camera-facing
        # constraints will be tightened after the simulated camera frame exists.
        goal.orientation_tolerance = 3.14
        goal.plan_only = plan_only
        goal.allowed_planning_time = 5.0 if plan_only else 15.0
        goal.num_planning_attempts = 30
        goal.velocity_scaling = self.get_parameter("velocity_scaling").value
        goal.acceleration_scaling = 0.15
        return goal

    def send_goal(self, task_id, position, look_at, plan_only=False):
        mode = "Checking" if plan_only else "Sending"
        self.get_logger().info(
            f"{mode} '{task_id}' target at "
            f"x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}."
        )
        send_future = self.client.send_goal_async(
            self.make_goal(task_id, position, look_at, plan_only),
            feedback_callback=lambda feedback: self.get_logger().info(
                f"{task_id}: {feedback.feedback.state}"
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Task '{task_id}' was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        message = (
            f"{result.message} error_code={result.moveit_error_code}, "
            f"planning_time={result.planning_time:.3f}s, "
            f"estimated_execution_time="
            f"{result.estimated_execution_time:.3f}s"
        )
        if result.success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return result.success

    def find_reachable_state(self, state_name, candidates):
        self.get_logger().info(f"Searching for a reachable {state_name} state.")
        for index, (_, position, look_at) in enumerate(candidates, 1):
            probe_id = f"{state_name}_probe_{index}"
            if self.send_goal(probe_id, position, look_at, plan_only=True):
                self.get_logger().info(
                    f"Selected reachable {state_name} candidate {index}."
                )
                return state_name, position, look_at

        self.get_logger().error(
            f"No {state_name} candidate produced a valid plan."
        )
        return None

    def run_sequence(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/plan_motion is unavailable.")
            return False

        pause_seconds = self.get_parameter("pause_seconds").value
        above_state = self.find_reachable_state(
            "above_board", ABOVE_BOARD_CANDIDATES
        )
        if above_state is None:
            return False

        task_id, position, look_at = above_state
        if not self.send_goal(task_id, position, look_at):
            self.get_logger().error("Failed to reach the above-board state.")
            return False

        self.get_logger().info(
            f"Reached board upper state; waiting {pause_seconds:.1f}s."
        )
        sleep(pause_seconds)

        below_state = self.find_reachable_state(
            "below_board", BELOW_BOARD_CANDIDATES
        )
        if below_state is None:
            return False

        task_id, position, look_at = below_state
        if not self.send_goal(task_id, position, look_at):
            self.get_logger().error("Failed to execute the selected below-board state.")
            return False

        self.get_logger().info("Completed above-board to below-board sequence.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ExampleBoardSequenceClient()
    try:
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
