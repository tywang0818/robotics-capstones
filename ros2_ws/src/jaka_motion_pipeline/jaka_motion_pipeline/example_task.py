#!/usr/bin/env python3

import rclpy
from jaka_planning_interfaces.action import PlanMotion
from rclpy.action import ActionClient
from rclpy.node import Node


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]


class ExampleTaskClient(Node):
    def __init__(self):
        super().__init__("example_task")
        self.declare_parameter("plan_only", False)
        self.declare_parameter("environment_version", 5)
        self.client = ActionClient(self, PlanMotion, "/plan_motion")

    def send_task(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/plan_motion is unavailable.")
            return False

        goal = PlanMotion.Goal()
        goal.task_id = "example_ready_to_zero"
        goal.environment_version = self.get_parameter("environment_version").value
        goal.planning_group = "jaka_a5"
        goal.end_effector_link = "tool0"
        goal.use_current_start_state = True
        goal.goal_type = PlanMotion.Goal.JOINT_GOAL
        goal.joint_goal.name = JOINT_NAMES
        goal.joint_goal.position = [0.0] * 6
        goal.plan_only = self.get_parameter("plan_only").value
        goal.allowed_planning_time = 5.0
        goal.num_planning_attempts = 10
        goal.velocity_scaling = 0.2
        goal.acceleration_scaling = 0.2

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.get_logger().info(
                f"State: {feedback.feedback.state}"
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Planning task was rejected.")
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


def main(args=None):
    rclpy.init(args=args)
    node = ExampleTaskClient()
    try:
        node.send_task()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
