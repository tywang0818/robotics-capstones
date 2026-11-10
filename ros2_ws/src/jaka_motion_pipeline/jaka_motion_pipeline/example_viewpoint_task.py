#!/usr/bin/env python3

from math import sqrt

import rclpy
from jaka_planning_interfaces.action import PlanMotion
from rclpy.action import ActionClient
from rclpy.node import Node


def subtract(left, right):
    return [left[index] - right[index] for index in range(3)]


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def normalize(vector):
    length = sqrt(sum(value * value for value in vector))
    if length < 1e-9:
        raise ValueError("Viewpoint position and look-at point must differ.")
    return [value / length for value in vector]


def quaternion_from_rotation_matrix(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        return [
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        ]
    if m00 > m11 and m00 > m22:
        scale = sqrt(1.0 + m00 - m11 - m22) * 2.0
        return [
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        ]
    if m11 > m22:
        scale = sqrt(1.0 + m11 - m00 - m22) * 2.0
        return [
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        ]

    scale = sqrt(1.0 + m22 - m00 - m11) * 2.0
    return [
        (m02 + m20) / scale,
        (m12 + m21) / scale,
        0.25 * scale,
        (m10 - m01) / scale,
    ]


def view_quaternion(position, look_at):
    # Assumption: the controlled camera/tool frame looks forward along local +Z.
    forward = normalize(subtract(look_at, position))
    reference_up = [0.0, 0.0, 1.0]
    if abs(sum(a * b for a, b in zip(forward, reference_up))) > 0.98:
        reference_up = [0.0, 1.0, 0.0]

    right = normalize(cross(reference_up, forward))
    up = cross(forward, right)
    rotation = [
        [right[0], up[0], forward[0]],
        [right[1], up[1], forward[1]],
        [right[2], up[2], forward[2]],
    ]
    return quaternion_from_rotation_matrix(rotation)


class ExampleViewpointTaskClient(Node):
    def __init__(self):
        super().__init__("example_viewpoint_task")
        self.declare_parameter("position", [0.45, 0.0, 0.45])
        self.declare_parameter("look_at", [0.45, 0.0, 0.0])
        self.declare_parameter("end_effector_link", "tool0")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("environment_version", 5)
        self.client = ActionClient(self, PlanMotion, "/plan_motion")

    def send_task(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/plan_motion is unavailable.")
            return False

        position = list(self.get_parameter("position").value)
        look_at = list(self.get_parameter("look_at").value)
        quaternion = view_quaternion(position, look_at)

        goal = PlanMotion.Goal()
        goal.task_id = "example_viewpoint"
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
        goal.orientation_tolerance = 0.10
        goal.plan_only = self.get_parameter("plan_only").value
        goal.allowed_planning_time = 10.0
        goal.num_planning_attempts = 20
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
            self.get_logger().error("Viewpoint task was rejected.")
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
    node = ExampleViewpointTaskClient()
    try:
        node.send_task()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
