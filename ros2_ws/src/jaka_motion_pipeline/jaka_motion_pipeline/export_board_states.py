#!/usr/bin/env python3

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from jaka_motion_pipeline.example_board_sequence import (
    ABOVE_BOARD_CANDIDATES,
    BELOW_BOARD_CANDIDATES,
)
from jaka_motion_pipeline.example_viewpoint_task import view_quaternion
from jaka_planning_interfaces.action import PlanMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class BoardStateExporter(Node):
    def __init__(self):
        super().__init__("export_board_states")
        self.declare_parameter(
            "output_path", str(Path.home() / ".ros" / "jaka_a5_board_states.srdf")
        )
        self.client = ActionClient(self, PlanMotion, "/plan_motion")
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/board_state_targets", marker_qos
        )

    @staticmethod
    def make_marker(marker_id, name, position, is_above):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "board_state_targets"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.a = 0.9
        marker.color.r = 0.1 if is_above else 0.95
        marker.color.g = 0.75 if is_above else 0.25
        marker.color.b = 0.95 if is_above else 0.1
        marker.text = name
        return marker

    def publish_target_markers(self):
        markers = MarkerArray()
        candidates = [
            ("above", ABOVE_BOARD_CANDIDATES),
            ("below", BELOW_BOARD_CANDIDATES),
        ]
        marker_id = 0
        for state_type, state_candidates in candidates:
            for index, (_, position, _) in enumerate(state_candidates, 1):
                name = f"board_{state_type}_{index}"
                markers.markers.append(
                    self.make_marker(
                        marker_id,
                        name,
                        position,
                        is_above=state_type == "above",
                    )
                )
                marker_id += 1
        self.marker_publisher.publish(markers)

    def make_goal(self, task_id, position, look_at):
        quaternion = view_quaternion(position, look_at)
        goal = PlanMotion.Goal()
        goal.task_id = task_id
        goal.environment_version = 5
        goal.planning_group = "jaka_a5"
        goal.end_effector_link = "tool0"
        goal.use_current_start_state = True
        goal.goal_type = PlanMotion.Goal.POSE_GOAL
        goal.pose_goal.header.frame_id = "world"
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
        goal.orientation_tolerance = 3.14
        goal.plan_only = True
        goal.allowed_planning_time = 8.0
        goal.num_planning_attempts = 30
        goal.velocity_scaling = 0.15
        goal.acceleration_scaling = 0.15
        return goal

    def plan_state(self, name, position, look_at):
        self.get_logger().info(f"Computing named state '{name}'.")
        send_future = self.client.send_goal_async(
            self.make_goal(name, position, look_at)
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Task '{name}' was rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        trajectory = result.planned_trajectory.joint_trajectory
        if not result.success or not trajectory.points:
            self.get_logger().error(
                f"Cannot create '{name}': MoveIt error {result.moveit_error_code}."
            )
            return None

        positions = trajectory.points[-1].positions
        state = dict(zip(trajectory.joint_names, positions))
        self.get_logger().info(f"Created named state '{name}'.")
        return state

    @staticmethod
    def group_state_xml(name, joints):
        lines = [f'  <group_state name="{name}" group="jaka_a5">']
        for joint_name in sorted(joints):
            lines.append(
                f'    <joint name="{joint_name}" value="{joints[joint_name]:.8f}"/>'
            )
        lines.append("  </group_state>")
        return "\n".join(lines)

    def export(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/plan_motion is unavailable.")
            return False

        self.publish_target_markers()
        states = {}
        for prefix, candidates in [
            ("board_above", ABOVE_BOARD_CANDIDATES),
            ("board_below", BELOW_BOARD_CANDIDATES),
        ]:
            for index, (_, position, look_at) in enumerate(candidates, 1):
                name = f"{prefix}_{index}"
                state = self.plan_state(name, position, look_at)
                if state is not None:
                    states[name] = state

        if not states:
            self.get_logger().error(
                "No valid named states were generated. Target markers were published "
                "on /board_state_targets for visual diagnosis."
            )
            return False

        base_srdf_path = (
            Path(get_package_share_directory("jaka_a5_moveit_config"))
            / "config"
            / "jaka_a5.srdf"
        )
        base_srdf = base_srdf_path.read_text()
        state_xml = "\n\n".join(
            self.group_state_xml(name, joints) for name, joints in states.items()
        )
        generated_srdf = base_srdf.replace("</robot>", f"\n{state_xml}\n</robot>")

        output_path = Path(self.get_parameter("output_path").value).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(generated_srdf)
        self.get_logger().info(
            f"Wrote {len(states)} named state(s) to {output_path}."
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BoardStateExporter()
    try:
        node.export()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
