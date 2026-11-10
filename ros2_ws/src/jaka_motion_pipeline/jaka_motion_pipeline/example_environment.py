#!/usr/bin/env python3

import rclpy
from jaka_planning_interfaces.msg import EnvironmentModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ExampleEnvironmentPublisher(Node):
    def __init__(self):
        super().__init__("example_environment")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            EnvironmentModel, "/known_environment", qos
        )
        self.publish_environment()

    def publish_environment(self):
        message = EnvironmentModel()
        now = self.get_clock().now().to_msg()
        message.header.stamp = now
        message.header.frame_id = "world"
        message.version = 5
        message.replace = True
        message.octomap.header.stamp = now
        message.octomap.header.frame_id = "world"
        message.octomap.origin.orientation.w = 1.0
        message.octomap.octomap.header.stamp = now
        message.octomap.octomap.header.frame_id = "world"
        message.octomap.octomap.binary = True
        message.octomap.octomap.id = "OcTree"
        message.octomap.octomap.resolution = 0.05
        message.octomap.octomap.data = []
        self.publisher.publish(message)
        self.get_logger().info("Published example OctoMap environment version 5.")


def main(args=None):
    rclpy.init(args=args)
    node = ExampleEnvironmentPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
