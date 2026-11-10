#!/usr/bin/env python3
"""Relay a sensor_msgs/Image from a BEST_EFFORT publisher to a RELIABLE
topic so that tools like rqt_image_view (which subscribes RELIABLE by
default and cannot pick QoS from the UI) actually receive frames.

Default: /camera/image (BE, published by ros_gz_bridge)
      -> /camera/image_rel (R, subscribed by rqt_image_view)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class ImageRelay(Node):
    def __init__(self):
        super().__init__("image_relay")
        self.declare_parameter("input_topic", "/camera/image")
        self.declare_parameter("output_topic", "/camera/image_rel")
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(Image, out_topic, pub_qos)
        self.create_subscription(Image, in_topic,
                                 lambda m: self.pub.publish(m), sub_qos)
        self.get_logger().info(
            f"image_relay: {in_topic} (BEST_EFFORT) -> {out_topic} (RELIABLE)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
