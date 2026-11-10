#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOPICS = {
    "environment": "/team/optimized_environment_json",
    "task": "/team/viewpoint_task_json",
}


def parse_args(args=None):
    parser = argparse.ArgumentParser(description="Publish one JSON file to a team topic.")
    parser.add_argument("kind", choices=sorted(TOPICS))
    parser.add_argument("json_file")
    return parser.parse_args(args)


def main(args=None):
    parsed = parse_args(args)
    data = json.loads(Path(parsed.json_file).read_text())
    serialized = json.dumps(data, separators=(",", ":"))

    rclpy.init(args=[])
    node = Node("publish_json_file")
    publisher = node.create_publisher(String, TOPICS[parsed.kind], 10)
    message = String()
    message.data = serialized

    deadline = monotonic() + 3.0
    while publisher.get_subscription_count() == 0 and monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    publisher.publish(message)
    node.get_logger().info(f"Published {parsed.json_file} to {TOPICS[parsed.kind]}.")
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
