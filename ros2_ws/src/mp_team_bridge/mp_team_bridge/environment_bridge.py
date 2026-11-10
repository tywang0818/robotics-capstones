#!/usr/bin/env python3

import base64
import binascii
import json

import rclpy
from jaka_planning_interfaces.msg import EnvironmentModel
from mp_team_bridge.conversions import (
    normalize_quaternion,
    require_bool,
    require_dict,
    require_integer,
    require_number,
    require_string,
    require_vector,
)
from octomap_msgs.msg import OctomapWithPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class EnvironmentBridge(Node):
    def __init__(self):
        super().__init__("environment_bridge")
        self.declare_parameter("input_topic", "/team/optimized_environment_json")
        self.declare_parameter("output_topic", "/known_environment")
        self.declare_parameter("status_topic", "/team/environment_status_json")

        environment_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.environment_publisher = self.create_publisher(
            EnvironmentModel,
            self.get_parameter("output_topic").value,
            environment_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            environment_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self.environment_callback,
            10,
        )
        self.latest_version = -1
        self.get_logger().info(
            f"Environment bridge ready on {self.get_parameter('input_topic').value}."
        )

    def publish_status(self, accepted, version=None, message="",
                       payload_bytes=0):
        # payload_bytes = size of the OctoMap that was accepted.  Downstream
        # (HPP-PT) uses this to skip cycles until a *non-empty* environment
        # has been published, so it doesn't fight with initial_scan for
        # motion_planner attention on a 0-byte handshake env.
        status = {
            "accepted": accepted,
            "version": version,
            "message": message,
            "payload_bytes": payload_bytes,
        }
        output = String()
        output.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(output)

    def convert_octomap(self, data, frame_id):
        data = require_dict(data, "octomap")
        octomap_id = require_string(data.get("id", "OcTree"), "octomap.id")
        resolution = require_number(
            data.get("resolution"), "octomap.resolution", minimum=0.000001
        )
        binary = require_bool(data.get("binary", True), "octomap.binary")
        payload = self.convert_octomap_data(data)

        origin_data = require_dict(data.get("origin", {}), "octomap.origin")
        position = require_vector(
            origin_data.get("position", [0.0, 0.0, 0.0]),
            "octomap.origin.position",
            3,
        )
        orientation = normalize_quaternion(
            origin_data.get("orientation", [0.0, 0.0, 0.0, 1.0]),
            "octomap.origin.orientation",
        )

        message = OctomapWithPose()
        now = self.get_clock().now().to_msg()
        message.header.stamp = now
        message.header.frame_id = frame_id
        message.origin.position.x, message.origin.position.y, message.origin.position.z = (
            position
        )
        (
            message.origin.orientation.x,
            message.origin.orientation.y,
            message.origin.orientation.z,
            message.origin.orientation.w,
        ) = orientation
        message.octomap.header.stamp = now
        message.octomap.header.frame_id = frame_id
        message.octomap.binary = binary
        message.octomap.id = octomap_id
        message.octomap.resolution = resolution
        message.octomap.data = payload
        return message

    @staticmethod
    def convert_octomap_data(data):
        has_array = "data" in data
        has_base64 = "data_base64" in data
        if has_array == has_base64:
            raise ValueError(
                "'octomap' must contain exactly one of 'data' or 'data_base64'."
        )

        if has_base64:
            encoded = data.get("data_base64")
            if not isinstance(encoded, str):
                raise ValueError("'octomap.data_base64' must be a string.")
            try:
                raw = base64.b64decode(encoded, validate=True)
            except (binascii.Error, ValueError) as error:
                raise ValueError("'octomap.data_base64' is not valid base64.") from error
            return [value if value < 128 else value - 256 for value in raw]

        values = data.get("data")
        if not isinstance(values, list):
            raise ValueError("'octomap.data' must be an array of int8 values.")

        converted = []
        for index, value in enumerate(values):
            value = require_integer(value, f"octomap.data[{index}]", minimum=-128)
            if value > 255:
                raise ValueError(
                    f"'octomap.data[{index}]' must be in the int8/byte range."
                )
            converted.append(value if value < 128 else value - 256)
        return converted

    def convert_environment(self, data):
        data = require_dict(data, "root")
        version = require_integer(data.get("version"), "version", minimum=0)
        if version <= self.latest_version:
            raise ValueError(
                f"Environment version {version} must be newer than latest "
                f"version {self.latest_version}. Versions are immutable."
            )

        frame_id = require_string(data.get("frame_id", "world"), "frame_id")
        replace = require_bool(data.get("replace", True), "replace")
        if not replace:
            raise ValueError("OctoMap environment updates must set 'replace' to true.")
        if "objects" in data:
            raise ValueError(
                "Primitive 'objects' are not accepted; publish a single 'octomap'."
            )

        message = EnvironmentModel()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.version = version
        message.replace = replace
        message.octomap = self.convert_octomap(data.get("octomap"), frame_id)
        return message

    def environment_callback(self, message):
        version = None
        try:
            data = json.loads(message.data)
            if (
                isinstance(data, dict)
                and isinstance(data.get("version"), int)
                and not isinstance(data.get("version"), bool)
            ):
                version = data["version"]
            environment = self.convert_environment(data)
        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().error(f"Rejected environment: {error}")
            self.publish_status(False, version, str(error))
            return

        self.environment_publisher.publish(environment)
        self.latest_version = environment.version
        payload_bytes = len(environment.octomap.octomap.data)
        status_message = (
            f"Published environment version {environment.version} with "
            f"OctoMap payload size {payload_bytes} byte(s)."
        )
        self.get_logger().info(status_message)
        self.publish_status(
            True, environment.version, status_message,
            payload_bytes=payload_bytes,
        )


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
