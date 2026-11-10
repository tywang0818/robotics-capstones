"""ROS 2 node for online semantic RGB-D inference."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from semantic_frontier.ros.online import (
    OnlineSemanticSegmenter,
    SemanticPoint,
    project_semantic_raster_to_points,
)


def require_ros_modules():
    """Import ROS modules only when the node actually starts."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        from sensor_msgs.msg import Image as RosImage
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header, String
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python modules are not available. Source ROS 2 Jazzy and the "
            "semantic_frontier_ros workspace before running this node."
        ) from exc
    return (
        rclpy,
        Node,
        qos_profile_sensor_data,
        CameraInfo,
        RosImage,
        PointCloud2,
        PointField,
        point_cloud2,
        Header,
        String,
    )


def stamp_seconds(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def image_message_to_array(message: Any) -> np.ndarray:
    """Decode the ROS image encodings accepted by the online node."""
    encoding = message.encoding.lower()
    if encoding in {"rgb8", "bgr8"}:
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
        if encoding == "bgr8":
            array = array[..., ::-1]
        return array.copy()
    if encoding in {"mono8", "8uc1"}:
        return np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width
        ).copy()
    if encoding == "16uc1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, message.width
        ).copy()
    if encoding == "32fc1":
        return np.frombuffer(message.data, dtype=np.float32).reshape(
            message.height, message.width
        ).copy()
    raise ValueError(f"Unsupported image encoding: {message.encoding}")


def depth_message_to_meters(message: Any) -> np.ndarray:
    depth = image_message_to_array(message)
    encoding = message.encoding.lower()
    if encoding == "32fc1":
        return depth.astype(np.float32, copy=False)
    if encoding == "16uc1":
        return depth.astype(np.float32) / 1000.0
    raise ValueError(f"Unsupported depth encoding: {message.encoding}")


class LatestMessages:
    def __init__(self) -> None:
        self.rgb: Any | None = None
        self.depth: Any | None = None
        self.camera_info: Any | None = None


def points_to_tuples(points: list[SemanticPoint]) -> list[tuple[float, float, float, int, float, float]]:
    return [
        (
            point.x,
            point.y,
            point.z,
            point.class_id,
            point.class_prob,
            point.relevance_prob,
        )
        for point in points
    ]


def point_fields(PointField):
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="class_id", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="class_prob", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="relevance_prob", offset=20, datatype=PointField.FLOAT32, count=1),
    ]


def resolve_checkpoint(config_path: str, checkpoint_path: str) -> str | Path:
    if checkpoint_path:
        return checkpoint_path
    from semantic_frontier.segmentation.config import load_config
    from semantic_frontier.segmentation.workflow import latest_run

    if not config_path:
        raise FileNotFoundError(
            "checkpoint_path and config_path are both empty. Provide a Hugging "
            "Face model id, a local checkpoint path, or a training config."
        )
    config = load_config(Path(config_path))
    run_dir = latest_run(config)
    checkpoint = run_dir / "checkpoints" / "best"
    if not checkpoint.exists():
        raise FileNotFoundError(
            "checkpoint_path was empty and the latest run has no best checkpoint: "
            f"{checkpoint}. Provide checkpoint_path explicitly."
        )
    return checkpoint


def build_node_class():
    (
        rclpy,
        Node,
        qos_profile_sensor_data,
        CameraInfo,
        RosImage,
        PointCloud2,
        PointField,
        point_cloud2,
        Header,
        String,
    ) = require_ros_modules()

    class SemanticInferenceNode(Node):
        def __init__(self) -> None:
            super().__init__("semantic_inference_node")
            self.latest = LatestMessages()
            self.last_processed_stamp: float | None = None
            self.last_status_time = 0.0

            self.declare_parameter("config_path", "")
            self.declare_parameter(
                "checkpoint_path",
                "nvidia/segformer-b0-finetuned-ade-512-512",
            )
            self.declare_parameter("image_size", 512)
            self.declare_parameter("label_space", "ade150")
            self.declare_parameter("device", "auto")
            self.declare_parameter("rgb_topic", "/camera/image_rel")
            self.declare_parameter("depth_topic", "/camera/depth_image")
            self.declare_parameter("camera_info_topic", "/camera/camera_info")
            self.declare_parameter("semantic_points_topic", "/semantic_frontier/semantic_points")
            self.declare_parameter("relevance_heatmap_topic", "/semantic_frontier/relevance_heatmap")
            self.declare_parameter("output_frame", "")
            self.declare_parameter("sync_tolerance_ms", 100.0)
            self.declare_parameter("max_depth_m", 5.0)
            self.declare_parameter("stride", 4)
            self.declare_parameter("min_relevance_prob", 0.0)
            self.declare_parameter("publish_heatmap", True)

            checkpoint = resolve_checkpoint(
                str(self.get_parameter("config_path").value),
                str(self.get_parameter("checkpoint_path").value),
            )
            image_size = int(self.get_parameter("image_size").value)
            label_space = str(self.get_parameter("label_space").value)
            device = str(self.get_parameter("device").value)
            self.segmenter = OnlineSemanticSegmenter(
                checkpoint,
                image_size=image_size,
                label_space=label_space,
                device=device,
            )

            self.sync_tolerance_s = float(self.get_parameter("sync_tolerance_ms").value) / 1000.0
            self.max_depth_m = float(self.get_parameter("max_depth_m").value)
            self.stride = int(self.get_parameter("stride").value)
            self.min_relevance_prob = float(self.get_parameter("min_relevance_prob").value)
            self.publish_heatmap = bool(self.get_parameter("publish_heatmap").value)
            self.output_frame = str(self.get_parameter("output_frame").value)

            self.points_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter("semantic_points_topic").value),
                10,
            )
            self.heatmap_publisher = self.create_publisher(
                RosImage,
                str(self.get_parameter("relevance_heatmap_topic").value),
                10,
            )
            self.status_publisher = self.create_publisher(
                String,
                "/semantic_frontier/status",
                10,
            )

            self.create_subscription(
                RosImage,
                str(self.get_parameter("rgb_topic").value),
                self._set_rgb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                RosImage,
                str(self.get_parameter("depth_topic").value),
                self._set_depth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._set_camera_info,
                qos_profile_sensor_data,
            )
            self.create_timer(0.02, self._try_process)
            self.get_logger().info(
                f"Loaded semantic checkpoint={checkpoint} label_space={label_space} "
                f"image_size={image_size} stride={self.stride}"
            )

        def _set_rgb(self, message: Any) -> None:
            self.latest.rgb = message

        def _set_depth(self, message: Any) -> None:
            self.latest.depth = message

        def _set_camera_info(self, message: Any) -> None:
            self.latest.camera_info = message

        def _messages_ready(self) -> bool:
            messages = {
                "rgb": self.latest.rgb,
                "depth": self.latest.depth,
                "camera_info": self.latest.camera_info,
            }
            if any(message is None for message in messages.values()):
                return False
            stamps = [stamp_seconds(message) for message in messages.values()]
            return max(stamps) - min(stamps) <= self.sync_tolerance_s

        def _header(self, rgb_message: Any) -> Any:
            header = Header()
            header.stamp = rgb_message.header.stamp
            header.frame_id = self.output_frame or rgb_message.header.frame_id
            return header

        def _publish_heatmap(self, header: Any, relevance: np.ndarray) -> None:
            image = RosImage()
            image.header = header
            image.height = int(relevance.shape[0])
            image.width = int(relevance.shape[1])
            image.encoding = "mono8"
            image.is_bigendian = False
            image.step = image.width
            image.data = np.clip(relevance * 255.0, 0, 255).astype(np.uint8).tobytes()
            self.heatmap_publisher.publish(image)

        def _publish_status(self, latency_ms: float, point_count: int) -> None:
            now = time.monotonic()
            if now - self.last_status_time < 1.0:
                return
            self.last_status_time = now
            message = String()
            message.data = json.dumps(
                {
                    "latency_ms": latency_ms,
                    "point_count": point_count,
                    "stride": self.stride,
                    "max_depth_m": self.max_depth_m,
                    "frame": self.output_frame or "camera",
                },
                sort_keys=True,
            )
            self.status_publisher.publish(message)
            self.get_logger().info(message.data)

        def _try_process(self) -> None:
            if not self._messages_ready():
                return
            assert self.latest.rgb is not None
            stamp = stamp_seconds(self.latest.rgb)
            if self.last_processed_stamp == stamp:
                return
            self.last_processed_stamp = stamp

            assert self.latest.depth is not None
            assert self.latest.camera_info is not None
            started = time.perf_counter()
            try:
                rgb = image_message_to_array(self.latest.rgb)
                depth = depth_message_to_meters(self.latest.depth)
                raster = self.segmenter.predict(rgb, depth.shape)
                points = project_semantic_raster_to_points(
                    depth,
                    list(self.latest.camera_info.k),
                    raster,
                    stride=self.stride,
                    max_depth_m=self.max_depth_m,
                    min_relevance_prob=self.min_relevance_prob,
                )
            except Exception as exc:
                self.get_logger().error(f"Semantic inference failed: {exc}")
                return

            header = self._header(self.latest.rgb)
            cloud = point_cloud2.create_cloud(
                header,
                point_fields(PointField),
                points_to_tuples(points),
            )
            self.points_publisher.publish(cloud)
            if self.publish_heatmap:
                self._publish_heatmap(header, raster.relevance_prob)
            latency_ms = (time.perf_counter() - started) * 1000.0
            self._publish_status(latency_ms, len(points))

    return rclpy, SemanticInferenceNode


def main(args=None) -> None:
    rclpy, SemanticInferenceNode = build_node_class()
    rclpy.init(args=args)
    node = SemanticInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
