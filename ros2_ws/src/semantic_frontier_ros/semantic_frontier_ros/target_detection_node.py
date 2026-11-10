"""ROS 2 node for YOLO target detection and RGB-D localization."""

from __future__ import annotations

import json
import math
import time
from typing import Any

import numpy as np

from semantic_frontier.ros.target_detection import (
    TargetDetection,
    TargetPosition,
    YoloTargetDetector,
    depth_centroid_position,
    select_target_detection,
)


def require_ros_modules():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        from sensor_msgs.msg import Image as RosImage
        from std_msgs.msg import Bool, String
        import tf2_ros
        from visualization_msgs.msg import Marker
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
        Bool,
        String,
        PoseStamped,
        Marker,
        tf2_ros,
    )


def stamp_seconds(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def image_message_to_array(message: Any) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding in {"rgb8", "bgr8"}:
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
        if encoding == "bgr8":
            array = array[..., ::-1]
        return array.copy()
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


def _rotate_vector(rotation: Any, vector: np.ndarray) -> np.ndarray:
    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("TF quaternion norm must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return matrix @ vector


def transform_position(position: TargetPosition, transform: Any) -> tuple[float, float, float]:
    vector = np.array([position.x, position.y, position.z], dtype=np.float64)
    rotated = _rotate_vector(transform.transform.rotation, vector)
    translation = transform.transform.translation
    return (
        float(rotated[0] + translation.x),
        float(rotated[1] + translation.y),
        float(rotated[2] + translation.z),
    )


def configure_pose_message(
    pose: Any,
    *,
    frame_id: str,
    stamp: Any,
    xyz: tuple[float, float, float],
) -> Any:
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.w = 1.0
    return pose


def configure_marker(
    marker: Any,
    pose: Any,
    *,
    marker_id: int,
    scale_m: float,
) -> Any:
    marker.header = pose.header
    marker.ns = "target_detection"
    marker.id = marker_id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose = pose.pose
    marker.scale.x = scale_m
    marker.scale.y = scale_m
    marker.scale.z = scale_m
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.0
    marker.color.a = 0.9
    return marker


def status_payload(
    *,
    found: bool,
    frame_id: str,
    latency_ms: float,
    position: TargetPosition | None,
    reason: str,
) -> str:
    payload: dict[str, object] = {
        "found": found,
        "frame_id": frame_id,
        "latency_ms": latency_ms,
        "reason": reason,
    }
    if position is not None:
        payload.update(
            {
                "class_name": position.class_name,
                "confidence": position.confidence,
                "valid_depth_pixels": position.valid_depth_pixels,
                "camera_xyz": [position.x, position.y, position.z],
            }
        )
    return json.dumps(payload, sort_keys=True)


def detections_payload(
    *,
    frame_id: str,
    stamp_sec: float,
    detections: list[TargetDetection],
    target_names: set[str],
    min_confidence: float,
) -> str:
    rows = []
    for detection in detections:
        if detection.confidence < min_confidence:
            continue
        rows.append(
            {
                "class_name": detection.class_name,
                "confidence": detection.confidence,
                "bbox_xyxy": list(detection.bbox_xyxy),
                "is_target": detection.class_name.strip().lower() in target_names,
            }
        )
    return json.dumps(
        {
            "frame_id": frame_id,
            "stamp": stamp_sec,
            "detections": rows,
            "count": len(rows),
        },
        sort_keys=True,
    )


def build_node_class():
    (
        rclpy,
        Node,
        qos_profile_sensor_data,
        CameraInfo,
        RosImage,
        Bool,
        String,
        PoseStamped,
        Marker,
        tf2_ros,
    ) = require_ros_modules()

    class TargetDetectionNode(Node):
        def __init__(self) -> None:
            super().__init__("target_detection_node")
            self.latest = LatestMessages()
            self.last_processed_stamp: float | None = None
            self.last_status_time = 0.0
            self.found = False
            self.last_pose: Any | None = None
            self.last_marker: Any | None = None

            self.declare_parameter("model_path", "/tmp/yolov8n.pt")
            self.declare_parameter("target_class_names", ["bottle"])
            self.declare_parameter("confidence_threshold", 0.5)
            self.declare_parameter("device", "auto")
            self.declare_parameter("rgb_topic", "/camera/image_rel")
            self.declare_parameter("depth_topic", "/camera/depth_image")
            self.declare_parameter("camera_info_topic", "/camera/camera_info")
            self.declare_parameter("found_topic", "/target/found")
            self.declare_parameter("pose_topic", "/target/pose")
            self.declare_parameter("marker_topic", "/target/marker")
            self.declare_parameter("status_topic", "/target/status")
            self.declare_parameter("detections_topic", "/target/detections")
            self.declare_parameter("output_frame", "world")
            self.declare_parameter("use_tf", True)
            self.declare_parameter("require_tf", False)
            self.declare_parameter("sync_tolerance_ms", 100.0)
            self.declare_parameter("min_depth_m", 0.05)
            self.declare_parameter("max_depth_m", 5.0)
            self.declare_parameter("min_valid_depth_pixels", 20)
            self.declare_parameter("sticky_found", True)
            self.declare_parameter("found_publish_period_s", 0.5)
            self.declare_parameter("marker_scale_m", 0.08)
            self.declare_parameter("publish_all_detections", False)
            self.declare_parameter("detections_min_confidence", 0.25)
            self.declare_parameter("detections_publish_period_s", 0.0)

            target_class_names_value = self.get_parameter("target_class_names").value
            target_class_names = (
                [target_class_names_value]
                if isinstance(target_class_names_value, str)
                else list(target_class_names_value)
            )
            self.target_names = {
                str(item).strip().lower()
                for item in target_class_names
                if str(item).strip()
            }
            self.detector = YoloTargetDetector(
                str(self.get_parameter("model_path").value),
                target_class_names,
                confidence_threshold=float(
                    self.get_parameter("confidence_threshold").value
                ),
                device=str(self.get_parameter("device").value),
            )
            self.output_frame = str(self.get_parameter("output_frame").value)
            self.use_tf = bool(self.get_parameter("use_tf").value)
            self.require_tf = bool(self.get_parameter("require_tf").value)
            self.sync_tolerance_s = (
                float(self.get_parameter("sync_tolerance_ms").value) / 1000.0
            )
            self.min_depth_m = float(self.get_parameter("min_depth_m").value)
            self.max_depth_m = float(self.get_parameter("max_depth_m").value)
            self.min_valid_depth_pixels = int(
                self.get_parameter("min_valid_depth_pixels").value
            )
            self.sticky_found = bool(self.get_parameter("sticky_found").value)
            self.marker_scale_m = float(self.get_parameter("marker_scale_m").value)
            self.publish_all_detections = bool(
                self.get_parameter("publish_all_detections").value
            )
            self.detections_min_confidence = float(
                self.get_parameter("detections_min_confidence").value
            )
            self.detections_publish_period_s = float(
                self.get_parameter("detections_publish_period_s").value
            )
            self.last_detections_time = 0.0

            self.tf_buffer = tf2_ros.Buffer() if self.use_tf else None
            self.tf_listener = (
                tf2_ros.TransformListener(self.tf_buffer, self)
                if self.tf_buffer
                else None
            )

            self.found_publisher = self.create_publisher(
                Bool, str(self.get_parameter("found_topic").value), 10
            )
            self.pose_publisher = self.create_publisher(
                PoseStamped, str(self.get_parameter("pose_topic").value), 10
            )
            self.marker_publisher = self.create_publisher(
                Marker, str(self.get_parameter("marker_topic").value), 10
            )
            self.status_publisher = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10
            )
            self.detections_publisher = (
                self.create_publisher(
                    String, str(self.get_parameter("detections_topic").value), 10
                )
                if self.publish_all_detections
                else None
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
            self.create_timer(
                float(self.get_parameter("found_publish_period_s").value),
                self._publish_state,
            )
            self.get_logger().info(
                "Loaded target_detection_node "
                f"model={self.get_parameter('model_path').value} "
                f"targets={target_class_names} output_frame={self.output_frame or 'camera'}"
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

        def _lookup_transform(self, source_frame: str) -> Any | None:
            if not self.tf_buffer or not self.output_frame or self.output_frame == source_frame:
                return None
            try:
                return self.tf_buffer.lookup_transform(
                    self.output_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except Exception as exc:
                if self.require_tf:
                    raise
                self.get_logger().warn(f"Target TF unavailable, using camera frame: {exc}")
                return None

        def _publish_status(
            self,
            *,
            found: bool,
            frame_id: str,
            latency_ms: float,
            position: TargetPosition | None,
            reason: str,
        ) -> None:
            now = time.monotonic()
            if now - self.last_status_time < 1.0:
                return
            self.last_status_time = now
            message = String()
            message.data = status_payload(
                found=found,
                frame_id=frame_id,
                latency_ms=latency_ms,
                position=position,
                reason=reason,
            )
            self.status_publisher.publish(message)
            self.get_logger().info(message.data)

        def _publish_detections(
            self,
            *,
            frame_id: str,
            stamp_sec: float,
            detections: list[TargetDetection],
        ) -> None:
            if not self.publish_all_detections:
                return
            if self.detections_publisher is None:
                return
            now = time.monotonic()
            if (
                self.detections_publish_period_s > 0.0
                and now - self.last_detections_time < self.detections_publish_period_s
            ):
                return
            self.last_detections_time = now
            message = String()
            message.data = detections_payload(
                frame_id=frame_id,
                stamp_sec=stamp_sec,
                detections=detections,
                target_names=self.target_names,
                min_confidence=self.detections_min_confidence,
            )
            self.detections_publisher.publish(message)
            self.get_logger().info(f"detections {message.data}")

        def _publish_state(self) -> None:
            message = Bool()
            message.data = bool(self.found)
            self.found_publisher.publish(message)
            if self.found and self.last_pose is not None:
                self.pose_publisher.publish(self.last_pose)
            if self.found and self.last_marker is not None:
                self.marker_publisher.publish(self.last_marker)

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
            source_frame = self.latest.rgb.header.frame_id
            publish_frame = source_frame
            position = None
            reason = "no_detection"
            found_now = False
            try:
                rgb = image_message_to_array(self.latest.rgb)
                depth = depth_message_to_meters(self.latest.depth)
                if self.publish_all_detections:
                    detections = self.detector.detect_all(rgb)
                    self._publish_detections(
                        frame_id=source_frame,
                        stamp_sec=stamp,
                        detections=detections,
                    )
                    detection = select_target_detection(
                        detections,
                        self.target_names,
                        confidence_threshold=float(
                            self.get_parameter("confidence_threshold").value
                        ),
                    )
                else:
                    detection = self.detector.detect(rgb)
                if detection is not None:
                    position = depth_centroid_position(
                        depth,
                        list(self.latest.camera_info.k),
                        detection,
                        min_depth_m=self.min_depth_m,
                        max_depth_m=self.max_depth_m,
                        min_valid_depth_pixels=self.min_valid_depth_pixels,
                    )
                    if position is None:
                        reason = "insufficient_valid_depth"
                    else:
                        xyz = (position.x, position.y, position.z)
                        transform = self._lookup_transform(source_frame)
                        if transform is not None:
                            xyz = transform_position(position, transform)
                            publish_frame = self.output_frame
                        elif self.output_frame and self.output_frame == source_frame:
                            publish_frame = self.output_frame
                        elif self.require_tf:
                            reason = "tf_required"
                            raise RuntimeError("Target TF is required but unavailable")

                        pose = configure_pose_message(
                            PoseStamped(),
                            frame_id=publish_frame,
                            stamp=self.latest.rgb.header.stamp,
                            xyz=xyz,
                        )
                        marker = configure_marker(
                            Marker(),
                            pose,
                            marker_id=0,
                            scale_m=self.marker_scale_m,
                        )
                        self.last_pose = pose
                        self.last_marker = marker
                        found_now = True
                        reason = "target_found"
            except Exception as exc:
                self.get_logger().error(f"Target detection failed: {exc}")
                reason = "error"

            if found_now:
                self.found = True
                self._publish_state()
            elif not self.sticky_found:
                self.found = False
            latency_ms = (time.perf_counter() - started) * 1000.0
            self._publish_status(
                found=self.found,
                frame_id=publish_frame,
                latency_ms=latency_ms,
                position=position,
                reason=reason,
            )

    return TargetDetectionNode


def main() -> None:
    rclpy, *_ = require_ros_modules()
    node_class = build_node_class()
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
