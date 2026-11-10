"""ROS-independent YOLO target detection and RGB-D localization helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class TargetDetection:
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: np.ndarray | None = None


@dataclass(frozen=True)
class TargetPosition:
    x: float
    y: float
    z: float
    valid_depth_pixels: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]


def normalize_target_names(names: str | Iterable[str]) -> set[str]:
    if isinstance(names, str):
        values = [item.strip() for item in names.split(",")]
    else:
        values = [str(item).strip() for item in names]
    normalized = {item.lower() for item in values if item}
    if not normalized:
        raise ValueError("target_class_names must contain at least one class name")
    return normalized


def select_target_detection(
    detections: Sequence[TargetDetection],
    target_class_names: str | Iterable[str],
    *,
    confidence_threshold: float,
) -> TargetDetection | None:
    targets = normalize_target_names(target_class_names)
    candidates = [
        detection
        for detection in detections
        if detection.class_name.strip().lower() in targets
        and detection.confidence >= confidence_threshold
    ]
    return max(candidates, key=lambda detection: detection.confidence) if candidates else None


def _to_numpy(value: object) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def detections_from_ultralytics_result(result: object) -> list[TargetDetection]:
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []
    xyxy = _to_numpy(getattr(boxes, "xyxy", []))
    cls = _to_numpy(getattr(boxes, "cls", []))
    conf = _to_numpy(getattr(boxes, "conf", []))
    if xyxy.size == 0:
        return []
    names = getattr(result, "names", {}) or {}
    masks = None
    result_masks = getattr(result, "masks", None)
    if result_masks is not None and getattr(result_masks, "data", None) is not None:
        masks = _to_numpy(result_masks.data)

    detections: list[TargetDetection] = []
    for index in range(len(xyxy)):
        class_id = int(cls[index])
        mask = None
        if masks is not None and index < len(masks):
            mask = np.asarray(masks[index]) > 0.5
        detections.append(
            TargetDetection(
                class_name=str(names.get(class_id, class_id)),
                confidence=float(conf[index]),
                bbox_xyxy=tuple(float(value) for value in xyxy[index]),
                mask=mask,
            )
        )
    return detections


class YoloTargetDetector:
    def __init__(
        self,
        model_path: str,
        target_class_names: str | Iterable[str],
        *,
        confidence_threshold: float,
        device: str = "auto",
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for target_detection_node"
            ) from exc
        self.model = YOLO(model_path)
        self.target_class_names = normalize_target_names(target_class_names)
        self.confidence_threshold = float(confidence_threshold)
        self.device = None if device == "auto" else device

    def detect(self, rgb: np.ndarray) -> TargetDetection | None:
        detections = self.detect_all(rgb)
        return select_target_detection(
            detections,
            self.target_class_names,
            confidence_threshold=self.confidence_threshold,
        )

    def detect_all(self, rgb: np.ndarray) -> list[TargetDetection]:
        kwargs = {"verbose": False}
        if self.device is not None:
            kwargs["device"] = self.device
        results = self.model.predict(rgb, **kwargs)
        return detections_from_ultralytics_result(results[0]) if results else []


def _bbox_mask(shape: tuple[int, int], bbox_xyxy: tuple[float, float, float, float]) -> np.ndarray:
    height, width = shape
    x1, y1, x2, y2 = bbox_xyxy
    left = max(0, min(width, int(np.floor(x1))))
    top = max(0, min(height, int(np.floor(y1))))
    right = max(0, min(width, int(np.ceil(x2))))
    bottom = max(0, min(height, int(np.ceil(y2))))
    mask = np.zeros((height, width), dtype=bool)
    if right > left and bottom > top:
        mask[top:bottom, left:right] = True
    return mask


def _resize_mask_nearest(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    if mask.shape == shape:
        return mask.astype(bool, copy=False)
    from PIL import Image

    image = Image.fromarray(mask.astype(np.uint8) * 255, mode="L")
    resized = image.resize((shape[1], shape[0]), Image.Resampling.NEAREST)
    return np.asarray(resized) > 0


def detection_region_mask(shape: tuple[int, int], detection: TargetDetection) -> np.ndarray:
    if detection.mask is not None:
        return _resize_mask_nearest(detection.mask, shape)
    return _bbox_mask(shape, detection.bbox_xyxy)


def depth_centroid_position(
    depth_m: np.ndarray,
    camera_k: Sequence[float],
    detection: TargetDetection,
    *,
    min_depth_m: float = 0.05,
    max_depth_m: float = 5.0,
    min_valid_depth_pixels: int = 20,
) -> TargetPosition | None:
    if depth_m.ndim != 2:
        raise ValueError(f"Expected HxW depth image, got shape {depth_m.shape}")
    if len(camera_k) != 9:
        raise ValueError("camera_k must contain the 3x3 row-major camera matrix")
    fx = float(camera_k[0])
    fy = float(camera_k[4])
    cx = float(camera_k[2])
    cy = float(camera_k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("camera intrinsics fx and fy must be non-zero")

    region = detection_region_mask(depth_m.shape, detection)
    valid = (
        region
        & np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    valid_count = int(valid.sum())
    if valid_count < min_valid_depth_pixels:
        return None

    rows, cols = np.nonzero(valid)
    z = depth_m[valid].astype(np.float32)
    x = (cols.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    return TargetPosition(
        x=float(x.mean()),
        y=float(y.mean()),
        z=float(z.mean()),
        valid_depth_pixels=valid_count,
        class_name=detection.class_name,
        confidence=detection.confidence,
        bbox_xyxy=detection.bbox_xyxy,
    )
