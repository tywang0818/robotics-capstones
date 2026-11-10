"""Online RGB-D semantic inference helpers for ROS integration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional

from semantic_frontier.segmentation.labels import (
    NUM_CLASSES,
    task_labels_from_model_prediction,
    task_relevance_from_model_probabilities,
)
from semantic_frontier.segmentation.modeling import (
    configure_segformer_backend,
    load_model,
    select_device,
)


IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


@dataclass(frozen=True)
class SemanticRaster:
    """Per-pixel semantic evidence in the compact task label space."""

    class_id: np.ndarray
    class_prob: np.ndarray
    relevance_prob: np.ndarray


@dataclass(frozen=True)
class SemanticPoint:
    """A camera-frame 3D point with lightweight semantic evidence."""

    x: float
    y: float
    z: float
    class_id: int
    class_prob: float
    relevance_prob: float


def normalize_rgb(rgb: np.ndarray, image_size: int) -> torch.Tensor:
    """Resize and normalize an RGB array into a SegFormer input tensor."""
    from PIL import Image

    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 RGB image, got shape {rgb.shape}")
    image = Image.fromarray(rgb.astype(np.uint8, copy=False), mode="RGB")
    image = image.resize((image_size, image_size), Image.Resampling.BILINEAR)
    pixels = np.asarray(image, dtype=np.float32) / 255.0
    pixels = (pixels - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(pixels.transpose(2, 0, 1)).float().unsqueeze(0)


def semantic_raster_from_probabilities(
    probabilities: np.ndarray,
    label_space: str,
) -> SemanticRaster:
    """Convert model probabilities into compact task IDs and scores."""
    if probabilities.ndim != 3:
        raise ValueError(
            f"Expected CxHxW probability array, got shape {probabilities.shape}"
        )
    prediction = probabilities.argmax(axis=0)
    class_id = task_labels_from_model_prediction(prediction, label_space)
    class_prob = probabilities.max(axis=0).astype(np.float32, copy=False)
    relevance_prob = task_relevance_from_model_probabilities(
        probabilities, label_space
    ).astype(np.float32, copy=False)
    return SemanticRaster(
        class_id=class_id.astype(np.uint8, copy=False),
        class_prob=class_prob,
        relevance_prob=np.clip(relevance_prob, 0.0, 1.0),
    )


def project_semantic_raster_to_points(
    depth_m: np.ndarray,
    camera_k: list[float] | tuple[float, ...],
    raster: SemanticRaster,
    *,
    stride: int = 4,
    max_depth_m: float = 5.0,
    min_relevance_prob: float = 0.0,
) -> list[SemanticPoint]:
    """Project semantic pixels into camera-frame 3D points."""
    if depth_m.ndim != 2:
        raise ValueError(f"Expected HxW depth image, got shape {depth_m.shape}")
    if raster.class_id.shape != depth_m.shape:
        raise ValueError(
            "Semantic raster and depth image must have matching HxW shapes: "
            f"{raster.class_id.shape} != {depth_m.shape}"
        )
    if stride < 1:
        raise ValueError("stride must be >= 1")
    if len(camera_k) != 9:
        raise ValueError("camera_k must contain the 3x3 row-major camera matrix")

    fx = float(camera_k[0])
    fy = float(camera_k[4])
    cx = float(camera_k[2])
    cy = float(camera_k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("Camera intrinsics fx and fy must be non-zero")

    points: list[SemanticPoint] = []
    height, width = depth_m.shape
    for v in range(0, height, stride):
        for u in range(0, width, stride):
            z = float(depth_m[v, u])
            relevance = float(raster.relevance_prob[v, u])
            if (
                not np.isfinite(z)
                or z <= 0.0
                or z > max_depth_m
                or relevance < min_relevance_prob
            ):
                continue
            x = (float(u) - cx) * z / fx
            y = (float(v) - cy) * z / fy
            points.append(
                SemanticPoint(
                    x=x,
                    y=y,
                    z=z,
                    class_id=int(raster.class_id[v, u]),
                    class_prob=float(raster.class_prob[v, u]),
                    relevance_prob=relevance,
                )
            )
    return points


def resize_raster(
    raster: SemanticRaster,
    output_shape: tuple[int, int],
) -> SemanticRaster:
    """Resize semantic evidence to match the depth image shape."""
    height, width = output_shape
    class_id = functional.interpolate(
        torch.from_numpy(raster.class_id.astype(np.float32))[None, None],
        size=(height, width),
        mode="nearest",
    )[0, 0].numpy().astype(np.uint8)
    class_prob = functional.interpolate(
        torch.from_numpy(raster.class_prob)[None, None],
        size=(height, width),
        mode="bilinear",
        align_corners=False,
    )[0, 0].numpy().astype(np.float32)
    relevance_prob = functional.interpolate(
        torch.from_numpy(raster.relevance_prob)[None, None],
        size=(height, width),
        mode="bilinear",
        align_corners=False,
    )[0, 0].numpy().astype(np.float32)
    return SemanticRaster(
        class_id=class_id,
        class_prob=np.clip(class_prob, 0.0, 1.0),
        relevance_prob=np.clip(relevance_prob, 0.0, 1.0),
    )


class OnlineSemanticSegmenter:
    """Load a saved SegFormer checkpoint and run single-frame inference."""

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        image_size: int,
        label_space: str,
        device: str = "auto",
    ):
        self.image_size = int(image_size)
        self.label_space = label_space
        self.device = select_device() if device == "auto" else torch.device(device)
        configure_segformer_backend(self.device)
        self.model = load_model(checkpoint_path).to(self.device)
        self.model.eval()

    def predict(self, rgb: np.ndarray, output_shape: tuple[int, int]) -> SemanticRaster:
        """Run inference and return a semantic raster at output_shape."""
        pixels = normalize_rgb(rgb, self.image_size).to(self.device)
        with torch.no_grad():
            logits = self.model(pixel_values=pixels).logits
            logits = functional.interpolate(
                logits,
                size=output_shape,
                mode="bilinear",
                align_corners=False,
            )
            probabilities = logits.softmax(dim=1)[0].cpu().numpy()
        if self.label_space == "task" and probabilities.shape[0] != NUM_CLASSES:
            raise ValueError(
                f"Task label space expects {NUM_CLASSES} model labels, "
                f"got {probabilities.shape[0]}"
            )
        return semantic_raster_from_probabilities(probabilities, self.label_space)
