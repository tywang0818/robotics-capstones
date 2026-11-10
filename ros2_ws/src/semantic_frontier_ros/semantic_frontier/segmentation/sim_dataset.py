"""Gazebo simulation semantic-segmentation dataset utilities.

This module keeps the ROS/Gazebo collection format independent from the model
training loop. Collection writes raw RGB/depth/semantic frames under
`data/semantic_segmentation/raw/sim/<dataset-id>/`; preparation validates those
frames and writes train/validation/test manifests under the matching processed
directory.
"""

from __future__ import annotations

import json
import random
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image

from .labels import (
    CLASS_NAMES, NUM_CLASSES, TASK_SOURCE_OTHER_LABEL,
    present_task_classes_from_task_annotation,
)

SIM_DATASET_NAME = "GazeboSimSemantic"
REQUIRED_FRAME_KEYS = (
    "scene_id", "frame_id", "rgb_path", "semantic_label_path",
    "depth_path", "camera_info_path", "pose_map_camera_path",
)


def default_sim_labels() -> list[dict[str, object]]:
    """Return the canonical source-label contract for collected sim masks."""
    rows = [{"id": 0, "name": "void", "train": False, "relevant": False}]
    rows.extend(
        {"id": index, "name": name, "train": True, "relevant": True}
        for index, name in enumerate(CLASS_NAMES[1:], start=1)
    )
    rows.append({
        "id": TASK_SOURCE_OTHER_LABEL,
        "name": "other",
        "train": True,
        "relevant": False,
    })
    return rows


def write_default_labels(path: Path) -> None:
    """Create `labels.json` when a collector starts a new simulation dataset."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(default_sim_labels(), indent=2) + "\n",
        encoding="utf-8",
    )


def read_jsonl(path: Path) -> list[dict[str, object]]:
    """Read a JSONL file and ignore blank lines."""
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def write_jsonl(path: Path, rows: Iterable[dict[str, object]]) -> None:
    """Write stable JSONL rows for manifests and coverage reports."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def _relative_path(root: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else root / path


def _label_ids(labels: list[dict[str, object]]) -> set[int]:
    return {int(row["id"]) for row in labels}


def read_labels(dataset_root: Path) -> list[dict[str, object]]:
    """Load and validate the simulation label contract."""
    path = dataset_root / "labels.json"
    if not path.exists():
        raise FileNotFoundError(f"Missing simulation labels: {path}")
    labels = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(labels, list):
        raise ValueError("labels.json must contain a list of label objects")
    ids = _label_ids(labels)
    expected = set(range(NUM_CLASSES)) | {TASK_SOURCE_OTHER_LABEL}
    missing = expected - ids
    if missing:
        raise ValueError(f"labels.json is missing required labels: {sorted(missing)}")
    return labels


def read_frame_manifest(dataset_root: Path) -> list[dict[str, object]]:
    """Load the raw frame manifest written by the ROS/Gazebo collector."""
    path = dataset_root / "manifest.jsonl"
    if not path.exists():
        raise FileNotFoundError(f"Missing simulation manifest: {path}")
    rows = read_jsonl(path)
    if not rows:
        raise ValueError(f"Simulation manifest is empty: {path}")
    return rows


def annotation_label_histogram(annotation: np.ndarray) -> dict[int, int]:
    """Count source labels in one semantic mask."""
    values, counts = np.unique(annotation, return_counts=True)
    return {int(value): int(count) for value, count in zip(values, counts)}


def validate_frame_rows(dataset_root: Path) -> list[dict[str, object]]:
    """Validate manifest rows, file presence, and semantic label IDs."""
    labels = read_labels(dataset_root)
    allowed_labels = _label_ids(labels)
    rows = read_frame_manifest(dataset_root)
    seen_frames: set[tuple[str, str]] = set()
    for row in rows:
        missing = [key for key in REQUIRED_FRAME_KEYS if key not in row]
        if missing:
            raise ValueError(f"Manifest row is missing keys {missing}: {row}")
        frame_key = (str(row["scene_id"]), str(row["frame_id"]))
        if frame_key in seen_frames:
            raise ValueError(f"Duplicate scene/frame pair in manifest: {frame_key}")
        seen_frames.add(frame_key)
        for key in REQUIRED_FRAME_KEYS[2:]:
            path = _relative_path(dataset_root, row[key])
            if not path.exists():
                raise FileNotFoundError(f"Manifest points to missing {key}: {path}")
        annotation = np.asarray(Image.open(_relative_path(dataset_root, row["semantic_label_path"])))
        unknown_labels = set(annotation_label_histogram(annotation)) - allowed_labels
        if unknown_labels:
            raise ValueError(
                f"Frame {frame_key} contains labels not in labels.json: "
                f"{sorted(unknown_labels)}"
            )
    return rows


@dataclass(frozen=True)
class SplitConfig:
    """Scene-disjoint split parameters for a simulation dataset."""

    train_fraction: float = 0.70
    validation_fraction: float = 0.15
    test_fraction: float = 0.15
    seed: int = 42


def scene_disjoint_split_indices(
    rows: list[dict[str, object]], config: SplitConfig
) -> dict[str, list[int]]:
    """Split frame indices by scene so no scene leaks across dataset splits."""
    scenes = sorted({str(row["scene_id"]) for row in rows})
    random.Random(config.seed).shuffle(scenes)
    if len(scenes) < 3:
        raise ValueError("At least three scenes are required for scene-disjoint splits")

    train_count = max(1, round(len(scenes) * config.train_fraction))
    validation_count = max(1, round(len(scenes) * config.validation_fraction))
    if train_count + validation_count >= len(scenes):
        validation_count = 1
        train_count = len(scenes) - 2
    train_scenes = set(scenes[:train_count])
    validation_scenes = set(scenes[train_count:train_count + validation_count])
    test_scenes = set(scenes[train_count + validation_count:])
    split_scenes = {
        "train": train_scenes,
        "validation": validation_scenes,
        "test": test_scenes,
    }
    return {
        split: [
            index for index, row in enumerate(rows)
            if str(row["scene_id"]) in scene_ids
        ]
        for split, scene_ids in split_scenes.items()
    }


def coverage_for_indices(
    dataset_root: Path, rows: list[dict[str, object]], indices: Iterable[int]
) -> Counter[int]:
    """Count how many images in a split contain each relevant task class."""
    coverage: Counter[int] = Counter()
    for index in indices:
        annotation = np.asarray(
            Image.open(_relative_path(dataset_root, rows[index]["semantic_label_path"]))
        )
        coverage.update(present_task_classes_from_task_annotation(annotation))
    return coverage


def write_sim_split_manifest(
    path: Path, rows: list[dict[str, object]], indices: Iterable[int]
) -> None:
    """Write split rows compatible with the existing training source loader."""
    write_jsonl(
        path,
        (
            {
                "source_split": "sim",
                "index": int(index),
                "scene_id": rows[index]["scene_id"],
                "frame_id": rows[index]["frame_id"],
            }
            for index in indices
        ),
    )


class GazeboSimSemanticSource:
    """List-like access to validated Gazebo semantic segmentation frames."""

    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.rows = validate_frame_rows(self.root)

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, object]:
        row = self.rows[index]
        return {
            "image": Image.open(_relative_path(self.root, row["rgb_path"])).copy(),
            "annotation": Image.open(
                _relative_path(self.root, row["semantic_label_path"])
            ).copy(),
            "scene_id": row["scene_id"],
            "frame_id": row["frame_id"],
        }
