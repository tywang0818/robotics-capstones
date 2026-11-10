"""SceneParse150 preparation and PyTorch dataset support."""

from __future__ import annotations

import json
import random
from collections import Counter
from pathlib import Path
from typing import Iterable

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

from .labels import (
    NUM_CLASSES, prepare_ade_labels, prepare_task_labels, present_task_classes,
    task_labels_from_annotation, task_labels_from_task_annotation,
)


def select_stratified_indices(
    annotations: Iterable[np.ndarray],
    size: int,
    minimum_per_class: int,
    seed: int,
    excluded: set[int] | None = None,
) -> tuple[list[int], Counter[int]]:
    """Select a deterministic subset while prioritizing under-covered classes."""
    excluded = excluded or set()
    candidates = [
        (index, present_task_classes(annotation))
        for index, annotation in enumerate(annotations)
        if index not in excluded
    ]
    random.Random(seed).shuffle(candidates)
    selected: list[int] = []
    coverage: Counter[int] = Counter()
    while candidates and len(selected) < size:
        best_position = max(
            range(len(candidates)),
            key=lambda pos: sum(
                max(minimum_per_class - coverage[class_id], 0)
                for class_id in candidates[pos][1]
            ),
        )
        index, classes = candidates.pop(best_position)
        selected.append(index)
        coverage.update(classes)
    return selected, coverage


def write_split_manifest(path: Path, source_split: str, indices: Iterable[int]) -> None:
    """Write a JSONL split manifest."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for index in indices:
            handle.write(json.dumps({"source_split": source_split, "index": index}) + "\n")


def read_split_manifest(path: Path) -> list[dict[str, object]]:
    """Read a JSONL split manifest."""
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def write_coverage_jsonl(path: Path, split_coverage: dict[str, Counter[int]]) -> None:
    """Write per-split image-level class coverage as readable JSONL."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for split, coverage in split_coverage.items():
            for class_id in range(1, NUM_CLASSES):
                handle.write(json.dumps({
                    "split": split,
                    "task_class_id": class_id,
                    "image_count": coverage[class_id],
                }) + "\n")


class TaskSegmentationDataset(Dataset):
    """Resize segmentation examples and expose training/evaluation labels.

    `label_space="ade150"` preserves the original SceneParse150 contract.
    `label_space="task"` is used by Gazebo simulation captures, where source
    labels are compact project IDs instead of ADE category IDs.
    """

    def __init__(
        self, dataset, indices: list[int], image_size: int, label_space: str = "ade150"
    ):
        self.dataset = dataset
        self.indices = indices
        self.image_size = image_size
        self.label_space = label_space

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, position: int) -> dict[str, object]:
        source_index = self.indices[position]
        example = self.dataset[source_index]
        image = example["image"].convert("RGB").resize(
            (self.image_size, self.image_size), Image.Resampling.BILINEAR
        )
        annotation = example["annotation"].resize(
            (self.image_size, self.image_size), Image.Resampling.NEAREST
        )
        pixels = np.asarray(image, dtype=np.float32) / 255.0
        pixels = (pixels - np.array([0.485, 0.456, 0.406])) / np.array(
            [0.229, 0.224, 0.225]
        )
        annotation_array = np.asarray(annotation)
        if self.label_space == "ade150":
            training_labels = prepare_ade_labels(annotation_array)
            task_labels = task_labels_from_annotation(annotation_array)
        elif self.label_space == "task":
            training_labels = prepare_task_labels(annotation_array)
            task_labels = task_labels_from_task_annotation(annotation_array)
        else:
            raise ValueError(f"Unsupported label space: {self.label_space}")
        return {
            "pixel_values": torch.from_numpy(pixels.transpose(2, 0, 1)).float(),
            "labels": torch.from_numpy(training_labels.astype(np.int64)),
            "task_labels": torch.from_numpy(task_labels.astype(np.int64)),
            "source_index": source_index,
        }
