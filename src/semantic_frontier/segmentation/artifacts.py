"""Experiment artifact writers and visualizations."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .labels import CLASS_NAMES, NUM_CLASSES


def ensure_run_layout(run_dir: Path) -> None:
    """Create the approved output-contract directories."""
    for relative in (
        "logs", "checkpoints/best", "checkpoints/last", "metrics", "figures",
        "predictions",
    ):
        (run_dir / relative).mkdir(parents=True, exist_ok=True)


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    """Write table-like rows as readable JSONL."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def plot_training_history(rows: list[dict[str, object]], path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    epochs = [row["epoch"] for row in rows]
    axes[0].plot(epochs, [row["train_loss"] for row in rows], marker="o")
    axes[0].set_title("Training Loss")
    axes[0].set_xlabel("Epoch")
    axes[1].plot(epochs, [row["validation_miou"] for row in rows], marker="o", label="mIoU")
    axes[1].plot(
        epochs, [row["validation_recall"] for row in rows], marker="o", label="Recall"
    )
    axes[1].set_title("Relevant Validation Metrics")
    axes[1].set_xlabel("Epoch")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_per_class(rows: list[dict[str, object]], path: Path) -> None:
    relevant = rows[1:]
    fig, ax = plt.subplots(figsize=(12, 5))
    positions = np.arange(len(relevant))
    ax.bar(positions - 0.2, [row["iou"] for row in relevant], 0.4, label="IoU")
    ax.bar(positions + 0.2, [row["recall"] for row in relevant], 0.4, label="Recall")
    ax.set_xticks(positions, [row["class_name"] for row in relevant], rotation=45, ha="right")
    ax.set_ylim(0, 1)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_confusion(matrix: np.ndarray, path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 9))
    image = ax.imshow(matrix, cmap="Blues")
    ax.set_xticks(range(NUM_CLASSES), CLASS_NAMES, rotation=45, ha="right")
    ax.set_yticks(range(NUM_CLASSES), CLASS_NAMES)
    ax.set_xlabel("Prediction")
    ax.set_ylabel("Ground truth")
    fig.colorbar(image, ax=ax)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_prediction(
    image: Image.Image,
    target: np.ndarray,
    prediction: np.ndarray,
    relevance: np.ndarray,
    path: Path,
) -> None:
    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    axes[0].imshow(image)
    axes[0].set_title("RGB")
    axes[1].imshow(target, vmin=0, vmax=NUM_CLASSES - 1, cmap="tab20")
    axes[1].set_title("Ground truth")
    axes[2].imshow(prediction, vmin=0, vmax=NUM_CLASSES - 1, cmap="tab20")
    axes[2].set_title("Prediction")
    heatmap = axes[3].imshow(relevance, vmin=0, vmax=1, cmap="magma")
    axes[3].set_title("Task-relevant ADE probability sum")
    fig.colorbar(heatmap, ax=axes[3])
    for axis in axes:
        axis.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
