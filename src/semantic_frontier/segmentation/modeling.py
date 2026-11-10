"""Replaceable semantic-segmentation model adapters."""

from __future__ import annotations

from pathlib import Path

import torch
from transformers import SegformerConfig, SegformerForSemanticSegmentation

from .labels import ADE_NUM_CLASSES, CLASS_NAMES


def build_segformer(
    checkpoint: str, num_labels: int = ADE_NUM_CLASSES
) -> SegformerForSemanticSegmentation:
    """Load a SegFormer checkpoint and adapt the head when explicitly requested."""
    if num_labels == ADE_NUM_CLASSES:
        model = SegformerForSemanticSegmentation.from_pretrained(checkpoint)
    else:
        # Simulation training reuses the ADE-pretrained encoder while replacing
        # the incompatible ADE150 decode head with a compact project head.
        names = CLASS_NAMES[:num_labels]
        model = SegformerForSemanticSegmentation.from_pretrained(
            checkpoint,
            num_labels=num_labels,
            id2label={index: name for index, name in enumerate(names)},
            label2id={name: index for index, name in enumerate(names)},
            ignore_mismatched_sizes=True,
        )
    if model.config.num_labels != num_labels:
        raise ValueError(
            f"Expected {num_labels} labels, got {model.config.num_labels}"
        )
    return model


def build_tiny_segformer(num_labels: int = ADE_NUM_CLASSES) -> SegformerForSemanticSegmentation:
    """Build a small random SegFormer for local smoke tests."""
    config = SegformerConfig(
        num_labels=num_labels,
        depths=[1, 1, 1, 1],
        hidden_sizes=[8, 16, 32, 64],
        decoder_hidden_size=32,
        num_attention_heads=[1, 1, 2, 4],
        sr_ratios=[8, 4, 2, 1],
    )
    return SegformerForSemanticSegmentation(config)


def save_model(model: SegformerForSemanticSegmentation, path: Path) -> None:
    """Save a model in the standard Transformers format."""
    path.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(path)


def load_model(path: str | Path) -> SegformerForSemanticSegmentation:
    """Load a saved SegFormer checkpoint."""
    return SegformerForSemanticSegmentation.from_pretrained(path)


def select_device() -> torch.device:
    """Prefer CUDA, then MPS on Apple Silicon, and otherwise CPU."""
    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def select_training_device() -> torch.device:
    """Prefer CUDA for training and otherwise keep the CPU fallback."""
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def configure_segformer_backend(device: torch.device) -> None:
    """Configure backend kernels for the active SegFormer device."""
    if device.type == "cuda":
        # cuDNN 9.2 intermittently fails inside SegFormer sequence-reduction
        # convolutions on the WSL RTX 5080 stack. Native CUDA kernels preserve
        # the training contract and completed the same batches reliably.
        torch.backends.cudnn.enabled = False
