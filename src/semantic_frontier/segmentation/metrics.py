"""Task-relevant segmentation metrics."""

from __future__ import annotations

import numpy as np

from .labels import CLASS_NAMES, NUM_CLASSES


def confusion_matrix(target: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """Build a target-by-prediction confusion matrix."""
    target_flat = np.asarray(target, dtype=np.int64).ravel()
    prediction_flat = np.asarray(prediction, dtype=np.int64).ravel()
    valid = (
        (target_flat >= 0) & (target_flat < NUM_CLASSES)
        & (prediction_flat >= 0) & (prediction_flat < NUM_CLASSES)
    )
    indices = NUM_CLASSES * target_flat[valid] + prediction_flat[valid]
    return np.bincount(indices, minlength=NUM_CLASSES**2).reshape(
        NUM_CLASSES, NUM_CLASSES
    )


def metrics_from_confusion(matrix: np.ndarray) -> dict[str, object]:
    """Compute relevant macro metrics and per-class values."""
    matrix = np.asarray(matrix, dtype=np.int64)
    true_positive = np.diag(matrix).astype(float)
    target_support = matrix.sum(axis=1).astype(float)
    prediction_support = matrix.sum(axis=0).astype(float)
    union = target_support + prediction_support - true_positive
    iou = np.divide(true_positive, union, out=np.zeros_like(true_positive), where=union > 0)
    recall = np.divide(
        true_positive, target_support, out=np.zeros_like(true_positive),
        where=target_support > 0,
    )
    relevant_present = target_support[1:] > 0
    macro_iou = float(iou[1:][relevant_present].mean()) if relevant_present.any() else 0.0
    macro_recall = (
        float(recall[1:][relevant_present].mean()) if relevant_present.any() else 0.0
    )
    per_class = [
        {
            "id": class_id,
            "class_name": CLASS_NAMES[class_id],
            "iou": float(iou[class_id]),
            "recall": float(recall[class_id]),
            "target_pixels": int(target_support[class_id]),
            "predicted_pixels": int(prediction_support[class_id]),
        }
        for class_id in range(NUM_CLASSES)
    ]
    return {
        "relevant_macro_miou": macro_iou,
        "relevant_macro_recall": macro_recall,
        "per_class": per_class,
    }
