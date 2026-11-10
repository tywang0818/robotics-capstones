"""Training-label and task-relevant evaluation mappings."""

from __future__ import annotations

import numpy as np

ADE_TO_TASK = {
    11: 1, 16: 2, 25: 3, 34: 4, 36: 5, 42: 6, 45: 7,
    46: 8, 63: 9, 65: 10, 71: 11, 113: 12, 116: 13,
}

CLASS_NAMES = (
    "other", "cabinet", "table", "shelf", "desk", "wardrobe", "box",
    "chest_of_drawers", "counter", "bookcase", "coffee_table", "countertop",
    "basket", "bag",
)
RELEVANT_CLASS_NAMES = CLASS_NAMES[1:]
NUM_CLASSES = len(CLASS_NAMES)
ADE_NUM_CLASSES = 150
IGNORE_LABEL = 255
RELEVANT_ADE_MODEL_IDS = tuple(ade_id - 1 for ade_id in ADE_TO_TASK)
TASK_SOURCE_IGNORE_LABEL = 0
TASK_SOURCE_OTHER_LABEL = 14


def prepare_ade_labels(annotation: np.ndarray) -> np.ndarray:
    """Convert ADE20K labels 1..150 to model labels 0..149 and ignore 0."""
    source = np.asarray(annotation)
    labels = np.full(source.shape, IGNORE_LABEL, dtype=np.uint8)
    foreground = source > 0
    labels[foreground] = source[foreground] - 1
    return labels


def task_labels_from_annotation(annotation: np.ndarray) -> np.ndarray:
    """Map an ADE20K annotation to the compact task-evaluation label space."""
    source = np.asarray(annotation)
    mapped = np.zeros(source.shape, dtype=np.uint8)
    mapped[source == 0] = IGNORE_LABEL
    for ade_id, task_id in ADE_TO_TASK.items():
        mapped[source == ade_id] = task_id
    return mapped


def task_labels_from_prediction(prediction: np.ndarray) -> np.ndarray:
    """Map a zero-indexed ADE150 model prediction to task-evaluation labels."""
    return task_labels_from_annotation(np.asarray(prediction) + 1)


def task_relevance_from_probabilities(probabilities: np.ndarray) -> np.ndarray:
    """Sum probabilities assigned to the 13 task-relevant ADE classes."""
    return np.asarray(probabilities)[list(RELEVANT_ADE_MODEL_IDS)].sum(axis=0)


def present_task_classes(annotation: np.ndarray) -> set[int]:
    """Return relevant task IDs present in a SceneParse150 annotation."""
    values = set(np.unique(annotation).tolist())
    return {task_id for ade_id, task_id in ADE_TO_TASK.items() if ade_id in values}


def prepare_task_labels(annotation: np.ndarray) -> np.ndarray:
    """Convert project simulation labels to compact training labels.

    Simulation label images reserve source label 0 for void pixels that should
    not affect loss or metrics. Task-relevant source labels 1..13 already match
    the compact task label IDs. Source label 14 represents non-task objects and
    trains the model's compact class 0 (`other`).
    """
    source = np.asarray(annotation)
    labels = np.full(source.shape, IGNORE_LABEL, dtype=np.uint8)
    task_pixels = (source >= 1) & (source < NUM_CLASSES)
    labels[task_pixels] = source[task_pixels].astype(np.uint8)
    labels[source == TASK_SOURCE_OTHER_LABEL] = 0
    return labels


def task_labels_from_task_annotation(annotation: np.ndarray) -> np.ndarray:
    """Map project simulation annotations to the shared task metric space."""
    return prepare_task_labels(annotation)


def task_labels_from_task_prediction(prediction: np.ndarray) -> np.ndarray:
    """Interpret compact model predictions as task metric labels directly."""
    return np.asarray(prediction).astype(np.uint8)


def task_relevance_from_task_probabilities(probabilities: np.ndarray) -> np.ndarray:
    """Sum probabilities assigned to compact task-relevant classes 1..13."""
    return np.asarray(probabilities)[1:NUM_CLASSES].sum(axis=0)


def present_task_classes_from_task_annotation(annotation: np.ndarray) -> set[int]:
    """Return task-relevant compact class IDs present in simulation labels."""
    values = set(np.unique(annotation).tolist())
    return {class_id for class_id in range(1, NUM_CLASSES) if class_id in values}


def task_labels_from_model_prediction(
    prediction: np.ndarray, label_space: str = "ade150"
) -> np.ndarray:
    """Map model predictions to task labels for the configured label space."""
    if label_space == "ade150":
        return task_labels_from_prediction(prediction)
    if label_space == "task":
        return task_labels_from_task_prediction(prediction)
    raise ValueError(f"Unsupported label space: {label_space}")


def task_relevance_from_model_probabilities(
    probabilities: np.ndarray, label_space: str = "ade150"
) -> np.ndarray:
    """Compute the task-relevant probability sum for the configured model head."""
    if label_space == "ade150":
        return task_relevance_from_probabilities(probabilities)
    if label_space == "task":
        return task_relevance_from_task_probabilities(probabilities)
    raise ValueError(f"Unsupported label space: {label_space}")
