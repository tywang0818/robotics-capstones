"""End-to-end preparation, training, evaluation, and prediction workflows."""

from __future__ import annotations

import json
import os
import platform
import re
import resource
import shutil
import time
import zipfile
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn.functional as functional
from torch.utils.data import DataLoader

from .artifacts import (
    ensure_run_layout, plot_confusion, plot_per_class, plot_prediction,
    plot_training_history, write_json, write_rows,
)
from .data import (
    TaskSegmentationDataset, read_split_manifest, select_stratified_indices,
    write_coverage_jsonl, write_split_manifest,
)
from .labels import (
    ADE_NUM_CLASSES, CLASS_NAMES, IGNORE_LABEL, NUM_CLASSES, present_task_classes,
    present_task_classes_from_task_annotation, task_labels_from_model_prediction,
    task_relevance_from_model_probabilities,
)
from .metrics import confusion_matrix, metrics_from_confusion
from .modeling import (
    build_segformer, configure_segformer_backend, load_model, save_model,
    select_device, select_training_device,
)
from .sim_dataset import (
    GazeboSimSemanticSource, SIM_DATASET_NAME, SplitConfig, coverage_for_indices,
    scene_disjoint_split_indices, validate_frame_rows, write_sim_split_manifest,
)


def _paths(config: dict) -> tuple[Path, Path, Path]:
    data_root = Path(config["data"]["data_root"])
    output_root = Path(config["experiment"]["output_root"])
    if config["data"]["dataset"] == SIM_DATASET_NAME:
        split_root = data_root / "processed" / "sim" / config["data"]["split_id"] / "splits"
        cache_root = data_root / "raw" / "sim" / config["data"]["split_id"]
    else:
        split_root = data_root / "processed" / "splits" / config["data"]["split_id"]
        cache_root = data_root / "raw" / "huggingface-cache"
    return split_root, output_root, cache_root


def _load_source(config: dict):
    _, _, cache_root = _paths(config)
    if config["data"]["dataset"] == SIM_DATASET_NAME:
        return {"sim": GazeboSimSemanticSource(cache_root)}
    raw_root = cache_root.parent
    archive = _download_files([config["data"]["official_archive_url"]], raw_root)[0]
    extracted_root = raw_root / "ADEChallengeData2016"
    if not extracted_root.exists():
        with zipfile.ZipFile(archive) as source:
            source.extractall(raw_root)
    return {
        "train": LocalSceneParseSource(extracted_root, "training"),
        "validation": LocalSceneParseSource(extracted_root, "validation"),
    }


class LocalSceneParseSource:
    """List-like access to the official ADEChallengeData2016 release."""

    def __init__(self, root: Path, split: str):
        self.images = sorted((root / "images" / split).glob("*.jpg"))
        self.annotations = sorted((root / "annotations" / split).glob("*.png"))
        if len(self.images) != len(self.annotations):
            raise ValueError(f"Mismatched image and annotation counts for {split}")

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> dict[str, object]:
        from PIL import Image

        return {
            "image": Image.open(self.images[index]).copy(),
            "annotation": Image.open(self.annotations[index]).copy(),
        }

    def __iter__(self):
        for index in range(len(self)):
            yield self[index]


def _download_files(urls: list[str], destination: Path) -> list[Path]:
    """Download fixed dataset files with visible byte progress."""
    destination.mkdir(parents=True, exist_ok=True)
    downloaded = []
    for url in urls:
        path = destination / url.rsplit("/", 1)[-1]
        downloaded.append(path)
        if path.exists() and path.stat().st_size > 0:
            continue
        partial = path.with_suffix(path.suffix + ".part")
        with urlopen(url, timeout=120) as response:
            total = int(response.headers.get("content-length", 0))
            written = 0
            with partial.open("wb") as handle:
                while chunk := response.read(1024 * 1024):
                    handle.write(chunk)
                    written += len(chunk)
                    print(
                        f"\rDownloading {path.name}: {written}/{total} bytes",
                        end="",
                        flush=True,
                    )
        partial.replace(path)
        print()
    return downloaded


def _annotations(dataset):
    for example in dataset:
        yield np.asarray(example["annotation"])


def _label_space(config: dict) -> str:
    return str(config["data"].get("label_space", "ade150"))


def _model_num_labels(config: dict) -> int:
    return int(config["model"].get("num_labels", ADE_NUM_CLASSES))


def _task_classes_for_annotation(annotation: np.ndarray, label_space: str) -> set[int]:
    if label_space == "ade150":
        return present_task_classes(annotation)
    if label_space == "task":
        return present_task_classes_from_task_annotation(annotation)
    raise ValueError(f"Unsupported label space: {label_space}")


def _prepare_gazebo_sim_data(config: dict) -> dict[str, object]:
    split_root, output_root, raw_root = _paths(config)
    rows = validate_frame_rows(raw_root)
    split_config = SplitConfig(
        train_fraction=float(config["data"].get("train_fraction", 0.70)),
        validation_fraction=float(config["data"].get("validation_fraction", 0.15)),
        test_fraction=float(config["data"].get("test_fraction", 0.15)),
        seed=int(config["experiment"]["seed"]),
    )
    splits = scene_disjoint_split_indices(rows, split_config)
    coverages = {
        name: coverage_for_indices(raw_root, rows, indices)
        for name, indices in splits.items()
    }
    requirements = {
        "train": int(config["data"].get("train_min_per_class", 0)),
        "validation": int(config["data"].get("evaluation_min_per_class", 0)),
        "test": int(config["data"].get("evaluation_min_per_class", 0)),
    }
    for name, coverage in coverages.items():
        missing = [
            CLASS_NAMES[class_id]
            for class_id in range(1, NUM_CLASSES)
            if coverage[class_id] < requirements[name]
        ]
        if missing:
            raise ValueError(f"{name} split does not meet class coverage: {missing}")

    for name, indices in splits.items():
        write_sim_split_manifest(split_root / f"{name}.jsonl", rows, indices)
    write_coverage_jsonl(split_root / "class_coverage.jsonl", coverages)

    figure_root = output_root / "dataset-preparation" / config["data"]["split_id"]
    figure_root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(split_root / "class_coverage.jsonl", figure_root / "class_coverage.jsonl")
    fig, ax = plt.subplots(figsize=(12, 5))
    for name, coverage in coverages.items():
        ax.plot(
            range(1, NUM_CLASSES),
            [coverage[index] for index in range(1, NUM_CLASSES)],
            marker="o",
            label=name,
        )
    ax.set_xticks(range(1, NUM_CLASSES), CLASS_NAMES[1:], rotation=45, ha="right")
    ax.set_ylabel("Images containing class")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_root / "class_coverage.png", dpi=160)
    plt.close(fig)
    return {name: len(indices) for name, indices in splits.items()}


def prepare_data(config: dict) -> dict[str, object]:
    """Download SceneParse150 and write deterministic split manifests."""
    if config["data"]["dataset"] == SIM_DATASET_NAME:
        return _prepare_gazebo_sim_data(config)

    split_root, output_root, _ = _paths(config)
    source = _load_source(config)
    seed = config["experiment"]["seed"]
    train_indices, train_coverage = select_stratified_indices(
        _annotations(source["train"]), config["data"]["train_size"],
        config["data"]["train_min_per_class"], seed,
    )
    if config["data"].get("validation_uses_training_split", False):
        validation_indices = train_indices.copy()
        validation_coverage = train_coverage.copy()
    else:
        validation_indices, validation_coverage = select_stratified_indices(
            _annotations(source["train"]), config["data"]["validation_size"],
            config["data"]["evaluation_min_per_class"], seed + 1, set(train_indices),
        )
    test_indices, test_coverage = select_stratified_indices(
        _annotations(source["validation"]), config["data"]["test_size"],
        config["data"]["evaluation_min_per_class"], seed + 2,
    )
    splits = {
        "train": ("train", train_indices, train_coverage),
        "validation": ("train", validation_indices, validation_coverage),
        "test": ("validation", test_indices, test_coverage),
    }
    requirements = {
        "train": config["data"]["train_min_per_class"],
        "validation": config["data"]["evaluation_min_per_class"],
        "test": config["data"]["evaluation_min_per_class"],
    }
    for name, (_, _, coverage) in splits.items():
        missing = [
            CLASS_NAMES[class_id]
            for class_id in range(1, NUM_CLASSES)
            if coverage[class_id] < requirements[name]
        ]
        if missing:
            raise ValueError(f"{name} split does not meet class coverage: {missing}")
    for name, (source_name, indices, _) in splits.items():
        write_split_manifest(split_root / f"{name}.jsonl", source_name, indices)
    coverages = {name: values[2] for name, values in splits.items()}
    write_coverage_jsonl(split_root / "class_coverage.jsonl", coverages)

    figure_root = output_root / "dataset-preparation" / config["data"]["split_id"]
    figure_root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(split_root / "class_coverage.jsonl", figure_root / "class_coverage.jsonl")
    fig, ax = plt.subplots(figsize=(12, 5))
    for name, coverage in coverages.items():
        ax.plot(range(1, NUM_CLASSES), [coverage[index] for index in range(1, NUM_CLASSES)], marker="o", label=name)
    ax.set_xticks(range(1, NUM_CLASSES), CLASS_NAMES[1:], rotation=45, ha="right")
    ax.set_ylabel("Images containing class")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_root / "class_coverage.png", dpi=160)
    plt.close(fig)
    return {name: len(values[1]) for name, values in splits.items()}


def _manifest_indices(split_root: Path, name: str) -> tuple[str, list[int]]:
    rows = read_split_manifest(split_root / f"{name}.jsonl")
    return str(rows[0]["source_split"]), [int(row["index"]) for row in rows]


def _resource_snapshot() -> dict[str, object]:
    maximum_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    if platform.system() != "Darwin":
        maximum_rss *= 1024
    snapshot: dict[str, object] = {
        "max_rss_gb": maximum_rss / (1024**3),
        "system_load_1m": float(os.getloadavg()[0]),
    }
    if torch.cuda.is_available():
        device_index = torch.cuda.current_device()
        snapshot.update({
            "cuda_device": torch.cuda.get_device_name(device_index),
            "cuda_memory_allocated_gb": torch.cuda.memory_allocated(device_index) / (1024**3),
            "cuda_memory_reserved_gb": torch.cuda.memory_reserved(device_index) / (1024**3),
            "cuda_max_memory_allocated_gb": torch.cuda.max_memory_allocated(device_index) / (1024**3),
        })
    return snapshot


def _record_progress(run_dir: Path, rows: list[dict[str, object]], row: dict[str, object]) -> None:
    rows.append(row)
    write_rows(run_dir / "logs" / "progress.jsonl", rows)
    print(json.dumps(row), flush=True)


def _evaluate_loader(
    model, loader, device, label_space: str = "ade150"
) -> tuple[dict[str, object], np.ndarray]:
    matrix = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
    model.eval()
    with torch.no_grad():
        for batch in loader:
            labels = batch["task_labels"]
            logits = model(pixel_values=batch["pixel_values"].to(device)).logits
            logits = functional.interpolate(logits, size=labels.shape[-2:], mode="bilinear", align_corners=False)
            predictions = task_labels_from_model_prediction(
                logits.argmax(dim=1).cpu().numpy(), label_space
            )
            for target, prediction in zip(labels.numpy(), predictions):
                matrix += confusion_matrix(target, prediction)
    return metrics_from_confusion(matrix), matrix


def _new_run(config: dict, config_path: Path, training_device: torch.device) -> Path:
    _, output_root, _ = _paths(config)
    slug = re.sub(r"[^a-z0-9]+", "-", config["experiment"]["slug"].lower()).strip("-")
    run_id = f"{datetime.now():%Y%m%d-%H%M%S}-{slug}-seed{config['experiment']['seed']}"
    run_dir = output_root / "runs" / run_id
    ensure_run_layout(run_dir)
    shutil.copy2(config_path, run_dir / "resolved_config.toml")
    write_json(run_dir / "run_manifest.json", {
        "run_id": run_id,
        "experiment_name": config["experiment"]["name"],
        "experiment_stage": config["experiment"]["stage"],
        "purpose": config["experiment"]["purpose"],
        "created_at": datetime.now().isoformat(),
        "python": platform.python_version(),
        "torch": torch.__version__,
        "training_device": str(training_device),
        "preferred_evaluation_device": str(select_device()),
        "dataset": config["data"]["dataset"],
        "split_id": config["data"]["split_id"],
        "model_checkpoint": config["model"]["checkpoint"],
        "model_num_labels": _model_num_labels(config),
        "label_space": _label_space(config),
        "seed": config["experiment"]["seed"],
    })
    (output_root / "latest_run.txt").write_text(run_id + "\n", encoding="utf-8")
    return run_dir


def train(config: dict, config_path: Path) -> Path:
    """Train the configured SegFormer and save best/last checkpoints."""
    torch.manual_seed(config["experiment"]["seed"])
    split_root, _, _ = _paths(config)
    source = _load_source(config)
    label_space = _label_space(config)
    train_source, train_indices = _manifest_indices(split_root, "train")
    validation_source, validation_indices = _manifest_indices(split_root, "validation")
    train_dataset = TaskSegmentationDataset(
        source[train_source], train_indices, config["data"]["image_size"], label_space
    )
    validation_dataset = TaskSegmentationDataset(
        source[validation_source], validation_indices, config["data"]["image_size"], label_space
    )
    loader_args = {"batch_size": config["training"]["batch_size"], "num_workers": config["training"]["num_workers"]}
    train_loader = DataLoader(train_dataset, shuffle=True, **loader_args)
    validation_loader = DataLoader(validation_dataset, shuffle=False, **loader_args)
    device = select_training_device()
    configure_segformer_backend(device)
    model = build_segformer(
        config["model"]["checkpoint"], _model_num_labels(config)
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=config["training"]["learning_rate"], weight_decay=config["training"]["weight_decay"])
    run_dir = _new_run(config, config_path, device)
    history: list[dict[str, object]] = []
    progress: list[dict[str, object]] = []
    best_miou = -1.0
    training_started = time.perf_counter()
    progress_interval = config["training"].get("progress_interval_batches", 25)
    total_epochs = config["training"]["epochs"]
    total_batches = len(train_loader)
    total_training_batches = total_epochs * total_batches
    _record_progress(run_dir, progress, {
        "event": "estimate",
        "estimated_duration_minutes": config["training"]["estimated_duration_minutes"],
        **_resource_snapshot(),
    })
    for epoch in range(1, config["training"]["epochs"] + 1):
        model.train()
        losses = []
        for batch_number, batch in enumerate(train_loader, start=1):
            optimizer.zero_grad()
            labels = batch["labels"].to(device)
            logits = model(pixel_values=batch["pixel_values"].to(device)).logits
            logits = functional.interpolate(logits, size=labels.shape[-2:], mode="bilinear", align_corners=False)
            loss = functional.cross_entropy(
                logits.contiguous(), labels.contiguous(), ignore_index=IGNORE_LABEL
            )
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
            if batch_number % progress_interval == 0 or batch_number == len(train_loader):
                completed_batches = (epoch - 1) * total_batches + batch_number
                elapsed_seconds = time.perf_counter() - training_started
                eta_seconds = (
                    elapsed_seconds / completed_batches
                    * (total_training_batches - completed_batches)
                )
                _record_progress(run_dir, progress, {
                    "event": "batch",
                    "epoch": epoch,
                    "total_epochs": total_epochs,
                    "batch": batch_number,
                    "total_batches": total_batches,
                    "mean_train_loss": float(np.mean(losses)),
                    "elapsed_seconds": elapsed_seconds,
                    "eta_seconds": eta_seconds,
                    **_resource_snapshot(),
                })
        validation_metrics, _ = _evaluate_loader(
            model, validation_loader, device, label_space
        )
        row = {
            "epoch": epoch,
            "train_loss": float(np.mean(losses)),
            "validation_miou": validation_metrics["relevant_macro_miou"],
            "validation_recall": validation_metrics["relevant_macro_recall"],
        }
        history.append(row)
        write_rows(run_dir / "logs" / "training_history.jsonl", history)
        elapsed_seconds = time.perf_counter() - training_started
        _record_progress(run_dir, progress, {
            "event": "epoch",
            **row,
            "total_epochs": total_epochs,
            "elapsed_seconds": elapsed_seconds,
            "eta_seconds": (
                elapsed_seconds / epoch
                * (total_epochs - epoch)
            ),
            **_resource_snapshot(),
        })
        if row["validation_miou"] > best_miou:
            best_miou = float(row["validation_miou"])
            save_model(model, run_dir / "checkpoints" / "best")
    save_model(model, run_dir / "checkpoints" / "last")
    plot_training_history(history, run_dir / "figures" / "training_curves.png")
    return run_dir


def _synchronize(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elif device.type == "mps":
        torch.mps.synchronize()


def evaluate(config: dict, run_dir: Path) -> dict[str, object]:
    """Evaluate the best checkpoint and write all approved result artifacts."""
    split_root, _, _ = _paths(config)
    source = _load_source(config)
    label_space = _label_space(config)
    source_name, indices = _manifest_indices(split_root, "test")
    dataset = TaskSegmentationDataset(
        source[source_name], indices, config["data"]["image_size"], label_space
    )
    loader = DataLoader(dataset, batch_size=config["training"]["batch_size"], shuffle=False)
    device = select_device()
    configure_segformer_backend(device)
    model = load_model(run_dir / "checkpoints" / "best").to(device)
    metrics, matrix = _evaluate_loader(model, loader, device, label_space)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))

    latency_samples = []
    sample = dataset[0]["pixel_values"].unsqueeze(0).to(device)
    with torch.no_grad():
        for _ in range(config["evaluation"]["warmup_runs"]):
            model(pixel_values=sample)
        _synchronize(device)
        for _ in range(config["evaluation"]["latency_runs"]):
            start = time.perf_counter()
            logits = model(pixel_values=sample).logits
            functional.interpolate(logits, size=(config["data"]["image_size"], config["data"]["image_size"]), mode="bilinear", align_corners=False).argmax(dim=1).cpu().numpy()
            _synchronize(device)
            latency_samples.append((time.perf_counter() - start) * 1000)
    summary = {
        "relevant_macro_miou": metrics["relevant_macro_miou"],
        "relevant_macro_recall": metrics["relevant_macro_recall"],
        "mean_inference_latency_ms": float(np.mean(latency_samples)),
        "training_device": manifest["training_device"],
        "evaluation_device": str(device),
        "test_images": len(dataset),
    }
    write_json(run_dir / "metrics" / "summary.json", summary)
    write_rows(run_dir / "metrics" / "per_class.jsonl", metrics["per_class"])
    write_json(run_dir / "metrics" / "confusion_matrix.json", matrix.tolist())
    plot_per_class(metrics["per_class"], run_dir / "figures" / "per_class_metrics.png")
    plot_confusion(matrix, run_dir / "figures" / "confusion_matrix.png")
    _write_predictions(config, run_dir, model, device, source[source_name], indices)
    _write_summary(run_dir, summary)
    _update_comparisons(config, run_dir, summary)
    return summary


def _write_predictions(config, run_dir, model, device, source, indices) -> None:
    scored = []
    label_space = _label_space(config)
    model.eval()
    with torch.no_grad():
        for source_index in indices:
            item = TaskSegmentationDataset(
                source, [source_index], config["data"]["image_size"], label_space
            )[0]
            logits = model(pixel_values=item["pixel_values"].unsqueeze(0).to(device)).logits
            probabilities = functional.interpolate(logits, size=item["labels"].shape, mode="bilinear", align_corners=False).softmax(dim=1)[0].cpu().numpy()
            prediction = task_labels_from_model_prediction(
                probabilities.argmax(axis=0), label_space
            )
            target = item["task_labels"].numpy()
            classes = _task_classes_for_annotation(
                np.asarray(source[source_index]["annotation"]), label_space
            )
            evaluated = target != IGNORE_LABEL
            sample_metrics = metrics_from_confusion(
                confusion_matrix(target[evaluated], prediction[evaluated])
            )
            relevant_miou = sample_metrics["relevant_macro_miou"]
            relevance = task_relevance_from_model_probabilities(probabilities, label_space)
            if classes:
                scored.append(
                    (source_index, classes, relevant_miou, target, prediction, relevance)
                )
    selections = select_prediction_extremes(
        scored, config["evaluation"]["prediction_count"]
    )
    prediction_root = run_dir / "predictions"
    for path in prediction_root.rglob("*.png"):
        path.unlink()
    for gallery_path in (run_dir / "figures").glob("*prediction_gallery.png"):
        gallery_path.unlink()
    for group_name, selected in selections.items():
        group_root = prediction_root / group_name
        group_root.mkdir(parents=True, exist_ok=True)
        images = []
        for source_index, _, _, target, prediction, relevance in selected:
            image = source[source_index]["image"].convert("RGB").resize((config["data"]["image_size"], config["data"]["image_size"]))
            path = group_root / f"{source_index}.png"
            plot_prediction(image, target, prediction, relevance, path)
            images.append(plt.imread(path))
        fig, axes = plt.subplots(len(images), 1, figsize=(16, 4 * len(images)))
        if len(images) == 1:
            axes = [axes]
        for axis, image in zip(axes, images):
            axis.imshow(image)
            axis.axis("off")
        fig.tight_layout()
        fig.savefig(
            run_dir / "figures" / f"{group_name}_prediction_gallery.png", dpi=100
        )
        plt.close(fig)


def select_prediction_extremes(scored: list[tuple], count: int) -> dict[str, list[tuple]]:
    """Select the highest- and lowest-relevant-mIoU prediction visualizations."""
    ordered = sorted(scored, key=lambda value: value[2])
    return {"best": ordered[-count:][::-1], "worst": ordered[:count]}


def _write_summary(run_dir: Path, summary: dict[str, object]) -> None:
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    text = (
        "# Experiment Summary\n\n"
        f"- Stage: {manifest['experiment_stage']}\n"
        f"- Purpose: {manifest['purpose']}\n"
        f"- Relevant Macro mIoU: {summary['relevant_macro_miou']:.4f}\n"
        f"- Relevant Macro Recall: {summary['relevant_macro_recall']:.4f}\n"
        f"- Mean inference latency: {summary['mean_inference_latency_ms']:.2f} ms\n"
        f"- Training device: {summary['training_device']}\n"
        f"- Evaluation device: {summary['evaluation_device']}\n"
        f"- Test images: {summary['test_images']}\n\n"
        "Prediction figures include the lowest- and highest-error test samples "
        "and do not affect aggregate test metrics.\n\n"
        "The relevance heatmap sums probabilities for the 13 task-relevant ADE "
        "classes and is not a key probability "
        "or true exploration utility.\n"
    )
    (run_dir / "experiment_summary.md").write_text(text, encoding="utf-8")


def _update_comparisons(config, run_dir, summary) -> None:
    _, output_root, _ = _paths(config)
    comparison_root = output_root / "comparisons"
    comparison_root.mkdir(parents=True, exist_ok=True)
    table_path = comparison_root / "runs.jsonl"
    rows = []
    if table_path.exists():
        with table_path.open(encoding="utf-8") as handle:
            rows.extend(json.loads(line) for line in handle if line.strip())
    rows = [row for row in rows if row["run_id"] != run_dir.name]
    rows.append({"run_id": run_dir.name, **summary})
    write_rows(table_path, rows)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar([row["run_id"] for row in rows], [float(row["relevant_macro_miou"]) for row in rows])
    ax.set_ylabel("Relevant Macro mIoU")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(comparison_root / "model_comparison.png", dpi=160)
    plt.close(fig)


def latest_run(config: dict) -> Path:
    """Resolve the latest run created by the training command."""
    _, output_root, _ = _paths(config)
    run_id = (output_root / "latest_run.txt").read_text(encoding="utf-8").strip()
    return output_root / "runs" / run_id


def run_status(config: dict) -> dict[str, object]:
    """Read the latest run state and most recent on-disk progress record."""
    run_dir = latest_run(config)
    if (run_dir / "metrics" / "summary.json").exists():
        state = "evaluated"
    elif (run_dir / "checkpoints" / "last" / "model.safetensors").exists():
        state = "trained"
    else:
        state = "running_or_interrupted"
    result: dict[str, object] = {"run_id": run_dir.name, "state": state}
    progress_path = run_dir / "logs" / "progress.jsonl"
    if progress_path.exists():
        rows = [json.loads(line) for line in progress_path.read_text().splitlines() if line]
        if rows:
            result["latest_progress"] = rows[-1]
    return result
