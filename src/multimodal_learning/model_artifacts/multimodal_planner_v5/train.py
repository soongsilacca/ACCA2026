from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

from multimodal_planner_v5.data import (
    PlannerDataset,
    deterministic_run_split,
    load_split_manifest,
    save_split_manifest,
)
from multimodal_planner_v5.model import (
    ModelConfig,
    MultiViewTemporalTrajectoryPlannerV5,
)
from multimodal_planner_v5.metrics import (
    TrajectoryMetricAccumulator,
    trajectory_diagnostics,
)
from multimodal_planner_v5.outliers import TopOutlierCollector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("training_outputs/multimodal_planner_v5"),
    )
    parser.add_argument(
        "--split-manifest",
        type=Path,
        default=Path("multimodal_planner_v5/splits/v001.json"),
    )
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--grad-accum", type=int, default=8)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--backbone-lr", type=float, default=2e-5)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--warmup-epochs", type=int, default=3)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--blackout-weight", type=float, default=2.0)
    parser.add_argument("--start-weight", type=float, default=1.0)
    parser.add_argument("--step-weight", type=float, default=0.5)
    parser.add_argument("--acc-weight", type=float, default=0.25)
    parser.add_argument("--yaw-weight", type=float, default=0.2)
    parser.add_argument("--heading-weight", type=float, default=0.1)
    parser.add_argument("--speed-weight", type=float, default=0.4)
    parser.add_argument(
        "--no-photometric-augmentation",
        action="store_true",
        help="disable V5 train-only color/brightness/contrast/fog augmentation",
    )
    parser.add_argument(
        "--outlier-count",
        type=int,
        default=24,
        help="validation outliers retained per overall/blackout gallery; 0 disables",
    )
    parser.add_argument("--max-train-samples", type=int, default=0)
    parser.add_argument("--max-val-samples", type=int, default=0)
    parser.add_argument("--log-every", type=int, default=20)
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--resume", type=Path)
    parser.add_argument(
        "--init-checkpoint",
        type=Path,
        help="initialize model weights only; optimizer and epoch start fresh",
    )
    parser.add_argument(
        "--allow-legacy-target-fields",
        action="store_true",
        help=(
            "explicitly map future_xy/future_yaw to the V5 relative target "
            "contract without changing numeric values"
        ),
    )
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument(
        "--no-amp",
        action="store_true",
        help="run forward/backward in FP32 instead of CUDA mixed precision",
    )
    parser.add_argument("--cpu", action="store_true")
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def discover_runs(data_root: Path) -> list[str]:
    return sorted(
        path.name
        for path in data_root.iterdir()
        if path.is_dir()
        and (path / "sample_index.npz").is_file()
        and (path / "frame_chunks.json").is_file()
    )


def planner_loss(
    outputs: dict[str, torch.Tensor],
    target: torch.Tensor,
    sample_weight: torch.Tensor,
    sample_weight_normalizer: float = 1.0,
    start_weight: float = 1.0,
    step_weight: float = 0.5,
    acc_weight: float = 0.25,
    yaw_weight: float = 0.2,
    heading_weight: float = 0.1,
    speed_weight: float = 0.4,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    if sample_weight_normalizer <= 0.0:
        raise ValueError("sample_weight_normalizer must be positive")
    weights = (
        start_weight,
        step_weight,
        acc_weight,
        yaw_weight,
        heading_weight,
        speed_weight,
    )
    if any(weight < 0.0 for weight in weights):
        raise ValueError("loss weights must be non-negative")
    prediction = outputs["trajectory"]
    if prediction.shape != target.shape:
        raise ValueError(
            "V5 prediction and target must both have shape [batch, horizon, 4], "
            f"got {tuple(prediction.shape)} and {tuple(target.shape)}"
        )

    position = F.smooth_l1_loss(
        prediction[..., :2], target[..., :2], reduction="none"
    ).mean(dim=(-1, -2))

    start = F.smooth_l1_loss(
        prediction[:, 0, :2], target[:, 0, :2], reduction="none"
    ).mean(dim=-1)

    # V5 supervises the controller boundary explicitly. Both sequences start
    # at the current ego-frame origin, so the first derivative includes
    # (0,0) -> first future waypoint instead of starting at p1 -> p2.
    origin_xy = torch.zeros_like(prediction[:, :1, :2])
    prediction_xy_with_origin = torch.cat(
        (origin_xy, prediction[..., :2]), dim=1
    )
    target_xy_with_origin = torch.cat((origin_xy, target[..., :2]), dim=1)
    prediction_step = (
        prediction_xy_with_origin[:, 1:]
        - prediction_xy_with_origin[:, :-1]
    )
    target_step = target_xy_with_origin[:, 1:] - target_xy_with_origin[:, :-1]
    step = F.smooth_l1_loss(
        prediction_step, target_step, reduction="none"
    ).mean(dim=(-1, -2))

    prediction_acc = prediction_step[:, 1:] - prediction_step[:, :-1]
    target_acc = target_step[:, 1:] - target_step[:, :-1]
    acceleration = F.smooth_l1_loss(
        prediction_acc, target_acc, reduction="none"
    ).mean(dim=(-1, -2))

    speed = F.smooth_l1_loss(
        prediction[..., 3], target[..., 3], reduction="none"
    ).mean(dim=-1)
    yaw_delta = (prediction[..., 2] - target[..., 2]) * math.pi
    yaw = (1.0 - torch.cos(yaw_delta)).mean(dim=-1)

    # Couple predicted yaw to the direction implied by predicted XY. Segments
    # where GT barely moves are excluded because path heading is undefined.
    # Compare cyclic directions without differentiating through atan2. Near a
    # zero-length predicted segment, atan2 has an ill-conditioned gradient and
    # can corrupt an optimizer update even when the forward loss is finite.
    prediction_step_float = prediction_step.float()
    origin_yaw = torch.zeros_like(prediction[:, :1, 2])
    prediction_segment_yaw = torch.cat(
        (origin_yaw, prediction[:, :-1, 2]), dim=1
    )
    prediction_yaw_rad = prediction_segment_yaw.float() * math.pi
    prediction_step_norm = torch.linalg.vector_norm(
        prediction_step_float, dim=-1, keepdim=True
    )
    prediction_direction = prediction_step_float / prediction_step_norm.clamp_min(
        1e-3
    )
    yaw_direction = torch.stack(
        (torch.cos(prediction_yaw_rad), torch.sin(prediction_yaw_rad)), dim=-1
    )
    heading_per_step = 1.0 - (
        prediction_direction * yaw_direction
    ).sum(dim=-1).clamp(-1.0, 1.0)
    moving = torch.linalg.vector_norm(target_step.float(), dim=-1) > 1e-4
    heading = (
        (heading_per_step * moving).sum(dim=-1)
        / moving.sum(dim=-1).clamp_min(1)
    )

    total_per_sample = (
        position
        + start_weight * start
        + step_weight * step
        + acc_weight * acceleration
        + yaw_weight * yaw
        + heading_weight * heading
        + speed_weight * speed
    )
    # Normalize by the fixed dataset-wide mean weight instead of the current
    # micro-batch sum. This preserves relative sample weights when batch_size=1
    # and remains correct when gradients are accumulated across micro-batches.
    def weighted_mean(value: torch.Tensor) -> torch.Tensor:
        return (value * sample_weight).mean() / sample_weight_normalizer

    total = weighted_mean(total_per_sample)
    terms = {
        "loss": total.detach(),
        "position": weighted_mean(position).detach(),
        "start": weighted_mean(start).detach(),
        "step": weighted_mean(step).detach(),
        "acceleration": weighted_mean(acceleration).detach(),
        "speed": weighted_mean(speed).detach(),
        "yaw": weighted_mean(yaw).detach(),
        "heading": weighted_mean(heading).detach(),
    }
    return total, terms


def move_batch(batch: dict[str, Any], device: torch.device) -> dict[str, Any]:
    return {
        key: value.to(device, non_blocking=True)
        if isinstance(value, torch.Tensor)
        else value
        for key, value in batch.items()
    }


def forward_batch(
    model: MultiViewTemporalTrajectoryPlannerV5, batch: dict[str, Any]
) -> dict[str, torch.Tensor]:
    return model(
        batch["front"],
        batch["left"],
        batch["right"],
        batch["lidar_bev"],
        batch["ego"],
        batch["mgeo"],
        batch["local_route"],
    )


def _nonfinite_names(
    *named_mappings: tuple[str, dict[str, torch.Tensor]],
) -> list[str]:
    bad: list[str] = []
    for prefix, mapping in named_mappings:
        for name, value in mapping.items():
            if isinstance(value, torch.Tensor) and not bool(torch.isfinite(value).all()):
                bad.append(f"{prefix}.{name}")
    return bad


def _batch_identity(batch: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": [str(value) for value in batch["run_id"]],
        "sample_id": [
            int(value) for value in batch["sample_id"].detach().cpu().tolist()
        ],
        "gps_blackout": [
            bool(value)
            for value in batch["gps_blackout"].detach().cpu().tolist()
        ],
    }


@torch.no_grad()
def evaluate(
    model: MultiViewTemporalTrajectoryPlannerV5,
    loader: DataLoader,
    device: torch.device,
    amp_enabled: bool,
    sample_weight_normalizer: float,
    loss_weights: tuple[float, float, float, float, float, float],
    outlier_collector: TopOutlierCollector | None = None,
) -> dict[str, Any]:
    model.eval()
    totals: dict[str, float] = {}
    count = 0
    trajectory_metrics = TrajectoryMetricAccumulator()
    for batch in loader:
        batch = move_batch(batch, device)
        with torch.autocast(
            device_type=device.type,
            dtype=torch.float16,
            enabled=amp_enabled,
        ):
            outputs = forward_batch(model, batch)
            _, terms = planner_loss(
                outputs,
                batch["target"],
                batch["sample_weight"],
                sample_weight_normalizer,
                *loss_weights,
            )
            diagnostics = trajectory_diagnostics(outputs, batch["target"])
        bad = _nonfinite_names(
            ("outputs", outputs), ("loss_terms", terms), ("diagnostics", diagnostics)
        )
        if bad and amp_enabled:
            print(
                "nonfinite_fallback "
                + json.dumps(
                    {
                        "phase": "validation",
                        "bad": bad,
                        **_batch_identity(batch),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=False,
            ):
                outputs = forward_batch(model, batch)
                _, terms = planner_loss(
                    outputs,
                    batch["target"],
                    batch["sample_weight"],
                    sample_weight_normalizer,
                    *loss_weights,
                )
                diagnostics = trajectory_diagnostics(outputs, batch["target"])
            bad = _nonfinite_names(
                ("outputs", outputs),
                ("loss_terms", terms),
                ("diagnostics", diagnostics),
            )
        if bad:
            raise FloatingPointError(
                "non-finite validation tensors after FP32 retry: "
                + json.dumps(
                    {"bad": bad, **_batch_identity(batch)}, sort_keys=True
                )
            )
        if outlier_collector is not None:
            outlier_collector.update(outputs, batch, diagnostics)
        trajectory_metrics.update(diagnostics, batch["gps_blackout"])
        batch_count = batch["target"].shape[0]
        count += batch_count
        for key, value in terms.items():
            totals[key] = totals.get(key, 0.0) + float(value) * batch_count
    result: dict[str, Any] = {
        key: value / max(count, 1) for key, value in totals.items()
    }
    result["trajectory_metrics"] = trajectory_metrics.result()
    return result


def make_optimizer(
    model: MultiViewTemporalTrajectoryPlannerV5,
    lr: float,
    backbone_lr: float,
    weight_decay: float,
) -> torch.optim.Optimizer:
    backbone_parameters = list(model.camera_encoder.backbone.parameters())
    backbone_ids = {id(parameter) for parameter in backbone_parameters}
    other_parameters = [
        parameter
        for parameter in model.parameters()
        if id(parameter) not in backbone_ids
    ]
    return torch.optim.AdamW(
        [
            {"params": other_parameters, "lr": lr},
            {"params": backbone_parameters, "lr": backbone_lr},
        ],
        weight_decay=weight_decay,
    )


def main() -> None:
    args = parse_args()
    set_seed(args.seed)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    run_ids = discover_runs(args.data_root)
    if args.split_manifest.exists():
        splits = load_split_manifest(args.split_manifest)
    else:
        splits = deterministic_run_split(run_ids, args.seed)
        save_split_manifest(
            args.split_manifest, splits, args.seed, args.data_root
        )
    missing = set(sum(splits.values(), [])) - set(run_ids)
    if missing:
        raise RuntimeError(f"split manifest references missing runs: {sorted(missing)}")

    train_dataset = PlannerDataset(
        args.data_root,
        splits["train"],
        blackout_weight=args.blackout_weight,
        max_samples=args.max_train_samples,
        seed=args.seed,
        allow_legacy_target_fields=args.allow_legacy_target_fields,
        photometric_augmentation=not args.no_photometric_augmentation,
    )
    val_dataset = PlannerDataset(
        args.data_root,
        splits["val"],
        blackout_weight=1.0,
        max_samples=args.max_val_samples,
        seed=args.seed + 1,
        allow_legacy_target_fields=args.allow_legacy_target_fields,
        photometric_augmentation=False,
    )
    print("train_dataset", json.dumps(train_dataset.summary(), sort_keys=True))
    print("val_dataset", json.dumps(val_dataset.summary(), sort_keys=True))
    print(
        "loss_weighting",
        json.dumps(
            {
                "policy": "dataset_mean_normalized_per_sample_weight",
                "train_mean_sample_weight": train_dataset.mean_sample_weight,
                "val_mean_sample_weight": val_dataset.mean_sample_weight,
                "blackout_relative_weight": args.blackout_weight,
                "batch_size_safe": True,
                "gradient_accumulation_safe": True,
            },
            sort_keys=True,
        ),
    )

    loader_kwargs = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": torch.cuda.is_available() and not args.cpu,
        "persistent_workers": args.num_workers > 0,
    }
    train_loader = DataLoader(
        train_dataset, shuffle=True, drop_last=False, **loader_kwargs
    )
    val_loader = DataLoader(
        val_dataset, shuffle=False, drop_last=False, **loader_kwargs
    )

    device = torch.device(
        "cpu" if args.cpu or not torch.cuda.is_available() else "cuda"
    )
    config = ModelConfig(
        pretrained_camera=not args.no_pretrained,
        freeze_camera_backbone=args.warmup_epochs > 0,
    )
    model = MultiViewTemporalTrajectoryPlannerV5(config).to(device)
    if args.resume and args.init_checkpoint:
        raise ValueError("--resume and --init-checkpoint are mutually exclusive")
    if args.init_checkpoint:
        initialization = torch.load(
            args.init_checkpoint, map_location=device, weights_only=False
        )
        model.load_state_dict(initialization["model_state"], strict=True)
        print(
            "initialization",
            json.dumps(
                {
                    "policy": "model_weights_only_fresh_optimizer",
                    "checkpoint": str(args.init_checkpoint.resolve()),
                    "source_epoch": int(initialization.get("epoch", -1)) + 1,
                },
                sort_keys=True,
            ),
        )
    print("device", device)
    print("parameters", json.dumps(model.parameter_counts(), sort_keys=True))
    optimizer = make_optimizer(
        model, args.lr, args.backbone_lr, args.weight_decay
    )
    scaler = torch.amp.GradScaler(
        "cuda", enabled=device.type == "cuda" and not args.no_amp
    )
    amp_enabled = device.type == "cuda" and not args.no_amp
    print(
        "precision",
        json.dumps(
            {
                "autocast": amp_enabled,
                "dtype": "float16" if amp_enabled else "float32",
            },
            sort_keys=True,
        ),
    )
    start_epoch = 0
    best_val = float("inf")
    history: list[dict[str, Any]] = []
    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(checkpoint["model_state"])
        optimizer.load_state_dict(checkpoint["optimizer_state"])
        scaler.load_state_dict(checkpoint["scaler_state"])
        start_epoch = int(checkpoint["epoch"]) + 1
        best_val = float(checkpoint["best_val"])
        history = list(checkpoint.get("history", []))

    for epoch in range(start_epoch, args.epochs):
        backbone_trainable = epoch >= args.warmup_epochs
        model.set_camera_backbone_trainable(backbone_trainable)
        model.train()
        optimizer.zero_grad(set_to_none=True)
        running: dict[str, float] = {}
        sample_count = 0
        fp32_fallback_count = 0
        epoch_start = time.time()
        for step, batch in enumerate(train_loader):
            batch = move_batch(batch, device)
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=amp_enabled,
            ):
                outputs = forward_batch(model, batch)
                loss, terms = planner_loss(
                    outputs,
                    batch["target"],
                    batch["sample_weight"],
                    train_dataset.mean_sample_weight,
                    args.start_weight,
                    args.step_weight,
                    args.acc_weight,
                    args.yaw_weight,
                    args.heading_weight,
                    args.speed_weight,
                )
            bad = _nonfinite_names(
                ("outputs", outputs),
                ("loss_terms", terms),
                ("loss", {"total": loss}),
            )
            if bad and amp_enabled:
                fp32_fallback_count += 1
                print(
                    "nonfinite_fallback "
                    + json.dumps(
                        {
                            "phase": "train",
                            "epoch": epoch + 1,
                            "step": step + 1,
                            "bad": bad,
                            **_batch_identity(batch),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                with torch.autocast(
                    device_type=device.type,
                    dtype=torch.float16,
                    enabled=False,
                ):
                    outputs = forward_batch(model, batch)
                    loss, terms = planner_loss(
                        outputs,
                        batch["target"],
                        batch["sample_weight"],
                        train_dataset.mean_sample_weight,
                        args.start_weight,
                        args.step_weight,
                        args.acc_weight,
                        args.yaw_weight,
                        args.heading_weight,
                        args.speed_weight,
                    )
                bad = _nonfinite_names(
                    ("outputs", outputs),
                    ("loss_terms", terms),
                    ("loss", {"total": loss}),
                )
            if bad:
                raise FloatingPointError(
                    "non-finite training tensors after FP32 retry: "
                    + json.dumps(
                        {
                            "epoch": epoch + 1,
                            "step": step + 1,
                            "bad": bad,
                            **_batch_identity(batch),
                        },
                        sort_keys=True,
                    )
                )
            scaled_loss = loss / args.grad_accum
            scaler.scale(scaled_loss).backward()
            if (step + 1) % args.grad_accum == 0 or step + 1 == len(train_loader):
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(
                    model.parameters(),
                    5.0,
                    error_if_nonfinite=not amp_enabled,
                )
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)

            batch_count = batch["target"].shape[0]
            sample_count += batch_count
            for key, value in terms.items():
                running[key] = running.get(key, 0.0) + float(value) * batch_count
            if (step + 1) % args.log_every == 0:
                metrics = {
                    key: value / sample_count for key, value in running.items()
                }
                print(
                    f"epoch={epoch + 1}/{args.epochs} step={step + 1}/"
                    f"{len(train_loader)} train={json.dumps(metrics, sort_keys=True)}",
                    flush=True,
                )

        train_metrics = {
            key: value / max(sample_count, 1) for key, value in running.items()
        }
        outlier_collector = (
            TopOutlierCollector(args.outlier_count)
            if args.outlier_count > 0
            else None
        )
        val_metrics = evaluate(
            model,
            val_loader,
            device,
            amp_enabled,
            val_dataset.mean_sample_weight,
            (
                args.start_weight,
                args.step_weight,
                args.acc_weight,
                args.yaw_weight,
                args.heading_weight,
                args.speed_weight,
            ),
            outlier_collector,
        )
        outlier_summary = (
            outlier_collector.export(args.output_dir / "outliers", epoch + 1)
            if outlier_collector is not None
            else None
        )
        record = {
            "epoch": epoch,
            "backbone_trainable": backbone_trainable,
            "seconds": time.time() - epoch_start,
            "fp32_fallback_count": fp32_fallback_count,
            "train": train_metrics,
            "val": val_metrics,
            "outliers": outlier_summary,
        }
        history.append(record)
        print("epoch_result", json.dumps(record, sort_keys=True), flush=True)
        metrics_dir = args.output_dir / "metrics"
        metrics_dir.mkdir(parents=True, exist_ok=True)
        (metrics_dir / f"epoch_{epoch + 1:03d}.json").write_text(
            json.dumps(val_metrics, indent=2), encoding="utf-8"
        )

        state = {
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "scaler_state": scaler.state_dict(),
            "epoch": epoch,
            "best_val": min(best_val, val_metrics["loss"]),
            "history": history,
            "model_config": vars(config),
            "train_args": vars(args),
            "splits": splits,
            "parameter_counts": model.parameter_counts(),
            "loss_weighting": {
                "policy": "dataset_mean_normalized_per_sample_weight",
                "train_mean_sample_weight": train_dataset.mean_sample_weight,
                "val_mean_sample_weight": val_dataset.mean_sample_weight,
                "start_weight": args.start_weight,
                "step_weight": args.step_weight,
                "acc_weight": args.acc_weight,
                "yaw_weight": args.yaw_weight,
                "heading_weight": args.heading_weight,
                "speed_weight": args.speed_weight,
            },
            "target_contract": {
                "fields": [
                    "relative_x",
                    "relative_y",
                    "relative_yaw",
                    "future_speed",
                ],
                "shape": [20, 4],
                "controller_shape_with_origin": [21, 4],
                "boundary_policy": (
                    "origin_to_first_future_step_acceleration_and_heading_supervised"
                ),
                "label_policy": "provided_relative_values_without_correction",
            },
        }
        torch.save(state, args.output_dir / "latest.pt")
        if val_metrics["loss"] < best_val:
            best_val = val_metrics["loss"]
            state["best_val"] = best_val
            torch.save(state, args.output_dir / "best.pt")
        if (epoch + 1) % args.save_every == 0:
            torch.save(state, args.output_dir / f"epoch_{epoch + 1:03d}.pt")
        (args.output_dir / "history.json").write_text(
            json.dumps(history, indent=2), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
