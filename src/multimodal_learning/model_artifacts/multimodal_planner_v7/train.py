from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader

from multimodal_planner_v5.outliers import TopOutlierCollector
from multimodal_planner_v7.data import (
    PlannerDataset,
    deterministic_run_split,
    load_split_manifest,
    save_split_manifest,
)
from multimodal_planner_v7.losses import planner_loss
from multimodal_planner_v7.metrics import (
    MotionStateMetricAccumulator,
    SpeedMetricAccumulator,
    TrajectoryMetricAccumulator,
    trajectory_diagnostics,
)
from multimodal_planner_v7.model import ModelConfig, RouteLockedSpeedPlannerV7


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("training_outputs/multimodal_planner_v7"),
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
    parser.add_argument("--head-warmup-epochs", type=int, default=1)
    parser.add_argument("--backbone-warmup-epochs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--blackout-weight", type=float, default=2.0)
    parser.add_argument("--stop-sample-weight", type=float, default=4.0)
    parser.add_argument("--drive-sample-weight", type=float, default=1.0)
    parser.add_argument("--avoidance-weight", type=float, default=6.0)
    parser.add_argument("--avoidance-threshold-m", type=float, default=0.75)
    parser.add_argument("--stop-class-weight", type=float, default=4.0)
    parser.add_argument("--drive-class-weight", type=float, default=1.0)
    parser.add_argument(
        "--normal-base-speed-mps",
        type=float,
        default=59.0 / 3.6,
    )
    parser.add_argument(
        "--speed-zone-base-speed-mps",
        type=float,
        default=20.0,
    )
    parser.add_argument("--stop-speed-threshold", type=float, default=0.6)
    parser.add_argument("--stop-max-endpoint-distance", type=float, default=10.0)
    parser.add_argument("--speed-weight", type=float, default=1.0)
    parser.add_argument("--speed-acceleration-weight", type=float, default=0.25)
    parser.add_argument("--speed-jerk-weight", type=float, default=0.1)
    parser.add_argument("--progress-weight", type=float, default=0.25)
    parser.add_argument("--lateral-weight", type=float, default=1.0)
    parser.add_argument("--lateral-step-weight", type=float, default=0.5)
    parser.add_argument(
        "--lateral-acceleration-weight",
        type=float,
        default=0.25,
    )
    parser.add_argument("--state-weight", type=float, default=0.2)
    parser.add_argument(
        "--speed-delta-regularization-weight",
        type=float,
        default=0.01,
    )
    parser.add_argument("--outlier-count", type=int, default=24)
    parser.add_argument("--max-train-samples", type=int, default=0)
    parser.add_argument("--max-val-samples", type=int, default=0)
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--init-checkpoint", type=Path)
    parser.add_argument("--allow-legacy-target-fields", action="store_true")
    parser.add_argument("--no-photometric-augmentation", action="store_true")
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument("--no-amp", action="store_true")
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


def move_batch(batch: dict[str, Any], device: torch.device) -> dict[str, Any]:
    return {
        key: value.to(device, non_blocking=True)
        if isinstance(value, torch.Tensor)
        else value
        for key, value in batch.items()
    }


def forward_batch(
    model: RouteLockedSpeedPlannerV7,
    batch: dict[str, Any],
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


def nonfinite_names(
    *named_mappings: tuple[str, dict[str, torch.Tensor]],
) -> list[str]:
    bad = []
    for prefix, mapping in named_mappings:
        for name, value in mapping.items():
            if isinstance(value, torch.Tensor) and not bool(
                torch.isfinite(value).all()
            ):
                bad.append(f"{prefix}.{name}")
    return bad


def batch_identity(batch: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": [str(value) for value in batch["run_id"]],
        "sample_id": [
            int(value) for value in batch["sample_id"].detach().cpu().tolist()
        ],
        "gps_blackout": [
            bool(value)
            for value in batch["gps_blackout"].detach().cpu().tolist()
        ],
        "motion_state": [
            int(value)
            for value in batch["motion_state"].detach().cpu().tolist()
        ],
        "avoidance": [
            bool(value) for value in batch["avoidance"].detach().cpu().tolist()
        ],
    }


def loss_kwargs(
    args: argparse.Namespace,
    *,
    training: bool,
) -> dict[str, float]:
    return {
        "speed_weight": args.speed_weight,
        "acceleration_weight": args.speed_acceleration_weight,
        "jerk_weight": args.speed_jerk_weight,
        "progress_weight": args.progress_weight,
        "lateral_weight": args.lateral_weight,
        "lateral_step_weight": args.lateral_step_weight,
        "lateral_acceleration_weight": args.lateral_acceleration_weight,
        "state_weight": args.state_weight,
        "stop_class_weight": args.stop_class_weight if training else 1.0,
        "drive_class_weight": args.drive_class_weight if training else 1.0,
        "speed_delta_regularization_weight": (
            args.speed_delta_regularization_weight
        ),
    }


def compute_loss(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, Any],
    normalizer: float,
    weights: dict[str, float],
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    return planner_loss(
        outputs,
        batch["target"],
        batch["target_route_progress_m"],
        batch["target_lateral_residual_m"],
        batch["motion_state"],
        batch["sample_weight"],
        normalizer,
        **weights,
    )


@torch.no_grad()
def evaluate(
    model: RouteLockedSpeedPlannerV7,
    loader: DataLoader,
    device: torch.device,
    amp_enabled: bool,
    sample_weight_normalizer: float,
    weights: dict[str, float],
    outlier_collector: TopOutlierCollector | None,
) -> dict[str, Any]:
    model.eval()
    totals: dict[str, float] = {}
    count = 0
    trajectory_metrics = TrajectoryMetricAccumulator()
    state_metrics = MotionStateMetricAccumulator()
    speed_metrics = SpeedMetricAccumulator()
    for batch in loader:
        batch = move_batch(batch, device)
        with torch.autocast(
            device_type=device.type,
            dtype=torch.float16,
            enabled=amp_enabled,
        ):
            outputs = forward_batch(model, batch)
            _, terms = compute_loss(
                outputs,
                batch,
                sample_weight_normalizer,
                weights,
            )
            diagnostics = trajectory_diagnostics(outputs, batch["target"])
        bad = nonfinite_names(
            ("outputs", outputs),
            ("loss_terms", terms),
            ("diagnostics", diagnostics),
        )
        if bad and amp_enabled:
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=False,
            ):
                outputs = forward_batch(model, batch)
                _, terms = compute_loss(
                    outputs,
                    batch,
                    sample_weight_normalizer,
                    weights,
                )
                diagnostics = trajectory_diagnostics(outputs, batch["target"])
            bad = nonfinite_names(
                ("outputs", outputs),
                ("loss_terms", terms),
                ("diagnostics", diagnostics),
            )
        if bad:
            raise FloatingPointError(
                "non-finite validation tensors after FP32 retry: "
                + json.dumps(
                    {"bad": bad, **batch_identity(batch)},
                    sort_keys=True,
                )
            )
        if outlier_collector is not None:
            outlier_collector.update(outputs, batch, diagnostics)
        trajectory_metrics.update(diagnostics, batch["gps_blackout"])
        state_metrics.update(
            outputs["motion_state_logits"],
            batch["motion_state"],
            diagnostics,
        )
        speed_metrics.update(
            outputs,
            batch["target"],
            batch["target_lateral_residual_m"],
            batch["avoidance"],
        )
        batch_count = batch["target"].shape[0]
        count += batch_count
        for key, value in terms.items():
            totals[key] = totals.get(key, 0.0) + float(value) * batch_count
    result = {key: value / max(count, 1) for key, value in totals.items()}
    result["trajectory_metrics"] = trajectory_metrics.result()
    result["motion_state_metrics"] = state_metrics.result()
    result["speed_residual_metrics"] = speed_metrics.result()
    return result


def make_optimizer(
    model: RouteLockedSpeedPlannerV7,
    lr: float,
    backbone_lr: float,
    weight_decay: float,
) -> torch.optim.Optimizer:
    backbone_parameters = list(model.camera_encoder.backbone.parameters())
    backbone_ids = {id(parameter) for parameter in backbone_parameters}
    other_parameters = [
        parameter for parameter in model.parameters() if id(parameter) not in backbone_ids
    ]
    return torch.optim.AdamW(
        [
            {"params": other_parameters, "lr": lr},
            {"params": backbone_parameters, "lr": backbone_lr},
        ],
        weight_decay=weight_decay,
    )


def set_training_phase(
    model: RouteLockedSpeedPlannerV7,
    epoch: int,
    head_warmup_epochs: int,
    backbone_warmup_epochs: int,
) -> str:
    if epoch < head_warmup_epochs:
        for parameter in model.parameters():
            parameter.requires_grad = False
        for module in (
            model.speed_delta_head,
            model.lateral_head,
            model.state_head,
        ):
            for parameter in module.parameters():
                parameter.requires_grad = True
        model.set_camera_backbone_trainable(False)
        return "new_heads_only"
    for parameter in model.parameters():
        parameter.requires_grad = True
    backbone_trainable = epoch >= backbone_warmup_epochs
    model.set_camera_backbone_trainable(backbone_trainable)
    return "full" if backbone_trainable else "full_except_camera_backbone"


def build_dataset(
    args: argparse.Namespace,
    run_ids: list[str],
    *,
    training: bool,
) -> PlannerDataset:
    return PlannerDataset(
        args.data_root,
        run_ids,
        blackout_weight=args.blackout_weight if training else 1.0,
        stop_weight=args.stop_sample_weight if training else 1.0,
        drive_weight=args.drive_sample_weight if training else 1.0,
        avoidance_weight=args.avoidance_weight if training else 1.0,
        avoidance_threshold_m=args.avoidance_threshold_m,
        max_samples=(
            args.max_train_samples if training else args.max_val_samples
        ),
        seed=args.seed if training else args.seed + 1,
        allow_legacy_target_fields=args.allow_legacy_target_fields,
        photometric_augmentation=(
            training and not args.no_photometric_augmentation
        ),
        stop_speed_threshold_mps=args.stop_speed_threshold,
        stop_max_endpoint_distance_m=args.stop_max_endpoint_distance,
    )


def main() -> None:
    args = parse_args()
    if args.resume and args.init_checkpoint:
        raise ValueError("--resume and --init-checkpoint are mutually exclusive")
    set_seed(args.seed)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_ids = discover_runs(args.data_root)
    if args.split_manifest.exists():
        splits = load_split_manifest(args.split_manifest)
    else:
        splits = deterministic_run_split(run_ids, args.seed)
        save_split_manifest(args.split_manifest, splits, args.seed, args.data_root)
    missing = set(sum(splits.values(), [])) - set(run_ids)
    if missing:
        raise RuntimeError(f"split manifest references missing runs: {sorted(missing)}")

    train_dataset = build_dataset(args, splits["train"], training=True)
    val_dataset = build_dataset(args, splits["val"], training=False)
    print("train_dataset", json.dumps(train_dataset.summary(), sort_keys=True))
    print("val_dataset", json.dumps(val_dataset.summary(), sort_keys=True))

    loader_kwargs = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": torch.cuda.is_available() and not args.cpu,
        "persistent_workers": args.num_workers > 0,
    }
    train_loader = DataLoader(
        train_dataset,
        shuffle=True,
        drop_last=False,
        **loader_kwargs,
    )
    val_loader = DataLoader(
        val_dataset,
        shuffle=False,
        drop_last=False,
        **loader_kwargs,
    )
    device = torch.device(
        "cpu" if args.cpu or not torch.cuda.is_available() else "cuda"
    )

    initialization = None
    if args.resume:
        initialization = torch.load(args.resume, map_location=device, weights_only=False)
    elif args.init_checkpoint:
        initialization = torch.load(
            args.init_checkpoint,
            map_location=device,
            weights_only=False,
        )
    if initialization is not None:
        config_values = dict(initialization["model_config"])
        config_values["pretrained_camera"] = False
        config_values["freeze_camera_backbone"] = True
        config = ModelConfig(**config_values)
    else:
        config = ModelConfig(
            pretrained_camera=not args.no_pretrained,
            freeze_camera_backbone=True,
        )
    model = RouteLockedSpeedPlannerV7(
        config,
        normal_base_speed_mps=args.normal_base_speed_mps,
        speed_zone_base_speed_mps=args.speed_zone_base_speed_mps,
    ).to(device)

    initialization_report = None
    if args.init_checkpoint:
        initialization_report = {
            **model.load_encoder_state_dict(initialization["model_state"]),
            "policy": "V5/V6 encoders only; fresh V7 residual heads and optimizer",
            "checkpoint": str(args.init_checkpoint.resolve()),
            "source_epoch": int(initialization.get("epoch", -1)) + 1,
        }
        print("initialization", json.dumps(initialization_report, sort_keys=True))

    optimizer = make_optimizer(
        model,
        args.lr,
        args.backbone_lr,
        args.weight_decay,
    )
    amp_enabled = device.type == "cuda" and not args.no_amp
    scaler = torch.amp.GradScaler("cuda", enabled=amp_enabled)
    print("device", device)
    print("parameters", json.dumps(model.parameter_counts(), sort_keys=True))
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
        model.load_state_dict(initialization["model_state"], strict=True)
        optimizer.load_state_dict(initialization["optimizer_state"])
        scaler.load_state_dict(initialization["scaler_state"])
        start_epoch = int(initialization["epoch"]) + 1
        best_val = float(initialization["best_val"])
        history = list(initialization.get("history", []))

    train_weights = loss_kwargs(args, training=True)
    val_weights = loss_kwargs(args, training=False)
    for epoch in range(start_epoch, args.epochs):
        model.train()
        phase = set_training_phase(
            model,
            epoch,
            args.head_warmup_epochs,
            args.backbone_warmup_epochs,
        )
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
                loss, terms = compute_loss(
                    outputs,
                    batch,
                    train_dataset.mean_sample_weight,
                    train_weights,
                )
            bad = nonfinite_names(
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
                            **batch_identity(batch),
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
                    loss, terms = compute_loss(
                        outputs,
                        batch,
                        train_dataset.mean_sample_weight,
                        train_weights,
                    )
                bad = nonfinite_names(
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
                            **batch_identity(batch),
                        },
                        sort_keys=True,
                    )
                )
            scaler.scale(loss / args.grad_accum).backward()
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
                    f"epoch={epoch + 1}/{args.epochs} phase={phase} "
                    f"step={step + 1}/{len(train_loader)} "
                    f"train={json.dumps(metrics, sort_keys=True)}",
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
            val_weights,
            outlier_collector,
        )
        outlier_summary = (
            outlier_collector.export(args.output_dir / "outliers", epoch + 1)
            if outlier_collector is not None
            else None
        )
        record = {
            "epoch": epoch,
            "training_phase": phase,
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
            json.dumps(val_metrics, indent=2),
            encoding="utf-8",
        )

        state = {
            "architecture": "multimodal_planner_v7_local_route_residual",
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
            "initialization": initialization_report,
            "output_contract": {
                "future_speed": [20],
                "lateral_route_residual_m": [20],
                "motion_state": ["STOP", "DRIVE"],
                "derived_trajectory": [20, 4],
                "formula": (
                    "xy=LocalRoute(s)+delta_d*route_normal; "
                    "s=cumsum(future_speed*0.2s)"
                ),
                "base_speed": (
                    "fixed non-learned mapping from MGeo speed-zone flag"
                ),
                "normal_base_speed_mps": args.normal_base_speed_mps,
                "speed_zone_base_speed_mps": (
                    args.speed_zone_base_speed_mps
                ),
            },
            "label_policy": (
                "source trajectory unchanged; s and delta_d are deterministic "
                "projections onto the provided Local Route"
            ),
            "loss_weighting": {
                **train_weights,
                "blackout_sample_weight": args.blackout_weight,
                "stop_sample_weight": args.stop_sample_weight,
                "drive_sample_weight": args.drive_sample_weight,
                "avoidance_sample_weight": args.avoidance_weight,
                "avoidance_threshold_m": args.avoidance_threshold_m,
                "train_mean_sample_weight": train_dataset.mean_sample_weight,
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
            json.dumps(history, indent=2),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
