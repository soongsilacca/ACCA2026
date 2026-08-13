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

from multimodal_planner_v8.data import (
    SPATIAL_ANCHORS_M,
    PlannerDataset,
    deterministic_run_split,
    load_split_manifest,
    save_split_manifest,
)
from multimodal_planner_v8.losses import planner_loss
from multimodal_planner_v8.metrics import (
    MotionStateMetricAccumulator,
    SpatialResidualMetricAccumulator,
)
from multimodal_planner_v8.model import ModelConfig, SpatialResidualPlannerV8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("training_outputs/multimodal_planner_v8"),
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
    parser.add_argument("--stop-speed-threshold", type=float, default=0.6)
    parser.add_argument("--stop-max-endpoint-distance", type=float, default=10.0)
    parser.add_argument("--lateral-weight", type=float, default=1.0)
    parser.add_argument("--lateral-step-weight", type=float, default=0.5)
    parser.add_argument("--lateral-acceleration-weight", type=float, default=0.25)
    parser.add_argument("--unobserved-prior-weight", type=float, default=0.01)
    parser.add_argument("--state-weight", type=float, default=0.2)
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
        name: value.to(device, non_blocking=True)
        if isinstance(value, torch.Tensor)
        else value
        for name, value in batch.items()
    }


def forward_batch(
    model: SpatialResidualPlannerV8,
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


def batch_identity(batch: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": [str(value) for value in batch["run_id"]],
        "sample_id": batch["sample_id"].detach().cpu().tolist(),
        "gps_blackout": batch["gps_blackout"].detach().cpu().tolist(),
        "motion_state": batch["motion_state"].detach().cpu().tolist(),
        "avoidance": batch["avoidance"].detach().cpu().tolist(),
    }


def loss_kwargs(
    args: argparse.Namespace,
    training: bool,
) -> dict[str, float]:
    return {
        "lateral_weight": args.lateral_weight,
        "lateral_step_weight": args.lateral_step_weight,
        "lateral_acceleration_weight": args.lateral_acceleration_weight,
        "unobserved_prior_weight": args.unobserved_prior_weight,
        "state_weight": args.state_weight,
        "stop_class_weight": args.stop_class_weight if training else 1.0,
        "drive_class_weight": args.drive_class_weight if training else 1.0,
    }


def compute_loss(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, Any],
    normalizer: float,
    weights: dict[str, float],
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    return planner_loss(
        outputs,
        batch["target_spatial_lateral_m"],
        batch["target_spatial_valid"],
        batch["motion_state"],
        batch["sample_weight"],
        normalizer,
        **weights,
    )


@torch.no_grad()
def evaluate(
    model: SpatialResidualPlannerV8,
    loader: DataLoader,
    device: torch.device,
    amp_enabled: bool,
    normalizer: float,
    weights: dict[str, float],
) -> dict[str, Any]:
    model.eval()
    totals: dict[str, float] = {}
    count = 0
    residual_metrics = {
        "all": SpatialResidualMetricAccumulator(),
        "blackout": SpatialResidualMetricAccumulator(),
        "non_blackout": SpatialResidualMetricAccumulator(),
    }
    state_metrics = MotionStateMetricAccumulator()
    for batch in loader:
        batch = move_batch(batch, device)
        with torch.autocast(
            device_type=device.type,
            dtype=torch.float16,
            enabled=amp_enabled,
        ):
            outputs = forward_batch(model, batch)
            _, terms = compute_loss(outputs, batch, normalizer, weights)
        tensors = {**outputs, **terms}
        bad = [
            name
            for name, value in tensors.items()
            if isinstance(value, torch.Tensor)
            and not bool(torch.isfinite(value).all())
        ]
        if bad:
            raise FloatingPointError(
                "non-finite validation tensors: "
                + json.dumps({"bad": bad, **batch_identity(batch)}, sort_keys=True)
            )
        prediction = outputs["lateral_residual_m"]
        target = batch["target_spatial_lateral_m"]
        valid = batch["target_spatial_valid"]
        avoidance = batch["avoidance"]
        residual_metrics["all"].update(prediction, target, valid, avoidance)
        blackout = batch["gps_blackout"].bool()
        for name, mask in (
            ("blackout", blackout),
            ("non_blackout", ~blackout),
        ):
            if bool(mask.any()):
                residual_metrics[name].update(
                    prediction[mask],
                    target[mask],
                    valid[mask],
                    avoidance[mask],
                )
        state_metrics.update(outputs["motion_state_logits"], batch["motion_state"])
        batch_count = prediction.shape[0]
        count += batch_count
        for name, value in terms.items():
            totals[name] = totals.get(name, 0.0) + float(value) * batch_count
    result = {name: value / max(count, 1) for name, value in totals.items()}
    result["spatial_residual_metrics"] = {
        name: accumulator.result()
        for name, accumulator in residual_metrics.items()
    }
    result["motion_state_metrics"] = state_metrics.result()
    return result


def make_optimizer(
    model: SpatialResidualPlannerV8,
    args: argparse.Namespace,
) -> torch.optim.Optimizer:
    backbone = list(model.camera_encoder.backbone.parameters())
    backbone_ids = {id(parameter) for parameter in backbone}
    other = [
        parameter for parameter in model.parameters()
        if id(parameter) not in backbone_ids
    ]
    return torch.optim.AdamW(
        [
            {"params": other, "lr": args.lr},
            {"params": backbone, "lr": args.backbone_lr},
        ],
        weight_decay=args.weight_decay,
    )


def set_training_phase(
    model: SpatialResidualPlannerV8,
    epoch: int,
    head_warmup_epochs: int,
    backbone_warmup_epochs: int,
) -> str:
    if epoch < head_warmup_epochs:
        for parameter in model.parameters():
            parameter.requires_grad = False
        for module in (model.lateral_head, model.state_head):
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
        max_samples=args.max_train_samples if training else args.max_val_samples,
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

    train_dataset = build_dataset(args, splits["train"], True)
    val_dataset = build_dataset(args, splits["val"], False)
    print("train_dataset", json.dumps(train_dataset.summary(), sort_keys=True))
    print("val_dataset", json.dumps(val_dataset.summary(), sort_keys=True))
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
    initialization = None
    if args.resume:
        initialization = torch.load(args.resume, map_location=device, weights_only=False)
    elif args.init_checkpoint:
        initialization = torch.load(
            args.init_checkpoint, map_location=device, weights_only=False
        )
    if initialization is not None:
        values = dict(initialization["model_config"])
        values["pretrained_camera"] = False
        values["freeze_camera_backbone"] = True
        config = ModelConfig(**values)
    else:
        config = ModelConfig(
            pretrained_camera=not args.no_pretrained,
            freeze_camera_backbone=True,
        )
    model = SpatialResidualPlannerV8(config).to(device)
    initialization_report = None
    if args.init_checkpoint:
        initialization_report = {
            **model.load_shared_state_dict(initialization["model_state"]),
            "policy": (
                "V7 shared encoders and STOP/DRIVE head transferred; "
                "fresh fixed-distance lateral head; learned speed removed"
            ),
            "checkpoint": str(args.init_checkpoint.resolve()),
            "source_epoch": int(initialization.get("epoch", -1)) + 1,
        }
        print("initialization", json.dumps(initialization_report, sort_keys=True))

    optimizer = make_optimizer(model, args)
    amp_enabled = device.type == "cuda" and not args.no_amp
    scaler = torch.amp.GradScaler("cuda", enabled=amp_enabled)
    print("device", device)
    print("parameters", json.dumps(model.parameter_counts(), sort_keys=True))
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

    train_weights = loss_kwargs(args, True)
    val_weights = loss_kwargs(args, False)
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
        start = time.time()
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
            tensors = {**outputs, **terms, "total": loss}
            bad = [
                name
                for name, value in tensors.items()
                if isinstance(value, torch.Tensor)
                and not bool(torch.isfinite(value).all())
            ]
            if bad:
                raise FloatingPointError(
                    "non-finite training tensors: "
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
                torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)
            batch_count = outputs["lateral_residual_m"].shape[0]
            sample_count += batch_count
            for name, value in terms.items():
                running[name] = running.get(name, 0.0) + float(value) * batch_count
            if (step + 1) % args.log_every == 0:
                metrics = {
                    name: value / sample_count
                    for name, value in running.items()
                }
                print(
                    f"epoch={epoch + 1}/{args.epochs} phase={phase} "
                    f"step={step + 1}/{len(train_loader)} "
                    f"train={json.dumps(metrics, sort_keys=True)}",
                    flush=True,
                )

        train_metrics = {
            name: value / max(sample_count, 1)
            for name, value in running.items()
        }
        val_metrics = evaluate(
            model,
            val_loader,
            device,
            amp_enabled,
            val_dataset.mean_sample_weight,
            val_weights,
        )
        record = {
            "epoch": epoch,
            "training_phase": phase,
            "seconds": time.time() - start,
            "train": train_metrics,
            "val": val_metrics,
        }
        history.append(record)
        print("epoch_result", json.dumps(record, sort_keys=True), flush=True)
        metrics_dir = args.output_dir / "metrics"
        metrics_dir.mkdir(parents=True, exist_ok=True)
        (metrics_dir / f"epoch_{epoch + 1:03d}.json").write_text(
            json.dumps(val_metrics, indent=2), encoding="utf-8"
        )
        state = {
            "architecture": "multimodal_planner_v8_spatial_residual_no_speed",
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
                "spatial_lateral_residual_m": [20],
                "spatial_anchors_m": SPATIAL_ANCHORS_M.tolist(),
                "motion_state": ["STOP", "DRIVE"],
                "learned_speed": False,
                "runtime_mpc_path": {
                    "length_m": 80.0,
                    "interval_m": 0.1,
                    "points": 801,
                    "minimum_route_ahead_m": 80.0,
                    "recommended_route_ahead_m": 100.0,
                },
            },
            "label_policy": (
                "raw target unchanged; only reached stations are interpolated "
                "and supervised; unreached stations are masked"
            ),
            "loss_weighting": {
                **train_weights,
                "blackout_sample_weight": args.blackout_weight,
                "stop_sample_weight": args.stop_sample_weight,
                "drive_sample_weight": args.drive_sample_weight,
                "avoidance_sample_weight": args.avoidance_weight,
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
            json.dumps(history, indent=2), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
