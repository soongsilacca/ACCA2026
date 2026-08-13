#!/usr/bin/env python3
import argparse

import torch
from torch.utils.data import DataLoader, random_split

from multimodal_learning.dataset import MultimodalDataset
from multimodal_learning.io_utils import load_yaml, package_config
from multimodal_learning.losses import planner_loss
from multimodal_learning.models import MultimodalPlanner


def set_camera_backbone_trainable(model, trainable):
    for parameter in model.camera.backbone.parameters():
        parameter.requires_grad = trainable


def main():
    parser = argparse.ArgumentParser(description="Train Planner V2 on RTX 2080")
    parser.add_argument("dataset")
    parser.add_argument("--config", default=package_config("model.yaml"))
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--gradient-accumulation", type=int, default=8)
    parser.add_argument("--frozen-backbone-epochs", type=int, default=3)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--log-interval", type=int, default=50)
    parser.add_argument("--output", default="planner_v2.pt")
    args = parser.parse_args()

    config = load_yaml(args.config)
    dataset = MultimodalDataset(args.dataset)
    if len(dataset) < 10:
        raise RuntimeError("V2 training requires at least 10 samples; this dataset has {}".format(len(dataset)))
    validation_size = max(1, int(len(dataset) * 0.1))
    training, validation = random_split(
        dataset, [len(dataset) - validation_size, validation_size],
        generator=torch.Generator().manual_seed(42),
    )
    loaders = {
        "train": DataLoader(training, args.batch_size, True, num_workers=2, pin_memory=True),
        "val": DataLoader(validation, args.batch_size, False, num_workers=1, pin_memory=True),
    }
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = MultimodalPlanner(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scaler = torch.cuda.amp.GradScaler(enabled=device.type == "cuda")
    best = float("inf")

    for epoch in range(args.epochs):
        set_camera_backbone_trainable(model, epoch >= args.frozen_backbone_epochs)
        for phase in ("train", "val"):
            training_phase = phase == "train"
            model.train(training_phase)
            optimizer.zero_grad(set_to_none=True)
            total, count = 0.0, 0
            for step, batch in enumerate(loaders[phase]):
                batch = {key: value.to(device, non_blocking=True) for key, value in batch.items()}
                with torch.set_grad_enabled(training_phase), torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                    terms = planner_loss(model(batch), batch)
                    loss = terms["total"]
                if training_phase:
                    scaler.scale(loss / args.gradient_accumulation).backward()
                    if (step + 1) % args.gradient_accumulation == 0 or step + 1 == len(loaders[phase]):
                        scaler.unscale_(optimizer)
                        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                        scaler.step(optimizer)
                        scaler.update()
                        optimizer.zero_grad(set_to_none=True)
                size = batch["trajectory"].shape[0]
                total += loss.item() * size
                count += size
                if (step + 1) % max(args.log_interval, 1) == 0 or step + 1 == len(loaders[phase]):
                    print(
                        "epoch={} phase={} step={}/{} running_loss={:.6f}".format(
                            epoch + 1, phase, step + 1, len(loaders[phase]),
                            total / max(count, 1),
                        ),
                        flush=True,
                    )
            score = total / max(count, 1)
            print("epoch={} {}_loss={:.6f}".format(epoch + 1, phase, score))
            if phase == "val" and score < best:
                best = score
                torch.save({
                    "model": model.state_dict(), "config": config,
                    "epoch": epoch + 1, "validation_loss": best,
                    "model_version": "multi_view_temporal_trajectory_planner_v2",
                }, args.output)


if __name__ == "__main__":
    main()
