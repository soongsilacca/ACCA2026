#!/usr/bin/env python3
import argparse

import torch
from torch.utils.data import DataLoader

from multimodal_learning.dataset import MultimodalDataset
from multimodal_learning.metrics import trajectory_metrics
from multimodal_learning.models import MultimodalPlanner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset")
    parser.add_argument("checkpoint")
    parser.add_argument("--batch-size", type=int, default=2)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(args.checkpoint, map_location=device)
    model_config = dict(checkpoint["config"])
    model_config["camera_pretrained"] = False
    model = MultimodalPlanner(model_config).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    keys = ("ade", "fde", "yaw_mae", "speed_mae", "horizon_1s", "horizon_2s", "horizon_4s")
    sums = {key: 0.0 for key in keys}
    count = 0
    with torch.no_grad():
        for batch in DataLoader(MultimodalDataset(args.dataset), args.batch_size, False):
            batch = {key: value.to(device) for key, value in batch.items()}
            metrics = trajectory_metrics(model(batch), batch["trajectory"])
            size = batch["trajectory"].shape[0]
            count += size
            for key in sums:
                sums[key] += metrics[key] * size
    print(" ".join("{}={:.4f}".format(key, sums[key] / count) for key in keys))


if __name__ == "__main__":
    main()
