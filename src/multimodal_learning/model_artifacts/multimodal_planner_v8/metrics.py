from __future__ import annotations

from typing import Any

import torch

from multimodal_planner_v8.data import MOTION_STATE_NAMES


class SpatialResidualMetricAccumulator:
    def __init__(self, anchors: int = 20) -> None:
        self.abs_sum = 0.0
        self.sq_sum = 0.0
        self.valid_count = 0
        self.sample_count = 0
        self.anchor_abs_sum = torch.zeros(anchors, dtype=torch.float64)
        self.anchor_count = torch.zeros(anchors, dtype=torch.int64)
        self.avoidance_abs_sum = 0.0
        self.avoidance_count = 0

    def update(
        self,
        prediction: torch.Tensor,
        target: torch.Tensor,
        valid: torch.Tensor,
        avoidance: torch.Tensor,
    ) -> None:
        error = (prediction.detach().float() - target.detach().float()).cpu()
        mask = valid.detach().bool().cpu()
        absolute = error.abs()
        self.abs_sum += float(absolute[mask].sum())
        self.sq_sum += float(error[mask].square().sum())
        self.valid_count += int(mask.sum())
        self.sample_count += prediction.shape[0]
        self.anchor_abs_sum += (absolute * mask).sum(dim=0).double()
        self.anchor_count += mask.sum(dim=0)
        avoid = avoidance.detach().bool().cpu().unsqueeze(1) & mask
        self.avoidance_abs_sum += float(absolute[avoid].sum())
        self.avoidance_count += int(avoid.sum())

    def result(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "valid_anchor_count": self.valid_count,
            "lateral_mae_m": self.abs_sum / max(self.valid_count, 1),
            "lateral_rmse_m": (self.sq_sum / max(self.valid_count, 1)) ** 0.5,
            "avoidance_lateral_mae_m": (
                self.avoidance_abs_sum / max(self.avoidance_count, 1)
            ),
            "per_anchor_mae_m": [
                float(self.anchor_abs_sum[i]) / max(int(self.anchor_count[i]), 1)
                for i in range(len(self.anchor_count))
            ],
            "per_anchor_count": self.anchor_count.tolist(),
        }


class MotionStateMetricAccumulator:
    def __init__(self) -> None:
        self.confusion = torch.zeros(2, 2, dtype=torch.int64)

    def update(self, logits: torch.Tensor, target: torch.Tensor) -> None:
        prediction = logits.detach().argmax(dim=-1).cpu()
        truth = target.detach().cpu()
        for actual, predicted in zip(truth.tolist(), prediction.tolist()):
            self.confusion[int(actual), int(predicted)] += 1

    def result(self) -> dict[str, Any]:
        total = int(self.confusion.sum())
        states = {}
        f1s = []
        for index, name in enumerate(MOTION_STATE_NAMES):
            tp = int(self.confusion[index, index])
            fn = int(self.confusion[index].sum()) - tp
            fp = int(self.confusion[:, index].sum()) - tp
            precision = tp / max(tp + fp, 1)
            recall = tp / max(tp + fn, 1)
            f1 = 2 * precision * recall / max(precision + recall, 1.0e-12)
            f1s.append(f1)
            states[name] = {
                "count": int(self.confusion[index].sum()),
                "precision": precision,
                "recall": recall,
                "f1": f1,
            }
        return {
            "accuracy": int(torch.diag(self.confusion).sum()) / max(total, 1),
            "macro_f1": sum(f1s) / len(f1s),
            "confusion_matrix_actual_rows_predicted_columns": self.confusion.tolist(),
            "states": states,
        }


__all__ = ["MotionStateMetricAccumulator", "SpatialResidualMetricAccumulator"]

