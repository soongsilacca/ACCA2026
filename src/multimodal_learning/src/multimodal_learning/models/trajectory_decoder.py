import torch
import torch.nn as nn
import torch.nn.functional as F


class MultiModeTrajectoryDecoder(nn.Module):
    def __init__(self, hidden_dim=192, horizon=20, modes=3, signal_classes=4, ttc_bins=5):
        super().__init__()
        self.horizon = horizon
        self.modes = modes
        self.shared = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim), nn.GELU(), nn.LayerNorm(hidden_dim)
        )
        self.trajectory = nn.Linear(hidden_dim, modes * horizon * 4)
        self.mode_score = nn.Linear(hidden_dim, modes)
        self.signal = nn.Linear(hidden_dim, signal_classes)
        self.stop = nn.Linear(hidden_dim, 2)
        self.hazard = nn.Linear(hidden_dim, 1 + ttc_bins)
        self.lane = nn.Linear(hidden_dim, 1)

    def forward(self, context):
        feature = self.shared(context)
        trajectory = self.trajectory(feature).view(
            context.shape[0], self.modes, self.horizon, 4
        )
        trajectory = torch.cat(
            (trajectory[..., :3], F.softplus(trajectory[..., 3:])), dim=-1
        )
        stop = self.stop(feature)
        hazard = self.hazard(feature)
        mode_logits = self.mode_score(feature)
        selected_index = mode_logits.argmax(dim=-1)
        batch_index = torch.arange(context.shape[0], device=context.device)
        return {
            "trajectories": trajectory,
            "mode_logits": mode_logits,
            "selected_index": selected_index,
            "selected_trajectory": trajectory[batch_index, selected_index],
            "signal_logits": self.signal(feature),
            "stop_logit": stop[:, 0],
            "stop_distance": F.softplus(stop[:, 1]),
            "collision_logit": hazard[:, 0],
            "ttc_logits": hazard[:, 1:],
            "lane_logit": self.lane(feature).squeeze(-1),
        }


TrajectoryDecoder = MultiModeTrajectoryDecoder
