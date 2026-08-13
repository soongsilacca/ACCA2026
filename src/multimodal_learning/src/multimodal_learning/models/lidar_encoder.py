import torch
import torch.nn as nn


class ConvStage(nn.Sequential):
    def __init__(self, input_channels, output_channels, stride=2):
        super().__init__(
            nn.Conv2d(input_channels, output_channels, 3, stride=stride, padding=1, bias=False),
            nn.BatchNorm2d(output_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(output_channels, output_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(output_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(output_channels, output_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(output_channels),
            nn.ReLU(inplace=True),
        )


class LightweightBEVEncoder(nn.Module):
    """Four-stage 3-channel BEV CNN with multi-scale token projection."""

    def __init__(self, hidden_dim=192, channels=(32, 64, 128, 192), grid_size=4):
        super().__init__()
        stages, previous = [], 3
        for channel in channels:
            stages.append(ConvStage(previous, channel))
            previous = channel
        self.stages = nn.ModuleList(stages)
        self.pool = nn.AdaptiveAvgPool2d((grid_size, grid_size))
        self.projections = nn.ModuleList(nn.Conv2d(c, hidden_dim, 1) for c in channels)
        self.scale_logits = nn.Parameter(torch.zeros(len(channels)))
        self.norm = nn.LayerNorm(hidden_dim)
        self.position = nn.Parameter(torch.empty(1, grid_size * grid_size, hidden_dim))
        nn.init.normal_(self.position, std=0.02)

    def forward(self, bev):
        features = []
        value = bev
        for stage, projection in zip(self.stages, self.projections):
            value = stage(value)
            features.append(self.pool(projection(value)))
        weights = nn.functional.softmax(self.scale_logits, dim=0)
        fused = sum(weight * feature for weight, feature in zip(weights, features))
        tokens = fused.flatten(2).transpose(1, 2)
        return self.norm(tokens + self.position)
LidarEncoder = LightweightBEVEncoder
