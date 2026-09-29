"""Runtime definition matching the frozen MORAI TransFuser checkpoint."""

from dataclasses import dataclass

import torch
from torch import nn
from torch.nn import functional as F
from torchvision.models import resnet18, resnet34


@dataclass
class TransFuserConfig:
    pred_len: int = 4
    transformer_layers: int = 4
    gru_hidden: int = 64


class FusionBlock(nn.Module):
    """Fuse 5x22 panorama tokens with 8x8 LiDAR tokens."""

    def __init__(self, channels, layers):
        super().__init__()
        self.position = nn.Parameter(torch.zeros(1, 174, channels))
        self.velocity = nn.Linear(1, channels)
        layer = nn.TransformerEncoderLayer(
            d_model=channels,
            nhead=4,
            dim_feedforward=channels * 4,
            dropout=0.1,
            activation="relu",
            batch_first=True,
            norm_first=False,
        )
        self.blocks = nn.TransformerEncoder(layer, num_layers=layers)
        self.norm = nn.LayerNorm(channels)

    def forward(self, image, lidar, speed):
        image_tokens = F.adaptive_avg_pool2d(image, (5, 22)).flatten(2).transpose(1, 2)
        lidar_tokens = F.adaptive_avg_pool2d(lidar, (8, 8)).flatten(2).transpose(1, 2)
        tokens = torch.cat((image_tokens, lidar_tokens), dim=1)
        tokens = tokens + self.position + self.velocity(speed).unsqueeze(1)
        tokens = self.norm(self.blocks(tokens))
        image_delta = tokens[:, :110].transpose(1, 2).reshape(-1, image.shape[1], 5, 22)
        lidar_delta = tokens[:, 110:].transpose(1, 2).reshape(-1, lidar.shape[1], 8, 8)
        image = image + F.interpolate(
            image_delta, size=image.shape[-2:], mode="bilinear", align_corners=False
        )
        lidar = lidar + F.interpolate(
            lidar_delta, size=lidar.shape[-2:], mode="bilinear", align_corners=False
        )
        return image, lidar


class TransFuserTrajectory(nn.Module):
    def __init__(self, config=None):
        super().__init__()
        self.config = config or TransFuserConfig()
        self.image_encoder = resnet34(weights=None)
        self.lidar_encoder = resnet18(weights=None)
        channels = (64, 128, 256, 512)
        self.fusions = nn.ModuleList(
            FusionBlock(channel, self.config.transformer_layers)
            for channel in channels
        )
        self.join = nn.Sequential(
            nn.Linear(512, 256), nn.ReLU(inplace=True),
            nn.Linear(256, 128), nn.ReLU(inplace=True),
            nn.Linear(128, self.config.gru_hidden), nn.ReLU(inplace=True),
        )
        self.decoder = nn.GRUCell(4, self.config.gru_hidden)
        self.output = nn.Linear(self.config.gru_hidden, 2)

    @staticmethod
    def _stem(encoder, value):
        value = encoder.conv1(value)
        value = encoder.bn1(value)
        value = encoder.relu(value)
        return encoder.maxpool(value)

    def forward(self, image, lidar_bev, speed, target_point):
        image = self._stem(self.image_encoder, image)
        lidar = self._stem(self.lidar_encoder, lidar_bev)
        for index, name in enumerate(("layer1", "layer2", "layer3", "layer4")):
            image = getattr(self.image_encoder, name)(image)
            lidar = getattr(self.lidar_encoder, name)(lidar)
            image, lidar = self.fusions[index](image, lidar, speed)
        image_context = F.adaptive_avg_pool2d(image, 1).flatten(1)
        lidar_context = F.adaptive_avg_pool2d(lidar, 1).flatten(1)
        hidden = self.join(image_context + lidar_context)
        point = torch.zeros_like(target_point)
        waypoints = []
        for _ in range(self.config.pred_len):
            hidden = self.decoder(torch.cat((point, target_point), dim=1), hidden)
            point = point + self.output(hidden)
            waypoints.append(point)
        return {"waypoints": torch.stack(waypoints, dim=1)}
