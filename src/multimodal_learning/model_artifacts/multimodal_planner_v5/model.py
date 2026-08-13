from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn
from torchvision.models import ResNet18_Weights, resnet18


@dataclass(frozen=True)
class ModelConfig:
    hidden_dim: int = 192
    history_frames: int = 5
    horizon: int = 20
    attention_heads: int = 4
    spatial_layers: int = 2
    route_layers: int = 2
    dropout: float = 0.1
    pretrained_camera: bool = True
    freeze_camera_backbone: bool = True


def prepend_local_origin(trajectory: torch.Tensor) -> torch.Tensor:
    """Prepend an ego-frame origin without changing supervised future points."""
    if trajectory.ndim != 3 or trajectory.shape[-1] != 4:
        raise ValueError(
            "trajectory must have shape [batch, horizon, 4], "
            f"got {tuple(trajectory.shape)}"
        )
    origin_xy_yaw = torch.zeros(
        trajectory.shape[0],
        1,
        3,
        dtype=trajectory.dtype,
        device=trajectory.device,
    )
    origin_speed = trajectory[:, :1, 3:4]
    origin = torch.cat((origin_xy_yaw, origin_speed), dim=-1)
    return torch.cat((origin, trajectory), dim=1)


class CrossAttentionBlock(nn.Module):
    def __init__(self, dim: int, heads: int, dropout: float) -> None:
        super().__init__()
        self.token_norm = nn.LayerNorm(dim)
        self.kv_norm = nn.LayerNorm(dim)
        self.attention = nn.MultiheadAttention(
            dim, heads, dropout=dropout, batch_first=True
        )
        self.ffn_norm = nn.LayerNorm(dim)
        self.ffn = nn.Sequential(
            nn.Linear(dim, dim * 4),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(dim * 4, dim),
            nn.Dropout(dropout),
        )

    def forward(self, tokens: torch.Tensor, key_value: torch.Tensor) -> torch.Tensor:
        attended, _ = self.attention(
            self.token_norm(tokens),
            self.kv_norm(key_value),
            self.kv_norm(key_value),
            need_weights=False,
        )
        tokens = tokens + attended
        return tokens + self.ffn(self.ffn_norm(tokens))


class SharedCameraEncoder(nn.Module):
    """One ImageNet ResNet18 shared by all three camera views."""

    def __init__(self, dim: int, pretrained: bool, frozen: bool) -> None:
        super().__init__()
        weights = ResNet18_Weights.DEFAULT if pretrained else None
        network = resnet18(weights=weights)
        self.backbone = nn.Sequential(*list(network.children())[:-2])
        self.projection = nn.Conv2d(512, dim, kernel_size=1, bias=False)
        self.pool = nn.AdaptiveAvgPool2d((4, 4))
        self.camera_id = nn.Parameter(torch.zeros(3, 1, dim))
        nn.init.normal_(self.camera_id, std=0.02)
        self.output_norm = nn.LayerNorm(dim)
        self.frozen = frozen
        self.set_backbone_trainable(not frozen)

    def set_backbone_trainable(self, trainable: bool) -> None:
        self.frozen = not trainable
        for parameter in self.backbone.parameters():
            parameter.requires_grad = trainable
        if self.frozen:
            self.backbone.eval()

    def train(self, mode: bool = True) -> "SharedCameraEncoder":
        super().train(mode)
        if self.frozen:
            self.backbone.eval()
        return self

    def encode_view(self, images: torch.Tensor, camera_index: int) -> torch.Tensor:
        if self.frozen:
            with torch.no_grad():
                feature = self.backbone(images)
        else:
            feature = self.backbone(images)
        feature = self.pool(self.projection(feature))
        tokens = feature.flatten(2).transpose(1, 2)
        return self.output_norm(tokens + self.camera_id[camera_index])

    def forward(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
    ) -> torch.Tensor:
        front_tokens = self.encode_view(front, 0)
        left_tokens = self.encode_view(left, 1)
        right_tokens = self.encode_view(right, 2)
        # 16 tokens per camera: 48 camera tokens per frame.
        return torch.cat((front_tokens, left_tokens, right_tokens), dim=1)


class ConvNormAct(nn.Sequential):
    def __init__(self, in_channels: int, out_channels: int, stride: int = 1) -> None:
        super().__init__(
            nn.Conv2d(
                in_channels,
                out_channels,
                kernel_size=3,
                stride=stride,
                padding=1,
                bias=False,
            ),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
        )


class LightBEVEncoder(nn.Module):
    """Four-stage VLP16 BEV encoder with multi-scale 4x4 token fusion."""

    def __init__(self, dim: int) -> None:
        super().__init__()
        self.stage1 = nn.Sequential(
            ConvNormAct(3, 32, 2),
            ConvNormAct(32, 32),
        )
        self.stage2 = nn.Sequential(
            ConvNormAct(32, 64, 2),
            ConvNormAct(64, 64),
        )
        self.stage3 = nn.Sequential(
            ConvNormAct(64, 128, 2),
            ConvNormAct(128, 128),
            ConvNormAct(128, 128),
        )
        self.stage4 = nn.Sequential(
            ConvNormAct(128, 192, 2),
            ConvNormAct(192, 192),
            ConvNormAct(192, 192),
            ConvNormAct(192, 192),
        )
        branch_dim = dim // 3
        self.scale2 = nn.Conv2d(64, branch_dim, 1)
        self.scale3 = nn.Conv2d(128, branch_dim, 1)
        self.scale4 = nn.Conv2d(192, dim - 2 * branch_dim, 1)
        self.pool = nn.AdaptiveAvgPool2d((4, 4))
        self.norm = nn.LayerNorm(dim)

    def forward(self, bev: torch.Tensor) -> torch.Tensor:
        x1 = self.stage1(bev)
        x2 = self.stage2(x1)
        x3 = self.stage3(x2)
        x4 = self.stage4(x3)
        fused = torch.cat(
            (
                self.pool(self.scale2(x2)),
                self.pool(self.scale3(x3)),
                self.pool(self.scale4(x4)),
            ),
            dim=1,
        )
        # 4x4 grid: 16 LiDAR tokens per frame.
        return self.norm(fused.flatten(2).transpose(1, 2))


class EgoLocalizationEncoder(nn.Module):
    def __init__(self, input_dim: int, dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.LayerNorm(64),
            nn.SiLU(),
            nn.Linear(64, dim),
            nn.LayerNorm(dim),
        )

    def forward(self, ego: torch.Tensor) -> torch.Tensor:
        # One Ego/Localization token per frame.
        return self.net(ego).unsqueeze(1)


class InputConditionedSlotGenerator(nn.Module):
    """Generate eight queries from the current KV tokens, without learned queries."""

    def __init__(self, dim: int, slots: int) -> None:
        super().__init__()
        self.norm = nn.LayerNorm(dim)
        self.slot_scores = nn.Linear(dim, slots, bias=False)

    def forward(self, key_value: torch.Tensor) -> torch.Tensor:
        # [B, 65, 8] -> [B, 8, 65], normalized across source tokens.
        weights = self.slot_scores(self.norm(key_value)).transpose(1, 2)
        weights = torch.softmax(weights, dim=-1)
        return torch.matmul(weights, key_value)


class MGeoEncoder(nn.Module):
    def __init__(self, dim: int, heads: int, dropout: float) -> None:
        super().__init__()
        self.point_mlp = nn.Sequential(
            nn.Linear(8, 64),
            nn.SiLU(),
            nn.Linear(64, dim),
            nn.LayerNorm(dim),
        )
        layer = nn.TransformerEncoderLayer(
            d_model=dim,
            nhead=heads,
            dim_feedforward=dim * 4,
            dropout=dropout,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.transformer = nn.TransformerEncoder(layer, num_layers=2)
        self.output_norm = nn.LayerNorm(dim)

    def forward(self, mgeo: torch.Tensor) -> torch.Tensor:
        return self.output_norm(self.transformer(self.point_mlp(mgeo)))


class LocalRouteEncoder(nn.Module):
    def __init__(self, dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 64),
            nn.SiLU(),
            nn.Linear(64, dim),
            nn.LayerNorm(dim),
        )

    def forward(self, route: torch.Tensor) -> torch.Tensor:
        return self.net(route)


class MultiViewTemporalTrajectoryPlannerV5(nn.Module):
    """Single-trajectory V5 planner with no mode selection head."""

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__()
        self.config = config or ModelConfig()
        cfg = self.config
        dim = cfg.hidden_dim

        self.camera_encoder = SharedCameraEncoder(
            dim, cfg.pretrained_camera, cfg.freeze_camera_backbone
        )
        self.lidar_encoder = LightBEVEncoder(dim)
        self.ego_encoder = EgoLocalizationEncoder(input_dim=16, dim=dim)

        self.slot_generator = InputConditionedSlotGenerator(dim, slots=8)
        self.spatial_fusion = nn.ModuleList(
            CrossAttentionBlock(dim, cfg.attention_heads, cfg.dropout)
            for _ in range(cfg.spatial_layers)
        )
        self.temporal_gru = nn.GRU(
            input_size=dim,
            hidden_size=dim,
            num_layers=1,
            batch_first=True,
        )

        self.mgeo_encoder = MGeoEncoder(dim, cfg.attention_heads, cfg.dropout)
        self.route_encoder = LocalRouteEncoder(dim)
        self.map_type_embedding = nn.Parameter(torch.zeros(2, 1, dim))
        nn.init.normal_(self.map_type_embedding, std=0.02)
        self.route_fusion = nn.ModuleList(
            CrossAttentionBlock(dim, cfg.attention_heads, cfg.dropout)
            for _ in range(cfg.route_layers)
        )
        self.output_norm = nn.LayerNorm(dim)

        self.trajectory_head = nn.Sequential(
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Dropout(cfg.dropout),
            nn.Linear(dim * 2, cfg.horizon * 4),
        )

    def set_camera_backbone_trainable(self, trainable: bool) -> None:
        self.camera_encoder.set_backbone_trainable(trainable)

    def encode_frames(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        ego: torch.Tensor,
    ) -> torch.Tensor:
        batch, history = front.shape[:2]
        flat = batch * history
        camera_tokens = self.camera_encoder(
            front.reshape(flat, *front.shape[2:]),
            left.reshape(flat, *left.shape[2:]),
            right.reshape(flat, *right.shape[2:]),
        )
        lidar_tokens = self.lidar_encoder(
            lidar_bev.reshape(flat, *lidar_bev.shape[2:])
        )
        ego_tokens = self.ego_encoder(ego.reshape(flat, -1))
        key_value = torch.cat((camera_tokens, lidar_tokens, ego_tokens), dim=1)
        if key_value.shape[1] != 65:
            raise RuntimeError(f"spatial fusion must receive KV65, got {key_value.shape}")

        slots = self.slot_generator(key_value)
        for layer in self.spatial_fusion:
            slots = layer(slots, key_value)
        return slots.reshape(batch, history, 8, self.config.hidden_dim)

    def forward(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        ego: torch.Tensor,
        mgeo: torch.Tensor,
        local_route: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        frame_slots = self.encode_frames(front, left, right, lidar_bev, ego)
        batch, history, slots, dim = frame_slots.shape
        temporal_input = frame_slots.permute(0, 2, 1, 3).reshape(
            batch * slots, history, dim
        )
        _, hidden = self.temporal_gru(temporal_input)
        temporal_slots = hidden[-1].reshape(batch, slots, dim)

        mgeo_tokens = self.mgeo_encoder(mgeo) + self.map_type_embedding[0]
        route_tokens = self.route_encoder(local_route) + self.map_type_embedding[1]
        map_route_tokens = torch.cat((mgeo_tokens, route_tokens), dim=1)

        fused = temporal_slots
        for layer in self.route_fusion:
            fused = layer(fused, map_route_tokens)
        fused = self.output_norm(fused)
        context = fused.mean(dim=1)

        trajectory = self.trajectory_head(context).reshape(
            batch, self.config.horizon, 4
        )
        # The 20 supervised points remain future samples over four seconds.
        # Controller/local-route consumers receive a separate current-ego origin.
        trajectory_with_origin = prepend_local_origin(trajectory)
        return {
            "trajectory": trajectory,
            "trajectory_with_origin": trajectory_with_origin,
            "spatial_tokens": fused,
        }

    def parameter_counts(self) -> dict[str, int]:
        components = {
            "camera": self.camera_encoder,
            "lidar": self.lidar_encoder,
            "ego": self.ego_encoder,
            "spatial_fusion": nn.ModuleList(
                [self.slot_generator, *self.spatial_fusion]
            ),
            "temporal_gru": self.temporal_gru,
            "mgeo": self.mgeo_encoder,
            "route": self.route_encoder,
            "route_fusion": self.route_fusion,
            "head": self.trajectory_head,
        }
        counts = {
            name: sum(parameter.numel() for parameter in module.parameters())
            for name, module in components.items()
        }
        counts["total"] = sum(parameter.numel() for parameter in self.parameters())
        counts["trainable"] = sum(
            parameter.numel() for parameter in self.parameters() if parameter.requires_grad
        )
        return counts
