import torch
import torch.nn as nn

from .camera_encoder import SharedCameraEncoder
from .fusion_transformer import CrossAttentionBlock, DynamicQueryFusion
from .lidar_encoder import LightweightBEVEncoder
from .map_encoder import MapEncoder
from .state_encoder import MLPEncoder, RouteEncoder
from .trajectory_decoder import MultiModeTrajectoryDecoder


class MultimodalPlanner(nn.Module):
    """Multi-View Temporal Trajectory Planner V2."""

    def __init__(self, config):
        super().__init__()
        hidden = config["hidden_dim"]
        grid = config.get("spatial_grid_size", 4)
        heads = config["attention_heads"]
        dropout = config.get("dropout", 0.1)
        self.history_frames = config["history_frames"]
        self.dynamic_queries = config.get("dynamic_queries", 8)

        self.camera = SharedCameraEncoder(
            hidden, config.get("camera_pretrained", True), grid,
            config.get("camera_views", 3),
        )
        self.lidar = LightweightBEVEncoder(
            hidden, tuple(config.get("lidar_channels", (32, 64, 128, 192))), grid
        )
        self.ego = MLPEncoder(config["ego_feature_dim"], hidden, 3)
        self.spatial_fusion = DynamicQueryFusion(
            hidden, heads, config.get("spatial_fusion_layers", 2),
            self.dynamic_queries, dropout,
        )
        self.temporal = nn.GRU(
            hidden, hidden, config.get("gru_layers", 1), batch_first=True
        )

        self.map = MapEncoder(
            config["map_feature_dim"], hidden, heads,
            config["map_transformer_layers"], dropout,
        )
        self.route = RouteEncoder(config["route_feature_dim"], hidden)
        self.static_type_embedding = nn.Parameter(torch.empty(2, hidden))
        nn.init.normal_(self.static_type_embedding, std=0.02)
        self.route_fusion = nn.ModuleList(
            CrossAttentionBlock(hidden, heads, dropout)
            for _ in range(config.get("route_fusion_layers", 2))
        )
        self.planning_norm = nn.LayerNorm(hidden)
        self.decoder = MultiModeTrajectoryDecoder(
            hidden,
            config["prediction_horizon"],
            config.get("prediction_modes", 3),
            config.get("signal_classes", 4),
            config.get("ttc_bins", 5),
        )

    @staticmethod
    def _latest(value):
        return value[:, -1] if value.ndim >= 4 else value

    def encode_static(self, map_tokens, map_mask, route_tokens, route_mask):
        map_encoded, map_valid = self.map(self._latest(map_tokens), self._latest(map_mask))
        route_encoded, route_valid = self.route(
            self._latest(route_tokens), self._latest(route_mask)
        )
        return (
            torch.cat((
                map_encoded + self.static_type_embedding[0],
                route_encoded + self.static_type_embedding[1],
            ), dim=1),
            torch.cat((map_valid, route_valid), dim=1),
        )

    def encode_dynamic_frame(self, camera_front, camera_left, camera_right, lidar, ego):
        camera_tokens = torch.cat((
            self.camera(camera_front, 0),
            self.camera(camera_left, 1),
            self.camera(camera_right, 2),
        ), dim=1)
        return self.spatial_fusion(
            camera_tokens, self.lidar(lidar), self.ego(ego)
        )

    def decode_dynamic(self, dynamic_features, static_cache):
        # [B,T,Q,H] -> shared GRU over time for each of Q dynamic slots.
        batch, history, queries, hidden = dynamic_features.shape
        slot_sequence = dynamic_features.permute(0, 2, 1, 3).reshape(
            batch * queries, history, hidden
        )
        temporal, _ = self.temporal(slot_sequence)
        slots = temporal[:, -1].reshape(batch, queries, hidden)
        static_tokens, static_mask = static_cache
        for block in self.route_fusion:
            slots = block(slots, static_tokens, static_mask)
        planning_feature = self.planning_norm(slots).mean(dim=1)
        return self.decoder(planning_feature)

    def forward(self, batch, static_cache=None):
        batch_size, history = batch["camera_front"].shape[:2]
        if history != self.history_frames:
            raise ValueError(
                "Expected {} history frames, got {}".format(self.history_frames, history)
            )

        def flatten(value):
            return value.reshape(batch_size * history, *value.shape[2:])

        frame_slots = self.encode_dynamic_frame(
            flatten(batch["camera_front"]),
            flatten(batch["camera_left"]),
            flatten(batch["camera_right"]),
            flatten(batch["lidar"]),
            flatten(batch["ego"]),
        ).view(batch_size, history, self.dynamic_queries, -1)
        if static_cache is None:
            static_cache = self.encode_static(
                batch["map_tokens"], batch["map_mask"],
                batch["route_tokens"], batch["route_mask"],
            )
        return self.decode_dynamic(frame_slots, static_cache)
