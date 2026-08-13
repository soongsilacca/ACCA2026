import torch
import torch.nn as nn


class CrossAttentionBlock(nn.Module):
    def __init__(self, hidden_dim=192, heads=4, dropout=0.1):
        super().__init__()
        self.query_norm = nn.LayerNorm(hidden_dim)
        self.memory_norm = nn.LayerNorm(hidden_dim)
        self.attention = nn.MultiheadAttention(
            hidden_dim, heads, dropout=dropout, batch_first=True
        )
        self.norm = nn.LayerNorm(hidden_dim)
        self.ffn = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim * 4),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim * 4, hidden_dim),
        )
        self.dropout = nn.Dropout(dropout)

    def forward(self, query, memory, memory_mask=None):
        padding_mask = None if memory_mask is None else ~memory_mask.bool()
        attended, _ = self.attention(
            self.query_norm(query),
            self.memory_norm(memory),
            self.memory_norm(memory),
            key_padding_mask=padding_mask,
            need_weights=False,
        )
        value = query + self.dropout(attended)
        return value + self.dropout(self.ffn(self.norm(value)))


class DynamicQueryFusion(nn.Module):
    """Extract eight object/scene slots instead of collapsing to one CLS token."""

    def __init__(self, hidden_dim=192, heads=4, layers=2, queries=8, dropout=0.1):
        super().__init__()
        self.queries = nn.Parameter(torch.empty(1, queries, hidden_dim))
        self.modality_embedding = nn.Parameter(torch.empty(3, hidden_dim))
        self.blocks = nn.ModuleList(
            CrossAttentionBlock(hidden_dim, heads, dropout) for _ in range(layers)
        )
        self.output_norm = nn.LayerNorm(hidden_dim)
        nn.init.normal_(self.queries, std=0.02)
        nn.init.normal_(self.modality_embedding, std=0.02)

    def forward(self, camera_tokens, lidar_tokens, ego_token):
        camera = camera_tokens + self.modality_embedding[0]
        lidar = lidar_tokens + self.modality_embedding[1]
        ego = ego_token.unsqueeze(1) + self.modality_embedding[2]
        memory = torch.cat((camera, lidar, ego), dim=1)
        slots = self.queries.expand(memory.shape[0], -1, -1)
        for block in self.blocks:
            slots = block(slots, memory)
        return self.output_norm(slots)


# V1 import compatibility.
FusionTransformer = DynamicQueryFusion
