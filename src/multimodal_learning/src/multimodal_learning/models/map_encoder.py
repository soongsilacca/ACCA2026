import torch
import torch.nn as nn


class MapEncoder(nn.Module):
    def __init__(self, input_dim=8, hidden_dim=256, heads=8, layers=2, dropout=0.1):
        super().__init__()
        self.point_mlp = nn.Sequential(nn.Linear(input_dim, hidden_dim), nn.ReLU(), nn.Linear(hidden_dim, hidden_dim))
        layer = nn.TransformerEncoderLayer(hidden_dim, heads, hidden_dim * 4, dropout, batch_first=True)
        self.transformer = nn.TransformerEncoder(layer, layers)

    def forward(self, tokens, valid_mask):
        valid_mask = valid_mask.bool()
        # Transformer attention cannot accept a row where every token is padded.
        safe_mask = valid_mask.clone()
        safe_mask[~safe_mask.any(dim=1), 0] = True
        encoded = self.transformer(self.point_mlp(tokens), src_key_padding_mask=~safe_mask)
        return encoded, safe_mask
