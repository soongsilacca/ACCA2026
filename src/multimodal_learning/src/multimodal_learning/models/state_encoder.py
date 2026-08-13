import torch.nn as nn


class MLPEncoder(nn.Module):
    def __init__(self, input_dim, hidden_dim=256, layers=3):
        super().__init__()
        modules, dim = [], input_dim
        for _ in range(layers):
            modules += [nn.Linear(dim, hidden_dim), nn.ReLU()]
            dim = hidden_dim
        self.net = nn.Sequential(*modules)

    def forward(self, value):
        return self.net(value)


class RouteEncoder(nn.Module):
    def __init__(self, input_dim=4, hidden_dim=256):
        super().__init__()
        self.mlp = MLPEncoder(input_dim, hidden_dim, 2)

    def forward(self, tokens, valid_mask):
        encoded = self.mlp(tokens)
        safe_mask = valid_mask.bool().clone()
        safe_mask[~safe_mask.any(dim=1), 0] = True
        return encoded, safe_mask
