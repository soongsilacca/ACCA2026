import torch
import torch.nn as nn
from torchvision.models import resnet18


class SharedCameraEncoder(nn.Module):
    """One ResNet18 shared by front/left/right cameras with camera-ID tokens."""

    def __init__(self, hidden_dim=192, pretrained=True, grid_size=4, views=3):
        super().__init__()
        try:
            from torchvision.models import ResNet18_Weights
            model = resnet18(weights=ResNet18_Weights.DEFAULT if pretrained else None)
        except (ImportError, TypeError):
            model = resnet18(pretrained=pretrained)
        self.backbone = nn.Sequential(*list(model.children())[:-2])
        self.pool = nn.AdaptiveAvgPool2d((grid_size, grid_size))
        self.projection = nn.Sequential(
            nn.Linear(model.fc.in_features, hidden_dim),
            nn.LayerNorm(hidden_dim),
        )
        self.spatial_embedding = nn.Parameter(torch.empty(1, grid_size * grid_size, hidden_dim))
        self.camera_embedding = nn.Embedding(views, hidden_dim)
        self.register_buffer("mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        nn.init.normal_(self.spatial_embedding, std=0.02)
        nn.init.normal_(self.camera_embedding.weight, std=0.02)

    def forward(self, image, camera_id):
        image = (image - self.mean) / self.std
        feature = self.pool(self.backbone(image)).flatten(2).transpose(1, 2)
        tokens = self.projection(feature) + self.spatial_embedding
        camera_ids = torch.full(
            (image.shape[0],), int(camera_id), dtype=torch.long, device=image.device
        )
        return tokens + self.camera_embedding(camera_ids).unsqueeze(1)


# Backward-compatible import name; V2 instantiates this shared module only once.
CameraEncoder = SharedCameraEncoder
