"""SimLingo-Base driving-only network used by the MORAI teacher checkpoint.

The checkpoint is self contained: both the CLIP vision tower and the scratch
LLaMA decoder are stored in ``model_state``.  Consequently this module never
downloads model weights at runtime.
"""

import torch
from torch import nn
from transformers import (
    CLIPVisionConfig,
    CLIPVisionModel,
    LlamaConfig,
    LlamaModel,
)


CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)


class SimLingoBaseMorai(nn.Module):
    """CLIP ViT-L/14-336 plus a compact LLaMA-style driving decoder."""

    def __init__(self):
        super().__init__()
        hidden_size = 512
        self.route_queries = nn.Parameter(torch.empty(1, 20, hidden_size))
        self.speed_queries = nn.Parameter(torch.empty(1, 10, hidden_size))

        vision_config = CLIPVisionConfig(
            hidden_size=1024,
            intermediate_size=4096,
            num_hidden_layers=24,
            num_attention_heads=16,
            image_size=336,
            patch_size=14,
            hidden_act="quick_gelu",
        )
        self.vision = CLIPVisionModel(vision_config)
        self.vision_projection = nn.Linear(1024, hidden_size, bias=False)
        self.speed_encoder = nn.Sequential(
            nn.Linear(1, 256), nn.ReLU(inplace=True), nn.Linear(256, hidden_size)
        )
        self.target_encoder = nn.Sequential(
            nn.Linear(2, 256), nn.ReLU(inplace=True), nn.Linear(256, hidden_size)
        )

        decoder_config = LlamaConfig(
            vocab_size=1,
            hidden_size=hidden_size,
            intermediate_size=2048,
            num_hidden_layers=12,
            num_attention_heads=8,
            num_key_value_heads=8,
            max_position_embeddings=2048,
            attention_bias=False,
            mlp_bias=False,
            use_cache=False,
        )
        self.decoder = LlamaModel(decoder_config)
        # This driving-only model supplies embeddings directly and was trained
        # without a vocabulary embedding table.
        del self.decoder.embed_tokens
        self.route_head = nn.Sequential(
            nn.Linear(hidden_size, 256), nn.SiLU(inplace=True), nn.Linear(256, 2, bias=False)
        )
        self.speed_head = nn.Sequential(
            nn.Linear(hidden_size, 256), nn.SiLU(inplace=True), nn.Linear(256, 2, bias=False)
        )

    def forward(self, image_patches, speed_mps, target_point):
        """Return the 20-point route and 10-point/2-second trajectory.

        ``image_patches`` has shape ``[B, 2, 3, 336, 336]``.  Speed and target
        normalization follow the original SimLingo-Base MORAI recipe.
        """
        batch_size, patch_count = image_patches.shape[:2]
        flat_images = image_patches.reshape(-1, 3, 336, 336)
        vision = self.vision(pixel_values=flat_images).last_hidden_state[:, 1:]
        vision = vision.reshape(batch_size, patch_count * vision.shape[1], 1024)
        vision = self.vision_projection(vision)

        speed_token = self.speed_encoder(speed_mps.reshape(batch_size, 1) / (64.0 / 3.6))
        target_token = self.target_encoder((target_point + 32.0) / 64.0)
        fixed = torch.cat((vision, speed_token[:, None], target_token[:, None]), dim=1)
        route_queries = self.route_queries.expand(batch_size, -1, -1)
        speed_queries = self.speed_queries.expand(batch_size, -1, -1)
        sequence = torch.cat((fixed, route_queries, speed_queries), dim=1)
        decoded = self.decoder(inputs_embeds=sequence, use_cache=False).last_hidden_state
        query_features = decoded[:, -30:]
        # Both heads were trained to emit point-to-point deltas, matching the
        # SimLingo DrivingAdaptor.  Convert them back to ego-relative points.
        route = self.route_head(query_features[:, :20]).cumsum(dim=1)
        trajectory = self.speed_head(query_features[:, 20:]).cumsum(dim=1)
        return route, trajectory


def preprocess_front_image(image_chw):
    """Turn an RGB 672x336 uint8 image into two normalized CLIP patches."""
    if image_chw.ndim != 3 or tuple(image_chw.shape) != (3, 336, 672):
        raise ValueError("front image must have shape [3, 336, 672]")
    image = torch.as_tensor(image_chw, dtype=torch.float32).div_(255.0)
    patches = torch.stack((image[:, :, :336], image[:, :, 336:]), dim=0)
    mean = patches.new_tensor(CLIP_MEAN).view(1, 3, 1, 1)
    std = patches.new_tensor(CLIP_STD).view(1, 3, 1, 1)
    return (patches - mean) / std
