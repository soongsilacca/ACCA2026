from pathlib import Path
import numpy as np
import torch
from torch.utils.data import Dataset


class MultimodalDataset(Dataset):
    def __init__(self, root):
        self.files = sorted(Path(root).expanduser().glob("*.npz"))
        if not self.files:
            raise RuntimeError("No .npz samples found in {}".format(root))

    def __len__(self):
        return len(self.files)

    def __getitem__(self, index):
        with np.load(str(self.files[index])) as item:
            sample = {key: torch.from_numpy(item[key].copy()) for key in item.files}
        required = {"camera_front", "camera_left", "camera_right", "lidar", "ego",
                    "map_tokens", "map_mask", "route_tokens", "route_mask", "trajectory"}
        missing = required.difference(sample)
        if missing:
            raise RuntimeError("Dataset is not V2; missing keys: {}".format(sorted(missing)))
        if sample["trajectory"].shape != (20, 4):
            raise RuntimeError(
                "Dataset trajectory must be V2 [20,4] absolute local x,y,yaw,speed"
            )
        for key in ("camera_front", "camera_left", "camera_right"):
            sample[key] = sample[key].float() / 255.0
        return sample
