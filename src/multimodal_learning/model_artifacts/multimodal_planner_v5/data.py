from __future__ import annotations

import json
import random
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
TARGET_SCALES = np.asarray([50.0, 50.0, np.pi, 20.0], dtype=np.float32)
V5_TARGET_FIELDS = (
    "relative_x",
    "relative_y",
    "relative_yaw",
    "future_speed",
)


@dataclass(frozen=True)
class PhotometricAugmentation:
    brightness: float = 1.0
    contrast: float = 1.0
    saturation: float = 1.0
    hue_shift: float = 0.0
    fog_strength: float = 0.0


def _sample_photometric_augmentation() -> PhotometricAugmentation:
    """Sample one transform shared by all views and history frames."""
    jitter_enabled = bool(torch.rand(()) < 0.8)
    fog_enabled = bool(torch.rand(()) < 0.25)

    def uniform(low: float, high: float) -> float:
        return float(torch.empty(()).uniform_(low, high))

    return PhotometricAugmentation(
        brightness=uniform(0.8, 1.2) if jitter_enabled else 1.0,
        contrast=uniform(0.8, 1.2) if jitter_enabled else 1.0,
        saturation=uniform(0.85, 1.15) if jitter_enabled else 1.0,
        hue_shift=uniform(-0.03, 0.03) if jitter_enabled else 0.0,
        fog_strength=uniform(0.08, 0.28) if fog_enabled else 0.0,
    )


def _apply_photometric_augmentation(
    image: np.ndarray,
    augmentation: PhotometricAugmentation,
) -> np.ndarray:
    """Apply geometry-preserving RGB jitter and depth-like fog."""
    if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
        raise ValueError("photometric augmentation expects uint8 RGB [H,W,3]")

    value = image.astype(np.float32) / 255.0
    value *= augmentation.brightness
    mean = value.mean(axis=(0, 1), keepdims=True)
    value = (value - mean) * augmentation.contrast + mean
    luminance = (
        0.299 * value[..., 0:1]
        + 0.587 * value[..., 1:2]
        + 0.114 * value[..., 2:3]
    )
    value = luminance + augmentation.saturation * (value - luminance)
    value = np.clip(value, 0.0, 1.0)

    if augmentation.hue_shift != 0.0:
        hsv = cv2.cvtColor(
            np.rint(value * 255.0).astype(np.uint8),
            cv2.COLOR_RGB2HSV,
        )
        hue = hsv[..., 0].astype(np.int16)
        hue = (hue + round(augmentation.hue_shift * 180.0)) % 180
        hsv[..., 0] = hue.astype(np.uint8)
        value = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).astype(np.float32) / 255.0

    if augmentation.fog_strength > 0.0:
        height = value.shape[0]
        vertical = np.linspace(1.0, 0.0, height, dtype=np.float32)[:, None, None]
        veil = augmentation.fog_strength * (0.65 + 0.35 * vertical)
        fog_color = np.asarray([0.92, 0.94, 0.96], dtype=np.float32)
        value = value * (1.0 - veil) + fog_color * veil

    return np.rint(np.clip(value, 0.0, 1.0) * 255.0).astype(np.uint8)


def deterministic_run_split(
    run_ids: list[str],
    seed: int,
    train_ratio: float = 0.80,
    val_ratio: float = 0.10,
) -> dict[str, list[str]]:
    if not run_ids:
        raise ValueError("no run IDs were provided")
    ids = sorted(run_ids)
    random.Random(seed).shuffle(ids)
    count = len(ids)
    test_count = max(1, round(count * (1.0 - train_ratio - val_ratio)))
    val_count = max(1, round(count * val_ratio))
    train_count = count - val_count - test_count
    if train_count < 1:
        raise ValueError("not enough runs for train/validation/test split")
    return {
        "train": sorted(ids[:train_count]),
        "val": sorted(ids[train_count : train_count + val_count]),
        "test": sorted(ids[train_count + val_count :]),
    }


def save_split_manifest(
    path: Path,
    splits: dict[str, list[str]],
    seed: int,
    data_root: Path,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "split_unit": "run",
        "seed": seed,
        "data_root": str(data_root.resolve()),
        "splits": splits,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_split_manifest(path: Path) -> dict[str, list[str]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("split_unit") != "run":
        raise ValueError(f"{path}: split_unit must be 'run'")
    return payload["splits"]


@dataclass(frozen=True)
class FrameChunk:
    path: Path
    start: int
    end: int


class RunStore:
    def __init__(
        self,
        run_dir: Path,
        cache_chunks: int = 3,
        allow_legacy_target_fields: bool = False,
    ) -> None:
        self.run_dir = run_dir
        self.run_id = run_dir.name
        with np.load(run_dir / "sample_index.npz", allow_pickle=False) as sample:
            self.sample_id = np.asarray(sample["sample_id"], dtype=np.int64)
            self.current_frame_idx = np.asarray(
                sample["current_frame_idx"], dtype=np.int64
            )
            self.history_frame_idx = np.asarray(
                sample["history_frame_idx"], dtype=np.int64
            )
            self.future_frame_idx = np.asarray(
                sample["future_frame_idx"], dtype=np.int64
            )
            self.gps_blackout = np.asarray(sample["gps_blackout"], dtype=np.bool_)
            if all(field in sample.files for field in V5_TARGET_FIELDS):
                self.target = np.stack(
                    [
                        np.asarray(sample[field], dtype=np.float32)
                        for field in V5_TARGET_FIELDS
                    ],
                    axis=-1,
                )
                self.target_schema = "v5_relative_fields"
            elif allow_legacy_target_fields and {
                "future_xy",
                "future_yaw",
                "future_speed",
            }.issubset(sample.files):
                future_xy = np.asarray(sample["future_xy"], dtype=np.float32)
                self.target = np.stack(
                    (
                        future_xy[..., 0],
                        future_xy[..., 1],
                        np.asarray(sample["future_yaw"], dtype=np.float32),
                        np.asarray(sample["future_speed"], dtype=np.float32),
                    ),
                    axis=-1,
                )
                self.target_schema = (
                    "legacy_future_fields_mapped_without_numeric_correction"
                )
            else:
                missing = [
                    field for field in V5_TARGET_FIELDS if field not in sample.files
                ]
                raise KeyError(
                    f"{run_dir / 'sample_index.npz'}: missing V5 target fields "
                    f"{missing}; expected {V5_TARGET_FIELDS}. Use "
                    "allow_legacy_target_fields=True only for an explicit "
                    "no-correction compatibility check."
                )
            if self.target.ndim != 3 or self.target.shape[-2:] != (20, 4):
                raise ValueError(
                    f"{run_dir / 'sample_index.npz'}: V5 target must have shape "
                    f"[samples, 20, 4], got {self.target.shape}"
                )
            if not np.isfinite(self.target).all():
                raise ValueError(
                    f"{run_dir / 'sample_index.npz'}: V5 target contains non-finite "
                    "values"
                )

        chunk_manifest = json.loads(
            (run_dir / "frame_chunks.json").read_text(encoding="utf-8")
        )
        entries = chunk_manifest.get(
            "frame_chunks", chunk_manifest.get("chunks", chunk_manifest)
        )
        self.chunks = [
            FrameChunk(
                run_dir / item["file"],
                int(item["start_frame"]),
                int(item["end_frame_exclusive"]),
            )
            for item in entries
        ]
        self.cache_chunks = cache_chunks
        self._cache: OrderedDict[int, dict[str, np.ndarray]] = OrderedDict()
        self.repaired_jump_count = 0

    def __len__(self) -> int:
        return len(self.sample_id)

    def _chunk_index(self, frame_index: int) -> int:
        # Chunks are contiguous and normally contain 100 frames.
        candidate = min(frame_index // 100, len(self.chunks) - 1)
        chunk = self.chunks[candidate]
        if chunk.start <= frame_index < chunk.end:
            return candidate
        for index, chunk in enumerate(self.chunks):
            if chunk.start <= frame_index < chunk.end:
                return index
        raise IndexError(f"{self.run_id}: frame index {frame_index} is out of range")

    def _load_chunk(self, chunk_index: int) -> dict[str, np.ndarray]:
        cached = self._cache.pop(chunk_index, None)
        if cached is not None:
            self._cache[chunk_index] = cached
            return cached
        path = self.chunks[chunk_index].path
        keys = (
            "lidar_bev",
            "imu",
            "vehicle",
            "health",
            "route",
            "mgeo",
            "front_jpeg_data",
            "front_jpeg_offsets",
            "left_jpeg_data",
            "left_jpeg_offsets",
            "right_jpeg_data",
            "right_jpeg_offsets",
        )
        with np.load(path, allow_pickle=False) as frame:
            loaded = {key: np.asarray(frame[key]) for key in keys}
        self._cache[chunk_index] = loaded
        while len(self._cache) > self.cache_chunks:
            self._cache.popitem(last=False)
        return loaded

    @staticmethod
    def _decode_image(
        chunk: dict[str, np.ndarray], camera: str, local_index: int
    ) -> np.ndarray:
        data = chunk[f"{camera}_jpeg_data"]
        offsets = chunk[f"{camera}_jpeg_offsets"]
        encoded = data[offsets[local_index] : offsets[local_index + 1]]
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to decode {camera} JPEG")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def frame(self, frame_index: int) -> dict[str, np.ndarray]:
        chunk_index = self._chunk_index(frame_index)
        chunk_info = self.chunks[chunk_index]
        local_index = frame_index - chunk_info.start
        chunk = self._load_chunk(chunk_index)
        return {
            "front": self._decode_image(chunk, "front", local_index),
            "left": self._decode_image(chunk, "left", local_index),
            "right": self._decode_image(chunk, "right", local_index),
            "lidar_bev": chunk["lidar_bev"][local_index],
            "imu": chunk["imu"][local_index],
            "vehicle": chunk["vehicle"][local_index],
            "health": chunk["health"][local_index],
            "route": chunk["route"][local_index],
            "mgeo": chunk["mgeo"][local_index],
        }


def _image_tensor(image: np.ndarray) -> torch.Tensor:
    value = image.astype(np.float32) / 255.0
    value = (value - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(np.ascontiguousarray(value.transpose(2, 0, 1)))


def _ego_vector(frame: dict[str, np.ndarray]) -> np.ndarray:
    vehicle = np.asarray(frame["vehicle"], dtype=np.float32).copy()
    imu = np.asarray(frame["imu"], dtype=np.float32).copy()
    health = np.asarray(frame["health"], dtype=np.float32).copy()
    vehicle /= np.asarray([30.0, 30.0, 10.0, 0.7, 10.0], dtype=np.float32)
    imu /= np.asarray([5.0, 5.0, 5.0, 20.0, 20.0, 20.0], dtype=np.float32)
    health[1] = np.clip(health[1] / 15.0, 0.0, 1.0)
    health[4] = np.clip(health[4] / 1.0, 0.0, 1.0)
    return np.concatenate((vehicle, imu, health)).astype(np.float32)


class PlannerDataset(Dataset):
    def __init__(
        self,
        data_root: Path,
        run_ids: list[str],
        blackout_weight: float = 2.0,
        max_samples: int = 0,
        seed: int = 2026,
        allow_legacy_target_fields: bool = False,
        photometric_augmentation: bool = False,
    ) -> None:
        self.runs = [
            RunStore(
                data_root / run_id,
                allow_legacy_target_fields=allow_legacy_target_fields,
            )
            for run_id in run_ids
        ]
        self.lookup = [
            (run_index, sample_index)
            for run_index, run in enumerate(self.runs)
            for sample_index in range(len(run))
        ]
        if max_samples > 0 and len(self.lookup) > max_samples:
            rng = random.Random(seed)
            rng.shuffle(self.lookup)
            self.lookup = self.lookup[:max_samples]
        self.blackout_weight = blackout_weight
        self.photometric_augmentation = photometric_augmentation
        self.blackout_samples = sum(
            int(self.runs[run_index].gps_blackout[sample_index])
            for run_index, sample_index in self.lookup
        )

    @property
    def mean_sample_weight(self) -> float:
        if not self.lookup:
            return 1.0
        blackout_fraction = self.blackout_samples / len(self.lookup)
        return 1.0 + (self.blackout_weight - 1.0) * blackout_fraction

    def __len__(self) -> int:
        return len(self.lookup)

    def __getitem__(self, item: int) -> dict[str, Any]:
        run_index, sample_index = self.lookup[item]
        run = self.runs[run_index]
        history_indices = run.history_frame_idx[sample_index]
        frames = [run.frame(int(index)) for index in history_indices]
        current = frames[-1]

        if self.photometric_augmentation:
            augmentation = _sample_photometric_augmentation()
            for frame in frames:
                for camera in ("front", "left", "right"):
                    frame[camera] = _apply_photometric_augmentation(
                        frame[camera], augmentation
                    )

        front = torch.stack([_image_tensor(frame["front"]) for frame in frames])
        left = torch.stack([_image_tensor(frame["left"]) for frame in frames])
        right = torch.stack([_image_tensor(frame["right"]) for frame in frames])
        lidar = torch.stack(
            [
                torch.from_numpy(
                    np.ascontiguousarray(frame["lidar_bev"].astype(np.float32) / 255.0)
                )
                for frame in frames
            ]
        )
        ego = torch.stack(
            [torch.from_numpy(_ego_vector(frame)) for frame in frames]
        )
        route = np.asarray(current["route"], dtype=np.float32).copy()
        route[:, :2] /= 50.0

        target = run.target[sample_index].copy()
        target /= TARGET_SCALES
        blackout = bool(run.gps_blackout[sample_index])
        return {
            "front": front,
            "left": left,
            "right": right,
            "lidar_bev": lidar,
            "ego": ego,
            "mgeo": torch.from_numpy(
                np.ascontiguousarray(current["mgeo"].astype(np.float32))
            ),
            "local_route": torch.from_numpy(np.ascontiguousarray(route)),
            "target": torch.from_numpy(np.ascontiguousarray(target)),
            "sample_weight": torch.tensor(
                self.blackout_weight if blackout else 1.0, dtype=torch.float32
            ),
            "gps_blackout": torch.tensor(blackout),
            "run_id": run.run_id,
            "sample_id": int(run.sample_id[sample_index]),
        }

    def summary(self) -> dict[str, Any]:
        return {
            "runs": len(self.runs),
            "samples": len(self.lookup),
            "blackout_samples": self.blackout_samples,
            "blackout_weight": self.blackout_weight,
            "mean_sample_weight": self.mean_sample_weight,
            "target_fields": list(V5_TARGET_FIELDS),
            "target_schemas": sorted({run.target_schema for run in self.runs}),
            "target_shape": [20, 4],
            "target_scales": TARGET_SCALES.tolist(),
            "label_policy": "provided_relative_values_without_correction",
            "augmentation": {
                "enabled": self.photometric_augmentation,
                "scope": "train_only_shared_across_5_frames_and_3_cameras",
                "color_jitter_probability": 0.8,
                "brightness_factor": [0.8, 1.2],
                "contrast_factor": [0.8, 1.2],
                "saturation_factor": [0.85, 1.15],
                "hue_shift": [-0.03, 0.03],
                "fog_probability": 0.25,
                "fog_strength": [0.08, 0.28],
            },
        }
