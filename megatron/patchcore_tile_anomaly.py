"""PatchCore-style anomaly detector for the live Task 2 tile pipeline.

Drop-in alternative to TileAnomalyDetector (tile_anomaly.py): same TileAnomalyResult
return type and from_paths()/detect() interface, but compares each sample's deep
patch features (frozen pretrained backbone) against a per-belt memory bank built
from real in-domain captures under assets/tiles/reference_good/, instead of doing an
ECC-aligned pixel diff against a single chosen reference image.

Each world/belt only has 3-5 real reference images (see assets/tiles/reference_good/),
too few for a held-out calibration split, so the threshold is calibrated via
leave-one-out: each reference is scored against a bank built from the others, and the
threshold is set a configurable margin above the worst of those held-out scores.
"""

from __future__ import annotations

import random
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import yaml
from torchvision.models import (
    ResNet18_Weights,
    ResNet50_Weights,
    Wide_ResNet50_2_Weights,
    resnet18,
    resnet50,
    wide_resnet50_2,
)

from megatron.tile_anomaly import TileAnomalyResult, binary_damage_mask

# Both OpenCV and PyTorch default to spawning a thread pool sized to the core count
# for every resize/cvtColor call and every CPU tensor op, respectively. That's fine in
# isolation, but task2_controller starts alongside Gazebo, Nav2, and ~30 other ROS
# nodes all competing for the same cores — letting this module's preprocessing fan out
# across all of them on top of that is what was driving the CPU to the point of a
# desktop-wide freeze. Capping both to a couple of threads keeps this module's CPU
# footprint small; the actual backbone forward pass still runs on the GPU.
cv2.setNumThreads(2)
try:
    torch.set_num_threads(2)
    torch.set_num_interop_threads(2)
except RuntimeError:
    pass  # interop threads can only be set once per process; ignore if already set

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

BACKBONES = {
    "resnet18": (resnet18, ResNet18_Weights.IMAGENET1K_V1),
    "resnet50": (resnet50, ResNet50_Weights.IMAGENET1K_V2),
    "wide_resnet50_2": (wide_resnet50_2, Wide_ResNet50_2_Weights.IMAGENET1K_V2),
}


@dataclass(frozen=True)
class PatchCoreConfig:
    image_size: int = 256
    max_bank_size: int = 6000
    backbone: str = "wide_resnet50_2"
    layers: tuple[str, ...] = ("layer2", "layer3")
    val_fraction: float = 0.2  # fraction of good refs held out to calibrate the threshold
    percentile: float = 99.0  # percentile of held-out good scores used as the threshold
    seed: int = 0
    batch_size: int = 16  # reference images embedded per forward pass during bank-building

    @classmethod
    def from_yaml(cls, path: str | Path) -> PatchCoreConfig:
        with Path(path).open() as stream:
            values = yaml.safe_load(stream) or {}
        values = values.get("patchcore_detector", values)
        known = cls.__dataclass_fields__
        filtered = {key: value for key, value in values.items() if key in known}
        if "layers" in filtered:
            filtered["layers"] = tuple(filtered["layers"])
        return cls(**filtered)


class _FeatureExtractor:
    """Frozen pretrained backbone exposing locally-aggregated, spatially-aligned patch
    embeddings concatenated across `layers` (the standard PatchCore feature recipe)."""

    def __init__(self, backbone: str, layers: Sequence[str], device: torch.device) -> None:
        ctor, weights = BACKBONES[backbone]
        model = ctor(weights=weights)
        model.eval()
        for parameter in model.parameters():
            parameter.requires_grad_(False)
        self.model = model.to(device)
        self.device = device
        self.layers = list(layers)
        self._outputs: dict[str, torch.Tensor] = {}
        for name in self.layers:
            getattr(model, name).register_forward_hook(self._make_hook(name))
        self.mean = torch.tensor(IMAGENET_MEAN, device=device).view(1, 3, 1, 1)
        self.std = torch.tensor(IMAGENET_STD, device=device).view(1, 3, 1, 1)

    def _make_hook(self, name: str):
        def hook(_module, _input, output):
            self._outputs[name] = output

        return hook

    @torch.no_grad()
    def __call__(self, batch: torch.Tensor) -> torch.Tensor:
        self._outputs.clear()
        normalized = (batch - self.mean) / self.std
        self.model(normalized)
        feats = [self._outputs[name] for name in self.layers]
        pooled = [F.avg_pool2d(f, kernel_size=3, stride=1, padding=1) for f in feats]
        target_size = pooled[0].shape[-2:]
        resized = [
            f
            if f.shape[-2:] == target_size
            else F.interpolate(f, size=target_size, mode="bilinear", align_corners=False)
            for f in pooled
        ]
        return torch.cat(resized, dim=1)


def _bgr_to_tensor(bgr: np.ndarray, image_size: int) -> torch.Tensor:
    resized = cv2.resize(bgr, (image_size, image_size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return torch.from_numpy(rgb).permute(2, 0, 1).contiguous()  # [3,H,W], CPU


def _embed(
    image: np.ndarray, extractor: _FeatureExtractor, image_size: int, device: torch.device
) -> torch.Tensor:
    """Returns [C,Hf,Wf] patch embeddings for one image."""
    batch = _bgr_to_tensor(image, image_size).unsqueeze(0).to(device)
    return extractor(batch)[0]


def _embed_many(
    images: Sequence[np.ndarray],
    extractor: _FeatureExtractor,
    image_size: int,
    device: torch.device,
    batch_size: int,
) -> list[torch.Tensor]:
    """Returns one [C,Hf,Wf] CPU patch-embedding tensor per image.

    Mirrors scripts/patchcore_tile_detector.py's embed_patches: images are embedded
    `batch_size` at a time and moved off the GPU immediately after each forward pass,
    so GPU memory stays bounded by one batch regardless of how many reference images
    are supplied (unlike embedding every image at once and keeping all results resident
    on GPU, which OOMs once the reference pool grows past a handful of images).
    """
    results: list[torch.Tensor] = []
    for start in range(0, len(images), batch_size):
        chunk = images[start : start + batch_size]
        batch = torch.stack([_bgr_to_tensor(image, image_size) for image in chunk]).to(device)
        feats = extractor(batch)
        results.extend(feat.cpu() for feat in feats)
    return results


def _patches_to_bank(patches: torch.Tensor) -> torch.Tensor:
    c, h, w = patches.shape
    return patches.permute(1, 2, 0).reshape(h * w, c)


def _subsample_bank(features: torch.Tensor, max_size: int, seed: int) -> torch.Tensor:
    if features.shape[0] <= max_size:
        return features
    generator = torch.Generator().manual_seed(seed)
    indices = torch.randperm(features.shape[0], generator=generator)[:max_size]
    return features[indices]


def _score(
    patches: torch.Tensor, bank: torch.Tensor, device: torch.device
) -> tuple[float, np.ndarray]:
    c, h, w = patches.shape
    flat = patches.permute(1, 2, 0).reshape(h * w, c).to(device)
    distances = torch.cdist(flat, bank.to(device))
    nearest = distances.min(dim=1).values
    anomaly_map = nearest.reshape(h, w).cpu().numpy()
    return float(anomaly_map.max()), anomaly_map


class PatchCoreTileDetector:
    """Drop-in TileAnomalyDetector alternative using deep patch-feature nearest-neighbor
    scoring instead of an ECC-aligned pixel diff."""

    def __init__(
        self,
        references: dict[str, np.ndarray],
        config: PatchCoreConfig | None = None,
        device: str | None = None,
    ) -> None:
        self.config = config or PatchCoreConfig()
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self._extractor = _FeatureExtractor(
            self.config.backbone, self.config.layers, self.device
        )
        if not references:
            raise ValueError("at least one good reference image is required")
        self._build(sorted(references), references.__getitem__)

    @classmethod
    def from_paths(
        cls,
        paths: Sequence[str | Path],
        config: PatchCoreConfig | None = None,
        device: str | None = None,
    ) -> PatchCoreTileDetector:
        names = sorted(str(Path(value)) for value in paths)
        if not names:
            raise ValueError("at least one good reference image is required")

        def _load(name: str) -> np.ndarray:
            image = cv2.imread(name, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"could not read reference image: {name}")
            return image

        self = cls.__new__(cls)
        self.config = config or PatchCoreConfig()
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self._extractor = _FeatureExtractor(
            self.config.backbone, self.config.layers, self.device
        )
        self._build(names, _load)
        return self

    def _build(self, names: list[str], image_for: Callable[[str], np.ndarray]) -> None:
        """Builds self.bank/self.threshold by streaming reference images through the
        backbone batch-by-batch, never holding more than one batch's embeddings (plus
        the capped bank) in memory at once.

        Holding one [C,Hf,Wf] embedding per reference image in a dict until calibration
        runs — what this used to do — costs ~6 MB/image regardless of batching the raw
        image decode: with a few hundred references that's gigabytes, which is what was
        actually driving task2_controller's RSS past several GB and getting it
        OOM-killed (confirmed via scripts/monitor_resources.sh: RSS climbed linearly and
        monotonically with the number of images processed, not in bounded batch steps).
        Folding each batch directly into a running, resubsampled bank avoids ever
        materializing that per-image dict.
        """
        rng = random.Random(self.config.seed)
        shuffled = list(names)
        rng.shuffle(shuffled)
        n_val = (
            round(len(shuffled) * self.config.val_fraction) if len(shuffled) > 4 else 0
        )
        calib_names = shuffled[:n_val]
        bank_names = shuffled[n_val:] if n_val else shuffled
        batch_size = self.config.batch_size

        if not n_val:
            # Too few references for a held-out split (<=4): fall back to leave-one-out,
            # which needs every embedding simultaneously anyway — cheap at this size.
            embeddings = dict(zip(names, self._embed_batches(names, image_for, batch_size)))
            full_bank = _subsample_bank(
                torch.cat([_patches_to_bank(embeddings[n]) for n in names], dim=0),
                self.config.max_bank_size,
                self.config.seed,
            )
            held_out_scores = [
                _score(
                    embeddings[held_out],
                    torch.cat(
                        [_patches_to_bank(embeddings[n]) for n in names if n != held_out],
                        dim=0,
                    ),
                    self.device,
                )[0]
                for held_out in names
            ]
            worst_good_score = max(held_out_scores) if held_out_scores else 0.0
            self.threshold = worst_good_score * 1.10
            self.bank = full_bank
            return

        bank = torch.empty((0, 0))
        for start in range(0, len(bank_names), batch_size):
            chunk = bank_names[start : start + batch_size]
            chunk_patches = torch.cat(
                [_patches_to_bank(emb) for emb in self._embed_batches(chunk, image_for, batch_size)],
                dim=0,
            )
            bank = chunk_patches if bank.numel() == 0 else torch.cat([bank, chunk_patches], dim=0)
            bank = _subsample_bank(bank, self.config.max_bank_size, self.config.seed)
        full_bank = bank

        held_out_scores = [
            _score(embedding, full_bank, self.device)[0]
            for embedding in self._embed_batches(calib_names, image_for, batch_size)
        ]
        self.threshold = float(np.percentile(held_out_scores, self.config.percentile))
        self.bank = full_bank

    def _embed_batches(
        self, names: list[str], image_for: Callable[[str], np.ndarray], batch_size: int
    ) -> list[torch.Tensor]:
        results: list[torch.Tensor] = []
        for start in range(0, len(names), batch_size):
            chunk = names[start : start + batch_size]
            images = [image_for(name) for name in chunk]
            results.extend(
                _embed_many(images, self._extractor, self.config.image_size, self.device, batch_size)
            )
            del images
        return results

    def detect(self, sample: np.ndarray) -> TileAnomalyResult:
        patches = _embed(sample, self._extractor, self.config.image_size, self.device)
        score, anomaly_map = _score(patches, self.bank, self.device)
        output_shape = sample.shape[:2]
        mask = binary_damage_mask(anomaly_map, output_shape, self.threshold)
        defect_area = int(cv2.countNonZero(mask))
        status = "DEFECT" if score >= self.threshold else "OK"
        return TileAnomalyResult(
            status=status,
            defect_area=defect_area,
            defect_ratio=defect_area / float(mask.size),
            reference_name="patchcore",
            anomaly_score=score,
            anomaly_threshold=self.threshold,
            mask=mask,
            anomaly_map=anomaly_map,
        )
