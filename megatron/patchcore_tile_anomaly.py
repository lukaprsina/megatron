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

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import yaml
from torchvision.models import Wide_ResNet50_2_Weights, wide_resnet50_2

from megatron.tile_anomaly import TileAnomalyResult, binary_damage_mask

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


@dataclass(frozen=True)
class PatchCoreConfig:
    image_size: int = 256
    max_bank_size: int = 6000
    threshold_margin: float = 0.10  # fraction above the worst leave-one-out good score

    @classmethod
    def from_yaml(cls, path: str | Path) -> PatchCoreConfig:
        with Path(path).open() as stream:
            values = yaml.safe_load(stream) or {}
        values = values.get("patchcore_detector", values)
        known = cls.__dataclass_fields__
        return cls(**{key: value for key, value in values.items() if key in known})


class _FeatureExtractor:
    """Frozen wide_resnet50_2 exposing locally-aggregated, spatially-aligned patch
    embeddings concatenated across layer2+layer3 (the standard PatchCore recipe)."""

    def __init__(self, device: torch.device) -> None:
        model = wide_resnet50_2(weights=Wide_ResNet50_2_Weights.IMAGENET1K_V2)
        model.eval()
        for parameter in model.parameters():
            parameter.requires_grad_(False)
        self.model = model.to(device)
        self.device = device
        self.layers = ["layer2", "layer3"]
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


def _bgr_to_batch(bgr: np.ndarray, image_size: int, device: torch.device) -> torch.Tensor:
    resized = cv2.resize(bgr, (image_size, image_size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).contiguous()
    return tensor.to(device)


def _embed(
    image: np.ndarray, extractor: _FeatureExtractor, image_size: int, device: torch.device
) -> torch.Tensor:
    """Returns [C,Hf,Wf] patch embeddings for one image."""
    return extractor(_bgr_to_batch(image, image_size, device))[0]


def _patches_to_bank(patches: torch.Tensor) -> torch.Tensor:
    c, h, w = patches.shape
    return patches.permute(1, 2, 0).reshape(h * w, c)


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
        self._extractor = _FeatureExtractor(self.device)

        if not references:
            raise ValueError("at least one good reference image is required")

        embeddings = {
            name: _embed(image, self._extractor, self.config.image_size, self.device)
            for name, image in references.items()
        }
        names = list(embeddings)

        # Leave-one-out calibration: score each reference against a bank built from
        # the others, since there are too few images (often 3-5) for a held-out split.
        held_out_scores = [
            _score(
                embeddings[held_out],
                torch.cat(
                    [_patches_to_bank(embeddings[n]) for n in names if n != held_out], dim=0
                ),
                self.device,
            )[0]
            for held_out in names
        ]
        worst_good_score = max(held_out_scores) if held_out_scores else 0.0
        self.threshold = worst_good_score * (1.0 + self.config.threshold_margin)

        full_bank = torch.cat([_patches_to_bank(embeddings[n]) for n in names], dim=0)
        if full_bank.shape[0] > self.config.max_bank_size:
            generator = torch.Generator().manual_seed(0)
            indices = torch.randperm(full_bank.shape[0], generator=generator)[
                : self.config.max_bank_size
            ]
            full_bank = full_bank[indices]
        self.bank = full_bank

    @classmethod
    def from_paths(
        cls,
        paths: Sequence[str | Path],
        config: PatchCoreConfig | None = None,
        device: str | None = None,
    ) -> PatchCoreTileDetector:
        references: dict[str, np.ndarray] = {}
        for value in paths:
            path = Path(value)
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"could not read reference image: {path}")
            references[path.name] = image
        return cls(references, config, device)

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
