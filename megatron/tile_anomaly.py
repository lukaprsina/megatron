from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import yaml

TileStatus = Literal["OK", "DEFECT", "UNKNOWN"]


@dataclass(frozen=True)
class DetectorConfig:
    image_size: int = 512
    border_inset: float = 0.06
    min_stddev: float = 7.0
    max_alignment_error: float = 0.34
    color_threshold: float = 8.0
    self_color_threshold: float = 6.0
    self_color_min_area: int = 180
    strong_color_area: int = 600
    intensity_threshold: float = 24.0
    gradient_threshold: float = 34.0
    dark_line_threshold: float = 18.0
    dark_segment_score: float = 180.0
    min_segment_length: float = 20.0
    strong_line_area: int = 250
    min_component_area: int = 18
    min_component_contrast: float = 10.0
    min_defect_area: int = 24
    ecc_iterations: int = 35
    ecc_epsilon: float = 1.0e-4

    @classmethod
    def from_yaml(cls, path: str | Path) -> DetectorConfig:
        with Path(path).open() as stream:
            values = yaml.safe_load(stream) or {}
        values = values.get("tile_detector", values)
        known = cls.__dataclass_fields__
        return cls(**{key: value for key, value in values.items() if key in known})


@dataclass
class TileAnomalyResult:
    status: TileStatus
    defect_area: int = 0
    defect_ratio: float = 0.0
    reference_name: str | None = None
    alignment_error: float | None = None
    reason: str | None = None
    mask: np.ndarray | None = field(default=None, repr=False)
    aligned_reference: np.ndarray | None = field(default=None, repr=False)
    normalized_sample: np.ndarray | None = field(default=None, repr=False)
    evidence: dict[str, np.ndarray] = field(default_factory=dict, repr=False)
    component_scores: list[dict[str, float | int]] = field(
        default_factory=list, repr=False
    )


@dataclass(frozen=True)
class _ReferenceVariant:
    name: str
    image: np.ndarray
    normalized: np.ndarray
    gray: np.ndarray


def order_quad(points: np.ndarray) -> np.ndarray:
    """Return corners clockwise as top-left, top-right, bottom-right, bottom-left."""
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered.sum(axis=1)))
    ordered = np.roll(ordered, -start, axis=0)
    if np.cross(ordered[1] - ordered[0], ordered[2] - ordered[1]) < 0:
        ordered = ordered[[0, 3, 2, 1]]
    return ordered.astype(np.float32)


def warp_tile(
    image: np.ndarray,
    quad: np.ndarray,
    size: int = 512,
    border_inset: float = 0.06,
) -> np.ndarray:
    if image is None or image.size == 0:
        raise ValueError("image is empty")
    source = order_quad(quad)
    center = source.mean(axis=0)
    source = source + border_inset * (center - source)
    destination = np.array(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(
        image,
        transform,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def normalize_lab(image: np.ndarray, size: int = 512) -> np.ndarray:
    image = _as_bgr(image)
    if image.shape[:2] != (size, size):
        image = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lightness, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lightness = clahe.apply(lightness)
    return cv2.merge((lightness, a_channel, b_channel))


class TileAnomalyDetector:
    def __init__(
        self,
        references: dict[str, np.ndarray],
        config: DetectorConfig | None = None,
    ) -> None:
        self.config = config or DetectorConfig()
        self._references: list[_ReferenceVariant] = []
        for name, image in references.items():
            canonical = cv2.resize(
                _as_bgr(image),
                (self.config.image_size, self.config.image_size),
                interpolation=cv2.INTER_AREA,
            )
            for turns in range(4):
                rotated = np.ascontiguousarray(np.rot90(canonical, turns))
                normalized = normalize_lab(rotated, self.config.image_size)
                self._references.append(
                    _ReferenceVariant(
                        name=f"{name}@{turns * 90}",
                        image=rotated,
                        normalized=normalized,
                        gray=normalized[:, :, 0],
                    )
                )
        if not self._references:
            raise ValueError("at least one good reference image is required")

    @classmethod
    def from_paths(
        cls,
        paths: Sequence[str | Path],
        config: DetectorConfig | None = None,
    ) -> TileAnomalyDetector:
        references: dict[str, np.ndarray] = {}
        for value in paths:
            path = Path(value)
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"could not read reference image: {path}")
            references[path.name] = image
        return cls(references, config)

    def detect(self, sample: np.ndarray) -> TileAnomalyResult:
        try:
            sample = cv2.resize(
                _as_bgr(sample),
                (self.config.image_size, self.config.image_size),
                interpolation=cv2.INTER_AREA,
            )
        except ValueError as exc:
            return TileAnomalyResult(status="UNKNOWN", reason=str(exc))

        sample_gray = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
        if float(sample_gray.std()) < self.config.min_stddev:
            return TileAnomalyResult(
                status="UNKNOWN", reason="tile image has insufficient contrast"
            )

        normalized_sample = normalize_lab(sample, self.config.image_size)
        self_evidence = self._build_self_evidence(sample)
        self_mask, self_components = self._build_self_mask(self_evidence)
        self_area = int(cv2.countNonZero(self_mask))
        line_area = int(cv2.countNonZero(self_evidence["line"]))
        if (
            line_area >= self.config.strong_line_area
            or self_area >= self.config.strong_color_area
        ):
            return TileAnomalyResult(
                status="DEFECT",
                defect_area=self_area,
                defect_ratio=self_area / float(self_mask.size),
                mask=self_mask,
                normalized_sample=cv2.cvtColor(normalized_sample, cv2.COLOR_LAB2BGR),
                evidence=self_evidence,
                component_scores=self_components,
            )
        variant = self._select_reference(normalized_sample[:, :, 0])
        aligned, alignment_error = self._align_reference(
            variant.normalized, normalized_sample
        )
        if aligned is None or alignment_error > self.config.max_alignment_error:
            if self_area >= self.config.min_defect_area:
                return TileAnomalyResult(
                    status="DEFECT",
                    defect_area=self_area,
                    defect_ratio=self_area / float(self_mask.size),
                    reference_name=variant.name,
                    alignment_error=alignment_error,
                    mask=self_mask,
                    normalized_sample=cv2.cvtColor(normalized_sample, cv2.COLOR_LAB2BGR),
                    evidence=self_evidence,
                    component_scores=self_components,
                )
            return TileAnomalyResult(
                status="UNKNOWN",
                reference_name=variant.name,
                alignment_error=alignment_error,
                reason="reference alignment quality is inadequate",
                normalized_sample=normalized_sample,
            )

        evidence = self._build_evidence(normalized_sample, aligned)
        evidence.update(self_evidence)
        mask, components = self._build_mask(evidence)
        defect_area = int(cv2.countNonZero(mask))
        status: TileStatus = (
            "DEFECT" if defect_area >= self.config.min_defect_area else "OK"
        )
        return TileAnomalyResult(
            status=status,
            defect_area=defect_area,
            defect_ratio=defect_area / float(mask.size),
            reference_name=variant.name,
            alignment_error=alignment_error,
            mask=mask,
            aligned_reference=cv2.cvtColor(aligned, cv2.COLOR_LAB2BGR),
            normalized_sample=cv2.cvtColor(normalized_sample, cv2.COLOR_LAB2BGR),
            evidence=evidence,
            component_scores=components,
        )

    def _select_reference(self, sample_gray: np.ndarray) -> _ReferenceVariant:
        size = 96
        sample_small = cv2.resize(sample_gray, (size, size), interpolation=cv2.INTER_AREA)
        sample_small = _standardize(sample_small)
        best = self._references[0]
        best_error = float("inf")
        for variant in self._references:
            ref_small = cv2.resize(
                variant.gray, (size, size), interpolation=cv2.INTER_AREA
            )
            error = float(np.mean(np.abs(sample_small - _standardize(ref_small))))
            if error < best_error:
                best = variant
                best_error = error
        return best

    def _align_reference(
        self, reference_lab: np.ndarray, sample_lab: np.ndarray
    ) -> tuple[np.ndarray | None, float]:
        reference_gray = reference_lab[:, :, 0].astype(np.float32) / 255.0
        sample_gray = sample_lab[:, :, 0].astype(np.float32) / 255.0
        warp = np.eye(2, 3, dtype=np.float32)
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self.config.ecc_iterations,
            self.config.ecc_epsilon,
        )
        try:
            cv2.findTransformECC(
                sample_gray,
                reference_gray,
                warp,
                cv2.MOTION_EUCLIDEAN,
                criteria,
                np.ones(sample_gray.shape, dtype=np.uint8),
                3,
            )
        except cv2.error:
            try:
                shift, _ = cv2.phaseCorrelate(reference_gray, sample_gray)
            except cv2.error:
                return None, float("inf")
            warp = np.array([[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]], np.float32)

        aligned = cv2.warpAffine(
            reference_lab,
            warp,
            (self.config.image_size, self.config.image_size),
            flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_REFLECT_101,
        )
        aligned_gray = aligned[:, :, 0].astype(np.float32)
        error = float(
            np.mean(
                np.abs(
                    _standardize(aligned_gray) - _standardize(sample_lab[:, :, 0])
                )
            )
        )
        return aligned, error

    def _build_self_evidence(self, sample: np.ndarray) -> dict[str, np.ndarray]:
        sample_lab = cv2.cvtColor(sample, cv2.COLOR_BGR2LAB)
        chroma = sample_lab[:, :, 1:].astype(np.float32)
        median = np.median(chroma.reshape(-1, 2), axis=0)
        color_distance = np.linalg.norm(chroma - median, axis=2)
        color = np.where(
            color_distance >= self.config.self_color_threshold, 255, 0
        ).astype(np.uint8)
        color = cv2.morphologyEx(
            color,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        lines = _dark_line_segments(
            cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY),
            self.config.min_segment_length,
            self.config.dark_segment_score,
        )
        return {"self_color": color, "line": lines}

    def _build_self_mask(
        self, evidence: dict[str, np.ndarray]
    ) -> tuple[np.ndarray, list[dict[str, float | int]]]:
        output = evidence["line"].copy()
        components: list[dict[str, float | int]] = []
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            evidence["self_color"]
        )
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.config.self_color_min_area:
                continue
            region = labels == label
            output[region] = 255
            components.append(
                {
                    "area": area,
                    "contrast": 255.0,
                    "x": int(stats[label, cv2.CC_STAT_LEFT]),
                    "y": int(stats[label, cv2.CC_STAT_TOP]),
                    "width": int(stats[label, cv2.CC_STAT_WIDTH]),
                    "height": int(stats[label, cv2.CC_STAT_HEIGHT]),
                }
            )
        output[:8, :] = 0
        output[-8:, :] = 0
        output[:, :8] = 0
        output[:, -8:] = 0
        return output, components

    def _build_evidence(
        self, sample_lab: np.ndarray, reference_lab: np.ndarray
    ) -> dict[str, np.ndarray]:
        sample_l = sample_lab[:, :, 0]
        reference_l = reference_lab[:, :, 0]

        color = cv2.magnitude(
            cv2.absdiff(sample_lab[:, :, 1], reference_lab[:, :, 1]).astype(np.float32),
            cv2.absdiff(sample_lab[:, :, 2], reference_lab[:, :, 2]).astype(np.float32),
        )
        intensity = cv2.absdiff(sample_l, reference_l)

        sample_gradient = _gradient_magnitude(sample_l)
        reference_gradient = _gradient_magnitude(reference_l)
        gradient = cv2.absdiff(sample_gradient, reference_gradient)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))
        sample_dark = cv2.morphologyEx(sample_l, cv2.MORPH_BLACKHAT, kernel)
        reference_dark = cv2.morphologyEx(reference_l, cv2.MORPH_BLACKHAT, kernel)
        dark_line = cv2.subtract(sample_dark, reference_dark)

        return {
            "color": np.clip(color, 0, 255).astype(np.uint8),
            "intensity": intensity,
            "gradient": gradient,
            "dark_line": dark_line,
        }

    def _build_mask(
        self, evidence: dict[str, np.ndarray]
    ) -> tuple[np.ndarray, list[dict[str, float | int]]]:
        color = evidence["color"] >= self.config.color_threshold
        intensity = evidence["intensity"] >= self.config.intensity_threshold
        gradient = evidence["gradient"] >= self.config.gradient_threshold
        dark_line = evidence["dark_line"] >= self.config.dark_line_threshold

        votes = (
            color.astype(np.uint8)
            + intensity.astype(np.uint8)
            + gradient.astype(np.uint8)
            + dark_line.astype(np.uint8)
        )
        candidate = np.where((votes >= 2) | color | dark_line, 255, 0).astype(np.uint8)
        candidate[:8, :] = 0
        candidate[-8:, :] = 0
        candidate[:, :8] = 0
        candidate[:, -8:] = 0
        candidate = cv2.morphologyEx(
            candidate,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        candidate = cv2.morphologyEx(
            candidate,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )

        count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate)
        output = np.zeros_like(candidate)
        components: list[dict[str, float | int]] = []
        residual_names = ("color", "intensity", "gradient", "dark_line")
        combined = np.maximum.reduce([evidence[name] for name in residual_names])
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.config.min_component_area:
                continue
            region = labels == label
            contrast = float(combined[region].mean())
            if contrast < self.config.min_component_contrast:
                continue
            output[region] = 255
            components.append(
                {
                    "area": area,
                    "contrast": contrast,
                    "x": int(stats[label, cv2.CC_STAT_LEFT]),
                    "y": int(stats[label, cv2.CC_STAT_TOP]),
                    "width": int(stats[label, cv2.CC_STAT_WIDTH]),
                    "height": int(stats[label, cv2.CC_STAT_HEIGHT]),
                }
            )
        return output, components


def _as_bgr(image: np.ndarray) -> np.ndarray:
    if image is None or image.size == 0:
        raise ValueError("tile image is empty")
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim != 3:
        raise ValueError(f"unsupported tile image shape: {image.shape}")
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if image.shape[2] != 3:
        raise ValueError(f"unsupported tile image shape: {image.shape}")
    return np.ascontiguousarray(image)


def _standardize(image: np.ndarray) -> np.ndarray:
    image = image.astype(np.float32)
    return (image - float(image.mean())) / max(float(image.std()), 1.0)


def _gradient_magnitude(image: np.ndarray) -> np.ndarray:
    x_gradient = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    y_gradient = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    return np.clip(cv2.magnitude(x_gradient, y_gradient), 0, 255).astype(np.uint8)


def _dark_line_segments(
    image: np.ndarray, min_length: float, min_score: float
) -> np.ndarray:
    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    blurred = cv2.GaussianBlur(image, (3, 3), 0)
    detected = detector.detect(blurred)[0]
    mask = np.zeros_like(image)
    if detected is None:
        return mask

    height, width = image.shape
    for line in detected[:, 0]:
        x1, y1, x2, y2 = (float(value) for value in line)
        length = float(np.hypot(x2 - x1, y2 - y1))
        if length < min_length:
            continue
        dx = (x2 - x1) / length
        dy = (y2 - y1) / length
        normal_x, normal_y = -dy, dx
        positions = np.linspace(0.0, 1.0, max(10, int(length)))
        x_positions = x1 + (x2 - x1) * positions
        y_positions = y1 + (y2 - y1) * positions

        def sample(
            offset: float,
            x_values: np.ndarray = x_positions,
            y_values: np.ndarray = y_positions,
            nx: float = normal_x,
            ny: float = normal_y,
        ) -> float:
            xs = np.clip(
                np.rint(x_values + nx * offset).astype(np.int32),
                0,
                width - 1,
            )
            ys = np.clip(
                np.rint(y_values + ny * offset).astype(np.int32),
                0,
                height - 1,
            )
            return float(image[ys, xs].mean())

        center = sample(0.0)
        surroundings = 0.5 * (sample(4.0) + sample(-4.0))
        darkness = max(0.0, surroundings - center)
        if length * darkness < min_score:
            continue
        cv2.line(
            mask,
            (int(round(x1)), int(round(y1))),
            (int(round(x2)), int(round(y2))),
            (255,),
            5,
            cv2.LINE_AA,
        )
    return mask
