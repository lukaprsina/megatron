from __future__ import annotations

import hashlib
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from megatron.tile_anomaly import (  # noqa: E402
    DetectorConfig,
    TileAnomalyDetector,
    order_quad,
    warp_tile,
)

CONFIG_PATH = PACKAGE_ROOT / "assets" / "tiles" / "model.yaml"


def _unique_textures(prefix: str) -> list[Path]:
    paths: list[Path] = []
    hashes: set[str] = set()
    for path in sorted((PACKAGE_ROOT / "worlds").glob(f"task2*_meshes/{prefix}*.png")):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in hashes:
            continue
        hashes.add(digest)
        paths.append(path)
    return paths


def _detector() -> TileAnomalyDetector:
    return TileAnomalyDetector.from_paths(
        _unique_textures("good"), DetectorConfig.from_yaml(CONFIG_PATH)
    )


def test_order_quad_is_stable_for_shuffled_points() -> None:
    expected = np.array([[10, 20], [90, 15], [95, 80], [5, 85]], np.float32)
    shuffled = expected[[2, 0, 3, 1]]
    assert np.allclose(order_quad(shuffled), expected)


def test_warp_tile_extracts_complete_inset_region() -> None:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 20), (130, 100), (40, 120, 220), -1)
    quad = np.array([[30, 20], [130, 20], [130, 100], [30, 100]], np.float32)
    warped = warp_tile(image, quad, size=96, border_inset=0.06)
    assert warped.shape == (96, 96, 3)
    assert np.allclose(warped[48, 48], (40, 120, 220), atol=2)
    assert float((warped[:, :, 2] > 200).mean()) > 0.98


def test_all_unique_supplied_textures_are_classified() -> None:
    detector = _detector()
    good = _unique_textures("good")
    damaged = _unique_textures("damaged")
    assert len(good) == 4
    assert len(damaged) == 15
    assert all(detector.detect(cv2.imread(str(path))).status == "OK" for path in good)
    assert all(
        detector.detect(cv2.imread(str(path))).status == "DEFECT"
        for path in damaged
    )


def test_good_textures_tolerate_basic_camera_variation() -> None:
    detector = _detector()
    for path in _unique_textures("good"):
        image = cv2.imread(str(path))
        variants = [
            cv2.convertScaleAbs(image, alpha=1.0, beta=30),
            cv2.convertScaleAbs(image, alpha=1.18, beta=-18),
            cv2.GaussianBlur(image, (5, 5), 1.2),
            np.ascontiguousarray(np.rot90(image)),
        ]
        assert all(detector.detect(variant).status == "OK" for variant in variants)


def test_registered_residual_detects_synthetic_gray_defect() -> None:
    references = _unique_textures("good")
    detector = _detector()
    image = cv2.imread(str(references[0]))
    cv2.line(image, (180, 100), (325, 350), (35, 35, 35), 4, cv2.LINE_AA)
    result = detector.detect(image)
    assert result.status == "DEFECT"
    assert result.mask is not None
    assert cv2.countNonZero(result.mask) >= 24


def test_green_cross_from_workstation_screenshot_is_localized() -> None:
    detector = _detector()
    screenshot = cv2.imread(str(REPO_ROOT / "image.png"))
    quad = np.array([[275, 244], [462, 247], [469, 383], [258, 383]], np.float32)
    tile = warp_tile(screenshot, quad)
    result = detector.detect(tile)
    assert result.status == "DEFECT"
    assert result.mask is not None
    # The cross appears in the upper-right quadrant of the canonical warp.
    assert cv2.countNonZero(result.mask[:256, 256:]) > 100
    assert result.defect_ratio < 0.08


def test_low_information_image_is_unknown() -> None:
    result = _detector().detect(np.full((512, 512, 3), 128, dtype=np.uint8))
    assert result.status == "UNKNOWN"
    assert result.reason == "tile image has insufficient contrast"


def test_cpu_runtime_is_below_100_ms_on_average() -> None:
    detector = _detector()
    samples = [cv2.imread(str(path)) for path in _unique_textures("good")]
    detector.detect(samples[0])
    started = time.perf_counter()
    for sample in samples:
        detector.detect(sample)
    mean_ms = (time.perf_counter() - started) * 1000.0 / len(samples)
    assert mean_ms < 100.0
