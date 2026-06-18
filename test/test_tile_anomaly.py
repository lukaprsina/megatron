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
    _dark_blobs,
    _lateral_border_intrusions,
    anomaly_panel_canvas,
    binary_damage_mask,
    order_quad,
    quad_from_contour,
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


def test_quad_from_contour_recovers_perspective_trapezoid() -> None:
    # Simulates a tile viewed at an angle: a trapezoid, not a rectangle.
    trapezoid = np.array([[40, 20], [160, 35], [150, 140], [50, 130]], np.float32)
    mask = np.zeros((180, 200), dtype=np.uint8)
    cv2.fillConvexPoly(mask, trapezoid.astype(np.int32), 255)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)

    quad = quad_from_contour(contour)
    assert quad is not None
    assert np.allclose(quad, order_quad(trapezoid), atol=2.0)

    rect_box = cv2.boxPoints(cv2.minAreaRect(contour))
    assert not np.allclose(quad, order_quad(rect_box), atol=2.0)


def test_quad_from_contour_falls_back_when_no_quad() -> None:
    # A triangle never reduces to 4 vertices at any approxPolyDP epsilon.
    triangle = np.array([[60, 10], [110, 100], [10, 100]], np.int32)
    mask = np.zeros((120, 120), dtype=np.uint8)
    cv2.fillConvexPoly(mask, triangle, 255)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    assert quad_from_contour(contour) is None


def test_warp_tile_extracts_complete_inset_region() -> None:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 20), (130, 100), (40, 120, 220), -1)
    quad = np.array([[30, 20], [130, 20], [130, 100], [30, 100]], np.float32)
    warped = warp_tile(image, quad, size=96, border_inset=0.06)
    assert warped.shape == (96, 96, 3)
    assert np.allclose(warped[48, 48], (40, 120, 220), atol=2)
    assert float((warped[:, :, 2] > 200).mean()) > 0.98


def test_binary_damage_mask_uses_raw_threshold() -> None:
    anomaly_map = np.array([[0.1, 1.1], [2.0, 0.2]], dtype=np.float32)
    mask = binary_damage_mask(anomaly_map, (2, 2), threshold=1.0)
    assert mask.tolist() == [[0, 255], [255, 0]]


def test_anomaly_panel_canvas_matches_patchcore_layout() -> None:
    image = np.full((32, 32, 3), 120, dtype=np.uint8)
    anomaly_map = np.zeros((8, 8), dtype=np.float32)
    anomaly_map[2:4, 3:5] = 2.0
    canvas = anomaly_panel_canvas(image, anomaly_map, threshold=1.0)
    assert canvas.shape == (32, 128, 3)
    assert cv2.countNonZero(cv2.cvtColor(canvas[:, 96:], cv2.COLOR_BGR2GRAY)) > 0


def test_all_unique_supplied_textures_are_classified() -> None:
    detector = _detector()
    good = _unique_textures("good")
    damaged = _unique_textures("damaged")
    assert len(good) == 4
    assert len(damaged) == 15
    assert all(detector.detect(cv2.imread(str(path))).status == "OK" for path in good)
    assert all(
        detector.detect(cv2.imread(str(path))).status == "DEFECT" for path in damaged
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


def test_task2_green_good_tile_02_does_not_require_ecc_alignment() -> None:
    capture = (
        REPO_ROOT
        / "artifacts"
        / "tile_captures"
        / "task2"
        / "green"
        / "canonical"
        / "tile-02-156981000000.png"
    )
    if not capture.exists():
        return
    references = sorted(
        (PACKAGE_ROOT / "assets" / "tiles" / "reference_good" / "task2" / "green").glob("*.png")
    )
    detector = TileAnomalyDetector.from_paths(
        references,
        DetectorConfig.from_yaml(CONFIG_PATH),
    )
    result = detector.detect(cv2.imread(str(capture)))
    assert result.status == "OK"
    assert result.reason is None
    assert result.alignment_error is None


def test_low_information_image_is_unknown() -> None:
    result = _detector().detect(np.full((512, 512, 3), 128, dtype=np.uint8))
    assert result.status == "UNKNOWN"
    assert result.reason == "tile image has insufficient contrast"


def test_narrow_conveyor_rail_is_not_a_side_intrusion() -> None:
    gray = np.full((512, 512), 180, dtype=np.uint8)
    gray[:, -24:] = 20
    mask = _lateral_border_intrusions(
        gray, min_area=1800, min_width_ratio=0.12, min_height_ratio=0.25
    )
    assert cv2.countNonZero(mask) == 0


def test_wide_side_separation_is_a_side_intrusion() -> None:
    gray = np.full((512, 512), 180, dtype=np.uint8)
    polygon = np.array([[512, 100], [512, 410], [420, 330], [435, 170]])
    cv2.fillConvexPoly(gray, polygon, (20,))
    mask = _lateral_border_intrusions(
        gray, min_area=1800, min_width_ratio=0.12, min_height_ratio=0.25
    )
    assert cv2.countNonZero(mask) >= 1800


def test_short_diagonal_side_shadow_is_not_a_side_intrusion() -> None:
    gray = np.full((512, 512), 180, dtype=np.uint8)
    polygon = np.array([[0, 135], [112, 235], [88, 235], [0, 155]])
    cv2.fillConvexPoly(gray, polygon, (35,))
    mask = _lateral_border_intrusions(
        gray, min_area=1800, min_width_ratio=0.12, min_height_ratio=0.25
    )
    assert cv2.countNonZero(mask) == 0


def test_task2_red_good_tile_01_is_not_a_side_intrusion_false_positive() -> None:
    capture = (
        REPO_ROOT
        / "artifacts"
        / "tile_captures"
        / "task2"
        / "red"
        / "canonical"
        / "tile-01-159456000000.png"
    )
    if not capture.exists():
        return
    references = sorted(
        (PACKAGE_ROOT / "assets" / "tiles" / "reference_good" / "task2" / "red").glob("*.png")
    )
    detector = TileAnomalyDetector.from_paths(
        references,
        DetectorConfig.from_yaml(CONFIG_PATH),
    )
    result = detector.detect(cv2.imread(str(capture)))
    assert result.status == "OK"


def test_dark_crack_endpoint_at_lower_edge_is_retained() -> None:
    gray = np.full((512, 512), 180, dtype=np.uint8)
    cv2.line(gray, (390, 180), (390, 511), 30, 8, cv2.LINE_AA)
    cv2.ellipse(gray, (390, 490), (55, 25), 0, 0, 360, 25, -1)
    mask = _dark_blobs(gray, threshold=45, min_area=200, margin=12)
    assert cv2.countNonZero(mask) >= 200


def test_small_camera_domain_dark_crack_is_retained() -> None:
    gray = np.full((512, 512), 180, dtype=np.uint8)
    cv2.line(gray, (340, 100), (340, 145), 20, 9, cv2.LINE_AA)
    mask = _dark_blobs(gray, threshold=45, min_area=120, margin=12)
    assert cv2.countNonZero(mask) >= 120


def test_cpu_runtime_is_below_100_ms_on_average() -> None:
    detector = _detector()
    samples = [cv2.imread(str(path)) for path in _unique_textures("good")]
    detector.detect(samples[0])
    started = time.perf_counter()
    for sample in samples:
        detector.detect(sample)
    mean_ms = (time.perf_counter() - started) * 1000.0 / len(samples)
    assert mean_ms < 100.0
