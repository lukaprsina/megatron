from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from megatron.tile_anomaly import TileAnomalyResult  # noqa: E402
from megatron.tile_capture import save_tile_capture  # noqa: E402


def test_save_tile_capture_writes_images_and_metadata(tmp_path: Path) -> None:
    frame = np.zeros((120, 180, 3), dtype=np.uint8)
    cv2.rectangle(frame, (35, 20), (145, 105), (30, 140, 220), -1)
    cv2.line(frame, (50, 35), (130, 90), (70, 70, 70), 4)
    quad = np.array([[35, 20], [145, 20], [145, 105], [35, 105]], np.float32)

    metadata_path = save_tile_capture(
        tmp_path,
        frame,
        quad,
        world="task2 demo",
        station="red",
        tile_index=2,
        stamp_ns=123456,
        pose={"x": 1.2, "y": -0.4, "yaw": 0.3},
    )

    station_dir = tmp_path / "task2-demo" / "red"
    assert metadata_path.parent == station_dir / "captures"
    metadata = yaml.safe_load(metadata_path.read_text())
    raw = cv2.imread(str(station_dir / metadata["raw_image"]))
    canonical = cv2.imread(str(station_dir / metadata["canonical_image"]))
    assert raw.shape == frame.shape
    assert canonical.shape == (512, 512, 3)
    assert metadata["raw_image"].startswith("raw/")
    assert metadata["canonical_image"].startswith("canonical/")
    assert metadata["quad_xy"] == quad.tolist()
    assert metadata["robot_pose"] == {"x": 1.2, "y": -0.4, "yaw": 0.3}
    assert metadata["quality"]["contrast_stddev"] > 0.0


def test_save_tile_capture_writes_anomaly_panel_and_mask(tmp_path: Path) -> None:
    frame = np.zeros((120, 180, 3), dtype=np.uint8)
    cv2.rectangle(frame, (35, 20), (145, 105), (30, 140, 220), -1)
    quad = np.array([[35, 20], [145, 20], [145, 105], [35, 105]], np.float32)
    anomaly_map = np.zeros((16, 16), dtype=np.float32)
    anomaly_map[5:8, 6:9] = 2.0
    result = TileAnomalyResult(
        status="DEFECT",
        defect_area=9,
        defect_ratio=9.0 / float(anomaly_map.size),
        anomaly_score=2.0,
        anomaly_threshold=1.0,
        anomaly_map=anomaly_map,
    )

    metadata_path = save_tile_capture(
        tmp_path,
        frame,
        quad,
        world="task2",
        station="red",
        tile_index=1,
        stamp_ns=42,
        anomaly_result=result,
        megatron_root=tmp_path / "megatron_root",
    )

    station_dir = tmp_path / "task2" / "red"
    metadata = yaml.safe_load(metadata_path.read_text())
    mask = cv2.imread(str(station_dir / metadata["anomaly"]["mask_image"]), cv2.IMREAD_GRAYSCALE)
    panel = cv2.imread(str(station_dir / metadata["anomaly"]["panel_image"]))
    report_warped = cv2.imread(str(station_dir / metadata["anomaly"]["report_warped_image"]))
    report_mask = cv2.imread(
        str(station_dir / metadata["anomaly"]["report_mask_image"]),
        cv2.IMREAD_GRAYSCALE,
    )
    megatron_warped = Path(metadata["anomaly"]["megatron_report_warped_image"])
    megatron_mask = Path(metadata["anomaly"]["megatron_report_mask_image"])
    assert metadata["anomaly"]["status"] == "DEFECT"
    assert metadata["anomaly"]["threshold"] == 1.0
    assert cv2.countNonZero(mask) > 0
    assert panel.shape == (512, 2048, 3)
    assert report_warped.shape == (512, 512, 3)
    assert cv2.countNonZero(report_mask) > 0
    assert megatron_warped.is_file()
    assert megatron_mask.is_file()


def test_save_tile_capture_rejects_empty_frame(tmp_path: Path) -> None:
    with np.testing.assert_raises_regex(ValueError, "frame is empty"):
        save_tile_capture(
            tmp_path,
            np.empty((0, 0, 3), dtype=np.uint8),
            np.zeros((4, 2), dtype=np.float32),
            world="task2",
            station="green",
            tile_index=0,
            stamp_ns=0,
        )
