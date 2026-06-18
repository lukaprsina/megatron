from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from megatron.tile_anomaly import (
    TileAnomalyResult,
    anomaly_panel_canvas,
    binary_damage_mask,
    warp_tile,
)


def save_tile_capture(
    output_dir: str | Path,
    frame: np.ndarray,
    quad: np.ndarray,
    *,
    world: str,
    station: str,
    tile_index: int,
    stamp_ns: int,
    pose: dict[str, float] | None = None,
    anomaly_result: TileAnomalyResult | None = None,
    megatron_root: str | Path | None = None,
) -> Path:
    """Save one camera-domain tile sample and return its metadata path.

    Directory layout (paths relative to station_dir)::

        <station_dir>/
          captures/tile-NN-<stamp>.yaml   ← immutable provenance, no label
          canonical/tile-NN-<stamp>.png
          panels/tile-NN-<stamp>.jpg       ← optional detector visualization
          masks/tile-NN-<stamp>.png        ← optional detector binary mask
          report/tile-NN-warped.png        ← stable copy for report use
          report/tile-NN-mask.png
          raw/tile-NN-<stamp>.png

    Labels live in a separate ``labels.yaml`` at the world level, keyed by
    station and tile_index, so capture files are never modified after writing.
    """
    if frame is None or frame.size == 0:
        raise ValueError("frame is empty")

    quad = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    station_dir = Path(output_dir).expanduser() / _safe_name(world) / _safe_name(station)
    captures_dir = station_dir / "captures"
    captures_dir.mkdir(parents=True, exist_ok=True)

    stem = f"tile-{tile_index:02d}-{stamp_ns}"
    raw_path = station_dir / "raw" / f"{stem}.png"
    warp_path = station_dir / "canonical" / f"{stem}.png"
    metadata_path = captures_dir / f"{stem}.yaml"
    raw_path.parent.mkdir(exist_ok=True)
    warp_path.parent.mkdir(exist_ok=True)

    canonical = warp_tile(frame, quad)
    if not cv2.imwrite(str(raw_path), frame):
        raise OSError(f"could not write capture image: {raw_path}")
    if not cv2.imwrite(str(warp_path), canonical):
        raw_path.unlink(missing_ok=True)
        raise OSError(f"could not write tile warp: {warp_path}")

    gray = cv2.cvtColor(canonical, cv2.COLOR_BGR2GRAY)
    # Paths are relative to station_dir (parent of captures/).
    metadata: dict[str, Any] = {
        "world": world,
        "station": station,
        "tile_index": tile_index,
        "stamp_ns": stamp_ns,
        "raw_image": f"raw/{stem}.png",
        "canonical_image": f"canonical/{stem}.png",
        "quad_xy": [[float(x), float(y)] for x, y in quad],
        "quality": {
            "brightness_mean": float(gray.mean()),
            "contrast_stddev": float(gray.std()),
            "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        },
    }
    if pose is not None:
        metadata["robot_pose"] = {key: float(value) for key, value in pose.items()}

    report_stem = f"{_safe_name(world)}-{_safe_name(station)}-tile-{tile_index:02d}"
    report_canonical_path = station_dir / "report" / f"tile-{tile_index:02d}-warped.png"
    report_mask_path = station_dir / "report" / f"tile-{tile_index:02d}-mask.png"

    if anomaly_result is not None:
        analysis = _save_anomaly_artifacts(
            station_dir,
            stem,
            canonical,
            anomaly_result,
            report_canonical_path,
            report_mask_path,
            _megatron_report_dir(megatron_root),
            report_stem,
        )
        metadata["anomaly"] = {
            "status": anomaly_result.status,
            "defect_area": int(anomaly_result.defect_area),
            "defect_ratio": float(anomaly_result.defect_ratio),
            "score": anomaly_result.anomaly_score,
            "threshold": anomaly_result.anomaly_threshold,
            "reference": anomaly_result.reference_name,
            "reason": anomaly_result.reason,
            **analysis,
        }

    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    return metadata_path


def _save_anomaly_artifacts(
    station_dir: Path,
    stem: str,
    canonical: np.ndarray,
    result: TileAnomalyResult,
    report_canonical_path: Path,
    report_mask_path: Path,
    megatron_report_dir: Path,
    report_stem: str,
) -> dict[str, str]:
    masks_dir = station_dir / "masks"
    panels_dir = station_dir / "panels"
    masks_dir.mkdir(exist_ok=True)
    panels_dir.mkdir(exist_ok=True)

    mask_path = masks_dir / f"{stem}.png"
    panel_path = panels_dir / f"{stem}.jpg"
    threshold = result.anomaly_threshold if result.anomaly_threshold is not None else 1.0
    anomaly_map = result.anomaly_map if result.anomaly_map is not None else result.mask

    if anomaly_map is None:
        mask = np.zeros(canonical.shape[:2], dtype=np.uint8)
        panel_map = mask.astype(np.float32)
    else:
        mask = binary_damage_mask(anomaly_map, canonical.shape[:2], threshold)
        panel_map = anomaly_map

    megatron_canonical_path = megatron_report_dir / f"{report_stem}-warped.png"
    megatron_mask_path = megatron_report_dir / f"{report_stem}-mask.png"

    if not cv2.imwrite(str(mask_path), mask):
        raise OSError(f"could not write anomaly mask: {mask_path}")
    panel = anomaly_panel_canvas(canonical, panel_map, threshold)
    if not cv2.imwrite(str(panel_path), panel):
        mask_path.unlink(missing_ok=True)
        raise OSError(f"could not write anomaly panel: {panel_path}")
    for path, image in (
        (report_canonical_path, canonical),
        (report_mask_path, mask),
        (megatron_canonical_path, canonical),
        (megatron_mask_path, mask),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            raise OSError(f"could not write report tile image: {path}")

    return {
        "mask_image": f"masks/{stem}.png",
        "panel_image": f"panels/{stem}.jpg",
        "report_warped_image": f"report/tile-{_tile_number_from_stem(stem)}-warped.png",
        "report_mask_image": f"report/tile-{_tile_number_from_stem(stem)}-mask.png",
        "megatron_report_warped_image": str(megatron_canonical_path),
        "megatron_report_mask_image": str(megatron_mask_path),
    }


def _megatron_report_dir(megatron_root: str | Path | None) -> Path:
    if megatron_root is not None:
        root = Path(megatron_root).expanduser()
    else:
        try:
            root = Path(get_package_share_directory("megatron"))
        except PackageNotFoundError:
            root = Path(__file__).resolve().parents[1]
    return root / "assets" / "report_tiles"


def _tile_number_from_stem(stem: str) -> str:
    match = re.match(r"tile-(\d+)-", stem)
    return match.group(1) if match else "unknown"


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip()).strip("-.")
    return cleaned or "unknown"
