from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from megatron.tile_anomaly import warp_tile


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
) -> Path:
    """Save one camera-domain tile sample and return its metadata path.

    Directory layout (paths relative to station_dir)::

        <station_dir>/
          captures/tile-NN-<stamp>.yaml   ← immutable provenance, no label
          canonical/tile-NN-<stamp>.png
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

    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    return metadata_path


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip()).strip("-.")
    return cleaned or "unknown"
