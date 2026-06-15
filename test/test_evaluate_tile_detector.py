from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))

from scripts.evaluate_tile_detector import load_captures  # noqa: E402


def _capture(station: Path, name: str, stamp_ns: int, mtime_ns: int) -> Path:
    captures = station / "captures"
    canonical = station / "canonical"
    captures.mkdir(parents=True, exist_ok=True)
    canonical.mkdir(exist_ok=True)
    image = canonical / f"{name}.png"
    image.touch()
    metadata = captures / f"{name}.yaml"
    metadata.write_text(
        yaml.safe_dump(
            {
                "tile_index": 0,
                "stamp_ns": stamp_ns,
                "canonical_image": f"canonical/{name}.png",
            }
        )
    )
    os.utime(metadata, ns=(mtime_ns, mtime_ns))
    return image


def test_latest_capture_uses_file_time_not_reset_simulation_time(
    tmp_path: Path,
) -> None:
    station = tmp_path / "red"
    _capture(station, "tile-00-900000000000", 900_000_000_000, 1_000)
    latest = _capture(station, "tile-00-100000000000", 100_000_000_000, 2_000)

    captures = load_captures(tmp_path, latest_only=True)

    assert captures == [(latest, "unknown")]
