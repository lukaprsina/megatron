from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))

from scripts.promote_good_tile_references import promote_world  # noqa: E402


def _capture(
    world: Path, station: str, tile_index: int, stamp: int, modified_ns: int
) -> Path:
    station_dir = world / station
    captures_dir = station_dir / "captures"
    canonical_dir = station_dir / "canonical"
    captures_dir.mkdir(parents=True, exist_ok=True)
    canonical_dir.mkdir(parents=True, exist_ok=True)
    canonical = canonical_dir / f"tile-{tile_index:02d}-{stamp}.png"
    canonical.write_bytes(f"image-{stamp}".encode())
    metadata = captures_dir / f"tile-{tile_index:02d}-{stamp}.yaml"
    metadata.write_text(
        yaml.safe_dump(
            {
                "tile_index": tile_index,
                "stamp_ns": stamp,
                "canonical_image": f"canonical/{canonical.name}",
            }
        )
    )
    os.utime(metadata, ns=(modified_ns, modified_ns))
    return canonical


def test_promotes_only_latest_label_confirmed_good_capture(tmp_path: Path) -> None:
    world = tmp_path / "task2"
    _capture(world, "green", 2, stamp=900, modified_ns=1_000)
    latest = _capture(world, "green", 2, stamp=100, modified_ns=2_000)
    _capture(world, "green", 3, stamp=200, modified_ns=3_000)
    (world / "labels.yaml").write_text(
        yaml.safe_dump({"green": {2: "good", 3: "damaged"}})
    )

    references = tmp_path / "references"
    copied, duplicates = promote_world(world, references)

    destination = references / "task2" / "green" / "good_02_100.png"
    assert copied == [destination]
    assert duplicates == []
    assert destination.read_bytes() == latest.read_bytes()


def test_skips_reference_with_identical_content(tmp_path: Path) -> None:
    world = tmp_path / "task2"
    source = _capture(world, "red", 1, stamp=123, modified_ns=1_000)
    (world / "labels.yaml").write_text(yaml.safe_dump({"red": {1: "good"}}))
    references = tmp_path / "references"
    existing = references / "task2" / "red" / "good_existing.png"
    existing.parent.mkdir(parents=True)
    existing.write_bytes(source.read_bytes())

    copied, duplicates = promote_world(world, references)

    assert copied == []
    assert duplicates == [source]
