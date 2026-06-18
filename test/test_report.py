from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from megatron.report import AnomalyTask, ReportBuilder  # noqa: E402


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)


def test_anomaly_section_falls_back_to_capture_artifacts(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.chdir(tmp_path)
    station_dir = tmp_path / "artifacts" / "tile_captures" / "task2" / "green"
    captures_dir = station_dir / "captures"
    canonical_dir = station_dir / "canonical"
    masks_dir = station_dir / "masks"
    captures_dir.mkdir(parents=True)
    canonical_dir.mkdir()
    masks_dir.mkdir()

    image = np.full((32, 32, 3), 180, dtype=np.uint8)
    mask = np.zeros((32, 32), dtype=np.uint8)
    mask[8:16, 8:16] = 255
    cv2.imwrite(str(canonical_dir / "tile-02-123.png"), image)
    cv2.imwrite(str(masks_dir / "tile-02-123.png"), mask)
    (captures_dir / "tile-02-123.yaml").write_text(
        yaml.safe_dump(
            {
                "world": "task2",
                "station": "green",
                "tile_index": 2,
                "canonical_image": "canonical/tile-02-123.png",
                "anomaly": {"mask_image": "masks/tile-02-123.png"},
            },
            sort_keys=False,
        )
    )

    logger = _Logger()
    html = ReportBuilder(logger).build(
        [
            AnomalyTask(
                requestor="cto",
                color="green",
                results=[{"tile_id": 2, "station": "green", "status": "DEFECT"}],
            )
        ]
    )

    assert "ID: 2" in html
    assert "canonical/tile-02-123.png" in html
    assert "masks/tile-02-123.png" in html
    assert logger.messages == []
