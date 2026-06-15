# 003 — Classical tile anomaly detection (supersedes SSIM)

**Status**: Superseded. The original SSIM decision was replaced by a multi-channel
classical detector after camera-domain evaluation showed SSIM against raw mesh textures
scores 0.20–0.25 regardless of tile condition (pipeline/reference domain mismatch).

---

## Decision

Use `TileAnomalyDetector` in `src/megatron/megatron/tile_anomaly.py`, a purely
classical OpenCV/NumPy pipeline. Classifies each tile as `OK`, `DEFECT`, or
`UNKNOWN`. References are camera-domain captures of known-good tiles, not raw mesh
textures.

## Rationale

- SSIM against mesh textures was unusable (all tiles scored ~0.20–0.25).
- Camera-domain captures as references let ECC alignment work correctly.
- A self-evidence fallback (chroma-outlier, dark-line, blob detection) allows
  classifying obvious defects even when reference alignment fails.
- `UNKNOWN` is an explicit outcome when neither reference alignment nor self-evidence
  is reliable — preferable to a wrong classification.
- Mean runtime: ~55–65 ms per tile in the supplied worlds (acceptable inline).

## Pipeline

1. Quad ordering + 6 % inset perspective warp → 512×512 BGR.
2. Lab normalization (CLAHE on L channel) for illumination robustness.
3. Best-aligned reference selected by 96×96 downsampled MAE across all
   rotation variants (0/90/180/270°).
4. ECC alignment (Euclidean) with phase-correlation fallback.
5. If alignment error ≤ 0.34: aligned color / intensity / gradient / dark-hat
   residuals → thresholded mask → `OK` or `DEFECT`.
6. If alignment fails: self-evidence channels (chroma-outlier, dark-line, dark-blob,
   border-intrusion) → `DEFECT` if strong evidence, else `UNKNOWN`.

## Reference bank

Camera-domain good-tile captures stored under
`src/megatron/assets/tiles/reference_good/<world>/<belt>/good_*.png`.
World- and belt-scoped at runtime (`world_name` parameter → controller loads only
`reference_good/<world_name>/<belt>/` references).

## Measured results (offline, `--latest-only`)

| World | Correct | Accuracy | UNKNOWN |
|---|---|---|---|
| task2 | 9/9 | 100 % | 0 |
| task2_blue_demo | 9/9 | 100 % | 0 |
| task2_green_demo | 8/9 | 88.9 % | 1 (damaged tile, belt-edge clipping) |
| task2_yellow_demo | 9/9 | 100 % | 0 |

Live run (task2, red belt): tiles 0/2 correctly DEFECT, tile 1 (good) false-positive
DEFECT, tile 3 UNKNOWN. False positive on the good tile is a known sensitivity issue
when the live camera angle differs from the reference capture angle.

## Configuration

Thresholds in `src/megatron/assets/tiles/model.yaml`. Key parameters:
`max_alignment_error=0.34`, `min_defect_area=24`, `ecc_iterations=35`.

## Code

- Detector: `src/megatron/megatron/tile_anomaly.py`
- Controller integration: `task2_controller.py:_score_tile()`
- Offline evaluator: `scripts/evaluate_tile_detector.py`
- Reference images: `src/megatron/assets/tiles/reference_good/`
