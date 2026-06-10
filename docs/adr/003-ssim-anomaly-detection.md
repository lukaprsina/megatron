# 003 — SSIM tile anomaly detection

**Decision**: Use warp+SSIM (Structural Similarity Index) for tile defect detection,
comparing arm-camera captures against a reference tile. PatchCore is retained as a
fallback if SSIM proves insufficient.

**Rationale**: SSIM on a 512×512 warped tile is ~5 ms — acceptable inline during Phase 4
forward scan without threading. The raw mesh texture (`good1.png`) does not match arm-camera
renders due to lighting and anti-aliasing, so the reference tile must be captured from the
sim camera itself using the same OTSU→contour→warp pipeline that Phase 4 will use.

**Reference tile procedure** (one-time):
1. Launch sim with task2 world, position arm at `look_at_belt_right`.
2. Drive robot to a belt with good tiles.
3. Capture one frame from `/top_camera/rgb/preview/image_raw`.
4. Apply OTSU binarisation → contour detection → 4-corner approximation →
   `warpPerspective` → 512×512.
5. Save as `src/megatron/assets/tile_reference/reference.png`.
6. Repeat with a damaged tile; record both SSIM scores to set the threshold.

All three task2 world variants use the same `good1.png` texture, so one capture suffices.

**Consequences**:
- SSIM threshold must be calibrated in Phase 6 from recorded scores.
- If upgraded to PatchCore (~50 ms), move inference to a separate executor thread.
- Phase 4 tile scan loop: brightness crossing threshold → stop → pause 0.3 s → capture →
  SSIM compare → record → resume forward.

**Code**: `src/megatron/megatron/task2_controller.py:_handle_inspection` (Phase 4, TODO)
