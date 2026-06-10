# 007 — Workstation detection via minAreaRect + ITM confirmation

**Decision**: Detect conveyor belt workstations using `cv2.minAreaRect` (oriented minimum
area rectangle) for aspect ratio filtering, NOT `cv2.boundingRect` (axis-aligned).

**Rationale**: Conveyor belts may be at arbitrary orientations in the image. A belt
rotated 35° has a near-square axis-aligned bounding box (e.g. 98×85, aspect 1.15) and
would fail `MIN_ASPECT_RATIO=3.0` if using `boundingRect`. `minAreaRect` reports the true
oriented dimensions regardless of rotation.

**Pipeline**:
1. HSV threshold: red (H ∈ [0,10]∪[170,180]) or green (H ∈ [40,80]), S ≥ 80, V ≥ 60.
2. Morphological close (5×5 ellipse), find contours.
3. For each contour with area ≥ 2000 px²: `minAreaRect` → aspect ratio ≥ 3.0.
4. `extract_3d_points_from_pc2()` on the mask region from organized PC2
   (`/top_camera/rgb/preview/depth/points`).
5. TF to map frame, centroid = mean XY of valid points.
6. ITM dedup: if any confirmed workstation of same color within 0.5 m → skip.
7. Confirmation: 5 votes required → publish `Marker` on `/detected_workstations`
   (ns = `"red"` or `"green"`).

**Risk**: Top camera FOV during PATROL (garage arm pose) sees floor ~0.5–2 m ahead.
Workstations are large multi-meter belts so any close pass should trigger detection.
Waypoints must include a pass within 2 m of each workstation's edge.

**Code**: `src/megatron/megatron/workstation_detector.py`
