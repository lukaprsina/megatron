# 002 — Cylinder encoding convention

**Decision**: `cylinder_detector.py` publishes `/detected_cylinders` as `PoseStamped`
with `frame_id = "map|{color}|{orientation}"` (e.g. `"map|green|horizontal"`), identity
quaternion (w=1). Axis yaw is deliberately not encoded.

**Rationale**: The C++ `cylinder_segmentation` already computes the RANSAC axis vector
(`coefficients_cylinder->values[3..5]`). Encoding it would be a 5-line addition. Skipped
because Nav2 + costmap resolves valid approach paths regardless of barrel orientation;
perpendicular approach provides no measurable benefit. Color and orientation are conveyed
via `frame_id` parsing, following the same `"map|…"` pattern used by `ring_detector.py`.

**Cluster dedup**: Custom `Cluster` class (not ITM) supports orientation-dependent merge
radii: 0.33 m (vertical) / 0.70 m (horizontal). Suppressed clusters (duplicates within
1.5 m horizontal radius) carry a `suppressed` flag and are excluded from
`_republish_confirmed` to prevent stale centroids from leaking to the controller.

**Consequences**:
- Controller parses `frame_id` by splitting on `|`: `parts[1]` = color, `parts[2]` =
  orientation.
- If perpendicular approach becomes necessary in sim, add axis yaw encoding to C++ publisher
  and update `_barrel_approach_candidates` in `task2_controller.py`.
- Horizontal barrels already compute axis yaw from `frame_id` parsing (orientation-based
  lateral shift in `_barrel_approach_candidates`).

**Code**: `src/megatron/megatron/cylinder_detector.py` (Cluster at line 102, publisher at
line ~300, `_republish_confirmed` skips suppressed at ~360)
