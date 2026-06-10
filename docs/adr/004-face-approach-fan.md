# 004 — Face approach via fan-of-8 from surface normal

**Decision**: Compute approach candidates for detected faces using a fan of 16 angles
(8 symmetric pairs) radiating from the surface normal, with distance-based scaling via
quadratic function `f(x) = (x/25)²`.

**Rationale**: A single approach position often fails due to costmap obstacles or
unreachable orientations. The fan provides fallback candidates at increasing angular
offsets, tried in priority order (closest to surface normal first). This is the same
pattern as the Task 1 `controller.py`.

**Angles** (degrees): `0, ±2, ±3, ±5, ±7, ±9, ±10, ±15`

Commented-out extensions up to ±70 degrees are available if the initial fan proves
insufficient. Distance scaling via `f(angle)` means wider-angle candidates are placed
farther from the face, giving Nav2 more room to maneuver. Default approach distance:
0.50 m (configurable via `face_approach_distance` parameter).

**Costmap pre-check**: Before sending a NavigateToPose action, the controller checks the
global costmap at the candidate pose. If cost ≥ 50, the candidate is skipped.

**Retry**: On abort, the next candidate is tried. On all candidates exhausted, the target
is re-queued with an increasing retry offset (`approach_retry_offset`, default 0.20 m) up
to `MAX_RETRY_CYCLES` (20).

**Code**: `src/megatron/megatron/task2_controller.py:_face_approach_candidates` (line 690)
