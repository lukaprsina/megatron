# 005 — Barrel approach (position-based, single candidate)

**Decision**: Barrels use a single approach candidate computed from robot→barrel direction
(vertical barrels) or perpendicular to barrel axis with lateral shift (horizontal
barrels). No fanout (unlike faces). Default approach distance: 0.60 m.

**Rationale**: Barrels are larger and more forgiving than faces. Nav2 + costmap resolves
the approach path without needing multiple angular candidates. The perpendicular lateral
offset for horizontal barrels keeps the robot clear of the barrel's long axis while still
facing it for spill check.

**Vertical**: Approach from robot's current position toward the barrel centroid, offset
by `barrel_approach_distance`. Single candidate.

**Horizontal**: Approach perpendicular to the barrel's axis, on the side closest to the
robot's current position, with a `barrel_lateral_offset` (0.30 m) to the robot's right.
Axis yaw is computed from the quaternion in `target["quat"]` (identity quat by convention
per ADR-002). If quat is unavailable, axis yaw defaults to 0.

**Approach distance bumped to 0.80 m** in yellow_avoider2 design to reduce off-angle
approaches near boundaries. The 0.60 m default in `task2_controller.py` should be
updated to match if boundary approaches become an issue.

**Code**: `src/megatron/megatron/task2_controller.py:_barrel_approach_candidates` (line 740)
