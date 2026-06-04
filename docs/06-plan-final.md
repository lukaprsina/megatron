# Task 2 — Final Implementation Plan

**Supersedes** 05-plan.md, 05-plan-revision.md, 05-plan-revision2.md.
All prior revision decisions are incorporated. Revised 2026-06-04 (grill-with-docs, Phase 2 split).
Locked 2026-06-03. Revised 2026-06-04. Revised 2026-06-04 (Phase 2 design session).

---

## Scope decisions

| Feature                     | Decision                                             |
| --------------------------- | ---------------------------------------------------- |
| Autonomous exploration      | Skip — fixed waypoints                               |
| Gender recognition          | Skip (QR shortcut replaces dialogue)                 |
| ASR dialogue                | Skip — QR code shortcut                              |
| Face recognition            | Implement — dlib, throttled every 5th YOLO hit       |
| Yellow line avoidance       | Implement — yellow_avoider, dual-topic override      |
| Blue line following         | Implement — blue_line_follower, /top_camera centroid |
| Cylinder / barrel detection | Implement — C++ RANSAC + cylinder_detector.py        |
| Spill detection             | Implement — /spill_check Trigger service             |
| Workstation detection       | Implement — color blob + ITM confirmation            |
| Anomaly detection           | Implement — warp + SSIM; PatchCore fallback          |
| Inspection report           | Implement — FPDF2 (no pandoc dependency)             |
| Multi-loop patrol           | Skip — single loop, no retry                         |
| Approach fanout (barrels)   | Skip — single position-based candidate               |
| Approach fanout (faces)     | Implement — fan of 8 from surface normal             |

---

## Node inventory

| Node                    | File                                | Status      | Notes                                     |
| ----------------------- | ----------------------------------- | ----------- | ----------------------------------------- |
| `task2_controller`      | `megatron/task2_controller.py`      | **New**     | Replaces `mission_controller`             |
| `face_detector`         | `megatron/face_detector.py`         | **Enhance** | Add face_recognition every 5th hit        |
| `ring_detector`         | `megatron/ring_detector.py`         | Keep        | Unchanged                                 |
| `cylinder_detector`     | `megatron/cylinder_detector.py`     | **New**     | Wraps C++ markers + /spill_check svc      |
| `yellow_avoider`        | `megatron/yellow_avoider.py`        | **New**     | Dual-topic /cmd_vel override at 50 Hz     |
| `workstation_detector`  | `megatron/workstation_detector.py`  | **New**     | Color blob + ITM → /detected_workstations |
| `blue_line_follower`    | `megatron/blue_line_follower.py`    | **New**     | /top_camera centroid + direct cmd_vel     |
| `qr_reader`             | `megatron/qr_reader.py`             | **New**     | Dual-engine; mode via /robot_state        |
| `perception_visualizer` | `megatron/perception_visualizer.py` | **Extend**  | Barrel/workstation/anomaly markers        |
| `cylinder_segmentation` | `src/cylinder_segmentation/` C++    | Done        | Already in task2.launch.py                |
| `arm_mover_actions`     | dis_tutorial7                       | Use as-is   | Already launched                          |

---

## Topic inventory

| Topic                    | Type               | Publisher                                            | QoS                                     | Notes                                                                                                                                    |
| ------------------------ | ------------------ | ---------------------------------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `/robot_state`           | `String`           | task2_controller                                     | TRANSIENT_LOCAL, RELIABLE, KEEP_LAST(1) | Global state broadcast — every node reads this                                                                                           |
| `/detected_faces`        | `PoseStamped`      | face_detector                                        | default                                 | frame_id = `"map"`, name in future field                                                                                                 |
| `/detected_rings`        | `PoseStamped`      | ring_detector                                        | default                                 | frame_id = `"map\|{color}"`                                                                                                              |
| `/detected_cylinders`    | `PoseStamped`      | cylinder_detector                                    | default                                 | **frame_id = `"map\|{color}\|{orientation}"`** (see §Cylinder encoding)                                                                  |
| `/detected_workstations` | `Marker`           | workstation_detector                                 | default                                 | ns = `"red"` or `"green"`, pose = centroid                                                                                               |
| `/yellow_line_seen`      | `Bool`             | yellow_avoider                                       | default (10)                            | True when yellow detected during INSPECT_WORKSTATION. Default QoS sufficient — controller is always subscribed before avoider publishes. |
| `/spill_check`           | `std_srvs/Trigger` | cylinder_detector                                    | service                                 | Point-count in Z-slice [0.005, 0.15m]                                                                                                    |
| `/qr_task`               | `String`           | qr_reader                                            | default                                 | Raw decoded QR text                                                                                                                      |
| `/cmd_vel_unstamped`     | `Twist`            | task2_controller, blue_line_follower, yellow_avoider | default                                 | Direct drive                                                                                                                             |
| `/cmd_vel`               | `TwistStamped`     | yellow_avoider                                       | default                                 | **Must also publish here** to override Nav2                                                                                              |
| `/arm_command`           | `String`           | task2_controller                                     | default                                 | Arm pose names                                                                                                                           |
| `/scan`                  | `LaserScan`        | robot                                                | sensor_qos                              | Inspection phases 0 + 3                                                                                                                  |

---

## Cylinder encoding convention (`/detected_cylinders`)

`cylinder_detector.py` publishes `PoseStamped`:

- `header.frame_id` = `"map|{color}|{orientation}"` — e.g. `"map|green|horizontal"`
  - Same `"map|…"` pattern as ring_detector
  - `{orientation}` = `"vertical"` or `"horizontal"` (from C++ `marker.text`)
  - `{color}` from average RGB → nearest HSV bucket
- `pose.position` = barrel centroid in map frame
- `pose.orientation` = **identity quaternion (w=1)** — axis yaw is NOT encoded

**Axis yaw encoding skipped (decided Phase 2 design session):** The C++ already has the RANSAC
axis vector in `coefficients_cylinder->values[3..5]`, so encoding it would be a 5-line addition.
Skipped because Nav2 + costmap resolves the approach path without a preferred direction; the
perpendicular offset provides no measurable benefit. Barrel orientation is still conveyed via
`marker.text` → frame_id → `barrel_report`.

Controller approach (both vertical and horizontal):

- Approach from current robot→barrel direction, 0.5 m offset
- Lateral shift parameter retained in code but unused for horizontal barrels

---

## Data structures

```python
# In Task2Controller.__init__:

# Ring report
self.ring_counts: dict[str, int] = {}          # color → confirmed count

# Barrel report
self.barrel_report: list[dict] = []
# each dict: {'id': int, 'color': str, 'orientation': str,
#             'leaking': bool, 'pos': np.array}

# Tile / anomaly report
self.tile_results: list[dict] = []
# each dict: {'station': 'red'|'green', 'tile_id': int,
#             'defect': bool, 'ssim': float, 'image': bytes|None}

# Workstation poses (from /detected_workstations)
self.workstation_poses: dict[str, np.ndarray] = {}   # 'red'→[x,y], 'green'→[x,y]

# Task assigned by person QR
self.assigned_task: str | None = None
# values: 'rings' | 'barrels' | 'defects_red' | 'defects_green' | 'nothing' | None
```

---

## State machine

```
INIT
  → wait for Nav2 lifecycle (amcl + bt_navigator + global_costmap all "active")
  → wait for initial amcl_pose
  → transition to PATROL, publish /robot_state

PATROL
  → single loop through waypoints (waypoint_index increments on complete OR abort)
  → on confirmed face in pending_targets → preempt nav, APPROACH_TARGET (face)
  → on confirmed barrel in pending_targets → preempt nav, APPROACH_TARGET (barrel)
  → on waypoint_index >= len(waypoints):
      if assigned_task starts with "defects" AND workstation_poses has that color
          → INSPECT_WORKSTATION
      else
          → FOLLOW_BLUE_LINE

APPROACH_TARGET
  → compute approach pose (face: fan-of-8 from normal; barrel: position-based)
  → navigate to first unblocked candidate
  → on abort: try next candidate; on all exhausted: re-queue target, resume PATROL
  → on success: INTERACT

INTERACT
  → face:
      speak greeting by gender (he/him → "man", she/her → "woman")
      wait for /qr_task to arrive
      parse QR text → assigned_task (see §QR parsing)
      speak confirmation
      mark face as "approached"
      resume PATROL
  → barrel:
      call /spill_check Trigger service
      speak result ("Alert! Alert! This barrel is leaking!" or "Barrel OK.")
      record in barrel_report
      mark barrel as "approached"
      resume PATROL

INSPECT_WORKSTATION
  → navigate to workstation pose (from workstation_poses[color])
  → run _tick_inspection() phases 0–5 (see §Inspection phases)
  → on complete → FOLLOW_BLUE_LINE

FOLLOW_BLUE_LINE
  → publish "FOLLOW_BLUE_LINE" to /robot_state
    (edge-triggers blue_line_follower to start)
  → scan QR codes via qr_reader (active in FOLLOW_BLUE_LINE mode)
  → on /qr_task contains "report" / "thanks" → generate report, DONE
  → also triggers on face detection of Jeff (CTO) → DONE (if report ready)

DONE
  → publish "DONE" to /robot_state
  → stop all motion
  → speak final summary
```

---

## QR text → task parsing

```python
def _parse_qr_task(text: str) -> str | None:
    t = text.lower()
    if "report" in t or "thanks" in t:     # qr_cto.png: "Hello there! Thanks for the report."
        return "report"
    if "red belt" in t or ("red" in t and "defect" in t):
        return "defects_red"
    if "green belt" in t or ("green" in t and "defect" in t):
        return "defects_green"
    if "barrel" in t:
        return "barrels"
    if "ring" in t:
        return "rings"
    if "visitor" in t or ("nothing" in t and "task" in t):
        return "nothing"
    return None
```

Known QR texts (from `worlds/task2_meshes/`):

- `qr_redbelt.png`: "Find all defects on the red belt." → `defects_red`
- `qr_greenbelt.png`: "Find all defects on the green belt." → `defects_green`
- `qr_barrels.png`: "Inspect the barrels. How many of each colour…" → `barrels`
- `qr_rings.png`: "Find all the rings." → `rings`
- `qr_nothing.png`: "I'm just a visitor :)" → `nothing`
- `qr_cto.png`: "Hello there! Thanks for the report." → `report`

---

## Inspection phases

Triggered when entering `INSPECT_WORKSTATION`. Arm pose published at entry.
All motion uses direct `cmd_vel_unstamped` (Twist). Nav2 used only for initial approach.

**Arm pose mapping:**

- `defects_red` → publish `"look_at_belt_right"` at phase 0 entry
- `defects_green` → publish `"look_at_belt_left"` at phase 0 entry
- Phase 5 exit → publish `"garage"`

_Verify in sim — mapping may be swapped depending on world orientation._

**Phase 0 — drive to wall**

```
subscribe /scan (sensor_qos)
forward at 0.08 m/s until min(ranges[0:30] + ranges[330:360]) ≤ 0.30 m
timeout: 10 s (fallback)
```

**Phase 1 — yaw alignment**

```
target_yaw: red=π, green=π/2 (absolute map frame)
P-controller: error = normalize_angle(target_yaw - current_yaw)
cmd_vel angular.z = clamp(1.5 * error, ±0.5 rad/s)
exit when |error| < 0.05 rad, 0-vel for 0.5 s
```

_Risk: hardcoded yaw values. If world orientation changes, update these._
_Derive from workstation_pose → line normal in future if needed._

**Phase 2 — tilt correction**

```
subscribe /top_camera/rgb/preview/image_raw
Hough line detection → angle of dominant lines
P-controller: cmd_vel angular.z = clamp(0.5 * tilt_rad, ±0.3 rad/s)
exit when |tilt_rad| < 0.5° (0.0087 rad) or 5 s timeout
```

**Phase 3 — reverse to start position**

```
reverse at -0.06 m/s
exit on:
  /yellow_line_seen == True  (yellow_avoider publishes when state = INSPECT_WORKSTATION)
  OR min(ranges[150:210]) ≤ 0.40 m (rear obstacle)
  OR timeout 10 s
```

**Phase 4 — forward tile scan**

```
forward at 0.08 m/s
per tick:
  brightness of central ROI in /top_camera/rgb/preview/image_raw
  if brightness crosses threshold (tile under camera) → stop, pause 0.3 s for settling
    arm camera → OTSU → contour → 4-corner → warpPerspective → 512×512
    SSIM vs reference_tile.png
    if SSIM < threshold → defect
    record in tile_results; log
    wait remainder of 2.0 s pause, resume forward
exit when /yellow_line_seen == True again (reached wall end) or timeout 30 s
```

**Phase 5 — escape**

```
publish "garage" to /arm_command
CW turn: cmd_vel angular.z = -0.5 rad/s for 130° / 0.5 = 4.5 s (≈ 2.27 rad CW)
forward: cmd_vel linear.x = 0.15 m/s for 4.0 s
stop
```

---

## SSIM reference tile

**Problem:** `good1.png` (raw mesh texture) does not match arm-camera renders (lighting, anti-aliasing).

**Procedure (Phase 0 verification, one-time):**

1. Launch sim with task2 world
2. Position arm at `look_at_belt_right`, drive robot to a belt with good tiles
3. Capture one frame from `/top_camera/rgb/preview/image_raw`
4. Apply same OTSU → contour → warp pipeline as Phase 4
5. Save as `src/megatron/assets/tile_reference/reference.png`
6. Repeat with a damaged tile → record both SSIM scores
7. Set threshold = midpoint; document in `config/task2_params.yaml`

All three task2 world variants use the same `good1.png` (MD5 `7bbed964a13b4bf449291cce74ce7528`), so one capture suffices.

---

## Yellow avoider design

**Problem:** nav2 publishes `/cmd_vel` (TwistStamped) continuously. Publishing only to
`/cmd_vel_unstamped` (Twist) does not stop Nav2's forward motion. Must publish to **both** topics
at high frequency to win the last-write-wins race.

**Design (in `yellow_avoider.py`):**

```
PATROL / APPROACH_TARGET / INTERACT states:
  Three ROIs along the bottom strip (92–100 % Y): CENTER (45–55 % X),
  LEFT (15–30 % X), RIGHT (70–85 % X).  All check contour-bottom.

  CENTER contour-bottom triggers → determine which side the line is on
  (centroid left/right of ROI center).

    Safe-zone ROI (opposite side) clear → STEERING:
      publish forward + angular on BOTH cmd_vel topics at 50 Hz
      (linear = steer_speed, angular = ±steer_angular)
      Nav2 sees forward progress → local planner re-routes around line
      Exit STEERING when CENTER clears

    Safe-zone ROI ALSO has yellow → BACKING fallback:
      speak "Prohibited zone!"
      reverse 1.8 s → CLEAR

  Contours NOT reaching the bottom edge (yellow ahead/to the side) are
  ignored — Nav2 + costmap handle those.

INSPECT_WORKSTATION state:
  Skip velocity commands entirely (controller owns cmd_vel during inspection)
  CENTER contour-bottom → publish Bool to /yellow_line_seen every tick

INIT / FOLLOW_BLUE_LINE / DONE states:
  Skip yellow detection entirely (avoider is passive)
```

**QoS note:** `/yellow_line_seen` uses default QoS (not TRANSIENT_LOCAL). The controller is always
subscribed before the avoider publishes during inspection; no late-joiner scenario exists.
Default QoS on both sides avoids a TRANSIENT_LOCAL/VOLATILE incompatibility.

---

## Spill check design

Service: `/spill_check` (`std_srvs/Trigger`) provided by `cylinder_detector.py`.

Called by controller during INTERACT when `current_target["type"] == "barrel"`.
At that point the robot is stationary at approach pose (~0.5 m from barrel), OAK-D facing it.

```
On service call:
  grab latest buffered /oakd/rgb/preview/depth/points PC2
  for each point in PC2 (unorganized OK here — we iterate all points):
    TF transform point camera_frame → map frame (tf_buffer, latest available)
    if abs(z_map - ground_z) ∈ [0.005, 0.15]:          # Z-slice near ground
      if dist_xy(point_map, barrel_centroid_map) < 0.5:  # XY radius guard
        count += 1
  return leaking = (count >= 4000)
```

`ground_z` = 0.0 (map frame ground is at z=0 in Gazebo).

**Subscriber:** `cylinder_detector.py` subscribes to `/oakd/rgb/preview/depth/points` with
`sensor_qos` (BEST_EFFORT, KEEP_LAST 1) and stores `self._latest_pc2`. The same topic feeds
`cylinder_segmentation` C++ for RANSAC — no extra overhead.

---

## Workstation detector design

Node: `workstation_detector.py`. Publishes to `/detected_workstations` (Marker, ns=color).

**Confirmed topic (Phase 2 design session):**
`/top_camera/rgb/preview/depth/points` — organized PC2, height=240, width=320,
frame `top_camera_rgb_camera_optical_frame`. Verified in sim.

**Pipeline per frame:**

```
Subscribe:
  /top_camera/rgb/preview/image_raw       → color mask
  /top_camera/rgb/preview/depth/points    → organized PC2 (loosely synced by latest-buffer)

Per frame:
  1. HSV threshold: red (H ∈ [0,10]∪[170,180]) or green (H ∈ [40,80]), S ≥ 80, V ≥ 60
  2. Morphological close (5×5 ellipse) → contours
  3. For each contour with area ≥ 2000 px²:
       **minAreaRect** aspect ratio ≥ 3.0 (oriented — rotated belts would appear
       near-square in a `boundingRect` and fail the filter; see §O)
  4. extract_3d_points_from_pc2(mask, latest_pc2_msg, max_range=4.0)
       → (N, 3) in top_camera_rgb_camera_optical_frame
  5. TF to map frame (tf_buffer, latest)
  6. centroid = mean XY of valid points
  7. ITM dedup: if any confirmed workstation of same color within 0.5 m → skip
  8. vote_count += 1; if vote_count ≥ 5 → confirm, publish Marker
```

**Imports from existing code:** `extract_3d_points_from_pc2` from `megatron.perception_utils`.

**Risk:** top camera FOV during PATROL (garage arm pose) sees floor ~0.5–2 m ahead.
Workstations are large (multi-meter belts) so any close pass should trigger detection.
Ensure waypoints include a pass within 2 m of each workstation's edge.

---

## Personnel + face recognition

Personnel photos: `src/megatron/worlds/task2_meshes/*.png` with naming:
`{firstname}_{pronoun1}_{pronoun2}_{job_title}.png`

| File                                  | Name   | Gender | Role              |
| ------------------------------------- | ------ | ------ | ----------------- |
| `anita_she_her_quality_inspector.png` | Anita  | F      | Quality inspector |
| `elena_she_her_forklift_driver.png`   | Elena  | F      | Forklift driver   |
| `fred_he_him_visitor.png`             | Fred   | M      | Visitor           |
| `jeff_he_him_cto.png`                 | Jeff   | M      | CTO               |
| `robert_he_him_line_painter.png`      | Robert | M      | Line painter      |

Face_detector enhancement:

1. Load all photos at startup; precompute `face_recognition.face_encodings()` per image
2. On every 5th YOLO hit on a confirmed track: run `face_recognition.compare_faces()`
3. Update track `label` = `"{firstname}|{pronoun}|{role}"`; publish in future `header.frame_id`
4. Gender derived from pronoun: `"he"` → male, `"she"` → female

---

## /detected_cylinders dedup and "approached" flag

`cylinder_detector.py` uses a custom `Cluster` class internally (not ITM — Cluster supports orientation-dependent merge radii); publishes on confirmed + updated (every 5 updates). Controller receives multiple messages per barrel.

Controller dedup:

```python
# In _cylinder_cb:
for b in self.found_barrels_tracked:
    if np.linalg.norm(pos - b["pos"]) < 0.8:   # dedup_distance
        b["pos"] = pos  # update position
        if not b.get("approached", False):
            _requeue_if_not_pending(...)
        return
# New barrel:
self.found_barrels_tracked.append({"pos": pos, "color": ..., "orientation": ...,
                                    "approached": False, ...})
self.pending_targets.append(...)
```

Same pattern as existing `controller.py:_requeue_if_not_pending()`.

---

## Report generation

Use **FPDF2** (Python-only, no system dependency). Sections:

1. Ring Counting: table of color → count
2. Barrel Inspection: table of id, color, orientation, leaking, position
3. Anomaly Detection: per-workstation per-tile table (tile_id, defect, ssim)

Trigger: controller receives QR task `"report"` (from Jeff's QR `"Hello there! Thanks for the report."`).

```python
# In _handle_follow_blue_line, on qr_task == "report":
self._generate_report()
self._transition(State.DONE)
```

---

## Adversarial findings (new — not in any prior revision)

### A. QR task text is NOT a simple token

All prior plans show `/qr_task` containing values like `"defects green"`. Actual QR content:
`"Find all defects on the green belt."` — needs substring matching, not equality. §QR parsing above handles this.

### B. CTO QR does not contain "report"

Prior plan: "on 'report' /qr_task → generate". CTO QR says "Hello there! Thanks for the report."
Match on `"thanks" in text or "report" in text` (§QR parsing).

### C. Yellow avoider must publish to both /cmd_vel AND /cmd_vel_unstamped

05-plan.md specifies only `/cmd_vel_unstamped`. Nav2 publishes `/cmd_vel` (TwistStamped) at
~10 Hz; emergency stop will be overridden within 100 ms if only `_unstamped` is written.
Fixed in §Yellow avoider design.

### D. `/detected_cylinders` encoding was undefined

No prior doc specified the frame_id format or how axis direction is encoded for horizontal barrels.
Fixed in §Cylinder encoding convention.

### E. `_requeue_if_not_pending` / "approached" flag missing

Without an `"approached"` flag, face_detector's repeated publishes (every 5 updates) re-queue
already-approached people every ~5 frames. Controller needs explicit dedup with flag.
Fixed in §/detected_cylinders dedup section (same pattern for faces).

### F. Pandoc not guaranteed on eval machine

Fixed: use FPDF2 only.

### G. Arm pose mapping needs sim verification

`"look_at_belt_right"` vs `"look_at_belt_left"` for red/green workstation is an assumption.
Document as requiring sim calibration in Phase 0.

### H. Blue line intersection handling not specified

Simple centroid P-controller will fail at T-junctions. Risk accepted for now — evaluate in sim.
If intersections cause wrong-branch behavior, add 3-ROI split-mode state machine from teammate's
`blue_line_explorer.py`.

### I. SSIM blocking in Phase 4

warp+SSIM on 512×512 ≈ 5 ms — acceptable inline. If upgraded to PatchCore (~50 ms), move to
executor thread. Document in §Inspection phases.

### J. C++ axis yaw encoding not needed

Prior plan proposed encoding RANSAC axis yaw into `marker.pose.orientation` to give `cylinder_detector`
the horizontal barrel's axis direction for perpendicular approach. Skipped: Nav2 + costmap already
resolves a valid approach path regardless of direction; perpendicular offset provides no measurable
benefit. The C++ already has the vector (`coefficients_cylinder->values[3..5]`) — can be added later
if perpendicular approach proves necessary in sim.

### K. `/yellow_line_seen` QoS — TRANSIENT_LOCAL unnecessary

Prior plan marked `/yellow_line_seen` as TRANSIENT_LOCAL, RELIABLE, KEEP_LAST(1). This creates a
QoS incompatibility: the controller subscribes with default (VOLATILE) QoS and won't receive the
latched buffer on connect. No late-joiner scenario exists here — the controller is always alive
before the avoider publishes during inspection. Changed to default QoS on both sides.

### L. Compactness check blocks barrel confirmation until ~17 sightings

`compact_enough(CONFIRM_THRESH)` passed CONFIRM_THRESH=10 as the minimum compact inlier count, but
`compact_points` only returns the inlier subset when it has ≥ 60% of all sightings. At 10 sightings
with 7 inliers: 7 ≥ max(3, 6) → returns 7 inliers → 7 ≥ 10 → False. Confirmation required ~17
sightings (17 × 0.6 ≈ 10 inliers). Decoupled with a separate **COMPACT_MIN=6** constant — same
protection against dispersion, correct 10-sighting confirmation threshold.

### M. `_suppress_duplicates` was undone by `_republish_confirmed`

Suppressed clusters were marked `confirmed=True` to silence primary publication, but
`_republish_confirmed` blindly re-broadcast all `confirmed` clusters every 3 s — including
suppressed ones with noisy centroids. Controller could see duplicate barrels if suppressed
cluster centroids were > 0.8 m apart (dedup) but < 1.5 m (horizontal suppress radius).
Added `suppressed` flag to `Cluster`; `_republish_confirmed` skips suppressed clusters.

### N. `yellow_avoider.py` never published `False` — latch effect across inspection phases

Prior code published `Bool(data=True)` only on yellow detection; when yellow disappeared, nothing
was published. Controller's `_yellow_seen` retained stale `True` from earlier phases. Phase 4's
"reverse-to-yellow" would trigger instantly if yellow had ever been seen during inspection.
Fixed: publish `Bool(data=danger_px >= thresh)` on **every** tick in INSPECT_WORKSTATION state,
so `/yellow_line_seen` tracks instantaneous sensor state, not a one-shot latch.

### O. `cv2.boundingRect` for workstation aspect ratio fails on rotated belts

Conveyor belts may be at arbitrary orientations in the image. A belt rotated 35° has a near-square
axis-aligned bounding box (e.g. 98×85, aspect 1.15) and would fail `MIN_ASPECT_RATIO=3.0`.
Switched to `cv2.minAreaRect` which reports the true oriented dimensions regardless of rotation.

### P. Yellow avoider blocks PATROL — arena has yellow boxes at spawn

Plan specified pixel-count threshold stop+reverse. Our task2 arena has ~1 m yellow boxes
at spawn, filling the danger ROI → CLEAR↔BACKING lockup. Controller oscillated
PATROL/APPROACH_TARGET, robot never reached first waypoint, `/yellow_line_seen` never fired.

Fix iteration 1: contour-bottom + tight ROI (45–55 %×92–100 %). Fixed spawn-box false positives
but pure backing on approach → Nav2 re-approached on same trajectory → endless BACKING loop.

Fix iteration 2 (TURNING): BACKING → 1.2 s TURNING away → CLEAR. Nav2 immediately undid the
turn (active NavigateToPose goal) and drove straight back into the line.

Fix iteration 3 (lateral STEERING): three ROIs — CENTER, LEFT, RIGHT — computed every frame.
CENTER contour-bottom triggers → check safe-zone ROI on the OPPOSITE side of the line.
Safe-zone clear → publish forward + angular (away from line, toward clear side) at 50 Hz.
Nav2 sees forward progress toward goal → local planner re-routes around line naturally.
Safe-zone blocked → BACKING fallback. Removed `danger_px_threshold`, TURNING state,
`turn_duration`, `_turn_end`. Added `steer_speed`, `steer_angular`, safe-zone params.

---

## Implementation phases

### Phase 0 — Infra verification (done)

- [x] `cylinder_segmentation` C++ package in `src/cylinder_segmentation/` (done)
- [x] `cylinder_segmentation` in `task2.launch.py` (done)
- [x] Verify `/cylinder_markers` publishes in sim (ros2 topic echo) — 1 pub, `visualization_msgs/Marker`
- [x] Verify `/top_camera/rgb/preview/image_raw` publishes — 1 pub, `sensor_msgs/Image`
- [x] Verify `arm_mover` accepts `look_at_belt_right` / `look_at_belt_left` — `ros2 topic pub` succeeded
- [x] Capture SSIM reference tile → `src/megatron/assets/tile_reference/reference.png` (_save_image_topic.py_)
- [x] Verify `/top_camera/rgb/preview/depth/points` — organized PC2, height=240, width=320, frame `top_camera_rgb_camera_optical_frame`
- [x] Verify `/oakd/rgb/preview/depth/points` — available for cylinder spill check buffer

### Phase 1 — Controller skeleton (done)

- [x] `task2_controller.py`: INIT + PATROL states
- [x] Subscribe to `/detected_faces`, `/detected_rings`, `/detected_cylinders`
- [x] Waypoint loading from `waypoints/task.yaml`
- [x] Publish `/robot_state` (TRANSIENT_LOCAL) on every transition
- [x] APPROACH_TARGET state (face fan-of-8, barrel position-based)
- [x] INTERACT stub (log only, resume PATROL)
- [x] Add `task2_controller` to `setup.py` entry_points
- [x] Smoke test: robot completes patrol loop in sim (waypoints recorded 2026-06-04)

### Phase 2 — New detectors

**Build order:** yellow_avoider → cylinder_detector → workstation_detector.
**No C++ changes in this phase** (axis yaw encoding skipped — see §Cylinder encoding convention).

- [x] `yellow_avoider.py`: HSV yellow mask, three contour-bottom ROIs (CENTER + LEFT + RIGHT) along 92–100% Y; dual-topic `/cmd_vel` + `/cmd_vel_unstamped` at 50 Hz; PATROL/APPROACH_TARGET/INTERACT → lateral STEERING (forward + angular away from line) when safe-zone is clear, BACKING fallback when blocked (§P); INSPECT_WORKSTATION → publish `/yellow_line_seen` True/False per tick based on CENTER contour-bottom
- [x] `cylinder_detector.py`: subscribe `/cylinder_markers` + buffer `/oakd/rgb/preview/depth/points` (sensor_qos); Cluster dedup (orientation-dependent radii: 0.33 m vertical / 0.70 m horizontal, CONFIRM_THRESH=10, COMPACT_MIN=6 — see §L); publish `/detected_cylinders` PoseStamped (`frame_id="map|{color}|{orientation}"`, identity quat); suppressed clusters excluded from republish (see §M); provide `/spill_check` Trigger service (see §Spill check design)
- [x] `workstation_detector.py`: subscribe `/top_camera/rgb/preview/image_raw` + `/top_camera/rgb/preview/depth/points`; HSV red/green mask → contour → **minAreaRect** aspect ≥ 3.0 (not boundingRect — see §O) → `extract_3d_points_from_pc2` → TF to map → ITM dedup (0.5 m, 5 votes) → publish `/detected_workstations` Marker (ns=color, pose=centroid)

### Phase 3 — Room 1 completion

- [ ] `face_detector.py`: load personnel photos, precompute encodings, run every 5th hit, update label
- [ ] `qr_reader.py`: OAK-D forward camera, dual-engine (WeChatQR + cv2), mode via `/robot_state`, publish `/qr_task`
- [ ] `task2_controller.py`: INTERACT state — face (QR wait + task assignment + speech) + barrel (/spill_check + speech)
- [ ] `blue_line_follower.py`: HSL blue mask, centroid P-controller, EMA smoothing, edge-detect `/robot_state`

### Phase 4 — Inspection pipeline

- [ ] `task2_controller.py`: INSPECT_WORKSTATION + `_tick_inspection()` phases 0–5
  - Phase 0: `/scan` subscriber, drive to wall
  - Phase 1: yaw P-controller
  - Phase 2: Hough tilt correction
  - Phase 3: reverse to yellow line
  - Phase 4: tile scan + warp + SSIM
  - Phase 5: escape turn
- [ ] `task2_controller.py`: FOLLOW_BLUE_LINE state
- [ ] `qr_reader.py`: active in FOLLOW_BLUE_LINE mode

### Phase 5 — Report + visualization

- [ ] `task2_controller.py`: `_generate_report()` using FPDF2
- [ ] `perception_visualizer.py`: barrel markers, workstation markers, tile overlays
- [ ] `config/production.rviz`: add `/detected_cylinders`, `/detected_workstations` topics

### Phase 6 — Tuning + end-to-end

- [ ] Calibrate SSIM threshold in sim
- [ ] Review `waypoints/task.yaml` — ensure patrol covers both conveyors
- [ ] Verify arm pose mapping (red/green → left/right)
- [ ] End-to-end run: room 1 → inspection → room 2 → report
- [ ] Tune yellow_avoider HSV ranges, SSIM threshold, LiDAR distances in sim

---

## Reference files

| Purpose                       | Path                                                                  |
| ----------------------------- | --------------------------------------------------------------------- |
| Teammate behavior_manager     | `src/vendor/teammate-project/src/task1/task1/behavior_manager.py`     |
| Teammate blue_line_explorer   | `src/vendor/teammate-project/src/task1/task1/blue_line_explorer.py`   |
| Teammate yellow_line_avoider  | `src/vendor/teammate-project/src/task1/task1/yellow_line_avoider.py`  |
| Teammate cylinder_localizator | `src/vendor/teammate-project/src/task1/task1/cylinder_localizator.py` |
| Teammate station_inspector    | `src/vendor/teammate-project/src/task1/task1/station_inspector.py`    |
| Teammate qr_reader            | `src/vendor/teammate-project/src/task1/task1/qr_reader.py`            |
| Existing Task 1 controller    | `src/megatron/megatron/controller.py`                                 |
| Perception utilities          | `src/megatron/megatron/perception_utils.py`                           |
| Arm poses                     | `src/vendor/dis_tutorial7/scripts/arm_mover_actions.py`               |
| Good tile reference           | `src/megatron/worlds/task2_meshes/good1.png`                          |
| Personnel photos              | `src/megatron/worlds/task2_meshes/*.png` (named by person)            |
| QR codes                      | `src/megatron/worlds/task2_meshes/qr_*.png`                           |
| Cylinder segmentation diff    | `cylinder_segmentation.diff` (workspace root)                         |
