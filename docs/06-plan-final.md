# Task 2 — Final Implementation Plan

**Supersedes** 05-plan.md, 05-plan-revision.md, 05-plan-revision2.md.
All prior revision decisions are incorporated. Revised 2026-06-04 (grill-with-docs, Phase 2 split).
Locked 2026-06-03. Revised 2026-06-04.

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

| Topic                    | Type               | Publisher                                            | QoS                                     | Notes                                                                   |
| ------------------------ | ------------------ | ---------------------------------------------------- | --------------------------------------- | ----------------------------------------------------------------------- |
| `/robot_state`           | `String`           | task2_controller                                     | TRANSIENT_LOCAL, RELIABLE, KEEP_LAST(1) | Global state broadcast — every node reads this                          |
| `/detected_faces`        | `PoseStamped`      | face_detector                                        | default                                 | frame_id = `"map"`, name in future field                                |
| `/detected_rings`        | `PoseStamped`      | ring_detector                                        | default                                 | frame_id = `"map\|{color}"`                                             |
| `/detected_cylinders`    | `PoseStamped`      | cylinder_detector                                    | default                                 | **frame_id = `"map\|{color}\|{orientation}"`** (see §Cylinder encoding) |
| `/detected_workstations` | `Marker`           | workstation_detector                                 | default                                 | ns = `"red"` or `"green"`, pose = centroid                              |
| `/yellow_line_seen`      | `Bool`             | yellow_avoider                                       | TRANSIENT_LOCAL, RELIABLE, KEEP_LAST(1) | Latched; True when yellow under robot during INSPECT_WORKSTATION        |
| `/spill_check`           | `std_srvs/Trigger` | cylinder_detector                                    | service                                 | Point-count in Z-slice [0.005, 0.15m]                                   |
| `/qr_task`               | `String`           | qr_reader                                            | default                                 | Raw decoded QR text                                                     |
| `/cmd_vel_unstamped`     | `Twist`            | task2_controller, blue_line_follower, yellow_avoider | default                                 | Direct drive                                                            |
| `/cmd_vel`               | `TwistStamped`     | yellow_avoider                                       | default                                 | **Must also publish here** to override Nav2                             |
| `/arm_command`           | `String`           | task2_controller                                     | default                                 | Arm pose names                                                          |
| `/scan`                  | `LaserScan`        | robot                                                | sensor_qos                              | Inspection phases 0 + 3                                                 |

---

## Cylinder encoding convention (`/detected_cylinders`)

`cylinder_detector.py` publishes `PoseStamped`:

- `header.frame_id` = `"map|{color}|{orientation}"` — e.g. `"map|green|horizontal"`
  - Same `"map|…"` pattern as ring_detector
  - `{orientation}` = `"vertical"` or `"horizontal"` (from C++ `marker.text`)
  - `{color}` from average RGB → nearest HSV bucket
- `pose.position` = barrel centroid in map frame
- `pose.orientation` = quaternion encoding **cylinder axis yaw in XY plane**
  - `yaw = atan2(axis_y, axis_x)` where `(axis_x, axis_y)` = RANSAC axis projected to XY
  - For vertical barrels: axis is (0,0,1), yaw = 0.0 (ignored by controller)
  - For horizontal barrels: controller reads yaw to compute perpendicular approach

Controller approach:

- **vertical barrel** → approach from current robot→barrel direction, 0.5 m offset
- **horizontal barrel** → perpendicular to axis in XY + 0.5 m offset + 0.3 m lateral shift (robot-right)

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
Normal (PATROL state):
  If yellow_pixel_count ≥ 300 in danger ROI (bottom 65–95%, center 30–70%):
    enter AVOIDING state
    for 1.8 s: publish stop+reverse to BOTH:
      /cmd_vel_unstamped (Twist, linear.x = -0.12, angular.z = 0)
      /cmd_vel (TwistStamped, same values)
    at 50 Hz
    speak "Prohibited zone!"
  Exit AVOIDING → resume publishing nothing (Nav2 takes over)

INSPECT_WORKSTATION state:
  Skip danger-zone logic entirely
  Instead: monitor same ROI; when yellow_pixel_count ≥ 300:
    publish True to /yellow_line_seen (TRANSIENT_LOCAL, KEEP_LAST(1))
```

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

`cylinder_detector.py` uses `IncrementalTrackManager` internally; publishes on confirmed + updated (every 5 updates). Controller receives multiple messages per barrel.

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

---

## Implementation phases

### Phase 0 — Infra verification (done)

- [x] `cylinder_segmentation` C++ package in `src/cylinder_segmentation/` (done)
- [x] `cylinder_segmentation` in `task2.launch.py` (done)
- [x] Verify `/cylinder_markers` publishes in sim (ros2 topic echo) — 1 pub, `visualization_msgs/Marker`
- [x] Verify `/top_camera/rgb/preview/image_raw` publishes — 1 pub, `sensor_msgs/Image`
- [x] Verify `arm_mover` accepts `look_at_belt_right` / `look_at_belt_left` — `ros2 topic pub` succeeded
- [x] Capture SSIM reference tile → `src/megatron/assets/tile_reference/reference.png` (_save_image_topic.py_)

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

- [ ] C++ `cylinder_segmentation`: encode RANSAC axis yaw in `marker.pose.orientation` for horizontal barrels (identity for vertical)
- [ ] `cylinder_detector.py`: subscribe to `/cylinder_markers` + OAK-D PC2 buffer, ITM dedup, publish `/detected_cylinders` using encoding convention, provide `/spill_check` Trigger service
- [ ] `yellow_avoider.py`: HSL yellow mask, danger-zone ROI, dual-topic `/cmd_vel` + `/cmd_vel_unstamped` override at 50 Hz, PATROL → stop+reverse, INSPECT_WORKSTATION → publish `/yellow_line_seen`
- [ ] `workstation_detector.py`: red/green color blob in floor ROI, aspect > 4:1, ITM confirmation, publish `/detected_workstations` Marker (ns=color, pose=centroid)

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
