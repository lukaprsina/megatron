# Task 2 — Implementation Plan

Decisions locked in the grill-with-docs session on 2026-06-03. Simulation only (RViz + Gazebo).

## Node inventory

| Node | File | Status | Notes |
|---|---|---|---|
| `task2_controller` | `megatron/task2_controller.py` | **New** | Replaces `mission_controller` for Task 2 |
| `face_detector` | `megatron/face_detector.py` | **Enhance** | Add throttled `face_recognition` |
| `ring_detector` | `megatron/ring_detector.py` | Keep | Unchanged |
| `cylinder_detector` | `megatron/cylinder_detector.py` | **New** | Wraps dis_tutorial5 C++ + `/spill_check` service |
| `line_detector` | `megatron/line_detector.py` | **New** | Yellow avoidance + workstation blob detection merged |
| `blue_line_follower` | `megatron/blue_line_follower.py` | **New** | `/top_camera` centroid steering, direct cmd_vel |
| `qr_reader` | `megatron/qr_reader.py` | **New** | Task QR + report QR, mode via `/robot_state` |
| `perception_visualizer` | `megatron/perception_visualizer.py` | **Extend** | Add barrel/workstation/anomaly markers |
| `cylinder_segmentation` | dis_tutorial5 C++ | Use as-is | Publishes `/cylinder_markers` |
| `arm_mover_actions` | dis_tutorial7 | Use as-is | Existing poses sufficient |

## Key topics

| Topic | Type | Published by | Purpose |
|---|---|---|---|
| `/robot_state` | `String` (latched) | `task2_controller` | Global state broadcast |
| `/detected_faces` | `PoseStamped` | `face_detector` | Confirmed faces with name in header |
| `/detected_rings` | `PoseStamped` | `ring_detector` | Confirmed rings with color in frame_id |
| `/detected_cylinders` | `PoseStamped` | `cylinder_detector` | Confirmed barrels with orientation + color |
| `/detected_workstations` | `Marker` | `line_detector` | Confirmed red/green workstation positions |
| `/spill_check` | `Trigger` (service) | `cylinder_detector` | Called by controller during INTERACT_BARREL |
| `/qr_task` | `String` | `qr_reader` | Decoded task string; controller reads in INTERACT |
| `/cmd_vel_unstamped` | `Twist` | `blue_line_follower`, `line_detector` (avoider) | Direct drive |

## Controller state machine

```
INIT
  → wait for Nav2 lifecycle + initial pose
PATROL
  → waypoint navigation, all detectors running
  → on confirmed face/barrel in queue → preempt, APPROACH_TARGET
  → on patrol complete → INSPECT_WORKSTATION (if task assigned) or FOLLOW_BLUE_LINE
APPROACH_TARGET
  → navigate to approach point (0.35m face-front or 0.5m+0.3m lateral barrel)
  → on arrival → INTERACT
INTERACT
  → face: wait for /qr_task → record task, speech greeting, resume PATROL
  → barrel: call /spill_check, speech result, resume PATROL
INSPECT_WORKSTATION
  → navigate to workstation (from /detected_workstations pose)
  → _tick_inspection() runs _inspection_phase 0–5:
      0: forward until LiDAR ≤ 0.30m from wall
      1: rotate to target yaw (red=π, green=π/2)
      2: Hough tilt correction via /top_camera (P-controller, stop <0.5°)
      3: reverse until yellow line seen or rear LiDAR ≤ 0.40m
      4: forward scan 0.08 m/s, pause 2s/tile, warp+SSIM classify
      5: escape: 130° CW turn, drive forward 4s
  → on complete → FOLLOW_BLUE_LINE
FOLLOW_BLUE_LINE
  → publish "FOLLOW_BLUE_LINE" to /robot_state (edge-triggers blue_line_follower)
  → on "report" /qr_task → generate markdown, call pandoc, speak summary, DONE
DONE
  → stop, RViz summary marker
```

## Anomaly detection

- **Reference**: `worlds/task2_meshes/good1.png` (512×512, identical across worlds)
- **Pipeline**: arm camera frame → OTSU → contour → 4-corner → `cv2.warpPerspective` → resize 512×512 → `skimage.metrics.structural_similarity` vs reference
- **Threshold**: calibrate in sim before evaluation (expect ~0.85+ for good, ~0.65 for cracked)
- **Dependencies**: `scikit-image` (SSIM), `opencv-python` (warp) — both already available

## Scope decisions

| Feature | Decision |
|---|---|
| Autonomous exploration | **Skip** — fixed waypoints |
| Gender recognition | **Skip** |
| ASR dialogue | **Skip** — QR code shortcut |
| Face recognition | **Implement** — dlib `face_recognition`, throttled every 5th YOLO hit |
| Yellow line avoidance | **Implement** — `line_detector.py`, dual-topic override |
| Blue line following | **Implement** — `blue_line_follower.py`, `/top_camera` |
| Cylinder/barrel detection | **Implement** — dis_tutorial5 C++ + `cylinder_detector.py` |
| Spill detection | **Implement** — `/spill_check` Trigger service on `cylinder_detector` |
| Workstation detection | **Implement** — color blob during patrol, aspect ratio filter |
| Anomaly detection | **Implement** — warp + SSIM |
| Inspection report | **Implement** — markdown + pandoc subprocess |

## Implementation phases

### Phase 0 — Launch infra (do first, unblocks everything)

- [ ] Add `cylinder_segmentation` (dis_tutorial5 C++) to `task2.launch.py`
- [ ] Verify `/cylinder_markers` publishes in task2 world
- [ ] Verify `/top_camera/rgb/preview/image_raw` publishes in task2 world
- [ ] Verify `arm_mover` accepts `look_at_belt_right` / `look_at_belt_left` commands

### Phase 1 — Controller skeleton + patrol

- [ ] `task2_controller.py`: INIT + PATROL states only
- [ ] Subscribe to `/detected_faces`, `/detected_rings`, `/detected_cylinders`
- [ ] Waypoint loading from `waypoints/task.yaml`
- [ ] Publish `/robot_state` on each transition
- [ ] Verify patrol runs end-to-end in sim, all waypoints reached

### Phase 2 — New detectors (parallel with Phase 1)

- [ ] `cylinder_detector.py`: subscribe to `/cylinder_markers`, `IncrementalTrackManager`, color + orientation detection, publish `/detected_cylinders`
- [ ] `cylinder_detector.py`: add `/spill_check` Trigger service (PC2 point count in Z-slice)
- [ ] `line_detector.py`: yellow avoidance first — danger zone ROI, HSV mask, dual-topic override on `/cmd_vel_unstamped` + `/cmd_vel`
- [ ] `line_detector.py`: add workstation detection — red/green blob in floor ROI, aspect ratio > 4:1, `IncrementalTrackManager`, publish `/detected_workstations`

### Phase 3 — Room 1 completion

- [ ] `face_detector.py`: load personnel photos from `worlds/task2_meshes/`, precompute encodings, run `face_recognition.compare_faces()` every 5th YOLO hit, include name in confirmed track
- [ ] `qr_reader.py`: subscribe to OAK-D image, dual-engine decode (WeChatQR + cv2), publish `/qr_task`; face mode only for now
- [ ] `task2_controller.py`: APPROACH_TARGET state (approach candidates, costmap guard, fan of 8 angles)
- [ ] `task2_controller.py`: INTERACT state — face branch (wait for `/qr_task`, record task, speech) + barrel branch (call `/spill_check`, speech)
- [ ] `blue_line_follower.py`: subscribe to `/top_camera`, HSL blue mask, centroid P-controller, EMA smoothing, publish `/cmd_vel_unstamped`; edge-detect `/robot_state` for activation

### Phase 4 — Inspection pipeline

- [ ] `task2_controller.py`: INSPECT_WORKSTATION + `_tick_inspection()` — phases 0–5
  - Phase 0: subscribe to `/scan` (LaserScan), drive forward until ≤ 0.30m
  - Phase 1: rotate to target yaw via Nav2 Spin action
  - Phase 2: subscribe to `/top_camera`, Hough lines, P-controller tilt correction
  - Phase 3: reverse until `/detected_workstations` yellow seen or rear LiDAR ≤ 0.40m
  - Phase 4: forward scan, arm to `look_at_belt_right`/`left`, warp+SSIM per tile
  - Phase 5: escape turn + drive forward
- [ ] `task2_controller.py`: FOLLOW_BLUE_LINE state
- [ ] `qr_reader.py`: add report QR mode (active during FOLLOW_BLUE_LINE)

### Phase 5 — Report + visualization

- [ ] Report: collect rings/barrels/anomalies in controller, format markdown, `subprocess.run(['pandoc', ...])` → PDF
- [ ] `perception_visualizer.py`: extend for barrel markers, workstation markers, anomaly tile overlays
- [ ] `config/production.rviz`: add new marker topics, `/detected_cylinders`, `/detected_workstations`

### Phase 6 — Tuning + end-to-end

- [ ] Calibrate SSIM threshold: run inspection in sim on known good/damaged tiles
- [ ] Review `waypoints/task.yaml`: ensure patrol passes close enough to both conveyors for workstation line detection
- [ ] End-to-end run: room 1 patrol → workstation inspection → room 2 blue line → report
- [ ] Tune `line_detector.py` HSV ranges and aspect ratio threshold in sim

## Reference files

| Purpose | Path |
|---|---|
| Teammate behavior_manager | `src/vendor/teammate-project/src/task1/task1/behavior_manager.py` |
| Teammate blue_line_explorer | `src/vendor/teammate-project/src/task1/task1/blue_line_explorer.py` |
| Teammate yellow_line_avoider | `src/vendor/teammate-project/src/task1/task1/yellow_line_avoider.py` |
| Teammate cylinder_localizator | `src/vendor/teammate-project/src/task1/task1/cylinder_localizator.py` |
| Teammate station_inspector | `src/vendor/teammate-project/src/task1/task1/station_inspector.py` |
| Teammate qr_reader | `src/vendor/teammate-project/src/task1/task1/qr_reader.py` |
| Teammate face_recognizer | `src/vendor/teammate-project/src/task1/task1/face_recognizer.py` |
| Good tile reference | `src/megatron/worlds/task2_meshes/good1.png` |
| Arm poses | `src/vendor/dis_tutorial7/scripts/arm_mover_actions.py` |
| Existing controller (Task 1) | `src/megatron/megatron/controller.py` |
| Perception utils | `src/megatron/megatron/perception_utils.py` |
