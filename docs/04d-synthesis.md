# Teammate Project Analysis — Task 2 Solutions

A thorough exploration of the teammate's complete reference implementation (`src/vendor/teammate-project/`) compared against our megatron codebase and available vendor tutorials. Generated for Task 2 planning.

---

## 1. Executive Summary

The teammate project is a **complete Task 2 implementation** — approximately **9,600 lines of Python** across 28 modules in a single ROS 2 package (`task1`), coordinating via ~30 topics, 1 service, and 1 Nav2 action. They achieved everything in the spec: blue-line following, yellow-line avoidance, face recognition/gender/dialogue, ring counting, barrel inspection with spill detection, anomaly detection (U-Net tile classifier), QR-code task delegation, inspection report generation (PDF + Markdown), and RViz visualization.

Our megatron package already has solid Task 1 components (face detector, ring detector, perception visualizer, speech, controller) and a working Task 2 launch chain with the arm robot. The gap is significant but the reference code shows proven patterns for every requirement.

---

## 2. Architecture Comparison

### 2.1 Teammate Architecture

```
Perception Layer          Localization Layer       Coordination Layer      Execution Layer
─────────────────        ──────────────────       ──────────────────      ────────────────
detect_people (YOLO) ──→ face_localizator ──→                              Nav2
face_recognizer      ──→ (clusters/dedups)                                   (all nodes use
                          ↓                                                  NavigateToPose
detect_rings_v2      ──→ ring_localizator  ──→  behavior_manager ──→         action client)
                          ↓                      (1319 lines)
cylinder_segmentation  → cylinder_localizator   Central state machine:     blue_line_explorer
(C++, dis_tutorial5)      ↓                     PATROL → APPROACH →          (direct cmd_vel)
                          ↓                     INTERACT → WORKSTATION       yellow_line_avoider
line_localizator     ──→ workstation_recorder   → BLUE_LINE → DONE           (races Nav2)
                          ↓
                          ↓                     orchestrator
station_inspector    ──→ tile_detect             (300 lines)
                     ──→ tile_classifier        Post-patrol anomaly flow:
                     ──→ report                 launches station_inspector
                                                as subprocess per color
barrel_inspector     ──→ pointcloud_viewer
(spill via Trigger)
                                                qr_reader
                                                Dual-engine QR → task dispatch
```

**Key design patterns:**

- **Detect → Localize → Confirm pipeline**: Raw detections are clustered in map frame, confirmed after N observations, then published as persistent markers. This prevents false positives and jitter.
- **Central behavior_manager (1319 lines)**: Main state machine orchestrating everything. Holds a task queue, decides what to approach next, manages patrol/detection balance.
- **Target deduplication**: Processed targets are tracked in a `handled` set so the same face/barrel isn't approached twice.
- **Deferred barrel handling**: Barrels detected during patrol are queued and processed when the patrol group ends (avoids preempting waypoint sequences).
- **Multiple nodes share Nav2 action server**: No mutex — each node checks its own state before sending goals. Works because states are mutually exclusive.
- **Last-write-wins for yellow-line override**: yellow_line_avoider publishes to BOTH `/cmd_vel_unstamped` (overrides keyboard/other nodes) AND `/cmd_vel` (TwistStamped, overrides Nav2) at 50 Hz during emergency back-off.
- **Subprocess pattern for anomaly inspection**: `orchestrator` launches `station_inspector` as a subprocess with specific parameters, avoiding keeping heavy ML nodes alive when not needed.

### 2.2 Megatron Architecture (for comparison)

```
Perception Layer          Controller              Execution
─────────────────        ──────────────────       ──────────
face_detector (YOLOv8) ──→                       Nav2
ring_detector (ellipse)──→ mission_controller     (NavigateToPose,
                          (1127 lines)            Spin, Undock)
perception_visualizer     WAITING → EXPLORING →
                          APPROACHING_OBJECT →
                          VERIFYING → DONE
                          Only handles faces + rings

speech (espeak-ng)        Waypoint-based patrol
                          Detection queue
                          Approach candidate fanning (8 angles)

                          arm_mover (launched but unused by controller)
```

**What megatron already has that the teammate does well too:**

- `IncrementalTrackManager` (perception_utils.py:375) — reusable tracking/dedup/confirmation class
- `extract_3d_points_from_pc2()` — organized PC2 extraction
- `compute_robust_surface()` — SVD-based plane fitting for surface normals
- `normal_to_quaternion()` — orientation encoding
- Approach candidate fanning with costmap guards
- Speech via espeak-ng (non-blocking)
- Working Task 2 launch chain with arm robot

**What megatron lacks vs teammate:**

- No clustering/localization layer (raw detections → confirmed markers)
- No face recognition or gender detection
- No barrel/cylinder detection
- No line detection (yellow or blue)
- No anomaly detection pipeline
- No dialogue / ASR
- No report generation
- No orchestrator / task dispatch from person interaction
- No QR code fallback
- Controller only handles faces + rings (no barrels, no anomaly, no two-room split)

---

## 3. How Each Task 2 Requirement Is Solved

### 3.1 Blue Line Following ✅ (Teammate Complete)

**Implementation:** `blue_line_explorer.py` (529 lines)

| Aspect                | Solution                                                                                    |
| --------------------- | ------------------------------------------------------------------------------------------- |
| Camera                | `/top_camera` (downward arm camera)                                                         |
| Color detection       | Dual mask: HSL ([82-102, ≥120, ≥60]) OR cyan dominance (B,G ≥ 80, B,G ≥ R+40)               |
| Steering              | Centroid-based P-controller with EMA smoothing (α=0.35)                                     |
| Intersection handling | 3 horizontal band ROIs (left, straight, right). Split-mode state machine prefers left turn. |
| Obstacle avoidance    | LIDAR: slow at 0.55m, U-turn at 0.37m                                                       |
| Lost-line recovery    | U-turn after 2s timeout or bump sensor trigger                                              |
| State machine         | SEARCH → FOLLOW → UTURN → SEARCH                                                            |
| Nav2                  | NOT used — direct cmd_vel only                                                              |
| Integration           | behavior_manager enables via `/blue_line_enabled` after final waypoint                      |

**Key parameters:** `kp_steer=1.2`, `max_angular_speed=0.7`, `left_bias=0.25`, `split_exit_hold=0.5s`, `line_lost_timeout=2.0s`

### 3.2 Yellow Line Avoidance ✅ (Teammate Complete)

**Implementation:** `yellow_line_avoider.py` (273 lines)

| Aspect             | Solution                                                                                                                         |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------- |
| Camera             | Downward OAK-D RGB (bottom 65-95% of frame, middle 30-70% horizontal ROI)                                                        |
| Color detection    | HSL mask: H [18-35], S [100-255], V [80-255]                                                                                     |
| Trigger            | ≥300 yellow pixels in danger zone                                                                                                |
| Response           | Immediate stop → 1.8s reverse at 0.12 m/s                                                                                        |
| Override strategy  | Publishes to BOTH `/cmd_vel_unstamped` AND `/cmd_vel` (TwistStamped) at 50 Hz during backing to win last-write-wins against Nav2 |
| Speech             | "Prohibited" on detection                                                                                                        |
| Active only during | PATROL state (disabled during blue-line, workstation approach)                                                                   |

**Key design insight:** Camera-only (no costmap editing). The high-frequency dual-topic override reliably beats Nav2's `/cmd_vel` output without modifying the navigation stack.

### 3.3 Face Detection + Recognition + Approach ✅ (Teammate Complete)

**Pipeline:**

```
detect_people.py (YOLO, dis_tutorial3)
    → /people_marker (3D position)
        ↓
face_recognizer.py (2 Hz)
    → dlib face_recognition against config/personnel/ photos
    → Caffe gender net (Levi & Hassner model)
    → /recognized_person (JSON: name, role, gender, map coords)
        ↓
face_localizator.py
    → Clusters detections (0.6m radius)
    → Confirms after 5 detections
    → /detected_face_locations (Marker with name label)
        ↓
behavior_manager
    → Computes 0.35m approach point in front of face
    → Navigates via Nav2
    → Speaks greeting
```

**Personnel format:** PNG files in `config/personnel/` named `firstname_pronoun1_pronoun2_job_title.png`

### 3.4 Dialogue / Task Delegation ✅ (Teammate Complete via QR)

The teammate uses **QR codes** as the dialogue shortcut (allowed by spec: "The robot can skip the dialogue with a person and can read the task requested in the QR code next to the person"). Actual ASR dialogue is NOT implemented.

**Implementation:** `qr_reader.py` (400 lines)

- Dual-engine: WeChatQR (primary) + OpenCV (fallback)
- Context-aware: face mode → task QR ("defects red", "defects green", "rings", "barrels"); blue_line mode → "report" QR
- Publishes to `/qr` topic consumed by behavior_manager and orchestrator
- Generates report on demand in blue_line mode

**Task delegation flow:**

```
QR scanned → /qr → behavior_manager learns task
                └→ orchestrator (if defects)
                     → saves waypoint color
                     → after patrol, launches station_inspector for that color
```

### 3.5 Ring Detection + Counting ✅ (Teammate Complete)

**Implementation:** `detect_rings_v2.py` (670 lines) + `ring_localizator.py` (368 lines)

| Aspect               | Solution                                                      |
| -------------------- | ------------------------------------------------------------- |
| Detection            | Hough circle on disparity from OAK-D depth                    |
| Filtering            | Combined mask: dilated circle + disparity mask + color filter |
| Color classification | HSV on masked ring pixels (rejects grey holder material)      |
| Localization         | Clusters in map frame (1.0m radius), 6 detections to confirm  |
| Ring approach        | Intentionally disabled — rings are detection/counting only    |

### 3.6 Barrel Inspection + Spill Detection ✅ (Teammate Complete)

**Implementation:** `cylinder_localizator.py` (498 lines) + `barrel_inspector.py` (257 lines) + `pointcloud_viewer.py` (222 lines)

| Aspect           | Solution                                                                                                                                              |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| Detection        | `cylinder_segmentation` (C++, dis_tutorial5): PCL RANSAC cylinder fit                                                                                 |
| Localization     | 0.33m cluster radius, 10 detections to confirm                                                                                                        |
| Orientation      | Horizontal vs vertical from point cloud spread along cylinder axis                                                                                    |
| Spill check      | `pointcloud_viewer` provides `/spill_check` Trigger service: analyzes points in Z-slice [0.005m, 0.15m] above ground within 1m. 4000 point threshold. |
| Approach         | 0.5m offset + 0.3m lateral shift (to robot-right)                                                                                                     |
| Report           | `barrel_report.json` with ID, color, orientation, leak flag, map coords                                                                               |
| Vertical barrels | Detection/report only (no approach needed)                                                                                                            |

### 3.7 Anomaly Detection (Tile Defects) ✅ (Teammate Complete)

**4-node pipeline:**

| Node                | Lines | Role                                                           |
| ------------------- | ----- | -------------------------------------------------------------- |
| `station_inspector` | 740   | Nav to workstation, 6-phase fine-positioning, launch sub-nodes |
| `tile_detect`       | 288   | Brightness-triggered, OTSU thresholding, perspective warp      |
| `tile_classifier`   | 153   | PyTorch U-Net (ResNet34 encoder), 0.20 threshold               |
| `report`            | 829   | Collects results, generates PDF + Markdown                     |

**Fine-positioning phases (station_inspector):**

```
Phase 0: Drive forward until LiDAR reads 0.30m from wall
Phase 1: Yaw-align to target (π for red, π/2 for green)
Phase 2: Hough-line tilt correction using arm camera (P-controller, stop at <0.5° tilt)
Phase 3: Reverse until yellow line detected or 0.40m rear obstacle
Phase 4: Forward scan at 0.08 m/s, stop 2s per tile
Phase 5: Escape turn 130° CW, drive forward 4s
```

**ML model:** U-Net with ResNet34 encoder (segmentation_models_pytorch), trained with ClDice loss on crack dataset. Stored at `anomaly_detection/results/unet/best_model.pth` (98 MB).

**⚠️ PROTECTED NODES:** `station_inspector`, `tile_detect`, `tile_classifier` — teammate's AGENTS.md explicitly forbids modifying these. The inspection parts of `report` are also protected.

### 3.8 Inspection Report ✅ (Teammate Complete)

**Implementation:** `report.py` (829 lines)

- PDF + Markdown output using FPDF library
- Sections: Ring Counting, Barrel Inspection, Anomaly Detection
- Includes tables, defect tile images (texture + heatmap overlays)
- Triggered by "report" QR code in blue-line mode
- Auto-numbered: `report00.pdf`, `report01.pdf`, etc.
- Tracks who requested each task via QR context
- Currently only wired with inspection; faces/rings/barrels report structure is prepared but not yet filled

### 3.9 Patrol / Goal-Based Navigation ✅ (Teammate Complete)

**Implementation:** `waypoint_navigator.py` (298 lines)

- Reads `config/waypoints.yaml` (7 waypoint groups with rotation sub-points)
- 360° rotation scan at each group (4 sub-waypoints at 90° intervals)
- Publishes `/patrol_finished` on completion, `/patrol_group_end` after each group
- behavior_manager enables/disables patrol via `/patrol_enabled`
- Resumes from saved pose after each interruption

### 3.10 Visualization ✅ (Teammate Complete)

| Visualization       | Implementation                                                                                                |
| ------------------- | ------------------------------------------------------------------------------------------------------------- |
| RViz markers        | All confirmed detections (faces, rings, barrels) with color-coded spheres + text labels                       |
| Robot state overlay | `robot_state_overlay.py` — 2D text overlay showing current state, color-coded                                 |
| Debug images        | Face recognition (green box + name), cylinder (projected circles + color), blue line (mask + centroid + ROIs) |
| Live camera streams | OAK-D RGB + top_camera, viewable in rqt_image_view                                                            |
| Speech              | `espeak-ng` for greetings, warnings, status announcements                                                     |

---

## 4. What Megatron Already Has (Reusable)

| Component               | File                                | Lines     | Quality      | Reuse strategy                                                                                                      |
| ----------------------- | ----------------------------------- | --------- | ------------ | ------------------------------------------------------------------------------------------------------------------- |
| `face_detector`         | `megatron/face_detector.py`         | 312       | Solid        | Keep. Add face recognition/gender on top.                                                                           |
| `ring_detector`         | `megatron/ring_detector.py`         | 856       | Solid        | Keep. Add purple/orange/brown to COLOR_RANGES. Add ring_localizator (clustering layer).                             |
| `perception_utils`      | `megatron/perception_utils.py`      | 375       | Excellent    | `IncrementalTrackManager`, `extract_3d_points_from_pc2`, `compute_robust_surface` — all reusable for new detectors. |
| `speech`                | `megatron/speech.py`                | 60        | Good         | Reuse for TTS. Need ASR for dialogue (or use QR fallback).                                                          |
| `perception_visualizer` | `megatron/perception_visualizer.py` | 299       | Good         | Extend for barrel markers, anomaly results, dialogue window.                                                        |
| `controller`            | `megatron/controller.py`            | 1127      | Task 1 only  | Needs major rewrite for Task 2. Can salvage approach-candidate logic, costmap guards, Nav2 lifecycle checks.        |
| `sim_arm_nav.launch.py` | `launch/sim_arm_nav.launch.py`      | 121       | Working      | Keep as-is.                                                                                                         |
| `task2.launch.py`       | `launch/task2.launch.py`            | 143       | Working      | Extend with new nodes.                                                                                              |
| `nav2.yaml`             | `config/nav2.yaml`                  | 312       | Working      | May need tuning for inspection zones.                                                                               |
| `task.yaml`             | `waypoints/task.yaml`               | 241       | 24 waypoints | May need restructuring for two-room split.                                                                          |
| `factory.{pgm,yaml}`    | `maps/`                             | —         | Working      | Keep.                                                                                                               |
| `task2_meshes/`         | `worlds/`                           | ~40 files | Working      | Contains damanged/good tile images for training, QR codes, person photos, colored textures.                         |

---

## 5. What Needs to Be Built (Gap Analysis)

### Must Do (from spec)

| Capability            | Status            | Priority | Teammate Reference                           |
| --------------------- | ----------------- | -------- | -------------------------------------------- |
| Goal-based navigation | ✅ Exists         | —        | `waypoint_navigator.py`                      |
| Face detection        | ✅ Exists         | —        | `detect_people.py`                           |
| Ring detection        | ✅ Exists         | —        | `detect_rings_v2.py`                         |
| Cylinder detection    | ❌ Missing        | HIGH     | `cylinder_segmentation` (C++, dis_tutorial5) |
| Color recognition     | ✅ Exists (rings) | MEDIUM   | Extend COLOR_RANGES for purple/orange/brown  |
| Approaching faces     | ✅ Exists         | —        | behavior_manager approach logic              |
| Speech synthesis      | ✅ Exists         | —        | speech.py                                    |

### Should Do (from spec)

| Capability                   | Status     | Priority       | Teammate Reference                                            |
| ---------------------------- | ---------- | -------------- | ------------------------------------------------------------- |
| Face recognition             | ❌ Missing | HIGH           | `face_recognizer.py` — dlib + Caffe gender                    |
| Gender recognition           | ❌ Missing | HIGH           | `face_recognizer.py` — Caffe model                            |
| Autonomous space exploration | ❌ Missing | LOW (optional) | Frontier exploration not implemented                          |
| Not crossing yellow line     | ❌ Missing | HIGH           | `yellow_line_avoider.py` — camera-only, dual-topic override   |
| Follow blue line             | ❌ Missing | HIGH           | `blue_line_explorer.py` — centroid + split-mode state machine |
| Correct cell detection       | ❌ Missing | MEDIUM         | `line_localizator.py` + `workstation_recorder.py`             |
| Tile detection               | ❌ Missing | HIGH           | `tile_detect.py` (protected — need own implementation)        |
| Defect detection             | ❌ Missing | HIGH           | `tile_classifier.py` (protected — need own model)             |
| Dialogue with ASR            | ❌ Missing | MEDIUM         | QR code fallback (`qr_reader.py`); no ASR                     |
| Creating inspection report   | ❌ Missing | MEDIUM         | `report.py`                                                   |

### New Components Needed

| Component                           | Description                                                                                                                                                                            | Complexity                                |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------- |
| **Cylinder/barrel detector**        | New node: PCL RANSAC cylinder fit on OAK-D PC2. Publish PoseStamped with orientation.                                                                                                  | Medium (can adapt dis_tutorial5 C++ node) |
| **Barrel localizer**                | Cluster + confirm cylinder detections. Classify orientation, color.                                                                                                                    | Medium                                    |
| **Line detector**                   | New node: HSV masking on OAK-D RGB. Detect yellow/blue/green/red lines. Publish line segments.                                                                                         | Medium                                    |
| **Blue line follower**              | New node: arm camera → HSV → centroid steering → cmd_vel. Split-mode intersection handling.                                                                                            | High                                      |
| **Yellow line avoider**             | New node: OAK-D danger zone ROI → stop + reverse. Dual-topic override.                                                                                                                 | Medium                                    |
| **Face recognizer**                 | Enhance face_detector: add face_recognition (dlib) + gender (Caffe). Load personnel photos.                                                                                            | Medium                                    |
| **Ring localizer**                  | New node: cluster ring detections in map frame, confirm, color count.                                                                                                                  | Low                                       |
| **Anomaly detector ML**             | Train U-Net (or PCA/AE/VAE) on tile images from task2_meshes/.                                                                                                                         | High (ML work)                            |
| **Tile detector**                   | New node: arm camera → OTSU thresholding → contour → warp → publish tile images.                                                                                                       | Medium                                    |
| **Station inspector**               | New node: navigate to workstation, fine-position (LiDAR wall stop, yaw align, camera tilt correction), scan tiles, escape.                                                             | High                                      |
| **Orchestrator/controller rewrite** | Replace mission_controller with task2-capable state machine: PATROL (room 1) → DETECT → APPROACH → INTERACT (dialogue/QR) → WORKSTATION (anomaly) → BLUE_LINE (room 2) → REPORT (CTO). | High                                      |
| **Report generator**                | New node: collect results, generate PDF/Markdown report.                                                                                                                               | Medium                                    |
| **QR reader**                       | New node: scan QR codes for task delegation (shortcut around ASR).                                                                                                                     | Low-Medium                                |
| **RViz extensions**                 | Add barrel/cylinder/line/anomaly markers, dialogue window, report display.                                                                                                             | Medium                                    |

---

## 6. Reusable Patterns from Teammate (Design Decisions to Adopt)

### 6.1 Detect → Cluster → Confirm Pipeline

Every perception system follows this pattern. Raw detections are noisy; clustering in map frame with a confirmation count produces stable, deduplicated targets.

```
Raw detector → /raw_markers (short lifetime)
    ↓
Localizer node → clusters by distance, counts observations
    ↓
Confirmed → /detected_*_locations (persistent Marker with text label)
    ↓
behavior_manager → queues, approaches, marks as handled
```

### 6.2 IncrementalTrackManager

Our `perception_utils.py:375` already has this. It's better than the teammate's manual clustering — uses inverse-distance-squared weighting for position averaging, has built-in dedup, confirmation counting, and staleness pruning. Use it for ALL new detectors.

### 6.3 Behavior Manager as Central Coordinator

The 1319-line `behavior_manager.py` holds the master state machine. All nodes publish states to `/robot_state` (String, latched). The behavior manager decides:

- When to start/stop patrol
- Which target to approach next
- When to transition between mission phases (PATROL → WORKSTATION → BLUE_LINE)
- What to do when patrol finishes

### 6.4 Subprocess Pattern for Heavy Nodes

`orchestrator.py:300` launches `station_inspector` as a subprocess with:

```python
subprocess.Popen(
    [sys.executable, os.path.join(_lib, "station_inspector"),
     "--ros-args", "-p", f"workstation:={color}",
     "-p", "use_yaml:=false",
     "-p", "use_orchestrator:=true"],
    preexec_fn=os.setpgrp,
)
```

This avoids keeping ML-heavy nodes alive and allows parameterized launches.

### 6.5 Yellow Line Override Strategy

The `yellow_line_avoider.py` publishes to BOTH `/cmd_vel_unstamped` AND `/cmd_vel` (TwistStamped) at 50 Hz during back-off. This race condition is intentional — it guarantees the emergency stop beats Nav2's normal output. No costmap modifications needed.

### 6.6 Blue Line Direct cmd_vel

The blue line follower does NOT use Nav2 at all. It publishes directly to `/cmd_vel_unstamped`. This avoids fighting the navigation stack and gives smooth, responsive line following.

### 6.7 Latched /robot_state Topic

Every node publishes its state to `/robot_state` (String, latched). The overlay displays it. QR reader switches modes based on it. `yellow_line_avoider` checks it to disable during non-PATROL states. This is a simple but powerful integration pattern.

### 6.8 Color in frame_id Hack

Our `ring_detector.py` already encodes color as `"map|{color}"` in PoseStamped's frame_id. The teammate passes data similarly through marker namespaces and text labels. Consider standardizing on a custom message instead (teammate uses `PersonInfo.msg` for faces).

---

## 7. Vendor Tutorials — What to Use Directly

| Package            | Component                                                           | How to use                                                                                                                                                                       |
| ------------------ | ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **dis_tutorial7**  | `arm_mover_actions.py` + `top_camera/*` topics                      | Already launched in task2.launch.py. Publish to `/arm_command` for arm poses. Subscribe to `top_camera/rgb/preview/image_raw` for anomaly inspection.                            |
| **dis_tutorial3**  | `robot_commander.py`, `detect_people.py`, task2 worlds, Nav2 config | `robot_commander.py` can serve as navigation base class. `detect_people.py` for face detection (we have our own, but theirs feeds the face_recognizer). World files for testing. |
| **dis_tutorial5**  | `cylinder_segmentation` (C++), `detect_rings.py`                    | Adapt cylinder_segmentation for barrel detection. Ring detection technique reusable for anomaly detection on circular features.                                                  |
| **dis_tutorial4**  | `transform_point.py`, `map_goals.py`                                | TF2 transform patterns. Map coordinate conversion.                                                                                                                               |
| **ros_navigation** | `nav2_costmap_filters_demo`, `nav2_gradient_costmap_plugin`         | Keep-out zones for inspection areas. Gradient layer for line-following costmap (alternative to direct cmd_vel).                                                                  |

---

## 8. Protected Code (Do Not Modify — from Teammate's AGENTS.md)

The following teammate nodes are explicitly protected and must NOT be modified:

- `station_inspector.py` — central anomaly inspection state machine
- `tile_detect.py` — tile detection on conveyor belt
- `tile_classifier.py` — U-Net defect classifier
- `report.py` — anomaly inspection parts (general report structure may be modified)

If any of these seem wrong, the teammate's instructions say: **contact Tristan**.

---

## 9. Recommended Implementation Order

Based on dependencies and the spec's grading system:

### Phase 1: Infrastructure (foundational)

1. Add `IncrementalTrackManager`-based localizer nodes for rings and faces (cluster + confirm)
2. Create barrel/cylinder detector (adapt dis_tutorial5 C++ node or write Python PCL wrapper)
3. Create barrel localizer (cluster, orientation, color)
4. Create ring localizer (cluster, confirm, count by color)

### Phase 2: Line Detection (enables room splitting)

5. Create line detector (HSV masking for yellow/blue/green/red on OAK-D)
6. Create yellow line avoider (camera-only, dual-topic override)
7. Create blue line follower (arm camera, centroid + split-mode)

### Phase 3: Mission Controller Rewrite

8. Rewrite controller for two-room state machine (or create new task2_controller)
9. Add face recognition + gender (dlib + Caffe model)
10. Add QR code reader (dual-engine) for task delegation shortcut
11. Wire up dialogue flow (QR-based task dispatch, arm movement, speech)

### Phase 4: Anomaly Detection

12. Train U-Net (or simpler PCA/AE) model on tile images from task2_meshes/
13. Create tile detector (arm camera → OTSU → contour → warp)
14. Create station inspector (nav + fine-position + tile scan + escape)
15. Wire orchestrator to launch inspector on QR command

### Phase 5: Reporting & Polish

16. Create report generator (PDF + Markdown)
17. Extend perception_visualizer for all new markers
18. Add RViz configs for barrel/cylinder/line/anomaly visualization
19. End-to-end testing in task2 world variants

---

## 10. File Reference Index

### Teammate Key Files (for reference)

| File                 | Path                                                                  | Lines  |
| -------------------- | --------------------------------------------------------------------- | ------ |
| behavior_manager     | `src/vendor/teammate-project/src/task1/task1/behavior_manager.py`     | 1319   |
| orchestrator         | `src/vendor/teammate-project/src/task1/task1/orchestrator.py`         | 300    |
| blue_line_explorer   | `src/vendor/teammate-project/src/task1/task1/blue_line_explorer.py`   | 529    |
| yellow_line_avoider  | `src/vendor/teammate-project/src/task1/task1/yellow_line_avoider.py`  | 273    |
| face_recognizer      | `src/vendor/teammate-project/src/task1/task1/face_recognizer.py`      | 311    |
| face_localizator     | `src/vendor/teammate-project/src/task1/task1/face_localizator.py`     | 394    |
| detect_rings_v2      | `src/vendor/teammate-project/src/task1/task1/detect_rings_v2.py`      | 670    |
| ring_localizator     | `src/vendor/teammate-project/src/task1/task1/ring_localizator.py`     | 368    |
| cylinder_localizator | `src/vendor/teammate-project/src/task1/task1/cylinder_localizator.py` | 498    |
| station_inspector    | `src/vendor/teammate-project/src/task1/task1/station_inspector.py`    | 740 ⚠️ |
| tile_detect          | `src/vendor/teammate-project/src/task1/task1/tile_detect.py`          | 288 ⚠️ |
| tile_classifier      | `src/vendor/teammate-project/src/task1/task1/tile_classifier.py`      | 153 ⚠️ |
| barrel_inspector     | `src/vendor/teammate-project/src/task1/task1/barrel_inspector.py`     | 257    |
| report               | `src/vendor/teammate-project/src/task1/task1/report.py`               | 829    |
| qr_reader            | `src/vendor/teammate-project/src/task1/task1/qr_reader.py`            | 400    |
| pointcloud_viewer    | `src/vendor/teammate-project/src/task1/task1/pointcloud_viewer.py`    | 222    |
| line_localizator     | `src/vendor/teammate-project/src/task1/task1/line_localizator.py`     | 302    |
| workstation_recorder | `src/vendor/teammate-project/src/task1/task1/workstation_recorder.py` | 284    |
| parallel_align       | `src/vendor/teammate-project/src/task1/task1/parallel_align.py`       | 158    |
| waypoint_navigator   | `src/vendor/teammate-project/src/task1/task1/waypoint_navigator.py`   | 298    |
| robot_state_overlay  | `src/vendor/teammate-project/src/task1/task1/robot_state_overlay.py`  | 250    |
| unet_train           | `src/vendor/teammate-project/src/anomaly_detection/unet_train.py`     | 242    |
| task1.launch.py      | `src/vendor/teammate-project/src/task1/launch/task1.launch.py`        | 206    |
| waypoints.yaml       | `src/vendor/teammate-project/src/task1/config/waypoints.yaml`         | 117    |

### Megatron Key Files (to build upon)

| File                  | Path                                             | Lines |
| --------------------- | ------------------------------------------------ | ----- |
| controller            | `src/megatron/megatron/controller.py`            | 1127  |
| face_detector         | `src/megatron/megatron/face_detector.py`         | 312   |
| ring_detector         | `src/megatron/megatron/ring_detector.py`         | 856   |
| perception_utils      | `src/megatron/megatron/perception_utils.py`      | 375   |
| perception_visualizer | `src/megatron/megatron/perception_visualizer.py` | 299   |
| speech                | `src/megatron/megatron/speech.py`                | 60    |
| task2.launch.py       | `src/megatron/launch/task2.launch.py`            | 143   |
| sim_arm_nav.launch.py | `src/megatron/launch/sim_arm_nav.launch.py`      | 121   |
| nav2.yaml             | `src/megatron/config/nav2.yaml`                  | 312   |
| production.rviz       | `src/megatron/config/production.rviz`            | 950   |
| task.yaml             | `src/megatron/waypoints/task.yaml`               | 241   |

---

## 11. Critical Integration Topics (Teammate)

| Topic                  | Type              | Purpose                                                         |
| ---------------------- | ----------------- | --------------------------------------------------------------- |
| `/robot_state`         | String (latched)  | Global state broadcast. Every node reads/writes this.           |
| `/patrol_enabled`      | Bool              | Start/pause patrol                                              |
| `/patrol_finished`     | Bool              | Patrol all-groups-complete signal                               |
| `/patrol_group_end`    | Empty             | Individual group complete (unlocks deferred barrels)            |
| `/blue_line_enabled`   | Bool              | Enable blue line follower                                       |
| `/yellow_line_enabled` | Bool              | Enable yellow line avoider                                      |
| `/cmd_vel_unstamped`   | Twist             | Direct drive (used by blue_line, yellow_line, behavior_manager) |
| `/cmd_vel`             | TwistStamped      | Nav2 velocity (raced by yellow_line_avoider)                    |
| `/qr`                  | String            | QR decoded text → task dispatch                                 |
| `/arm_command`         | String            | Arm pose commands → arm_mover_actions                           |
| `/inspector_finish`    | String            | Anomaly inspection complete signal                              |
| `/orchestrator_out`    | PoseStamped       | Waypoint from orchestrator to inspector                         |
| `/workstation_markers` | MarkerArray       | Red/green workstation positions                                 |
| `/report_commands`     | String            | "make" / "clear" → report generation                            |
| `/spill_check`         | Trigger (service) | Point cloud spill analysis                                      |
