# Teammate Project — Complete Architecture & Task 2 Analysis

## 1. PACKAGE OVERVIEW

### `/src/task1/` — The main custom implementation package (28 Python modules, ~9,648 lines)

This is where ALL custom mission logic lives. Contains 22 standalone ROS2 nodes.

### `/src/msg_srv/` — Custom message definitions

- **1 custom message**: `PersonInfo.msg` (name, pronouns, role, map_x, map_y, confidence)
- C++ (ament_cmake) package. Depends on `std_msgs`, `rosidl_default_generators`.

### `/src/anomaly_detection/` — ML model for tile defect detection

- `unet_train.py` (242 lines): Trains a U-Net (ResNet34 encoder, `segmentation_models_pytorch`) with ClDice loss on crack dataset. Outputs `best_model.pth`. Uses albumentations, CLAHE normalization.
- `unet_evaluate.py` (200 lines): Sweeps thresholds to find best IoU, applies morphological post-processing (open 3x3 -> close 5x5), saves evaluation images.
- `data/` — training images/masks, `results/unet/` — checkpoint output, `eval_output/` — evaluation results.

### `/src/dis_tutorial3/` — Tutorial package providing `detect_people.py` (YOLO-based person detection)

Used in the main launch to provide face detection markers via `/people_marker`.

### `/src/dis_tutorial5/` — Tutorial package providing `cylinder_segmentation` (barrel point-cloud segmentation)

Provides `/cylinder_markers` for barrel detection.

### `/src/dis_tutorial4/` — Tutorial with helper scripts (not directly used by task1).

### `/src/dis_tutorial7/` — Robotic arm camera package (not directly used by task1).

### `/src/rviz_2d_overlay_plugins/` — Empty; overlay messages come from system-installed `rviz_2d_overlay_msgs`.

---

## 2. ARCHITECTURE OVERVIEW — Nodes and Their Interactions

The system is organized into three mission phases with a central behavior coordinator:

### **Core Coordination Layer**

| Node                    | File                     | Lines | Role                                                                                                                                                                                                                                                                                      |
| ----------------------- | ------------------------ | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **behavior_manager**    | `behavior_manager.py`    | 1319  | Master state machine. Manages robot state transitions (IDLE, PATROL, APPROACH*\*, INTERACT*_, WORKSTATION, BLUE*LINE*_, FINISHING_ROUNDS). Detects targets, queues/activates them, sends Nav2 approach goals, handles post-patrol workstation visit + final waypoint + blue-line handoff. |
| **orchestrator**        | `orchestrator.py`        | 300   | Manages defect inspection workflow post-patrol. Receives `/qr` requests for "defects red/green", receives workstation markers from `/workstation_markers`, launches `station_inspector` as a subprocess for the requested color, orchestrates return navigation.                          |
| **waypoint_navigator**  | `waypoint_navigator.py`  | 298   | Patrol waypoint sequencer. Reads `config/waypoints.yaml` (7 waypoint groups with rotation sub-points). Listens to `/patrol_enabled` from behavior_manager. Publishes `/patrol_finished` and `/patrol_group_end`. Uses Nav2 `NavigateToPose`.                                              |
| **robot_state_overlay** | `robot_state_overlay.py` | 250   | Renders robot state as RViz 2D overlay text. Color-coded per state. Listens to `/robot_state`.                                                                                                                                                                                            |

### **Perception Layer**

| Node                     | File                      | Lines | Role                                                                                                                                                                                                                                                                                                                               |
| ------------------------ | ------------------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **face_localizator**     | `face_localizator.py`     | 394   | Clusters `/people_marker` detections in map frame. Confirms faces after 5 detections. Publishes confirmed face markers on `/detected_face_locations` with face_yaw encoded in orientation. Cross-references with `/recognized_person`.                                                                                             |
| **face_recognizer**      | `face_recognizer.py`      | 311   | Runs `face_recognition` library at 2 Hz on OAK-D images. Loads personnel photos from `config/personnel/`. Gender-cross-validates matches. Publishes recognized person JSON on `/recognized_person`.                                                                                                                                |
| **detect_rings_v2**      | `detect_rings_v2.py`      | 670   | Hough circle detection on disparity map from OAK-D depth. Extracts ring patches, filters by color mask (black, green, blue, red), classifies color by HSV on masked pixels. Publishes raw markers on `/ring_marker` with 200ms lifetime.                                                                                           |
| **ring_localizator**     | `ring_localizator.py`     | 368   | Clusters raw ring detections in map frame (1.0m merge radius). Confirms at 6 detections. Publishes persistent markers on `/detected_ring_locations`. Marks rings as actionable/confirmed.                                                                                                                                          |
| **cylinder_localizator** | `cylinder_localizator.py` | 498   | Clusters barrel detections from `/cylinder_markers` (from `dis_tutorial5`). Confirms at 10 detections, detects vertical/horizontal orientation. Saves JSON report to `barrell_detection/barrel_report.json`. Publishes confirmed markers on `/detected_cylinder_locations`. Labels barrels "horizontal"/"vertical" in marker text. |
| **line_localizator**     | `line_localizator.py`     | 302   | Detects colored floor lines (red, green, blue, yellow) from OAK-D. Uses skeletonization + HoughLinesP + 3D PCA fit. Publishes `/line_markers` (3D LINE_STRIP in camera frame). Flatness filter at ~3cm std dev.                                                                                                                    |
| **qr_reader**            | `qr_reader.py`            | 400   | Dual-engine QR scanning (WeChatQR + OpenCV). Mode-switches based on `/robot_state`: face mode detects defect/ring/barrel/ring QR requests; blue_line mode scans for "report" QR. Generates report on demand; shuts down all non-report nodes after 3s.                                                                             |

### **Navigation & Control Layer**

| Node                     | File                      | Lines | Role                                                                                                                                                                                                                                                                                                                    |
| ------------------------ | ------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **blue_line_explorer**   | `blue_line_explorer.py`   | 529   | Vision-based blue line following via top camera. HSL masks + cyan dominance for blue. Left/straight/right band ROI analysis. Split-mode state machine for intersections (prefers left). LIDAR forward obstacle avoidance with U-turn on obstacle or line loss. Smooths angular via EMA. Publishes `/cmd_vel_unstamped`. |
| **yellow_line_avoider**  | `yellow_line_avoider.py`  | 273   | Camera-only yellow line avoidance. Danger zone at bottom of frame. Immediately stops and backs away when yellow enters. **Overrides BOTH** `/cmd_vel_unstamped` AND `/cmd_vel` (TwistStamped for Nav2) at 50 Hz during backing to win the last-write-wins race.                                                         |
| **parallel_align**       | `parallel_align.py`       | 158   | Hough-line-based parallel alignment using top camera vertical derivatives. Used by station_inspector for fine-positioning at workstation.                                                                                                                                                                               |
| **simple_waypoints_nav** | `simple_waypoints_nav.py` | 135   | Simplified single-shot waypoint navigator (no behavior_manager integration).                                                                                                                                                                                                                                            |

### **Inspection Layer (Anomaly Detection)**

| Node                     | File                      | Lines | Role                                                                                                                                                                                                                                                                                                                                                                                             |
| ------------------------ | ------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **station_inspector**    | `station_inspector.py`    | 740   | ⚠️ PROTECTED NODE (Tristan's code, do NOT modify). Multi-phase state machine: INACTIVE -> NAV_TO_WS -> FINE_POSITION (forward approach -> parallel yaw align -> Hough camera align -> rear backup to yellow line -> forward tile scan with arm look -> escape 130deg turn). Launches `tile_detect` and `tile_classifier` as subprocesses. Integrates with orchestrator for waypoint acquisition. |
| **tile_detect**          | `tile_detect.py`          | 288   | ⚠️ PROTECTED. Brightness-triggered tile detection from top camera. OTSU thresholding, contour detection, warp perspective to fronto-parallel view. Publishes warped tiles on `/tile_warped`, tile status on `/tile_status` (TILE_FOUND/TILE_LEFT).                                                                                                                                               |
| **tile_classifier**      | `tile_classifier.py`      | 153   | ⚠️ PROTECTED. Loads U-Net model from `anomaly_detection/results/unet/best_model.pth`. Classifies warped tiles as DEFECT/OK. Threshold=0.20, min_defect_ratio=0.002. Publishes `/tile_classification` and `/tile_heatmap`.                                                                                                                                                                        |
| **barrel_inspector**     | `barrel_inspector.py`     | 257   | Listens to `/detected_cylinder_locations`. For horizontal barrels, waits for spill check trigger from behavior_manager. Calls `/spill_check` service (pointcloud_viewer). Publishes results on `/barrel_inspection_result`. Speaks barrel findings.                                                                                                                                              |
| **pointcloud_viewer**    | `pointcloud_viewer.py`    | 222   | Provides `/spill_check` Trigger service. Analyzes point cloud in Z-slice [0.005m, 0.15m] above ground within 1m. 4000 points threshold for spill detection.                                                                                                                                                                                                                                      |
| **workstation_recorder** | `workstation_recorder.py` | 284   | Aggregates line detections from `/line_markers` to compute red/green workstation approach positions. Requires 10 confirmations. Publishes `/workstation_markers` MarkerArray. Optionally writes YAML.                                                                                                                                                                                            |

### **Reporting Layer**

| Node       | File        | Lines | Role                                                                                                                                                                                                                                              |
| ---------- | ----------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **report** | `report.py` | 829   | Generates PDF + Markdown inspection reports using FPDF. Collects ring counts, barrel data (with leak results), and tile defect results per station. Handles image embedding for defect visuals. Responds to `/report_commands` ("make", "clear"). |

### **Utility / Viewer Nodes**

| Node                    | File                     | Lines |                           |
| ----------------------- | ------------------------ | ----- | ------------------------- |
| **arm_camera_viewer**   | `arm_camera_viewer.py`   | 49    |                           |
| **oakd_camera_viewer**  | `oakd_camera_viewer.py`  | 49    |                           |
| **cylinder_debug_view** | `cylinder_debug_view.py` | 320   |                           |
| **color_mask_viewer**   | `color_mask_viewer.py`   | 107   |                           |
| **path_listener**       | `path_listener.py`       | 145   | (not in launch - utility) |

---

## 3. HOW EACH TASK 2 REQUIREMENT IS SOLVED

### 3.1 Blue Line Following

**Solved** via `blue_line_explorer.py`. Vision-based using top camera:

- HSL blue mask + cyan dominance detection (B and G both high vs R)
- Three horizontal band ROIs: left (bottom 0.42-0.90), straight (0.20-0.65), right (0.42-0.90)
- **Split-mode state machine**: When left/right branch fires, enters split mode. In split mode: if left branch visible, steers left with bias; re-centers when only "straight" lit for 0.5s
- LIDAR forward obstacle with two thresholds: slow at 0.55m, U-turn at 0.37m
- U-turn after 2s line loss or obstacle — rotates 180° at 0.5 rad/s for 6.5s
- EMA angular smoothing (alpha=0.35) to prevent jerks
- **Integration**: behavior_manager enables it via `/blue_line_enabled` Bool after final waypoint

### 3.2 Yellow Line Avoidance

**Solved** via `yellow_line_avoider.py`. Camera-only, NO costmap editing:

- Danger zone ROI: 0.65-0.95 fraction of bottom frame, middle 30-70% horizontally
- Yellow HSL mask: H [18-35], S [100-255], V [80-255]
- 300 pixel threshold in danger zone triggers immediate STOP then 1.8s reverse at 0.12 m/s
- **Critical design**: During backing, publishes to BOTH `/cmd_vel_unstamped` (overrides keyboard/other nodes) AND `/cmd_vel` (TwistStamped, overrides Nav2 collision_monitor) at 50 Hz to win the last-write-wins race
- Speaks "Prohibited" on detection
- Only active during PATROL state; disabled during blue-line following, workstation approach

### 3.3 Anomaly Detection (Tile Defect)

**Solved** via 4-node pipeline:

1. **`station_inspector`**: Navigates to workstation, fine-positions (forward to wall, parallel yaw, Hough camera align, back to yellow line), scans tiles on conveyor belt
2. **`tile_detect`**: Brightness-triggered (small ROI at image center-left), OTSU thresholding, contour detection with aspect ratio filter (>2.0 rejected), perspective warp for each tile
3. **`tile_classifier`**: U-Net (ResNet34 encoder) trained on crack dataset. CLAHE normalization, classify as DEFECT/OK at threshold 0.20, min_defect_ratio 0.002
4. **`report`**: Collects tile classification results per station, saves texture + heatmap images for defects, includes in PDF

### 3.4 Person Recognition / Dialogue

**Solved** via 3-node pipeline:

1. **`face_recognizer`**: `face_recognition` library + Caffe gender net. Personnel loaded from `config/personnel/` (PNG files named `firstname_he_him_job_title.png`). Gender cross-validation (stricter 0.45 threshold for cross-gender matches). Publishes JSON to `/recognized_person`.
2. **`face_localizator`**: 0.6m cluster radius, 5 detections to confirm. Computes face_yaw from robot->face direction. Publishes on `/detected_face_locations` with ns="face_confirmed".
3. **`behavior_manager`**: On face detection during PATROL or BLUE_LINE states, pauses patrol, computes face approach point (0.35m in front), navigates to approach, then enters INTERACT_FACE. Speaks a random line from face_lines list (7 options). Uses tts (`espeak-ng`/`espeak`/`spd-say`).

### 3.5 Barrel Inspection

**Solved** via 3-node pipeline:

1. **`cylinder_localizator`**: Confirms barrels from `/cylinder_markers` (0.33m cluster radius, 10 detections). Detects orientation (horizontal/vertical). Saves to `barrel_report.json`.
2. **`behavior_manager`**: For horizontal barrels only (vertical = detection/report only). Computes barrel approach point with 0.5m offset and 0.3m lateral shift (to robot-right). Triggers spill check via `/spill_check_trigger` during INTERACT_BARREL.
3. **`barrel_inspector`**: Receives spill check trigger, calls `/spill_check` service (pointcloud_viewer), publishes `/barrel_inspection_result` JSON, speaks barrel color + orientation + leak status.

### 3.6 Ring Counting

**Solved** via 2-node pipeline:

1. **`detect_rings_v2`**: Hough circle detection on disparity from OAK-D depth. Circles filtered by combined ring mask (dilated circle + disparity + color filter). Color classified via HSV on masked pixels. Publishes on `/ring_marker` with 200ms lifetime.
2. **`ring_localizator`**: Clusters raw markers in map frame. 1.0m merge radius, 6 detections to confirm. Publishes persistent markers on `/detected_ring_locations`. `behavior_manager` ring*callback is intentionally **empty** — *"Rings no longer require approach or interaction — detection only"\_ (line 990-991).

### 3.7 Report Generation

**Solved** via `report.py`:

- PDF + Markdown output using FPDF library
- Sections: Ring Counting, Barrel Inspection, Anomaly Detection
- Includes tables, defect tile images (texture + heatmap)
- Triggered by "report" QR code (via `/report_commands`) or `/qr` reader in blue-line mode
- Tracks who requested each task via QR context ("approaching face" -> person name -> task tracking)
- Report numbering: `report00.pdf`, `report01.pdf`, etc. Auto-resumes from last index.

---

## 4. KEY DESIGN PATTERNS AND STATE MACHINES

### behavior_manager State Machine

```
IDLE -> PATROL -> [detection interrupts]:
  -> APPROACH_FACE -> INTERACT_FACE (2.5s) -> back to PATROL/IDLE
  -> APPROACH_RING -> INTERACT_RING (2.5s) -> back (functionally deprecated)
  -> APPROACH_BARREL -> INTERACT_BARREL (10s, triggers spill check) -> back
Post-patrol:
  -> APPROACH_WORKSTATION -> WORKSTATION (waits for /inspector_finish)
  -> FINISHING_ROUNDS -> FOLLOW_BLUE_LINE
```

### Detection Flow

1. Perception nodes publish raw markers
2. Localizator nodes cluster and confirm
3. Confirmed markers published on `/detected_*_locations`
4. `behavior_manager` subscribes to these, queues targets, deduplicates against handled list
5. Deferred barrel targets unlocked when patrol group ends (`/patrol_group_end`)

### Nav2 Integration

- All nodes use Nav2 `navigate_to_pose` action server directly via ActionClient
- `behavior_manager` sends approach goals for faces/rings/barrels (temporary goals)
- `waypoint_navigator` sends patrol waypoints from YAML
- `orchestrator` + `station_inspector` send their own Nav2 goals
- Multiple nodes share the same action server — **no centralized navigation coordinator**; each node sends and manages its own goals
- `behavior_manager` cancels active goals when patrol is paused or new targets appear
- `yellow_line_avoider` races against Nav2's `/cmd_vel` output at higher frequency

### blue_line_explorer Split-Mode State Machine

```
SEARCH (rotating slowly until line found) ->
FOLLOW (normal following) ->
  on branch detection -> split_active=True
  on left branch -> steer left with bias, reset exit timer
  on right-only -> clamp steer to left
  on "straight" lit + centroid <0.30 for 0.5s -> split_active=False
UTURN (obstacle/line loss) -> SEARCH
```

### station_inspector Fine-Positioning Phases

```
phase 0: Drive forward until 0.30m from wall
phase 1: Rotate to target yaw (pi for red, pi/2 for green)
phase 2: Hough line alignment (stop when tilt < 0.5deg)
phase 3: Back up until yellow line or 0.40m rear obstacle
phase 4: Forward scan (0.08 m/s), stop 2s per tile, detect end of station by color absence
phase 5: Escape turn 130deg CW, drive forward 4s
```

---

## 5. INTEGRATION POINTS: TOPICS, SERVICES, ACTIONS

### Key Topics (Pub/Sub)

| Topic                          | Type            | Publisher(s)                                                                 | Subscriber(s)                              | Purpose                              |
| ------------------------------ | --------------- | ---------------------------------------------------------------------------- | ------------------------------------------ | ------------------------------------ |
| `/robot_state`                 | `String`        | behavior_manager, blue_line_explorer, station_inspector                      | Most nodes                                 | Global state broadcast (latched)     |
| `/patrol_enabled`              | `Bool`          | behavior_manager                                                             | waypoint_navigator                         | Start/pause patrol                   |
| `/patrol_finished`             | `Bool`          | waypoint_navigator                                                           | behavior_manager, orchestrator             | Patrol complete signal               |
| `/patrol_group_end`            | `Empty`         | waypoint_navigator                                                           | behavior_manager                           | Group rotation complete              |
| `/patrol_command`              | `Bool`          | External/QrReader                                                            | behavior_manager                           | Request patrol start/stop            |
| `/detected_face_locations`     | `Marker`        | face_localizator                                                             | behavior_manager                           | Confirmed face markers               |
| `/detected_ring_locations`     | `Marker`        | ring_localizator                                                             | behavior_manager, report                   | Confirmed ring markers               |
| `/detected_cylinder_locations` | `Marker`        | cylinder_localizator                                                         | behavior_manager, barrel_inspector, report | Barrels                              |
| `/ring_marker`                 | `Marker`        | detect_rings_v2                                                              | ring_localizator                           | Raw ring detections                  |
| `/ring_colour`                 | `String`        | detect_rings_v2                                                              | ring_localizator                           | Ring color strings                   |
| `/people_marker`               | `Marker`        | detect_people (tutorial3)                                                    | face_recognizer, face_localizator          | Face detections                      |
| `/recognized_person`           | `String` (JSON) | face_recognizer                                                              | face_localizator, qr_reader, report        | Recognition results                  |
| `/qr`                          | `String`        | qr_reader                                                                    | behavior_manager, orchestrator, report     | QR decoded data                      |
| `/blue_line_enabled`           | `Bool`          | behavior_manager                                                             | blue_line_explorer                         | Enable/disable blue line following   |
| `/yellow_line_enabled`         | `Bool`          | behavior_manager                                                             | yellow_line_avoider                        | Enable/disable yellow line avoidance |
| `/cmd_vel_unstamped`           | `Twist`         | behavior_manager, blue_line_explorer, yellow_line_avoider, station_inspector | iRobot Create3 motion_control              | Direct drive commands                |
| `/cmd_vel`                     | `TwistStamped`  | yellow_line_avoider                                                          | Nav2 collision_monitor                     | Override Nav2 output                 |
| `/arm_command`                 | `String`        | station_inspector                                                            | arm node                                   | Robotic arm control                  |
| `/tile_warped`                 | `Image`         | tile_detect                                                                  | tile_classifier, report                    | Warped tile images                   |
| `/tile_status`                 | `String`        | tile_detect                                                                  | station_inspector, report                  | TILE_FOUND/TILE_LEFT                 |
| `/tile_classification`         | `String`        | tile_classifier                                                              | report                                     | DEFECT/OK results                    |
| `/spill_check_trigger`         | `String`        | behavior_manager                                                             | barrel_inspector                           | Request spill check                  |
| `/barrel_inspection_result`    | `String` (JSON) | barrel_inspector                                                             | report                                     | Barrel leak results                  |
| `/inspector_finish`            | `String`        | station_inspector                                                            | behavior_manager                           | Inspector done signal                |
| `/inspector_phase`             | `Int32`         | station_inspector                                                            | tile_detect                                | Phase synchronization                |
| `/inspector_station`           | `String`        | station_inspector                                                            | report                                     | Current station name                 |
| `/orchestrator_out`            | `PoseStamped`   | orchestrator                                                                 | station_inspector                          | Waypoint from orchestrator           |
| `/orchestrator_in`             | `String`        | station_inspector                                                            | orchestrator                               | Waypoint request                     |
| `/workstation_markers`         | `MarkerArray`   | workstation_recorder                                                         | orchestrator, behavior_manager             | Workstation positions                |
| `/workstation_done`            | `Empty`         | orchestrator                                                                 | behavior_manager                           | Workstation inspection done          |
| `/line_markers`                | `Marker`        | line_localizator                                                             | workstation_recorder                       | Colored floor lines                  |
| `/report_commands`             | `String`        | qr_reader                                                                    | report                                     | "make"/"clear" commands              |
| `/target_done`                 | `Empty`         | behavior_manager                                                             | behavior_manager                           | Current target complete              |
| `/resume_patrol`               | `Empty`         | Not used externally                                                          | behavior_manager                           | Resume patrol                        |

### Key Services

| Service        | Type      | Server            | Client           | Purpose                    |
| -------------- | --------- | ----------------- | ---------------- | -------------------------- |
| `/spill_check` | `Trigger` | pointcloud_viewer | barrel_inspector | Point cloud spill analysis |

### Key Actions

| Action              | Type                    | Client(s)                                                             | Purpose        |
| ------------------- | ----------------------- | --------------------------------------------------------------------- | -------------- |
| `/navigate_to_pose` | `NavigateToPose` (Nav2) | behavior_manager, waypoint_navigator, orchestrator, station_inspector | All navigation |

---

## 6. WHAT'S FINISHED VS TODO/NOT IMPLEMENTED

### Finished

- **Blue line following**: Complete with LIDAR integration, split-mode state machine, U-turn recovery
- **Yellow line avoidance**: Complete with dual-topic override strategy
- **Ring detection**: Hough circle + color classification, confirmed at 6 detections
- **Barrel detection + inspection**: Full pipeline including horizontal spill check
- **Face detection + recognition**: 5-detection confirmation, gender-aware matching, approach behavior
- **Anomaly detection**: U-Net tile classifier with warp + heatmap pipeline
- **Patrol**: 7-waypoint patrol with multi-angle rotation scans
- **Report**: Full PDF + Markdown generation with images
- **QR reading**: Dual-engine (WeChatQR + OpenCV), context-aware modes
- **Workstation recording**: Red/green workstation approach position computation
- **Orchestrator**: Post-patrol defect flow coordination

### Notable Design Notes / Limitations

- **Ring approach is intentionally disabled**: `ring_callback` in behavior*manager is a no-op — *"Rings no longer require approach or interaction — detection only"\_ (line 990)
- **Vertical barrels** are detection/report only (no approach needed)
- **Face approach during BARREL interaction**: blocked (`accepts_face_detections()` returns False)
- **Multiple nodes share Nav2 action server** — no mutex; relies on each node checking its own state before sending goals
- **blue_line_explorer** does NOT use Nav2 at all — direct `cmd_vel` only
- **yellow_line_avoider** races against Nav2 at 50 Hz
- **station_inspector** and tensor_detect **MUST NOT be modified** per AGENTS.md warning
- **Report node** anomaly inspection parts are protected; general structure may still be modified
- **No TODO markers** found in any source files — the codebase is production-ready for their competition
- **person_data** from `face_localizator`: sends the face toward-robot yaw to enable head-on approach — uses orientation quaternion encoding in markers
- Ring detection uses **masked color extraction** to filter out the grey holder material

---

## 7. FILE PATHS AND LINE COUNTS (Key Nodes)

| File                              | Absolute Path                                                                               | Lines              |
| --------------------------------- | ------------------------------------------------------------------------------------------- | ------------------ |
| orchestrator                      | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/orchestrator.py`         | 300                |
| behavior_manager                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/behavior_manager.py`     | 1319               |
| waypoint_navigator                | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/waypoint_navigator.py`   | 298                |
| blue_line_explorer                | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/blue_line_explorer.py`   | 529                |
| yellow_line_avoider               | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/yellow_line_avoider.py`  | 273                |
| face_recognizer                   | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/face_recognizer.py`      | 311                |
| face_localizator                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/face_localizator.py`     | 394                |
| detect_rings_v2                   | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/detect_rings_v2.py`      | 670                |
| ring_localizator                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/ring_localizator.py`     | 368                |
| cylinder_localizator              | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/cylinder_localizator.py` | 498                |
| station_inspector                 | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/station_inspector.py`    | 740                |
| tile_detect                       | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/tile_detect.py`          | 288                |
| tile_classifier                   | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/tile_classifier.py`      | 153                |
| barrel_inspector                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/barrel_inspector.py`     | 257                |
| report                            | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/report.py`               | 829                |
| qr_reader                         | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/qr_reader.py`            | 400                |
| workstation_recorder              | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/workstation_recorder.py` | 284                |
| line_localizator                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/line_localizator.py`     | 302                |
| pointcloud_viewer                 | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/pointcloud_viewer.py`    | 222                |
| parallel_align                    | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/parallel_align.py`       | 158                |
| robot_state_overlay               | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/task1/robot_state_overlay.py`  | 250                |
| PersonInfo.msg                    | `/home/luka/coding/dis/src/vendor/teammate-project/src/msg_srv/msg/PersonInfo.msg`          | 7                  |
| unet_train.py                     | `/home/luka/coding/dis/src/vendor/teammate-project/src/anomaly_detection/unet_train.py`     | 242                |
| unet_evaluate.py                  | `/home/luka/coding/dis/src/vendor/teammate-project/src/anomaly_detection/unet_evaluate.py`  | 200                |
| waypoints.yaml                    | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/config/waypoints.yaml`         | 117                |
| task1.launch.py                   | `/home/luka/coding/dis/src/vendor/teammate-project/src/task1/launch/task1.launch.py`        | 206                |
| workstation locations             | `/home/luka/coding/dis/src/vendor/teammate-project/test_workstation_locations.yaml`         | (external to src/) |
| **TOTAL lines** (28 Python files) |                                                                                             | **9,648**          |
