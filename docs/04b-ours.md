## MEGATRON PACKAGE — COMPREHENSIVE SUMMARY

---

### 1. NODES: WHAT EXISTS AND WHAT THEY DO

There are **5 Python source files** in `megatron/megatron/`, 4 of which are registered as ROS 2 nodes. The 5th (`perception_utils.py` and `speech.py`) are shared utility modules.

| File                       | Lines    | Node Name               | Executable              | Purpose                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| -------------------------- | -------- | ----------------------- | ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `controller.py`            | **1127** | `mission_controller`    | `controller`            | Central state-machine mission controller. Loads waypoints from YAML, orchestrates Nav2 navigation, listens for detection callbacks, manages approach/verify behavior, controls speech output.                                                                                                                                                                                                                                                                                                                                                                                |
| `face_detector.py`         | **312**  | `face_detector`         | `face_detector`         | YOLOv8-based face-poster detector. Uses synced RGB + PointCloud2 via `message_filters`, projects face ROIs to 3D, fits surface normals via SVD (from `perception_utils`), uses `IncrementalTrackManager` for tracking/dedup, publishes `PoseStamped` on `/detected_faces`.                                                                                                                                                                                                                                                                                                   |
| `ring_detector.py`         | **856**  | `ring_detector`         | `ring_detector`         | Concentric ellipse-pair ring detector. Four-stage pipeline: (1) adaptive threshold + saturation mask, (2) contour extraction & ellipse fitting, (3) concentric pair matching with scoring, (4) color classification (red/green/blue/yellow/black via HSV), depth-discontinuity check (rejects wall-mounted rings), 3D projection via organized PointCloud2. Publishes `PoseStamped` on `/detected_rings` with color packed in `frame_id` as `"map&#124;{color}"`. Also publishes 4 debug image topics (`/ring_debug/binary`, `/ellipses`, `/pairs`, `/color`). |
| `perception_visualizer.py` | **299**  | `perception_visualizer` | `perception_visualizer` | Demo-friendly composed visualizer. Subscribes to face/ring detection images, `/mission_status`, ring debug images (4 stages), counts unique detections by position. Composes a stacked image canvas (header + side-by-side face/ring row + 4 debug panels) and publishes to `/task1_visualization_image` and `/task1_rviz_image`. Optional OpenCV window.                                                                                                                                                                                                                    |
| `speech.py`                | **60**   | N/A (utility)           | —                       | Lightweight non-blocking TTS via `espeak-ng`. `Speaker` class with `speak()`, `is_busy()`, `set_node_logger()` methods. Uses `subprocess.Popen` for fire-and-forget speech.                                                                                                                                                                                                                                                                                                                                                                                                  |
| `perception_utils.py`      | **375**  | N/A (utility)           | —                       | Shared perception utilities: `DepthCameraGeometry` (pinhole projection, **not currently used** since nodes use PC2 directly), `extract_3d_points_from_pc2()` (organized PointCloud2 → 3D points), `compute_robust_surface()` (SVD-based plane fitting, median depth outlier removal), `transform_point_and_normal()` (TF2 wrapper), `normal_to_quaternion()` (2D normal → orientation quaternion), `IncrementalTrackManager` (distance-weighted multi-observation tracker with inverse-distance-squared weighting, deduplication, confirmation counting, staleness pruning). |
| `__init__.py`              | **0**    | —                       | —                       | Empty package init file.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

---

### 2. LAUNCH CHAIN FOR TASK 2

`task2.launch.py` (143 lines) is the entry point:

```
ros2 launch megatron task2.launch.py
```

**Launch arguments:**

- `world` (default: `task2`) — Gazebo world name
- `map` (default: `maps/factory.yaml`) — Nav2 map
- `launch_rviz` (default: `true`)
- `visualization` (default: `true`) — perception visualizer
- `show_debug_window` (default: `false`) — OpenCV popup
- `use_sim_time` (default: `true`)
- `rviz_config` (default: `config/production.rviz`)
- `nav2_config` (default: `config/nav2.yaml`)

**Chain of included/sub-launched components:**

```
task2.launch.py
├── sim_arm_nav.launch.py              [megatron]
│   ├── dis_tutorial3 sim.launch.py    (Gazebo + task2 world)
│   ├── dis_tutorial7 turtlebot4_spawn (arm-enabled robot + top_camera bridge)
│   ├── dis_tutorial3 localization     (AMCL + factory map)
│   └── dis_tutorial3 nav2             (custom nav2.yaml config)
├── rviz2                              (only if launch_rviz=true)
├── face_detector                      (megatron node)
├── ring_detector                      (megatron node)
├── arm_mover                          (dis_tutorial7, arm_mover_actions.py)
├── mission_controller                 (megatron, with waypoints=task.yaml, total_faces=3, total_rings=4)
└── perception_visualizer              (megatron, only if visualization=true)
```

Key differences from Task 1:

- Uses `sim_arm_nav.launch.py` instead of `sim_turtlebot_nav.launch.py` — the former spawns the **arm-enabled robot** from `dis_tutorial7` instead of the standard `dis_tutorial3` spawn.
- Adds an explicit `arm_mover` node from `dis_tutorial7` (executable `arm_mover_actions.py`).
- Uses `maps/factory.yaml` and `waypoints/task.yaml`.

**Contrast with Task 1 (`task1.launch.py`):**

- Uses `sim_turtlebot_nav.launch.py` (standard TurtleBot4 without arm).
- No `arm_mover` node.
- Uses `maps/task1_orig.yaml`, `total_rings=2` (instead of 4).

---

### 3. WHAT'S REUSABLE FROM TASK 1 FOR TASK 2

| Component                      | Reusable? | Notes                                                                                                                                                                                                                                                                                                                                                                                                       |
| ------------------------------ | --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`controller.py`**            | PARTIALLY | The state machine (WAITING→UNDOCKING→EXPLORING→APPROACHING→VERIFYING→DONE) handles face/ring detection approach-and-greet. Task 2 adds: barrel inspection, cylinder counting, anomaly detection, dialogue, line detection/following, **AND a completely different navigation strategy** (blue-line following in room 2, forbidden yellow zones). The controller needs significant extension or replacement. |
| **`face_detector.py`**         | YES       | Needs enhancement: face **recognition** (name, gender). Currently only detects faces as "posters" — needs to classify identity/gender from given photo set. The 3D projection, SVD surface fitting, and tracking infrastructure are fully reusable.                                                                                                                                                         |
| **`ring_detector.py`**         | YES       | Fully reusable for ring detection. Color classification already covers red/green/blue/yellow/black. Note: Task 2 specifies colors "red, green, blue, yellow, purple, orange, brown" — purple/orange/brown need to be added to COLOR_RANGES. The depth-discontinuity check already rejects wall-mounted 2D circles (Task 2 spec: "detect all 3D rings, discard 2D printed circles").                         |
| **`perception_visualizer.py`** | YES       | Reusable with extensions for new topics (barrel markers, anomaly results, dialogue window).                                                                                                                                                                                                                                                                                                                 |
| **`perception_utils.py`**      | YES       | Core utilities are fully reusable: PC2 extraction, SVD surface fitting, TF2 transforms, `IncrementalTrackManager`. The `DepthCameraGeometry` class exists but is unused (nodes use PC2 directly).                                                                                                                                                                                                           |
| **`speech.py`**                | YES       | TTS is reusable. Task 2 needs **speech recognition** (ASR) and **dialogue processing**, which does NOT exist yet.                                                                                                                                                                                                                                                                                           |
| **`sim_arm_nav.launch.py`**    | YES       | Already set up for Task 2 (arm robot, factory map, nav2 config).                                                                                                                                                                                                                                                                                                                                            |
| **`nav2.yaml`**                | PARTIALLY | Navigation config may need tuning for task 2 specifics (line following, yellow-line avoidance via costmap).                                                                                                                                                                                                                                                                                                 |
| **Waypoints**                  | PARTIALLY | `task.yaml` has 24 waypoints for full factory exploration. Task 2 splits into two rooms with different navigation modes, so waypoints may need restructuring.                                                                                                                                                                                                                                               |
| **Worlds**                     | YES       | `task2` world (via `dis_tutorial3`) and `task2_meshes/` directory with factory assets. `task2_green_demo_meshes/` is also present.                                                                                                                                                                                                                                                                          |

---

### 4. WHAT'S MISSING AND NEEDS TO BE BUILT

This is a large gap. Based on `docs/01-task2.md`, here's what exists vs what needs to be built:

| Required Capability                                | Status             | Needs                                                                                                                                                                    |
| -------------------------------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Goal-based navigation**                          | EXISTS             | Controller already sends Nav2 goals. Reusable.                                                                                                                           |
| **Face detection**                                 | EXISTS             | `face_detector.py` works.                                                                                                                                                |
| **Face recognition (name, gender)**                | MISSING            | Need to add face recognition against known photo set. Gender classification.                                                                                             |
| **Ring detection + color**                         | EXISTS             | `ring_detector.py` works. Add purple/orange/brown to COLOR_RANGES.                                                                                                       |
| **Cylinder detection**                             | MISSING            | New detector node needed.                                                                                                                                                |
| **Color recognition**                              | EXISTS (rings)     | Extend to barrels, cylinders.                                                                                                                                            |
| **Approaching objects**                            | EXISTS             | Controller's APPROACHING_OBJECT state handles faces/rings. May need extension for barrels.                                                                               |
| **Speech synthesis**                               | EXISTS             | `speech.py` works.                                                                                                                                                       |
| **Speech recognition (ASR)**                       | MISSING            | No ASR node exists. Need STT + dialogue management.                                                                                                                      |
| **Dialogue processing**                            | MISSING            | Need stateful dialogue: ask for task, handle gender-based logic (men answer directly, women may reconsider), confirm tasks. QR code fallback shortcut available.         |
| **Line detection (yellow forbidden, blue follow)** | MISSING            | Need line detector node using down-facing camera (`/top_camera`). Need to integrate with costmap (yellow = lethal obstacle) and controller (blue = line-following mode). |
| **Barrel inspection**                              | MISSING            | Need barrel detector: locate, color, orientation (horizontal/vertical), leak detection. Approach and warn if leaking.                                                    |
| **Anomaly detection (red/green cell)**             | MISSING            | Need to train anomaly detection model (PCA/AE/VAE/diffusion/etc.). Test/sample tile images exist in `task2_meshes/` (damaged*.png, good*.png).                           |
| **Working cell detection**                         | MISSING            | Identify red vs green cell via line color detection.                                                                                                                     |
| **Arm manipulation**                               | EXISTS (partially) | `arm_mover` node from `dis_tutorial7` is launched. May need additional actions for camera-over-conveyor positioning.                                                     |
| **Inspection report**                              | MISSING            | Generate report and present to CTO at end of blue line in room 2.                                                                                                        |
| **CTO interaction**                                | MISSING            | Find CTO at end of blue line, present report.                                                                                                                            |
| **Autonomous space exploration**                   | MISSING (optional) | Current approach uses hand-coded waypoints. Autonomous frontier exploration would be an enhancement.                                                                     |
| **Perception visualizer extensions**               | MISSING            | Need to add: dialogue window, anomaly detection visualization, barrel markers, cylinder markers, inspection report display.                                              |

---

### 5. THE CONTROLLER'S STATE MACHINE / LOGIC

From `controller.py` (1127 lines), the state machine is:

```
WAITING_FOR_NAV2
    │
    ├── Nav2 lifecycle nodes active + initial pose received
    ├── If docked → UNDOCKING
    └── Else → EXPLORING

UNDOCKING
    │
    └── Undock complete → EXPLORING (+ send first waypoint)

EXPLORING
    │
    ├── Detection queue non-empty? → pop, compute approach candidates, → APPROACHING_OBJECT
    │
    ├── Navigation complete? → increment waypoint_index
    │   ├── All found? → DONE
    │   ├── Loop exhausted (max_loops)? → DONE
    │   └── Else → next waypoint or spin
    │
    └── Spin complete? → next waypoint, back to EXPLORING

APPROACHING_OBJECT
    │
    ├── Nav succeeded → VERIFYING (pause for verify_pause_sec)
    │
    ├── Nav aborted/rejected → retry next candidate (fan of 8 approach angles)
    │   ├── More candidates? → send next approach goal
    │   ├── Retry with increased distance (approach_retry_offset per cycle)
    │   └── All exhausted → give up, back to EXPLORING
    │
    └── Costmap check: if goal in high-cost area → immediately abort

VERIFYING
    │
    └── Pause for verify_pause_sec → greet face ("Hello!") / announce ring color ("I see a {color} ring")
        └── Mark as greeted, check all_found → EXPLORING or DONE

DONE
    └── Log summary, speak "Mission complete!"
```

**Key controller mechanisms:**

1. **Costmap guard** (`_cost_at_goal_ok`): Before sending any nav goal, checks if the target cell in `/global_costmap/costmap` has cost >= 50. If so, aborts immediately and tries the next candidate. Uses world-to-map coordinate conversion.

2. **Approach candidate fanning** (`_approach_candidates`): Generates 8 approach poses fanned out at 45-degree intervals from the surface normal direction, ordered by angular preference. The controller tries them sequentially on nav failures. After exhausting all, it retries with increased approach distance (`approach_retry_offset` per cycle, up to 10 full cycles of 8 attempts = 80 total retries before giving up).

3. **Detection deduplication**: Uses `dedup_distance` (0.8m) to avoid counting the same face/ring twice. Stale detections (>120s unseen) are pruned.

4. **Pending approach queue**: Detections are queued. The controller interrupts waypoint navigation to handle them. Ungreeted but seen-before objects are re-queued to avoid getting stuck in waypoint loops.

5. **Staleness pruning**: Detections not seen for `detection_max_age` seconds (120s) are removed from tracking.

6. **Flight guard** (`nav_in_flight`): Prevents stale result futures from being read by the state machine tick.

7. **Speech queue**: A separate timer (2Hz) dequeues and speaks messages non-blockingly via `espeak-ng`.

---

### 6. TOPIC AND ACTION INTERFACES USED

#### Topics Published by Megatron Nodes:

| Topic                        | Type          | Publisher               | Purpose                                                      |
| ---------------------------- | ------------- | ----------------------- | ------------------------------------------------------------ |
| `/detected_faces`            | `PoseStamped` | `face_detector`         | Face detections (position + surface-normal orientation)      |
| `/detected_rings`            | `PoseStamped` | `ring_detector`         | Ring detections (color in `frame_id` as `"map\|{color}"`)    |
| `/face_markers`              | `MarkerArray` | `face_detector`         | RViz sphere + text for confirmed faces                       |
| `/ring_markers`              | `MarkerArray` | `ring_detector`         | RViz sphere + arrow (normal) + text per ring                 |
| `/face_detections_image`     | `Image`       | `face_detector`         | Annotated BGR detection image                                |
| `/ring_detections_image`     | `Image`       | `ring_detector`         | Annotated BGR detection image                                |
| `/ring_debug/binary`         | `Image`       | `ring_detector`         | Combined threshold binary mask                               |
| `/ring_debug/ellipses`       | `Image`       | `ring_detector`         | All fitted ellipses with scores                              |
| `/ring_debug/pairs`          | `Image`       | `ring_detector`         | Matched concentric pairs                                     |
| `/ring_debug/color`          | `Image`       | `ring_detector`         | Color classification + depth-gap overlay                     |
| `/goal_markers`              | `MarkerArray` | `mission_controller`    | Waypoint arrows (visited=green, current=yellow, future=blue) |
| `/mission_status`            | `String`      | `mission_controller`    | Status text: state + face/ring/waypoint/loop counts          |
| `/approaching_object`        | `Marker`      | `mission_controller`    | Downward arrow + label at approach goal position             |
| `/task1_visualization_image` | `Image`       | `perception_visualizer` | Full debug panel (header + face/ring + 4 debug stages)       |
| `/task1_rviz_image`          | `Image`       | `perception_visualizer` | Rviz-friendly panel (header + face/ring row only)            |

#### Topics Subscribed by Megatron Nodes:

| Topic                                       | Type                        | Subscriber                                    | Purpose                                                 |
| ------------------------------------------- | --------------------------- | --------------------------------------------- | ------------------------------------------------------- |
| `/oakd/rgb/preview/image_raw`               | `Image`                     | `face_detector`, `ring_detector`              | RGB camera feed (synced with depth via message_filters) |
| `/oakd/rgb/preview/depth/points`            | `PointCloud2`               | `face_detector`, `ring_detector`              | Organized point cloud (synced with RGB)                 |
| `dock_status`                               | `DockStatus`                | `mission_controller`                          | Is robot docked?                                        |
| `amcl_pose`                                 | `PoseWithCovarianceStamped` | `mission_controller`                          | Current robot pose                                      |
| `/global_costmap/costmap`                   | `OccupancyGrid`             | `mission_controller`                          | For cost-at-goal check                                  |
| `/detected_faces`                           | `PoseStamped`               | `mission_controller`, `perception_visualizer` | Face detection consumption                              |
| `/detected_rings`                           | `PoseStamped`               | `mission_controller`, `perception_visualizer` | Ring detection consumption                              |
| `/mission_status`                           | `String`                    | `perception_visualizer`                       | Status display                                          |
| `/face_detections_image`                    | `Image`                     | `perception_visualizer`                       | Composition source                                      |
| `/ring_detections_image`                    | `Image`                     | `perception_visualizer`                       | Composition source                                      |
| `/ring_debug/{binary,ellipses,pairs,color}` | `Image`                     | `perception_visualizer`                       | Debug panel composition                                 |

#### Action Clients:

| Action                                | Client Node          | Purpose                         |
| ------------------------------------- | -------------------- | ------------------------------- |
| `navigate_to_pose` (`NavigateToPose`) | `mission_controller` | Nav2 waypoint/approach goals    |
| `spin` (`Spin`)                       | `mission_controller` | Optional 360° spin at waypoints |
| `undock` (`Undock`)                   | `mission_controller` | iRobot Create3 undocking        |

#### Action Server (external):

| Action                 | Server           | Package         |
| ---------------------- | ---------------- | --------------- |
| `arm_mover_actions.py` | `arm_mover` node | `dis_tutorial7` |

#### TF Transforms:

- `face_detector` and `ring_detector` both use `tf2_ros` to lookup `map` ← `oakd_rgb_camera_optical_frame` for each detection frame.
- `perception_utils.transform_point_and_normal()` handles Point + Vector3 transforms.
- Controller does not directly use TF (receives AMCL poses and detections already in map frame).

#### Lifecycle Services (Controller):

Calls `GetState` on `amcl/get_state`, `bt_navigator/get_state`, `global_costmap/global_costmap/get_state` to verify Nav2 readiness.

---

### 7. IMPORTED UTILITIES AND DEPENDENCIES

**Python package dependencies** (from `package.xml`):

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `nav_msgs`
- `cv_bridge`, `tf2_ros`, `tf2_geometry_msgs`, `message_filters`
- `nav2_msgs`, `action_msgs`, `lifecycle_msgs`, `builtin_interfaces`
- `irobot_create_msgs` (for `Undock` and `DockStatus`)
- `turtle_tf2_py` (for `quaternion_from_euler` in controller)

**External Python packages** (from imports):

- `ultralytics` — YOLO model for face detection (`yolov8n-face.pt`)
- `opencv-python` (cv2) — image processing, ellipse fitting, color classification
- `numpy` — array math, SVD, norms
- `pyyaml` — waypoint file parsing
- `message_filters` — time-synchronized RGB+PointCloud2 subscribers
- `sensor_msgs_py.point_cloud2` — organized PC2 → numpy extraction
- `tf2_geometry_msgs` (`do_transform_point`, `do_transform_vector3`) — frame transforms

**External ROS 2 packages** (launch-time dependencies):

- `dis_tutorial3` — Gazebo world, localization (AMCL), Nav2 launch, TurtleBot4 spawn
- `dis_tutorial7` — arm-enabled robot spawn, `arm_mover_actions` node
- `rviz2` — visualization
- `nav2_bringup` — navigation stack (BT navigator, controller, planner, costmaps, etc.)

---

### 8. ADDITIONAL FILE INVENTORY

**Launch files** (6 total):

- `task1.launch.py` (144 lines) — Task 1 full stack
- `task1_no_config.launch.py` (107 lines) — Task 1 without custom RViz config
- `task2.launch.py` (143 lines) — Task 2 full stack (arm robot)
- `sim_turtlebot_nav.launch.py` (133 lines) — Simulation + nav (Task 1 reuse of dis_tutorial3)
- `sim_arm_nav.launch.py` (121 lines) — Simulation + nav with arm robot (Task 2)
- `nav_only.launch.py` (114 lines) — Navigation-only variant

**Config files** (5):

- `config/nav2.yaml` (312 lines) — Full Nav2 parameters for Task 2
- `config/nav2_old.yaml` (313+ lines) — Older nav2 config
- `config/nav2 copy.yaml` — Backup
- `config/production.rviz` (950 lines) — Main RViz preset (map, robot, scan, costmaps, face/ring markers, goal markers, approaching object, perception image, AMCL particles)
- `config/topdown_only.rviz` (789+ lines) — Alternative top-down RViz config

**Waypoint files** (9 YAML files):

- `task.yaml` (241 lines, 24 waypoints) — Full factory exploration waypoints, used by task2.launch.py
- `test1.yaml` (141 lines, 14 waypoints) — Task 1 testing waypoints
- `test1-original.yaml` — Backup of original test1
- `test2.yaml` (31 lines, 3 waypoints) — A small three-waypoint test path
- `face_check.yaml` (11 lines, 1 waypoint) — Single waypoint near face location
- `ring_check.yaml` (51 lines, 5 waypoints) — Ring-specific test waypoints
- `error.yaml` (41 lines, 4 waypoints) — Error-waypoint test
- `old_error.yaml`, `ring_check.yaml.yaml` — Backups/duplicates

**Map files** (6):

- `maps/factory.pgm` + `factory.yaml` — Task 2 factory map (origin [-6.858, -9.600], resolution 0.05)
- `maps/task1.pgm` + `maps/task1.yaml` — Task 1 map (origin [-3.108, -4.384])
- `maps/task1_orig.pgm` + `maps/task1_orig.yaml` — Task 1 original map (origin [-3.109, -4.384])

**World files** (3):

- `worlds/task1.sdf` — Task 1 world (references `task1_meshes/`)
- `worlds/task1_green_demo.sdf` — Task 1 green demo variant
- `worlds/task2_meshes/` — ~40 files: map.obj, collider.obj, textures, QR codes (qr\_\*.png), person photos (fred, elena, jeff, anita, robert), damaged/good tile images (damaged5-27.png, good1-6.png), fake rings (fakering1-3.jpg), colored textures (red, green, blue, yellow, black, belt, asphalt, ground)
- `worlds/task2_green_demo_meshes/` — Same structure with different persons (maria, luka added) and different damaged tiles

**Test files** (3):

- `test/test_copyright.py` — Copyright header check (SKIPPED via `pytest.mark.skip`)
- `test/test_flake8.py` — Flake8 style check
- `test/test_pep257.py` — PEP257 docstring check
- (No unit tests for logic exist — only boilerplate lint checks from `ros2 pkg create`)

**Docs** (3):

- `docs/00-requirements.md` — Task 1 requirements, evaluation criteria
- `docs/01-task2.md` — Task 2 full specification: navigation (yellow/blue lines, two rooms), perception (barrels, rings, cylinders, faces, anomaly detection), dialogue (gender-based logic, speech recognition), shortcuts for lower grades, grading rubric (35 points total)
- `docs/02-structure.md` — Setup notes, launch chain diagram, file inventory

---

### SUMMARY OF KEY ARCHITECTURAL OBSERVATIONS

1. **Perception pipeline is modular**: Detectors publish `PoseStamped` with surface normals encoded as orientation and (for rings) color in `frame_id`. The controller is fully decoupled — it just subscribes to detection topics. This makes adding new detectors (cylinders, barrels) straightforward.

2. **The controller is tightly coupled to Task 1 semantics**: It only handles faces and rings. The state machine has no notion of barrels, dialogue, line detection, anomaly inspection, or the two-room split. It will need a major rewrite or a parallel higher-level state machine for Task 2.

3. **No existing ASR/speech-recognition infrastructure**: Only TTS exists (`speech.py` using espeak-ng). Dialogue processing and speech recognition must be built from scratch.

4. **No anomaly detection model**: Tile images exist in `task2_meshes/` but no ML pipeline is implemented. This can be developed mostly outside ROS and Gazebo per the spec.

5. **Arm integration is launch-only right now**: `arm_mover` node is launched but neither the controller nor detectors interact with it. Actions need to be wired up for camera positioning over conveyor belts.

6. **Costmap-based clearance checking** is already built and will be useful for yellow-line avoidance (by marking yellow areas as high-cost/lethal in the costmap).

7. **The `IncrementalTrackManager`** in `perception_utils.py` is well-designed for reuse — any new detector can use it for track/dedup/confirmation/filter logic without reinventing the wheel.
