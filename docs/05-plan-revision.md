# 05-plan.md — Revision Notes

Issues discovered during adversarial review of 05-plan.md against source code.
Omitted: issues we dismissed as moot (rings already use IncrementalTrackManager,
DONE state already handles early completion, controller already re-queues
approaches, approach fanout disabled per user preference, cmd_vel race is
negligible).

---

## 1. SSIM reference tile — raw texture won't match Gazebo renders

**Problem:** 05-plan.md uses `worlds/task2_meshes/good1.png` (the mesh texture,
512×512 RGB) as the SSIM reference. Gazebo applies lighting, shadowing, and the
arm camera has its own resolution, color profile, and anti-aliasing. SSIM
between the raw texture and an arm-camera render will be low even for good
tiles, making threshold discrimination unreliable.

The plan's self-review caught this ("capture from arm camera") but settled on
the raw texture because it's identical across worlds (same MD5).

**Solution:** During Phase 0 (launch infra verification), run one capture pass:
launch sim, position the arm over a good tile, warp one tile from the arm
camera output, save as `config/reference_tile.png`. One-time, 5 minutes. The
reference must come from the same pipeline that produces runtime images.

This also calibrates the SSIM threshold empirically — run the same pipeline on
a known damaged tile and record both scores. Document thresholds in config.

**Alternatives considered:**

- Raw texture (`good1.png`): rejected — apples-to-oranges comparison
- Runtime capture (`good1.png`): not guaranteed any tile is good on eval day
- Gemini API: rejected for now
- PatchCore: kept as fallback if SSIM underperforms, ~30 lines via anomalib

---

## 2. Yellow avoider / inspection phase 3 conflict

**Problem:** `line_detector.py` does two things: (a) yellow danger-zone
avoidance during PATROL (yellow → stop + reverse), and (b) red/green workstation
blob detection. During inspection phase 3, the controller must reverse _until_
yellow is seen — the opposite behavior. Same camera, same node. The plan does
not specify how these modes are toggled.

**Solution:** `line_detector.py` subscribes to `/robot_state`. When state is
`"INSPECT_WORKSTATION"`, the danger-zone logic flips: instead of
stop + reverse, it publishes `True` on a new `/yellow_line_seen` Bool topic
(latched). The controller subscribes to this during phase 3 to know when to
stop reversing. The workstation blob detection path continues unchanged
(red/green line markers still accumulate).

Fast-path avoidance check (pixel count in danger ROI) runs every frame
regardless; the mode switch is a single `if` branch.

**Alternatives considered:**

- Separate toggle topics: adds surface area for no benefit
- Disable line_detector entirely during inspection: loses workstation markers
- Two separate nodes: adds a second camera subscription, anti-pattern

---

## 3. Unknown element counts vs `_all_found()`

**Problem:** The existing controller uses `_all_found()` (checks
`found_faces >= total_faces and found_rings >= total_rings`) with params like
`total_faces=3`. Task 2 spec: "The number of any element present is not known
in advance." Setting `total_cylinders=999` means `_all_found()` never fires
and the controller depends entirely on loop exhaustion. Setting it to a guess
risks missing barrels.

**Solution:** Remove `_all_found()` from the Task 2 controller entirely. Run
a single pass through all waypoints (one loop, no retry cycles). Transition to
FOLLOW_BLUE_LINE when `waypoint_index >= len(waypoints)`. All detectors
accumulate during the loop — whatever is seen is reported. No early exit.

This matches the user's decision: hardcode one loop, skip multi-loop retries.
The last waypoint in `waypoints/task.yaml` is the Room 1 → Room 2 exit point.

**Alternatives considered:**

- Multi-loop with detection-based completion: requires known totals, which the
  spec says are unknown
- Indefinite loops until manual stop: against spec (must operate autonomously)

---

## 4. No `cmd_vel_unstamped` publisher in controller

**Problem:** 05-plan.md specifies inspection phases 0-5 inside the controller:

- Phase 0: drive forward until LiDAR ≤ 0.30m
- Phase 3: reverse until yellow line or rear LiDAR ≤ 0.40m
- Phase 4: forward scan at 0.08 m/s, pause 2s per tile
- Phase 5: escape 130° CW turn, drive forward 4s

The existing `controller.py` has no `cmd_vel_unstamped` publisher. It uses
Nav2 action clients exclusively (`navigate_to_pose`, `spin`, `undock`).

**Solution:** Add `create_publisher(Twist, "/cmd_vel_unstamped", 10)` to
`task2_controller.py`. Use it during inspection phases for tight velocity
control. For broader moves (phase 0 forward until wall, phase 5 escape drive),
Nav2 `navigate_to_pose` goals work — they get obstacle avoidance. For the tile
scan at 0.08 m/s and the reversing precision, direct cmd_vel is more reliable.

**Phase 1 clarification (replaces Nav2 Spin):** Phase 1 yaw alignment uses a
cmd_vel angular P-controller, not Nav2 Spin. AMCL is already subscribed; yaw
is `atan2(2*(qw*qz+qx*qy), 1-2*(qy²+qz²))`. Per tick:
`error = normalize_angle(target_yaw - current_yaw)`, publish
`Twist(angular.z=clamp(Kp*error, max_w))`, advance to phase 2 when
`|error| < 0.05 rad`. No action client, no async callbacks, no
delta-vs-absolute normalization hazard at the ±π boundary.

**Alternatives considered:**

- Nav2-only: short goals for every phase. Works but adds 0.5-1s planning
  latency per move, problematic for 2s tile pauses
- Nav2 BehaviorTree `Backup`/`DriveOnHeading` actions: cleaner API, but
  behavior_server may not be configured in our nav2.yaml
- Nav2 Spin for phase 1: rejected — re-engages Nav2 stack mid-inspection,
  requires delta angle (wrap-around hazard), inconsistent with cmd_vel phases
- Mixed: Nav2 for wall approach + escape, cmd_vel for scan + backup + yaw.
  Pragmatic, adopted

---

## 5. Vendor C++ is vertical-only — teammate's version is modified

**Problem:** `src/vendor/dis_tutorial5/src/cylinder_segmentation.cpp` hardcodes
`axis(0, 0, 1)` — detects **only vertical** cylinders. The teammate's copy
(`src/vendor/teammate-project/src/dis_tutorial5/src/cylinder_segmentation.cpp`)
is substantially modified (~225 lines of changes):

- Removed axis constraint → detects horizontal cylinders too
- Classifies orientation from RANSAC axis → writes `"vertical"`/`"horizontal"`
  to `marker.text`
- Computes average color from cylinder point cloud → sets `marker.color`
- Added plane removal, voxel downsampling, z-spread rejection, saturation
  filtering, odometry turning-speed gate

Using the vendor version as-is means `cylinder_detector.py` gets only vertical
cylinders, no orientation info, and no color.

**Solution:** Use the teammate's modified C++ node. The diff is available at
`cylinder_segmentation.diff` (root of workspace). Options:

- Build the teammate's modified source (it's in
  `src/vendor/teammate-project/src/dis_tutorial5/`)
- Apply the diff to our vendor copy (but vendor is read-only submodule)
- Copy the teammate's modified `cylinder_segmentation.cpp` into megatron's
  own C++ package or a new package

The teammate's C++ node publishes `marker.text` with orientation and
`marker.color` with average RGB. `cylinder_detector.py` reads these from the
subscribed Marker — no separate color or orientation detection needed.

**Alternative:** Write orientation + color detection in Python inside
`cylinder_detector.py`. Avoids C++ dependency but reimplements geometry that
already works. Not recommended — the diff is battle-tested.

---

## 6. Barrel approach normal is undefined

**Problem:** The existing controller's `_approach_candidates` (line 643) fans
8 approach angles from a surface normal computed by SVD. For faces pasted on
walls, the normal points toward the robot — candidates radiate meaningfully.
For barrels, SVD on the cylinder point cloud gives the cylinder _axis_, not a
useful approach direction. A vertical barrel is round in XY — you can approach
from any angle.

The plan says "0.5m+0.3m lateral barrel" but this is a fixed Cartesian offset,
not a fan from a normal. Different compute path needed.

**Solution:** For approach candidates, the fan of 8 angles is disabled per
user preference. The controller computes a single approach point:

- Vertical barrel: distance from barrel center in the direction of
  `robot_pose → barrel_center`. Approach from the robot's current side.
- Horizontal barrel: approach perpendicular to cylinder axis in XY plane, with
  0.5m offset + 0.3m lateral shift (to robot-right, per teammate pattern).

Implementation: `cylinder_detector.py` publishes the barrel orientation as
part of the `PoseStamped` or a custom field. `task2_controller.py` switches
approach strategy based on detection type (face → normal-based, barrel →
position-based).

**Alternatives considered:**

- Fanout for barrels: circular approach from any angle works for vertical
  barrels, but adds complexity for an unused feature
- Custom message: `PersonInfo.msg`-style for barrels. Overkill for
  orientation + color — encode in existing fields

---

## 7. No `/arm_command` publisher in controller

**Problem:** Inspection phase 4 moves the arm to `look_at_belt_right` / `left`
for tile scanning. This is published as a `String` to `/arm_command`. The
existing `controller.py` has no such publisher.

**Solution:** Add `create_publisher(String, "/arm_command", 10)` to
`task2_controller.py`. Publish `"look_at_belt_right"` or `"look_at_belt_left"`
when entering `INSPECT_WORKSTATION` (before phase 0 begins). Phases 0
(forward to wall, ~2–5s) + 1 (yaw rotate, ~1–2s) overlap with the arm's ~3s
movement time, so the camera is settled before phase 2 needs `/top_camera` for
Hough tilt correction. Publish `"garage"` at the start of phase 5 (escape).

`garage` is a valid pose: `[0., -0.45, 2.8, -0.8]` — confirmed in
`dis_tutorial7/scripts/arm_mover_actions.py` line 47.

---

## 8. Fine-positioning needs LiDAR subscriber

**Problem:** Inspection phases 0 and 3 check LiDAR distances:

- Phase 0: drive forward until front LiDAR ≤ 0.30m from wall
- Phase 3: reverse until `/yellow_line_seen` or rear LiDAR ≤ 0.40m

The existing controller subscribes to `dock_status`, `amcl_pose`,
`/detected_faces`, `/detected_rings`, and costmap — no `/scan`.

**Solution:** Add `create_subscription(LaserScan, "/scan", callback, sensor_qos)`.
Callback stores the latest ranges array. Helper methods:

- `_min_forward_range()` → `min(ranges[0:30] + ranges[330:360])` (front ~60°)
- `_min_rear_range()` → `min(ranges[150:210])` (rear ~60°)

~30 lines. Sensor QoS (BEST_EFFORT, depth=1) to match scan publisher.

**Alternative:** Use costmap values at the robot's position. Less precise —
costmap resolution is 0.05m, laser is millimeter-level. Not suitable for the
0.30m wall stop.

---

## 9. Face recognition timing vs INTERACT

**Problem:** 05-plan.md throttles `face_recognition` to every 5th YOLO hit
(~2 Hz). INTERACT state waits for `/qr_task` and needs the person's name to
log "Jeff gave task: defects green." If recognition hasn't fired by the time
INTERACT starts, the name is `None`.

**Solution:** Recognition throttling is adequate: approach navigation takes
5-10s, during which recognition fires 2-5 times. The name is set on the
`IncrementalTrackManager` track's `label` field before INTERACT begins.

Edge case: first face seen at waypoint 1, recognition fires once during
approach, ~50% chance of match. Acceptable — single-face encounters are rare
in practice (robot patrols past multiple waypoints before engaging).

Controller behavior: if `track["label"]` is `None` during INTERACT, speak
generic greeting ("Hello!"), then update greeting when label arrives. No
blocking wait.

`face_detector.py` publishes name in the confirmed track's `frame_id` or
a custom field. `task2_controller.py` reads it from the detection callback.

**Alternatives considered:**

- Unthrottled recognition: every YOLO hit. ~10 Hz face_recognition on CPU.
  Simulation has headroom. Higher quality, 4-line code change. Acceptable
  if needed — increase throttle to every hit or every other hit
- Blocking wait in controller: anti-pattern, no

---

## 10. C++ cylinder_segmentation — build location

**Problem:** Issue 5 identifies the vendor C++ as unusable and the teammate's
modified version as correct, but leaves the build location unresolved.
`src/vendor/` is COLCON_IGNORE'd entirely, so
`src/vendor/teammate-project/src/dis_tutorial5/` won't build.

**Solution:** Copy the teammate's modified `dis_tutorial5/` directory to
`src/cylinder_segmentation/` (standalone ament_cmake package — already has its
own `CMakeLists.txt` and `package.xml`). One directory copy, no diff application,
no submodule modification. Update `task2.launch.py` to launch the
`cylinder_segmentation` executable from this new package.

This is the first action in Phase 0 — it unblocks all cylinder detection work.

**Alternatives considered:**

- Apply `cylinder_segmentation.diff` to vendor copy: vendor is read-only submodule
- Remove COLCON_IGNORE from teammate-project: builds 28 unwanted nodes
- Reimplement orientation + color in Python: duplicates battle-tested geometry
