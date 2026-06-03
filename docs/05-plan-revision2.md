# Task 2 Plan Issues

## Needs action

**1. No data structures for report aggregation.** Controller tracks `found_faces` + `found_rings` (list of dicts with pos/normal/color). Plan says "collect rings/barrels/anomalies in controller" but never defines fields. **Need:** ring color counts `{"red": 2, "blue": 1}`, barrel list `[{"id": 1, "color": "green", "orientation": "horizontal", "leaking": false}]`, tile results `[{"station": "green", "tile_id": 0, "defect": false, "ssim": 0.92}]`.

**2. Workstation task-to-pose mapping implicit.** QR says "defects green"; controller must map "green" to a specific pose from `/detected_workstations`. **Fix:** `line_detector.py` publishes `Marker` with `ns="red"`/`ns="green"`. Controller stores `{"red": (x,y), "green": (x,y)}` dict on marker receipt.

**3. `/robot_state` latched QoS unspecified.** Plan says "latched" but no `QoSProfile` given. Nodes that edge-detect state transitions (blue_line_follower, line_detector mode switch, qr_reader) miss initial state on startup if durability isn't TRANSIENT_LOCAL. **Fix:** `QoSProfile(durability=TRANSIENT_LOCAL, reliability=RELIABLE, history=KEEP_LAST, depth=1)` — same as `amcl_pose_qos` at controller.py:53.

**4. `line_detector.py` camera not explicitly decided.** Plan omits which camera it subscribes to. **Decision:** `/top_camera/rgb/preview/image_raw` (arm camera). Same as teammate's `yellow_line_avoider.py:51`. Works for both yellow danger-zone ROI (bottom 65-95%) and workstation blob ROI (center) from default arm pose during PATROL.

**5. Barrel approach needs separate path from face approach.** Current `_approach_candidates()` calls `_quaternion_to_normal_2d()` — produces nonsense for cylinders. **Fix:** controller dispatches on detection type: face → normal-based fan, barrel → position-based (vertical: robot→barrel direction, horizontal: perpendicular to axis + 0.5m offset + 0.3m lateral).

**6. SSIM reference must be captured from arm camera, not raw texture.** Plan used `good1.png` (raw mesh texture). Gazebo renders differ (lighting, camera profile). **Fix:** during Phase 0 verification, launch sim, position arm over good tile, warp one tile from `/top_camera` output, save as `src/megatron/assets/tile_reference/reference.png`. All three worlds share identical `good1.png` (MD5 `7bbed964a13b4bf449291cce74ce7528`), so one capture suffices.

**7. Inspection tile processing blocks controller tick.** Phase 4 warp+SSIM in `_tick()` blocks ROS spin for ~5-50ms. During the 2s tile pause this is harmless (SSIM on 512×512 is ~5ms). If we later upgrade to PatchCore/U-Net, move to separate thread or subprocess. For now: inline, no change.

## Dismissed

- **Synthesis doc camera error** (04d-synthesis.md says OAK-D for yellow, teammate uses top_camera). Synthesis doc is wrong; teammate code is correct. Top_camera is the only downward-facing camera. No bug.
- **Top camera conflict during inspection** (arm at look_at_belt_left points at belt, not floor). Moot — revision's mode switch (`/robot_state` → flip danger-zone to `/yellow_line_seen` Bool) handles this, and phase 3 uses LiDAR not camera for backup stop.
- **Arm transition between phases 2 and 3.** No transition needed — arm stays at `look_at_belt_left` through phases 2-4.
- **LiDAR rear occlusion.** TurtleBot4 RPLIDAR is on a riser plate with 360° unobstructed view. Rear sectors return valid ranges.
- **Yellow avoider blocks barrel approach.** Barrels are in Room 2 (FOLLOW_BLUE_LINE state). Yellow avoider only activates in PATROL (Room 1). No conflict.
- **Blue line follower boot ordering.** Edge-detect on `/robot_state` works if QoS is latched (see issue 4). No separate topic needed.
- **WeChatQR dependency.** `opencv-contrib-python` added via uv. Model files (`detect.{caffemodel,prototxt}`, `sr.{caffemodel,prototxt}`) cloned to `src/megatron/assets/wechat_qrcode/`.
- **Blue line arm pose.** Top_camera in default pose already sees floor (teammate's blue_line_explorer has zero `/arm_command` references). No arm movement needed.
- **Vendor C++ vertical-only.** Teammate's modified `cylinder_segmentation.cpp` (horizontal + vertical, color, orientation) copied to `src/cylinder_segmentation/`, builds as standalone ament_cmake package, launched in `task2.launch.py`.
