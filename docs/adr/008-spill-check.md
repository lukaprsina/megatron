# 008 — Spill check via ground-level point cloud density

**Decision**: Detect barrel spills by counting OAK-D depth points in a Z-slice near
ground level within a 0.5 m XY radius of the barrel centroid. Threshold: ≥ 4000 points.

**Rationale**: Spilled fluid pools on the ground. Points within [0.005, 0.15 m] of ground
level (z=0 in Gazebo map frame) that are within 0.5 m XY of the barrel centroid indicate
a spill. A dry floor has negligible point density in this narrow Z-band near the barrel.
The 4000-point threshold was set empirically based on PC2 density from the OAK-D at 0.5 m
range.

**Service**: `/spill_check` (`std_srvs/Trigger`) provided by `cylinder_detector.py`.
Called by `task2_controller.py` during INTERACT when `current_target["type"] == "barrel"`.
At that point the robot is stationary at the approach pose (~0.6 m from barrel), OAK-D
facing it.

**Implementation**:
1. Read latest buffered `/oakd/rgb/preview/depth/points` (sensor_qos, BEST_EFFORT KEEP_LAST 1).
2. TF transform all points from `oakd_rgb_camera_optical_frame` → `map`.
3. Filter: `abs(z_map - 0.0) ∈ [0.005, 0.15]` AND `dist_xy(point_map, barrel_centroid) < 0.5`.
4. Return `leaking = (count >= 4000)`.

The same `/oakd/rgb/preview/depth/points` topic feeds `cylinder_segmentation` C++ for
RANSAC — no extra overhead.

**Code**: `src/megatron/megatron/cylinder_detector.py:_spill_check_cb` (line 381)
