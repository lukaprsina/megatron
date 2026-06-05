# yellow_avoider2 — implementation notes

## What changed from v1

v1 looks at the bottom 8% of the frame with fixed ROIs. v2 fits a line through all yellow pixels in the lower 70% of the frame and computes where that line crosses the robot's forward path (center column). This gives proportional control — steer harder the closer the crossing.

## Why BEV+TF was abandoned

The handoff planned a bird's-eye-view homography computed from `base_link → top_camera_rgb_camera_optical_frame` TF. This permanently fails in simulation because the `arm_controller` never starts (ros2_control lock acquisition fails all 5 attempts), so `joint_states` has 0 publishers, breaking the arm kinematic chain in TF. The TF trees for `base_link` and `top_camera_rgb_camera_optical_frame` remain disconnected for the entire session.

## How image-space TTI works

For a forward-looking camera, image row encodes ground distance: closer = lower row. The algorithm:

1. Yellow HSV mask over the lower 70% of the image (`scan_top_frac = 0.3`)
2. `cv2.fitLine` on all yellow pixels → line `(vx, vy, x0, y0)`
3. Find `v_cross`: row where the fitted line intersects the center column (`u = w/2`)
4. `remaining = h - v_cross` — pixels until the crossing reaches the robot
5. Trigger steering when `v_cross > h * trigger_frac` (default 0.45 — lower 55%)
6. Angular: `dist_sign * kp / max(remaining, 1)`, clipped to `±max_angular`
7. `dist_sign`: sign of perpendicular distance from `(cx, h)` to the fitted line — negative means line is to the left → steer right

## Tuning knobs

| Parameter | Default | Effect |
|---|---|---|
| `scan_top_frac` | 0.3 | How high up to look for yellow |
| `trigger_frac` | 0.45 | How far away to start steering (lower = later reaction) |
| `panic_frac` | 0.08 | When to back up instead of steer |
| `kp` | 30.0 | Proportional gain (pixel units) |
| `min_pixels` | 30 | Ignore sparse detections |

If detection is too aggressive, raise `trigger_frac` (e.g. 0.55) or lower `kp`.
