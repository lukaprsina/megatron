# Delta from YELLOW_AVOIDER2.md

## yellow_avoider2.py

**BEV abandoned.** Removed ~130 lines of homography/TF/CameraInfo/`_quat_to_rot`. TTI now computed directly in image pixels (row = proxy for distance). No TF dependency.

**Contour filtering.** `_fit_line` uses `cv2.findContours`, selects the contour that reaches lowest in the image, fits line through *only* that contour's filled pixels. `min_pixels` raised to 150.

**Two phantom-corner guards:**
- **v_cross margin** — rejects if extrapolated center-column crossing is >30% outside the contour's vertical extent.
- **Bottom-pixel check** — after computing `x_bottom` (line's intersection with image bottom row), samples a 40×40px window at that location in the mask. Rejects if yellow pixel count < `bot_check_min_px` (8). New params: `bot_check_size`, `bot_check_min_px`.

**Hysteresis (30 ticks).** On threat loss, keeps publishing last steering angular + `steer_spd` forward for 30 ticks before CLEAR. Counteracts Nav2 counter-steering oscillation. New state vars: `_clear_ticks`, `_last_avoidance_ang`.

**Constant forward speed.** Linear velocity during STEERING is now `steer_spd` (0.06), not scaled by distance. Robot actually moves while steering.

**Steering direction:** `x_bottom > cx` → steer LEFT (positive), `< cx` → steer RIGHT. Removed the old cross-product sign method.

**Return type:** 6-tuple `(vx, vy, x0, y0, v_cross, x_bottom)` — `x_bottom` computed once in `_fit_line`, reused by `_run_avoidance` and debug.

**Debug image:** single view with yellow overlay, fitted line drawn across frame, intersection dot + center-column line, bottom-check ROI rectangle (cyan), scan-zone boundary.

**Param changes:** `scan_top_frac` 0.3→0.7, `trigger_frac` 0.45→0.75, `panic_frac` 0.08→0.05, `kp` 30.0→15.0, `min_pixels` 30→150.

## task2_controller.py

**barrel_approach_distance:** 0.80 → 1.50.

**Nav2 feedback blind zone:**
- `_nav_feedback_cb` receives `NavigateToPose.Feedback` via `send_goal_async(feedback_callback=...)`, stores `distance_remaining`.
- `_publish_approach_state()` runs every tick in APPROACH_TARGET. Publishes `"APPROACH_TARGET_FINAL"` when `distance_remaining < avoidance_blind_distance` (0.40m), otherwise `"APPROACH_TARGET"`.
- Avoider's `_ACTIVE_STATES` gate only fires on `PATROL`, `APPROACH_TARGET`, `INTERACT` — `APPROACH_TARGET_FINAL` disables avoidance.
- Stale-check: ignores feedback older than 2s.
- New param: `avoidance_blind_distance` (0.40).
