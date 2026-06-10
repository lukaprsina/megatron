# Task 2 — Remaining work (2026-06-10)

Phases 0–3 complete (infra verification, controller skeleton, all detectors).
See `docs/adr/` for design decisions.

## Phase 4 — Inspection pipeline

### INSPECT_WORKSTATION in `task2_controller.py:_handle_inspection` (line 951)

- [ ] **Phase 1** — yaw alignment: P-controller on `/amcl_pose` yaw, target π (red) or π/2 (green), exit when |error| < 0.05 rad.
- [ ] **Phase 2** — tilt correction: Hough line detection on `/top_camera/rgb/preview/image_raw`, P-controller angular.z, exit at |tilt| < 0.5°.
- [ ] **Phase 3** — reverse to yellow: reverse at -0.06 m/s until `/yellow_line_seen` True or rear obstacle or 10 s timeout.
- [ ] **Phase 4** — forward tile scan: forward 0.08 m/s, brightness crossing → stop → capture → OTSU→contour→warp→SSIM vs reference tile. Exit on `/yellow_line_seen` True or 30 s timeout.
- [ ] **Phase 5** — escape: arm `"garage"`, CW turn 130° at -0.5 rad/s, forward 0.15 m/s for 4 s.

### FOLLOW_BLUE_LINE in `task2_controller.py:_handle_follow_blue_line` (line 960)

- [ ] Handle CTO face detection (Jeff) as alternative trigger for DONE (in addition to QR)
- [ ] Handle T-junction cases if blue line intersection causes wrong-branch behavior

## Phase 5 — Report + visualization

- [ ] `_generate_report()` — upgrade from text file to FPDF2 PDF
- [ ] `perception_visualizer.py` — add barrel markers, workstation markers, tile overlays
- [ ] `config/production.rviz` — add `/detected_cylinders`, `/detected_workstations` topics

## Phase 6 — Tuning + end-to-end

- [ ] Calibrate SSIM threshold (see ADR-003 reference tile procedure)
- [ ] Verify arm pose mapping: red→`look_at_belt_right`, green→`look_at_belt_left` (may be swapped)
- [ ] Review `waypoints/task.yaml` — ensure patrol covers both conveyors within 2 m
- [ ] End-to-end run: room 1 → INSPECT_WORKSTATION → FOLLOW_BLUE_LINE → report → DONE
- [ ] Tune yellow_line_injector HSV ranges, SSIM threshold, LiDAR distances in sim

## Phase 7 — Launch file

- [ ] Uncomment `face_detector`, `ring_detector`, `qr_reader`, `workstation_detector` in `task2.launch.py`
