# Handoff — Blue Line Follower

## Suggested skills

- `ros2-engineering-skills` — ROS 2 node development, `/cmd_vel`, LiDAR
- `grill-with-docs` — if design decisions need revisiting against CONTEXT.md

## References

| Artifact | Path |
|----------|------|
| Task 2 spec | `src/megatron/docs/01-task2.md` (lines 30–35, Room 2) |
| Final plan | `src/megatron/docs/06-plan-final.md` (§Adversarial H, §State machine FOLLOW_BLUE_LINE) |
| Mockup (current) | `src/megatron/megatron/blue_line_follower.py` |
| Controller integration | `src/megatron/megatron/task2_controller.py` (State.FOLLOW_BLUE_LINE, `_handle_follow_blue_line`) |
| Teammate's solution | `src/vendor/teammate-project/src/task1/task1/blue_line_explorer.py` (529 lines, proven against same world) |
| CONTEXT.md | Root — glossary: Room 2, CTO, Patrol |
| Launch file | `src/megatron/launch/task2.launch.py` (line 208: `blue_line_follower` commented out) |
| Arm poses | `src/vendor/dis_tutorial7/scripts/arm_mover_actions.py` |

## What exists

### Mockup (`blue_line_follower.py`)
98-line stub. HSV blue mask (`H ∈ [90,140]`), centroid P-controller, EMA smoothing. Activated by `/robot_state == "FOLLOW_BLUE_LINE"`. Publishes `/cmd_vel_unstamped`. No junction handling. No termination. Completely untested — not even wired into the launch file.

### Controller integration
`task2_controller.py` `_handle_follow_blue_line()` (line 969) watches `/qr_task` for `"report"` token → generates report → publishes `DONE` on `/robot_state`. The `_on_patrol_complete()` transition to `FOLLOW_BLUE_LINE` at line 606 is **commented out** — must be uncommented for the full-room flow.

### Teammate's blue_line_explorer.py
Proven against the same Gazebo world. Contains:
- **SEARCH → FOLLOW → UTURN** modes (no termination — cycles forever)
- **3-ROI direction detection**: left/straight/right horizontal bands with pixel-count thresholds
- **Split-mode state machine**: `split_active` flag enters when left/right branch visible, exits when centroid is centered in straight zone for 0.5s hold
- **Left bias**: when left branch is visible during split, adds +0.25 to steering
- **LiDAR dead end**: forward cone at -90° scan angle, ±15° cone. U-turn at 0.37m, slow at 0.55m
- **U-turn**: 0.5 rad/s for 6.5s (~180°), then resume SEARCH
- **Line lost timeout**: 2s → U-turn
- **Bump detection**: `/hazard_detection` fallback
- **Speeds**: 0.15 m/s in split, 0.40 m/s on straight
- **HSV**: `H ∈ [82,102]`, plus cyan dominance check (B,G high, R low)
- **EMA smoothing**: α=0.35 on angular

## The blue line layout (not documented anywhere else)

Room 2 is a branching maze with 4 T-junctions. The blue line is visible to `/top_camera/rgb/preview/image_raw`.

```
Junction 1 — left fork: bait QR on fencepost, dead end (no wall, LiDAR sees post)
Junction 2 — right fork: 4 barrels blocking path, impassable
Junction 3 — left fork: large yellow tank, bait QR, LiDAR sees tank
Junction 4 — left fork: CTO + real QR on a wall, roundabout with hole in center

        incinerator
            |
            |
            |
          / |
  CTO ---<  |
          \ |
            |
            |
            |
          robot
```

At J4, the CTO branch is the left fork. The incinerator branch goes straight. Going to the incinerator is a dead end — the robot cannot navigate back to the CTO because J4 is an asymmetric fork.

## Decisions

### Junction strategy: prefer left, U-turn on dead end

A single heuristic navigates the entire maze: always take the left fork. On LiDAR obstacle ≤0.37m or line lost for >2s, execute a 180° U-turn and resume following. At J2, left is the only direction (barrels block right). At J4, left leads to CTO → QR termination.

**Discarded**: hardcoded junction map. Fragile if evaluation rearranges obstacles. Left-bias is simpler and works given the world geometry never changes.

**Risk**: if the world were mirrored (CTO on right fork), the robot would U-turn past him. Rejected as out of scope — the task spec guarantees fixed setup.

### Termination: QR code based, with a seam for end-of-line detection

The CTO's QR (`"Hello there! Thanks for the report."`) is on a wall at the end of the J4-left branch. `qr_reader.py` (already active in `FOLLOW_BLUE_LINE` mode) publishes `"report"` on `/qr_task`. Controller's existing `_handle_follow_blue_line` transitions to DONE — **no controller changes needed**.

A `/blue_line_finished` topic is designed as the future **seam**. The follower can later publish this topic from an end-of-line detector, and the controller subscribes to it as an OR gate alongside `/qr_task`. Not implemented now — only one adapter (QR) exists, so the seam is hypothetical. Do not introduce it unless a second termination method is actually needed.

**Discarded**: follower detects CTO via LiDAR. CTO face is a flat image on a wall — LiDAR sees it as a wall, same as J1/J3 dead ends. Cannot distinguish. QR is the only signal.

### Architecture: separate node, adapt teammate's algorithms

The follower stays a separate node (`blue_line_follower.py`) keeping `/robot_state` as controller→nodes broadcast only. It does **not** publish to `/robot_state` (the teammate's pattern of bidirectional /robot_state was rejected — muddles FSM authority).

**Interface** (inputs):
- `/robot_state` (String): activates on `FOLLOW_BLUE_LINE`, deactivates on `DONE`
- `/top_camera/rgb/preview/image_raw` (Image): downward camera, 320×240
- `/scan` (LaserScan): forward obstacle detection

**Interface** (outputs):
- `/cmd_vel_unstamped` (Twist): direct velocity commands (bypasses Nav2)

**Implementation**: adapt the teammate's algorithms (`blue_line_explorer.py` lines 227–488) but with our conventions:
- Constants at module level (like the mockup), not ROS parameters
- `/scan` instead of hazard_detection bumper (more reliable, no Create3 dependency)
- 0.12 m/s linear speed (our existing constant, tunable)
- HSV range from teammate (`H ∈ [82,102]` + cyan dominance) — theirs was tuned against this exact Gazebo world
- Arm to `garage` on FOLLOW_BLUE_LINE entry (controller publishes `/arm_command`)

**Discarded**: fold into `task2_controller.py`. Controller is already 1331 lines. Separate node keeps control rate (10 Hz) decoupled from serial FSM operations.

### LiDAR integration
Subscribe `/scan` with `sensor_qos`. Compute forward cone minimum range (the teammate's approach: forward at -90° scan angle, ±15° cone half-angle). U-turn at ≤0.37m, slow at ≤0.55m.

### QR codes at bait junctions (J1, J3)
Not scanned. LiDAR detects the obstacle before the robot reaches the QR. U-turn triggers first. Only the CTO's QR at J4 matters — wall proximity + stopped robot = clean decode.

### Incinerator
No special handling. Left-bias takes the CTO branch. If the robot somehow takes the incinerator path (right/straight), it's trapped — dead-end with no way back to CTO. Accept as unrecoverable. Do not program for this failure mode.

## Implementation plan

1. **Uncomment** `/robot_state` transition at `task2_controller.py:606` — `self._transition(State.FOLLOW_BLUE_LINE)`
2. **Uncomment** `blue_line_follower` in `task2.launch.py:208` — `ld.add_action(blue_line_follower)`
3. **Rewrite** `blue_line_follower.py`:
   - Add `/scan` subscriber, forward cone minimum-range computation
   - Add 3-ROI direction detection (left/straight/right bands)
   - Add split-mode state machine (enter on branch, exit when centered in straight)
   - Add left-bias steering (+0.25 when left branch visible in split)
   - Add SEARCH → FOLLOW → UTURN mode cycle
   - Add line-lost timeout (2s → U-turn)
   - Use teammate's HSV range + cyan dominance for blue mask
   - Keep our module-level constants, 0.12 m/s speed, `/cmd_vel_unstamped` output
   - Keep our `/robot_state` activation pattern (read-only, exit on DONE)
4. **Test** end-to-end: PATROL → patrol complete → FOLLOW_BLUE_LINE → navigate junctions → CTO QR → report → DONE

## Controller changes (minimal)

- `task2_controller.py:606`: uncomment `self._transition(State.FOLLOW_BLUE_LINE)`
- On FOLLOW_BLUE_LINE entry: publish `garage` to `/arm_command`
- No other changes needed. `_handle_follow_blue_line` stays as-is (watches `/qr_task` for `"report"`).

## Future seam

When/if QR termination proves unreliable, add end-of-line detection to the follower:
1. Follower publishes `Bool(True)` on `/blue_line_finished` when the line ends (no blue pixels for N seconds, or line depth discontinuity detected)
2. Controller subscribes to `/blue_line_finished` as an additional trigger in `_handle_follow_blue_line`
3. Both `/qr_task == "report"` and `/blue_line_finished` trigger report generation → DONE

Do **not** implement `/blue_line_finished` now. One adapter = hypothetical seam.
