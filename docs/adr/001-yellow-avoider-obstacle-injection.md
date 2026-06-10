# 001 — Yellow line avoidance via Nav2 obstacle injection

**Decision**: Use `yellow_line_injector.py` to convert yellow ground pixels into a
`/yellow_line_cloud` PointCloud2, consumed by Nav2's local costmap obstacle layer, rather
than publishing a competing `/cmd_vel` override.

**Rationale**: The original `yellow_avoider2.py` published `/cmd_vel` at 50 Hz on both
`/cmd_vel` and `/cmd_vel_unstamped` to win a last-write-wins race with Nav2's
RegulatedPurePursuit controller. This created a two-authority motion system with
hysteresis, state-machine complexity, and BACKING fallback paths. Injecting yellow pixels
as obstacles into Nav2 makes Nav2 the single authority — it sees yellow lines as
obstacles and routes around them naturally via its global+local planner chain. No
dual-write race, no hysteresis, no state machine, no BACKING panic.

**Consequences**:
- Simpler: `yellow_line_injector.py` is 186 lines vs ~400+ for yellow_avoider2.
- Nav2 must be configured with `obstacle_layer` subscribing to `/yellow_line_cloud`
  (`src/megatron/config/nav2.yaml`).
- Active only in PATROL/APPROACH_TARGET/INTERACT states, gated by `/robot_state`.
- INSPECT_WORKSTATION still publishes `/yellow_line_seen` via a separate mechanism
  (TODO in `task2_controller.py:_handle_inspection`, phase 3 reverse-to-yellow).

**Code**: `src/megatron/megatron/yellow_line_injector.py`
