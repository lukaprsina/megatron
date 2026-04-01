Hey Luka, I've gone through both versions carefully. Let me give you my honest assessment.

## On the two versions in general

Blaž's task queue idea is conceptually clean but the execution has serious problems. Your version is simpler and more correct in the areas that matter (nav_in_flight, greeted tracking, retry logic). The synthesis should start from yours and cherry-pick maybe one idea from his.

## Addressing your three points

### 1. Path computation — kill it all

Yes, the sync versions are genuinely dangerous. `rclpy.spin_once(self, ...)` inside a callback that's already being executed by the executor is **reentrant spinning**. It can deadlock, reorder callbacks, corrupt state. And Blaž's `_schedule_detection` calls `_compute_path_from_to` for *every* waypoint in the queue — if you have 10 waypoints, that's 10 sequential blocking calls each up to 1 second, all from within a subscription callback. That's catastrophic.

The async version `_compute_path_length_to` is also broken in a different way — it fires and forgets, storing the result in `self.last_path_length`. If two computations are in flight, they race on that single variable.

**My recommendation: delete every line related to `ComputePathToPose`.** No action client, no sync, no async. For a small competition map with 5 objects, the scheduling optimization is pure overhead. Euclidean distance is more than sufficient for deciding "approach now vs. later," and even that is questionable value (more below).

### 2. Scheduling strategy — simplify radically

The current Blaž logic is fundamentally broken anyway: it finds the waypoint *closest to the object* and inserts after it, but doesn't account for where the robot currently is relative to all the intermediate waypoints. The "savings" number is meaningless.

A priority queue sorted by distance sounds nice but distances change as the robot moves, so you'd need to re-sort constantly. Not worth it.

Here's what I'd actually do, from simplest to slightly smarter:

**Option A (simplest):** Don't interrupt waypoints at all. Navigate all waypoints, and after reaching each one, check if there are pending approaches. Go approach the nearest one, come back, continue waypoints. This avoids all the complexity of canceling Nav2 mid-navigation (which can cause weird recovery behavior) and the logic is trivial.

**Option B (slightly smarter):** When a detection comes in, compare euclidean distance to the object vs. euclidean distance to the next waypoint. If the object is closer, interrupt and approach it first. This is a one-liner decision with no path computation needed.

**Option C (what I'd actually pick):** Option A, but with a twist — the waypoints should be designed so the robot naturally passes near where objects are likely to be. Then "passing by" on the way to a waypoint counts, and you only need to stop for objects you didn't pass close enough to. This is a planning problem, not a controller problem.

For a 2-week deadline, I'd go with Option A. The time difference between optimal scheduling and "approach after each waypoint" is maybe 15-20 seconds on a small map. Not worth the bugs.

### 3. Approach pose and costmaps

You're right that this is a real problem. But I'd push back slightly on subscribing to the costmap.

**Why costmap subscription is more complexity than it's worth here:**
- You need to handle the `OccupancyGrid` message, maintain a reference, convert map coordinates to costmap indices, handle the costmap potentially not being available yet, deal with the resolution/origin offset, etc.
- The global costmap is mostly static after SLAM, so checking once is usually enough — but "usually" means you still need the retry logic as a fallback.
- The local costmap changes constantly, and checking it doesn't help if it changes *after* you check but *before* you arrive.

**What I'd do instead — a candidate pose generator:**

Rather than checking the costmap, generate a sequence of candidate approach poses and let Nav2 tell you which one works:

1. Normal approach along the surface normal at `approach_distance`
2. Normal approach at `approach_distance + 0.15m`
3. Normal approach at `approach_distance + 0.30m`
4. Offset 30cm perpendicular left, at `approach_distance`
5. Offset 30cm perpendicular right, at `approach_distance`
6. Give up and requeue

Each time Nav2 aborts/rejects, try the next candidate. This handles both the "approach point inside wall" case and the "costmap changed mid-approach" case, without ever looking at a costmap directly. Nav2 *is* your costmap checker — it already does this, it just tells you "no" instead of "here's a free cell."

This is simpler to implement, more robust (it uses Nav2's actual planning, not a snapshot), and handles the mid-approach costmap update case naturally (Nav2 aborts → you try next candidate at the new location).

The perpendicular offsets are key for your wall scenario. If the ring is next to a wall, going straight at it puts you in the wall, but offsetting 30cm to the side might give you a clear approach along the wall.

## Other things I noticed

**`nav_in_flight` is necessary.** Without it, here's the race condition: you finish a goal, `_nav_result` fires and sets `self.status = SUCCEEDED`. You pop the next task, call `_send_nav_goal` which calls `send_goal_async()` — but `self.result_future` still points to the *old* completed future. On the very next tick, `_is_nav_complete()` returns True immediately because the old future is still done. You'd skip the goal entirely. The guard fixes this correctly.

**The `greeted` flag in your version is important.** Blaž's version has no concept of "already greeted" — if a face is re-detected after being approached, it gets approached again, violating the "do not approach more than once" rule.

**Your `_requeue_if_not_pending` is a good idea but has a subtle issue:** it fires on every re-detection of an ungreeted object. If the robot is currently approaching that object, it correctly checks `current_approach` and skips. But if the approach just failed and the object was pushed back to the queue, and then it's re-detected before the robot gets to it, you'd add a duplicate. The dedup check inside it should catch this, but the comparison uses the new detection's position vs. the stored position, which might differ slightly. In practice this probably works fine, but be aware.

**Bug in your diff: `_send_spin()` is called unconditionally** at waypoints (line ~584 in the diff), ignoring the `spin_at_waypoints` parameter. Looks like a copy-paste issue.

**Blaž's loop management rebuilds the task queue on completion**, which could silently drop queued detections if a detection is added between "queue empty" and "rebuild." Your index-based approach doesn't have this problem.

## What I'd build

Starting from your version:
1. **Delete** all `ComputePathToPose` code and the `compute_path_client`
2. **Delete** `_schedule_detection` and the task queue. Go back to your simpler `pending_approaches` list
3. **Change scheduling to Option A**: don't interrupt mid-waypoint. After reaching a waypoint (and optionally spinning), check `pending_approaches` and go approach the nearest one. After approaching, resume waypoints.
4. **Replace the single retry with the candidate pose generator** (normal, farther, left, right)
5. **Keep** `nav_in_flight`, `greeted` tracking, `max_approach_attempts`, your loop management
6. **Fix** the `_send_spin()` conditional
7. **Add** the `approaching_object` marker publisher back from Blaž's version — it's nice for the demo visualization requirement

The resulting controller would be significantly shorter, have no reentrant spinning, have no race conditions on path computation results, and handle the wall-adjacent ring case. Want me to sketch the structure or talk through any of these points more?