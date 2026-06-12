"""Task 2 mission controller.

State machine:
  INIT → PATROL → APPROACH_TARGET → INTERACT → INSPECT_WORKSTATION → FOLLOW_BLUE_LINE → DONE

Detectors publish PoseStamped on their respective topics; this controller deduplicates
by position and queues pending_targets for approach.
"""

import math
from enum import Enum, auto
from pathlib import Path
from typing import cast

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from megatron.speech import Speaker

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


class State(Enum):
    INIT = auto()
    PATROL = auto()
    APPROACH_TARGET = auto()
    INTERACT = auto()
    INSPECT_WORKSTATION = auto()
    FOLLOW_BLUE_LINE = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

# Latched: late-joining subscribers get the last message immediately.
robot_state_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

amcl_pose_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# ---------------------------------------------------------------------------
# Waypoint loading (same format as controller.py)
# ---------------------------------------------------------------------------


def _quaternion_to_yaw(q_list):
    try:
        w, x, y, z = q_list
    except Exception:
        return 0.0
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints_from_yaml(path):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Waypoints file not found: {p}")
    data = yaml.safe_load(p.read_text())
    out = []
    candidates = []
    if isinstance(data, dict) and "waypoints" in data:
        candidates = list(data["waypoints"].values())
    elif isinstance(data, list):
        candidates = data
    elif isinstance(data, dict):
        candidates = list(data.values())

    for entry in candidates:
        x = y = yaw = None
        if isinstance(entry, dict):
            pose = entry.get("pose")
            orient = entry.get("orientation")
            if pose and len(pose) >= 2:
                x, y = float(pose[0]), float(pose[1])
            if orient and len(orient) == 4:
                yaw = _quaternion_to_yaw(orient)
        elif isinstance(entry, (list, tuple)):
            if len(entry) >= 2:
                x, y = float(entry[0]), float(entry[1])
            if len(entry) >= 3:
                yaw = float(entry[2])
        if x is None or y is None:
            continue
        out.append((x, y, yaw or 0.0))
    return out


# ---------------------------------------------------------------------------
# Normal extraction from quaternion (same as controller.py)
# ---------------------------------------------------------------------------


def _quaternion_to_normal_2d(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return -math.cos(yaw), -math.sin(yaw)


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ---------------------------------------------------------------------------
# QR task text → task token
# ---------------------------------------------------------------------------


def _parse_qr_task(text: str) -> str | None:
    t = text.lower()
    if "report" in t or "thanks" in t:  # qr_cto.png
        return "report"
    if "red belt" in t or ("red" in t and ("defect" in t or "fault" in t)):
        return "defects_red"
    if "green belt" in t or ("green" in t and ("defect" in t or "fault" in t)):
        return "defects_green"
    if "barrel" in t:
        return "barrels"
    if "ring" in t:
        return "rings"
    if "visitor" in t:
        return "nothing"
    if "emergency" in t or "incinerator" in t:
        return "emergency"
    return None


# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------


class Task2Controller(Node):
    NODES_TO_CHECK = ["amcl", "bt_navigator", "global_costmap/global_costmap"]
    DEDUP_DISTANCE = 0.01  # metres — two detections within this are the same object
    MAX_RETRY_CYCLES = (
        20  # bump distance up to 3× approach_retry_offset before giving up
    )

    def __init__(self):
        super().__init__("task2_controller")

        # --- Parameters ---
        self.declare_parameter("waypoints_file", "waypoints/task.yaml")
        self.declare_parameter("face_approach_distance", 0.50)
        self.declare_parameter("barrel_approach_distance", 0.60)
        self.declare_parameter("barrel_lateral_offset", 0.30)
        self.declare_parameter("approach_retry_offset", 0.20)
        self.declare_parameter("manual_mode", False)
        self.declare_parameter("avoidance_blind_distance", 0.40)

        wp_file = cast(str, self.get_parameter("waypoints_file").value)
        self.waypoints = load_waypoints_from_yaml(wp_file)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wp_file}")
        self.manual_mode = cast(bool, self.get_parameter("manual_mode").value)
        if self.manual_mode:
            self.get_logger().info(
                "MANUAL MODE — perception only, no navigation. Drive with keyboard."
            )

        # --- State ---
        self.state = State.INIT
        self.waypoint_index = 0

        # --- Nav2 lifecycle ---
        self._nav2_states = {n: "Unknown" for n in self.NODES_TO_CHECK}
        self.nav2_ready = False
        self._last_nav2_check = 0.0

        # --- Current robot pose ---
        self.current_pose = None
        self.initial_pose_received = False

        # --- Navigation async state ---
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_in_flight = False
        self._nav_ever_sent = False
        self._nav_seq = 0  # incremented on cancel; filters stale callbacks
        self._nav_rejected = False  # True when server rejected the last goal
        self._last_feedback_distance: float | None = None
        self._last_feedback_time: float | None = None
        self._nav_update = False  # Turn when the navigation needs to be updated

        # --- Costmap ---
        self.costmap = None

        # --- Detection tracking ---
        # Each entry: {'pos': np.array, 'normal': (nx,ny), 'type': str,
        #              'color': str|None, 'orientation': str|None, 'approached': bool,
        #              'label': str|None, 'quat': quaternion|None}
        self.found_faces: list[dict] = []
        self.found_rings: list[dict] = []
        self.found_barrels: list[dict] = []
        self.pending_targets: list[dict] = []  # queue of objects to approach

        # --- Task assignment (from person dialogue) ---
        self.assigned_task: str | None = None
        self.workstation_poses: dict[str, np.ndarray] = {}

        # --- Report data ---
        self.ring_counts: dict[str, int] = {}
        self.barrel_report: list[dict] = []
        self.tile_results: list[dict] = []

        # --- Approach tracking ---
        self.current_target: dict | None = None
        self._approach_attempt = 0
        self._patrol_complete = False

        # --- Interact tracking ---
        self._qr_task_raw: str | None = None

        # --- Inspection state ---
        self._inspection_phase = 0
        self._inspection_color: str | None = None
        self._last_scan_ranges = None
        self._yellow_seen = False
        self._tile_index = 0
        self._phase_start_time: float | None = None
        self._tile_pause_start: float | None = None

        # --- Speech ---
        self.speaker = Speaker()
        self.speaker.set_node_logger(self)

        # --- Spill check async state ---
        self._spill_future = None
        self._spill_start_time: float | None = None

        # --- Publishers ---
        self.robot_state_pub = self.create_publisher(
            String, "/robot_state", robot_state_qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self._spill_target_pub = self.create_publisher(PointStamped, "/spill_target", 1)
        self.goal_marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)
        self.approaching_object_pub = self.create_publisher(
            Marker, "/approaching_object", 10
        )

        # --- Subscribers ---
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._amcl_cb, amcl_pose_qos
        )
        self.create_subscription(PoseStamped, "/detected_faces", self._face_cb, 10)
        self.create_subscription(PoseStamped, "/detected_rings", self._ring_cb, 10)
        self.create_subscription(
            PoseStamped, "/detected_cylinders", self._cylinder_cb, 10
        )
        self.create_subscription(
            Marker, "/detected_workstations", self._workstation_cb, 10
        )
        self.create_subscription(String, "/qr_task", self._qr_cb, 10)
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/yellow_line_seen", self._yellow_seen_cb, 10)
        costmap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, costmap_qos
        )
        # --- Nav2 action client ---
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._spill_check_client = self.create_client(Trigger, "/spill_check")

        # --- Nav2 lifecycle clients ---
        self._nav2_clients = {
            n: self.create_client(GetState, f"{n}/get_state")
            for n in self.NODES_TO_CHECK
        }

        # --- Main tick timer (10 Hz) ---
        self.create_timer(0.1, self._tick)
        self.get_logger().info("Task2Controller initialised.")

    # ── Callbacks ─────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def _face_cb(self, msg: PoseStamped):
        parts = msg.header.frame_id.split("|")
        label = parts[1] if len(parts) > 1 else "unknown"
        id = parts[2] if len(parts) > 1 else "ID_NONDE"

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        nx, ny = _quaternion_to_normal_2d(msg.pose.orientation)
        now = self.get_clock().now()

        for f in self.found_faces:
            if f["id"] == id:  # face id found in found_faces
                if (
                    self.current_target is not None
                    and self.current_target["type"] == "face"
                    and self.current_target["id"] == id
                ):  # update approach goal
                    self.get_logger().info("Seting up to recalculate approach")
                    f["pos"] = pos
                    f["normal"] = (nx, ny)
                    f["last_seen"] = now

                    self.current_target["pos"] = pos
                    self.current_target["normal"] = (nx, ny)
                    self.current_target["last_seen"] = now
                    self._nav_update = True
                    return

                elif np.linalg.norm(pos - f["pos"]) < self.DEDUP_DISTANCE:
                    self.get_logger().info(
                        f"Requing face {f['id']} since the new detected location is more far away"
                    )
                    f["pos"] = pos
                    f["normal"] = (nx, ny)
                    f["last_seen"] = now
                    return
                else:
                    return

        self.get_logger().info(
            f"New face {label}(ID:{id}) at ({pos[0]:.2f}, {pos[1]:.2f})"
        )
        entry = {
            "id": id,
            "type": "face",
            "pos": pos,
            "normal": (nx, ny),
            "label": label,
            "approached": False,
            # "re_approach": True,
            "last_seen": now,
        }
        self.found_faces.append(entry)
        if self.state == State.PATROL:
            self.get_logger().info(f"New face {label}(ID:{id}) added to pending")
            self.pending_targets.append(dict(entry))

    def _ring_cb(self, msg: PoseStamped):
        parts = msg.header.frame_id.split("|")
        color = parts[1] if len(parts) > 1 else "unknown"
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        now = self.get_clock().now()

        for r in self.found_rings:
            if np.linalg.norm(pos - r["pos"]) < self.DEDUP_DISTANCE:
                r["pos"] = pos
                r["last_seen"] = now
                if (
                    self.current_target is not None
                    and self.current_target.get("type") == "ring"
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    < self.DEDUP_DISTANCE
                ):
                    self.current_target["pos"] = pos
                    self.current_target["last_seen"] = now
                    self._nav_update = True
                return

        self.get_logger().info(f"New ring ({color}) at ({pos[0]:.2f}, {pos[1]:.2f})")
        self.found_rings.append(
            {
                "type": "ring",
                "pos": pos,
                "color": color,
                "last_seen": self.get_clock().now(),
            }
        )
        self.ring_counts[color] = self.ring_counts.get(color, 0) + 1

    def _cylinder_cb(self, msg: PoseStamped):
        parts = msg.header.frame_id.split("|")
        color = parts[1] if len(parts) > 1 else "unknown"
        orientation = parts[2] if len(parts) > 2 else "vertical"
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        now = self.get_clock().now()

        for b in self.found_barrels:
            if np.linalg.norm(pos - b["pos"]) < self.DEDUP_DISTANCE:
                b["pos"] = pos
                b["last_seen"] = now
                if (
                    self.current_target is not None
                    and self.current_target.get("type") == "barrel"
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    < self.DEDUP_DISTANCE
                ):
                    self.current_target["pos"] = pos
                    self.current_target["last_seen"] = now
                    self._nav_update = True
                return

        self.get_logger().info(
            f"New {orientation} {color} barrel at ({pos[0]:.2f}, {pos[1]:.2f})"
        )
        entry = {
            "type": "barrel",
            "pos": pos,
            "color": color,
            "orientation": orientation,
            "quat": msg.pose.orientation,
            "approached": False,
            "last_seen": now,
        }
        self.found_barrels.append(entry)

    def _workstation_cb(self, msg: Marker):
        color = msg.ns  # "red" or "green"
        self.workstation_poses[color] = np.array(
            [msg.pose.position.x, msg.pose.position.y]
        )
        self.get_logger().info(
            f"Workstation '{color}' at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def _qr_cb(self, msg: String):
        self._qr_task_raw = msg.data
        self.get_logger().info(f"QR received: {msg.data!r}")

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_ranges = msg.ranges

    def _yellow_seen_cb(self, msg: Bool):
        self._yellow_seen = msg.data

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    # ── Dedup helper ──────────────────────────────────────────────────

    def _requeue_if_not_pending(self, obj_type: str, track: dict):
        if track.get("approached", False):
            return
        pos = track["pos"]
        for t in self.pending_targets:
            if (
                t["type"] == obj_type
                and np.linalg.norm(pos - np.array(t["pos"])) < self.DEDUP_DISTANCE
            ):
                return
        if (
            self.current_target is not None
            and self.current_target["type"] == obj_type
            and np.linalg.norm(pos - np.array(self.current_target["pos"]))
            < self.DEDUP_DISTANCE
        ):
            return
        self.get_logger().info(
            f"Re-queuing unapproached {obj_type} at ({pos[0]:.2f}, {pos[1]:.2f})"
        )
        self.pending_targets.append(dict(track))

    # ── Main tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self.state == State.INIT:
            self._handle_init()
        elif self.state == State.PATROL:
            self._handle_patrol()
        elif self.state == State.APPROACH_TARGET:
            self._handle_approach()
            # self._publish_approach_state()
        elif self.state == State.INTERACT:
            self._handle_interact()
        elif self.state == State.INSPECT_WORKSTATION:
            self._handle_inspection()
        elif self.state == State.FOLLOW_BLUE_LINE:
            self._handle_follow_blue_line()
        elif self.state == State.DONE:
            pass

    def _transition(self, new_state: State):
        self.get_logger().info(f"→ {new_state.name}")
        self.state = new_state
        msg = String()
        msg.data = new_state.name
        self.robot_state_pub.publish(msg)
        if new_state == State.PATROL:
            self._pub_arm("look_down")
        if new_state == State.DONE:
            self._cancel_nav()
            self.cmd_vel_pub.publish(Twist())

    # ── INIT ──────────────────────────────────────────────────────────

    def _handle_init(self):
        if not self.initial_pose_received:
            return

        self._check_nav2_states()

        if self.manual_mode:
            return

        if not self.nav2_ready:
            return
        self.get_logger().info("Nav2 ready — starting patrol.")
        self._transition(State.PATROL)
        self._send_next_waypoint()

    # ── PATROL ────────────────────────────────────────────────────────

    def _handle_patrol(self):
        if not self._is_nav_complete():
            return

        if self._nav_aborted():
            self.get_logger().warn(f"Waypoint {self.waypoint_index} aborted, skipping.")

        self.waypoint_index += 1

        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info("Patrol loop complete.")
            self._on_patrol_complete()
            return

        self._send_next_waypoint()

    def _on_patrol_complete(self):
        self.get_logger().info("Patrol loop complete — building post-patrol queue.")
        self._patrol_complete = True

        for f in self.found_faces:
            if not f.get("approached", False):
                self.pending_targets.append(dict(f))
                self.get_logger().info(
                    f"Queuing face {f.get('id')} at ({f['pos'][0]:.2f}, {f['pos'][1]:.2f})"
                )

        for b in self.found_barrels:
            if not b.get("approached", False):
                self.pending_targets.append(dict(b))
                self.get_logger().info(
                    f"Queuing {b.get('orientation')} {b.get('color')} barrel "
                    f"at ({b['pos'][0]:.2f}, {b['pos'][1]:.2f})"
                )

        if not self.pending_targets:
            self.get_logger().info("No targets to approach — heading to room 2.")
            self._pub_arm("look_down")
            self._transition(State.FOLLOW_BLUE_LINE)
            return

        target = self.pending_targets.pop(0)
        self.current_target = target
        self._approach_attempt = 0
        self._transition(State.APPROACH_TARGET)
        if not self._send_approach(target, attempt=0):
            self.get_logger().warn(
                "Initial approach candidates blocked — re-queuing target."
            )
            target["approached"] = False
            self.pending_targets.append(target)
            self.current_target = None
            self._next_post_patrol_target()

    # ── APPROACH_TARGET ───────────────────────────────────────────────

    def _handle_approach(self):
        if self.nav_in_flight:
            return

        if self._nav_update:
            self._cancel_nav()
            self._nav_update = False
            if self.current_target is not None:
                self._send_approach(self.current_target, self._approach_attempt)
            return

        if not self._is_nav_complete():
            return

        if self._nav_succeeded():
            self.get_logger().info("DEBUG!! NAV SUCCEDED ")
            self._publish_approaching_object(0.0, 0.0, none=True)
            self._transition(State.INTERACT)
            self._start_interact()
            return

        if self._nav_aborted():
            self._approach_attempt += 1
            if self.current_target is not None:
                if not self._send_approach(self.current_target, self._approach_attempt):
                    self.get_logger().warn(
                        "All approach candidates exhausted — re-queuing target."
                    )
                    self.current_target["approached"] = False
                    self.pending_targets.append(self.current_target)
                    self.current_target = None
                    if self._patrol_complete:
                        self._next_post_patrol_target()
                    else:
                        self._transition(State.PATROL)
                        self._send_next_waypoint()

    def _send_approach(self, target: dict, attempt: int) -> bool:
        """Try candidate at `attempt`, costmap-skip forward if blocked.

        Returns True if a nav goal was sent, False if all candidates at all
        retry distances have been exhausted.
        """

        _type = target["type"]
        if _type == "face":
            n_candidates = 8
            base_dist = cast(float, self.get_parameter("face_approach_distance").value)

            def _gen(d):
                return self._face_approach_candidates(
                    target["pos"], target["normal"], distance=d
                )
        else:
            n_candidates = 1
            base_dist = cast(
                float, self.get_parameter("barrel_approach_distance").value
            )

            def _gen(d):
                return self._barrel_approach_candidates(target, distance=d)

        retry_offset = cast(float, self.get_parameter("approach_retry_offset").value)
        total = self.MAX_RETRY_CYCLES * n_candidates
        remaining_cycles = self.MAX_RETRY_CYCLES - (attempt // n_candidates)
        while remaining_cycles > 0:
            cycle = attempt // n_candidates
            distance = base_dist + retry_offset * cycle
            candidates = _gen(distance)
            idx = attempt % n_candidates

            while idx < len(candidates):
                ax, ay, yaw = candidates[idx]
                self._publish_approaching_object(ax, ay, yaw, attempt, total=total)
                if self._cost_at_goal_ok(ax, ay):
                    self._send_nav_goal(ax, ay, yaw)
                    self._approach_attempt = attempt  # sync for next abort
                    return True
                idx += 1
                attempt += 1

            attempt = (cycle + 1) * n_candidates
            remaining_cycles -= 1

        return False  # fully exhausted

    def _face_approach_candidates(self, pos, normal, distance=None):
        """Fan of 8 from surface normal — same as controller.py."""
        dist = (
            distance
            if distance is not None
            else cast(float, self.get_parameter("face_approach_distance").value)
        )
        nx, ny = normal
        base = math.atan2(ny, nx)
        px, py = float(pos[0]), float(pos[1])

        def f(x):
            return (x**2) / 25**2

        # degrees = [val for i in range(0,30,5) for val in (i, -i)]
        degrees = [
            0,
            2,
            -2,
            3,
            -3,
            5,
            -5,
            7,
            -7,
            9,
            -9,
            10,
            -10,
            15,
            -15,
            # 20,          -20,
            # 25,          -25,
            # 30,          -30,
            # 35,          -35,
            # 40,          -40,
            # 50,          -50,
            # 60,          -60,
            # 70,          -70,
        ]
        offsets = [math.radians(d) for d in degrees]
        return [
            (
                px + math.cos(base + o) * dist * f(degrees[i]),
                py + math.sin(base + o) * dist * f(degrees[i]),
                math.atan2(-math.sin(base + o), -math.cos(base + o)),
            )
            for i, o in enumerate(offsets)
        ]

    def _barrel_approach_candidates(self, target: dict, distance=None):
        """Single approach candidate — position-based (no fanout)."""
        pos = target["pos"]
        dist = (
            distance
            if distance is not None
            else cast(float, self.get_parameter("barrel_approach_distance").value)
        )
        lateral = cast(float, self.get_parameter("barrel_lateral_offset").value)
        px, py = float(pos[0]), float(pos[1])

        if target.get("orientation", "vertical") == "vertical":
            # Approach from robot's current direction toward barrel
            if self.current_pose is not None:
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                dx, dy = px - rx, py - ry
                d = math.sqrt(dx**2 + dy**2)
                if d > 1e-3:
                    dx, dy = dx / d, dy / d
                else:
                    dx, dy = 1.0, 0.0
            else:
                dx, dy = 1.0, 0.0
            ax = px - dx * dist
            ay = py - dy * dist
            yaw = math.atan2(dy, dx)
            return [(ax, ay, yaw)]

        # Horizontal barrel: perpendicular to axis + lateral shift
        q = target.get("quat")
        if q is not None:
            ax_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )
        else:
            ax_yaw = 0.0
        # Two perpendiculars; pick the one toward robot
        perp_x, perp_y = -math.sin(ax_yaw), math.cos(ax_yaw)
        if self.current_pose is not None:
            rx, ry = self.current_pose.position.x, self.current_pose.position.y
            if (ry - px) * perp_x + (ry - py) * perp_y < 0:
                perp_x, perp_y = -perp_x, -perp_y
        # Lateral shift to robot-right (cross product of approach direction and Z-up)
        right_x = -perp_y  # rotate approach 90° CW in XY
        right_y = perp_x
        approach_x = px + perp_x * dist + right_x * lateral
        approach_y = py + perp_y * dist + right_y * lateral
        yaw = math.atan2(-perp_y, -perp_x)
        return [(approach_x, approach_y, yaw)]

    # ── INTERACT ──────────────────────────────────────────────────────

    def _start_interact(self):
        target = self.current_target
        if target is None:
            self._resume_patrol()
            return

        if target["type"] == "face":
            label = target.get("label") or ""
            name = label
            # pronoun = "woman" if "she" in label.lower() else "man"
            self.speaker.speak(f"Hi {name}! What task should I perform?")
            self._qr_task_raw = None

        elif target["type"] == "barrel":
            self.speaker.speak("Inspecting barrel.")
            self._spill_future = None
            self._spill_start_time = None

            if target.get("orientation") == "horizontal":
                pt = PointStamped()
                pt.header.frame_id = "map"
                pt.header.stamp = self.get_clock().now().to_msg()
                pos = target["pos"]
                pt.point.x, pt.point.y, pt.point.z = (
                    float(pos[0]),
                    float(pos[1]),
                    float(pos[2]),
                )
                self._spill_target_pub.publish(pt)

                if self._spill_check_client.service_is_ready():
                    self._spill_future = self._spill_check_client.call_async(
                        Trigger.Request()
                    )
                    self._spill_start_time = self.get_clock().now().nanoseconds * 1e-9
                else:
                    self.get_logger().warn("SpillCheck service not ready")

    def _handle_interact(self):
        target = self.current_target
        if target is None:
            self._resume_patrol()
            return

        if target["type"] == "barrel":
            orientation = target.get("orientation", "unknown")
            leaking = False
            spill_count = 0

            if orientation == "horizontal" and self._spill_future is not None:
                now = self.get_clock().now().nanoseconds * 1e-9
                elapsed = now - (self._spill_start_time or now)
                if not self._spill_future.done() and elapsed < 5.0:
                    return  # still waiting
                try:
                    resp = self._spill_future.result()
                    if resp is not None and resp.success:
                        leaking = "LEAK" in resp.message
                        try:
                            spill_count = int(
                                resp.message.split("count=")[1].split()[0]
                            )
                        except (IndexError, ValueError):
                            pass
                    elif resp is not None:
                        self.get_logger().warn(f"SpillCheck error: {resp.message}")
                except Exception as e:
                    self.get_logger().warn(f"SpillCheck failed: {e}")
            # vertical barrels and error cases fall through with leaking=False

            if leaking:
                self.speaker.speak("Alert! Alert! This barrel is leaking!")
            else:
                self.speaker.speak("Barrel OK.")

            self.barrel_report.append(
                {
                    "id": len(self.barrel_report) + 1,
                    "color": target.get("color", "unknown"),
                    "orientation": orientation,
                    "leaking": leaking,
                    "spill_count": spill_count,
                    "spill_threshold": 4000,
                    "pos": target["pos"].tolist(),
                }
            )
            self._mark_approached(target)
            self._resume_patrol()
            return

        # Face: wait for QR
        if (
            self._qr_task_raw is None or self._qr_task_raw == ""
        ):  # if qr_task is not seen it is failed
            self.get_logger().info("Waitting for qr task")
            return

        task_token = _parse_qr_task(self._qr_task_raw)
        self._qr_task_raw = None

        if task_token == "emergency":
            self.get_logger().info(
                "Emergency Intelligence Incinerator triggered - falling down Glados oven!"
            )
            self.speaker.speak("Emergency testing! Initiating system shutdown...")
            self._transition(State.DONE)
            rclpy.shutdown()
            return

        if task_token and task_token != "nothing" and task_token != "report":
            self.assigned_task = task_token
            # TODO: not used
            _color_word = task_token.split("_")[1] if "_" in task_token else task_token
            self.speaker.speak(
                f"OK. I will {task_token.replace('_', ' ').replace('defects', 'detect anomalies on the')}."
            )
        elif task_token == "nothing":
            self.speaker.speak("OK, no task for me. Continuing patrol.")
        else:
            self.speaker.speak("Understood.")

        self._mark_approached(target)
        self._resume_patrol()

    def _mark_approached(self, target: dict):
        pos = target["pos"]
        for f in self.found_faces + self.found_barrels:
            if np.linalg.norm(pos - f["pos"]) < self.DEDUP_DISTANCE:
                f["approached"] = True
        self.current_target = None

    def _resume_patrol(self):
        self.current_target = None
        if self._patrol_complete:
            self._next_post_patrol_target()
        else:
            self._transition(State.PATROL)
            self._send_next_waypoint()

    def _next_post_patrol_target(self):
        if self.pending_targets:
            target = self.pending_targets.pop(0)
            self.current_target = target
            self._approach_attempt = 0
            self._transition(State.APPROACH_TARGET)
            if not self._send_approach(target, attempt=0):
                self.get_logger().warn(
                    "Post-patrol approach candidates blocked — re-queuing."
                )
                target["approached"] = False
                self.pending_targets.append(target)
                self.current_target = None
                self._next_post_patrol_target()
        else:
            self.get_logger().info(
                "All post-patrol targets handled — heading to room 2."
            )
            self._pub_arm("look_down")
            self._transition(State.FOLLOW_BLUE_LINE)

    # ── INSPECT_WORKSTATION ───────────────────────────────────────────

    def _start_inspection(self, color: str):
        self._inspection_color = color
        self._inspection_phase = 0
        self._tile_index = 0
        self._yellow_seen = False
        self._phase_start_time = self.get_clock().now().nanoseconds / 1e9
        ws_pos = self.workstation_poses[color]
        self._send_nav_goal(
            ws_pos[0], ws_pos[1], math.pi if color == "red" else math.pi / 2
        )

    def _handle_inspection(self):
        # Phase 0: navigate to workstation
        if self._inspection_phase == 0 and not self._is_nav_complete():
            return
        if self._inspection_phase == 0 and self._is_nav_complete():
            arm_pose = (
                "look_at_belt_right"
                if self._inspection_color == "red"
                else "look_at_belt_left"
            )
            self._pub_arm(arm_pose)
            self._inspection_phase = 1
            self._phase_start_time = self.get_clock().now().nanoseconds / 1e9
            return

        # TODO Phase 4: implement inspection phases 1–5.
        # Stub: phases 1+ skip straight to FOLLOW_BLUE_LINE.
        # Remove this block when real phase logic is added.
        if self._inspection_phase >= 1:
            self.get_logger().warn(
                "INSPECT_WORKSTATION not yet implemented — skipping to FOLLOW_BLUE_LINE."
            )
            self._pub_arm("look_down")
            self._transition(State.FOLLOW_BLUE_LINE)

    # ── FOLLOW_BLUE_LINE ──────────────────────────────────────────────

    def _handle_follow_blue_line(self):
        # blue_line_follower.py activates on /robot_state == "FOLLOW_BLUE_LINE"
        if self._qr_task_raw is not None:
            task_token = _parse_qr_task(self._qr_task_raw)
            self._qr_task_raw = None
            if task_token == "report":
                self.get_logger().info("CTO QR received — generating report.")
                self._generate_report()
                self._transition(State.DONE)
                self.speaker.speak("Inspection complete. Report delivered.")

    # ── DONE ─────────────────────────────────────────────────────────

    # (no periodic action; _tick does nothing in DONE)

    # ── Report ────────────────────────────────────────────────────────

    def _generate_report(self):
        """Write a simple text report. Replace with FPDF2 in Phase 5."""
        lines = ["# Task 2 Inspection Report\n"]
        lines.append("## Ring Counts")
        for color, count in sorted(self.ring_counts.items()):
            lines.append(f"  {color}: {count}")
        lines.append("\n## Barrel Inspection")
        for b in self.barrel_report:
            leak = "LEAKING" if b["leaking"] else "OK"
            lines.append(f"  #{b['id']} {b['color']} {b['orientation']} — {leak}")
        lines.append("\n## Tile Anomalies")
        for t in self.tile_results:
            flag = "DEFECT" if t["defect"] else "OK"
            lines.append(
                f"  {t['station']} tile {t['tile_id']}: {flag} (ssim={t['ssim']:.3f})"
            )
        report_text = "\n".join(lines)
        self.get_logger().info("\n" + report_text)
        try:
            import os
            import tempfile

            path = os.path.join(tempfile.gettempdir(), "megatron_report.txt")
            with open(path, "w") as fh:
                fh.write(report_text)
            self.get_logger().info(f"Report written to {path}")
        except Exception as e:
            self.get_logger().error(f"Report write failed: {e}")

    # ── Arm command ────────────────────────────────────────────────────

    def _pub_arm(self, pose_name: str):
        msg = String()
        msg.data = pose_name
        self.arm_pub.publish(msg)

    # ── Navigation helpers ─────────────────────────────────────────────

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            return
        x, y, yaw = self.waypoints[self.waypoint_index]
        self.get_logger().info(
            f"Navigating to waypoint {self.waypoint_index}: ({x:.2f}, {y:.2f})"
        )
        self._send_nav_goal(x, y, yaw)
        self._publish_goal_markers()

    def _publish_goal_markers(self):
        markers: list[Marker] = []
        for i, (x, y, yaw) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.05
            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = 0.3
            m.scale.y = 0.08
            m.scale.z = 0.08
            if i < self.waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.7, 0.3, 0.5
            elif i == self.waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.3, 1.0, 0.5
            m.lifetime.sec = 0
            markers.append(m)
        self.goal_marker_pub.publish(MarkerArray(markers=markers))

    def _send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 server not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))  # returns [x, y, z, w]
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.nav_in_flight = True
        self._nav_ever_sent = True
        self._nav_rejected = False
        self._nav_update = False
        seq = self._nav_seq
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(lambda f: self._nav_goal_response(f, seq))
        return True

    def _nav_goal_response(self, future, seq: int):
        if seq != self._nav_seq:
            return  # stale callback from a cancelled goal
        self.nav_goal_handle = future.result()
        self.nav_in_flight = False
        if not self.nav_goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected by server.")
            self.nav_result_future = None
            self._nav_rejected = True
            return
        self.nav_result_future = self.nav_goal_handle.get_result_async()

    def _nav_feedback_cb(self, feedback_msg):
        self._last_feedback_distance = feedback_msg.feedback.distance_remaining
        self._last_feedback_time = self.get_clock().now().nanoseconds / 1e9

    def _cancel_nav(self):
        self._nav_seq += 1  # invalidates any in-flight callback
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_in_flight = False
        self._nav_rejected = False
        self._nav_update = False
        self._last_feedback_distance = None
        self._last_feedback_time = None

    def _is_near_goal(self) -> bool:
        if self._last_feedback_distance is None or self._last_feedback_time is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_feedback_time > 2.0:
            return False  # feedback stale
        threshold = cast(float, self.get_parameter("avoidance_blind_distance").value)
        return self._last_feedback_distance < threshold

    def _publish_approach_state(self):
        msg = String()
        msg.data = (
            "APPROACH_TARGET_FINAL" if self._is_near_goal() else "APPROACH_TARGET"
        )
        self.robot_state_pub.publish(msg)

    def _is_nav_complete(self) -> bool:
        if not self._nav_ever_sent or self.nav_in_flight:
            return False
        if self._nav_rejected:
            return True

        if self.nav_result_future is None:
            return False
        return self.nav_result_future.done()

    def _nav_succeeded(self) -> bool:
        if not self._is_nav_complete():
            return False
        if self._nav_rejected:
            return False
        try:
            assert self.nav_result_future is not None
            result = self.nav_result_future.result()
        except Exception:
            return False
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    def _nav_update_target(self) -> bool:
        if not self._is_nav_complete():
            return False
        if self._nav_rejected:
            return False
        if self._nav_update is True:
            return True
        return False

    def _nav_aborted(self) -> bool:
        if not self._is_nav_complete():
            return False
        if self._nav_rejected:
            return True
        try:
            assert self.nav_result_future is not None
            result = self.nav_result_future.result()
        except Exception:
            return True
        return result is not None and result.status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        )

    # ── Costmap check ──────────────────────────────────────────────────
    def _publish_approaching_object(
        self, ax, ay, yaw=None, attempt=0, total=8, none=False
    ):
        """
        Publish an approach marker at the current Nav2 approach goal position.

        ax, ay  — goal position in map frame
        yaw     — if provided, orient the small arrow using this heading
        attempt — 0-indexed candidate number being tried (shown in label)
        total   — total candidates available (shown in label)
        none    — if True, delete both markers
        """
        now = self.get_clock().now().to_msg()

        if none:
            for mid in (0, 1):
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = now
                m.ns = "approaching_object"
                m.id = mid
                m.action = Marker.DELETE
                self.approaching_object_pub.publish(m)
            return

        if yaw is not None:
            # Small arrow centered on the goal and oriented by yaw.
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = now
            arrow.ns = "approaching_object"
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = float(ax)
            arrow.pose.position.y = float(ay)
            arrow.pose.position.z = 0.12
            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            arrow.pose.orientation.x = q[0]
            arrow.pose.orientation.y = q[1]
            arrow.pose.orientation.z = q[2]
            arrow.pose.orientation.w = q[3]
            arrow.scale.x = 0.18
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.r = 1.0
            arrow.color.g = 0.55
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            arrow.lifetime.sec = 0
            self.approaching_object_pub.publish(arrow)
        else:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "approaching_object"
            m.id = 0
            m.action = Marker.DELETE
            self.approaching_object_pub.publish(m)

        # Text label above the arrow
        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = now
        label.ns = "approaching_object"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(ax)
        label.pose.position.y = float(ay)
        label.pose.position.z = 1.05
        label.pose.orientation.w = 1.0
        label.scale.z = 0.15
        label.color.r = 1.0
        label.color.g = 0.55
        label.color.b = 0.0
        label.color.a = 1.0
        label.text = f"GOAL {attempt + 1}/{total}"
        label.lifetime.sec = 0
        self.approaching_object_pub.publish(label)

    # ── Costmap check ──────────────────────────────────────────────────

    def _cost_at_goal_ok(self, x: float, y: float) -> bool:
        if self.costmap is None:
            return True
        mx, my = self._world_to_map(x, y)
        w, h = self.costmap.info.width, self.costmap.info.height
        if not (0 <= mx < w and 0 <= my < h):
            return False
        cost = self.costmap.data[my * w + mx]
        if cost >= 50 or cost < 0:
            c_type = (
                self.current_target["type"]
                if self.current_target is not None
                else "NONE"
            )
            self.get_logger().warn(
                f"Approach {c_type}({x:.2f}, {y:.2f}) blocked (cost={cost})."
            )
            return False
        return True

    def _world_to_map(self, x: float, y: float):
        assert self.costmap is not None
        res = self.costmap.info.resolution
        ox, oy = (
            self.costmap.info.origin.position.x,
            self.costmap.info.origin.position.y,
        )
        return int((x - ox) / res), int((y - oy) / res)

    # ── Nav2 lifecycle check ───────────────────────────────────────────

    def _check_nav2_states(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_nav2_check < 1.0:
            return
        self._last_nav2_check = now
        for name, client in self._nav2_clients.items():
            if not client.service_is_ready():
                continue
            req = GetState.Request()
            future = client.call_async(req)
            future.add_done_callback(lambda f, n=name: self._nav2_state_cb(f, n))

    def _nav2_state_cb(self, future, node_name: str):
        try:
            result = future.result()
            self._nav2_states[node_name] = result.current_state.label
        except Exception:
            self._nav2_states[node_name] = "error"
        self.nav2_ready = all(s == "active" for s in self._nav2_states.values())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = Task2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
