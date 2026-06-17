"""Task 2 mission controller.

State machine:
  INIT → PATROL → APPROACH_TARGET → INTERACT → INSPECT_WORKSTATION
       → NAVIGATE_ROOM2_ENTRY → FOLLOW_BLUE_LINE → DONE

Detectors publish PoseStamped on their respective topics; this controller deduplicates
by position and queues pending_targets for approach.
"""

import math
from collections.abc import Sequence
from enum import Enum, auto
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from megatron.report import AnomalyTask, BarrelTask, ReportBuilder, RingTask
from megatron.speech import Speaker
from megatron.tile_anomaly import (
    DetectorConfig,
    TileAnomalyDetector,
    TileAnomalyResult,
    quad_from_contour,
    warp_tile,
)
from megatron.tile_capture import save_tile_capture

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


class State(Enum):
    INIT = auto()
    PATROL = auto()
    APPROACH_TARGET = auto()
    INTERACT = auto()
    RETREATING = auto()
    INSPECT_WORKSTATION = auto()
    NAVIGATE_ROOM2_ENTRY = auto()
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
            pose = cast(Sequence[Any], entry.get("pose"))
            orient = cast(Sequence[Any], entry.get("orientation"))
            if pose and len(pose) >= 2:
                x, y = float(pose[0]), float(pose[1])
            if orient and len(orient) == 4:
                yaw = _quaternion_to_yaw(orient)
        elif isinstance(entry, (list, tuple)):
            seq = cast(Sequence[Any], entry)
            if len(seq) >= 2:
                x, y = float(seq[0]), float(seq[1])
            if len(seq) >= 3:
                yaw = float(seq[2])
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


def _inspection_completion_reason(
    tile_count: int,
    expected_tiles: int,
    front_hit_count: int,
    scan_distance: float,
    max_scan_distance: float,
    elapsed: float,
) -> str | None:
    if tile_count >= expected_tiles:
        return f"{expected_tiles} tiles"
    if front_hit_count >= 3:
        return "front obstacle"
    if scan_distance >= max_scan_distance:
        return "scan distance"
    if elapsed > 70.0:
        return "timeout"
    return None


def _inspection_steering(yaw_correction: float, distance_correction: float) -> float:
    return max(-0.10, min(0.10, yaw_correction + distance_correction))


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
    DEDUP_DISTANCE = 0.5  # metres — two detections within this are the same object
    MIN_UPDATE_DISTANCE = 0.05
    WORKSTATION_TILE_COUNTS = {"red": 4, "green": 5}
    WORKSTATION_SCAN_DISTANCES = {"red": 2.4, "green": 3.0}
    MAX_RETRY_CYCLES = (
        20  # bump distance up to 3× approach_retry_offset before giving up
    )
    WAYPOINT_WAIT_TIME = 0.2  # ms

    # Retreat (Problem A): when an approach parks tighter than this strict
    # cost threshold, back straight out along the line Nav2 just drove
    # before sending the next (possibly far-away) goal — avoids handing
    # Nav2's RegulatedPurePursuitController a near-180° goal that would
    # trigger its own rotate-to-heading in the same tight corner.
    RETREAT_TIGHT_COST_THRESHOLD = 80
    RETREAT_DISTANCE_M = 0.10  # 0.35
    RETREAT_SPEED = -0.08
    RETREAT_MIN_REAR_CLEARANCE_M = 0.15
    RETREAT_TIMEOUT_S = 8.0
    REAR_SCAN_ANGLE = math.pi / 2  # opposite of the front cone used elsewhere (-pi/2)

    def __init__(self):
        super().__init__("task2_controller")

        # --- Parameters ---
        self.declare_parameter("waypoints_file", "waypoints/task.yaml")
        self.declare_parameter("room2_entry_file", "waypoints/bluelinepoint.yaml")
        self.declare_parameter("workstation_file", "waypoints/workstation.yaml")
        self.declare_parameter("face_approach_distance", 0.50)
        self.declare_parameter("barrel_approach_distance", 0.60)
        self.declare_parameter("barrel_lateral_offset", 0.30)
        self.declare_parameter("approach_retry_offset", 0.20)
        self.declare_parameter("approach_stuck_timeout", 15.0)
        self.declare_parameter("qr_wait_timeout", 20.0)
        self.declare_parameter("manual_mode", False)
        self.declare_parameter("avoidance_blind_distance", 0.40)
        self.declare_parameter("capture_tiles", False)
        self.declare_parameter("tile_capture_dir", "/tmp/megatron_tile_captures")
        self.declare_parameter("world_name", "task2")

        wp_file = cast(str, self.get_parameter("waypoints_file").value)
        self.waypoints = load_waypoints_from_yaml(wp_file)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wp_file}")
        room2_file = cast(str, self.get_parameter("room2_entry_file").value)
        room2_entries = load_waypoints_from_yaml(room2_file)
        if len(room2_entries) != 1:
            raise ValueError(
                f"Room 2 entry file must contain exactly one waypoint: {room2_file}"
            )
        self.room2_entry = room2_entries[0]
        self.get_logger().info(
            f"Loaded room 2 entry from {room2_file}: "
            f"({self.room2_entry[0]:.2f}, {self.room2_entry[1]:.2f}) "
            f"yaw={self.room2_entry[2]:.2f}"
        )
        ws_file = cast(str, self.get_parameter("workstation_file").value)
        ws_entries = load_waypoints_from_yaml(ws_file)
        if len(ws_entries) != 2:
            raise ValueError(
                f"Workstation file must contain exactly two waypoints (red, green): {ws_file}"
            )
        self._workstation_approaches: dict[str, tuple[float, float, float]] = {
            "red": ws_entries[0],
            "green": ws_entries[1],
        }
        self.get_logger().info(
            f"Loaded workstation approaches from {ws_file}: "
            f"red=({ws_entries[0][0]:.2f}, {ws_entries[0][1]:.2f}) "
            f"green=({ws_entries[1][0]:.2f}, {ws_entries[1][1]:.2f})"
        )
        self.manual_mode = cast(bool, self.get_parameter("manual_mode").value)
        self._capture_tiles = cast(bool, self.get_parameter("capture_tiles").value)
        self._tile_capture_dir = Path(
            cast(str, self.get_parameter("tile_capture_dir").value)
        ).expanduser()
        self._world_name = cast(str, self.get_parameter("world_name").value)
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
        self._nav_goal_sent_time: float | None = None
        self._room2_nav_attempt = 0
        self._room2_nav_started: float | None = None
        self._room2_goal_sent = False

        # --- Costmap ---
        self.costmap = None
        self.local_costmap = None
        self.keepout_mask = None

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
        self._ring_requestor: str = "Unknown"
        self._barrel_requestor: str = "Unknown"
        self._anomaly_requestor: str = "Unknown"
        self.workstation_poses: dict[str, np.ndarray] = {}

        # --- Report data ---
        self.ring_counts: dict[str, int] = {}
        self.barrel_report: list[dict] = []
        self.tile_results: list[dict] = []

        # --- Approach tracking ---
        self.current_target: dict | None = None
        self._approach_attempt = 0
        self._patrol_complete = False
        self._waypoint_arrive_time: float | None = None

        # --- Retreat tracking (Problem A) ---
        self._tight_parking = False
        self._retreat_start_xy: np.ndarray | None = None
        self._retreat_start_time: float | None = None

        # --- Interact tracking ---
        self._qr_task_raw: str | None = None
        self._qr_wait_start_time: float | None = None
        self.ring_task = False
        self.barrel_task = False
        self.barrel_task_done = False
        self.anomaly_task = False
        self.workstation_color: str | None = None  # "red" or "green" from QR

        # --- Inspection state ---
        self._inspection_phase = 0
        self._inspection_color: str | None = None
        self._last_scan: LaserScan | None = None
        self._yellow_seen = False
        self._tile_index = 0
        self._phase_start_time: float | None = None
        self._tile_pause_start: float | None = None
        self._tile_scored_this_pause = False
        self._arm_settle_until: float | None = None
        self._prev_inspection_phase: int = -1
        self._inspection_scan_yaw: float | None = None
        self._inspection_side_target: float | None = None
        self._inspection_side_samples: list[float] = []
        self._tile_visible = False
        self._tile_hit_count = 0
        self._tile_miss_count = 0
        self._inspection_scan_start: np.ndarray | None = None
        self._inspection_front_hit_count = 0

        # --- Top camera ---
        self._cv_bridge = CvBridge()
        self._last_top_frame: np.ndarray | None = None
        self._last_top_stamp_ns = 0
        _share = Path(get_package_share_directory("megatron"))
        _ref_root = _share / "assets" / "tiles" / "reference_good"
        _model_path = _share / "assets" / "tiles" / "model.yaml"
        _detector_config = DetectorConfig.from_yaml(_model_path)
        _world_ref_root = _ref_root / self._world_name
        _ref_red = (
            sorted((_world_ref_root / "red").rglob("*.png"))
            if (_world_ref_root / "red").is_dir()
            else []
        )
        _ref_green = (
            sorted((_world_ref_root / "green").rglob("*.png"))
            if (_world_ref_root / "green").is_dir()
            else []
        )
        if not _ref_red:
            _ref_red = sorted(
                p for p in _ref_root.rglob("*.png") if "/red/" in p.as_posix()
            )
        if not _ref_green:
            _ref_green = sorted(
                p for p in _ref_root.rglob("*.png") if "/green/" in p.as_posix()
            )
        self._tile_detector_red = TileAnomalyDetector.from_paths(
            _ref_red, _detector_config
        )
        self._tile_detector_green = TileAnomalyDetector.from_paths(
            _ref_green, _detector_config
        )
        self.get_logger().info(
            f"Tile detectors loaded: red={len(_ref_red)} green={len(_ref_green)} refs "
            f"from {_world_ref_root}"
        )
        if self._capture_tiles:
            self.get_logger().info(
                f"Tile capture enabled: {self._tile_capture_dir} "
                f"(world={self._world_name})"
            )

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
        self._snapshot_bottom_pub = self.create_publisher(
            String, "/megatron/snapshot/bottom", 10
        )
        self.goal_marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)
        self.approaching_object_pub = self.create_publisher(
            Marker, "/approaching_object", 10
        )
        self._ws_debug_live_pub = self.create_publisher(
            Image, "/workstation_debug/live", 1
        )
        self._ws_debug_raw_pub = self.create_publisher(
            Image, "/workstation_debug/raw", 1
        )
        self._ws_debug_thresh_pub = self.create_publisher(
            Image, "/workstation_debug/thresh", 1
        )
        self._ws_debug_warped_pub = self.create_publisher(
            Image, "/workstation_debug/warped", 1
        )
        self._ws_debug_phase_pub = self.create_publisher(
            String, "/workstation_debug/phase", 1
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
        self.create_subscription(
            Image,
            "/top_camera/rgb/preview/image_raw",
            self._top_camera_cb,
            qos_profile_sensor_data,
        )
        costmap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, costmap_qos
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._local_costmap_cb,
            costmap_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/keepout_filter_mask",
            self._keepout_mask_cb,
            costmap_qos,
        )
        self.create_subscription(String, "/robot_state", self._robot_state_cb, 10)

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
        id = parts[2] if len(parts) > 2 else "ID_NONDE"

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
                    old_pos = np.array(f["pos"])
                    f["pos"] = pos
                    f["normal"] = (nx, ny)
                    f["last_seen"] = now
                    f["label"] = label

                    self.current_target["pos"] = pos
                    self.current_target["normal"] = (nx, ny)
                    self.current_target["last_seen"] = now
                    if np.linalg.norm(pos[:2] - old_pos[:2]) > self.MIN_UPDATE_DISTANCE:
                        self.get_logger().info("Seting up to recalculate approach")
                        self._nav_update = True
                    return

                elif np.linalg.norm(pos - f["pos"]) < self.DEDUP_DISTANCE:
                    self.get_logger().info(
                        f"Requing face {f['id']} since the new detected location is more far away"
                    )
                    f["pos"] = pos
                    f["normal"] = (nx, ny)
                    f["last_seen"] = now
                    f["label"] = label

                    # if not f.get("approached", False) and self.state == State.PATROL:
                    #     self._requeue_if_not_pending("face", f)
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
            "last_seen": now,
        }
        self.found_faces.append(entry)
        self.get_logger().info(f"New face {label}(ID:{id}) detected ")

    def _ring_cb(self, msg: PoseStamped):
        parts = msg.header.frame_id.split("|")
        color = parts[1] if len(parts) > 1 else "unknown"
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        now = self.get_clock().now()

        for r in self.found_rings:
            if np.linalg.norm(pos - r["pos"]) < self.DEDUP_DISTANCE:
                if (
                    self.current_target is not None
                    and self.current_target.get("type") == "ring"
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    > self.MIN_UPDATE_DISTANCE
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    < self.DEDUP_DISTANCE
                ):
                    self.current_target["pos"] = pos
                    self.current_target["last_seen"] = now
                    self._nav_update = True

                # elif np.linalg.norm(pos - r["pos"]) < self.DEDUP_DISTANCE:
                #     self.get_logger().info(
                #         f"Requing Ring {r['color']} since the new detected location is more far away"
                #     )
                #     # if not f.get("approached", False) and self.state == State.PATROL:
                #     #     self._requeue_if_not_pending("face", f)
                #     return
                r["pos"] = pos
                r["last_seen"] = now
                return

        self.get_logger().info(f"New ring ({color}) at ({pos[0]:.2f}, {pos[1]:.2f})")
        self.found_rings.append({
            "type": "ring",
            "pos": pos,
            "color": color,
            "last_seen": self.get_clock().now(),
        })
        self.ring_counts[color] = self.ring_counts.get(color, 0) + 1

    def _cylinder_cb(self, msg: PoseStamped):
        parts = msg.header.frame_id.split("|")
        color = parts[1] if len(parts) > 1 else "unknown"
        orientation = parts[2] if len(parts) > 2 else "vertical"
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        now = self.get_clock().now()

        for b in self.found_barrels:
            if np.linalg.norm(pos - b["pos"]) < self.DEDUP_DISTANCE:
                if (
                    self.current_target is not None
                    and self.current_target.get("type") == "barrel"
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    > self.MIN_UPDATE_DISTANCE
                    and np.linalg.norm(pos - np.array(self.current_target["pos"]))
                    < self.DEDUP_DISTANCE
                ):
                    self.current_target["pos"] = pos
                    self.current_target["last_seen"] = now
                    self._nav_update = True
                    # self.get_logger().info("Set flag Nav Update to TRUE from barrel cb")

                # elif np.linalg.norm(pos - b["pos"]) < self.DEDUP_DISTANCE:
                # self.get_logger().info(
                #     f"Requing barrel {b['color']} since the new detected location is more far away"
                # )
                # if not f.get("approached", False) and self.state == State.PATROL:
                #     self._requeue_if_not_pending("face", f)
                #     return
                # if (
                #     b["orientation"] == "horizontal"
                #     and not b.get("approached", False)
                #     and self.state == State.PATROL
                # ):
                #     self._requeue_if_not_pending("barrel", b)
                b["pos"] = pos
                b["last_seen"] = now
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
            "detection_rx": self.current_pose.position.x if self.current_pose else None,
            "detection_ry": self.current_pose.position.y if self.current_pose else None,
        }
        self.found_barrels.append(entry)

    def _workstation_cb(self, msg: Marker):
        color = msg.ns  # "red" or "green"
        self.workstation_poses[color] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
        ])
        # self.get_logger().info(
        #     f"Workstation '{color}' at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        # )

    def _qr_cb(self, msg: String):
        self._qr_task_raw = msg.data
        self.get_logger().info(f"QR received: {msg.data!r}")

    def _robot_state_cb(self, msg: String):
        try:
            new_state = State[msg.data]
        except KeyError:
            self.get_logger().warn(f"Unknown state from /robot_state: {msg.data!r}")
            return

        if new_state == self.state:
            return

        self.get_logger().info(
            f"Manual state override from /robot_state: {self.state.name} → {new_state.name}"
        )
        self._cancel_nav()
        self._stop_cmd_vel()

        if new_state == State.INSPECT_WORKSTATION:
            if self.assigned_task is None and self._qr_task_raw not in (None, ""):
                task_token = _parse_qr_task(self._qr_task_raw)
                self._qr_task_raw = None
                if task_token and task_token not in ("nothing", "report"):
                    self.assigned_task = task_token
                    self.get_logger().info(
                        f"Eagerly parsed QR: assigned_task = {task_token!r}"
                    )

            color = (
                self.assigned_task.split("_")[1]
                if self.assigned_task and self.assigned_task.startswith("defects")
                else next(iter(self.workstation_poses.keys()), None)
            )
            if color is None:
                self.get_logger().error(
                    "No task assigned and no workstations detected — "
                    "can't enter INSPECT_WORKSTATION"
                )
                return
            self._start_inspection(color)

        self._transition(new_state)

    def _scan_cb(self, msg: LaserScan):
        self._last_scan = msg

    def _yellow_seen_cb(self, msg: Bool):
        self._yellow_seen = msg.data

    def _top_camera_cb(self, msg: Image):
        frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self._last_top_frame = frame
        self._last_top_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if self.state == State.INSPECT_WORKSTATION:
            try:
                # Annotate with ROI mean brightness so we can see the trigger threshold
                h, w = frame.shape[:2]
                roi = frame[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4]
                brightness = float(roi.mean())
                annotated = frame.copy()
                cv2.rectangle(
                    annotated,
                    (w // 4, h // 4),
                    (3 * w // 4, 3 * h // 4),
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    annotated,
                    f"ROI mean: {brightness:.1f}",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                if self._inspection_phase == 5:
                    tile_box, _ = self._detect_tile(frame)
                    if tile_box is not None:
                        cv2.drawContours(
                            annotated,
                            [tile_box.astype(np.int32)],
                            -1,
                            (0, 255, 0),
                            2,
                        )
                        cv2.putText(
                            annotated,
                            "TILE",
                            (8, 42),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )
                self._ws_debug_live_pub.publish(
                    self._cv_bridge.cv2_to_imgmsg(annotated, "bgr8")
                )
            except Exception:
                pass
        if self.state == State.INSPECT_WORKSTATION and self._inspection_phase in (4, 5):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h = frame.shape[0]
            yellow = cv2.inRange(
                hsv[-h // 8 :, :],
                np.array([18, 100, 80], dtype=np.uint8),
                np.array([35, 255, 255], dtype=np.uint8),
            )
            yellow_ratio = cv2.countNonZero(yellow) / yellow.size
            self._yellow_seen = yellow_ratio >= 0.05
            self.get_logger().info(
                f"[inspection] yellow ROI={yellow_ratio:.3f} seen={self._yellow_seen}",
                throttle_duration_sec=0.5,
            )

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def _local_costmap_cb(self, msg: OccupancyGrid):
        self.local_costmap = msg

    def _keepout_mask_cb(self, msg: OccupancyGrid):
        self.keepout_mask = msg

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
        self.pending_targets.append(track)

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
        elif self.state == State.RETREATING:
            self._handle_retreating()
        elif self.state == State.INSPECT_WORKSTATION:
            self._handle_inspection()
        elif self.state == State.NAVIGATE_ROOM2_ENTRY:
            self._handle_room2_navigation()
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
        if new_state == State.FOLLOW_BLUE_LINE:
            self._generate_report()
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

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._waypoint_arrive_time is None:
            self._waypoint_arrive_time = now
            return
        if now - self._waypoint_arrive_time < self.WAYPOINT_WAIT_TIME:
            return
        self._waypoint_arrive_time = None

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
                self.pending_targets.append(f)
                self.get_logger().info(
                    f"Queuing face {f.get('id')} at ({f['pos'][0]:.2f}, {f['pos'][1]:.2f})"
                )

        if not self.pending_targets:
            self.get_logger().info("No targets to approach — heading to room 2.")
            self._pub_arm("look_down")
            self._start_room2_navigation()
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
                self.get_logger().info(f"approach is updated for {self.current_target}")
                self._approach_attempt = 0
                self._send_approach(self.current_target, self._approach_attempt)
            return

        if not self._is_nav_complete():
            if self._approach_is_stuck():
                self.get_logger().warn(
                    "Nav2 stuck recovering on approach goal — cancelling and resending."
                )
                self._cancel_nav()
                self._handle_approach_failure()
            return

        if self._nav_succeeded():
            self._publish_approaching_object(0.0, 0.0, none=True)
            self._tight_parking = self._is_parking_tight()
            self._transition(State.INTERACT)
            self._start_interact()
            return

        if self._nav_aborted():
            self.get_logger().info("approach aborted resending approach")
            self._handle_approach_failure()

    def _approach_is_stuck(self) -> bool:
        """True if the in-flight approach goal has been pending too long.

        Nav2's own recovery behaviours (spin/backup) can loop forever if the
        robot is wedged against an obstacle, never returning a terminal
        status — so we time out independently instead of waiting on
        _nav_aborted()/_nav_succeeded().
        """
        if self._nav_goal_sent_time is None:
            return False
        timeout = cast(float, self.get_parameter("approach_stuck_timeout").value)
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._nav_goal_sent_time
        return elapsed > timeout

    def _handle_approach_failure(self):
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
        self.get_logger().info(f"Sending approach to {_type} at {target['pos']}")

        if _type == "face":
            n_candidates = 8
            base_dist = cast(float, self.get_parameter("face_approach_distance").value)

            def _gen(d):
                return self._face_approach_candidates(
                    target["pos"], target["normal"], distance=d
                )

        else:
            n_candidates = len(self.BARREL_FAN_DEGREES)
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

        # All candidates at fixed approach distance; angular offsets spread the fan
        def f(_x):
            return 1.0

        degrees = [
            0,
            5,
            -5,
            10,
            -10,
            15,
            -15,
            20,
            -20,
            25,
            -25,
            30,
            -30,
            40,
            -40,
            50,
            -50,
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

    BARREL_FAN_DEGREES = [0, 15, -15]  # barrels are cylindrical — only need a small fan

    def _barrel_approach_candidates(self, target: dict, distance=None):
        """Small fan of candidates around the primary approach direction.

        Barrels are radially symmetric, so unlike faces we don't need a wide
        8-way fan — a few small angular offsets around the direction the
        barrel was first seen from are enough to route around a locally
        blocked costmap cell.
        """
        pos = target["pos"]
        dist = (
            distance
            if distance is not None
            else cast(float, self.get_parameter("barrel_approach_distance").value)
        )
        lateral = cast(float, self.get_parameter("barrel_lateral_offset").value)
        px, py = float(pos[0]), float(pos[1])

        if target.get("orientation", "vertical") == "vertical":
            # Approach from the direction the barrel was first seen from.
            # Using detection-time pose avoids trajectories that cross walls
            # when the robot has moved since detection.
            rx = target.get("detection_rx")
            ry = target.get("detection_ry")
            if rx is None and self.current_pose is not None:
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
            if rx is not None and ry is not None:
                dx, dy = px - rx, py - ry
                d = math.sqrt(dx**2 + dy**2)
                if d > 1e-3:
                    dx, dy = dx / d, dy / d
                else:
                    dx, dy = 1.0, 0.0
            else:
                dx, dy = 1.0, 0.0
            base_angle = math.atan2(dy, dx)
            return [
                (
                    px - math.cos(base_angle + o) * dist,
                    py - math.sin(base_angle + o) * dist,
                    math.atan2(math.sin(base_angle + o), math.cos(base_angle + o)),
                )
                for o in (math.radians(d) for d in self.BARREL_FAN_DEGREES)
            ]

        # Horizontal barrel: perpendicular to axis + lateral shift
        q = target.get("quat")
        if q is not None:
            ax_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )
        else:
            ax_yaw = 0.0
        # Two perpendiculars; pick the one toward robot at detection time
        perp_x, perp_y = -math.sin(ax_yaw), math.cos(ax_yaw)
        rx = target.get("detection_rx")
        ry = target.get("detection_ry")
        if rx is None and self.current_pose is not None:
            rx, ry = self.current_pose.position.x, self.current_pose.position.y
        if rx is not None and ry is not None:
            if (rx - px) * perp_x + (ry - py) * perp_y < 0:
                perp_x, perp_y = -perp_x, -perp_y
        base_angle = math.atan2(perp_y, perp_x)
        candidates = []
        for d in self.BARREL_FAN_DEGREES:
            angle = base_angle + math.radians(d)
            cx, cy = math.cos(angle), math.sin(angle)
            # Lateral shift to robot-right (cross product of approach direction and Z-up)
            right_x, right_y = -cy, cx  # rotate approach 90° CW in XY
            approach_x = px + cx * dist + right_x * lateral
            approach_y = py + cy * dist + right_y * lateral
            yaw = math.atan2(-cy, -cx)
            candidates.append((approach_x, approach_y, yaw))
        return candidates
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
            self._qr_wait_start_time = self.get_clock().now().nanoseconds / 1e9

        elif target["type"] == "barrel":
            self.speaker.speak(f"Inspecting {target['color']} barrel.")
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
                barrel_id = len(self.barrel_report) + 1
                msg = String()
                msg.data = f"{barrel_id}_leak"
                self._snapshot_bottom_pub.publish(msg)
                self.speaker.speak("Alert! Alert! This barrel is leaking!")
            else:
                self.speaker.speak("Barrel OK.")

            self.barrel_report.append({
                "id": len(self.barrel_report) + 1,
                "color": target.get("color", "unknown"),
                "orientation": orientation,
                "leaking": leaking,
                "spill_count": spill_count,
                "spill_threshold": 4000,
                "pos": target["pos"].tolist(),
            })
            self._mark_approached(target)
            self._resume_patrol()
            return

        # Face: wait for QR
        if (
            self._qr_task_raw is None or self._qr_task_raw == ""
        ):  # if qr_task is not seen it is failed
            timeout = cast(float, self.get_parameter("qr_wait_timeout").value)
            start = self._qr_wait_start_time
            if (
                start is not None
                and self.get_clock().now().nanoseconds / 1e9 - start > timeout
            ):
                self.get_logger().warn(
                    "Timed out waiting for QR task — giving up on this worker."
                )
                self.speaker.speak("I couldn't read your task. Moving on.")
                self._qr_wait_start_time = None
                self._mark_approached(target)
                self._resume_patrol()
                return
            self.get_logger().info("Waitting for qr task")
            return

        self._qr_wait_start_time = None
        qr_raw = self._qr_task_raw
        task_token = _parse_qr_task(qr_raw)
        self._qr_task_raw = None

        if task_token == "emergency":
            self.get_logger().info(
                "Emergency Intelligence Incinerator triggered - falling down Glados oven!"
            )
            self.speaker.speak("Emergency testing! Initiating system shutdown...")
            self._transition(State.DONE)
            rclpy.shutdown()
            return

        if (
            task_token is not None
            and task_token != "nothing"
            and task_token != "emergency"
        ):
            requestor = (target.get("label") or "Unknown") if target else "Unknown"
            if "defects" in task_token:
                self._anomaly_requestor = requestor
                self.anomaly_task = True
                if "red" in task_token:
                    self.workstation_color = "red"
                elif "green" in task_token:
                    self.workstation_color = "green"
                self.get_logger().info(
                    f"Workstation color set to '{self.workstation_color}' from QR."
                )

            elif task_token == "barrels":
                self._barrel_requestor = requestor
                self.barrel_task = True

            elif task_token == "rings":
                self._ring_requestor = requestor
                self.ring_task = True

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
        if self._tight_parking:
            self._tight_parking = False
            self._start_retreat()
            return
        if self._patrol_complete:
            self._next_post_patrol_target()
        else:
            self._transition(State.PATROL)
            self._send_next_waypoint()

    # ── RETREATING ────────────────────────────────────────────────────

    def _is_parking_tight(self) -> bool:
        """True if the just-parked pose is tighter than the strict threshold.

        `_cost_at_goal_ok` accepts goals up to cost 150 (raised from 80 to
        let the robot park close enough to read QR codes). That means some
        accepted parks are right against a wall/corner — too tight for
        Nav2's own Spin/BackUp recovery to execute later if the next goal
        requires a sharp turn-and-exit. Re-check the actual settled pose
        against the old strict threshold so we know whether to retreat
        before sending the next goal.
        """
        xy = self._current_xy()
        if xy is None:
            return False
        x, y = float(xy[0]), float(xy[1])
        threshold = self.RETREAT_TIGHT_COST_THRESHOLD
        if self.costmap is not None:
            ok, _ = self._cost_ok_on(
                self.costmap, x, y, out_of_bounds_ok=False, threshold=threshold
            )
            if not ok:
                return True
        if self.local_costmap is not None:
            ok, _ = self._cost_ok_on(
                self.local_costmap, x, y, out_of_bounds_ok=True, threshold=threshold
            )
            if not ok:
                return True
        return False

    def _start_retreat(self):
        self._retreat_start_xy = self._current_xy()
        self._retreat_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            "Tight parking detected — retreating before sending next goal."
        )
        self._transition(State.RETREATING)

    def _handle_retreating(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - (self._retreat_start_time or now)
        current_xy = self._current_xy()
        traveled = (
            float(np.linalg.norm(current_xy - self._retreat_start_xy))
            if (current_xy is not None and self._retreat_start_xy is not None)
            else 0.0
        )

        if traveled >= self.RETREAT_DISTANCE_M or elapsed > self.RETREAT_TIMEOUT_S:
            self._stop_cmd_vel()
            self.get_logger().info(
                f"Retreat complete: traveled={traveled:.2f}m elapsed={elapsed:.1f}s"
            )
            self._resume_patrol()
            return

        rear_clearance = self._min_scan_distance(
            self.REAR_SCAN_ANGLE, math.radians(20.0)
        )
        if (
            rear_clearance is not None
            and rear_clearance < self.RETREAT_MIN_REAR_CLEARANCE_M
        ):
            self._stop_cmd_vel()
            self.get_logger().warn(
                f"Retreat stopped early: rear clearance {rear_clearance:.2f}m"
            )
            self._resume_patrol()
            return

        self._send_cmd_vel(self.RETREAT_SPEED, 0.0)

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
            # barrel task
            if self.barrel_task and not self.barrel_task_done:
                self.barrel_task_done = True
                for b in self.found_barrels:
                    if not b.get("approached", False):
                        self._requeue_if_not_pending("barrel", b)
                        self.get_logger().info(
                            f"Queuing {b.get('orientation')} {b.get('color')} barrel "
                            f"at ({b['pos'][0]:.2f}, {b['pos'][1]:.2f})"
                        )

                if self.pending_targets:  # pending targets is not empty
                    self.speaker.speak("Inspecting barrels")
                    self._next_post_patrol_target()
                    return
                else:  # pending targets are empty, which means None barrels were found
                    # continue with the workstation task
                    pass

            # workstation
            if (
                self.workstation_color is not None
                and self.workstation_color in self.workstation_poses
            ):
                color = self.workstation_color
                self.get_logger().info(
                    f"Post-patrol done — starting {color} workstation inspection."
                )
                self._start_inspection(color)
                self._transition(State.INSPECT_WORKSTATION)
                return
            self.get_logger().info(
                "All post-patrol targets handled — heading to room 2."
            )
            self._pub_arm("look_down")
            self._start_room2_navigation()

    # ── INSPECT_WORKSTATION ───────────────────────────────────────────

    def _start_inspection(self, color: str):
        self._inspection_color = color
        self._inspection_phase = 0
        self._tile_index = 0
        self._yellow_seen = False
        self._inspection_scan_yaw = None
        self._inspection_side_target = None
        self._inspection_side_samples = []
        self._tile_visible = False
        self._tile_hit_count = 0
        self._tile_miss_count = 0
        self._inspection_scan_start = None
        self._inspection_front_hit_count = 0
        self._prev_inspection_phase = -1
        self._phase_start_time = self.get_clock().now().nanoseconds / 1e9

        approach_x, approach_y, approach_yaw = self._workstation_approaches[color]
        self.get_logger().info(
            f"[inspection] navigating to {color} workstation approach: "
            f"({approach_x:.2f}, {approach_y:.2f}) yaw={approach_yaw:.2f}"
        )
        self._send_nav_goal(approach_x, approach_y, approach_yaw)

    def _handle_inspection(self):
        if self._phase_start_time is None:
            return
        self._ws_debug_phase_pub.publish(
            String(
                data=f"phase={self._inspection_phase} color={self._inspection_color}"
            )
        )
        if self._inspection_phase != self._prev_inspection_phase:
            self.get_logger().info(
                f"[inspection] → phase {self._inspection_phase} (was {self._prev_inspection_phase})"
            )
            self._prev_inspection_phase = self._inspection_phase
        # Phase 0: navigate to workstation (Nav2) — 90 s timeout
        if self._inspection_phase == 0:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self._phase_start_time
            if not self._is_nav_complete() and elapsed < 90.0:
                return
            if elapsed >= 90.0:
                self._abort_inspection("phase 0 navigation timed out")
                return
            if not self._nav_succeeded():
                self._abort_inspection(
                    f"phase 0 navigation failed with status {self._nav_status_name()}"
                )
                return

            pose = self._current_xy()
            yaw = self._current_yaw()
            pose_text = (
                f"({pose[0]:.2f}, {pose[1]:.2f})" if pose is not None else "unknown"
            )
            yaw_text = f"{yaw:.3f}" if yaw is not None else "unknown"
            self.get_logger().info(
                f"[inspection] phase 0 nav succeeded in {elapsed:.1f}s; "
                f"pose={pose_text} yaw={yaw_text}"
            )
            # Clear our completed action before direct velocity control begins.
            self._cancel_nav()
            self._stop_cmd_vel()
            arm_pose = "look_at_belt_right"
            self.get_logger().info(f"[inspection] moving top camera to {arm_pose}")
            self._pub_arm(arm_pose)
            self._arm_settle_until = now + 5.0
            self._inspection_phase = 1
            self._phase_start_time = now
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._phase_start_time

        # Phase 1 — Approach to a 0.60 m front LiDAR stand-off.
        if self._inspection_phase == 1:
            if self._arm_settle_until is not None and now < self._arm_settle_until:
                return  # wait for arm to finish swinging before driving
            if self._arm_settle_until is not None:
                self._arm_settle_until = None
                self._phase_start_time = now
                elapsed = 0.0
                self.get_logger().info(
                    "[inspection] arm settled; starting wall approach"
                )

            front_distance = self._min_scan_distance(-math.pi / 2, math.radians(30.0))
            if front_distance is None:
                self._stop_cmd_vel()
                self.get_logger().warn(
                    "[inspection] phase 1 waiting for valid front LiDAR",
                    throttle_duration_sec=1.0,
                )
                return

            self.get_logger().info(
                f"[inspection] phase 1 front={front_distance:.3f}m elapsed={elapsed:.1f}s",
                throttle_duration_sec=0.5,
            )
            if front_distance <= 0.60 or elapsed > 12.0:
                self._stop_cmd_vel()
                reason = "wall reached" if front_distance <= 0.60 else "timeout"
                self.get_logger().info(
                    f"[inspection] phase 1 complete: {reason}, front={front_distance:.3f}m"
                )
                self._inspection_phase = 2
                self._phase_start_time = now
            else:
                speed = 0.03 if front_distance <= 0.80 else 0.08
                self._send_cmd_vel(speed, 0.0)
            return

        # Phase 2 — Turn parallel to the belt in the direction of its center.
        if self._inspection_phase == 2:
            target_yaw = self._inspection_target_yaw()
            if self.current_pose is not None:
                q = self.current_pose.orientation
                cur_yaw = _quaternion_to_yaw([q.w, q.x, q.y, q.z])
                error = _normalize_angle(target_yaw - cur_yaw)
                self.get_logger().info(
                    f"[inspection] phase 2 yaw={cur_yaw:.3f} "
                    f"target={target_yaw:.3f} error={error:+.3f}",
                    throttle_duration_sec=0.5,
                )
                if abs(error) < 0.10:
                    self._stop_cmd_vel()
                    self.get_logger().info(
                        f"[inspection] phase 2 aligned within tolerance; "
                        f"yaw error={error:+.3f} rad"
                    )
                    self._inspection_phase = 3
                    self._phase_start_time = now
                elif elapsed > 15.0:
                    self._abort_inspection(
                        f"phase 2 could not align to belt; yaw error={error:+.3f} rad"
                    )
                else:
                    gain = 0.6 if abs(error) < 0.30 else 1.0
                    self._send_cmd_vel(
                        0.0,
                        max(-0.4, min(0.4, gain * error)),
                    )
            elif elapsed > 15.0:
                self._abort_inspection("phase 2 has no current robot pose")
            return

        # Phase 3 — Capture the map-aligned scan heading and side distance.
        # Capture the map-aligned scan heading and (for green only) calibrate the
        # lateral distance target. Red's calibration position sits on structural
        # gaps that produce unreliable readings; red relies on yaw-only control.
        if self._inspection_phase == 3:
            self._stop_cmd_vel()
            self._inspection_scan_yaw = self._inspection_target_yaw()
            if self._inspection_color == "green":
                sample = self._belt_side_distance()
                if sample is not None:
                    self._inspection_side_samples.append(sample)
            elapsed_p3 = now - self._phase_start_time
            if elapsed_p3 < 0.5:
                return
            if self._inspection_color == "green" and self._inspection_side_samples:
                self._inspection_side_target = float(
                    np.median(self._inspection_side_samples)
                )
            else:
                self._inspection_side_target = None
            self._stop_cmd_vel()
            self._yellow_seen = False
            self._tile_pause_start = None
            self._tile_visible = False
            self._tile_hit_count = 0
            self._tile_miss_count = 0
            self._inspection_front_hit_count = 0
            self._inspection_scan_start = self._current_xy()
            self.get_logger().info(
                "[inspection] phase 3 calibrated: "
                f"scan_yaw={self._inspection_scan_yaw} "
                f"side_target={self._inspection_side_target} "
                f"n_samples={len(self._inspection_side_samples)} "
                f"scan_start={self._inspection_scan_start}"
            )
            self._inspection_phase = 5
            self._phase_start_time = now
            return

        # Phase 5 — Forward tile scan (contour enter/leave + anomaly detection)
        if self._inspection_phase == 5:
            expected_tiles = self.WORKSTATION_TILE_COUNTS.get(
                self._inspection_color or "", 4
            )
            max_scan_distance = self.WORKSTATION_SCAN_DISTANCES.get(
                self._inspection_color or "", 2.4
            )
            scan_distance = self._inspection_scan_distance()
            front_distance = self._min_scan_distance(-math.pi / 2, math.radians(15.0))
            if (
                scan_distance >= 1.0
                and front_distance is not None
                and front_distance <= 0.50
            ):
                self._inspection_front_hit_count += 1
            else:
                self._inspection_front_hit_count = 0

            self.get_logger().info(
                f"[inspection] phase 5 progress: "
                f"distance={scan_distance:.2f}/{max_scan_distance:.2f}m "
                f"tiles={self._tile_index}/{expected_tiles} front={front_distance} "
                f"front_hits={self._inspection_front_hit_count}/3 "
                f"yellow={self._yellow_seen} elapsed={elapsed:.1f}s",
                throttle_duration_sec=1.0,
            )
            completion_reason = _inspection_completion_reason(
                self._tile_index,
                expected_tiles,
                self._inspection_front_hit_count,
                scan_distance,
                max_scan_distance,
                elapsed,
            )
            if completion_reason is not None:
                self._stop_cmd_vel()
                self.get_logger().info(
                    f"[inspection] phase 5 complete: {completion_reason}, "
                    f"tiles={self._tile_index} "
                    f"distance={scan_distance:.2f}m"
                )
                self._start_room2_navigation()
                return

            if self._tile_pause_start is not None:
                pause_elapsed = now - self._tile_pause_start
                if pause_elapsed >= 0.3 and self._last_top_frame is not None:
                    if not self._tile_scored_this_pause:
                        if self._capture_tiles:
                            self._capture_current_tile(self._last_top_frame)
                        _result = self._score_tile(self._last_top_frame)
                        self.tile_results.append({
                            "station": self._inspection_color,
                            "tile_id": self._tile_index,
                            "status": _result.status,
                            "defect_area": _result.defect_area,
                            "defect_ratio": _result.defect_ratio,
                        })
                        self._tile_index += 1
                        self._tile_scored_this_pause = True
                        self.get_logger().info(
                            f"Tile {self._tile_index - 1}: status={_result.status} "
                            f"area={_result.defect_area} reason={_result.reason}"
                        )
                    if pause_elapsed >= 2.0:
                        self._tile_pause_start = None
                        self._tile_scored_this_pause = False
                return

            if self._last_top_frame is not None:
                tile_box, _ = self._detect_tile(self._last_top_frame)
                if tile_box is not None:
                    self._tile_hit_count += 1
                    self._tile_miss_count = 0
                else:
                    self._tile_miss_count += 1
                    self._tile_hit_count = 0

                tile_centered = False
                if tile_box is not None:
                    tile_center_x = float(tile_box[:, 0].mean())
                    frame_width = self._last_top_frame.shape[1]
                    tile_centered = (
                        0.42 * frame_width <= tile_center_x <= 0.58 * frame_width
                    )

                if (
                    not self._tile_visible
                    and self._tile_hit_count >= 3
                    and tile_centered
                ):
                    self._tile_visible = True
                    self.get_logger().info(
                        f"[phase5] tile {self._tile_index} centered; pausing to score"
                    )
                    self._stop_cmd_vel()
                    self._tile_pause_start = now
                    self._tile_scored_this_pause = False
                    return
                if self._tile_visible and self._tile_miss_count >= 5:
                    self._tile_visible = False
                    self._tile_miss_count = 0
                    self.get_logger().info("[phase5] tile left view")

            self._send_cmd_vel(0.05, self._belt_follow_correction())
            return

    # ── NAVIGATE_ROOM2_ENTRY ─────────────────────────────────────────

    def _start_room2_navigation(self):
        self._stop_cmd_vel()
        self._cancel_nav()
        # Blue-line following depends on this calibrated downward camera view.
        # Centralizing it here covers inspection success, abort, and skip paths.
        self._pub_arm("look_down")
        self._room2_nav_attempt = 0
        self._room2_goal_sent = False
        self._transition(State.NAVIGATE_ROOM2_ENTRY)
        self._send_room2_entry_goal()

    def _send_room2_entry_goal(self):
        x, y, yaw = self.room2_entry
        self._room2_nav_started = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"Navigating to room 2 entry (attempt {self._room2_nav_attempt + 1}/2): "
            f"({x:.2f}, {y:.2f}) yaw={yaw:.2f}"
        )
        self._room2_goal_sent = self._send_nav_goal(x, y, yaw)

    def _handle_room2_navigation(self):
        now = self.get_clock().now().nanoseconds / 1e9
        timed_out = (
            self._room2_nav_started is not None and now - self._room2_nav_started > 90.0
        )
        if self._room2_goal_sent and not timed_out and not self._is_nav_complete():
            return

        if self._room2_goal_sent and self._nav_succeeded():
            self.get_logger().info(
                "Reached room 2 entry — starting blue-line following."
            )
            self._transition(State.FOLLOW_BLUE_LINE)
            return

        status = "TIMEOUT" if timed_out else self._nav_status_name()
        if self._room2_nav_attempt < 1:
            self._room2_nav_attempt += 1
            self.get_logger().warn(
                f"Room 2 entry navigation failed ({status}); retrying once."
            )
            self._cancel_nav()
            self._send_room2_entry_goal()
            return

        self.get_logger().error(
            f"Room 2 entry navigation failed after two attempts ({status}); "
            "falling back to blue-line following."
        )
        self._cancel_nav()
        self._transition(State.FOLLOW_BLUE_LINE)

    # ── FOLLOW_BLUE_LINE ──────────────────────────────────────────────

    def _handle_follow_blue_line(self):
        # blue_line_follower.py activates on /robot_state == "FOLLOW_BLUE_LINE"
        if self._qr_task_raw is not None:
            task_token = _parse_qr_task(self._qr_task_raw)
            if task_token == "report":
                self._qr_task_raw = None
                self.get_logger().info("CTO QR received — mission complete.")
                self._transition(State.DONE)
                self.speaker.speak("Inspection complete. Report delivered.")

    # ── DONE ─────────────────────────────────────────────────────────

    # (no periodic action; _tick does nothing in DONE)

    # ── Report ────────────────────────────────────────────────────────

    def _generate_report(self):
        """Write a PDF inspection report via ReportBuilder."""
        tasks = []
        if self.ring_task:
            tasks.append(
                RingTask(requestor=self._ring_requestor, counts=self.ring_counts)
            )
        if self.barrel_task:
            tasks.append(
                BarrelTask(requestor=self._barrel_requestor, results=self.barrel_report)
            )
        if self.anomaly_task:
            tasks.append(
                AnomalyTask(
                    requestor=self._anomaly_requestor,
                    color=self._inspection_color or "unknown",
                    results=self.tile_results,
                )
            )
        try:
            path = ReportBuilder().save_pdf(tasks)
            self.get_logger().info(f"Report written to {path}")
        except Exception as e:
            self.get_logger().error(f"Report write failed: {e}")

    # ── Arm command ────────────────────────────────────────────────────

    def _pub_arm(self, pose_name: str):
        msg = String()
        msg.data = pose_name
        self.arm_pub.publish(msg)

    def _send_cmd_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_vel_pub.publish(t)

    def _stop_cmd_vel(self):
        self._send_cmd_vel(0.0, 0.0)

    def _min_scan_distance(self, center_angle: float, half_cone: float) -> float | None:
        scan = self._last_scan
        if scan is None:
            return None

        minimum = float("inf")
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            error = _normalize_angle(angle - center_angle)
            if abs(error) <= half_cone:
                minimum = min(minimum, distance)
        return minimum if math.isfinite(minimum) else None

    def _current_yaw(self) -> float | None:
        if self.current_pose is None:
            return None
        q = self.current_pose.orientation
        return _quaternion_to_yaw([q.w, q.x, q.y, q.z])

    def _current_xy(self) -> np.ndarray | None:
        if self.current_pose is None:
            return None
        return np.array(
            [self.current_pose.position.x, self.current_pose.position.y],
            dtype=float,
        )

    def _abort_inspection(self, reason: str):
        self.get_logger().error(f"[inspection] aborted: {reason}")
        self._stop_cmd_vel()
        self._cancel_nav()
        self._start_room2_navigation()

    def _inspection_scan_distance(self) -> float:
        current_xy = self._current_xy()
        if current_xy is None or self._inspection_scan_start is None:
            return 0.0
        return float(np.linalg.norm(current_xy - self._inspection_scan_start))

    def _inspection_target_yaw(self) -> float:
        # Workstation colors have known belt axes, but each axis has two possible
        # headings. Choose the one whose forward vector points toward the detected
        # workstation center from the robot's current position.
        axis_yaw = 0.0 if self._inspection_color == "red" else math.pi / 2
        current_xy = self._current_xy()
        workstation_xy = self.workstation_poses.get(self._inspection_color or "")
        if current_xy is None or workstation_xy is None:
            self.get_logger().warn(
                f"[inspection] missing geometry for scan direction; using yaw={axis_yaw:.3f}"
            )
            return axis_yaw

        toward_center = workstation_xy - current_xy
        axis = np.array([math.cos(axis_yaw), math.sin(axis_yaw)])
        if float(np.dot(axis, toward_center)) < 0.0:
            axis_yaw = _normalize_angle(axis_yaw + math.pi)
        self.get_logger().info(
            f"[inspection] scan direction robot=({current_xy[0]:.2f},{current_xy[1]:.2f}) "
            f"workstation=({workstation_xy[0]:.2f},{workstation_xy[1]:.2f}) "
            f"target_yaw={axis_yaw:.3f}",
            throttle_duration_sec=1.0,
        )
        return axis_yaw

    def _median_scan_distance(
        self, center_angle: float, half_cone: float
    ) -> float | None:
        scan = self._last_scan
        if scan is None:
            return None

        distances = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if abs(_normalize_angle(angle - center_angle)) <= half_cone:
                distances.append(distance)
        return float(np.median(distances)) if distances else None

    def _belt_world_perp(self) -> float:
        return math.pi / 2 if self._inspection_color == "red" else math.pi

    def _belt_side_distance(self) -> float | None:
        current_yaw = self._current_yaw()
        if current_yaw is None:
            return None
        body_angle = _normalize_angle(
            self._belt_world_perp() - current_yaw - math.pi / 2
        )
        dist = self._median_scan_distance(body_angle, math.radians(8.0))
        if dist is None:
            return None
        # Reject structural gaps where the perpendicular ray shoots past the belt face.
        # Red belt face is ~1.8 m away; green is ~0.5 m away. Use per-belt ceilings
        # well above the observed face range but below the nearest gap reading.
        max_valid = 2.5 if self._inspection_color == "red" else 1.0
        if dist > max_valid:
            return None
        return dist

    def _belt_follow_correction(self) -> float:
        current_yaw = self._current_yaw()
        yaw_correction = 0.0
        if current_yaw is not None and self._inspection_scan_yaw is not None:
            yaw_error = _normalize_angle(self._inspection_scan_yaw - current_yaw)
            if abs(yaw_error) > 0.03:
                yaw_correction = 0.35 * yaw_error

        side = self._belt_side_distance()
        distance_correction = 0.0
        if side is not None and self._inspection_side_target is not None:
            distance_error = side - self._inspection_side_target
            if abs(distance_error) > 0.04:
                belt_side_sign = math.copysign(
                    1.0, math.sin(self._belt_world_perp() - (current_yaw or 0.0))
                )
                distance_correction = belt_side_sign * 0.25 * distance_error

        correction = _inspection_steering(yaw_correction, distance_correction)
        self.get_logger().info(
            f"[inspection] belt follow side={side} "
            f"target={self._inspection_side_target} "
            f"yaw_cmd={yaw_correction:+.3f} "
            f"distance_cmd={distance_correction:+.3f} cmd={correction:+.3f}",
            throttle_duration_sec=0.5,
        )
        return correction

    def _detect_tile(self, frame: np.ndarray) -> tuple[np.ndarray | None, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        crop_y = gray.shape[0] // 5
        crop = gray[crop_y:, :]
        _, threshold = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        center_y, center_x = crop.shape[0] // 2, crop.shape[1] // 2
        if threshold[center_y, center_x] == 0:
            threshold = 255 - threshold
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)

        full_mask = np.zeros_like(gray)
        full_mask[crop_y:, :] = threshold
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        frame_area = crop.shape[0] * crop.shape[1]
        best = None
        best_metrics = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < frame_area * 0.05 or area > frame_area * 0.50:
                continue
            box = cv2.boxPoints(cv2.minAreaRect(contour))
            width = float(np.linalg.norm(box[0] - box[1]))
            height = float(np.linalg.norm(box[1] - box[2]))
            aspect = max(width, height) / max(min(width, height), 1.0)
            if aspect > 1.65:
                continue
            contour_mask = np.zeros_like(crop)
            cv2.drawContours(contour_mask, [contour], -1, (255,), -1)
            mean_brightness = float(cv2.mean(crop, mask=contour_mask)[0])
            if mean_brightness < 70.0:
                continue
            center_x, center_y = box.mean(axis=0)
            if not (0.12 * crop.shape[1] <= center_x <= 0.88 * crop.shape[1]):
                continue
            if best is None or area > cv2.contourArea(best):
                best = contour
                best_metrics = (
                    area / frame_area,
                    aspect,
                    center_x,
                    center_y,
                    mean_brightness,
                )

        if best is None:
            return None, full_mask
        if (
            self.state == State.INSPECT_WORKSTATION
            and self._inspection_phase == 5
            and best_metrics is not None
        ):
            area_ratio, aspect, center_x, center_y, mean_brightness = best_metrics
            self.get_logger().info(
                f"[inspection] tile candidate area={area_ratio:.3f} "
                f"aspect={aspect:.2f} center=({center_x:.0f},{center_y + crop_y:.0f}) "
                f"brightness={mean_brightness:.1f}",
                throttle_duration_sec=0.5,
            )
        box = quad_from_contour(best)
        if box is None:
            box = cv2.boxPoints(cv2.minAreaRect(best))
        box[:, 1] += crop_y  # ty: ignore[unsupported-operator]  # ndarray += int is valid at runtime; stub lacks this overload
        return box.astype(np.float32), full_mask

    def _score_tile(self, frame: np.ndarray) -> TileAnomalyResult:
        """Score a tile frame using TileAnomalyDetector."""
        try:
            self._ws_debug_raw_pub.publish(self._cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception:
            pass
        box, thresh = self._detect_tile(frame)
        try:
            self._ws_debug_thresh_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(thresh, "mono8")
            )
        except Exception:
            pass
        if box is None:
            return TileAnomalyResult(
                status="UNKNOWN", reason="tile contour not detected"
            )
        canonical = warp_tile(frame, box)
        try:
            self._ws_debug_warped_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(
                    cv2.cvtColor(canonical, cv2.COLOR_BGR2GRAY), "mono8"
                )
            )
        except Exception:
            pass
        detector = (
            self._tile_detector_red
            if self._inspection_color == "red"
            else self._tile_detector_green
        )
        return detector.detect(canonical)

    def _capture_current_tile(self, frame: np.ndarray) -> None:
        box, _ = self._detect_tile(frame)
        if box is None:
            self.get_logger().warn(
                "Tile capture skipped: contour was lost during pause"
            )
            return

        pose = None
        current_xy = self._current_xy()
        current_yaw = self._current_yaw()
        if current_xy is not None and current_yaw is not None:
            pose = {
                "x": float(current_xy[0]),
                "y": float(current_xy[1]),
                "yaw": float(current_yaw),
            }
        try:
            metadata_path = save_tile_capture(
                self._tile_capture_dir,
                frame,
                box,
                world=self._world_name,
                station=self._inspection_color or "unknown",
                tile_index=self._tile_index,
                stamp_ns=self._last_top_stamp_ns,
                pose=pose,
            )
            self.get_logger().info(f"Captured tile sample: {metadata_path}")
        except (OSError, ValueError, cv2.error) as exc:
            self.get_logger().error(f"Tile capture failed: {exc}")

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
        self._nav_goal_sent_time = self.get_clock().now().nanoseconds / 1e9
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
        self.get_logger().warning("Canceling Nav2 goal ")
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
        self._nav_goal_sent_time = None

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

    def _nav_status_name(self) -> str:
        if self._nav_rejected:
            return "REJECTED"
        if self.nav_result_future is None or not self.nav_result_future.done():
            return "INCOMPLETE"
        try:
            result = self.nav_result_future.result()
        except Exception as exc:
            return f"ERROR({exc})"
        if result is None:
            return "NO_RESULT"
        names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return names.get(result.status, str(result.status))

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

    # Robot footprint radius (~0.189 m, see nav2.yaml) plus a small safety
    # margin. The check window must cover at least this much, or a goal can
    # pass while the robot's actual body still overlaps a near-obstacle cell.
    APPROACH_CLEARANCE_RADIUS_M = 0.28

    def _cost_ok_on(
        self,
        costmap: OccupancyGrid,
        x: float,
        y: float,
        out_of_bounds_ok: bool,
        threshold: float = 150,
    ) -> tuple[bool, float | None]:
        mx, my = self._world_to_map(costmap, x, y)
        w, h = costmap.info.width, costmap.info.height
        half = max(1, round(self.APPROACH_CLEARANCE_RADIUS_M / costmap.info.resolution))
        costs = []

        for dy in range(-half, half + 1):
            for dx in range(-half, half + 1):
                nx_, ny_ = mx + dx, my + dy
                if 0 <= nx_ < w and 0 <= ny_ < h:
                    c = costmap.data[ny_ * w + nx_]
                    if c >= 0:
                        costs.append(c)
        if not costs:
            # No cells in this costmap cover the point at all — e.g. the
            # local costmap is a small window around the robot and the
            # candidate is still far away. That's "no information," not
            # "blocked"; only the global costmap (which covers the whole
            # known map) should treat unseen ground as blocked.
            return out_of_bounds_ok, None
        median_cost = sorted(costs)[len(costs) // 2]
        return median_cost < threshold, median_cost

    def _cost_at_goal_ok(self, x: float, y: float) -> bool:
        """Check the goal clears both costmaps.

        The global costmap is what we have readily available for planning,
        but Nav2's controller_server and behavior_server (the things that
        actually drive and recover) act on the local costmap, which has its
        own more reactive voxel layer. A goal can look clear on one and not
        the other, so both must agree before we commit to it.

        The keepout mask is checked separately from the live costmap: Nav2's
        recovery behaviors call clearEntirely on the costmap, which briefly
        wipes the KeepoutFilter's painted cells until it re-subscribes and
        repaints — sampling the costmap in that window can read a keepout
        cell as clear. The mask topic itself is latched and never cleared,
        so checking it directly is immune to that race.
        """
        if (
            self.keepout_mask is not None
            and not self._cost_ok_on(
                self.keepout_mask, x, y, out_of_bounds_ok=True, threshold=50
            )[0]
        ):
            self.get_logger().warn("Approach blocked (keepout mask).")
            return False
        if self.costmap is not None:
            ok, median_cost = self._cost_ok_on(
                self.costmap, x, y, out_of_bounds_ok=False
            )
            if not ok:
                self.get_logger().warn(
                    f"Approach blocked (global costmap), median_cost={median_cost}."
                )
                return False
        if self.local_costmap is not None:
            ok, median_cost = self._cost_ok_on(
                self.local_costmap, x, y, out_of_bounds_ok=True
            )
            if not ok:
                self.get_logger().warn(
                    f"Approach blocked (local costmap), median_cost={median_cost}."
                )
                return False
        return True

    def _world_to_map(self, costmap: OccupancyGrid, x: float, y: float):
        res = costmap.info.resolution
        ox, oy = (
            costmap.info.origin.position.x,
            costmap.info.origin.position.y,
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
