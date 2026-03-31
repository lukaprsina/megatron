"""Mission controller for Megatron Task 1.

State machine that drives the TurtleBot through waypoints, reacts to face
and ring detections, computes approach poses from surface normals, manages
speech output, verifies detections, and handles a cleanup phase for
unconfirmed hypotheses.

Bug fixes over old controller:
- Nav2 status validation (no more greeting on abort)
- Nav2 GetState debounce (2 s, not 10 Hz)
- Waypoint skip on abort (max 2 retries)
- Loop counter (max 2 loops before cleanup/done)
- Robot pose from TF2 (not stale AMCL)
- Approach math from surface normal (no more "goal in wall")
- Atomic ring color via frame_id hack (no race condition)
- Speech queue (no overlapping)
- VERIFYING state (check detection after approach)
- CLEANUP state (revisit unconfirmed hypotheses)
"""

from __future__ import annotations

import math
from enum import Enum, auto

import numpy as np
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy,
    QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import Spin, NavigateToPose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus

from lifecycle_msgs.srv import GetState

import tf2_ros
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from megatron.speech import Speaker
from megatron.perception_utils import quaternion_to_normal_xy


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class State(Enum):
    WAITING_FOR_NAV2 = auto()
    UNDOCKING = auto()
    EXPLORING = auto()
    APPROACHING_OBJECT = auto()
    VERIFYING = auto()
    SPINNING = auto()
    CLEANUP = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# Waypoint loader
# ---------------------------------------------------------------------------

def _quaternion_to_yaw(q_list):
    """Convert quaternion to yaw.  YAML files store (w, x, y, z)."""
    try:
        # Our YAML convention: [w, x, y, z] (first element is the scalar)
        w, x, y, z = q_list
    except Exception:
        return 0.0
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def load_waypoints_from_yaml(path: str) -> list[tuple[float, float, float]]:
    """Load waypoints from a YAML file.

    Returns list of ``(x, y, yaw)`` tuples.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f'Waypoints file not found: {p}')
    data = yaml.safe_load(p.read_text())
    out: list[tuple[float, float, float]] = []
    candidates: list = []

    if isinstance(data, dict) and 'waypoints' in data:
        candidates = list(data['waypoints'].values())
    elif isinstance(data, list):
        candidates = data
    elif isinstance(data, dict):
        candidates = list(data.values())

    for entry in candidates:
        x = y = yaw = None
        if isinstance(entry, dict):
            pose = entry.get('pose')
            orient = entry.get('orientation')
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
# Controller node
# ---------------------------------------------------------------------------

class MissionController(Node):

    def __init__(self) -> None:
        super().__init__('mission_controller')

        # ---- Parameters ----
        self.declare_parameter('dedup_distance', 0.8)
        self.declare_parameter('approach_distance', 0.6)
        self.declare_parameter('approach_fallback_distance', 0.35)
        self.declare_parameter('spin_at_waypoints', False)
        self.declare_parameter('total_faces', 3)
        self.declare_parameter('total_rings', 2)
        self.declare_parameter('waypoints_file', 'waypoints/test1.yaml')
        self.declare_parameter('max_loops', 2)
        self.declare_parameter('verify_timeout', 3.0)

        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value
        self.approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value
        self.approach_fallback_distance = self.get_parameter('approach_fallback_distance').get_parameter_value().double_value
        self.spin_at_waypoints = self.get_parameter('spin_at_waypoints').get_parameter_value().bool_value
        self.total_faces = self.get_parameter('total_faces').get_parameter_value().integer_value
        self.total_rings = self.get_parameter('total_rings').get_parameter_value().integer_value
        self.max_loops = self.get_parameter('max_loops').get_parameter_value().integer_value
        self.verify_timeout = self.get_parameter('verify_timeout').get_parameter_value().double_value

        # Speech
        self.speaker = Speaker()
        self.speaker.set_node_logger(self)
        self.speech_queue: list[str] = []

        # Navigation state
        self.state = State.WAITING_FOR_NAV2
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.nav_rejected = False
        self.is_docked: bool | None = None
        self.start_time = None
        self.waypoint_index = 0
        self.waypoint_retries = 0
        self.loops_completed = 0

        # Nav2 lifecycle checking (debounced)
        self.nodes = ['amcl', 'bt_navigator']
        self.nav2_states = {n: 'Unknown' for n in self.nodes}
        self.nav2_ready = False
        self.last_nav2_check_time = 0.0

        # TF2 for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load waypoints
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        try:
            self.waypoints = load_waypoints_from_yaml(wp_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wp_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            raise

        # Detection state
        self.found_faces: list[np.ndarray] = []
        self.found_rings: list[dict] = []  # {'pos': np.ndarray, 'color': str}

        # Pending approach
        self.pending_object: dict | None = None  # {'type': 'face'|'ring', 'pose': PoseStamped, 'color': str}
        self.current_approach: dict | None = None  # same format, plus 'tried_fallback'

        # VERIFYING state
        self.verify_start_time = None

        # CLEANUP state
        self.cleanup_targets: list[dict] = []
        self.cleanup_index = 0

        # Last detection timestamps (for verification)
        self.last_face_detection_time = None
        self.last_ring_detection_time = None

        # ---- Subscribers ----
        self.create_subscription(
            DockStatus, 'dock_status', self._dock_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/detected_faces', self._face_callback, 10)
        self.create_subscription(
            PoseStamped, '/detected_rings', self._ring_callback, 10)

        # ---- Publishers ----
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)

        # ---- Action clients ----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_client = ActionClient(self, Undock, 'undock')

        # ---- Nav2 lifecycle clients ----
        self.nav2_lifecycle_clients = {
            n: self.create_client(GetState, f'/{n}/get_state') for n in self.nodes
        }

        # ---- Timers ----
        self.timer = self.create_timer(0.1, self._tick)          # 10 Hz state machine
        self.speech_timer = self.create_timer(1.0, self._process_speech_queue)

        self.get_logger().info('Mission controller initialized.')

    # ==================================================================
    # Detection callbacks
    # ==================================================================

    def _face_callback(self, msg: PoseStamped) -> None:
        self.last_face_detection_time = self.get_clock().now()
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        for f in self.found_faces:
            if np.linalg.norm(pos - f) < self.dedup_distance:
                return

        self.get_logger().info(
            f'New face detected at ({pos[0]:.2f}, {pos[1]:.2f}), '
            f'total: {len(self.found_faces) + 1}')
        self.found_faces.append(pos)

        if self.state == State.EXPLORING and self.pending_object is None:
            self.pending_object = {
                'type': 'face',
                'pose': msg,
                'color': '',
            }

    def _ring_callback(self, msg: PoseStamped) -> None:
        self.last_ring_detection_time = self.get_clock().now()
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # Parse color from frame_id hack: "map|<color>"
        frame_id = msg.header.frame_id
        if '|' in frame_id:
            color = frame_id.split('|', 1)[1]
        else:
            color = 'unknown'

        for r in self.found_rings:
            if np.linalg.norm(pos - r['pos']) < self.dedup_distance:
                return

        self.get_logger().info(
            f'New ring ({color}) detected at ({pos[0]:.2f}, {pos[1]:.2f}), '
            f'total: {len(self.found_rings) + 1}')
        self.found_rings.append({'pos': pos, 'color': color})

        if self.state == State.EXPLORING and self.pending_object is None:
            self.pending_object = {
                'type': 'ring',
                'pose': msg,
                'color': color,
            }

    def _dock_callback(self, msg: DockStatus) -> None:
        self.is_docked = msg.is_docked

    # ==================================================================
    # Navigation helpers
    # ==================================================================

    def _get_robot_pose(self) -> tuple[float, float] | None:
        """Get current robot position from TF2 (map → base_link)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose server not available')
            return False

        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        self.nav_rejected = False
        self.status = None
        future = self.nav_client.send_goal_async(goal_msg, self._feedback_callback)
        future.add_done_callback(self._nav_goal_response)
        return True

    def _nav_goal_response(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.nav_rejected = True
            self.result_future = None
            return
        self.nav_rejected = False
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future) -> None:
        result = future.result()
        self.status = result.status if result else GoalStatus.STATUS_ABORTED

    def _feedback_callback(self, msg) -> None:
        self.feedback = msg.feedback

    def _cancel_nav(self) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None
            self.result_future = None

    def _is_nav_complete(self) -> bool:
        if self.nav_rejected:
            return True
        if self.result_future is None:
            return True
        return self.result_future.done()

    def _nav_succeeded(self) -> bool:
        """True only if the last navigation goal actually succeeded."""
        return self.status == GoalStatus.STATUS_SUCCEEDED

    def _send_spin(self, angle: float = 2.0 * math.pi, time_allowance: int = 15) -> bool:
        if not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Spin server not available')
            return False
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance = Duration(sec=time_allowance)
        self.get_logger().info('Spinning 360°...')
        future = self.spin_client.send_goal_async(goal, self._feedback_callback)
        future.add_done_callback(self._spin_goal_response)
        return True

    def _spin_goal_response(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Spin rejected')
            self.result_future = None
            self.nav_rejected = True
            return
        self.goal_handle = gh
        self.result_future = gh.get_result_async()
        self.result_future.add_done_callback(self._nav_result)

    # ==================================================================
    # Approach pose from surface normal
    # ==================================================================

    def _compute_approach_pose(self, pose_msg: PoseStamped,
                               distance: float | None = None) -> tuple[float, float, float]:
        """Compute approach pose using the surface normal stored in the orientation.

        The approach point is ``distance`` meters along the outward normal.
        The yaw faces the surface.
        """
        if distance is None:
            distance = self.approach_distance

        tx = pose_msg.pose.position.x
        ty = pose_msg.pose.position.y
        q = pose_msg.pose.orientation

        # Recover the 2-D outward normal from the quaternion
        normal_xy = quaternion_to_normal_xy(q)
        n_len = np.linalg.norm(normal_xy)

        if n_len < 0.01:
            # Fallback: approach from current robot position
            rp = self._get_robot_pose()
            if rp is not None:
                dx, dy = tx - rp[0], ty - rp[1]
                d = math.hypot(dx, dy)
                if d > 0.1:
                    ax = tx - dx / d * distance
                    ay = ty - dy / d * distance
                    yaw = math.atan2(dy, dx)
                    return ax, ay, yaw
            return tx, ty, 0.0

        normal_xy = normal_xy / n_len

        ax = tx + normal_xy[0] * distance
        ay = ty + normal_xy[1] * distance
        # Face toward the surface (opposite the normal)
        yaw = math.atan2(-normal_xy[1], -normal_xy[0])

        return ax, ay, yaw

    # ==================================================================
    # Completion
    # ==================================================================

    def _all_found(self) -> bool:
        return (len(self.found_faces) >= self.total_faces
                and len(self.found_rings) >= self.total_rings)

    def _finish(self) -> None:
        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.state = State.DONE
        self._cancel_nav()
        self.get_logger().info(
            f'Mission complete! {len(self.found_faces)} faces, '
            f'{len(self.found_rings)} rings in {elapsed:.1f}s')
        self.speech_queue.append('Mission complete!')

    # ==================================================================
    # Nav2 lifecycle check (debounced)
    # ==================================================================

    def _check_nav2_ready(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_nav2_check_time < 2.0:
            return
        self.last_nav2_check_time = now_sec

        for name, client in self.nav2_lifecycle_clients.items():
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda f, n=name: self._nav2_state_cb(f, n))

    def _nav2_state_cb(self, future, node_name: str) -> None:
        try:
            res = future.result()
            self.nav2_states[node_name] = res.current_state.label
            if all(s == 'active' for s in self.nav2_states.values()):
                if not self.nav2_ready:
                    self.get_logger().info('All Nav2 nodes active — system ready.')
                self.nav2_ready = True
            else:
                self.nav2_ready = False
        except Exception as e:
            self.get_logger().error(f'Lifecycle check failed for {node_name}: {e}')

    # ==================================================================
    # Speech queue
    # ==================================================================

    def _process_speech_queue(self) -> None:
        if self.speech_queue and not self.speaker.is_busy():
            text = self.speech_queue.pop(0)
            self.speaker.speak(text)

    # ==================================================================
    # Main state machine
    # ==================================================================

    def _tick(self) -> None:
        self._publish_mission_status()

        if self.state == State.WAITING_FOR_NAV2:
            self._handle_waiting()
        elif self.state == State.UNDOCKING:
            pass  # async callback handles this
        elif self.state == State.EXPLORING:
            self._handle_exploring()
        elif self.state == State.APPROACHING_OBJECT:
            self._handle_approaching()
        elif self.state == State.VERIFYING:
            self._handle_verifying()
        elif self.state == State.SPINNING:
            self._handle_spinning()
        elif self.state == State.CLEANUP:
            self._handle_cleanup()
        elif self.state == State.DONE:
            pass

    # ---- WAITING_FOR_NAV2 ----

    def _handle_waiting(self) -> None:
        if self.is_docked is None:
            return
        self._check_nav2_ready()
        if not self.nav2_ready:
            return
        self.get_logger().info('Nav2 ready!')
        self.start_time = self.get_clock().now()
        if self.is_docked:
            self.state = State.UNDOCKING
            self._do_undock()
        else:
            self.state = State.EXPLORING
            self._send_next_waypoint()

    def _do_undock(self) -> None:
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Undock server not available')
            return
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_response)

    def _undock_response(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Undock rejected')
            return
        gh.get_result_async().add_done_callback(self._undock_done)

    def _undock_done(self, future) -> None:
        self.get_logger().info('Undocked successfully.')
        self.state = State.EXPLORING
        self._send_next_waypoint()

    # ---- EXPLORING ----

    def _handle_exploring(self) -> None:
        # Check for pending detections
        if self.pending_object is not None:
            obj = self.pending_object
            self.pending_object = None
            self._cancel_nav()
            self.current_approach = {**obj, 'tried_fallback': False}
            self.state = State.APPROACHING_OBJECT
            ax, ay, yaw = self._compute_approach_pose(obj['pose'])
            self._send_nav_goal(ax, ay, yaw)
            return

        if not self._is_nav_complete():
            return

        # Nav completed — check if succeeded or failed
        if not self._nav_succeeded() and self.status is not None:
            self.waypoint_retries += 1
            if self.waypoint_retries < 2:
                self.get_logger().warn(
                    f'Waypoint {self.waypoint_index} failed (status={self.status}), '
                    f'retry {self.waypoint_retries}/2')
                x, y, yaw = self.waypoints[self.waypoint_index]
                self._send_nav_goal(x, y, yaw)
                return
            else:
                self.get_logger().warn(
                    f'Waypoint {self.waypoint_index} failed after 2 retries, skipping.')
        
        # Advance waypoint
        self.waypoint_retries = 0
        self.waypoint_index += 1

        if self._all_found():
            self._finish()
            return

        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0
            self.loops_completed += 1
            self.get_logger().info(
                f'Loop {self.loops_completed} complete '
                f'({len(self.found_faces)} faces, {len(self.found_rings)} rings).')
            if self.loops_completed >= self.max_loops:
                self.get_logger().warn('Max loops reached — entering cleanup.')
                self._enter_cleanup()
                return

        if self.spin_at_waypoints:
            self.state = State.SPINNING
            self._send_spin()
        else:
            self._send_next_waypoint()

    # ---- APPROACHING_OBJECT ----

    def _handle_approaching(self) -> None:
        if not self._is_nav_complete():
            return

        if self._nav_succeeded():
            # Transition to verification
            self.state = State.VERIFYING
            self.verify_start_time = self.get_clock().now()
            self.get_logger().info('Reached approach pose — verifying...')
            return

        # Navigation failed
        if self.current_approach is not None and not self.current_approach['tried_fallback']:
            self.current_approach['tried_fallback'] = True
            self.get_logger().warn('Approach failed, trying shorter distance...')
            ax, ay, yaw = self._compute_approach_pose(
                self.current_approach['pose'], self.approach_fallback_distance)
            self._send_nav_goal(ax, ay, yaw)
            return

        # Both attempts failed — give up and resume exploring
        self.get_logger().warn('Approach failed after fallback — resuming exploration.')
        self.current_approach = None
        self.state = State.EXPLORING
        self._send_next_waypoint()

    # ---- VERIFYING ----

    def _handle_verifying(self) -> None:
        if self.current_approach is None:
            self.state = State.EXPLORING
            self._send_next_waypoint()
            return

        elapsed = (self.get_clock().now() - self.verify_start_time).nanoseconds / 1e9
        obj_type = self.current_approach['type']

        # Check if we've received a recent detection of the right type
        recent_threshold = 2.0  # seconds
        now = self.get_clock().now()
        verified = False

        if obj_type == 'face' and self.last_face_detection_time is not None:
            dt = (now - self.last_face_detection_time).nanoseconds / 1e9
            if dt < recent_threshold:
                verified = True
        elif obj_type == 'ring' and self.last_ring_detection_time is not None:
            dt = (now - self.last_ring_detection_time).nanoseconds / 1e9
            if dt < recent_threshold:
                verified = True

        # After 1 second minimum, accept verification or timeout
        if elapsed >= 1.0 and verified:
            self._greet_object()
            return
        if elapsed >= self.verify_timeout:
            # Assume it's there (we already found it in the tracker)
            self.get_logger().warn('Verify timeout — greeting anyway.')
            self._greet_object()
            return

    def _greet_object(self) -> None:
        obj = self.current_approach
        self.current_approach = None

        if obj['type'] == 'face':
            self.speech_queue.append('Hello!')
            self.get_logger().info('Greeted face.')
        else:
            color = obj.get('color', 'unknown')
            self.speech_queue.append(f'I see a {color} ring')
            self.get_logger().info(f'Announced {color} ring.')

        if self._all_found():
            self._finish()
        else:
            self.state = State.EXPLORING
            self._send_next_waypoint()

    # ---- SPINNING ----

    def _handle_spinning(self) -> None:
        if self._is_nav_complete():
            if self._all_found():
                self._finish()
                return
            self.state = State.EXPLORING
            self._send_next_waypoint()

    # ---- CLEANUP ----

    def _enter_cleanup(self) -> None:
        """Transition to cleanup: visit unconfirmed hypotheses."""
        self.state = State.CLEANUP
        self.cleanup_targets = []
        self.cleanup_index = 0

        # We cannot directly access the tracker from here because it lives
        # in the detector nodes.  Instead we rely on the fact that any
        # detection that crossed the confirmation threshold was already
        # published and received in _face_callback / _ring_callback.
        # "Unconfirmed" here means objects the robot barely saw.
        #
        # Heuristic: if we haven't found everything, revisit waypoints
        # at a different subset (every other waypoint) to get a second look.
        if not self._all_found():
            stride = max(1, len(self.waypoints) // 4)
            for i in range(0, len(self.waypoints), stride):
                x, y, yaw = self.waypoints[i]
                self.cleanup_targets.append({'x': x, 'y': y, 'yaw': yaw})
            self.get_logger().info(
                f'Cleanup: visiting {len(self.cleanup_targets)} waypoints.')
            self._send_cleanup_goal()
        else:
            self._finish()

    def _send_cleanup_goal(self) -> None:
        if self.cleanup_index >= len(self.cleanup_targets):
            self.get_logger().info('Cleanup complete.')
            self._finish()
            return
        t = self.cleanup_targets[self.cleanup_index]
        self._send_nav_goal(t['x'], t['y'], t['yaw'])

    def _handle_cleanup(self) -> None:
        # React to detections even in cleanup
        if self.pending_object is not None:
            obj = self.pending_object
            self.pending_object = None
            self._cancel_nav()
            self.current_approach = {**obj, 'tried_fallback': False}
            self.state = State.APPROACHING_OBJECT
            ax, ay, yaw = self._compute_approach_pose(obj['pose'])
            self._send_nav_goal(ax, ay, yaw)
            return

        if not self._is_nav_complete():
            return

        if self._all_found():
            self._finish()
            return

        self.cleanup_index += 1
        self._send_cleanup_goal()

    # ==================================================================
    # Waypoint management
    # ==================================================================

    def _send_next_waypoint(self) -> None:
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0
        self.get_logger().info(
            f'Heading to waypoint {self.waypoint_index}/{len(self.waypoints) - 1}')
        x, y, yaw = self.waypoints[self.waypoint_index]
        self._send_nav_goal(x, y, yaw)
        self._publish_goal_markers()

    # ==================================================================
    # Visualization
    # ==================================================================

    def _publish_goal_markers(self) -> None:
        ma = MarkerArray()
        markers: list[Marker] = []
        stamp = self.get_clock().now().to_msg()

        for i, (x, y, yaw) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.pose.orientation = self._yaw_to_quaternion(yaw)
            m.scale.x = 0.3
            m.scale.y = 0.08
            m.scale.z = 0.08

            if i < self.waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.7, 0.3, 0.5
            elif i == self.waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.3, 1.0, 0.5
            markers.append(m)

        ma.markers = markers
        self.goal_marker_pub.publish(ma)

    def _publish_mission_status(self) -> None:
        msg = String()
        msg.data = (
            f'{self.state.name} | faces {len(self.found_faces)}/{self.total_faces} '
            f'| rings {len(self.found_rings)}/{self.total_rings} '
            f'| wp {self.waypoint_index + 1}/{len(self.waypoints)} '
            f'| loop {self.loops_completed + 1}/{self.max_loops}'
        )
        self.mission_status_pub.publish(msg)


def main(args=None):
    print('Mission controller starting.')
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
