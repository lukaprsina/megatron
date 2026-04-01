"""Mission controller for Task 1: waypoint exploration, face/ring approach.

State machine:
  WAITING_FOR_NAV2 → UNDOCKING → EXPLORING → APPROACHING_OBJECT → VERIFYING → EXPLORING
                                                                             → DONE

Detectors publish PoseStamped with surface normal encoded as orientation.
Ring color is packed in frame_id as "map|{color}".
Approach pose is computed along the surface normal (in free space, facing the object).
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy,
    QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import Spin, NavigateToPose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus

from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from megatron.speech import Speaker

import numpy as np
import yaml
from pathlib import Path


class State(Enum):
    WAITING_FOR_NAV2 = auto()
    UNDOCKING = auto()
    EXPLORING = auto()
    APPROACHING_OBJECT = auto()
    VERIFYING = auto()
    DONE = auto()


amcl_pose_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# ---------------------------------------------------------------------------
# Waypoint loading
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
        raise FileNotFoundError(f'Waypoints file not found: {p}')
    data = yaml.safe_load(p.read_text())
    out = []
    candidates = []
    if isinstance(data, dict) and 'waypoints' in data:
        wp_dict = data['waypoints']
        candidates = list(wp_dict.values())
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
                x = float(pose[0])
                y = float(pose[1])
            if orient and len(orient) == 4:
                yaw = _quaternion_to_yaw(orient)
        elif isinstance(entry, (list, tuple)):
            if len(entry) >= 2:
                x = float(entry[0])
                y = float(entry[1])
            if len(entry) >= 3:
                yaw = float(entry[2])

        if x is None or y is None:
            continue
        if yaw is None:
            yaw = 0.0
        out.append((x, y, yaw))

    return out


# ---------------------------------------------------------------------------
# Normal extraction from quaternion
# ---------------------------------------------------------------------------

def _quaternion_to_normal_2d(q: Quaternion):
    """Extract 2D normal from a quaternion that encodes a facing direction.

    The detectors encode the surface normal via normal_to_quaternion() which
    produces a Z-only rotation facing *opposite* the normal. So the normal
    direction is opposite to the quaternion's forward (x-axis) direction.
    """
    # yaw from quaternion (z-only rotation)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    # The quaternion faces *toward* the surface (atan2(-ny, -nx)),
    # so the actual normal is 180° opposite:
    nx = -math.cos(yaw)
    ny = -math.sin(yaw)
    return nx, ny


# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------

class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # Parameters
        self.declare_parameter('dedup_distance', 0.8)
        self.declare_parameter('approach_distance', 0.55)
        self.declare_parameter('approach_retry_offset', 0.1)
        self.declare_parameter('spin_at_waypoints', False)
        self.declare_parameter('total_faces', 3)
        self.declare_parameter('total_rings', 4)
        self.declare_parameter('max_loops', 2)
        self.declare_parameter('waypoints_file', 'waypoints/test1.yaml')
        self.declare_parameter('verify_pause_sec', 1.0)
        self.declare_parameter('max_approach_attempts', 3)
        self.declare_parameter('approach_side_offset', 0.30)

        self.dedup_distance = self.get_parameter('dedup_distance').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.approach_retry_offset = self.get_parameter('approach_retry_offset').value
        self.spin_at_waypoints = self.get_parameter('spin_at_waypoints').value
        self.total_faces = self.get_parameter('total_faces').value
        self.total_rings = self.get_parameter('total_rings').value
        self.max_loops = self.get_parameter('max_loops').value
        self.verify_pause_sec = self.get_parameter('verify_pause_sec').value
        self.max_approach_attempts = self.get_parameter('max_approach_attempts').value
        self.approach_side_offset = self.get_parameter('approach_side_offset').value

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
        self.initial_pose_received = False
        self.is_docked = None
        self.current_pose = None
        self.waypoint_index = 0
        self.loop_count = 0
        self.start_time = None

        # Nav2 lifecycle check
        self.nodes = ['amcl', 'bt_navigator']
        self.states = {n: 'Unknown' for n in self.nodes}
        self.nav2_ready = False
        self.last_nav2_check = 0.0

        # Load waypoints
        try:
            wp_file = self.get_parameter('waypoints_file').value
        except Exception:
            wp_file = 'waypoints/test1.yaml'

        try:
            self.waypoints = load_waypoints_from_yaml(wp_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wp_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from {wp_file}: {e}')
            raise

        # Detection state
        self.found_faces: list[dict] = []   # [{'pos': np.array, 'normal': (nx,ny)}]
        self.found_rings: list[dict] = []   # [{'pos': np.array, 'color': str, 'normal': (nx,ny)}]
        self.pending_approaches: list[dict] = []  # queue of {'type': 'face'|'ring', 'pos', 'normal', 'color'}

        # Approach tracking
        self.current_approach = None  # the dict from pending_approaches being executed
        self.current_approach_candidates: list[tuple[float, float, float]] = []
        self.current_approach_candidate_index = 0
        self.verify_start_time = None

        # In-flight guard: True from send_goal_async() until _nav_goal_response fires.
        # Prevents stale result_future/status from being read on the next tick.
        self.nav_in_flight = False

        # Subscribers
        self.create_subscription(DockStatus, 'dock_status', self._dock_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_callback, amcl_pose_qos)
        self.create_subscription(PoseStamped, '/detected_faces', self._face_callback, 10)
        self.create_subscription(PoseStamped, '/detected_rings', self._ring_callback, 10)

        # Publishers
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)
        self.approaching_object = self.create_publisher(Marker, '/approaching_object', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_client = ActionClient(self, Undock, 'undock')

        # Nav2 lifecycle clients
        self.nav2_lifecycle_clients = {
            n: self.create_client(GetState, f'/{n}/get_state') for n in self.nodes
        }

        # Timers
        self.timer = self.create_timer(0.1, self._tick)              # 10 Hz state machine
        self.speech_timer = self.create_timer(0.5, self._speech_tick) # 2 Hz speech queue

        self.get_logger().info('Mission controller initialized.')

    # ── Detection callbacks ───────────────────────────────────────────

    def _face_callback(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        for f in self.found_faces:
            if np.linalg.norm(pos - f['pos']) < self.dedup_distance:
                # Already known — if approach previously failed, re-queue it
                if not f.get('greeted', False) and self.state != State.DONE:
                    self._requeue_if_not_pending('face', pos,
                                                 _quaternion_to_normal_2d(msg.pose.orientation),
                                                 None)
                return

        nx, ny = _quaternion_to_normal_2d(msg.pose.orientation)

        self.get_logger().info(
            f'New face detected at ({pos[0]:.2f}, {pos[1]:.2f}), total: {len(self.found_faces) + 1}')

        self.found_faces.append({'pos': pos, 'normal': (nx, ny), 'greeted': False})

        if self.state != State.DONE:
            self.pending_approaches.append({
                'type': 'face',
                'pos': pos,
                'normal': (nx, ny),
                'color': None,
            })

    def _ring_callback(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # Parse color from frame_id
        frame_id = msg.header.frame_id
        color = 'unknown'
        if '|' in frame_id:
            color = frame_id.split('|', 1)[1]

        for r in self.found_rings:
            if np.linalg.norm(pos - r['pos']) < self.dedup_distance:
                # Already known — if approach previously failed, re-queue it
                if not r.get('greeted', False) and self.state != State.DONE:
                    self._requeue_if_not_pending('ring', pos,
                                                 _quaternion_to_normal_2d(msg.pose.orientation),
                                                 color)
                return

        nx, ny = _quaternion_to_normal_2d(msg.pose.orientation)

        self.get_logger().info(
            f'New ring ({color}) detected at ({pos[0]:.2f}, {pos[1]:.2f}), total: {len(self.found_rings) + 1}')

        self.found_rings.append({'pos': pos, 'color': color, 'normal': (nx, ny), 'greeted': False})

        if self.state != State.DONE:
            self.pending_approaches.append({
                'type': 'ring',
                'pos': pos,
                'normal': (nx, ny),
                'color': color,
            })

    # ── Nav2 / dock callbacks ─────────────────────────────────────────

    def _requeue_if_not_pending(self, obj_type, pos, normal, color):
        """Re-add a known-but-ungreeted object to pending if not already queued."""
        nx, ny = normal
        # Check it's not already in the pending queue or currently being approached
        for a in self.pending_approaches:
            if a['type'] == obj_type and np.linalg.norm(pos - np.array(a['pos'])) < self.dedup_distance:
                return
        if (self.current_approach is not None
                and self.current_approach['type'] == obj_type
                and np.linalg.norm(pos - np.array(self.current_approach['pos'])) < self.dedup_distance):
            return
        self.get_logger().info(
            f'Re-queuing ungreeted {obj_type} at ({pos[0]:.2f}, {pos[1]:.2f})')
        self.pending_approaches.append({
            'type': obj_type,
            'pos': pos,
            'normal': (nx, ny),
            'color': color,
        })

    def _dock_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.initial_pose_received = True
        self.current_pose = msg.pose

    def _feedback_callback(self, msg):
        self.feedback = msg.feedback

    # ── Speech queue ──────────────────────────────────────────────────

    def _speech_tick(self):
        if self.speech_queue and not self.speaker.is_busy():
            text = self.speech_queue.pop(0)
            self.speaker.speak(text)

    def _say(self, text: str):
        self.speech_queue.append(text)

    # ── Navigation helpers ────────────────────────────────────────────

    def _check_nav2_states(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_nav2_check < 2.0:
            return
        self.last_nav2_check = now

        for name, client in self.nav2_lifecycle_clients.items():
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda f, n=name: self._nav2_state_cb(f, n))

    def _nav2_state_cb(self, future, node_name):
        try:
            res = future.result()
            self.states[node_name] = res.current_state.label
            if all(s == 'active' for s in self.states.values()):
                if not self.nav2_ready:
                    self.get_logger().info('All Nav2 nodes active — system ready.')
                self.nav2_ready = True
            else:
                self.nav2_ready = False
        except Exception as e:
            self.get_logger().error(f'Nav2 state check failed for {node_name}: {e}')

    def _yaw_to_quaternion(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _send_nav_goal(self, x, y, yaw):
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
        self.nav_in_flight = True
        future = self.nav_client.send_goal_async(goal_msg, self._feedback_callback)
        future.add_done_callback(self._nav_goal_response)
        return True

    def _nav_goal_response(self, future):
        self.nav_in_flight = False
        self.status = 0
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.nav_rejected = True
            self.result_future = None
            return

        self.nav_rejected = False
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        self.status = result.status if result else GoalStatus.STATUS_ABORTED

    def _cancel_nav(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.result_future = None

    def _is_nav_complete(self):
        if self.nav_in_flight:
            return False
        if self.result_future is None:
            return True
        return self.result_future.done() and not self.nav_rejected

    def _nav_succeeded(self):
        return self.status == GoalStatus.STATUS_SUCCEEDED

    def _nav_aborted(self):
        return self.status == GoalStatus.STATUS_ABORTED

    def _send_spin(self, angle=math.pi * 2, time_allowance=15):
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

    def _spin_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Spin request rejected')
            self.result_future = None
            return
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._nav_result)

    # ── Approach pose computation (surface normal based) ──────────────

    def _compute_approach_pose(self, pos, normal, distance=None):
        """Compute approach point along the surface normal.

        The normal points away from the wall/surface. The approach point
        is at pos + normal * distance, and the robot faces back toward pos.
        """
        if distance is None:
            distance = self.approach_distance
        nx, ny = normal
        ax = float(pos[0]) + nx * distance
        ay = float(pos[1]) + ny * distance
        # Face toward the object
        yaw = math.atan2(-ny, -nx)
        return ax, ay, yaw

    def _compute_approach_candidates(self, pos, normal):
        """Generate robust approach candidates (normal, farther, side-offset)."""
        nx, ny = normal
        base = float(self.approach_distance)
        side = float(self.approach_side_offset)
        yaw = math.atan2(-ny, -nx)
        distances = (base, base + 0.15, base + 0.30)
        candidates: list[tuple[float, float, float]] = []

        for dist in distances:
            ax = float(pos[0]) + nx * dist
            ay = float(pos[1]) + ny * dist
            candidates.append((ax, ay, yaw))

        # Perpendicular offsets around the normal-direction approach point.
        px, py = -ny, nx
        bx = float(pos[0]) + nx * base
        by = float(pos[1]) + ny * base
        candidates.append((bx + px * side, by + py * side, yaw))
        candidates.append((bx - px * side, by - py * side, yaw))
        return candidates

    def _pop_nearest_pending_approach(self):
        if not self.pending_approaches:
            return None
        if self.current_pose is None:
            return self.pending_approaches.pop(0)
        rx = float(self.current_pose.pose.position.x)
        ry = float(self.current_pose.pose.position.y)
        rpos = np.array([rx, ry, 0.0])
        best_idx = min(
            range(len(self.pending_approaches)),
            key=lambda i: np.linalg.norm(np.array(self.pending_approaches[i]['pos']) - rpos),
        )
        return self.pending_approaches.pop(best_idx)

    def _start_approach(self, approach):
        self.current_approach = approach
        self.current_approach_candidates = self._compute_approach_candidates(
            approach['pos'], approach['normal'])
        self.current_approach_candidate_index = 0
        approach['attempts'] = 0
        self.state = State.APPROACHING_OBJECT
        self._publish_approaching_object(approach['pos'])
        ax, ay, yaw = self.current_approach_candidates[self.current_approach_candidate_index]
        self._send_nav_goal(ax, ay, yaw)

    # ── Completion check ──────────────────────────────────────────────

    def _all_found(self):
        return (len(self.found_faces) >= self.total_faces and
                len(self.found_rings) >= self.total_rings)

    # ── Main state machine tick ───────────────────────────────────────

    def _tick(self):
        self._publish_mission_status()

        if self.state == State.WAITING_FOR_NAV2:
            self._handle_waiting()
        elif self.state == State.UNDOCKING:
            pass  # async callback handles transition
        elif self.state == State.EXPLORING:
            self._handle_exploring()
        elif self.state == State.APPROACHING_OBJECT:
            self._handle_approaching()
        elif self.state == State.VERIFYING:
            self._handle_verifying()
        elif self.state == State.DONE:
            pass

    def _handle_waiting(self):
        if self.is_docked is None:
            return

        self._check_nav2_states()
        if not self.nav2_ready:
            return
        if not self.initial_pose_received:
            return

        self.get_logger().info('Nav2 is ready!')
        self.start_time = self.get_clock().now()

        if self.is_docked:
            self.state = State.UNDOCKING
            self._do_undock()
        else:
            self.state = State.EXPLORING
            self._send_next_waypoint()

    def _do_undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Undock server not available')
            return

        goal = Undock.Goal()
        future = self.undock_client.send_goal_async(goal)
        future.add_done_callback(self._undock_response)

    def _undock_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_done)

    def _undock_done(self, future):
        self.get_logger().info('Undocked successfully.')
        self.state = State.EXPLORING
        self._send_next_waypoint()

    def _handle_exploring(self):
        # Check if current navigation is done
        if not self._is_nav_complete():
            return

        if self._all_found():
            self._finish()
            return

        if self._nav_aborted():
            # Waypoint aborted — skip to next
            self.get_logger().warn(
                f'Waypoint {self.waypoint_index} navigation aborted, skipping.')
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.waypoint_index = 0
                self.loop_count += 1
                self.get_logger().info(
                    f'Completed waypoint loop {self.loop_count}/{self.max_loops}')
                if self.loop_count >= self.max_loops:
                    self._finish()
                    return
            self._send_next_waypoint()
            return

        if self._nav_succeeded():
            # Move waypoint index first, then optionally run pending approaches
            # before continuing to the next waypoint.
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.waypoint_index = 0
                self.loop_count += 1
                self.get_logger().info(
                    f'Completed waypoint loop {self.loop_count}/{self.max_loops}')
                if self.loop_count >= self.max_loops:
                    self._finish()
                    return

            if self.pending_approaches:
                approach = self._pop_nearest_pending_approach()
                if approach is not None:
                    self._start_approach(approach)
                    return

            if self.spin_at_waypoints:
                self._send_spin()
            else:
                self._send_next_waypoint()

    def _handle_approaching(self):
        if not self._is_nav_complete():
            return

        if self._nav_succeeded():
            # Reached approach pose — enter verify state
            self.state = State.VERIFYING
            self.verify_start_time = self.get_clock().now()
            return

        if self._nav_aborted() or self.nav_rejected:
            approach = self.current_approach
            if approach is None:
                self.state = State.EXPLORING
                self._send_next_waypoint()
                return
            attempts = approach.get('attempts', 0) + 1
            approach['attempts'] = attempts
            self.current_approach_candidate_index += 1

            max_candidates = min(
                len(self.current_approach_candidates),
                max(1, int(self.max_approach_attempts)),
            )
            if self.current_approach_candidate_index < max_candidates:
                ax, ay, yaw = self.current_approach_candidates[self.current_approach_candidate_index]
                self.get_logger().warn(
                    f'Approach aborted (attempt {attempts}/{max_candidates}), trying candidate {self.current_approach_candidate_index + 1}/{max_candidates}.')
                self._send_nav_goal(ax, ay, yaw)
            else:
                # All candidates exhausted — push back to end of queue to try later
                self.get_logger().warn(
                    f'Approach failed after {attempts} attempts, re-queuing for later.')
                self.pending_approaches.append(approach)
                self.current_approach = None
                self.current_approach_candidates = []
                self.current_approach_candidate_index = 0
                self._publish_approaching_object(None, none=True)
                self.state = State.EXPLORING
                self._send_next_waypoint()

    def _handle_verifying(self):
        elapsed = (self.get_clock().now() - self.verify_start_time).nanoseconds / 1e9
        if elapsed < self.verify_pause_sec:
            return

        approach = self.current_approach
        if approach is None:
            self.state = State.EXPLORING
            self._send_next_waypoint()
            return

        if approach['type'] == 'face':
            self.get_logger().info('Verified face — greeting!')
            self._say('Hello!')
        elif approach['type'] == 'ring':
            color = approach.get('color', 'unknown')
            self.get_logger().info(f'Verified ring — announcing color: {color}!')
            self._say(f'I see a {color} ring')

        # Mark this object as successfully greeted
        target_pos = np.array(approach['pos'])
        if approach['type'] == 'face':
            for f in self.found_faces:
                if np.linalg.norm(target_pos - f['pos']) < self.dedup_distance:
                    f['greeted'] = True
                    break
        elif approach['type'] == 'ring':
            for r in self.found_rings:
                if np.linalg.norm(target_pos - r['pos']) < self.dedup_distance:
                    r['greeted'] = True
                    break

        self.current_approach = None
        self.current_approach_candidates = []
        self.current_approach_candidate_index = 0
        self._publish_approaching_object(None, none=True)

        if self._all_found():
            self._finish()
        else:
            self.state = State.EXPLORING
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0

        self.get_logger().info(f'Heading to waypoint {self.waypoint_index}')
        x, y, yaw = self.waypoints[self.waypoint_index]
        self._send_nav_goal(x, y, yaw)
        self._publish_goal_markers()

    def _finish(self):
        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.state = State.DONE
        self._cancel_nav()
        self._publish_approaching_object(None, none=True)
        self.get_logger().info(
            f'Mission complete! Found {len(self.found_faces)} faces and '
            f'{len(self.found_rings)} rings in {elapsed:.1f}s')
        self._say('Mission complete!')

    # ── Visualization ─────────────────────────────────────────────────

    def _publish_approaching_object(self, pos, none=False):
        t = Marker()
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns = 'approaching_object'
        t.id = 0

        if none or pos is None:
            t.action = Marker.DELETE
            self.approaching_object.publish(t)
            return

        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = float(pos[0]) + 0.1
        t.pose.position.y = float(pos[1])
        t.pose.position.z = float(pos[2])
        t.pose.orientation.w = 1.0
        t.scale.z = 0.12
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 0.0
        t.color.a = 1.0
        t.text = 'APPROACHING'
        t.lifetime.sec = 0
        self.approaching_object.publish(t)

    def _publish_goal_markers(self):
        marker_array = MarkerArray()
        markers: list[Marker] = []
        for i, (x, y, yaw) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
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
            m.lifetime.sec = 0
            markers.append(m)

        marker_array.markers = markers
        self.goal_marker_pub.publish(marker_array)

    def _publish_mission_status(self):
        msg = String()
        ring_colors = ', '.join(r['color'] for r in self.found_rings) if self.found_rings else 'none'
        msg.data = (
            f'{self.state.name} | faces {len(self.found_faces)}/{self.total_faces} '
            f'| rings {len(self.found_rings)}/{self.total_rings} '
            f'| waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
            f'| loop {self.loop_count + 1}/{self.max_loops} '
            f'| rings: {ring_colors}'
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
