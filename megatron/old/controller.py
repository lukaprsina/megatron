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

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from megatron.speech import Speaker

import numpy as np
import yaml
from pathlib import Path

from lifecycle_msgs.srv import GetState

class State(Enum):
    WAITING_FOR_NAV2 = auto()
    UNDOCKING = auto()
    EXPLORING = auto()
    APPROACHING_FACE = auto()
    APPROACHING_RING = auto()
    SPINNING = auto()
    DONE = auto()


amcl_pose_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# Function for waypoints 
def _quaternion_to_yaw(q_list):
    """Convert quaternion (x,y,z,w) to yaw angle (radians)."""
    try:
        w, x, y, z = q_list
        #x, y, z, w = q_list
    except Exception:
        return 0.0
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

# Function for waypoints 
def load_waypoints_from_yaml(path):
    """Load waypoints from a YAML file.

    Supports formats:
      - dict under top-level key 'waypoints' (mapping names -> {pose, orientation})
      - top-level list of entries like [x, y, yaw] or {pose: [...], orientation: [...]}
    Returns list of (x, y, yaw) tuples.
    """
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


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('dedup_distance', 0.8)
        self.declare_parameter('approach_distance', 0.8)
        self.declare_parameter('spin_at_waypoints', False)
        self.declare_parameter('total_faces', 3)
        self.declare_parameter('total_rings', 2)
        self.declare_parameter('waypoints_file', 'waypoints/test1.yaml')

        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value
        self.approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value
        self.spin_at_waypoints = self.get_parameter('spin_at_waypoints').get_parameter_value().bool_value
        self.total_faces = self.get_parameter('total_faces').get_parameter_value().integer_value
        self.total_rings = self.get_parameter('total_rings').get_parameter_value().integer_value

        # Speech
        self.speaker = Speaker()
        self.speaker.set_node_logger(self)
        
        # Navigation state
        self.state = State.WAITING_FOR_NAV2
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None
        self.current_pose = None
        self.waypoint_index = 0
        self.start_time = None
        self.nodes = ['amcl', 'bt_navigator']
        self.states = {n: "Unknown" for n in self.nodes}
        
        # Load waypoints from YAML if provided, fallback to global WAYPOINTS
        try:
            wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        except Exception:
            wp_file = 'waypoints/test1.yaml'

        try:
            self.waypoints = load_waypoints_from_yaml(wp_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wp_file}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load waypoints from {wp_file}: {e}; using default WAYPOINTS')
            raise e

        # Detection state
        self.found_faces = []  # [np.array([x,y,z])]
        self.found_rings = []  # [{'pos': np.array, 'color': str}]
        self.pending_face = None   # PoseStamped to approach
        self.pending_ring = None   # {'pose': PoseStamped, 'color': str}
        self.last_ring_color = None  # Track paired color messages

        # Subscribers
        self.create_subscription(DockStatus, 'dock_status', self._dock_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_callback, amcl_pose_qos)
        self.create_subscription(PoseStamped, '/detected_faces', self._face_callback, 10)
        self.create_subscription(PoseStamped, '/detected_rings', self._ring_callback, 10)
        self.create_subscription(String, '/detected_ring_color', self._ring_color_callback, 10)

        # Publishers
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_client = ActionClient(self, Undock, 'undock')


        # Create clients for Nav2 checks
        self.nav2_lifecycle_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in self.nodes}
        self.nav2_ready = False

        # Main loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('Mission controller initialized.')

    # ── Detection callbacks ───────────────────────────────────────────

    def _face_callback(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # Dedup against already found faces
        for f in self.found_faces:
            if np.linalg.norm(pos - f) < self.dedup_distance:
                return

        self.get_logger().info(
            f'New face detected at ({pos[0]:.2f}, {pos[1]:.2f}), total: {len(self.found_faces) + 1}')

        self.found_faces.append(pos)

        # Queue for approach if we're exploring
        if self.state == State.EXPLORING:
            self.pending_face = msg

    def _ring_callback(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        for r in self.found_rings:
            if np.linalg.norm(pos - r['pos']) < self.dedup_distance:
                return

        # Wait for paired color message
        color = self.last_ring_color if self.last_ring_color else 'unknown'
        self.last_ring_color = None

        self.get_logger().info(
            f'New ring ({color}) detected at ({pos[0]:.2f}, {pos[1]:.2f}), total: {len(self.found_rings) + 1}')

        self.found_rings.append({'pos': pos, 'color': color})

        if self.state == State.EXPLORING:
            self.pending_ring = {'pose': msg, 'color': color}

    def _ring_color_callback(self, msg: String):
        self.last_ring_color = msg.data

    # ── Nav2 / dock callbacks ─────────────────────────────────────────

    def _dock_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.initial_pose_received = True
        self.current_pose = msg.pose

    def _feedback_callback(self, msg):
        # self._logger.info(f'1!Action feedback: {msg.feedback}')
        self.feedback = msg.feedback

    # ── Navigation helpers ────────────────────────────────────────────

    def check_states(self):
        #self.get_logger().info('sending req to check nav2')
        for name, client in self.nav2_lifecycle_clients.items():
            if client.service_is_ready():
                # Use a lambda to pass the node 'name' into the callback
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda f, n=name: self.state_cb(f, n))
    
    def state_cb(self, future, node_name):
        #self.get_logger().info(f'Checking state for {node_name}')
        try:
            res = future.result()
            self.states[node_name] = res.current_state.label
            #self.get_logger().info(f"STATUS -> {node_name}: {self.states[node_name]}")
            
            # Check if everything is Active (ID 3)
            if all(s == "active" for s in self.states.values()):
                self.get_logger().info("ALL NODES ACTIVE: System is ready for goals.")
                self.nav2_ready = True
            else:  
                self.nav2_ready = False
        except Exception as e:
            self.get_logger().error(f"Service call failed for {node_name}: {e}")

    def _yaw_to_quaternion(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _send_nav_goal(self, x, y, yaw):
        """Send a NavigateToPose goal. Non-blocking."""
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
        future = self.nav_client.send_goal_async(goal_msg, self._feedback_callback)
        future.add_done_callback(self._nav_goal_response)
        return True

    def _nav_goal_response(self, future):
        self.status = 0
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn(f'Navigation goal rejected ')
            self.nav_rejected = True
            if self.nav2_ready:
                self.get_logger().warn('This is unexpected since Nav2 is active')

            self.result_future = None
            return
        else: 
            self.nav_rejected = False
            
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        self.status = result.status if result else GoalStatus.STATUS_ABORTED
        
        # when a goal is complete, check if we were exploring and if so, move to the next waypoint
        if self.status == GoalStatus.STATUS_SUCCEEDED and self.state == State.EXPLORING:
            self.waypoint_index += 1  # Move to next waypoint on success

    def _cancel_nav(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.result_future = None

    def _is_nav_complete(self):
        if self.result_future is None:
            return True
        return self.result_future.done() and not self.nav_rejected

    def _send_spin(self, angle=math.pi * 2, time_allowance=15):
        """Send a Spin action. Non-blocking."""
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

    # ── Approach pose computation ─────────────────────────────────────

    def _compute_approach_pose(self, target_pose: PoseStamped):
        """Compute a pose ~approach_distance meters from target, facing it.
        Uses current robot position to determine approach direction."""
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y

        # Default: approach from robot's current position direction
        rx, ry = 0.0, 0.0
        if self.current_pose is not None:
            rx = self.current_pose.pose.position.x
            ry = self.current_pose.pose.position.y

        dx = tx - rx
        dy = ty - ry
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.1:
            # Too close, just face forward
            return tx, ty, 0.0

        # Approach pose: offset from target toward the robot
        scale = self.approach_distance / dist
        ax = tx - dx * scale
        ay = ty - dy * scale
        yaw = math.atan2(dy, dx)  # Face the target

        return ax, ay, yaw

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
            self._handle_undocking()
        elif self.state == State.EXPLORING:
            self._handle_exploring()
        elif self.state == State.APPROACHING_FACE:
            self._handle_approaching_face()
        elif self.state == State.APPROACHING_RING:
            self._handle_approaching_ring()
        elif self.state == State.SPINNING:
            self._handle_spinning()
        elif self.state == State.DONE:
            pass

    def _handle_waiting(self):
        """Wait for Nav2 to become active, then undock or start exploring."""
        if self.is_docked is None:
            return  # Still waiting for dock status

        # Check if Nav2 is ready (simplified: check if action server is available)
        self.check_states()
           
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

    def _handle_undocking(self):
        pass  # Waiting for async undock callback

    def _handle_exploring(self):
        # Check if a detection needs approach
        if self.pending_face is not None:
            self._cancel_nav()
            ax, ay, yaw = self._compute_approach_pose(self.pending_face)
            self.pending_face = None
            self.state = State.APPROACHING_FACE
            self._send_nav_goal(ax, ay, yaw)
            return

        if self.pending_ring is not None:
            self._cancel_nav()
            ax, ay, yaw = self._compute_approach_pose(self.pending_ring['pose'])
            self.pending_ring = None
            self.state = State.APPROACHING_RING
            self._send_nav_goal(ax, ay, yaw)
            return

        # Check if current navigation is done
        if self._is_nav_complete():
            if self._all_found():
                self._finish()
                return

            if self.spin_at_waypoints:
                self.state = State.SPINNING
                self._send_spin()
            else:
                self._send_next_waypoint()

    def _handle_approaching_face(self):
        if self._is_nav_complete():
            self.get_logger().info('Reached face — greeting!')
            self.speaker.speak('Hello!')

            if self._all_found():
                self._finish()
            else:
                self.state = State.EXPLORING
                self._send_next_waypoint()

    def _handle_approaching_ring(self):
        if self._is_nav_complete():
            # Find the color of the most recently added ring
            if self.found_rings:
                color = self.found_rings[-1]['color']
            else:
                color = 'unknown'
            self.get_logger().info(f'Reached ring — announcing color: {color}!')
            self.speaker.speak(f'I see a {color} ring')

            if self._all_found():
                self._finish()
            else:
                self.state = State.EXPLORING
                self._send_next_waypoint()

    def _handle_spinning(self):
        if self._is_nav_complete():
            if self._all_found():
                self._finish()
                return
            self.state = State.EXPLORING
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            # Loop back to the start of waypoints
            self.waypoint_index = 0
            self.get_logger().info('Completed all waypoints, looping.')

        self.get_logger().info(f'Heading to waypoint {self.waypoint_index} ')
        x, y, yaw = self.waypoints[self.waypoint_index]
        #self.waypoint_index += 1
        self._send_nav_goal(x, y, yaw)
        self._publish_goal_markers()

    def _finish(self):
        if self.start_time is None:
            elapsed = 0.0
        else:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.state = State.DONE
        self._cancel_nav()
        self.get_logger().info(
            f'Mission complete! Found {len(self.found_faces)} faces and '
            f'{len(self.found_rings)} rings in {elapsed:.1f}s')
        self.speaker.speak('Mission complete!')

    # ── Visualization ─────────────────────────────────────────────────

    def _publish_goal_markers(self):
        """Publish waypoint goal markers for RViz."""
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
                # Visited: dim green
                m.color.r = 0.3
                m.color.g = 0.7
                m.color.b = 0.3
                m.color.a = 0.5
            elif i == self.waypoint_index:
                # Current target: bright yellow
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                # Future: blue
                m.color.r = 0.3
                m.color.g = 0.3
                m.color.b = 1.0
                m.color.a = 0.5
            m.lifetime.sec = 0
            markers.append(m)

        marker_array.markers = markers
        self.goal_marker_pub.publish(marker_array)

    def _publish_mission_status(self):
        msg = String()
        msg.data = (
            f'{self.state.name} | faces {len(self.found_faces)}/{self.total_faces} '
            f'| rings {len(self.found_rings)}/{self.total_rings} '
            f'| waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
            f'| last ring: {self.last_ring_color}'
        )
        self.get_logger().info(f'Mission status: {msg.data}, status: {self.status} f: {self.feedback}')
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
