#!/usr/bin/env python3

import enum
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

# ── Color detection (teammate's HSV tuned against this Gazebo world) ──────────
BLUE_LO = np.array([82, 80, 80])
BLUE_HI = np.array([102, 255, 255])
BLUE_MIN_CHANNEL = 100  # B, G channels must each exceed this
BLUE_DOMINANCE_MARGIN = 30  # B,G − R must each exceed this
MIN_BLUE_PX = 100  # pixels in full frame required to declare "line found"
MORPH_KERNEL_SIZE = 5

# ── Direction ROI bands (fractions of image width; bottom ROI_Y_FRAC strip) ───
ROI_Y_FRAC = 0.40
ROI_LEFT_X1 = 0.35
ROI_STRAIGHT_X0 = 0.30
ROI_STRAIGHT_X1 = 0.70
ROI_RIGHT_X0 = 0.65
ROI_MIN_PX = 30
ROI_MIN_RATIO = 0.02

# ── Control ───────────────────────────────────────────────────────────────────
LINEAR_SPEED = 0.12  # m/s — slow: split mode and near-obstacle
FAST_LINEAR_SPEED = 0.30  # m/s — fast: clear straight
KP = 0.8
MAX_ANGULAR = 0.6  # rad/s
LEFT_BIAS = 0.25  # extra rad/s leftward at junctions
ANGULAR_ALPHA = 0.35  # EMA smoothing factor
SPLIT_EXIT_HOLD = 0.5  # s centred before exiting split mode

# ── LiDAR — forward is at −π/2 in this Gazebo world ──────────────────────────
LIDAR_FORWARD_ANGLE = -math.pi / 2
LIDAR_CONE_HALF = 0.2618  # ±15°
LIDAR_SLOW_DIST = 0.55  # m
LIDAR_STOP_DIST = 0.37  # m — triggers U-turn

# ── Recovery ──────────────────────────────────────────────────────────────────
LINE_LOST_TIMEOUT = 2.0  # s
UTURN_SPEED = 0.5  # rad/s
UTURN_DURATION = 6.5  # s (~180°)
SEARCH_TURN_SPEED = 0.25  # rad/s


class Mode(enum.Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    FOLLOW = "FOLLOW"
    UTURN = "UTURN"


class BlueLineFollower(Node):
    def __init__(self):
        super().__init__("blue_line_follower")

        self._bridge = CvBridge()
        self._active = False
        self._mode = Mode.IDLE
        self._frame: np.ndarray | None = None
        self._front_dist = float("inf")
        self._prev_angular = 0.0
        self._line_lost_since: float | None = None
        self._uturn_end: float | None = None
        self._split_active = False
        self._split_exit_since: float | None = None

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)

        self.create_subscription(String, "/robot_state", self._state_cb, 10)
        self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._image_cb, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )

        self.create_timer(0.1, self._tick)
        self.get_logger().info("blue_line_follower ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        was_active = self._active
        self._active = msg.data == "FOLLOW_BLUE_LINE"
        if self._active and not was_active:
            self.get_logger().info("FOLLOW_BLUE_LINE — starting line search")
            self._mode = Mode.SEARCH
            self._prev_angular = 0.0
            self._line_lost_since = None
            self._uturn_end = None
            self._split_active = False
            self._split_exit_since = None
        elif not self._active and was_active:
            self.get_logger().info("Deactivating blue line follower")
            self._mode = Mode.IDLE
            self._stop()

    def _image_cb(self, msg: Image):
        if not self._active:
            return
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan):
        if msg.angle_increment == 0.0:
            return
        fwd_idx = int(
            round((LIDAR_FORWARD_ANGLE - msg.angle_min) / msg.angle_increment)
        )
        cone = int(round(LIDAR_CONE_HALF / msg.angle_increment))
        lo = max(0, fwd_idx - cone)
        hi = min(len(msg.ranges) - 1, fwd_idx + cone)
        valid = [
            r for r in msg.ranges[lo : hi + 1] if msg.range_min < r < msg.range_max
        ]
        self._front_dist = min(valid) if valid else float("inf")

    # ── Perception ────────────────────────────────────────────────────────────

    def _detect(self):
        """Return (cx_norm, directions, pixel_count) or (None, set(), 0).

        cx_norm is computed from the bottom ROI_Y_FRAC strip only, giving
        near-field steering that avoids perspective-induced corner cutting.
        """
        if self._frame is None:
            return None, set(), 0

        hsv = cv2.cvtColor(self._frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

        # Cyan dominance: B and G must dominate R
        bgr = self._frame.astype(np.int16)
        B, G, R = bgr[:, :, 0], bgr[:, :, 1], bgr[:, :, 2]
        dominance = (
            (B >= BLUE_MIN_CHANNEL)
            & (G >= BLUE_MIN_CHANNEL)
            & ((B - R) >= BLUE_DOMINANCE_MARGIN)
            & ((G - R) >= BLUE_DOMINANCE_MARGIN)
        ).astype(np.uint8) * 255
        mask = cv2.bitwise_and(mask, dominance)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        if int(np.count_nonzero(mask)) < MIN_BLUE_PX:
            return None, set(), 0

        H, W = mask.shape
        y0 = int(H * (1.0 - ROI_Y_FRAC))
        roi_mask = mask[y0:, :]

        # Centroid from bottom strip only — avoids look-ahead bias from far pixels
        moments = cv2.moments(roi_mask)
        if moments["m00"] == 0:
            return None, set(), 0
        cx = moments["m10"] / moments["m00"]
        cx_norm = (cx - W / 2.0) / (W / 2.0)  # −1 = far left, +1 = far right

        # Direction bands within the same bottom strip
        def _band(x0_frac: float, x1_frac: float):
            x0, x1 = int(W * x0_frac), int(W * x1_frac)
            region = roi_mask[:, x0:x1]
            return int(np.count_nonzero(region)), region.size

        directions: set[str] = set()
        for name, (xlo, xhi) in (
            ("left", (0.0, ROI_LEFT_X1)),
            ("straight", (ROI_STRAIGHT_X0, ROI_STRAIGHT_X1)),
            ("right", (ROI_RIGHT_X0, 1.0)),
        ):
            cnt, area = _band(xlo, xhi)
            if area > 0 and cnt >= ROI_MIN_PX and cnt / area >= ROI_MIN_RATIO:
                directions.add(name)

        return cx_norm, directions, int(np.count_nonzero(roi_mask))

    # ── Main control loop ─────────────────────────────────────────────────────

    def _tick(self):
        if not self._active:
            return

        cx_norm, directions, _ = self._detect()
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._mode == Mode.UTURN:
            if self._uturn_end is not None and now < self._uturn_end:
                self._pub_twist(0.0, UTURN_SPEED)
            else:
                self.get_logger().info("U-turn complete — resuming SEARCH")
                self._mode = Mode.SEARCH
            return

        if self._mode == Mode.SEARCH:
            if cx_norm is not None:
                self.get_logger().info("Line acquired — entering FOLLOW")
                self._mode = Mode.FOLLOW
                self._line_lost_since = None
            else:
                self._pub_twist(0.0, SEARCH_TURN_SPEED)
            return

        # FOLLOW mode ──────────────────────────────────────────────────────────

        if self._front_dist <= LIDAR_STOP_DIST:
            self.get_logger().info(f"Obstacle at {self._front_dist:.2f} m — U-turn")
            self._start_uturn(now)
            return

        if cx_norm is None:
            if self._line_lost_since is None:
                self._line_lost_since = now
            elif now - self._line_lost_since >= LINE_LOST_TIMEOUT:
                self.get_logger().info("Line lost — U-turn")
                self._start_uturn(now)
            else:
                self._pub_twist(LINEAR_SPEED, self._prev_angular)
            return
        else:
            self._line_lost_since = None

        # Split-mode state machine
        has_branch = "left" in directions or "right" in directions
        if has_branch and not self._split_active:
            self._split_active = True
            self._split_exit_since = None

        if self._split_active:
            centred = abs(cx_norm) < 0.30 and "straight" in directions
            if centred:
                if self._split_exit_since is None:
                    self._split_exit_since = now
                elif now - self._split_exit_since >= SPLIT_EXIT_HOLD:
                    self._split_active = False
                    self._split_exit_since = None
            else:
                self._split_exit_since = None

        # Speed
        slow = self._split_active or self._front_dist <= LIDAR_SLOW_DIST
        linear = LINEAR_SPEED if slow else FAST_LINEAR_SPEED

        # Steering: positive angular.z = turn left
        if self._split_active:
            if "left" in directions:
                raw = -KP * cx_norm + LEFT_BIAS
            else:
                steer = min(cx_norm, 0.0) if "straight" not in directions else cx_norm
                raw = -KP * steer
        else:
            raw = -KP * cx_norm
        raw = float(np.clip(raw, -MAX_ANGULAR, MAX_ANGULAR))

        angular = ANGULAR_ALPHA * raw + (1.0 - ANGULAR_ALPHA) * self._prev_angular
        self._prev_angular = angular
        self._pub_twist(linear, angular)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _start_uturn(self, now: float):
        self._mode = Mode.UTURN
        self._split_active = False
        self._split_exit_since = None
        self._uturn_end = now + UTURN_DURATION
        self._pub_twist(0.0, UTURN_SPEED)

    def _pub_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._pub_twist(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = BlueLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
