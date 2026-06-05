#!/usr/bin/env python3
"""
Yellow line avoider v2 — image-space TTI (Time-To-Intersection).

Improvement over v1: instead of looking only at the bottom 8% of the frame,
fit a line through ALL yellow pixels in the lower 70% of the image, then
compute where that line intersects the robot's forward path (center column).
The row of this intersection is a direct proxy for "how close is the crossing
point" — closer → lower row value remaining → steer harder.

No TF, no BEV warp, no camera_info required.

State machine (same as v1):
  PATROL / APPROACH_TARGET / INTERACT → proportional STEERING, BACKING fallback
  INSPECT_WORKSTATION                  → publish /yellow_line_seen per tick
  INIT / FOLLOW_BLUE_LINE / DONE       → fully passive
"""

from typing import cast

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

YELLOW_LO = np.array([18, 100, 80], dtype=np.uint8)
YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

_ACTIVE_STATES = frozenset(("PATROL", "APPROACH_TARGET", "INTERACT"))
_INSPECT_STATES = frozenset(("INSPECT_WORKSTATION",))


class YellowAvoider2(Node):
    def __init__(self):
        super().__init__("yellow_avoider2")

        self.declare_parameter("camera_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("publish_debug_image", True)

        # Fraction of image height below which we look for yellow pixels [0..1]
        # It's inverted, we scan 1-scan_top_frac from the bottom, so smaller
        self.declare_parameter("scan_top_frac", 0.7)

        # Line must cross center column below this row fraction to start steering
        # Only react when crossing is in bottom 1-trigger_frac of frame
        self.declare_parameter("trigger_frac", 0.75)

        # Fraction of image height remaining (h - v_cross)/h below which BACKING
        # line crossing is within bottom panic_frac of image = very close
        self.declare_parameter("panic_frac", 0.05)

        self.declare_parameter("min_pixels", 150)
        self.declare_parameter("kp", 15.0)  # gain in pixel units
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("steer_speed", 0.06)
        self.declare_parameter("back_speed", 0.12)
        self.declare_parameter("back_duration", 1.8)

        cam = cast(str, self.get_parameter("camera_topic").value)
        self.bridge = CvBridge()

        self.create_subscription(Image, cam, self._image_cb, SENSOR_QOS)
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self.cmd_unstamped = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.cmd_stamped = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.yellow_seen = self.create_publisher(Bool, "/yellow_line_seen", 10)
        self.debug_pub = self.create_publisher(Image, "/yellow_line/debug_image", 10)

        self.robot_state = "INIT"
        self._avoider_state = "CLEAR"
        self._back_end = 0.0
        self._latest: np.ndarray | None = None

        self.create_timer(0.02, self._update)  # 50 Hz
        self.get_logger().info(f"YellowAvoider2 ready on {cam}.")

    def _state_cb(self, msg: String):
        if msg.data != self.robot_state:
            self.robot_state = msg.data
            self._avoider_state = "CLEAR"
            self._back_end = 0.0

    def _image_cb(self, msg: Image):
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _update(self):
        if self._latest is None:
            return

        img = self._latest
        h, w = img.shape[:2]
        now = self.get_clock().now().nanoseconds / 1e9

        # Yellow mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_K)

        # Restrict to lower scan zone
        scan_top = int(h * cast(float, self.get_parameter("scan_top_frac").value))
        scan_mask = mask.copy()
        scan_mask[:scan_top, :] = 0

        # Fit line
        line_result = self._fit_line(scan_mask, w, h)

        # Dispatch by state
        if self.robot_state in _INSPECT_STATES:
            if line_result is not None:
                _vx, _vy, _x0, _y0, v_cross = line_result
                trigger_row = int(
                    h * cast(float, self.get_parameter("trigger_frac").value)
                )
                in_range = v_cross > trigger_row
                self.yellow_seen.publish(Bool(data=in_range))
            else:
                self.yellow_seen.publish(Bool(data=False))
        elif self.robot_state in _ACTIVE_STATES:
            self._run_avoidance(line_result, h, w, now)

        if cast(bool, self.get_parameter("publish_debug_image").value):
            self._publish_debug(img, scan_mask, line_result, h, w)

    # ------------------------------------------------------------------
    # Line fitting
    # ------------------------------------------------------------------

    def _fit_line(self, scan_mask, w, h):
        """Return (vx, vy, x0, y0, v_cross) or None.

        Finds the lowest-reaching yellow contour and fits a line through its
        filled pixels — ignores disconnected noise (sky cubes, far boxes).
        """
        contours, _ = cv2.findContours(
            scan_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        min_px = cast(int, self.get_parameter("min_pixels").value)
        best_contour = None
        best_bottom = -1
        for cnt in contours:
            if cv2.contourArea(cnt) < min_px:
                continue
            max_y = int(cnt[:, 0, 1].max())
            if max_y > best_bottom:
                best_bottom = max_y
                best_contour = cnt

        if best_contour is None:
            return None

        cnt_mask = np.zeros_like(scan_mask)
        cv2.drawContours(cnt_mask, [best_contour], -1, (255,), cv2.FILLED)
        ys, xs = np.where(cnt_mask > 0)
        pts = np.column_stack([xs, ys]).astype(np.float32)

        result = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
        vx = float(result[0][0])
        vy = float(result[1][0])
        x0 = float(result[2][0])
        y0 = float(result[3][0])

        cx = w / 2.0
        if abs(vx) < 1e-6:
            v_cross = y0
        else:
            t = (cx - x0) / vx
            v_cross = y0 + t * vy

        v_cross = float(np.clip(v_cross, 0, h))
        return (vx, vy, x0, y0, v_cross)

    # ------------------------------------------------------------------
    # Avoidance state machine
    # ------------------------------------------------------------------

    def _run_avoidance(self, line_result, h: int, w: int, now: float):
        trigger_row = int(h * cast(float, self.get_parameter("trigger_frac").value))
        panic_remain = int(h * cast(float, self.get_parameter("panic_frac").value))
        kp = cast(float, self.get_parameter("kp").value)
        max_ang = cast(float, self.get_parameter("max_angular").value)
        steer_spd = cast(float, self.get_parameter("steer_speed").value)

        if self._avoider_state == "BACKING":
            if now >= self._back_end:
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self.get_logger().info("Backing done — CLEAR.")
            else:
                self._pub_vel(-cast(float, self.get_parameter("back_speed").value), 0.0)
            return

        if line_result is None or line_result[4] < trigger_row:
            # No threatening line detected
            if self._avoider_state == "STEERING":
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self.get_logger().info("Yellow cleared — CLEAR.")
            return

        vx, vy, x0, y0, v_cross = line_result
        remaining = h - v_cross  # pixels remaining before line is "at robot"

        if remaining < panic_remain:
            self._avoider_state = "BACKING"
            self._back_end = now + cast(
                float, self.get_parameter("back_duration").value
            )
            self._pub_vel(0.0, 0.0)
            self.get_logger().warn(
                f"PANIC v_cross={v_cross:.0f} (remain={remaining:.0f}px) — BACKING."
            )
            return

        # Steering sign: which side of the robot's forward path (center column) is the line?
        # Use signed perpendicular distance from center-bottom (cx, h) to the fitted line.
        # Line eq: A*u + B*v + C = 0 where A=vy, B=-vx, C=vx*y0 - vy*x0
        A = vy
        B = -vx
        C = vx * y0 - vy * x0
        cx = w / 2.0
        # Distance from (cx, h) to line — positive means steer negative angular (right)
        dist_sign = float(np.sign(A * cx + B * h + C))
        if dist_sign == 0.0:
            dist_sign = 1.0

        # TTI-proportional angular: closer → harder steer
        # dist_sign > 0: line passes on the right → steer left (pos angular)
        # dist_sign < 0: line passes on the left  → steer right (neg angular)
        angular = float(
            np.clip(dist_sign * kp / max(remaining, 1.0), -max_ang, max_ang)
        )
        linear = steer_spd * float(np.clip(remaining / (h * 0.4), 0.15, 1.0))

        if self._avoider_state != "STEERING":
            self.get_logger().info(
                f"STEERING v_cross={v_cross:.0f}px remain={remaining:.0f}px ang={angular:.2f}"
            )
        self._avoider_state = "STEERING"
        self._pub_vel(linear, angular)

    # ------------------------------------------------------------------
    # Velocity publishing
    # ------------------------------------------------------------------

    def _pub_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_unstamped.publish(t)

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist.linear.x = linear
        ts.twist.angular.z = angular
        self.cmd_stamped.publish(ts)

    # ------------------------------------------------------------------
    # Debug image
    # ------------------------------------------------------------------

    def _publish_debug(self, img, scan_mask, line_result, h, w):
        debug = img.copy()

        # Yellow overlay
        overlay = debug.copy()
        overlay[scan_mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.40, debug, 0.60, 0, debug)

        # Scan zone boundary
        scan_top = int(h * cast(float, self.get_parameter("scan_top_frac").value))
        cv2.line(debug, (0, scan_top), (w, scan_top), (80, 80, 80), 1)

        cx = w // 2

        if line_result is not None:
            vx, vy, x0, y0, v_cross = line_result

            # Draw fitted line across the image
            if abs(vx) > 1e-6:
                t_left = (0 - x0) / vx
                t_right = (w - x0) / vx
                y_left = int(y0 + t_left * vy)
                y_right = int(y0 + t_right * vy)
                cv2.line(debug, (0, y_left), (w, y_right), (255, 100, 0), 2)

            # Draw intersection point with center column
            v_cross_i = int(np.clip(v_cross, 0, h - 1))
            cv2.circle(debug, (cx, v_cross_i), 6, (0, 0, 255), -1)
            cv2.line(debug, (cx, v_cross_i), (cx, h), (0, 0, 200), 1)

            remaining = h - v_cross
            trigger_row = int(h * cast(float, self.get_parameter("trigger_frac").value))
            in_range = v_cross > trigger_row
            state_str = self._avoider_state
            col = (0, 100, 255) if state_str != "CLEAR" else (0, 200, 0)
            _label(
                debug,
                f"v={v_cross:.0f} rem={remaining:.0f}px {state_str}",
                (8, 20),
                col,
            )
            if in_range:
                _label(debug, "THREAT", (8, 38), (0, 0, 255))
        else:
            _label(debug, "NO LINE", (8, 20), (0, 200, 0))

        # Center column marker
        cv2.line(debug, (cx, h - 20), (cx, h), (200, 200, 200), 1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception:
            pass

    def destroy_node(self):
        self._pub_vel(0.0, 0.0)
        super().destroy_node()


def _label(img, text, pos, color):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 2)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)


def main(args=None):
    rclpy.init(args=args)
    node = YellowAvoider2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
