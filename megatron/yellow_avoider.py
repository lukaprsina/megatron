#!/usr/bin/env python3
"""
Yellow line avoider — camera-only, no costmap editing.

APPROACH_TARGET / INTERACT / PATROL:
  Three tight ROIs along the bottom of the frame: CENTER, LEFT, RIGHT.
  CENTER contour-bottom triggers detection.  If a safe-zone ROI (opposite
  side of the line) is clear → STEER forward + angular away from the line
  while Nav2 re-plans.  If both sides blocked → BACKING fallback.

INSPECT_WORKSTATION:
  CENTER contour-bottom → publish True/False to /yellow_line_seen
  (default QoS) every tick.  No velocity commands.

INIT / FOLLOW_BLUE_LINE / DONE:
  Fully passive — no avoidance, no /yellow_line_seen publishing.
  Debug image keeps updating in all states.
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

from megatron.speech import Speaker

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


class YellowAvoider(Node):
    def __init__(self):
        super().__init__("yellow_avoider")

        self.declare_parameter("camera_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("danger_zone_top", 0.92)
        self.declare_parameter("danger_zone_bottom", 1.0)
        self.declare_parameter("roi_left", 0.45)
        self.declare_parameter("roi_right", 0.55)
        self.declare_parameter("back_speed", 0.12)
        self.declare_parameter("back_duration", 1.8)
        self.declare_parameter("steer_speed", 0.06)
        self.declare_parameter("steer_angular", 0.4)
        self.declare_parameter("safe_zone_left_l", 0.15)
        self.declare_parameter("safe_zone_left_r", 0.30)
        self.declare_parameter("safe_zone_right_l", 0.70)
        self.declare_parameter("safe_zone_right_r", 0.85)

        cam = cast(str, self.get_parameter("camera_topic").value)
        self.bridge = CvBridge()
        self.speaker = Speaker()
        self.speaker.set_node_logger(self)

        self.create_subscription(Image, cam, self._image_cb, SENSOR_QOS)
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self.cmd_unstamped = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.cmd_stamped = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.yellow_seen = self.create_publisher(Bool, "/yellow_line_seen", 10)
        self.debug_pub = self.create_publisher(Image, "/yellow_line/debug_image", 10)

        self.robot_state = "INIT"
        self._avoider_state = "CLEAR"
        self.back_end = 0.0
        self._steer_dir = 1
        self._safe_side = None
        self._spoke = False
        self._latest = None

        # 50 Hz — beats Nav2 (~10-20 Hz) in last-write-wins race during backing
        self.create_timer(0.02, self._update)
        self.get_logger().info(f"YellowAvoider ready on {cam}.")

    def _state_cb(self, msg: String):
        if msg.data != self.robot_state:
            self.robot_state = msg.data
            self._avoider_state = "CLEAR"
            self.back_end = 0.0
            self._safe_side = None
            self._spoke = False

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

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_K)

        xc_l = int(w * cast(float, self.get_parameter("roi_left").value))
        xc_r = int(w * cast(float, self.get_parameter("roi_right").value))
        xl_l = int(w * cast(float, self.get_parameter("safe_zone_left_l").value))
        xl_r = int(w * cast(float, self.get_parameter("safe_zone_left_r").value))
        xr_l = int(w * cast(float, self.get_parameter("safe_zone_right_l").value))
        xr_r = int(w * cast(float, self.get_parameter("safe_zone_right_r").value))
        dt = int(h * cast(float, self.get_parameter("danger_zone_top").value))
        db = int(h * cast(float, self.get_parameter("danger_zone_bottom").value))

        roi_c = mask[dt:db, xc_l:xc_r]
        roi_l = (
            mask[dt:db, xl_l:xl_r]
            if xl_r > xl_l
            else np.zeros((db - dt, 1), dtype=np.uint8)
        )
        roi_r = (
            mask[dt:db, xr_l:xr_r]
            if xr_r > xr_l
            else np.zeros((db - dt, 1), dtype=np.uint8)
        )

        c_reaches, centroid_x = self._analyze_contours(roi_c)
        l_reaches, _ = self._analyze_contours(roi_l)
        r_reaches, _ = self._analyze_contours(roi_r)
        danger_px = int(cv2.countNonZero(roi_c))

        safe_clear = True
        if c_reaches and centroid_x is not None:
            crop_w = roi_c.shape[1]
            if centroid_x < crop_w / 2:
                self._steer_dir = -1  # line on left → steer right
                self._safe_side = "R"
                safe_clear = not r_reaches
            else:
                self._steer_dir = 1  # line on right → steer left
                self._safe_side = "L"
                safe_clear = not l_reaches
        else:
            self._safe_side = None

        if self.robot_state in _INSPECT_STATES:
            self.yellow_seen.publish(Bool(data=c_reaches))
        elif self.robot_state in _ACTIVE_STATES:
            self._run_avoidance(c_reaches, safe_clear, now)

        if cast(bool, self.get_parameter("publish_debug_image").value):
            self._publish_debug(
                img,
                mask,
                xc_l,
                xc_r,
                xl_l,
                xl_r,
                xr_l,
                xr_r,
                dt,
                db,
                danger_px,
                c_reaches,
                l_reaches,
                r_reaches,
            )

    @staticmethod
    def _analyze_contours(mask_crop):
        contours, _ = cv2.findContours(
            mask_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return False, None
        bottom_y = mask_crop.shape[0] - 1
        best_cx = None
        best_n = 0
        for cnt in contours:
            pts = cnt[:, 0, :]
            if np.any(pts[:, 1] >= bottom_y - 3):
                cx = float(np.mean(pts[:, 0]))
                n = len(pts)
                if n > best_n:
                    best_cx, best_n = cx, n
        if best_n > 0:
            return True, best_cx
        return False, None

    def _run_avoidance(self, c_reaches: bool, safe_clear: bool, now: float):
        if self._avoider_state == "STEERING":
            if not c_reaches:
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self._spoke = False
                self.get_logger().info("Yellow cleared — CLEAR.")
            elif not safe_clear:
                self._avoider_state = "BACKING"
                self.back_end = now + cast(
                    float, self.get_parameter("back_duration").value
                )
                self._pub_vel(0.0, 0.0)
                self.get_logger().warn("Safe zone blocked — BACKING.")
            else:
                speed = cast(float, self.get_parameter("steer_speed").value)
                ang = self._steer_dir * cast(
                    float, self.get_parameter("steer_angular").value
                )
                self._pub_vel(speed, ang)
        elif self._avoider_state == "BACKING":
            if now >= self.back_end:
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self._spoke = False
                self.get_logger().info("Backing done — CLEAR.")
            else:
                self._pub_vel(-cast(float, self.get_parameter("back_speed").value), 0.0)
        elif c_reaches:
            if safe_clear:
                self._avoider_state = "STEERING"
                self.get_logger().info(f"STEERING away ({self._steer_dir:+d}).")
            else:
                self._pub_vel(0.0, 0.0)
                if not self._spoke:
                    self.get_logger().warn("PROHIBITED — yellow line under robot.")
                    self.speaker.speak("Prohibited zone!")
                    self._spoke = True
                    self._avoider_state = "BACKING"
                    self.back_end = now + cast(
                        float, self.get_parameter("back_duration").value
                    )
        else:
            self._avoider_state = "CLEAR"

    def _publish_debug(
        self,
        img,
        mask,
        xc_l,
        xc_r,
        xl_l,
        xl_r,
        xr_l,
        xr_r,
        dt,
        db,
        danger_px,
        c_reaches,
        l_reaches,
        r_reaches,
    ):
        debug = img.copy()
        overlay = debug.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.40, debug, 0.60, 0, debug)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 2)

        dim = (80, 80, 80)
        red = (0, 0, 255)
        grn = (0, 198, 0)

        # Center ROI — red when triggered
        col_c = red if c_reaches else dim
        cv2.rectangle(debug, (xc_l, dt), (xc_r, db), col_c, 2)

        # Safe-zone ROIs — green when clear, red when blocked (only if relevant)
        col_l = (
            grn
            if self._safe_side == "L" and not l_reaches
            else (red if self._safe_side == "L" else dim)
        )
        cv2.rectangle(debug, (xl_l, dt), (xl_r, db), col_l, 2)
        col_r = (
            grn
            if self._safe_side == "R" and not r_reaches
            else (red if self._safe_side == "R" else dim)
        )
        cv2.rectangle(debug, (xr_l, dt), (xr_r, db), col_r, 2)

        status = f"{danger_px}px FEET" if c_reaches else f"{danger_px}px"
        self._label(debug, status, (xc_l + 4, dt + 16), col_c)
        mode = (
            f"INSPECT|{self._avoider_state}"
            if self.robot_state in _INSPECT_STATES
            else self._avoider_state
        )
        col = (0, 200, 0) if self._avoider_state == "CLEAR" else (0, 100, 255)
        if self._avoider_state == "STEERING":
            col = (255, 140, 0)
            self._label(debug, f"STEER {self._steer_dir:+d}", (8, 40), col)
        self._label(debug, mode, (8, 20), col)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception:
            pass

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

    @staticmethod
    def _label(img, text, pos, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 2)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)

    def destroy_node(self):
        self._pub_vel(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YellowAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
