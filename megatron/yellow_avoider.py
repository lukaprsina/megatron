#!/usr/bin/env python3
"""
Yellow line avoider — camera-only, no costmap editing.

PATROL / APPROACH_TARGET / INTERACT:
  Yellow in tight danger ROI (bottom 80–95 %, center 40–60 %) → stop + back-up
  at 50 Hz on BOTH /cmd_vel topics, overrunning Nav2's ~10-20 Hz output.
  Tight ROI prevents false triggers on yellow boxes adjacent to the path.

INSPECT_WORKSTATION:
  Yellow in danger ROI → publish True/False to /yellow_line_seen (default QoS)
  every tick.  No velocity commands — controller owns cmd_vel during inspection.

PATROL / INIT / FOLLOW_BLUE_LINE / DONE:
  Fully passive — no avoidance, no /yellow_line_seen publishing.  Nav2 + costmap
  are the sole navigation authority during PATROL.  Debug image keeps updating.
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
        self.declare_parameter("danger_zone_top", 0.80)
        self.declare_parameter("danger_zone_bottom", 0.95)
        self.declare_parameter("roi_left", 0.40)
        self.declare_parameter("roi_right", 0.60)
        self.declare_parameter("danger_px_threshold", 150)
        self.declare_parameter("back_speed", 0.12)
        self.declare_parameter("back_duration", 1.8)

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
        self._spoke = False
        self._latest = None

        # 50 Hz — beats Nav2 (~10-20 Hz) in last-write-wins race during backing
        self.create_timer(0.02, self._update)
        self.get_logger().info(f"YellowAvoider ready on {cam}.")

    def _state_cb(self, msg: String):
        if msg.data != self.robot_state:
            self.robot_state = msg.data
            self._avoider_state = "CLEAR"
            self._spoke = False

    def _image_cb(self, msg: Image):
        # TODO: CPU savings: why burn cycles on a downward cam when the avoider's off duty?
        # however, the debug image then doesn't display.
        # if (
        #     self.robot_state not in _ACTIVE_STATES
        #     and self.robot_state not in _INSPECT_STATES
        # ):
        #     return
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _update(self):
        # same as _image_cb
        # if (
        #     self.robot_state not in _ACTIVE_STATES
        #     and self.robot_state not in _INSPECT_STATES
        # ):
        #     return
        if self._latest is None:
            return

        img = self._latest
        h, w = img.shape[:2]
        now = self.get_clock().now().nanoseconds / 1e9

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_K)

        xl = int(w * cast(float, self.get_parameter("roi_left").value))
        xr = int(w * cast(float, self.get_parameter("roi_right").value))
        dt = int(h * cast(float, self.get_parameter("danger_zone_top").value))
        db = int(h * cast(float, self.get_parameter("danger_zone_bottom").value))
        danger_px = int(cv2.countNonZero(mask[dt:db, xl:xr]))
        thresh = cast(int, self.get_parameter("danger_px_threshold").value)

        if self.robot_state in _INSPECT_STATES:
            # Publish on every tick so controller sees False when line disappears,
            # not a stale True from a previous phase.
            self.yellow_seen.publish(Bool(data=danger_px >= thresh))
        elif self.robot_state in _ACTIVE_STATES:
            self._run_avoidance(danger_px, thresh, now)

        if cast(bool, self.get_parameter("publish_debug_image").value):
            self._publish_debug(img, mask, xl, xr, dt, db, danger_px, thresh)

    def _run_avoidance(self, danger_px: int, thresh: int, now: float):
        if self._avoider_state == "BACKING":
            if now >= self.back_end:
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self._spoke = False
                self.get_logger().info("Backing done — CLEAR.")
            else:
                self._pub_vel(-cast(float, self.get_parameter("back_speed").value), 0.0)
        elif danger_px >= thresh:
            self._pub_vel(0.0, 0.0)
            if not self._spoke:
                self.get_logger().warn(
                    f"PROHIBITED — {danger_px} px yellow in danger zone."
                )
                self.speaker.speak("Prohibited zone!")
                self._spoke = True
                self._avoider_state = "BACKING"
                self.back_end = now + cast(
                    float, self.get_parameter("back_duration").value
                )
        else:
            self._avoider_state = "CLEAR"

    def _publish_debug(self, img, mask, xl, xr, dt, db, danger_px, thresh):
        debug = img.copy()
        overlay = debug.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.40, debug, 0.60, 0, debug)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 2)
        d_col = (0, 0, 255) if danger_px >= thresh else (80, 80, 80)
        cv2.rectangle(debug, (xl, dt), (xr, db), d_col, 2)
        self._label(debug, f"DANGER {danger_px}px", (xl + 4, dt + 16), d_col)
        mode = (
            f"INSPECT|{self._avoider_state}"
            if self.robot_state in _INSPECT_STATES
            else self._avoider_state
        )
        col = (0, 200, 0) if self._avoider_state == "CLEAR" else (0, 100, 255)
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
