#!/usr/bin/env python3
"""
Yellow line avoider v2 — Bird's-eye-view warp + Time-To-Intersection.

Geometric redesign of yellow_avoider.py:
  - Compute a homography H from camera intrinsics + TF (once at startup).
  - Warp the yellow HSV mask into a metric bird's-eye view (BEV).
  - Fit a line in BEV pixel space; convert to base_link meters.
  - Compute x_int = X-axis intersection distance (TTI).
  - Proportional angular control: steer harder the closer the line is.

State machine (same roles as v1):
  PATROL / APPROACH_TARGET / INTERACT → proportional STEERING, BACKING fallback
  INSPECT_WORKSTATION                  → publish /yellow_line_seen True/False per tick
  INIT / FOLLOW_BLUE_LINE / DONE       → fully passive
"""

from typing import cast

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import tf2_ros

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

TRANSIENT_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

YELLOW_LO = np.array([18, 100, 80], dtype=np.uint8)
YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

_ACTIVE_STATES = frozenset(("PATROL", "APPROACH_TARGET", "INTERACT"))
_INSPECT_STATES = frozenset(("INSPECT_WORKSTATION",))

# BEV canvas covers [0.5, 2.5] m forward × [-1.0, 1.0] m lateral in base_link.
_BEV_X_NEAR = 0.5   # nearest forward distance (m)
_BEV_X_FAR  = 2.5   # farthest forward distance (m)
_BEV_Y_HALF = 1.0   # half lateral width (m)


def _quat_to_rot(x, y, z, w) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class YellowAvoider2(Node):
    def __init__(self):
        super().__init__("yellow_avoider2")

        self.declare_parameter("camera_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("forward_limit", _BEV_X_FAR)
        self.declare_parameter("near_limit", _BEV_X_NEAR)
        self.declare_parameter("lateral_half", _BEV_Y_HALF)
        self.declare_parameter("pix_per_m", 200)
        self.declare_parameter("min_pixels", 30)
        self.declare_parameter("danger_threshold", 0.8)
        self.declare_parameter("panic_threshold", 0.15)
        self.declare_parameter("kp", 0.3)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("steer_speed", 0.06)
        self.declare_parameter("back_speed", 0.12)
        self.declare_parameter("back_duration", 1.8)

        cam = cast(str, self.get_parameter("camera_topic").value)
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Homography state
        self._K: np.ndarray | None = None
        self._H: np.ndarray | None = None
        self._bev_w = 0
        self._bev_h = 0

        # Camera info subscriber (unsubscribed after first message)
        self._info_sub = self.create_subscription(
            CameraInfo,
            "/top_camera/rgb/preview/camera_info",
            self._info_cb,
            TRANSIENT_QOS,
        )

        # Image subscriber (active always)
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
        self.get_logger().info("YellowAvoider2 starting — waiting for camera_info + TF.")

    # ------------------------------------------------------------------
    # Camera info → K matrix
    # ------------------------------------------------------------------

    def _info_cb(self, msg: CameraInfo):
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info(
            f"Camera K received: fx={self._K[0,0]:.1f} fy={self._K[1,1]:.1f}"
        )
        self.destroy_subscription(self._info_sub)

    # ------------------------------------------------------------------
    # Homography computation
    # ------------------------------------------------------------------

    def _try_build_homography(self):
        """Try to compute H once K and TF are both available."""
        if self._K is None:
            return
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                "top_camera_rgb_camera_optical_frame",
                "base_link",
                rclpy.time.Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        ppm = cast(int, self.get_parameter("pix_per_m").value)
        x_near = cast(float, self.get_parameter("near_limit").value)
        x_far = cast(float, self.get_parameter("forward_limit").value)
        y_half = cast(float, self.get_parameter("lateral_half").value)

        # 4 ground-plane corners in base_link frame (x=forward, y=left, z=0)
        #   order: far-left, far-right, near-right, near-left
        src_3d = np.array([
            [x_far,  -y_half, 0.0],
            [x_far,   y_half, 0.0],
            [x_near,  y_half, 0.0],
            [x_near, -y_half, 0.0],
        ], dtype=np.float64)

        # Transform to camera optical frame: p_cam = R @ p_base + T
        pts_cam = (R @ src_3d.T).T + T  # (4, 3)

        # Project to pixels
        K = self._K
        src_pts = []
        for Xc, Yc, Zc in pts_cam:
            if Zc <= 0:
                self.get_logger().warn("Ground point behind camera — TF may be wrong.")
                return
            u = K[0, 0] * Xc / Zc + K[0, 2]
            v = K[1, 1] * Yc / Zc + K[1, 2]
            src_pts.append([u, v])
        src_pts = np.array(src_pts, dtype=np.float32)

        # BEV canvas dimensions
        w = int(2.0 * y_half * ppm)
        h = int((x_far - x_near) * ppm)

        # BEV layout:
        #   rows: top=x_far (far), bottom=x_near (near)
        #   cols: left=y=-y_half, right=y=+y_half
        dst_pts = np.array([
            [0,     0    ],   # far-left  → top-left
            [w - 1, 0    ],   # far-right → top-right
            [w - 1, h - 1],   # near-right → bottom-right
            [0,     h - 1],   # near-left  → bottom-left
        ], dtype=np.float32)

        self._H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self._bev_w = w
        self._bev_h = h
        self.get_logger().info(
            f"Homography computed. BEV canvas: {w}×{h} px "
            f"({x_near}–{x_far} m forward, ±{y_half} m lateral)"
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Main 50 Hz tick
    # ------------------------------------------------------------------

    def _update(self):
        if self._latest is None:
            return

        if self._H is None:
            self._try_build_homography()
            if self._H is None:
                return

        img = self._latest
        now = self.get_clock().now().nanoseconds / 1e9

        # Yellow mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_K)

        # BEV warp
        bev = cv2.warpPerspective(mask, self._H, (self._bev_w, self._bev_h))

        # Line fit
        line_result = self._fit_line_bev(bev)

        # Dispatch by robot state
        if self.robot_state in _INSPECT_STATES:
            if line_result is not None:
                _, _, x_int = line_result
                in_range = 0.0 < x_int < cast(float, self.get_parameter("forward_limit").value)
                self.yellow_seen.publish(Bool(data=in_range))
            else:
                self.yellow_seen.publish(Bool(data=False))
        elif self.robot_state in _ACTIVE_STATES:
            self._run_avoidance(line_result, now)

        if cast(bool, self.get_parameter("publish_debug_image").value):
            self._publish_debug(img, mask, bev, line_result)

    # ------------------------------------------------------------------
    # Line fitting in BEV
    # ------------------------------------------------------------------

    def _fit_line_bev(self, bev: np.ndarray):
        """Return (vx_m, vy_m, x_int) or None if no line detected.

        vx_m, vy_m: line direction unit vector in base_link meters (x=forward, y=lateral)
        x_int: X-axis intersection distance (TTI) in meters
        """
        min_px = cast(int, self.get_parameter("min_pixels").value)
        ys, xs = np.where(bev > 0)
        if len(xs) < min_px:
            return None

        pts = np.column_stack([xs, ys]).astype(np.float32)
        # fitLine returns [vx, vy, x0, y0] — direction vector + point on line (pixel space)
        result = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
        vx_px, vy_px, x0_px, y0_px = float(result[0]), float(result[1]), float(result[2]), float(result[3])

        ppm = cast(int, self.get_parameter("pix_per_m").value)
        x_near = cast(float, self.get_parameter("near_limit").value)
        y_half = cast(float, self.get_parameter("lateral_half").value)

        # Convert BEV pixel point → base_link meters
        # BEV rows: top=x_far, bottom=x_near → x_m = x_near + (bev_h - y0_px) / ppm
        # BEV cols: left=y=-y_half, right=y=+y_half → y_m = (x0_px / ppm) - y_half
        x0_m = x_near + (self._bev_h - y0_px) / ppm
        y0_m = (x0_px / ppm) - y_half

        # Direction in BEV pixel space → base_link meters
        # BEV col increases → y increases; BEV row increases → x decreases
        # vx_bev (col direction) → +y in base_link
        # vy_bev (row direction) → -x in base_link
        # So: (vx_m, vy_m) direction in base_link (forward=x, left=y):
        vx_m = -vy_px   # row direction flipped to forward
        vy_m =  vx_px   # col direction is lateral

        # Normalize
        mag = max(np.hypot(vx_m, vy_m), 1e-6)
        vx_m /= mag
        vy_m /= mag

        # Line equation: A*x + B*y + C = 0
        # A = vy_m, B = -vx_m, C = vx_m*y0_m - vy_m*x0_m
        A = vy_m
        B = -vx_m
        C = vx_m * y0_m - vy_m * x0_m

        forward_limit = cast(float, self.get_parameter("forward_limit").value)

        if abs(A) < 1e-6:
            # Line parallel to X-axis → no forward intersection
            return None

        x_int = -C / A

        if x_int < 0 or x_int > forward_limit:
            return None

        return (vx_m, vy_m, x_int)

    # ------------------------------------------------------------------
    # Avoidance state machine
    # ------------------------------------------------------------------

    def _run_avoidance(self, line_result, now: float):
        danger = cast(float, self.get_parameter("danger_threshold").value)
        panic = cast(float, self.get_parameter("panic_threshold").value)
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

        if line_result is None:
            if self._avoider_state == "STEERING":
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
                self.get_logger().info("Yellow cleared — CLEAR.")
            return

        vx_m, vy_m, x_int = line_result

        if x_int > danger:
            # Far enough — let Nav2 drive
            if self._avoider_state == "STEERING":
                self._pub_vel(0.0, 0.0)
                self._avoider_state = "CLEAR"
            return

        if x_int < panic:
            self._avoider_state = "BACKING"
            self._back_end = now + cast(float, self.get_parameter("back_duration").value)
            self._pub_vel(0.0, 0.0)
            self.get_logger().warn(f"PANIC x_int={x_int:.2f}m — BACKING.")
            return

        # Proportional steering
        # Cross product vx*y0 - vy*x0 → positive = line to the left → steer right (neg angular)
        # We need a point on the line: use (0 + t*vx_m, 0 + ...) but we have (x0_m, y0_m)
        # side = sign of perpendicular distance from origin to line
        # = sign(A*0 + B*0 + C) = sign(C) = sign(vx_m*y0_m - vy_m*x0_m)
        # But we don't have y0_m here — recompute from A,B,C:
        # Actually: angular sign = -sign(A) since A = vy_m and line is at y>0 (left)
        # Simpler: use A itself. If A>0 (vy_m>0), line intercepts at positive X from left → steer right
        A = vy_m
        angular = float(np.clip(-np.sign(A) * kp / max(x_int, 0.05), -max_ang, max_ang))
        linear = steer_spd * float(np.clip(x_int / 1.5, 0.2, 1.0))

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

    def _publish_debug(self, img, mask, bev, line_result):
        h_img, w_img = img.shape[:2]

        # Left panel: BEV mask (jet colormap)
        bev_color = cv2.applyColorMap(bev, cv2.COLORMAP_JET)
        bev_resized = cv2.resize(bev_color, (w_img // 2, h_img))

        # Right panel: raw image with yellow overlay
        right = img.copy()
        overlay = right.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.35, right, 0.65, 0, right)

        # Status text
        if line_result is not None:
            vx_m, vy_m, x_int = line_result
            angle_deg = float(np.degrees(np.arctan2(vy_m, vx_m)))
            state_str = self._avoider_state
            txt = f"d={x_int:.2f}m a={angle_deg:.0f}d {state_str}"
            col = (0, 100, 255) if self._avoider_state != "CLEAR" else (0, 200, 0)
        else:
            txt = "CLEAR (no line)"
            col = (0, 200, 0)

        _label(right, txt, (8, 20), col)
        right_resized = cv2.resize(right, (w_img // 2, h_img))

        debug = np.concatenate([bev_resized, right_resized], axis=1)
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
