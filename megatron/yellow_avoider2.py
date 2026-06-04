#!/usr/bin/env python3
"""
Yellow line avoider v2 — bird's-eye geometry with proportional TTI steering.

Homography computed once at startup from camera_info + TF. No calibration.
PATROL/APPROACH_TARGET/INTERACT → TTI-based proportional steering + BACKING panic.
INSPECT_WORKSTATION → publish /yellow_line_seen from d_x proximity.
"""

import math
from typing import cast

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

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

_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
_YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
_ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
_WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class YellowAvoiderV2(Node):
    def __init__(self):
        super().__init__("yellow_avoider")

        self._init_params()

        self.bridge = CvBridge()
        self.speaker = Speaker()
        self.speaker.set_node_logger(self)

        self.K = None
        self.H = None
        self._bev_w = 0
        self._bev_h = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cam_topic = cast(str, self.get_parameter("camera_topic").value)
        self.create_subscription(Image, cam_topic, self._image_cb, SENSOR_QOS)
        self.create_subscription(
            CameraInfo, "/top_camera/rgb/preview/camera_info",
            self._cam_info_cb, 10,
        )
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self.cmd_unstamped = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.cmd_stamped = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.yellow_seen = self.create_publisher(Bool, "/yellow_line_seen", 10)
        self.debug_pub = self.create_publisher(Image, "/yellow_line/debug_image", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/yellow_line/markers", 10)

        self.robot_state = "INIT"
        self._latest = None
        self._back_end = 0.0
        self._backing = False
        self._spoke = False
        self._last_marker_pub = 0.0
        self._no_line_published = False

        self.create_timer(0.02, self._update)
        self.get_logger().info(f"YellowAvoiderV2 ready on {cam_topic}.")

    def _init_params(self):
        p = self.declare_parameter
        p("camera_topic", "/top_camera/rgb/preview/image_raw")
        p("publish_debug_image", True)
        p("publish_markers", True)

        p("bev_x_min", 0.0)
        p("bev_x_max", 2.0)
        p("bev_y_min", -1.0)
        p("bev_y_max", 1.0)
        p("bev_scale", 0.01)

        p("min_line_pixels", 30)
        p("inspection_threshold", 0.15)

        p("danger_threshold", 0.8)
        p("panic_threshold", 0.2)
        p("steer_kp", 0.8)
        p("max_angular", 0.6)
        p("steer_speed", 0.08)

        p("back_speed", 0.12)
        p("back_duration", 1.8)

    # ── camera_info + homography ────────────────────────────────────────

    def _cam_info_cb(self, msg: CameraInfo):
        if self.K is not None:
            return
        k = msg.k
        self.K = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)
        self.get_logger().info("Camera intrinsics received.")
        self._try_compute_homography()

    def _try_compute_homography(self):
        if self.K is None:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                "top_camera_rgb_camera_optical_frame",
                "base_link",
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().debug(f"TF not ready for homography: {e}")
            return
        self.H = self._compute_homography(self.K, t)
        if self.H is not None:
            self.get_logger().info(
                f"Homography ready — BEV {self._bev_w}x{self._bev_h}"
            )

    def _compute_homography(self, K, transform):
        x_min = cast(float, self.get_parameter("bev_x_min").value)
        x_max = cast(float, self.get_parameter("bev_x_max").value)
        y_min = cast(float, self.get_parameter("bev_y_min").value)
        y_max = cast(float, self.get_parameter("bev_y_max").value)

        base_pts = np.array([
            [x_min, y_min, 0.0],
            [x_min, y_max, 0.0],
            [x_max, y_max, 0.0],
            [x_max, y_min, 0.0],
        ], dtype=np.float64)

        t = transform.transform.translation
        q = transform.transform.rotation
        tx, ty, tz = t.x, t.y, t.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
        ], dtype=np.float64)
        t_vec = np.array([tx, ty, tz], dtype=np.float64)

        cam_pts = (R @ base_pts.T).T + t_vec

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        src_pts = np.zeros((4, 2), dtype=np.float32)
        for i in range(4):
            X, Y, Z = cam_pts[i]
            if Z <= 0:
                self.get_logger().error(
                    f"Ground point {i} Z={Z:.3f} behind camera — wrong TF frame?"
                )
                return None
            src_pts[i] = [fx * X / Z + cx, fy * Y / Z + cy]

        scale = cast(float, self.get_parameter("bev_scale").value)
        self._bev_w = int((y_max - y_min) / scale)
        self._bev_h = int((x_max - x_min) / scale)

        dst_pts = np.zeros((4, 2), dtype=np.float32)
        for i, (x, y, _) in enumerate(base_pts):
            dst_pts[i] = [(y - y_min) / scale, (x_max - x) / scale]

        return cv2.getPerspectiveTransform(src_pts, dst_pts)

    # ── callbacks ───────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _state_cb(self, msg: String):
        if msg.data != self.robot_state:
            self.robot_state = msg.data
            self._backing = False
            self._spoke = False

    # ── main loop ───────────────────────────────────────────────────────

    def _update(self):
        if self._latest is None or self.H is None:
            return

        img = self._latest
        now = self.get_clock().now().nanoseconds / 1e9

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_K)

        bev = cv2.warpPerspective(mask, self.H, (self._bev_w, self._bev_h))

        line = self._fit_line(bev)

        if self.robot_state in _INSPECT_STATES:
            self._handle_inspect(line)
        elif self.robot_state in _ACTIVE_STATES:
            self._handle_avoid(line, now)

        if cast(bool, self.get_parameter("publish_debug_image").value):
            self._publish_debug(img, mask, bev, line)

        if cast(bool, self.get_parameter("publish_markers").value):
            self._publish_markers(line)

    # ── line fitting ────────────────────────────────────────────────────

    def _fit_line(self, bev):
        pts = cv2.findNonZero(bev)
        if pts is None or len(pts) < cast(int, self.get_parameter("min_line_pixels").value):
            return None

        result = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy = float(result[0][0]), float(result[1][0])
        x0, y0 = float(result[2][0]), float(result[3][0])

        scale = cast(float, self.get_parameter("bev_scale").value)
        x_max = cast(float, self.get_parameter("bev_x_max").value)
        y_min = cast(float, self.get_parameter("bev_y_min").value)

        dx_base = -vy * scale
        dy_base = vx * scale

        px_base = x_max - y0 * scale
        py_base = y_min + x0 * scale

        a = -dy_base
        b = dx_base
        c = -(a * px_base + b * py_base)

        d_x = float("inf")
        y_at_robot = float("inf")
        if abs(a) > 1e-6:
            d_x = -c / a
        if abs(b) > 1e-6:
            y_at_robot = -c / b

        line_yaw = math.atan2(dy_base, dx_base)

        return {
            "valid": True,
            "d_x": d_x,
            "y_at_robot": y_at_robot,
            "line_yaw": line_yaw,
            "a": a, "b": b, "c": c,
            "px": px_base, "py": py_base,
            "dx": dx_base, "dy": dy_base,
            "vx": vx, "vy": vy, "x0": x0, "y0": y0,
        }

    # ── avoidance ───────────────────────────────────────────────────────

    def _handle_inspect(self, line):
        seen = False
        if line is not None:
            thresh = cast(float, self.get_parameter("inspection_threshold").value)
            seen = line["valid"] and 0.0 < line["d_x"] < thresh
        self.yellow_seen.publish(Bool(data=seen))

    def _handle_avoid(self, line, now):
        danger = cast(float, self.get_parameter("danger_threshold").value)
        panic = cast(float, self.get_parameter("panic_threshold").value)

        if self._backing:
            if now >= self._back_end:
                self._pub_vel(0.0, 0.0)
                self._backing = False
                self._spoke = False
                self.get_logger().info("Backing done — CLEAR.")
            else:
                speed = cast(float, self.get_parameter("back_speed").value)
                self._pub_vel(-speed, 0.0)
            return

        if line is None or not line["valid"] or line["d_x"] <= 0 or line["d_x"] > danger:
            self._pub_vel(0.0, 0.0)
            return

        d_x = line["d_x"]

        if d_x < panic:
            self._pub_vel(0.0, 0.0)
            if not self._spoke:
                self.get_logger().warn(f"Panic: line at {d_x:.2f}m — BACKING.")
                self.speaker.speak("Prohibited zone!")
                self._spoke = True
            self._backing = True
            self._back_end = now + cast(
                float, self.get_parameter("back_duration").value
            )
            return

        y_at = line["y_at_robot"]
        if abs(y_at) < 1e-3 or math.isinf(y_at):
            steer_dir = -1.0
        else:
            steer_dir = -math.copysign(1.0, y_at)

        kp = cast(float, self.get_parameter("steer_kp").value)
        max_ang = cast(float, self.get_parameter("max_angular").value)
        angular = steer_dir * kp / max(d_x, 0.05)
        angular = float(np.clip(angular, -max_ang, max_ang))

        speed = cast(float, self.get_parameter("steer_speed").value)
        self._pub_vel(speed, angular)

    # ── velocity output ─────────────────────────────────────────────────

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

    # ── debug image ─────────────────────────────────────────────────────

    def _publish_debug(self, img, mask, bev, line):
        debug = img.copy()
        overlay = debug.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.35, debug, 0.65, 0, debug)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 1)

        if line is not None:
            self._label(debug, f"{line['d_x']:.2f}m", (8, 20), (0, 255, 0))
            self._label(debug,
                        f"yaw {math.degrees(line['line_yaw']):+.0f}deg",
                        (8, 38), (180, 180, 0))

        bev_col = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        if line is not None:
            h_bev, w_bev = bev.shape
            vx, vy = line["vx"], line["vy"]
            x0, y0 = line["x0"], line["y0"]
            le = 1000.0
            p1 = (int(x0 - vx * le), int(y0 - vy * le))
            p2 = (int(x0 + vx * le), int(y0 + vy * le))
            cv2.line(bev_col, p1, p2, (0, 255, 255), 2)

            cx = int(w_bev / 2)
            robot_row = int(
                (cast(float, self.get_parameter("bev_x_max").value) - 0.0)
                / cast(float, self.get_parameter("bev_scale").value)
            )
            cv2.circle(bev_col, (cx, robot_row), 4, (0, 255, 0), -1)

            if not math.isinf(line["d_x"]) and line["d_x"] > 0:
                ix = int(
                    (0.0 - cast(float, self.get_parameter("bev_y_min").value))
                    / cast(float, self.get_parameter("bev_scale").value)
                )
                iy = int(
                    (cast(float, self.get_parameter("bev_x_max").value) - line["d_x"])
                    / cast(float, self.get_parameter("bev_scale").value)
                )
                cv2.circle(bev_col, (ix, iy), 6, (0, 0, 255), -1)

        side_panel = np.pad(bev_col, ((0, 0), (4, 4), (0, 0)), constant_values=60)

        if debug.shape[0] != side_panel.shape[0]:
            total_h = min(debug.shape[0], side_panel.shape[0])
            debug = debug[:total_h]
            side_panel = side_panel[:total_h]
        combined = np.hstack([debug, side_panel])

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined, "bgr8"))
        except Exception:
            pass

    # ── RViz markers ────────────────────────────────────────────────────

    def _publish_markers(self, line):
        now = self.get_clock().now().nanoseconds / 1e9

        if line is None or not line.get("valid"):
            if not self._no_line_published:
                ma = MarkerArray()
                for ns in ("line", "intersection", "steer_arrow", "info"):
                    ma.markers.append(self._mk(ns, Marker.DELETE))
                self.marker_pub.publish(ma)
                self._no_line_published = True
            return
        self._no_line_published = False

        if now - self._last_marker_pub < 0.25:
            return
        self._last_marker_pub = now

        y_span = 3.0
        px, py = line["px"], line["py"]
        dx, dy = line["dx"], line["dy"]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return
        ux, uy = dx / norm, dy / norm

        p_start = (px - ux * y_span, py - uy * y_span)
        p_end = (px + ux * y_span, py + uy * y_span)

        m_line = self._mk("line", Marker.LINE_STRIP)
        m_line.pose.orientation.w = 1.0
        m_line.scale.x = 0.03
        m_line.color = _YELLOW
        m_line.points = [
            Point(x=float(p_start[0]), y=float(p_start[1]), z=0.0),
            Point(x=float(p_end[0]), y=float(p_end[1]), z=0.0),
        ]
        ma.markers.append(m_line)

        d_x = line["d_x"]
        if not math.isinf(d_x) and d_x > 0:
            m_pt = self._mk("intersection", Marker.SPHERE)
            m_pt.pose.position = Point(x=float(d_x), y=0.0, z=0.05)
            m_pt.pose.orientation.w = 1.0
            m_pt.scale.x = m_pt.scale.y = m_pt.scale.z = 0.08
            m_pt.color = _RED
            ma.markers.append(m_pt)

            m_arrow = self._mk("steer_arrow", Marker.ARROW)
            steer_dir = -math.copysign(1.0, line["y_at_robot"]) if (
                abs(line["y_at_robot"]) > 1e-3 and not math.isinf(line["y_at_robot"])
            ) else -1.0
            m_arrow.pose.position = Point(x=float(d_x), y=0.0, z=0.05)
            half = steer_dir * math.pi / 4.0
            m_arrow.pose.orientation.z = math.sin(half)
            m_arrow.pose.orientation.w = math.cos(half)
            m_arrow.scale.x = 0.15
            m_arrow.scale.y = 0.04
            m_arrow.scale.z = 0.04
            m_arrow.color = _GREEN
            ma.markers.append(m_arrow)

        m_text = self._mk("info", Marker.TEXT_VIEW_FACING)
        m_text.pose.position = Point(x=-0.15, y=0.0, z=0.45)
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.06
        m_text.color = _WHITE
        d_str = f"{d_x:.2f}" if not math.isinf(d_x) else "inf"
        m_text.text = f"Dist: {d_str}m | Yaw: {math.degrees(line['line_yaw']):+.0f}deg"
        ma.markers.append(m_text)

        self.marker_pub.publish(ma)

    def _mk(self, ns, mtype, action=Marker.ADD):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.type = mtype
        m.action = action
        if action == Marker.DELETE:
            m.id = 0
            m.scale.x = m.scale.y = m.scale.z = 0.0
            m.color.a = 0.0
            m.pose.orientation.w = 1.0
            return m
        m.id = 0
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        m.pose.orientation.w = 1.0
        return m

    # ── helpers ─────────────────────────────────────────────────────────

    @staticmethod
    def _label(img, text, pos, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    def destroy_node(self):
        self._pub_vel(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YellowAvoiderV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
