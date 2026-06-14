#!/usr/bin/env python3
"""
workstation_detector2.py — Line-based workstation detector (v2).

Replaces the blob-based v1 (workstation_detector.py) which failed because:
  - ITM _is_compact() requires observations within 0.35 m — belts scatter 2 m+
  - 50 ms TF timeout discarded ~80–90 % of valid frames silently
  - No floor ROI → barrels and rings flooded the tracker
  - Centroid instead of PCA → wrong position for L-shaped belts
  - Identity orientation → useless for approach navigation

This version ports the teammate's line_localizator + workstation_recorder pipeline:
  - Floor ROI (bottom half) eliminates non-floor objects
  - PCA on 3D points + flatness filter (stds[0] < 0.03 m) rejects barrels/rings
  - 200 ms TF timeout (4× longer than v1)
  - Map-frame belt-length filter (≥ 1.5 m) rejects small objects
  - 10-sighting median accumulation then lock (no ITM compaction)

Publishes /detected_workstations (Marker, ns="red"|"green") with the belt
center in map frame — same format and semantics as v1 so task2_controller
needs no changes.
"""

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from megatron.perception_utils import (
    extract_3d_points_from_pc2,
    quaternion_to_rotation_matrix,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

DEBUG_TOPIC = "/workstation_detector/debug"

# HSV thresholds — identical to v1
RED_LO1 = np.array([0, 80, 60], dtype=np.uint8)
RED_HI1 = np.array([10, 255, 255], dtype=np.uint8)
RED_LO2 = np.array([170, 80, 60], dtype=np.uint8)
RED_HI2 = np.array([180, 255, 255], dtype=np.uint8)
GREEN_LO = np.array([40, 80, 60], dtype=np.uint8)
GREEN_HI = np.array([80, 255, 255], dtype=np.uint8)
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

MIN_CONTOUR_AREA = 80  # px² — reduced from v1's 2000; floor ROI already prunes FPs
MIN_ASPECT_RATIO = 3.0  # belt shape; rings/barrels have aspect ~1
MIN_POINTS_3D = 15  # minimum 3D points to attempt PCA
MAX_FLAT_STD = 0.03  # m — flat belt ≈ 0.01 m, barrel ≈ 0.15 m in normal direction
MIN_BELT_LENGTH_M = 0.8  # map-frame belt length; barrels project ~0.2 m
CONFIRM_COUNT = 10  # sightings before locking (median of these)
TF_TIMEOUT_S = 0.2  # 4× longer than v1's 0.05 s

_INACTIVE_STATES = frozenset(
    ("INSPECT_WORKSTATION", "NAVIGATE_ROOM2_ENTRY", "FOLLOW_BLUE_LINE", "DONE")
)
_MARKER_COLORS = {"red": (1.0, 0.0, 0.0), "green": (0.0, 1.0, 0.0)}
_MARKER_IDS = {"red": 0, "green": 1}


class WorkstationDetector2(Node):
    def __init__(self):
        super().__init__("workstation_detector2")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._bridge = CvBridge()

        self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._image_cb, SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2,
            "/top_camera/rgb/preview/depth/points",
            self._pc2_cb,
            SENSOR_QOS,
        )
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self._pub = self.create_publisher(Marker, "/detected_workstations", 10)
        self._debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 1)

        self._latest_pc2: PointCloud2 | None = None
        self._robot_state: str = "INIT"

        # Per-color accumulation: list of (center_x, center_y) in map frame
        self._candidates: dict[str, list[tuple[float, float]]] = {
            "red": [],
            "green": [],
        }
        self._locked: dict[str, bool] = {"red": False, "green": False}
        self._locked_markers: dict[str, Marker] = {}
        self._last_reject: dict[str, str] = {"red": "no contour", "green": "no contour"}
        self._debug_frame_skip = 0  # publish every 3rd frame

        self.create_timer(2.0, self._republish_locked)
        self.get_logger().info(
            "WorkstationDetector2 ready  "
            "(floor-ROI + PCA flatness + 200 ms TF + median lock)"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String) -> None:
        self._robot_state = msg.data

    def _pc2_cb(self, msg: PointCloud2) -> None:
        self._latest_pc2 = msg

    def _image_cb(self, msg: Image) -> None:
        inactive = self._robot_state in _INACTIVE_STATES
        no_pc2 = self._latest_pc2 is None
        all_locked = self._locked["red"] and self._locked["green"]

        try:
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        h, w = img.shape[:2]

        if not inactive and not no_pc2 and not all_locked:
            pc2_snap: PointCloud2 = self._latest_pc2  # type: ignore[assignment]
            floor_img = img.copy()
            floor_img[: h // 2, :] = 0
            hsv = cv2.cvtColor(floor_img, cv2.COLOR_BGR2HSV)
            masks = dict(self._build_masks(hsv))
            for color, mask in masks.items():
                if not self._locked[color]:
                    self._process_color(color, mask, h, w, pc2_snap)
        else:
            masks = {"red": None, "green": None}

        # Publish debug image every 3rd frame to keep bandwidth reasonable
        self._debug_frame_skip = (self._debug_frame_skip + 1) % 3
        if self._debug_frame_skip == 0 and self._debug_pub.get_subscription_count() > 0:
            reason = (
                "inactive"
                if inactive
                else ("no_pc2" if no_pc2 else "all_locked" if all_locked else "")
            )
            self._publish_debug(img, masks, h, w, reason)

    def _publish_debug(
        self, img: np.ndarray, masks: dict, h: int, w: int, skip_reason: str
    ) -> None:
        debug = img.copy()

        # Floor ROI boundary
        cv2.line(debug, (0, h // 2), (w, h // 2), (255, 255, 0), 1)
        cv2.putText(
            debug,
            "floor ROI",
            (4, h // 2 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 0),
            1,
        )

        # Overlay color masks as tinted regions + contours
        overlay_colors = {"red": (0, 0, 200), "green": (0, 200, 0)}
        for color, mask in masks.items():
            if mask is None:
                continue
            bgr = overlay_colors[color]
            tint = np.zeros_like(debug)
            tint[mask > 0] = bgr
            cv2.addWeighted(tint, 0.35, debug, 1.0, 0, debug)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(debug, contours, -1, bgr, 1)

        # Status text per color
        status_lines = []
        for color in ("red", "green"):
            n = len(self._candidates[color])
            if self._locked[color]:
                status_lines.append(f"{color}: LOCKED ({n} sightings)")
            elif skip_reason:
                status_lines.append(f"{color}: SKIP ({skip_reason})")
            else:
                status_lines.append(
                    f"{color}: {n}/{CONFIRM_COUNT} — {self._last_reject[color]}"
                )

        if skip_reason:
            status_lines.insert(0, f"state={self._robot_state}")

        for i, line in enumerate(status_lines):
            cv2.putText(
                debug,
                line,
                (4, 14 + i * 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                line,
                (4, 14 + i * 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                line,
                (4, 14 + i * 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        try:
            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception:
            pass

    # ── Detection pipeline ─────────────────────────────────────────────────────

    def _build_masks(self, hsv: np.ndarray):
        red = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LO1, RED_HI1),
            cv2.inRange(hsv, RED_LO2, RED_HI2),
        )
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, MORPH_K)

        green = cv2.inRange(hsv, GREEN_LO, GREEN_HI)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, MORPH_K)

        return [("red", red), ("green", green)]

    def _process_color(
        self,
        color: str,
        mask: np.ndarray,
        h: int,
        w: int,
        pc2_msg: PointCloud2,
    ) -> None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Take the single largest contour that passes shape filters
        best_cnt = None
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            _, (rw, rh), _ = cv2.minAreaRect(cnt)
            if rw < 1 or rh < 1 or max(rw, rh) / min(rw, rh) < MIN_ASPECT_RATIO:
                continue
            if area > best_area:
                best_area = area
                best_cnt = cnt

        if best_cnt is None:
            self._last_reject[color] = "no contour"
            return

        cnt_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(cnt_mask, [best_cnt], -1, (255,), cv2.FILLED)

        pts_cam = extract_3d_points_from_pc2(cnt_mask, pc2_msg, max_range=5.0)
        if len(pts_cam) < MIN_POINTS_3D:
            self._last_reject[color] = f"pts_3d={len(pts_cam)}<{MIN_POINTS_3D}"
            return

        # PCA: fit a 3D line through the point cloud
        mean = pts_cam.mean(axis=0)
        centered = pts_cam - mean
        cov = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(cov)  # ascending order
        stds = np.sqrt(np.abs(eigenvalues))

        # Flatness filter: stds[0] = spread in the direction normal to the floor.
        # A flat belt has stds[0] ≈ 0.01 m; a barrel or ring has stds[0] ≈ 0.15 m.
        if stds[0] >= MAX_FLAT_STD:
            self._last_reject[color] = f"flat_std={stds[0]:.3f}>={MAX_FLAT_STD}"
            self.get_logger().debug(
                f"[{color}] rejected: flat_std={stds[0]:.3f} >= {MAX_FLAT_STD}"
            )
            return

        # Principal axis = along the belt length (largest eigenvalue)
        direction = eigenvectors[:, 2]
        proj = pts_cam @ direction
        half_len = max(float(np.std(proj)) * 2.0, 0.2)

        # TF camera → map; 200 ms timeout instead of v1's 50 ms
        source_frame = pc2_msg.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                "map",
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=TF_TIMEOUT_S),
            )
        except Exception as e:
            self._last_reject[color] = f"tf_fail:{e}"
            return

        pts_to_xf = np.array(
            [
                mean,
                mean - half_len * direction,
                mean + half_len * direction,
            ]
        )
        pts_map = self._apply_tf(pts_to_xf, tf)
        center_map, p1_map, p2_map = pts_map

        # Map-frame length filter: barrels project ~0.2 m, belts ≥ 1.5 m
        belt_length = float(np.linalg.norm(p2_map - p1_map))
        if belt_length < MIN_BELT_LENGTH_M:
            self._last_reject[color] = f"len={belt_length:.2f}m<{MIN_BELT_LENGTH_M}"
            self.get_logger().debug(
                f"[{color}] rejected: belt_length={belt_length:.2f} m < {MIN_BELT_LENGTH_M}"
            )
            return

        cx, cy = float(center_map[0]), float(center_map[1])
        self._candidates[color].append((cx, cy))
        self._last_reject[color] = f"ok len={belt_length:.2f}m"
        n = len(self._candidates[color])

        self.get_logger().debug(
            f"[{color}] candidate {n}/{CONFIRM_COUNT}  "
            f"center=({cx:.2f}, {cy:.2f})  "
            f"len={belt_length:.2f} m  flat_std={stds[0]:.3f}"
        )

        if n >= CONFIRM_COUNT:
            arr = np.array(self._candidates[color])
            median_center = np.median(arr, axis=0)
            self._lock(color, median_center)

    # ── Lock & publish ─────────────────────────────────────────────────────────

    def _lock(self, color: str, center: np.ndarray) -> None:
        self._locked[color] = True
        m = self._make_marker(color, center)
        self._locked_markers[color] = m
        self._pub.publish(m)
        self.get_logger().info(
            f"Workstation '{color}' locked at "
            f"({center[0]:.2f}, {center[1]:.2f}) after {CONFIRM_COUNT} sightings"
        )

    def _make_marker(self, color: str, center: np.ndarray) -> Marker:
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = color
        m.id = _MARKER_IDS[color]
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(center[0])
        m.pose.position.y = float(center[1])
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        r, g, b = _MARKER_COLORS[color]
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        return m

    def _republish_locked(self) -> None:
        for marker in self._locked_markers.values():
            marker.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(marker)

    # ── TF helper ──────────────────────────────────────────────────────────────

    @staticmethod
    def _apply_tf(pts: np.ndarray, tf) -> np.ndarray:
        """Batch-transform (N, 3) points using a TransformStamped."""
        R = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = tf.transform.translation
        return (R @ pts.T).T + np.array([t.x, t.y, t.z])


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationDetector2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
