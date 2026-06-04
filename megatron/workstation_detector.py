#!/usr/bin/env python3
"""
Workstation detector.

Subscribes to top-camera RGB + organized depth point cloud. HSV-masks red/green
elongated blobs (aspect ≥ 3.0 → conveyor belt shape), projects visible pixels to
map frame, and publishes confirmed workstation centroids as Marker on
/detected_workstations (ns = "red" | "green").

Confirmation uses IncrementalTrackManager (dedup_distance=0.5 m, 5 votes).
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
    IncrementalTrackManager,
    extract_3d_points_from_pc2,
    quaternion_to_rotation_matrix,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# HSV thresholds — workstation belts
RED_LO1 = np.array([0, 80, 60], dtype=np.uint8)
RED_HI1 = np.array([10, 255, 255], dtype=np.uint8)
RED_LO2 = np.array([170, 80, 60], dtype=np.uint8)
RED_HI2 = np.array([180, 255, 255], dtype=np.uint8)
GREEN_LO = np.array([40, 80, 60], dtype=np.uint8)
GREEN_HI = np.array([80, 255, 255], dtype=np.uint8)
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

MIN_CONTOUR_AREA = 2000  # px² — reject small blobs
MIN_ASPECT_RATIO = 3.0  # long belt shape; rings and faces have aspect ~1

_INACTIVE_STATES = frozenset(("INSPECT_WORKSTATION", "FOLLOW_BLUE_LINE", "DONE"))

_MARKER_COLORS = {
    "red": (1.0, 0.0, 0.0),
    "green": (0.0, 1.0, 0.0),
}
_MARKER_IDS = {"red": 0, "green": 1}


class WorkstationDetector(Node):
    def __init__(self):
        super().__init__("workstation_detector")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

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

        self.detected_pub = self.create_publisher(Marker, "/detected_workstations", 10)

        self._latest_pc2: PointCloud2 | None = None
        self.robot_state: str = "INIT"

        # One ITM per color — dedup_distance=0.5 m, 5 votes to confirm
        self._trackers: dict[str, IncrementalTrackManager] = {
            color: IncrementalTrackManager(dedup_distance=0.5, confirmation_count=5)
            for color in ("red", "green")
        }

        self.create_timer(3.0, self._republish_confirmed)
        self.get_logger().info("WorkstationDetector ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        self.robot_state = msg.data

    def _pc2_cb(self, msg: PointCloud2):
        self._latest_pc2 = msg

    def _image_cb(self, msg: Image):
        if self.robot_state in _INACTIVE_STATES:
            return
        if self._latest_pc2 is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = img.shape[:2]
        pc2_snap = self._latest_pc2  # local snapshot to avoid race

        for color, mask in self._compute_masks(hsv):
            self._process_mask(color, mask, (h, w), pc2_snap)

    # ── Detection pipeline ────────────────────────────────────────────────────

    def _compute_masks(self, hsv):
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LO1, RED_HI1),
            cv2.inRange(hsv, RED_LO2, RED_HI2),
        )
        red_mask = cv2.morphologyEx(
            cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, MORPH_K),
            cv2.MORPH_OPEN,
            MORPH_K,
        )
        green_mask = cv2.inRange(hsv, GREEN_LO, GREEN_HI)
        green_mask = cv2.morphologyEx(
            cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, MORPH_K),
            cv2.MORPH_OPEN,
            MORPH_K,
        )
        return [("red", red_mask), ("green", green_mask)]

    def _process_mask(
        self, color: str, mask: np.ndarray, img_shape, pc2_msg: PointCloud2
    ):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                continue
            # minAreaRect gives the true oriented dimensions; boundingRect would
            # give a near-square box for a rotated belt and fail the aspect filter.
            _, (rw, rh), _ = cv2.minAreaRect(cnt)
            if rw < 1 or rh < 1 or max(rw, rh) / min(rw, rh) < MIN_ASPECT_RATIO:
                continue

            # Per-contour mask for extract_3d_points_from_pc2
            cnt_mask = np.zeros(img_shape, dtype=np.uint8)
            cv2.drawContours(cnt_mask, [cnt], -1, (255,), cv2.FILLED)

            pts_cam = extract_3d_points_from_pc2(cnt_mask, pc2_msg, max_range=4.0)
            if len(pts_cam) < 5:
                continue

            # TF camera → map
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    pc2_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                continue

            pts_map = self._apply_tf(pts_cam, tf)
            centroid = pts_map.mean(axis=0)  # 3D centroid in map frame
            cam_dist = float(np.linalg.norm(pts_cam.mean(axis=0)))

            status, track = self._trackers[color].add_observation(
                centroid,
                np.array([0.0, 0.0, 1.0]),  # dummy normal
                cam_dist,
                self.get_clock().now().to_msg(),
            )

            if status in ("confirmed", "updated"):
                pos, _ = self._trackers[color].get_best_estimate(track)
                self._publish_marker(color, pos)
                if status == "confirmed":
                    self.get_logger().info(
                        f"Workstation '{color}' confirmed at ({pos[0]:.2f}, {pos[1]:.2f})"
                    )

    # ── TF helper ─────────────────────────────────────────────────────────────

    @staticmethod
    def _apply_tf(pts_cam: np.ndarray, tf) -> np.ndarray:
        """Batch-transform (N,3) points from camera frame to map frame."""
        R = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = tf.transform.translation
        T = np.array([t.x, t.y, t.z])
        return (R @ pts_cam.T).T + T

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish_marker(self, color: str, pos: np.ndarray):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = color
        m.id = _MARKER_IDS[color]
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        r, g, b = _MARKER_COLORS[color]
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        self.detected_pub.publish(m)

    def _republish_confirmed(self):
        for color, tracker in self._trackers.items():
            confirmed = tracker.get_confirmed_tracks()
            if confirmed:
                pos, _ = tracker.get_best_estimate(confirmed[-1])
                self._publish_marker(color, pos)


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
