"""Ring detector node for Megatron Task 1.

Detects colored paper rings using adaptive threshold → contour + ellipse
fitting → concentric pair matching → HSV color classification → hole check.
Projects detections into 3-D via a 32FC1 depth image, computes SVD surface
normals, and tracks with inverse-distance weighting.

Publishes confirmed ring poses on ``/detected_rings`` (color encoded in
``header.frame_id`` as ``"map|<color>"``) and RViz markers on
``/ring_markers``.  Also publishes 4 debug stage images for the perception
visualizer.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

import message_filters

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf2_ros

from megatron.perception_utils import (
    DepthCameraGeometry,
    IncrementalTrackManager,
    compute_robust_surface,
    transform_point,
    transform_vector,
    normal_to_yaw,
    yaw_to_quaternion,
)

# ---------------------------------------------------------------------------
# HSV color ranges for ring classification
# ---------------------------------------------------------------------------

COLOR_RANGES = {
    'red':    {'lower1': (0, 100, 100),   'upper1': (10, 255, 255),
               'lower2': (160, 100, 100), 'upper2': (180, 255, 255),
               'rgb': (1.0, 0.0, 0.0)},
    'green':  {'lower1': (35, 80, 80),    'upper1': (85, 255, 255),
               'rgb': (0.0, 1.0, 0.0)},
    'blue':   {'lower1': (90, 80, 80),    'upper1': (130, 255, 255),
               'rgb': (0.0, 0.0, 1.0)},
    'yellow': {'lower1': (20, 100, 100),  'upper1': (35, 255, 255),
               'rgb': (1.0, 1.0, 0.0)},
    'black':  {'lower1': (0, 0, 0),       'upper1': (180, 255, 50),
               'rgb': (0.2, 0.2, 0.2)},
}


# ---------------------------------------------------------------------------
# 2-D detection helpers (kept from old implementation, cleaned up)
# ---------------------------------------------------------------------------

def _build_annular_mask(shape: tuple,
                        outer_ellipse: tuple,
                        inner_ellipse: tuple) -> np.ndarray:
    """Return uint8 mask (255 on ring band, 0 elsewhere)."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.ellipse(mask, outer_ellipse, (255,), -1)
    cv2.ellipse(mask, inner_ellipse, (0,), -1)
    return mask


def classify_ring_color(
    image_bgr: np.ndarray,
    outer_ellipse: tuple,
    inner_ellipse: tuple,
    color_ranges: dict = COLOR_RANGES,
    min_pixels: int = 10,
) -> tuple[Optional[str], float]:
    """HSV histogram voting on the annular band.

    Returns ``(color_name, confidence)`` or ``(None, 0.0)``.
    """
    mask = _build_annular_mask(image_bgr.shape, outer_ellipse, inner_ellipse)
    total = int(cv2.countNonZero(mask))
    if total < min_pixels:
        return None, 0.0

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    best_name: Optional[str] = None
    best_count = 0

    for name, ranges in color_ranges.items():
        cmask = cv2.inRange(hsv, np.array(ranges['lower1']), np.array(ranges['upper1']))
        if 'lower2' in ranges:
            cmask2 = cv2.inRange(hsv, np.array(ranges['lower2']), np.array(ranges['upper2']))
            cmask = cv2.bitwise_or(cmask, cmask2)
        hit = cv2.bitwise_and(cmask, mask)
        count = int(cv2.countNonZero(hit))
        if count > best_count:
            best_count = count
            best_name = name

    confidence = best_count / total if total > 0 else 0.0
    return best_name, confidence


def _check_hole(image_gray: np.ndarray,
                outer_ellipse: tuple,
                inner_ellipse: tuple,
                min_brightness_diff: float = 20.0) -> bool:
    """Return True if the inner region differs from the band (= real hole)."""
    h, w = image_gray.shape[:2]
    band_mask = _build_annular_mask((h, w), outer_ellipse, inner_ellipse)
    inner_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.ellipse(inner_mask, inner_ellipse, (255,), -1)

    if cv2.countNonZero(band_mask) == 0 or cv2.countNonZero(inner_mask) == 0:
        return False

    band_mean = cv2.mean(image_gray, mask=band_mask)[0]
    inner_mean = cv2.mean(image_gray, mask=inner_mask)[0]
    return abs(band_mean - inner_mean) >= min_brightness_diff


def _ellipse_area(e: tuple) -> float:
    return math.pi * (e[1][0] / 2.0) * (e[1][1] / 2.0)


def _ellipse_score(contour: np.ndarray, ellipse: tuple) -> float:
    """Contour-area / ellipse-area ratio, clamped to [0, 1]."""
    ca = cv2.contourArea(contour)
    ea = _ellipse_area(ellipse)
    return min(ca / ea, 1.0) if ea >= 1.0 else 0.0


def _score_to_bgr(score: float) -> tuple[int, int, int]:
    g = int(255 * score)
    r = int(255 * (1.0 - score))
    return (0, g, r)


def _ellipse_contains(larger: tuple, smaller: tuple) -> bool:
    return larger[1][0] >= smaller[1][0] and larger[1][1] >= smaller[1][1]


def _pair_score(e1: tuple, e2: tuple) -> float:
    """Concentric ellipse pair score in [0, 1]."""
    dist = math.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1])
    le, se = (e1, e2) if e1[1][0] * e1[1][1] >= e2[1][0] * e2[1][1] else (e2, e1)

    max_axis = max(le[1][0], le[1][1], 1.0)
    center_score = max(0.0, 1.0 - dist / (max_axis * 0.5))

    ratio = (se[1][0] * se[1][1]) / max(le[1][0] * le[1][1], 1.0)
    ratio_score = (1.0 - abs(ratio - 0.7) / 0.5) if 0.2 <= ratio <= 0.98 else 0.0

    ecc1 = max(le[1]) / max(min(le[1]), 1.0)
    ecc2 = max(se[1]) / max(min(se[1]), 1.0)
    ecc_score = max(0.0, 1.0 - abs(ecc1 - ecc2) / 2.0)

    return 0.5 * center_score + 0.3 * ratio_score + 0.2 * ecc_score


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class RingDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('ring_detector')

        # ---- Parameters ----
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('min_inference_interval', 0.2)
        # Adaptive threshold
        self.declare_parameter('thresh_block_size', 15)
        self.declare_parameter('thresh_c', 25)
        # Ellipse filtering
        self.declare_parameter('min_contour_points', 20)
        self.declare_parameter('max_axis', 200.0)
        self.declare_parameter('min_axis', 6.0)
        self.declare_parameter('max_aspect_ratio', 1.8)
        # Pair matching
        self.declare_parameter('center_thr', 15.0)
        self.declare_parameter('min_pair_score', 0.30)
        # Color
        self.declare_parameter('min_color_confidence', 0.15)
        # Hole
        self.declare_parameter('min_brightness_diff', 20.0)

        self.confirmation_count = self.get_parameter('confirmation_count').get_parameter_value().integer_value
        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value
        self.min_inference_interval = self.get_parameter('min_inference_interval').get_parameter_value().double_value
        self.thresh_block_size = self.get_parameter('thresh_block_size').get_parameter_value().integer_value
        self.thresh_c = self.get_parameter('thresh_c').get_parameter_value().integer_value
        self.min_contour_points = self.get_parameter('min_contour_points').get_parameter_value().integer_value
        self.max_axis = self.get_parameter('max_axis').get_parameter_value().double_value
        self.min_axis = self.get_parameter('min_axis').get_parameter_value().double_value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').get_parameter_value().double_value
        self.center_thr = self.get_parameter('center_thr').get_parameter_value().double_value
        self.min_pair_score = self.get_parameter('min_pair_score').get_parameter_value().double_value
        self.min_color_confidence = self.get_parameter('min_color_confidence').get_parameter_value().double_value
        self.min_brightness_diff = self.get_parameter('min_brightness_diff').get_parameter_value().double_value

        # Enforce odd block size ≥ 3
        if self.thresh_block_size % 2 == 0:
            self.thresh_block_size += 1
        if self.thresh_block_size < 3:
            self.thresh_block_size = 3

        self.bridge = CvBridge()

        # Depth camera geometry
        self.cam_geom = DepthCameraGeometry()
        self.cam_geom.initialise(self)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tracking
        self.tracker = IncrementalTrackManager(
            dedup_distance=self.dedup_distance,
            confirmation_count=self.confirmation_count,
        )

        # Synchronized subscribers
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/depth',
            qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Publishers — detections
        self.ring_pub = self.create_publisher(PoseStamped, '/detected_rings', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/ring_markers',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.image_pub = self.create_publisher(Image, '/ring_detections_image', 10)

        # Publishers — 4 debug stages
        self.debug_binary_pub = self.create_publisher(Image, '/ring_debug/binary', 10)
        self.debug_ellipses_pub = self.create_publisher(Image, '/ring_debug/ellipses', 10)
        self.debug_pairs_pub = self.create_publisher(Image, '/ring_debug/pairs', 10)
        self.debug_color_pub = self.create_publisher(Image, '/ring_debug/color', 10)

        # Rate limiting
        self.last_inference_time = self.get_clock().now()

        self.get_logger().info('Ring detector initialized (ellipse-pair mode).')

    # ------------------------------------------------------------------
    # Synchronized callback
    # ------------------------------------------------------------------

    def sync_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        now = self.get_clock().now()
        if (now - self.last_inference_time).nanoseconds < self.min_inference_interval * 1e9:
            return
        self.last_inference_time = now

        if not self.cam_geom.ready:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        h, w = cv_image.shape[:2]

        # ---- Stage 1: adaptive threshold → binary ----
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            self.thresh_block_size, self.thresh_c)
        self._publish_debug(self.debug_binary_pub, thresh, mono=True)

        # ---- Stage 2: contour extraction → ellipse fitting ----
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        ellipses: list[tuple[tuple, np.ndarray, float]] = []
        debug_ell_img = cv_image.copy()

        for cnt in contours:
            if len(cnt) < self.min_contour_points:
                continue
            ellipse = cv2.fitEllipse(cnt)
            (ex, ey), (ew, eh), _angle = ellipse

            if ew < self.min_axis or eh < self.min_axis:
                continue
            if ew > self.max_axis or eh > self.max_axis:
                continue
            ratio = max(ew, eh) / max(min(ew, eh), 1.0)
            if ratio > self.max_aspect_ratio:
                continue
            if ex < 0 or ey < 0 or ex >= w or ey >= h:
                continue

            score = _ellipse_score(cnt, ellipse)
            ellipses.append((ellipse, cnt, score))

            color_bgr = _score_to_bgr(score)
            cv2.ellipse(debug_ell_img, ellipse, color_bgr, 1)
            cv2.circle(debug_ell_img, (int(ex), int(ey)), 2, color_bgr, -1)
            cv2.putText(debug_ell_img, f'{score:.2f}',
                        (int(ex) + 5, int(ey) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color_bgr, 1)

        self._publish_debug(self.debug_ellipses_pub, debug_ell_img)

        # ---- Stage 3: concentric pair matching ----
        debug_pairs_img = cv_image.copy()
        ring_candidates: list[tuple] = []
        used: set[int] = set()

        for i in range(len(ellipses)):
            if i in used:
                continue
            e1, _cnt1, _s1 = ellipses[i]
            for j in range(i + 1, len(ellipses)):
                if j in used:
                    continue
                e2, _cnt2, _s2 = ellipses[j]

                dist = math.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1])
                if dist > self.center_thr:
                    continue

                outer, inner = (e1, e2) if e1[1][0] * e1[1][1] >= e2[1][0] * e2[1][1] else (e2, e1)
                if not _ellipse_contains(outer, inner):
                    continue

                ps = _pair_score(e1, e2)
                if ps >= self.min_pair_score:
                    cx_px = int((e1[0][0] + e2[0][0]) / 2)
                    cy_px = int((e1[0][1] + e2[0][1]) / 2)
                    ring_candidates.append((outer, inner, ps, (cx_px, cy_px)))
                    used.add(i)
                    used.add(j)

                    cv2.ellipse(debug_pairs_img, outer, (0, 255, 0), 2)
                    cv2.ellipse(debug_pairs_img, inner, (0, 255, 0), 2)
                    cv2.circle(debug_pairs_img, (cx_px, cy_px), 5, (0, 255, 0), -1)
                    cv2.putText(debug_pairs_img, f'ps={ps:.2f}',
                                (cx_px + 8, cy_px - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    break
                else:
                    cv2.ellipse(debug_pairs_img, outer, (0, 0, 120), 1)
                    cv2.ellipse(debug_pairs_img, inner, (0, 0, 120), 1)

        self._publish_debug(self.debug_pairs_pub, debug_pairs_img)

        # ---- Stage 4: color + hole check + 3-D localization ----
        debug_color_img = cv_image.copy()
        output_img = cv_image.copy()

        # TF lookup
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', depth_msg.header.frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            transform = None

        for outer, inner, ps, (cx_px, cy_px) in ring_candidates:
            # Hole check
            has_hole = _check_hole(gray, outer, inner, self.min_brightness_diff)
            if not has_hole:
                cv2.ellipse(debug_color_img, outer, (0, 0, 180), 1)
                cv2.putText(debug_color_img, 'solid',
                            (cx_px + 8, cy_px),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 180), 1)
                continue

            # Color classification
            color_name, confidence = classify_ring_color(
                cv_image, outer, inner,
                min_pixels=10)

            if color_name is None or confidence < self.min_color_confidence:
                cv2.ellipse(debug_color_img, outer, (128, 128, 128), 1)
                cv2.putText(debug_color_img, f'? ({confidence:.2f})',
                            (cx_px + 8, cy_px),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                continue

            # Debug visualization — colored overlay
            rgb = COLOR_RANGES.get(color_name, {'rgb': (1.0, 1.0, 1.0)})['rgb']
            fill_color = (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255))
            band_mask = _build_annular_mask(cv_image.shape, outer, inner)
            overlay = debug_color_img.copy()
            overlay[band_mask > 0] = fill_color
            cv2.addWeighted(overlay, 0.4, debug_color_img, 0.6, 0, debug_color_img)
            cv2.ellipse(debug_color_img, outer, fill_color, 2)
            cv2.ellipse(debug_color_img, inner, fill_color, 2)
            cv2.circle(debug_color_img, (cx_px, cy_px), 5, fill_color, -1)
            cv2.putText(debug_color_img, f'{color_name} ({confidence:.2f})',
                        (cx_px + 8, cy_px),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, fill_color, 2)

            # Output image annotation
            cv2.ellipse(output_img, outer, (0, 255, 0), 2)
            cv2.ellipse(output_img, inner, (0, 255, 0), 2)
            cv2.circle(output_img, (cx_px, cy_px), 4, (0, 255, 0), -1)
            cv2.putText(output_img, color_name, (cx_px + 10, cy_px),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # ---- 3-D localization via depth image ----
            if transform is None:
                continue

            # Use the full annular mask for depth sampling (not edge points!)
            pts_3d = self.cam_geom.extract_3d_points(band_mask, depth_image)
            if pts_3d.shape[0] < 10:
                continue

            result = compute_robust_surface(pts_3d)
            if result is None:
                continue
            centroid_cam, normal_cam = result

            if centroid_cam[2] < 0.3 or centroid_cam[2] > 5.0:
                continue

            cam_dist = float(centroid_cam[2])

            map_pos = transform_point(centroid_cam, transform)
            map_normal = transform_vector(normal_cam, transform)

            self.get_logger().debug(
                f'Ring 3D ({color_name}): cam_pos={centroid_cam} cam_normal={normal_cam} '
                f'→ map_pos={map_pos} map_normal={map_normal}')

            track_id, newly_confirmed = self.tracker.add_observation(
                map_pos, map_normal, cam_dist,
                rgb_msg.header.stamp, label=color_name)

            if newly_confirmed:
                est = self.tracker.get_best_estimate(track_id)
                self._publish_ring(est, rgb_msg.header.stamp)
                self.tracker.mark_reported(track_id)

        # Publish any unreported confirmed tracks
        for tid, est in self.tracker.get_confirmed_unreported():
            self._publish_ring(est, rgb_msg.header.stamp)
            self.tracker.mark_reported(tid)

        self._publish_debug(self.debug_color_pub, debug_color_img)
        self._publish_debug(self.image_pub, output_img)
        self._publish_markers()

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_ring(self, estimate: dict, stamp) -> None:
        ring_id = sum(1 for t in self.tracker.tracks if t.reported or t.confirmed)
        pos = estimate['map_pos']
        normal = estimate['normal']
        color_name = estimate['label']

        self.get_logger().info(
            f'Ring #{ring_id} ({color_name}) confirmed at '
            f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        pose = PoseStamped()
        # frame_id hack: "map|<color>" to atomically deliver color with the pose
        pose.header.frame_id = f'map|{color_name}'
        pose.header.stamp = stamp
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])

        yaw = normal_to_yaw(normal[:2])
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.ring_pub.publish(pose)

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        markers: list[Marker] = []
        stamp = self.get_clock().now().to_msg()

        idx = 0
        for hyp in self.tracker.tracks:
            if not hyp.confirmed or hyp.rejected:
                continue
            est = hyp.weighted_estimate()
            pos = est['map_pos']
            color_name = est['label']
            rgb = COLOR_RANGES.get(color_name, {'rgb': (1.0, 1.0, 1.0)})['rgb']

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'rings'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = float(rgb[0])
            m.color.g = float(rgb[1])
            m.color.b = float(rgb[2])
            m.color.a = 1.0
            markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = stamp
            t.ns = 'ring_labels'
            t.id = idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pos[0])
            t.pose.position.y = float(pos[1])
            t.pose.position.z = float(pos[2]) + 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = float(rgb[0])
            t.color.g = float(rgb[1])
            t.color.b = float(rgb[2])
            t.color.a = 1.0
            t.text = f'{color_name} ring'
            markers.append(t)

            idx += 1

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    def _publish_debug(self, publisher, image: np.ndarray, mono: bool = False) -> None:
        try:
            if mono:
                msg = self.bridge.cv2_to_imgmsg(image, 'mono8')
            else:
                msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            publisher.publish(msg)
        except CvBridgeError:
            pass


def main(args=None):
    print('Ring detection node starting.')
    rclpy.init(args=args)
    node = RingDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
