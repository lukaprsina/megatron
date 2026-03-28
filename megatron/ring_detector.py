import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros

# HSV ranges for ring colors — used only for color classification, NOT for geometry
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
# Color classification — abstracted so the strategy can be swapped later
# ---------------------------------------------------------------------------

def _build_annular_mask(shape: tuple,
                        outer_ellipse: tuple, inner_ellipse: tuple) -> np.ndarray:
    """Return a uint8 mask (255 inside the ring band, 0 elsewhere)."""
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
    """Classify the ring color by HSV histogram voting on the annular band.

    Returns (color_name, confidence) where confidence is the fraction of
    non-zero ring-band pixels that matched the winning color range.
    Returns (None, 0.0) if too few pixels in the band.
    """
    mask = _build_annular_mask(image_bgr.shape, outer_ellipse, inner_ellipse)
    total_pixels = int(cv2.countNonZero(mask))
    if total_pixels < min_pixels:
        return None, 0.0

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    best_name: Optional[str] = None
    best_count = 0

    for name, ranges in color_ranges.items():
        color_mask = cv2.inRange(hsv, np.array(ranges['lower1']), np.array(ranges['upper1']))
        if 'lower2' in ranges:
            color_mask2 = cv2.inRange(hsv, np.array(ranges['lower2']), np.array(ranges['upper2']))
            color_mask = cv2.bitwise_or(color_mask, color_mask2)
        # Intersection: pixels that are both in the ring band AND match this color
        hit = cv2.bitwise_and(color_mask, mask)
        count = int(cv2.countNonZero(hit))
        if count > best_count:
            best_count = count
            best_name = name

    confidence = best_count / total_pixels if total_pixels > 0 else 0.0
    return best_name, confidence


# ---------------------------------------------------------------------------
# Ellipse quality scoring
# ---------------------------------------------------------------------------

def _ellipse_area(e: tuple) -> float:
    """Area of an ellipse from the fitEllipse tuple ((cx,cy),(w,h),angle)."""
    return math.pi * (e[1][0] / 2.0) * (e[1][1] / 2.0)


def _ellipse_score(contour: np.ndarray, ellipse: tuple) -> float:
    """Score in [0, 1]: how well a contour matches its fitted ellipse.

    Ratio of contour area to the theoretical ellipse area. Values close to 1
    mean the contour is almost a perfect ellipse.
    """
    ca = cv2.contourArea(contour)
    ea = _ellipse_area(ellipse)
    if ea < 1.0:
        return 0.0
    return min(ca / ea, 1.0)


def _score_to_bgr(score: float) -> tuple[int, int, int]:
    """Map a score in [0, 1] to a BGR color gradient: red (0) -> green (1)."""
    g = int(255 * score)
    r = int(255 * (1.0 - score))
    return (0, g, r)


# ---------------------------------------------------------------------------
# Ellipse-pair ring candidate detection
# ---------------------------------------------------------------------------

def _ellipse_contains(larger: tuple, smaller: tuple) -> bool:
    """Check that the larger ellipse's axes are both >= the smaller's."""
    return (larger[1][0] >= smaller[1][0] and larger[1][1] >= smaller[1][1])


def _pair_score(e1: tuple, e2: tuple) -> float:
    """Score a concentric ellipse pair — higher is more ring-like.

    Components:
      - center alignment (distance between centers, normalized)
      - axis ratio consistency (inner/outer ratio plausibility)
      - eccentricity similarity (how close their aspect ratios are)
    Returns value in [0, 1].
    """
    dist = math.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1])
    # Identify larger/smaller
    if e1[1][0] * e1[1][1] >= e2[1][0] * e2[1][1]:
        le, se = e1, e2
    else:
        le, se = e2, e1

    # Center alignment: perfect = 0 distance
    max_axis = max(le[1][0], le[1][1], 1.0)
    center_score = max(0.0, 1.0 - dist / (max_axis * 0.5))

    # Size ratio: inner should be smaller but not tiny (0.2 - 0.98 is plausible)
    ratio = (se[1][0] * se[1][1]) / max(le[1][0] * le[1][1], 1.0)
    if 0.2 <= ratio <= 0.98:
        ratio_score = 1.0 - abs(ratio - 0.7) / 0.5  # peak at 0.7
    else:
        ratio_score = 0.0

    # Eccentricity similarity
    ecc1 = max(le[1]) / max(min(le[1]), 1.0)
    ecc2 = max(se[1]) / max(min(se[1]), 1.0)
    ecc_score = max(0.0, 1.0 - abs(ecc1 - ecc2) / 2.0)

    return 0.5 * center_score + 0.3 * ratio_score + 0.2 * ecc_score


# ---------------------------------------------------------------------------
# Depth sampling helpers
# ---------------------------------------------------------------------------

def _sample_ellipse_points(ellipse: tuple, n: int = 12) -> list[tuple[int, int]]:
    """Return n pixel coordinates evenly spaced around an ellipse boundary."""
    (cx, cy), (w, h), angle = ellipse
    a, b = w / 2.0, h / 2.0
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    pts = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        x = a * math.cos(theta)
        y = b * math.sin(theta)
        px = int(cx + x * cos_a - y * sin_a)
        py = int(cy + x * sin_a + y * cos_a)
        pts.append((px, py))
    return pts


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class RingDetectorNode(Node):

    def __init__(self):
        super().__init__('ring_detector')

        # ---- Parameters ----
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)
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
        # Color classification
        self.declare_parameter('min_color_confidence', 0.15)
        # Debug
        self.declare_parameter('show_debug_window', False)

        self.confirmation_count = self.get_parameter('confirmation_count').get_parameter_value().integer_value
        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value
        self.thresh_block_size = self.get_parameter('thresh_block_size').get_parameter_value().integer_value
        self.thresh_c = self.get_parameter('thresh_c').get_parameter_value().integer_value
        self.min_contour_points = self.get_parameter('min_contour_points').get_parameter_value().integer_value
        self.max_axis = self.get_parameter('max_axis').get_parameter_value().double_value
        self.min_axis = self.get_parameter('min_axis').get_parameter_value().double_value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').get_parameter_value().double_value
        self.center_thr = self.get_parameter('center_thr').get_parameter_value().double_value
        self.min_pair_score = self.get_parameter('min_pair_score').get_parameter_value().double_value
        self.min_color_confidence = self.get_parameter('min_color_confidence').get_parameter_value().double_value
        self.show_debug_window = self.get_parameter('show_debug_window').get_parameter_value().bool_value

        # Enforce odd block size >= 3
        if self.thresh_block_size % 2 == 0:
            self.thresh_block_size += 1
        if self.thresh_block_size < 3:
            self.thresh_block_size = 3

        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.rgb_image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        # Publishers — detections (same API as before)
        self.ring_pub = self.create_publisher(PoseStamped, '/detected_rings', 10)
        self.color_pub = self.create_publisher(String, '/detected_ring_color', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/ring_markers', QoSReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, '/ring_detections_image', 10)

        # Publishers — debug (4 stages)
        self.debug_binary_pub = self.create_publisher(Image, '/ring_debug/binary', 10)
        self.debug_ellipses_pub = self.create_publisher(Image, '/ring_debug/ellipses', 10)
        self.debug_pairs_pub = self.create_publisher(Image, '/ring_debug/pairs', 10)
        self.debug_color_pub = self.create_publisher(Image, '/ring_debug/color', 10)

        # State
        # Each detection: (outer_ellipse, inner_ellipse, center_xy, color_name)
        self.detections_px: list[tuple] = []
        self.candidates: list[dict] = []   # [{'pos': np.array, 'color': str, 'count': int}]
        self.confirmed: list[dict] = []    # [{'pos': np.array, 'color': str}]

        if self.show_debug_window:
            cv2.namedWindow('Ring Debug', cv2.WINDOW_NORMAL)

        self.get_logger().info('Ring detector initialized (ellipse-pair mode).')

    # ------------------------------------------------------------------
    # RGB callback — the main detection pipeline
    # ------------------------------------------------------------------

    def rgb_callback(self, data: Image) -> None:
        self.detections_px = []

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        h, w = cv_image.shape[:2]

        # ---- Stage 1: adaptive threshold -> binary ----
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            self.thresh_block_size, self.thresh_c,
        )
        # Publish debug 1 — binary
        self._publish_debug(self.debug_binary_pub, thresh, mono=True)

        # ---- Stage 2: contour extraction -> ellipse fitting & scoring ----
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        ellipses: list[tuple[tuple, np.ndarray, float]] = []  # (ellipse, contour, score)
        debug_ellipses_img = cv_image.copy()

        for cnt in contours:
            if len(cnt) < self.min_contour_points:
                continue
            ellipse = cv2.fitEllipse(cnt)
            (ex, ey), (ew, eh), angle = ellipse

            # Filter by axis size
            if ew < self.min_axis or eh < self.min_axis:
                continue
            if ew > self.max_axis or eh > self.max_axis:
                continue
            # Filter by aspect ratio
            ratio = max(ew, eh) / max(min(ew, eh), 1.0)
            if ratio > self.max_aspect_ratio:
                continue
            # Filter out-of-bounds centers
            if ex < 0 or ey < 0 or ex >= w or ey >= h:
                continue

            score = _ellipse_score(cnt, ellipse)
            ellipses.append((ellipse, cnt, score))

            # Draw on debug image — color by score
            color_bgr = _score_to_bgr(score)
            cv2.ellipse(debug_ellipses_img, ellipse, color_bgr, 1)
            cv2.circle(debug_ellipses_img, (int(ex), int(ey)), 2, color_bgr, -1)
            label = f'{score:.2f}'
            cv2.putText(debug_ellipses_img, label,
                        (int(ex) + 5, int(ey) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color_bgr, 1)

        # Publish debug 2 — all candidate ellipses
        self._publish_debug(self.debug_ellipses_pub, debug_ellipses_img)

        # ---- Stage 3: concentric pair matching ----
        debug_pairs_img = cv_image.copy()
        ring_candidates: list[tuple[tuple, tuple, float, tuple[int, int]]] = []
        # (outer_ellipse, inner_ellipse, pair_score, center_px)

        used = set()  # track indices already matched
        for i in range(len(ellipses)):
            if i in used:
                continue
            e1, cnt1, s1 = ellipses[i]
            for j in range(i + 1, len(ellipses)):
                if j in used:
                    continue
                e2, cnt2, s2 = ellipses[j]

                dist = math.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1])
                if dist > self.center_thr:
                    continue

                # Determine outer / inner
                if e1[1][0] * e1[1][1] >= e2[1][0] * e2[1][1]:
                    outer, inner = e1, e2
                else:
                    outer, inner = e2, e1

                if not _ellipse_contains(outer, inner):
                    continue

                ps = _pair_score(e1, e2)

                if ps >= self.min_pair_score:
                    cx_px = int((e1[0][0] + e2[0][0]) / 2)
                    cy_px = int((e1[0][1] + e2[0][1]) / 2)
                    ring_candidates.append((outer, inner, ps, (cx_px, cy_px)))
                    used.add(i)
                    used.add(j)

                    # Draw accepted pair
                    cv2.ellipse(debug_pairs_img, outer, (0, 255, 0), 2)
                    cv2.ellipse(debug_pairs_img, inner, (0, 255, 0), 2)
                    cv2.circle(debug_pairs_img, (cx_px, cy_px), 5, (0, 255, 0), -1)
                    cv2.putText(debug_pairs_img, f'ps={ps:.2f}',
                                (cx_px + 8, cy_px - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    break  # one match per ellipse
                else:
                    # Draw rejected pair faded
                    cv2.ellipse(debug_pairs_img, outer, (0, 0, 120), 1)
                    cv2.ellipse(debug_pairs_img, inner, (0, 0, 120), 1)

        # Publish debug 3 — pairs
        self._publish_debug(self.debug_pairs_pub, debug_pairs_img)

        # ---- Stage 4: color classification ----
        debug_color_img = cv_image.copy()
        output_img = cv_image.copy()  # for /ring_detections_image

        for outer, inner, ps, (cx_px, cy_px) in ring_candidates:
            color_name, confidence = classify_ring_color(cv_image, outer, inner)

            if color_name is None or confidence < self.min_color_confidence:
                # Draw grayed out on debug
                cv2.ellipse(debug_color_img, outer, (128, 128, 128), 1)
                cv2.ellipse(debug_color_img, inner, (128, 128, 128), 1)
                cv2.putText(debug_color_img, f'? ({confidence:.2f})',
                            (cx_px + 8, cy_px),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                continue

            # Draw colored overlay on debug image
            overlay = debug_color_img.copy()
            band_mask = _build_annular_mask(cv_image.shape, outer, inner)
            rgb = COLOR_RANGES.get(color_name, {'rgb': (1.0, 1.0, 1.0)})['rgb']
            fill_color = (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255))
            overlay[band_mask > 0] = fill_color
            cv2.addWeighted(overlay, 0.4, debug_color_img, 0.6, 0, debug_color_img)
            cv2.ellipse(debug_color_img, outer, fill_color, 2)
            cv2.ellipse(debug_color_img, inner, fill_color, 2)
            cv2.circle(debug_color_img, (cx_px, cy_px), 5, fill_color, -1)
            cv2.putText(debug_color_img, f'{color_name} ({confidence:.2f})',
                        (cx_px + 8, cy_px),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, fill_color, 2)

            # Draw on main output image
            cv2.ellipse(output_img, outer, (0, 255, 0), 2)
            cv2.ellipse(output_img, inner, (0, 255, 0), 2)
            cv2.circle(output_img, (cx_px, cy_px), 4, (0, 255, 0), -1)
            cv2.putText(output_img, color_name, (cx_px + 10, cy_px),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.detections_px.append((outer, inner, (cx_px, cy_px), color_name))

        # Publish debug 4 — color
        self._publish_debug(self.debug_color_pub, debug_color_img)

        # Publish main detections image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
        except CvBridgeError:
            pass

        # Optional local debug window (2x2 grid)
        if self.show_debug_window:
            self._show_local_debug(thresh, debug_ellipses_img, debug_pairs_img, debug_color_img)

    # ------------------------------------------------------------------
    # Point cloud callback — 3D localization with multi-point sampling
    # ------------------------------------------------------------------

    def pointcloud_callback(self, data: PointCloud2) -> None:
        if not self.detections_px:
            return

        height = data.height
        width = data.width

        a = pc2.read_points_numpy(data, field_names=["x", "y", "z"])
        a = a.reshape((height, width, 3))

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', data.header.frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        for outer, inner, (cx_px, cy_px), color_name in self.detections_px:
            # Sample multiple points around the outer ellipse boundary
            sample_pts = _sample_ellipse_points(outer, n=12)
            valid_xyz = []
            for px, py in sample_pts:
                if 0 <= py < height and 0 <= px < width:
                    pt = a[py, px, :]
                    if not np.isnan(pt).any():
                        valid_xyz.append(pt)

            # Also try the center pixel
            if 0 <= cy_px < height and 0 <= cx_px < width:
                cpt = a[cy_px, cx_px, :]
                if not np.isnan(cpt).any():
                    valid_xyz.append(cpt)

            if len(valid_xyz) < 3:
                continue

            # Median of valid points — robust to outliers and NaN-adjacent noise
            median_point = np.median(np.array(valid_xyz), axis=0)

            map_point = self._transform_point(median_point, transform)
            if map_point is None:
                continue

            self._process_detection(map_point, color_name, data.header.stamp)

        self.detections_px = []

    # ------------------------------------------------------------------
    # Transform, confirmation, markers — kept from original
    # ------------------------------------------------------------------

    def _transform_point(self, point, transform):
        """Transform a 3D point using a geometry_msgs Transform."""
        t = transform.transform.translation
        q = transform.transform.rotation
        px, py, pz = float(point[0]), float(point[1]), float(point[2])

        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        ux, uy, uz = qx, qy, qz
        cx1 = uy * pz - uz * py
        cy1 = uz * px - ux * pz
        cz1 = ux * py - uy * px
        cx2 = uy * cz1 - uz * cy1
        cy2 = uz * cx1 - ux * cz1
        cz2 = ux * cy1 - uy * cx1

        rx = px + 2.0 * (qw * cx1 + cx2)
        ry = py + 2.0 * (qw * cy1 + cy2)
        rz = pz + 2.0 * (qw * cz1 + cz2)

        return np.array([rx + t.x, ry + t.y, rz + t.z])

    def _process_detection(self, map_point, color_name, stamp):
        """Multi-frame confirmation and deduplication."""
        for conf in self.confirmed:
            if np.linalg.norm(map_point - conf['pos']) < self.dedup_distance:
                return

        matched = False
        for cand in self.candidates:
            if np.linalg.norm(map_point - cand['pos']) < self.dedup_distance:
                cand['count'] += 1
                cand['pos'] = (cand['pos'] * (cand['count'] - 1) + map_point) / cand['count']
                matched = True
                if cand['count'] >= self.confirmation_count:
                    self._confirm_ring(cand['pos'], cand['color'], stamp)
                    self.candidates.remove(cand)
                break

        if not matched:
            self.candidates.append({'pos': map_point.copy(), 'color': color_name, 'count': 1})

    def _confirm_ring(self, map_point, color_name, stamp):
        """Publish a confirmed ring detection."""
        self.confirmed.append({'pos': map_point, 'color': color_name})
        ring_id = len(self.confirmed)

        self.get_logger().info(
            f'Ring #{ring_id} ({color_name}) confirmed at '
            f'({map_point[0]:.2f}, {map_point[1]:.2f}, {map_point[2]:.2f})')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(map_point[0])
        pose.pose.position.y = float(map_point[1])
        pose.pose.position.z = float(map_point[2])
        pose.pose.orientation.w = 1.0
        self.ring_pub.publish(pose)

        color_msg = String()
        color_msg.data = color_name
        self.color_pub.publish(color_msg)

        self._publish_markers()

    def _publish_markers(self):
        """Publish all confirmed rings as a MarkerArray."""
        marker_array = MarkerArray()
        markers: list[Marker] = []
        for i, ring in enumerate(self.confirmed):
            pos = ring['pos']
            color = COLOR_RANGES.get(ring['color'], {'rgb': (1.0, 1.0, 1.0)})['rgb']

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rings'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            markers.append(marker)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'ring_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = float(pos[2]) + 0.2
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 1.0
            text.text = f'{ring["color"]} ring'
            text.lifetime.sec = 0
            markers.append(text)

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Debug helpers
    # ------------------------------------------------------------------

    def _publish_debug(self, publisher, image: np.ndarray, mono: bool = False) -> None:
        """Publish a debug image, converting mono to 3-channel if needed."""
        try:
            if mono:
                msg = self.bridge.cv2_to_imgmsg(image, "mono8")
            else:
                msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            publisher.publish(msg)
        except CvBridgeError:
            pass

    def _show_local_debug(self, binary: np.ndarray, ellipses: np.ndarray,
                          pairs: np.ndarray, color: np.ndarray) -> None:
        """Show a 2x2 grid of debug images in a local OpenCV window."""
        target_h, target_w = 240, 320

        def _resize(img: np.ndarray) -> np.ndarray:
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return cv2.resize(img, (target_w, target_h))

        top = np.hstack([_resize(binary), _resize(ellipses)])
        bottom = np.hstack([_resize(pairs), _resize(color)])

        # Add small labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        for label, x in [('Binary', 5), ('Ellipses', target_w + 5)]:
            cv2.putText(top, label, (x, 18), font, 0.5, (0, 255, 255), 1)
        for label, x in [('Pairs', 5), ('Color', target_w + 5)]:
            cv2.putText(bottom, label, (x, 18), font, 0.5, (0, 255, 255), 1)

        grid = np.vstack([top, bottom])
        cv2.imshow('Ring Debug', grid)
        cv2.waitKey(1)


def main(args=None):
    print('Ring detection node starting.')
    rclpy.init(args=args)
    node = RingDetectorNode()
    rclpy.spin(node)
    if node.show_debug_window:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
