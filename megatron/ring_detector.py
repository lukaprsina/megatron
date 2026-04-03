"""Ring detector with PointCloud2-based 3D projection and SVD surface fitting.

Subscribes to synced RGB + PointCloud2 via message_filters, detects concentric
ellipse pairs (rings), classifies color via HSV, extracts 3D points from the
annular region via the organized PointCloud2, fits surface normals, and publishes
confirmed detections as PoseStamped with color packed in frame_id ("map|{color}").
"""

import math
import random as rnd
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.time import Time

import message_filters
import numpy as np
import cv2
import tf2_ros

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError

from megatron.perception_utils import (
    IncrementalTrackManager,
    compute_robust_surface,
    extract_3d_points_from_pc2,
    normal_to_quaternion,
    transform_point_and_normal,
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
    'black':  {'lower1': (0, 0, 0),       'upper1': (180, 255, 35),
               'rgb': (0.2, 0.2, 0.2)},
}


# ---------------------------------------------------------------------------
# Color classification helpers
# ---------------------------------------------------------------------------

def _build_annular_mask(shape: tuple,
                        outer_ellipse: tuple, inner_ellipse: tuple) -> np.ndarray:
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
        hit = cv2.bitwise_and(color_mask, mask)
        count = int(cv2.countNonZero(hit))
        if count > best_count:
            best_count = count
            best_name = name

    confidence = best_count / total_pixels if total_pixels > 0 else 0.0
    return best_name, confidence


def _check_hole(image_gray: np.ndarray,
                outer_ellipse: tuple, inner_ellipse: tuple,
                min_brightness_diff: float = 20.0, logger = None) -> bool:
    h, w = image_gray.shape[:2]
    band_mask = _build_annular_mask((h, w), outer_ellipse, inner_ellipse)
    inner_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.ellipse(inner_mask, inner_ellipse, (255,), -1)
    
    # band_pixels = cv2.mean(image_gray, mask=band_mask)[0]
    # inner_pixels = cv2.mean(image_gray, mask=inner_mask)[0]

    # if cv2.countNonZero(band_mask) == 0 or cv2.countNonZero(inner_mask) == 0:
    #     return False

    # creates median
    band_vals = image_gray[band_mask > 0]
    inner_vals = image_gray[inner_mask > 0]
    if band_vals.size == 0 or inner_vals.size == 0:
        return False

    band_median = float(np.median(band_vals))
    inner_median = float(np.median(inner_vals))
    if logger:
        logger.info(f'[DBG] Hole check: band median={band_median:.1f}, inner median={inner_median:.1f}, diff={abs(band_median - inner_median):.1f}')
    return abs(band_median - inner_median) >= min_brightness_diff
    

def _check_band_uniformity(image_bgr: np.ndarray,
                           outer_ellipse: tuple, inner_ellipse: tuple,
                           max_std: float = 35.0) -> bool:
    """Reject candidates whose annular band has high color variance.

    Real rings are a single solid color (low V-channel std).  Face regions,
    textured surfaces, etc. have high variance and get rejected.
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    band_mask = _build_annular_mask(image_bgr.shape, outer_ellipse, inner_ellipse)
    if cv2.countNonZero(band_mask) == 0:
        return False
    v_channel = hsv[:, :, 2]
    pixels = v_channel[band_mask > 0]
    return float(np.std(pixels)) <= max_std


def _check_depth_discontinuity(pc2_msg: PointCloud2,
                                outer_ellipse: tuple, inner_ellipse: tuple,
                                shape: tuple,
                                min_depth_gap: float = 0.15) -> bool:
    """Check if the ring hole shows a depth gap vs. the band.

    For hanging rings the hole is open air (NaN/far) while the band is at
    ring distance.  Returns True if the hole is significantly farther or
    mostly invalid, confirming a real ring.  Returns False (inconclusive)
    for wall-mounted rings where depths are similar — caller should NOT
    use this as a hard reject, only as additive evidence.
    """
    h, w = shape[:2]
    band_mask = _build_annular_mask((h, w), outer_ellipse, inner_ellipse)
    inner_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.ellipse(inner_mask, inner_ellipse, (255,), -1)

    band_pts = extract_3d_points_from_pc2(band_mask, pc2_msg)
    inner_pts = extract_3d_points_from_pc2(inner_mask, pc2_msg)

    if len(band_pts) < 3:
        return False

    band_depths = np.linalg.norm(band_pts, axis=1)
    band_median = float(np.median(band_depths))

    # If most inner points are invalid/missing → open air behind the ring
    inner_pixel_count = int(cv2.countNonZero(inner_mask))
    if inner_pixel_count > 0 and len(inner_pts) < inner_pixel_count * 0.6:
        return True

    if len(inner_pts) < 3:
        return True  # very few valid inner points → likely open air

    inner_depths = np.linalg.norm(inner_pts, axis=1)
    inner_median = float(np.median(inner_depths))

    return (inner_median - band_median) >= min_depth_gap


# ---------------------------------------------------------------------------
# Ellipse quality scoring
# ---------------------------------------------------------------------------

def _ellipse_area(e: tuple) -> float:
    return math.pi * (e[1][0] / 2.0) * (e[1][1] / 2.0)


def _ellipse_score(contour: np.ndarray, ellipse: tuple) -> float:
    ca = cv2.contourArea(contour)
    ea = _ellipse_area(ellipse)
    if ea < 1.0:
        return 0.0
    return min(ca / ea, 1.0)


def _score_to_bgr(score: float) -> tuple[int, int, int]:
    g = int(255 * score)
    r = int(255 * (1.0 - score))
    return (0, g, r)


# ---------------------------------------------------------------------------
# Ellipse-pair ring candidate detection
# ---------------------------------------------------------------------------

def _ellipse_contains(larger: tuple, smaller: tuple) -> bool:
    return (larger[1][0] >= smaller[1][0] and larger[1][1] >= smaller[1][1])


def _pair_score(e1: tuple, e2: tuple) -> float:
    dist = math.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1])
    if e1[1][0] * e1[1][1] >= e2[1][0] * e2[1][1]:
        le, se = e1, e2
    else:
        le, se = e2, e1

    max_axis = max(le[1][0], le[1][1], 1.0)
    center_score = max(0.0, 1.0 - dist / (max_axis * 0.5))

    ratio = (se[1][0] * se[1][1]) / max(le[1][0] * le[1][1], 1.0)
    if 0.2 <= ratio <= 0.98:
        ratio_score = 1.0 - abs(ratio - 0.7) / 0.5
    else:
        ratio_score = 0.0

    ecc1 = max(le[1]) / max(min(le[1]), 1.0)
    ecc2 = max(se[1]) / max(min(se[1]), 1.0)
    ecc_score = max(0.0, 1.0 - abs(ecc1 - ecc2) / 2.0)

    return 0.5 * center_score + 0.3 * ratio_score + 0.2 * ecc_score


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class RingDetectorNode(Node):

    def __init__(self):
        super().__init__('ring_detector')

        # Commented out are the old parameter values

        # ---- Parameters ----
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('min_inference_period', 0.2)
        
        # Adaptive threshold
        self.declare_parameter('thresh_block_size', 15)
        self.declare_parameter('thresh_c', 25)
        # Ellipse filtering
        self.declare_parameter('min_contour_points', 8)
        self.declare_parameter('max_axis', 200.0)
        self.declare_parameter('min_axis', 3.0)
        self.declare_parameter('max_aspect_ratio', 1.8)

        #self.declare_parameter('min_contour_points', 20)
        #self.declare_parameter('min_axis', 6.0)

        # Pair matching
        self.declare_parameter('center_thr', 15.0)
        # self.declare_parameter('min_pair_score', 0.40)
        self.declare_parameter('min_pair_score', 0.30)
        # Color classification
        self.declare_parameter('min_color_confidence', 0.15)
        self.declare_parameter('min_color_pixels', 6)
        # Hole check
        self.declare_parameter('min_brightness_diff', 8.0)
        # self.declare_parameter('min_brightness_diff', 20.0)
        
        # Band uniformity (low = solid color ring, high = textured face/wall)
        self.declare_parameter('max_band_std', 35.0)
        # Depth discontinuity (hanging rings have gap behind hole)
        self.declare_parameter('min_depth_gap', 0.15)
        self.declare_parameter('min_ring_points_3d', 3)

            

        self.confirmation_count = self.get_parameter('confirmation_count').value
        self.dedup_distance = self.get_parameter('dedup_distance').value
        self.min_inference_period = self.get_parameter('min_inference_period').value
        self.thresh_block_size = self.get_parameter('thresh_block_size').value
        self.thresh_c = self.get_parameter('thresh_c').value
        self.min_contour_points = self.get_parameter('min_contour_points').value
        self.max_axis = self.get_parameter('max_axis').value
        self.min_axis = self.get_parameter('min_axis').value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').value
        self.center_thr = self.get_parameter('center_thr').value
        self.min_pair_score = self.get_parameter('min_pair_score').value
        self.min_color_confidence = self.get_parameter('min_color_confidence').value
        self.min_color_pixels = self.get_parameter('min_color_pixels').value
        self.min_brightness_diff = self.get_parameter('min_brightness_diff').value
        self.max_band_std = self.get_parameter('max_band_std').value
        self.min_depth_gap = self.get_parameter('min_depth_gap').value
        self.min_ring_points_3d = self.get_parameter('min_ring_points_3d').value

        # Enforce odd block size >= 3
        if self.thresh_block_size % 2 == 0:
            self.thresh_block_size += 1
        if self.thresh_block_size < 3:
            self.thresh_block_size = 3

        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Synced RGB + PointCloud2 via message_filters
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.pc2_sub = message_filters.Subscriber(
            self, PointCloud2, '/oakd/rgb/preview/depth/points',
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.pc2_sub], queue_size=10, slop=0.15)
        self.sync.registerCallback(self._synced_callback)

        # Publishers — detections
        self.ring_pub = self.create_publisher(PoseStamped, '/detected_rings', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/ring_markers', QoSReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, '/ring_detections_image', 10)

        # Publishers — debug (4 stages)
        self.debug_binary_pub = self.create_publisher(Image, '/ring_debug/binary', 10)
        self.debug_ellipses_pub = self.create_publisher(Image, '/ring_debug/ellipses', 10)
        self.debug_pairs_pub = self.create_publisher(Image, '/ring_debug/pairs', 10)
        self.debug_color_pub = self.create_publisher(Image, '/ring_debug/color', 10)

        # Track manager
        self.track_manager = IncrementalTrackManager(
            dedup_distance=self.dedup_distance,
            confirmation_count=self.confirmation_count)

        # Rate limiting
        self.last_inference_time = 0.0

        self.get_logger().info('Ring detector initialized (PointCloud2 mode).')

    # ------------------------------------------------------------------
    # Synced RGB + PointCloud2 callback
    # ------------------------------------------------------------------

    def _synced_callback(self, rgb_msg: Image, pc2_msg: PointCloud2):
        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_inference_time < self.min_inference_period:
            return
        self.last_inference_time = now

        # Convert RGB image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        h, w = cv_image.shape[:2]

        # Get TF: PC2 frame → map
        frame_id = pc2_msg.header.frame_id
        if not frame_id:
            frame_id = 'oakd_rgb_camera_optical_frame'
        try:
            tf_stamped = self.tf_buffer.lookup_transform('map', frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        # ---- Stage 1: adaptive threshold -> binary ----
        # ---- Stage 1: adaptive threshold + saturation mask -> binary ----
        
        # 1. Existing grayscale adaptive threshold (background white, rings/edges black)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh_gray = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            self.thresh_block_size, self.thresh_c,
        )

        # 2. Global saturation threshold
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        s_channel = hsv[:, :, 1]
        
        # INVERTED THRESHOLD: Vivid things become BLACK (0), dull things become WHITE (255)
        # S-values in Gazebo for these rings are usually very high, 60 is a safe threshold
        _, thresh_color_inv = cv2.threshold(s_channel, 130, 255, cv2.THRESH_BINARY_INV)

        # 3. Combine with AND! 
        # If a pixel is black (0) in EITHER image, it becomes black in the final image.
        thresh = cv2.bitwise_and(thresh_gray, thresh_color_inv)

        # Reconnect thin breaks in ring bands caused by small occluders.
        # !!!!!!!!!!!!!!!1 This is needed for the situations where there is a stick inside the ring or infront so that it doesn't break the ring     
        fg = cv2.bitwise_not(thresh)
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, k_close, iterations=2)
        # k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, k_open, iterations=1)
        
        thresh = cv2.bitwise_not(fg)
        
        self._publish_debug(self.debug_binary_pub, thresh, mono=True)
        # ---- Stage 2: contour extraction -> ellipse fitting & scoring ----
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        ellipses: list[tuple[tuple, np.ndarray, float]] = []
        debug_ellipses_img = cv_image.copy()

        for cnt in contours:
            if len(cnt) < self.min_contour_points:
                continue
            ellipse = cv2.fitEllipse(cnt)
            (ex, ey), (ew, eh), angle = ellipse

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

            #color_bgr = _score_to_bgr(score)
            color_bgr = (rnd.randint(0, 255), rnd.randint(0, 255), rnd.randint(0, 255))
            cv2.ellipse(debug_ellipses_img, ellipse, color_bgr, 1)
            cv2.circle(debug_ellipses_img, (int(ex), int(ey)), 2, color_bgr, -1)
            cv2.putText(debug_ellipses_img, f'{score:.2f}',
                        (int(ex) + 5, int(ey) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color_bgr, 1)

            
        self._publish_debug(self.debug_ellipses_pub, debug_ellipses_img)

        # ---- Stage 3: concentric pair matching ----
        debug_pairs_img = cv_image.copy()
        ring_candidates: list[tuple[tuple, tuple, float, tuple[int, int]]] = []

        used = set()
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

        # ---- Stage 4: color classification + hole check + 3D projection ----
        debug_color_img = cv_image.copy()
        output_img = cv_image.copy()

        for outer, inner, ps, (cx_px, cy_px) in ring_candidates:
            #Skipped these two because they failed for the green one
            # Hole check: main_brightness_diff should have been less than 2 for the green ring to work or at least 8
            # Band uniformity: it just failed for green one i dont know why  
            
            # Hole check
            # has_hole = _check_hole(gray, outer, inner, self.min_brightness_diff, logger=self.get_logger())
            # if not has_hole:
            #     cv2.ellipse(debug_color_img, outer, (0, 0, 180), 1)
            #     cv2.ellipse(debug_color_img, inner, (0, 0, 180), 1)
            #     cv2.putText(debug_color_img, 'solid',
            #                 (cx_px - 8, cy_px),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 180), 1)
            #     continue

            # Band uniformity: reject high-variance regions (faces, textures)
            # if not _check_band_uniformity(cv_image, outer, inner, self.max_band_std):
            #     cv2.ellipse(debug_color_img, outer, (0, 128, 180), 1)
            #     cv2.ellipse(debug_color_img, inner, (0, 128, 180), 1)
            #     cv2.putText(debug_color_img, 'nonuniform',
            #                 (cx_px + 8, cy_px),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 128, 180), 1)
            #     self.get_logger().info(f'[DBG] Candidate at ({cx_px}, {cy_px}) failed band uniformity check (likely face/texture), skipping.')
            #     continue

            color_name, confidence = classify_ring_color(
                cv_image, outer, inner, min_pixels=self.min_color_pixels)

            if color_name is None or confidence < self.min_color_confidence:
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

        
            #!!!!!!!!!!!!!!! This is the main checking function for cheking if this ring is 3d  if there is something behind the ring we are fucked !!!!!!!!!!!!!!
            # Depth discontinuity: additive signal for hanging rings
            has_depth_gap = _check_depth_discontinuity(
                pc2_msg, outer, inner, cv_image.shape, self.min_depth_gap)
        
            if not has_depth_gap:
                # Hole is at same depth as ring band → wall-mounted, reject
                cv2.putText(debug_color_img, 'wall', (cx_px + 8, cy_px + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 80, 200), 1)
                # Yes this filters rings on walls
                # self.get_logger().info(
                #     f'[DBG] Candidate at ({cx_px}, {cy_px}) classified as {color_name} but failed depth gap check (likely wall-mounted), skipping.')
                continue

            # ---- 3D projection via annular mask + PointCloud2 ----
            annular_mask = _build_annular_mask((h, w), outer, inner)
            points_3d = extract_3d_points_from_pc2(annular_mask, pc2_msg)
            if len(points_3d) < self.min_ring_points_3d:
                continue
            
            result = compute_robust_surface(points_3d)
            if result is None:
                continue
            
            centroid, normal = result

            # Transform to map frame
            map_point, map_normal = transform_point_and_normal(
                centroid, normal, tf_stamped)

            cam_dist = float(np.linalg.norm(centroid))

            # Feed to tracker
            status, track = self.track_manager.add_observation(
                map_point, map_normal, cam_dist,
                rgb_msg.header.stamp, label=color_name)

            if status == 'confirmed':
                self._publish_detection(track, rgb_msg.header.stamp)

        # Publish debug 4 — color
        self._publish_debug(self.debug_color_pub, debug_color_img)

        # Publish main detections image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))
        except CvBridgeError:
            pass

    # ------------------------------------------------------------------
    # Publish confirmed detection
    # ------------------------------------------------------------------

    def _publish_detection(self, track, stamp):
        pos, normal = self.track_manager.get_best_estimate(track)
        color_name = track.get('label', 'unknown')

        self.get_logger().info(
            f'Ring #{track["id"]} ({color_name}) confirmed at '
            f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        # PoseStamped: pack color into frame_id as "map|{color}"
        pose = PoseStamped()
        pose.header.frame_id = f'map|{color_name}'
        pose.header.stamp = stamp
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])

        qx, qy, qz, qw = normal_to_quaternion(normal[:2])
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.ring_pub.publish(pose)
        self._publish_markers()

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------

    def _publish_markers(self):
        marker_array = MarkerArray()
        markers: list[Marker] = []
        now_stamp = self.get_clock().now().to_msg()
        for track in self.track_manager.get_confirmed_tracks():
            pos, nrm = self.track_manager.get_best_estimate(track)
            color_name = track.get('label', 'unknown')
            i = track['id'] - 1

            rgb = COLOR_RANGES.get(color_name, {'rgb': (1.0, 1.0, 1.0)})['rgb']

            # Sphere marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_stamp
            m.ns = 'rings'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = rgb[0]
            m.color.g = rgb[1]
            m.color.b = rgb[2]
            m.color.a = 1.0
            m.lifetime.sec = 0
            markers.append(m)

            # Arrow marker (surface normal direction)
            qx, qy, qz, qw = normal_to_quaternion(nrm[:2])
            a = Marker()
            a.header.frame_id = 'map'
            a.header.stamp = now_stamp
            a.ns = 'ring_normals'
            a.id = i
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.position.x = float(pos[0])
            a.pose.position.y = float(pos[1])
            a.pose.position.z = float(pos[2])
            a.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            a.scale.x = 0.5   # shaft length
            a.scale.y = 0.05  # shaft diameter
            a.scale.z = 0.08  # head diameter
            a.color.r = rgb[0]
            a.color.g = rgb[1]
            a.color.b = rgb[2]
            a.color.a = 1.0
            a.lifetime.sec = 0
            markers.append(a)

            # Text label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now_stamp
            t.ns = 'ring_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pos[0])
            t.pose.position.y = float(pos[1])
            t.pose.position.z = float(pos[2]) + 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = rgb[0]
            t.color.g = rgb[1]
            t.color.b = rgb[2]
            t.color.a = 1.0
            t.text = f'{color_name} ring'
            t.lifetime.sec = 0
            markers.append(t)

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Debug helpers
    # ------------------------------------------------------------------

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
