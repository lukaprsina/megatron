from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Shared UI helpers
# ---------------------------------------------------------------------------

def _overlay_label(image: np.ndarray, text: str, pos: tuple[int, int] = (4, 4),
                   font_scale: float = 0.5, thickness: int = 1,
                   alpha: float = 0.9) -> None:
    """Draw a text label with a semi-transparent black background, in-place."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = pos
    pad = 4
    # Background rectangle
    overlay = image.copy()
    cv2.rectangle(overlay, (x, y), (x + tw + 2 * pad, y + th + 2 * pad + baseline),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
    # Text
    cv2.putText(image, text, (x + pad, y + pad + th),
                font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)


def _fit_image(image: Optional[np.ndarray], target_w: int, target_h: int) -> np.ndarray:
    """Resize an image to fit within target_w x target_h, preserving aspect ratio,
    centered on a dark background. Returns a BGR image."""
    canvas = np.full((target_h, target_w, 3), 18, dtype=np.uint8)
    if image is None or image.size == 0:
        return canvas
    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    scale = min(target_w / image.shape[1], target_h / image.shape[0])
    rw = max(1, int(image.shape[1] * scale))
    rh = max(1, int(image.shape[0] * scale))
    resized = cv2.resize(image, (rw, rh))
    y = (target_h - rh) // 2
    x = (target_w - rw) // 2
    canvas[y:y + rh, x:x + rw] = resized
    return canvas


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class PerceptionVisualizer(Node):
    """Compose detector outputs into a single demo-friendly visualization."""

    WINDOW_NAME = 'Faces and Rings Camera'

    def __init__(self) -> None:
        super().__init__('perception_visualizer')

        self.declare_parameter('show_window', False)
        self.declare_parameter('refresh_rate', 10.0)
        self.declare_parameter('panel_height', 360)

        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().double_value
        self.panel_height = self.get_parameter('panel_height').get_parameter_value().integer_value

        if self.refresh_rate <= 0.0:
            self.refresh_rate = 10.0
        if self.panel_height <= 0:
            self.panel_height = 360

        self.bridge = CvBridge()

        # Main detector images
        self.face_image: Optional[np.ndarray] = None
        self.ring_image: Optional[np.ndarray] = None
        self.face_count = 0
        self.ring_count = 0
        self.last_ring_color = '—'
        self.mission_status = 'WAITING_FOR_NAV2'

        # Ring debug images (4 stages)
        self.ring_debug_binary: Optional[np.ndarray] = None
        self.ring_debug_ellipses: Optional[np.ndarray] = None
        self.ring_debug_pairs: Optional[np.ndarray] = None
        self.ring_debug_color: Optional[np.ndarray] = None

        # Subscriptions — main
        self.create_subscription(
            Image, '/face_detections_image', self._face_image_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/ring_detections_image', self._ring_image_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/detected_faces', self._face_pose_cb, 10)
        self.create_subscription(PoseStamped, '/detected_rings', self._ring_pose_cb, 10)
        self.create_subscription(String, '/detected_ring_color', self._ring_color_cb, 10)
        self.create_subscription(String, '/mission_status', self._mission_status_cb, 10)

        # Subscriptions — ring debug
        self.create_subscription(Image, '/ring_debug/binary', self._rd_binary_cb, 10)
        self.create_subscription(Image, '/ring_debug/ellipses', self._rd_ellipses_cb, 10)
        self.create_subscription(Image, '/ring_debug/pairs', self._rd_pairs_cb, 10)
        self.create_subscription(Image, '/ring_debug/color', self._rd_color_cb, 10)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/task1_visualization_image', 10)
        self.rviz_pub = self.create_publisher(Image, '/task1_rviz_image', 10)

        self.timer = self.create_timer(1.0 / self.refresh_rate, self._tick)

        self.get_logger().info('Perception visualizer initialized.')

    # --- Callbacks --------------------------------------------------------

    def _face_image_cb(self, msg: Image) -> None:
        self.face_image = self._to_bgr(msg)

    def _ring_image_cb(self, msg: Image) -> None:
        self.ring_image = self._to_bgr(msg)

    def _face_pose_cb(self, _: PoseStamped) -> None:
        self.face_count += 1

    def _ring_pose_cb(self, _: PoseStamped) -> None:
        self.ring_count += 1

    def _ring_color_cb(self, msg: String) -> None:
        self.last_ring_color = msg.data or 'unknown'

    def _mission_status_cb(self, msg: String) -> None:
        self.mission_status = msg.data or 'unknown'

    def _rd_binary_cb(self, msg: Image) -> None:
        self.ring_debug_binary = self._to_any(msg)

    def _rd_ellipses_cb(self, msg: Image) -> None:
        self.ring_debug_ellipses = self._to_bgr(msg)

    def _rd_pairs_cb(self, msg: Image) -> None:
        self.ring_debug_pairs = self._to_bgr(msg)

    def _rd_color_cb(self, msg: Image) -> None:
        self.ring_debug_color = self._to_bgr(msg)

    # --- Image conversion -------------------------------------------------

    def _to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        except CvBridgeError as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return None

    def _to_any(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg))
        except CvBridgeError as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return None

    # --- Timer callback ----------------------------------------------------

    def _tick(self) -> None:
        det_row = self._build_detection_row()
        header = self._build_header(det_row.shape[1])

        rviz_canvas = np.vstack([header, det_row])
        debug_canvas = np.vstack([header, det_row, self._build_debug_row()])

        try:
            self.rviz_pub.publish(self.bridge.cv2_to_imgmsg(rviz_canvas, encoding='bgr8'))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_canvas, encoding='bgr8'))
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to publish: {exc}')

        if self.show_window:
            cv2.imshow(self.WINDOW_NAME, debug_canvas)
            cv2.waitKey(1)

    # --- Canvas construction -----------------------------------------------

    def _build_header(self, width: int) -> np.ndarray:
        header = np.full((64, width, 3), 24, dtype=np.uint8)
        cv2.putText(header, 'Megatron Task 1 Perception', (16, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.82, (245, 245, 245), 2, cv2.LINE_AA)
        status = f'{self.mission_status}'
        cv2.putText(header, status, (16, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1, cv2.LINE_AA)
        return header

    def _build_detection_row(self) -> np.ndarray:
        pw, ph = 480, self.panel_height
        face_panel = _fit_image(self.face_image, pw, ph)
        _overlay_label(face_panel, 'Faces')
        ring_panel = _fit_image(self.ring_image, pw, ph)
        _overlay_label(ring_panel, 'Rings')
        return np.hstack([face_panel, ring_panel])

    def _build_debug_row(self) -> np.ndarray:
        cw, ch = 240, self.panel_height // 2
        cells = []
        for img, label in [
            (self.ring_debug_binary,   'Binary'),
            (self.ring_debug_ellipses, 'Ellipses'),
            (self.ring_debug_pairs,    'Pairs'),
            (self.ring_debug_color,    'Color'),
        ]:
            cell = _fit_image(img, cw, ch)
            _overlay_label(cell, label, font_scale=0.4)
            cells.append(cell)
        return np.hstack(cells)

    # --- Cleanup -----------------------------------------------------------

    def destroy_node(self) -> None:
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
