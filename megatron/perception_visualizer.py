from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from typing import Optional as _Optional
from megatron.compression_utils import Decoder

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
    DEPTH_WINDOW_NAME = 'Gemini Depth'
    COLOR_WINDOW_NAME = "Gemini Image"

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
        self.decoder = Decoder()
        # Main detector images
        self.face_image: Optional[np.ndarray] = None
        self.ring_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_raw_image: Optional[np.ndarray] = None
        self.color_raw_image: Optional[np.ndarray] = None
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
            CompressedImage, '/gemini/color/image_raw/compressed', self._color_image_cb, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, '/gemini/depth/image_raw/compressedDepth', self._depth_image_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/detected_faces', self._face_pose_cb, 10)
        self.create_subscription(PoseStamped, '/detected_rings', self._ring_pose_cb, 10)
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

    def _color_image_cb(self, msg: CompressedImage) -> None:
        self.color_raw_image = self.decoder.decode_img(msg)
    
    def _depth_image_cb(self, msg: CompressedImage) -> None:
        self.depth_raw_image = self.decoder.decode_depth(msg)


    def _compressed_color_image_cb(self, msg: CompressedImage) -> None:
        # self.get_logger().info('Received Color image for visualization.')
        try:
            # depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # h, w = depth_img.shape[:2]
            # cx, cy = w // 2, h // 2
            
            # if len(depth_img.shape) == 3:
            #     depth_img = depth_img[:, :, 0]
                
            # dist = float(depth_img[cy, cx])
                
            # if depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
            #     vis_img = np.clip(depth_img / 5.0 * 255.0, 0, 255).astype(np.uint8)
            # else:
            #     vis_img = np.clip(depth_img / 5000.0 * 255.0, 0, 255).astype(np.uint8)
            #     dist = dist / 1000.0
                
            # vis_img_bgr = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)
            # cv2.circle(vis_img_bgr, (cx, cy), 5, (0, 0, 255), -1)
            # cv2.putText(vis_img_bgr, f"{dist:.2f}m", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # self.depth_image = vis_img_bgr
            self.color_raw_image = self._to_bgr_compressed(msg)
            self.depth_image = self._to_bgr_compressed(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f'Color image conversion failed: {exc}')

    def _compressed_depth_cb(self, msg: CompressedImage) -> None:
        depth_img = self._to_depth_compressed(msg)
        if depth_img is None:
            return
        # self.get_logger().info(f" type: {type(depth_img)}, {depth_img.shape}")
        try:
            # depth_img = self.bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')
            h, w = depth_img.shape[:2]
            cx, cy = w // 2, h // 2
            
            if len(depth_img.shape) == 3:
                self.get_logger().info(f"Depth image shape: {len(depth_img.shape)} -> {depth_img.shape}")
                depth_img = depth_img[:, :, 0]
                
            dist = float(depth_img[cy, cx])
                
            if depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
                vis_img = np.clip(depth_img / 5.0 * 255.0, 0, 255).astype(np.uint8)
            else:
                vis_img = np.clip(depth_img / 5000.0 * 255.0, 0, 255).astype(np.uint8)
                dist = dist / 1000.0
                
            vis_img_bgr = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)
            cv2.circle(vis_img_bgr, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(vis_img_bgr, f"{dist:.2f}m", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # self.depth_image = vis_img_bgr
            # self.depth_image = self._to_bgr_compressed(msg)
            self.depth_raw_image = vis_img_bgr
        except CvBridgeError as exc:
            self.get_logger().warn(f'Depth calculation failed: {exc}')
            # if not isinstance(depth, np.ndarray) or depth.ndim < 2:
            #     self.get_logger().warn('Compressed depth decode returned invalid image shape.')
            #     return
            self.depth_raw_image = depth_img
            # self.depth_vis_image = self._depth_to_vis(depth)

    def _ring_image_cb(self, msg: Image) -> None:
        self.ring_image = self._to_bgr(msg)

    def _face_pose_cb(self, _: PoseStamped) -> None:
        self.face_count += 1

    def _ring_pose_cb(self, msg: PoseStamped) -> None:
        self.ring_count += 1
        # Parse color from frame_id ("map|color")
        frame_id = msg.header.frame_id
        if '|' in frame_id:
            self.last_ring_color = frame_id.split('|', 1)[1]
        else:
            self.last_ring_color = 'unknown'

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
        
    def _to_passthrough(self, msg:Image) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        except CvBridgeError as exc:
            self.get_logger().warn(f'Image conversion failed passthrough: {exc}')
            return None
    def _to_any(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg))
        except CvBridgeError as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return None

    def _to_bgr_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(
                self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Compressed image conversion failed: {exc}')
            return None

    def _to_depth_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            decoded = None
            if 'compressedDepth' in msg.format:
            # 1. Strip the 12-byte ROS compressedDepth header
                depth_header_size = 12
                raw_data = msg.data[depth_header_size:]
                
                # 2. Decode the raw PNG data using NumPy and OpenCV
                np_arr = np.frombuffer(raw_data, np.uint8)
                decoded = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            else:
                # It's a standard compressed color image (JPEG/PNG), cv_bridge handles it fine
                decoded = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

            if decoded is None: 
                self.get_logger().warn('1 -> Compressed depth decode returned None.')
                self.get_logger().info(f"Format: {msg.format}, Data length: {len(msg.data)}")
                return None
            # cv_bridge handles compressedDepth when image_transport plugins are present.
            # decoded = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # if decoded is None:
            #     return None
            # disp_depth = cv2.normalize(decoded, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # return disp_depth
            return decoded
            # return np.ascontiguousarray(decoded)
        except CvBridgeError:
            self.get_logger().error("1 -> Failed to decompress image")
            return None
            # self.get_logger().warn("Using fallback mode for decoding compressed depth")
            # # Fallback: decode PNG payload directly for compressedDepth transport.
            # data = np.frombuffer(msg.data, dtype=np.uint8)
            # depth = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
            # if depth is None:
            #     self.get_logger().warn('Compressed depth decode failed.')
            #     return None
            # return np.ascontiguousarray(depth)

    def _depth_to_vis(self, depth: np.ndarray) -> np.ndarray:
        if depth is None or not isinstance(depth, np.ndarray) or depth.ndim == 0:
            return np.full((240, 320), 0, dtype=np.uint8)

        if depth.ndim == 3:
            depth = depth[:, :, 0]

        valid = depth > 0
        if not np.any(valid):
            return np.full((depth.shape[0], depth.shape[1]), 0, dtype=np.uint8)

        if depth.dtype in (np.float32, np.float64):
            depth_m = depth.astype(np.float32)
        elif depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        max_m = np.percentile(depth_m[valid], 98.0)
        max_m = max(max_m, 0.5)
        scaled = np.clip((depth_m / max_m) * 255.0, 0, 255).astype(np.uint8)
        gray = 255 - scaled
        gray[~valid] = 0
        return gray

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
            # if self.depth_vis_image is not None:
            #     cv2.imshow(self.DEPTH_WINDOW_NAME, self.depth_vis_image)
            if self.depth_raw_image is not None:
                cv2.imshow(self.DEPTH_WINDOW_NAME, self.depth_raw_image)
            if self.color_raw_image is not None:
                cv2.imshow(self.COLOR_WINDOW_NAME, self.color_raw_image)
            
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
        depth_panel = _fit_image(self.depth_image, pw, ph)
        _overlay_label(depth_panel, 'Depth Debug')
        return np.hstack([face_panel, depth_panel])

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
