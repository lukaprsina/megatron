from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from megatron.perception_visualizer import _fit_image, _overlay_label


class WorkstationVisualizer(Node):
    """Compose workstation inspection debug images into a single visualization.

    Layout:
      [ HEADER: robot state | inspection phase ]
      [ LIVE: annotated top-camera frame (large) ]
      [ RAW (last tile) | THRESH | WARPED ]
    """

    WINDOW_NAME = "Workstation Inspection Debug"

    def __init__(self) -> None:
        super().__init__("workstation_visualizer")

        self.declare_parameter("show_window", False)
        self.declare_parameter("refresh_rate", 10.0)
        self.declare_parameter("live_height", 320)
        self.declare_parameter("debug_height", 240)

        self.show_window = self.get_parameter("show_window").get_parameter_value().bool_value
        self.refresh_rate = self.get_parameter("refresh_rate").get_parameter_value().double_value
        self.live_height = self.get_parameter("live_height").get_parameter_value().integer_value
        self.debug_height = self.get_parameter("debug_height").get_parameter_value().integer_value

        if self.refresh_rate <= 0.0:
            self.refresh_rate = 10.0

        self.bridge = CvBridge()

        self.robot_state = "UNKNOWN"
        self.inspection_phase = "—"

        self.live_image: np.ndarray | None = None
        self.raw_image: np.ndarray | None = None
        self.thresh_image: np.ndarray | None = None
        self.warped_image: np.ndarray | None = None

        self.create_subscription(String, "/robot_state", self._state_cb, 10)
        self.create_subscription(String, "/workstation_debug/phase", self._phase_cb, 10)
        self.create_subscription(
            Image, "/workstation_debug/live", self._live_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/workstation_debug/raw", self._raw_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/workstation_debug/thresh", self._thresh_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/workstation_debug/warped", self._warped_cb, qos_profile_sensor_data
        )

        self.image_pub = self.create_publisher(Image, "/workstation_visualization_image", 10)

        self._tick_count = 0
        self.timer = self.create_timer(1.0 / self.refresh_rate, self._tick)
        self.get_logger().info("Workstation visualizer initialized.")

    # --- Callbacks -----------------------------------------------------------

    def _state_cb(self, msg: String) -> None:
        self.robot_state = msg.data or "UNKNOWN"

    def _phase_cb(self, msg: String) -> None:
        self.inspection_phase = msg.data or "—"

    def _live_cb(self, msg: Image) -> None:
        self.live_image = self._to_bgr(msg)

    def _raw_cb(self, msg: Image) -> None:
        self.raw_image = self._to_bgr(msg)

    def _thresh_cb(self, msg: Image) -> None:
        self.thresh_image = self._to_any(msg)

    def _warped_cb(self, msg: Image) -> None:
        self.warped_image = self._to_any(msg)

    # --- Image helpers -------------------------------------------------------

    def _to_bgr(self, msg: Image) -> np.ndarray | None:
        try:
            return np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        except CvBridgeError as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return None

    def _to_any(self, msg: Image) -> np.ndarray | None:
        try:
            return np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg))
        except CvBridgeError as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return None

    # --- Canvas construction -------------------------------------------------

    def _build_header(self, width: int) -> np.ndarray:
        header = np.full((56, width, 3), 24, dtype=np.uint8)
        cv2.putText(
            header,
            "Workstation Inspection Debug",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (245, 245, 245),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            header,
            f"state={self.robot_state}  {self.inspection_phase}",
            (12, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (180, 220, 180),
            1,
            cv2.LINE_AA,
        )
        return header

    def _build_live_row(self, width: int) -> np.ndarray:
        panel = _fit_image(self.live_image, width, self.live_height)
        _overlay_label(panel, "Live top camera (ROI in cyan = brightness trigger zone)")
        return panel

    def _build_debug_row(self, width: int) -> np.ndarray:
        cell_w = width // 3
        cells = []
        for img, label in [
            (self.raw_image, "Raw frame (at score time)"),
            (self.thresh_image, "Otsu threshold"),
            (self.warped_image, "Perspective warp"),
        ]:
            cell = _fit_image(img, cell_w, self.debug_height)
            _overlay_label(cell, label, font_scale=0.42)
            cells.append(cell)
        return np.hstack(cells)

    # --- Timer ---------------------------------------------------------------

    def _tick(self) -> None:
        self._tick_count += 1
        base_w = self.live_image.shape[1] if self.live_image is not None else 640
        base_w = max(base_w, 640)
        # Round up to next multiple of 3 so debug cells tile exactly
        canvas_w = base_w + (-base_w % 3)

        if self._tick_count % 30 == 1:
            self.get_logger().info(
                f"[ws_viz tick={self._tick_count}] canvas_w={canvas_w} "
                f"live={'yes ' + str(self.live_image.shape) if self.live_image is not None else 'None'} "
                f"raw={'yes' if self.raw_image is not None else 'None'} "
                f"thresh={'yes' if self.thresh_image is not None else 'None'} "
                f"warped={'yes' if self.warped_image is not None else 'None'}"
            )

        header = self._build_header(canvas_w)
        live_row = self._build_live_row(canvas_w)
        debug_row = self._build_debug_row(canvas_w)

        canvas = np.vstack([header, live_row, debug_row])

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))
            if self._tick_count % 30 == 1:
                self.get_logger().info(f"[ws_viz] published {canvas.shape} bgr8")
        except Exception as exc:
            self.get_logger().error(f"[ws_viz] publish failed: {exc}")

        if self.show_window:
            cv2.imshow(self.WINDOW_NAME, canvas)
            cv2.waitKey(1)

    # --- Cleanup -------------------------------------------------------------

    def destroy_node(self) -> None:
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkstationVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
