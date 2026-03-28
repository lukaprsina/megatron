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


class PerceptionVisualizer(Node):
    """Compose detector outputs into a single demo-friendly visualization."""

    def __init__(self) -> None:
        super().__init__('perception_visualizer')

        self.declare_parameter('show_window', False)
        self.declare_parameter('publish_combined_image', True)
        self.declare_parameter('refresh_rate', 10.0)
        self.declare_parameter('panel_height', 360)
        self.declare_parameter('window_name', 'Megatron Perception')

        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.publish_combined_image = self.get_parameter('publish_combined_image').get_parameter_value().bool_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().double_value
        self.panel_height = self.get_parameter('panel_height').get_parameter_value().integer_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value

        if self.refresh_rate <= 0.0:
            self.refresh_rate = 10.0
        if self.panel_height <= 0:
            self.panel_height = 360

        self.bridge = CvBridge()

        self.face_image: Optional[np.ndarray] = None
        self.ring_image: Optional[np.ndarray] = None
        self.face_count = 0
        self.ring_count = 0
        self.last_ring_color = '—'
        self.mission_status = 'WAITING_FOR_NAV2'

        self.create_subscription(
            Image, '/face_detections_image', self._face_image_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/ring_detections_image', self._ring_image_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/detected_faces', self._face_pose_callback, 10)
        self.create_subscription(PoseStamped, '/detected_rings', self._ring_pose_callback, 10)
        self.create_subscription(String, '/detected_ring_color', self._ring_color_callback, 10)
        self.create_subscription(String, '/mission_status', self._mission_status_callback, 10)

        self.image_pub = self.create_publisher(Image, '/task1_visualization_image', 10)

        self.timer = self.create_timer(1.0 / self.refresh_rate, self._publish_visualization)

        self.get_logger().info('Perception visualizer initialized.')

    def _face_image_callback(self, msg: Image) -> None:
        self.face_image = self._msg_to_bgr(msg)

    def _ring_image_callback(self, msg: Image) -> None:
        self.ring_image = self._msg_to_bgr(msg)

    def _face_pose_callback(self, _: PoseStamped) -> None:
        self.face_count += 1

    def _ring_pose_callback(self, _: PoseStamped) -> None:
        self.ring_count += 1

    def _ring_color_callback(self, msg: String) -> None:
        self.last_ring_color = msg.data or 'unknown'

    def _mission_status_callback(self, msg: String) -> None:
        self.mission_status = msg.data or 'unknown'

    def _msg_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return None
        return np.ascontiguousarray(image)

    def _publish_visualization(self) -> None:
        canvas = self._build_canvas()

        if self.publish_combined_image:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8'))
            except CvBridgeError as exc:
                self.get_logger().warn(f'Failed to publish visualization image: {exc}')

        if self.show_window:
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)

    def _build_canvas(self) -> np.ndarray:
        left = self._prepare_panel(self.face_image, 'Faces', (60, 80, 220))
        right = self._prepare_panel(self.ring_image, 'Rings', (60, 180, 75))

        body = np.hstack([left, right])
        header = np.full((88, body.shape[1], 3), 24, dtype=np.uint8)

        info_text = f'Faces: {self.face_count}   Rings: {self.ring_count}   Last ring: {self.last_ring_color}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.62
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(info_text, font, font_scale, thickness)
        x = max(20, body.shape[1] - text_width - 20)
        y = 68
        
        cv2.putText(
            header,
            'Megatron Task 1 Perception',
            (20, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            (245, 245, 245),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            header,
            f'State: {self.mission_status}',
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (210, 210, 210),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            header,
            info_text,
            (x, y),
            font,
            font_scale,
            (210, 210, 210),
            thickness,
            cv2.LINE_AA,
        )

        return np.vstack([header, body])

    def _prepare_panel(self, image: Optional[np.ndarray], title: str, accent_bgr: tuple[int, int, int]) -> np.ndarray:
        panel = np.full((self.panel_height, 640, 3), 18, dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (panel.shape[1] - 1, panel.shape[0] - 1), (60, 60, 60), 1)
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 44), accent_bgr, -1)
        cv2.putText(panel, title, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        if image is None or image.size == 0:
            cv2.putText(
                panel,
                'Waiting for detector image...',
                (120, self.panel_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (180, 180, 180),
                2,
                cv2.LINE_AA,
            )
            return panel

        content_height = self.panel_height - 60
        scale = min((panel.shape[1] - 24) / image.shape[1], content_height / image.shape[0])
        resized = cv2.resize(image, (max(1, int(image.shape[1] * scale)), max(1, int(image.shape[0] * scale))))

        y = 52 + max(0, (content_height - resized.shape[0]) // 2)
        x = max(0, (panel.shape[1] - resized.shape[1]) // 2)
        panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        return panel

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
