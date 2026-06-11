#!/usr/bin/env python3
"""Minimal ROS 2 Python node named qr_reader."""

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QRReader(Node):
    def __init__(self):
        super().__init__("qr_reader")

        self.debug_window_name = "QR Reader Debug"

        self.bridge = CvBridge()
        self.latest_image: Image | None = None
        self.busy = False
        self.latest_cv_image = None
        self.detector = None

        self.robot_state = "None"

        self.load_detector()

        self.create_subscription(
            String,
            "/robot_state",
            self.robot_state_cb,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            "/oakd/rgb/preview/image_raw",
            self.image_callback,
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image, "/qr_reader/debug_image", 10
        )
        self.publish_task = self.create_publisher(String, "/qr_task", 10)

        cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
        self.create_timer(
            1,
            self._tick,
        )
        self.get_logger().info("qr_reader node started")

    def _tick(self):
        if self.robot_state in ("INTERACT", "FOLLOW_BLUE_LINE"):
            self.task_cb()

        if self.latest_cv_image is not None:
            cv2.imshow(self.debug_window_name, self.latest_cv_image)
            cv2.waitKey(1)

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def robot_state_cb(self, robot_state: String):
        self.robot_state = robot_state.data

    def task_cb(self):
        if self.busy is True:
            return

        self.busy = True
        current_image = self.latest_image
        if current_image is None:
            self.get_logger().warn("No image received yet for qr_reader debug view")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(current_image, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Failed to convert QR image: {exc}")
            return

        msg = self.handle_qr(cv_image)
        if msg is None:
            msg = String()
            msg.data = ""

        self.publish_task.publish(msg)
        self.get_logger().info(f"DEBUG published {msg.data}")

        self.busy = False
        self.latest_cv_image = cv_image
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        )

    def handle_qr(self, cv_image) -> String | None:
        if self.detector is None:
            return None

        texts, points = self.detector.detectAndDecode(cv_image)
        if len(texts) > 1:
            self.get_logger().warning(f"warn got more than one qr code: {len(texts)}")
        if texts:
            for text in texts:
                self.get_logger().info(f"QR Code Found: {text}")

                # Publish the decoded string to a separate topic!
                str_msg = String()
                str_msg.data = text
                return str_msg
        return None

    def load_detector(self):
        try:
            # Resolve model paths from workspace src/vendor/we-chat-qrcode.
            model_dir = (
                Path(__file__).resolve().parents[2] / "vendor" / "we-chat-qrcode"
            )
            detect_proto = model_dir / "detect.prototxt"
            detect_model = model_dir / "detect.caffemodel"
            sr_proto = model_dir / "sr.prototxt"
            sr_model = model_dir / "sr.caffemodel"

            self.detector = cv2.wechat_qrcode_WeChatQRCode(  # pyright: ignore[reportAttributeAccessIssue]
                str(detect_proto),
                str(detect_model),
                str(sr_proto),
                str(sr_model),
            )
            self.get_logger().info(f"Loaded WeChat QR models from: {model_dir}")
        except Exception as e:
            print(f"Error loading model files from {model_dir}: {e}")
            print(
                "Ensure detect/sr .prototxt and .caffemodel files exist in src/vendor/we-chat-qrcode."
            )
            exit()


def main(args=None):
    rclpy.init(args=args)
    node = QRReader()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
