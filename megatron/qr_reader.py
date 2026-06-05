#!/usr/bin/env python3
"""Minimal ROS 2 Python node named qr_reader."""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QRReader(Node):
    def __init__(self):
        super().__init__("qr_reader")

        self.bridge = CvBridge()
        self.latest_image: Image | None = None
        self.debug_window_name = "QR Reader Debug"
        self.busy = False
        self.latest_cv_image = None
        self.robot_state = "None"

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

        self.debug_image_pub = self.create_publisher(Image, "/qr_reader/debug_image", 10)
        self.publish_task = self.create_publisher(String, "/qr_task", 10)

        cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
        self.create_timer(1, self._tick, )
        self.get_logger().info("qr_reader node started")
    
    def _tick(self):
        if self.robot_state == "INTERACT":
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
        
        msg = String()
        msg.data = "None"
        self.publish_task.publish(msg)
        self.get_logger().info(f"DEBUG published {msg.data}")

        self.busy = False
        self.latest_cv_image = cv_image
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))





def main(args=None):
    rclpy.init(args=args)
    node = QRReader()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
