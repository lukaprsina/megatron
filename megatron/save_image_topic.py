#!/usr/bin/python3
"""Save a single frame from a ROS image topic to a file.

Usage:
    ros2 run megatron save_image_topic --ros-args -p topic:=/top_camera/rgb/preview/image_raw -p path:=/tmp/frame.png
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self):
        super().__init__("image_saver")
        self.declare_parameter("topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("path", "/tmp/frame.png")
        self.bridge = CvBridge()

        topic = self.get_parameter("topic").value
        if not topic:
            raise ValueError("Topic parameter is required.")

        path = self.get_parameter("path").value
        if not path:
            raise ValueError("Path parameter is required.")
        path = str(path)

        self.get_logger().info(f"Waiting for one frame on '{topic}' → '{path}' ...")

        self.sub = self.create_subscription(
            Image,
            topic,
            self._cb,
            10,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),  # type: ignore
        )
        self.timer = self.create_timer(10.0, self._timeout)

        self.path = path
        self.saved = False

    def _cb(self, msg: Image):
        if self.saved:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(self.path, cv_image)
            self.get_logger().info(
                f"Saved {cv_image.shape[1]}×{cv_image.shape[0]} to '{self.path}'"
            )
            self.saved = True
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")

    def _timeout(self):
        if not self.saved:
            self.get_logger().error("Timed out waiting for image.")
            rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(ImageSaver())


if __name__ == "__main__":
    main()
