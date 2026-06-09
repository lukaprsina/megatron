#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

BLUE_LO = np.array([90, 80, 40])
BLUE_HI = np.array([140, 255, 255])

STEER_KP = 0.8
EMA_ALPHA = 0.7
MAX_ANGULAR = 0.6
FORWARD_SPEED = 0.12
IMAGE_CENTER_X = 160.0


class BlueLineFollower(Node):
    def __init__(self):
        super().__init__("blue_line_follower")

        self.active = False
        self.bridge = CvBridge()
        self._latest_img: np.ndarray | None = None
        self._smoothed_angular = 0.0

        self.create_subscription(String, "/robot_state", self._state_cb, 10)
        self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._image_cb, 10
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)

        self.create_timer(0.05, self._tick)
        self.get_logger().info("blue_line_follower ready (mock)")

    def _state_cb(self, msg: String):
        self.active = msg.data == "FOLLOW_BLUE_LINE"
        if self.active:
            self.get_logger().info("FOLLOW_BLUE_LINE activated")

    def _image_cb(self, msg: Image):
        if not self.active:
            return
        try:
            self._latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _tick(self):
        if not self.active:
            return

        twist = Twist()

        if self._latest_img is None:
            self._cmd_pub.publish(twist)
            return

        hsv = cv2.cvtColor(self._latest_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

        moments = cv2.moments(mask)
        if moments["m00"] < 50:
            twist.angular.z = self._smoothed_angular
            self._cmd_pub.publish(twist)
            return

        cx = int(moments["m10"] / moments["m00"])
        error = IMAGE_CENTER_X - cx
        raw = STEER_KP * error / IMAGE_CENTER_X
        raw = np.clip(raw, -MAX_ANGULAR, MAX_ANGULAR)

        self._smoothed_angular = (
            EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * self._smoothed_angular
        )

        twist.linear.x = FORWARD_SPEED
        twist.angular.z = self._smoothed_angular
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BlueLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
