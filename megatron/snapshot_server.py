#!/usr/bin/python3
"""Snapshot server — publish a filename to a trigger topic to save the latest camera frame.

Trigger topics (std_msgs/String — data field is the desired filename stem):
    /megatron/snapshot/top     → saves from /top_camera/rgb/preview/image_raw
    /megatron/snapshot/bottom  → saves from /oakd/rgb/preview/image_raw

If the data field is empty a timestamp stem is used automatically.
Images are written to <package_share>/assets/snapshots/<stem>.png.

Usage:
    ros2 run megatron save_image_topic
    ros2 topic pub --once /megatron/snapshot/top    std_msgs/msg/String '{data: ""}'
    ros2 topic pub --once /megatron/snapshot/bottom std_msgs/msg/String '{data: "barrel1_leak"}'
"""

from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

TOP_CAMERA_TOPIC = "/top_camera/rgb/preview/image_raw"
BOTTOM_CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"
SNAPSHOT_DIR = Path(get_package_share_directory("megatron")) / "assets" / "snapshots"


class SnapshotServer(Node):
    def __init__(self):
        super().__init__("snapshot_server")
        self.bridge = CvBridge()
        self._latest: dict[str, Image | None] = {"top": None, "bottom": None}

        self.create_subscription(Image, TOP_CAMERA_TOPIC, self._store_top, 1)
        self.create_subscription(Image, BOTTOM_CAMERA_TOPIC, self._store_bottom, 1)

        self.create_subscription(String, "/megatron/snapshot/top", self._trigger_top, 10)
        self.create_subscription(String, "/megatron/snapshot/bottom", self._trigger_bottom, 10)

        SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Snapshot server ready. Saving to '{SNAPSHOT_DIR}'\n"
            "  Trigger top:    ros2 topic pub --once /megatron/snapshot/top    std_msgs/msg/String '{data: \"\"}'\n"
            "  Trigger bottom: ros2 topic pub --once /megatron/snapshot/bottom std_msgs/msg/String '{data: \"barrel1_leak\"}'"
        )

    def _store_top(self, msg: Image) -> None:
        self._latest["top"] = msg

    def _store_bottom(self, msg: Image) -> None:
        self._latest["bottom"] = msg

    def _trigger_top(self, msg: String) -> None:
        self._save("top", msg.data)

    def _trigger_bottom(self, msg: String) -> None:
        self.get_logger().info(f"Triggered bottom save image: {msg}")
        self._save("bottom", msg.data)

    def _save(self, camera: str, stem: str) -> None:
        frame_msg = self._latest[camera]
        if frame_msg is None:
            self.get_logger().warning(f"No frame received yet from {camera} camera.")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(frame_msg, desired_encoding="bgr8")
            if not stem:
                stem = f"{camera}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
            path = SNAPSHOT_DIR / f"{stem}.png"
            cv2.imwrite(str(path), frame)
            self.get_logger().info(
                f"[{camera}] Saved {frame.shape[1]}×{frame.shape[0]} → '{path}'"
            )
        except Exception as e:
            self.get_logger().error(f"[{camera}] Failed to save snapshot: {e}")


def main():
    rclpy.init()
    rclpy.spin(SnapshotServer())


if __name__ == "__main__":
    main()
