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

class Decoder():
    
    def __init__(self):
        self.bridge = CvBridge()

    def _docode_compressed_depth(self, msg: CompressedImage) -> Optional[np.ndarray]:
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
            return decoded
        except CvBridgeError:
            self.get_logger().error("1 -> Failed to decompress image")
            return None
          
    def decode_depth(self, msg: CompressedImage) -> Optional[np.ndarray]:
        return self._docode_compressed_depth(msg)
    def decode_img(self, msg: CompressedImage) -> Optional[np.ndarray] :
        # self.get_logger().info('Received Color image for visualization.')
        try:
            return self._to_bgr_compressed(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f'Color image conversion failed: {exc}')
            return None
    def _to_bgr_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            return np.ascontiguousarray(
                self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Compressed image conversion failed: {exc}')
            return None

