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
          
    def decode_depth(self, msg: CompressedImage) -> None:
        depth_img = self._docode_compressed_depth(msg)
        if depth_img is None:
            return None
        # self.get_logger().info(f" type: {type(depth_img)}, {depth_img.shape}")
        try:
            # depth_img = self.bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')
            # h, w = depth_img.shape[:2]
            # cx, cy = w // 2, h // 2
            
            # if len(depth_img.shape) == 3:
            #     self.get_logger().info(f"Depth image shape: {len(depth_img.shape)} -> {depth_img.shape}")
            #     depth_img = depth_img[:, :, 0]
                
            # dist = float(depth_img[cy, cx])
                
            if depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
                vis_img = np.clip(depth_img / 5.0 * 255.0, 0, 255).astype(np.uint8)
            else:
                vis_img = np.clip(depth_img / 5000.0 * 255.0, 0, 255).astype(np.uint8)
            #    dist = dist / 1000.0
                
            vis_img_bgr = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)
            # cv2.circle(vis_img_bgr, (cx, cy), 5, (0, 0, 255), -1)
            # cv2.putText(vis_img_bgr, f"{dist:.2f}m", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # self.depth_image = vis_img_bgr
            # self.depth_image = self._to_bgr_compressed(msg)
            return vis_img_bgr
        except CvBridgeError as exc:
            self.get_logger().warn(f'Depth calculation failed: {exc}')
            return None
            # if not isinstance(depth, np.ndarray) or depth.ndim < 2:
            #     self.get_logger().warn('Compressed depth decode returned invalid image shape.')
            #     return
            self.depth_raw_image = depth_img
            # self.depth_vis_image = self._depth_to_vis(depth)
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

