"""Project a (possibly unorganized) PointCloud2 onto an RGB image and draw dots.

This is a debugging utility to visualize 3D points even when PointCloud2 is
unorganized (height == 1) by projecting XYZ points with camera intrinsics
from CameraInfo.

Typical usage with your existing remaps:
- image: /rgb/image_raw (remap to /gemini/color/image_raw)
- camera_info: /rgb/camera_info (remap to /gemini/color/camera_info)
- points: /depth/points (remap to /gemini/depth_registered/points)

Publishes:
- /pc2_overlay_image (sensor_msgs/Image)

Assumptions:
- Point cloud points are expressed in the same optical frame as the camera
  intrinsics (e.g., registered depth points in the color optical frame).
"""

from __future__ import annotations

import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class PC2ImageOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('pc2_image_overlay')

        # Topics
        self.declare_parameter('image_topic', '/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')
        self.declare_parameter('points_topic', '/depth/points')
        self.declare_parameter('output_topic', '/pc2_overlay_image')

        # Sync
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop', 0.5)

        # Rendering / filtering
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('stride', 4)  # take every Nth point after filtering
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('point_radius', 1)
        self.declare_parameter('colorize_by_depth', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.points_topic = str(self.get_parameter('points_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        self.sync_slop = float(self.get_parameter('sync_slop').value)

        self.max_range = float(self.get_parameter('max_range').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.max_points = max(0, int(self.get_parameter('max_points').value))
        self.point_radius = max(1, int(self.get_parameter('point_radius').value))
        self.colorize_by_depth = bool(self.get_parameter('colorize_by_depth').value)

        self.bridge = CvBridge()

        # Cached camera intrinsics (from CameraInfo)
        self._intrinsics_ready = False
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._camera_info_frame: str | None = None

        # Subscribers
        # Cache CameraInfo separately (often not time-synced with images)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )

        # Sync Image + PointCloud2
        self.image_sub = message_filters.Subscriber(
            self, Image, self.image_topic, qos_profile=qos_profile_sensor_data
        )
        self.pc2_sub = message_filters.Subscriber(
            self, PointCloud2, self.points_topic, qos_profile=qos_profile_sensor_data
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pc2_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self._cb)

        self.pub = self.create_publisher(Image, self.output_topic, 10)

        # Debug: warn if no synced messages arrive
        self._last_cb_time = None
        self._no_data_timer = self.create_timer(2.0, self._check_no_data)

        self.get_logger().info(
            'PC2 image overlay initialized. '
            f'image={self.image_topic}, info={self.camera_info_topic}, points={self.points_topic}, '
            f'sync_slop={self.sync_slop:.3f}s, stride={self.stride}, max_points={self.max_points}'
        )

    def _camera_info_cb(self, info_msg: CameraInfo) -> None:
        k = info_msg.k
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            self.get_logger().warn(
                'CameraInfo intrinsics invalid (fx/fy ~= 0).',
                throttle_duration_sec=2.0,
            )
            return

        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy
        self._intrinsics_ready = True
        self._camera_info_frame = info_msg.header.frame_id or None

    def _cb(self, image_msg: Image, pc2_msg: PointCloud2) -> None:
        self._last_cb_time = self.get_clock().now()
        # Convert image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Image conversion failed: {e}', throttle_duration_sec=2.0)
            return

        h_img, w_img = cv_image.shape[:2]

        if not self._intrinsics_ready:
            self.get_logger().warn(
                f'Waiting for CameraInfo on {self.camera_info_topic}...',
                throttle_duration_sec=2.0,
            )
            self._publish_with_hud(cv_image, image_msg, 'Waiting for CameraInfo...')
            return

        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        # Read point cloud XYZ
        from sensor_msgs_py import point_cloud2 as pc2_lib

        try:
            pts = pc2_lib.read_points_numpy(pc2_msg, field_names=['x', 'y', 'z']).astype(np.float32)
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 read failed: {e}', throttle_duration_sec=2.0)
            return

        pts = pts.reshape((-1, 3))
        if pts.size == 0:
            self.get_logger().info('PointCloud2 empty.', throttle_duration_sec=2.0)
            self._publish_with_hud(cv_image, image_msg, 'PointCloud2 empty')
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        positive = z > 1e-6
        in_range = np.sqrt(x * x + y * y + z * z) < self.max_range
        valid = finite & positive & in_range
        pts = pts[valid]

        if pts.shape[0] == 0:
            self.get_logger().info('No valid points after filtering.', throttle_duration_sec=2.0)
            self._publish_with_hud(cv_image, image_msg, 'No valid points after filtering')
            return

        # Downsample
        pts = pts[:: self.stride]
        if self.max_points > 0 and pts.shape[0] > self.max_points:
            # deterministic decimation to avoid RNG variability
            step = int(math.ceil(pts.shape[0] / self.max_points))
            pts = pts[::step]

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        # Project
        u = (fx * x / z + cx).astype(np.int32)
        v = (fy * y / z + cy).astype(np.int32)

        inside = (u >= 0) & (u < w_img) & (v >= 0) & (v < h_img)
        u = u[inside]
        v = v[inside]
        z = z[inside]

        # Draw
        overlay = cv_image
        if self.colorize_by_depth:
            # map depth to color (near=red, far=blue)
            z_norm = np.clip((z - 0.3) / max(1e-6, (self.max_range - 0.3)), 0.0, 1.0)
            # OpenCV colormap expects 0..255 uint8
            cmap_in = (255.0 * (1.0 - z_norm)).astype(np.uint8)
            colors = cv2.applyColorMap(cmap_in, cv2.COLORMAP_JET)
            # colors may come back as (N, 1, 3); reshape to (N, 3)
            colors = colors.reshape((-1, 3))
            for (uu, vv, c) in zip(u, v, colors):
                b, g, r = int(c[0]), int(c[1]), int(c[2])
                cv2.circle(overlay, (int(uu), int(vv)), self.point_radius, (b, g, r), -1)
        else:
            for (uu, vv) in zip(u, v):
                cv2.circle(overlay, (int(uu), int(vv)), self.point_radius, (0, 255, 0), -1)

        # Add a small HUD
        self._publish_with_hud(
            overlay,
            image_msg,
            f'pc2 h={pc2_msg.height} w={pc2_msg.width} valid={len(u)}',
        )

    def _publish_with_hud(self, image: np.ndarray, image_msg: Image, text: str) -> None:
        cv2.putText(
            image,
            text,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        try:
            out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            out.header = image_msg.header
            self.pub.publish(out)
        except CvBridgeError as e:
            self.get_logger().warn(f'Publish failed: {e}', throttle_duration_sec=2.0)

    def _check_no_data(self) -> None:
        if self._last_cb_time is None:
            self.get_logger().warn(
                f'No image/pointcloud sync yet. '
                f'Waiting on image={self.image_topic}, points={self.points_topic}, '
                f'camera_info={self.camera_info_topic}',
                throttle_duration_sec=5.0,
            )
            return

        age = (self.get_clock().now() - self._last_cb_time).nanoseconds / 1e9
        if age > 5.0:
            self.get_logger().warn(
                f'No synced image/pointcloud for {age:.1f}s. '
                f'Check image={self.image_topic} and points={self.points_topic}',
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PC2ImageOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
