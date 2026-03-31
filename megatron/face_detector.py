"""YOLO-based face detector with depth image projection and SVD surface fitting.

Subscribes to synced RGB + depth images via message_filters, runs YOLOv8
inference, projects face ROIs to 3D via pinhole model, fits surface normals,
and publishes confirmed detections as PoseStamped (with normal as orientation).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

import message_filters
import numpy as np
import cv2
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO

from megatron.perception_utils import (
    DepthCameraGeometry,
    IncrementalTrackManager,
    compute_robust_surface,
    normal_to_quaternion,
    transform_point_and_normal,
)


class FaceDetectorNode(Node):

    def __init__(self):
        super().__init__('face_detector')

        # Parameters
        self.declare_parameter('device', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('min_inference_period', 0.2)
        self.declare_parameter('roi_shrink', 0.3)

        self.device = self.get_parameter('device').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.confirmation_count = self.get_parameter('confirmation_count').value
        self.dedup_distance = self.get_parameter('dedup_distance').value
        self.min_inference_period = self.get_parameter('min_inference_period').value
        self.roi_shrink = self.get_parameter('roi_shrink').value

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Depth geometry (intrinsics loaded on first CameraInfo)
        self.geometry = DepthCameraGeometry()
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/oakd/rgb/preview/camera_info',
            self._camera_info_callback, 10)

        # Synced RGB + Depth via message_filters
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/depth',
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self._synced_callback)

        # Publishers
        self.face_pub = self.create_publisher(PoseStamped, '/detected_faces', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/face_markers', 10)
        self.image_pub = self.create_publisher(Image, '/face_detections_image', 10)

        # Tracker
        self.track_manager = IncrementalTrackManager(
            dedup_distance=self.dedup_distance,
            confirmation_count=self.confirmation_count)

        # Rate limiting
        self.last_inference_time = 0.0

        self.get_logger().info('Face detector initialized (depth image mode).')

    # ------------------------------------------------------------------
    # CameraInfo — grab intrinsics once, then ignore
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo):
        if self.geometry.ready:
            return
        self.geometry.update_intrinsics(msg)
        self.get_logger().info(
            f'Camera intrinsics loaded: fx={self.geometry.fx:.1f} '
            f'fy={self.geometry.fy:.1f} cx={self.geometry.cx:.1f} cy={self.geometry.cy:.1f}')

    # ------------------------------------------------------------------
    # Synced RGB + Depth callback
    # ------------------------------------------------------------------

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_inference_time < self.min_inference_period:
            return
        self.last_inference_time = now

        if not self.geometry.ready:
            return

        # Convert images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        h, w = cv_image.shape[:2]

        # Run YOLO
        results = self.model.predict(
            cv_image, imgsz=(256, 320), show=False, verbose=False,
            classes=[0], device=self.device, conf=self.confidence_threshold)

        # Get TF: camera optical frame → map
        frame_id = depth_msg.header.frame_id
        if not frame_id:
            frame_id = 'oakd_rgb_camera_optical_frame'
        try:
            tf_stamped = self.tf_buffer.lookup_transform('map', frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        # Process each detection
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                if box.xyxy is None or len(box.xyxy) == 0:
                    continue
                bbox = box.xyxy[0]
                x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

                # Draw bounding box on display image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                # Shrink ROI toward center to avoid background pixels
                bw, bh = x2 - x1, y2 - y1
                sx = int(bw * self.roi_shrink / 2)
                sy = int(bh * self.roi_shrink / 2)
                rx1 = max(0, x1 + sx)
                ry1 = max(0, y1 + sy)
                rx2 = min(w, x2 - sx)
                ry2 = min(h, y2 - sy)

                if rx2 <= rx1 or ry2 <= ry1:
                    continue

                # Create mask for the shrunk ROI
                mask = np.zeros((h, w), dtype=np.uint8)
                mask[ry1:ry2, rx1:rx2] = 255

                # Project to 3D
                points_3d = self.geometry.extract_3d_points(mask, depth_image)
                if len(points_3d) < 5:
                    continue

                # Fit surface
                result = compute_robust_surface(points_3d)
                if result is None:
                    continue
                centroid, normal = result

                # Transform to map frame
                map_point, map_normal = transform_point_and_normal(
                    centroid, normal, tf_stamped)

                cam_dist = float(np.linalg.norm(centroid))

                # Feed to tracker
                status, track = self.track_manager.add_observation(
                    map_point, map_normal, cam_dist,
                    rgb_msg.header.stamp)

                if status == 'confirmed':
                    self._publish_detection(track, rgb_msg.header.stamp)

                # Draw center point on display
                cx = (rx1 + rx2) // 2
                cy = (ry1 + ry2) // 2
                cv2.circle(cv_image, (cx, cy), 4, (0, 0, 255), -1)

        # Publish annotated image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError:
            pass

    # ------------------------------------------------------------------
    # Publish confirmed detection
    # ------------------------------------------------------------------

    def _publish_detection(self, track, stamp):
        pos, normal = self.track_manager.get_best_estimate(track)

        self.get_logger().info(
            f'Face #{track["id"]} confirmed at '
            f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        # PoseStamped: position + normal encoded as orientation
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])

        qx, qy, qz, qw = normal_to_quaternion(normal[:2])
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.face_pub.publish(pose)
        self._publish_markers()

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------

    def _publish_markers(self):
        marker_array = MarkerArray()
        markers = []
        for track in self.track_manager.get_confirmed_tracks():
            pos, _ = self.track_manager.get_best_estimate(track)
            i = track['id'] - 1

            # Sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'faces'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.lifetime.sec = 0
            markers.append(m)

            # Text label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'face_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pos[0])
            t.pose.position.y = float(pos[1])
            t.pose.position.z = float(pos[2]) + 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'Face {track["id"]}'
            t.lifetime.sec = 0
            markers.append(t)

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    print('Face detection node starting.')
    rclpy.init(args=args)
    node = FaceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
