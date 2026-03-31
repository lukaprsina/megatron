"""Face detector node for Megatron Task 1.

Detects face posters using YOLOv8, projects them into 3-D via a 32FC1 depth
image + pinhole model, computes SVD-based surface normals, and tracks
detections with an inverse-distance-weighted system.

Publishes confirmed face poses (with surface normal encoded in orientation)
on ``/detected_faces`` and RViz markers on ``/face_markers``.
"""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

import message_filters

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf2_ros

from ultralytics import YOLO

from megatron.perception_utils import (
    DepthCameraGeometry,
    IncrementalTrackManager,
    compute_robust_surface,
    transform_point,
    transform_vector,
    normal_to_yaw,
    yaw_to_quaternion,
)


class FaceDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('face_detector')

        # Parameters
        self.declare_parameter('device', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('min_inference_interval', 0.2)
        self.declare_parameter('roi_shrink', 0.30)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.confirmation_count = self.get_parameter('confirmation_count').get_parameter_value().integer_value
        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value
        self.min_inference_interval = self.get_parameter('min_inference_interval').get_parameter_value().double_value
        self.roi_shrink = self.get_parameter('roi_shrink').get_parameter_value().double_value

        # YOLO model
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        # Depth camera geometry (pinhole model)
        self.cam_geom = DepthCameraGeometry()
        self.cam_geom.initialise(self)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tracking
        self.tracker = IncrementalTrackManager(
            dedup_distance=self.dedup_distance,
            confirmation_count=self.confirmation_count,
        )

        # Synchronized subscribers (RGB + depth)
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/oakd/rgb/preview/depth',
            qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Publishers
        self.face_pub = self.create_publisher(PoseStamped, '/detected_faces', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/face_markers',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.image_pub = self.create_publisher(Image, '/face_detections_image', 10)

        # Rate limiting
        self.last_inference_time = self.get_clock().now()

        self.get_logger().info('Face detector initialized.')

    # ------------------------------------------------------------------
    # Synchronized callback
    # ------------------------------------------------------------------

    def sync_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        # Rate limit
        now = self.get_clock().now()
        if (now - self.last_inference_time).nanoseconds < self.min_inference_interval * 1e9:
            return
        self.last_inference_time = now

        if not self.cam_geom.ready:
            return

        # Decode images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        h, w = cv_image.shape[:2]

        # Run YOLO
        results = self.model.predict(
            cv_image, imgsz=(256, 320), show=False, verbose=False,
            classes=[0], device=self.device or None,
            conf=self.confidence_threshold)

        # TF lookup (camera → map)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', depth_msg.header.frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            # Still publish the annotated image below
            transform = None

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                if box.xyxy is None or len(box.xyxy) == 0:
                    continue
                bbox = box.xyxy[0]
                x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

                # Draw bounding box on debug image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                if transform is None:
                    continue

                # Shrink ROI toward center to avoid background pixels
                bw, bh = x2 - x1, y2 - y1
                cx_px, cy_px = (x1 + x2) // 2, (y1 + y2) // 2
                shrink = self.roi_shrink
                sx1 = max(0, int(cx_px - bw * (0.5 - shrink / 2)))
                sy1 = max(0, int(cy_px - bh * (0.5 - shrink / 2)))
                sx2 = min(w, int(cx_px + bw * (0.5 - shrink / 2)))
                sy2 = min(h, int(cy_px + bh * (0.5 - shrink / 2)))

                if sx2 <= sx1 or sy2 <= sy1:
                    continue

                # Build boolean mask for the shrunk ROI
                mask = np.zeros((h, w), dtype=np.uint8)
                mask[sy1:sy2, sx1:sx2] = 255

                # Extract 3-D points from depth image via pinhole model
                pts_3d = self.cam_geom.extract_3d_points(mask, depth_image)
                if pts_3d.shape[0] < 10:
                    continue

                # SVD-based robust surface estimation
                result = compute_robust_surface(pts_3d)
                if result is None:
                    continue
                centroid_cam, normal_cam = result

                # Sanity check: object must be in front of camera
                if centroid_cam[2] < 0.3 or centroid_cam[2] > 5.0:
                    continue

                cam_dist = float(centroid_cam[2])

                # Transform to map frame
                map_pos = transform_point(centroid_cam, transform)
                map_normal = transform_vector(normal_cam, transform)

                self.get_logger().debug(
                    f'Face 3D: cam_pos={centroid_cam} cam_normal={normal_cam} '
                    f'→ map_pos={map_pos} map_normal={map_normal}')

                # Feed into tracker
                track_id, newly_confirmed = self.tracker.add_observation(
                    map_pos, map_normal, cam_dist,
                    rgb_msg.header.stamp, label='face')

                if newly_confirmed:
                    est = self.tracker.get_best_estimate(track_id)
                    self._publish_face(est, rgb_msg.header.stamp)
                    self.tracker.mark_reported(track_id)

                # Draw center dot
                cv2.circle(cv_image, (cx_px, cy_px), 4, (0, 255, 0), -1)

        # Also publish any tracks that got confirmed but not yet reported
        # (can happen if confirmation triggers from a later observation)
        for tid, est in self.tracker.get_confirmed_unreported():
            self._publish_face(est, rgb_msg.header.stamp)
            self.tracker.mark_reported(tid)

        # Always publish the annotated image
        self._publish_annotated(cv_image)
        # Always publish markers (so RViz stays current)
        self._publish_markers()

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_face(self, estimate: dict, stamp) -> None:
        face_id = sum(1 for t in self.tracker.tracks if t.reported or t.confirmed)
        pos = estimate['map_pos']
        normal = estimate['normal']

        self.get_logger().info(
            f'Face #{face_id} confirmed at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])

        # Encode surface normal as orientation (yaw facing the surface)
        yaw = normal_to_yaw(normal[:2])
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.face_pub.publish(pose)

    def _publish_annotated(self, image: np.ndarray) -> None:
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        except CvBridgeError:
            pass

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        markers: list[Marker] = []
        stamp = self.get_clock().now().to_msg()

        idx = 0
        for hyp in self.tracker.tracks:
            if not hyp.confirmed or hyp.rejected:
                continue
            est = hyp.weighted_estimate()
            pos = est['map_pos']

            # Sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'faces'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            markers.append(m)

            # Text
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = stamp
            t.ns = 'face_labels'
            t.id = idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pos[0])
            t.pose.position.y = float(pos[1])
            t.pose.position.z = float(pos[2]) + 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'Face {idx + 1}'
            markers.append(t)

            idx += 1

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
