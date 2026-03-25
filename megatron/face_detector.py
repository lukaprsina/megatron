import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy import time

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros

from ultralytics import YOLO


class FaceDetectorNode(Node):

    def __init__(self):
        super().__init__('face_detector')

        self.declare_parameter('device', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.confirmation_count = self.get_parameter('confirmation_count').get_parameter_value().integer_value
        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # TF2 for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.rgb_image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        # Publishers
        self.face_pub = self.create_publisher(PoseStamped, '/detected_faces', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/face_markers',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        self.image_pub = self.create_publisher(Image, '/face_detections_image', 10)

        # State
        self.detections_px = []  # [(cx, cy)] from latest RGB frame
        self.candidates = []     # [{'pos': np.array([x,y,z]), 'count': int}]
        self.confirmed = []      # [np.array([x,y,z])]

        self.get_logger().info('Face detector initialized.')

    def rgb_callback(self, data):
        self.detections_px = []

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        res = self.model.predict(
            cv_image, imgsz=(256, 320), show=False, verbose=False,
            classes=[0], device=self.device, conf=self.confidence_threshold)

        for r in res:
            if r.boxes is None:
                continue
            for box in r.boxes:
                if box.xyxy is None or len(box.xyxy) == 0:
                    continue
                bbox = box.xyxy[0]
                x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(cv_image, (cx, cy), 4, (0, 0, 255), -1)
                self.detections_px.append((cx, cy))

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError:
            pass

    def pointcloud_callback(self, data):
        if not self.detections_px:
            return

        height = data.height
        width = data.width

        a = pc2.read_points_numpy(data, field_names=["x", "y", "z"])
        a = a.reshape((height, width, 3))

        # Get transform from camera frame to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', data.header.frame_id, time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        for cx, cy in self.detections_px:
            if cy >= height or cx >= width:
                continue

            point = a[cy, cx, :]
            if np.isnan(point).any():
                continue

            # Transform point to map frame
            map_point = self._transform_point(point, transform)
            if map_point is None:
                continue

            self._process_detection(map_point, data.header.stamp)

        self.detections_px = []

    def _transform_point(self, point, transform):
        """Transform a 3D point using a geometry_msgs Transform."""
        t = transform.transform.translation
        q = transform.transform.rotation

        # Quaternion rotation
        # q * p * q_conjugate (Hamilton product)
        px, py, pz = float(point[0]), float(point[1]), float(point[2])

        # Rotation via quaternion
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # Rotate vector by quaternion: v' = q * v * q^-1
        # Using the formula: v' = v + 2*q_w*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
        ux, uy, uz = qx, qy, qz
        # cross1 = q_xyz x v
        cx1 = uy * pz - uz * py
        cy1 = uz * px - ux * pz
        cz1 = ux * py - uy * px
        # cross2 = q_xyz x cross1
        cx2 = uy * cz1 - uz * cy1
        cy2 = uz * cx1 - ux * cz1
        cz2 = ux * cy1 - uy * cx1

        rx = px + 2.0 * (qw * cx1 + cx2)
        ry = py + 2.0 * (qw * cy1 + cy2)
        rz = pz + 2.0 * (qw * cz1 + cz2)

        return np.array([rx + t.x, ry + t.y, rz + t.z])

    def _process_detection(self, map_point, stamp):
        """Multi-frame confirmation and deduplication."""
        # Check against already confirmed faces
        for conf in self.confirmed:
            if np.linalg.norm(map_point - conf) < self.dedup_distance:
                return  # Already found this face

        # Check against candidates
        matched = False
        for cand in self.candidates:
            if np.linalg.norm(map_point - cand['pos']) < self.dedup_distance:
                cand['count'] += 1
                # Update position with running average
                cand['pos'] = (cand['pos'] * (cand['count'] - 1) + map_point) / cand['count']
                matched = True
                if cand['count'] >= self.confirmation_count:
                    self._confirm_face(cand['pos'], stamp)
                    self.candidates.remove(cand)
                break

        if not matched:
            self.candidates.append({'pos': map_point.copy(), 'count': 1})

    def _confirm_face(self, map_point, stamp):
        """Publish a confirmed face detection."""
        self.confirmed.append(map_point)
        face_id = len(self.confirmed)

        self.get_logger().info(
            f'Face #{face_id} confirmed at ({map_point[0]:.2f}, {map_point[1]:.2f}, {map_point[2]:.2f})')

        # Publish PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(map_point[0])
        pose.pose.position.y = float(map_point[1])
        pose.pose.position.z = float(map_point[2])
        pose.pose.orientation.w = 1.0
        self.face_pub.publish(pose)

        # Publish markers
        self._publish_markers()

    def _publish_markers(self):
        """Publish all confirmed faces as a MarkerArray."""
        marker_array = MarkerArray()
        markers: list[Marker] = []
        for i, pos in enumerate(self.confirmed):
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'faces'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            markers.append(marker)

            # Text marker
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'face_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = float(pos[2]) + 0.2
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'Face {i + 1}'
            text.lifetime.sec = 0
            markers.append(text)

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

