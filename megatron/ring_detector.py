import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros

# HSV ranges for ring colors: (lower, upper)
# Each color has a name, HSV lower bound, HSV upper bound, and an RGB display color
COLOR_RANGES = {
    'red':    {'lower1': (0, 100, 100),   'upper1': (10, 255, 255),
               'lower2': (160, 100, 100), 'upper2': (180, 255, 255),
               'rgb': (1.0, 0.0, 0.0)},
    'green':  {'lower1': (35, 80, 80),    'upper1': (85, 255, 255),
               'rgb': (0.0, 1.0, 0.0)},
    'blue':   {'lower1': (90, 80, 80),    'upper1': (130, 255, 255),
               'rgb': (0.0, 0.0, 1.0)},
    'yellow': {'lower1': (20, 100, 100),  'upper1': (35, 255, 255),
               'rgb': (1.0, 1.0, 0.0)},
    'black':  {'lower1': (0, 0, 0),       'upper1': (180, 255, 50),
               'rgb': (0.2, 0.2, 0.2)},
}


class RingDetectorNode(Node):

    def __init__(self):
        super().__init__('ring_detector')

        self.declare_parameter('min_area', 500)
        self.declare_parameter('min_circularity', 0.5)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('dedup_distance', 0.5)

        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.min_circularity = self.get_parameter('min_circularity').get_parameter_value().double_value
        self.confirmation_count = self.get_parameter('confirmation_count').get_parameter_value().integer_value
        self.dedup_distance = self.get_parameter('dedup_distance').get_parameter_value().double_value

        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.rgb_image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        # Publishers
        self.ring_pub = self.create_publisher(PoseStamped, '/detected_rings', 10)
        self.color_pub = self.create_publisher(String, '/detected_ring_color', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/ring_markers',
            QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.image_pub = self.create_publisher(Image, '/ring_detections_image', 10)

        # State
        self.detections_px = []  # [(cx, cy, color_name)] from latest RGB frame
        self.candidates = []     # [{'pos': np.array, 'color': str, 'count': int}]
        self.confirmed = []      # [{'pos': np.array, 'color': str}]

        self.get_logger().info('Ring detector initialized.')

    def rgb_callback(self, data):
        self.detections_px = []

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        for color_name, ranges in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(ranges['lower1']), np.array(ranges['upper1']))
            # Red wraps around hue=0, so merge two ranges
            if 'lower2' in ranges:
                mask2 = cv2.inRange(hsv, np.array(ranges['lower2']), np.array(ranges['upper2']))
                mask = cv2.bitwise_or(mask, mask2)

            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Find contours with hierarchy (for ring hole detection)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            if hierarchy is None:
                continue

            for i, contour in enumerate(contours):
                # Only consider outer contours that have a child (inner hole = ring)
                if hierarchy[0][i][3] != -1:
                    continue  # Skip inner contours
                if hierarchy[0][i][2] == -1:
                    continue  # No child = no hole = not a ring

                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue

                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue

                circularity = 4.0 * math.pi * area / (perimeter * perimeter)
                if circularity < self.min_circularity:
                    continue

                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Draw detection
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cx, cy), 4, (0, 255, 0), -1)
                cv2.putText(cv_image, color_name, (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.detections_px.append((cx, cy, color_name))

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

        try:
            transform = self.tf_buffer.lookup_transform(
            'map', data.header.frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        for cx, cy, color_name in self.detections_px:
            if cy >= height or cx >= width:
                continue

            point = a[cy, cx, :]
            if np.isnan(point).any():
                continue

            map_point = self._transform_point(point, transform)
            if map_point is None:
                continue

            self._process_detection(map_point, color_name, data.header.stamp)

        self.detections_px = []

    def _transform_point(self, point, transform):
        """Transform a 3D point using a geometry_msgs Transform."""
        t = transform.transform.translation
        q = transform.transform.rotation
        px, py, pz = float(point[0]), float(point[1]), float(point[2])

        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        ux, uy, uz = qx, qy, qz
        cx1 = uy * pz - uz * py
        cy1 = uz * px - ux * pz
        cz1 = ux * py - uy * px
        cx2 = uy * cz1 - uz * cy1
        cy2 = uz * cx1 - ux * cz1
        cz2 = ux * cy1 - uy * cx1

        rx = px + 2.0 * (qw * cx1 + cx2)
        ry = py + 2.0 * (qw * cy1 + cy2)
        rz = pz + 2.0 * (qw * cz1 + cz2)

        return np.array([rx + t.x, ry + t.y, rz + t.z])

    def _process_detection(self, map_point, color_name, stamp):
        """Multi-frame confirmation and deduplication."""
        for conf in self.confirmed:
            if np.linalg.norm(map_point - conf['pos']) < self.dedup_distance:
                return

        matched = False
        for cand in self.candidates:
            if np.linalg.norm(map_point - cand['pos']) < self.dedup_distance:
                cand['count'] += 1
                cand['pos'] = (cand['pos'] * (cand['count'] - 1) + map_point) / cand['count']
                matched = True
                if cand['count'] >= self.confirmation_count:
                    self._confirm_ring(cand['pos'], cand['color'], stamp)
                    self.candidates.remove(cand)
                break

        if not matched:
            self.candidates.append({'pos': map_point.copy(), 'color': color_name, 'count': 1})

    def _confirm_ring(self, map_point, color_name, stamp):
        """Publish a confirmed ring detection."""
        self.confirmed.append({'pos': map_point, 'color': color_name})
        ring_id = len(self.confirmed)

        self.get_logger().info(
            f'Ring #{ring_id} ({color_name}) confirmed at '
            f'({map_point[0]:.2f}, {map_point[1]:.2f}, {map_point[2]:.2f})')

        # Publish PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(map_point[0])
        pose.pose.position.y = float(map_point[1])
        pose.pose.position.z = float(map_point[2])
        pose.pose.orientation.w = 1.0
        self.ring_pub.publish(pose)

        # Publish color
        color_msg = String()
        color_msg.data = color_name
        self.color_pub.publish(color_msg)

        # Publish markers
        self._publish_markers()

    def _publish_markers(self):
        """Publish all confirmed rings as a MarkerArray."""
        marker_array = MarkerArray()
        markers: list[Marker] = []
        for i, ring in enumerate(self.confirmed):
            pos = ring['pos']
            color = COLOR_RANGES.get(ring['color'], {'rgb': (1.0, 1.0, 1.0)})['rgb']

            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rings'
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
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            markers.append(marker)

            # Text marker
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'ring_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = float(pos[2]) + 0.2
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 1.0
            text.text = f'{ring["color"]} ring'
            text.lifetime.sec = 0
            markers.append(text)

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    print('Ring detection node starting.')
    rclpy.init(args=args)
    node = RingDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

