"""YOLO-based face detector with depth image projection and SVD surface fitting.

Subscribes to synced RGB + depth images via message_filters, runs YOLOv8
inference, projects face ROIs to 3D via pinhole model, fits surface normals,
and publishes confirmed detections as PoseStamped (with normal as orientation).
"""

import os
from typing import cast

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray

from megatron.perception_utils import (
    IncrementalTrackManager,
    compute_robust_surface,
    extract_3d_points_from_pc2,
    normal_to_quaternion,
    transform_point_and_normal,
)


# Reference photos are 512x512 portraits; face crops from the 256x320
# YOLO input are often only tens of pixels wide. ORB's scale pyramid
# (8 levels, 1.2x each) only covers ~4.3x — far less than that gap — so
# matching raw-resolution crops against full-res references starves ORB
# of correspondences. Resizing both sides to the same fixed size keeps
# them within ORB's working range.
RECOGNITION_SIZE = 200

# Debug dump of every face detection, for inspecting why crops are tiny.
DEBUG_DETECTED_DIR = "/home/luka/coding/dis/debug_faces_detected"
DEBUG_CROPPED_DIR = "/home/luka/coding/dis/debug_faces_cropped"


def parse_personnel_filename(filename):
    """Parse 'firstname_he_him_job_title.png' -> name."""
    stem = os.path.splitext(filename)[0]
    parts = stem.split("_")
    name = parts[0].capitalize()
    return name


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__("face_detector")

        # Parameters
        self.declare_parameter("device", "")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("confirmation_count", 6)
        self.declare_parameter("dedup_distance", 1)
        self.declare_parameter("min_inference_period", 0.2)
        self.declare_parameter("roi_shrink", 0.3)
        self.declare_parameter("track_max_age", 30.0)
        self.declare_parameter("lateral_offset", 0.2)
        self.declare_parameter("recognition_min_inliers", 12)
        self.declare_parameter("recognition_ratio_threshold", 0.8)
        self.declare_parameter(
            "personnel_dir",
            "/home/iota/dis/src/vendor/teammate-project/src/task1/config/personnel",
        )

        self.device = cast(str, self.get_parameter("device").value)
        self.confidence_threshold = cast(
            float, self.get_parameter("confidence_threshold").value
        )
        self.confirmation_count = cast(
            int, self.get_parameter("confirmation_count").value
        )
        self.dedup_distance = cast(float, self.get_parameter("dedup_distance").value)
        self.min_inference_period = cast(
            float, self.get_parameter("min_inference_period").value
        )
        self.roi_shrink = cast(float, self.get_parameter("roi_shrink").value)
        self.track_max_age = cast(float, self.get_parameter("track_max_age").value)
        self.lateral_offset = cast(float, self.get_parameter("lateral_offset").value)
        self.recognition_min_inliers = cast(
            int, self.get_parameter("recognition_min_inliers").value
        )
        self.recognition_ratio_threshold = cast(
            float, self.get_parameter("recognition_ratio_threshold").value
        )
        self.personnel_dir = cast(str, self.get_parameter("personnel_dir").value)

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-face.pt")

        # Face recognition bank: ORB keypoints/descriptors per personnel photo.
        # The photos are pasted directly onto walls in-sim as flat billboards,
        # so matching against the exact reference texture (planar homography)
        # is far more reliable here than a deep face-embedding model trained
        # on real photographs of actual human faces.
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.known_keypoints = []
        self.known_descriptors = []
        self.known_names = []
        self._load_personnel()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Synced RGB + PointCloud2 via message_filters
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/oakd/rgb/preview/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.pc2_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            "/oakd/rgb/preview/depth/points",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.pc2_sub], queue_size=10, slop=0.15
        )
        self.sync.registerCallback(self._synced_callback)

        # Publishers
        self.face_pub = self.create_publisher(PoseStamped, "/detected_faces", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/face_markers", 10)
        self.image_pub = self.create_publisher(Image, "/face_detections_image", 10)

        # Tracker
        self.track_manager = IncrementalTrackManager(
            dedup_distance=self.dedup_distance,
            confirmation_count=self.confirmation_count,
            max_cam_dist=2.5,
        )

        # Rate limiting
        self.last_inference_time = 0.0

        # Debug image dump
        os.makedirs(DEBUG_DETECTED_DIR, exist_ok=True)
        os.makedirs(DEBUG_CROPPED_DIR, exist_ok=True)
        self.debug_save_counter = 0

        self.get_logger().info("Face detector initialized (PointCloud2 mode).")

    def _load_personnel(self):
        if not os.path.isdir(self.personnel_dir):
            self.get_logger().error(
                f"Personnel directory not found: {self.personnel_dir}"
            )
            return

        for filename in sorted(os.listdir(self.personnel_dir)):
            if not filename.lower().endswith(".png"):
                continue

            filepath = os.path.join(self.personnel_dir, filename)
            name = parse_personnel_filename(filename)

            try:
                img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    self.get_logger().error(f"Failed to read {filename}")
                    continue
                img = cv2.resize(
                    img, (RECOGNITION_SIZE, RECOGNITION_SIZE),
                    interpolation=cv2.INTER_AREA,
                )
                kp, desc = self.orb.detectAndCompute(img, None)
                if desc is None or len(kp) == 0:
                    self.get_logger().warn(f"No ORB features found in {filename}")
                    continue
                self.known_keypoints.append(kp)
                self.known_descriptors.append(desc)
                self.known_names.append(name)
                self.get_logger().info(
                    f"Loaded personnel: <{name}> ({len(kp)} ORB keypoints)"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load {filename}: {e}")

    def _recognize_face(self, face_crop) -> str:
        """Match a BGR image crop against the personnel wall photos via ORB +
        homography RANSAC. Returns 'Unknown' or name.

        The personnel photos are pasted as flat billboards in-sim, so the
        crop is a perspective-warped view of the exact same texture rather
        than a different photo of the same person — planar feature matching
        is the right tool, not a face-embedding model trained on real photos.
        """
        if face_crop.size == 0 or not self.known_descriptors:
            return "Unknown"

        try:
            crop_gray = cv2.cvtColor(face_crop, cv2.COLOR_BGR2GRAY)
            crop_h, crop_w = crop_gray.shape[:2]
            crop_gray = cv2.resize(
                crop_gray, (RECOGNITION_SIZE, RECOGNITION_SIZE),
                interpolation=cv2.INTER_CUBIC,
            )
            kp_crop, desc_crop = self.orb.detectAndCompute(crop_gray, None)
            if desc_crop is None or len(kp_crop) < 4:
                self.get_logger().info(
                    f"Recognition: crop {crop_w}x{crop_h} -> "
                    f"{0 if kp_crop is None else len(kp_crop)} ORB keypoints, "
                    "too few to match",
                    throttle_duration_sec=1.0,
                )
                return "Unknown"

            best_name = "Unknown"
            best_inliers = self.recognition_min_inliers - 1
            debug_scores = []

            for kp_ref, desc_ref, name in zip(
                self.known_keypoints,
                self.known_descriptors,
                self.known_names,
                strict=True,
            ):
                knn_matches = self.matcher.knnMatch(desc_crop, desc_ref, k=2)
                good = [
                    m
                    for m, n in (pair for pair in knn_matches if len(pair) == 2)
                    if m.distance < self.recognition_ratio_threshold * n.distance
                ]
                if len(good) < 4:
                    debug_scores.append(f"{name}:{len(good)}good/0in")
                    continue

                src_pts = np.float32([
                    kp_crop[m.queryIdx].pt for m in good
                ]).reshape(-1, 1, 2)
                dst_pts = np.float32([
                    kp_ref[m.trainIdx].pt for m in good
                ]).reshape(-1, 1, 2)
                _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                inliers = 0 if mask is None else int(mask.sum())
                debug_scores.append(f"{name}:{len(good)}good/{inliers}in")

                if inliers > best_inliers:
                    best_inliers = inliers
                    best_name = name

            self.get_logger().info(
                f"Recognition: crop {crop_w}x{crop_h}, {len(kp_crop)} kp -> "
                f"{', '.join(debug_scores)} => {best_name}",
                throttle_duration_sec=1.0,
            )
            return best_name
        except Exception as e:
            self.get_logger().error(f"Face recognition error: {e}")
            return "Unknown"

    # ------------------------------------------------------------------
    # Synced RGB + PointCloud2 callback
    # ------------------------------------------------------------------

    def _synced_callback(self, rgb_msg: Image, pc2_msg: PointCloud2):
        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_inference_time < self.min_inference_period:
            return
        self.last_inference_time = now

        # Convert RGB image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        h, w = cv_image.shape[:2]

        # Run YOLO
        results = self.model.predict(
            cv_image,
            imgsz=(256, 320),
            show=False,
            verbose=False,
            device=self.device,
            conf=self.confidence_threshold,
        )

        # Get TF: PC2 frame → map
        frame_id = pc2_msg.header.frame_id
        if not frame_id:
            frame_id = "oakd_rgb_camera_optical_frame"
        try:
            tf_stamped = self.tf_buffer.lookup_transform("map", frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
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
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)

                # Draw bounding box on display image
                # cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                # Shrink ROI toward center to avoid background pixels.
                # Used for the depth/point-cloud mask only — recognition
                # uses the full YOLO box below, since it's already a tight
                # face crop and shrinking it further cuts into the face.
                bw, bh = x2 - x1, y2 - y1
                sx = int(bw * self.roi_shrink / 2)
                sy = int(bh * self.roi_shrink / 2)
                rx1 = max(0, x1 + sx)
                ry1 = max(0, y1 + sy)
                rx2 = min(w, x2 - sx)
                ry2 = min(h, y2 - sy)

                if rx2 <= rx1 or ry2 <= ry1:
                    continue

                # Debug dump: full frame with both raw YOLO box and shrunk
                # recognition ROI drawn, plus the exact crop fed to ORB.
                self.debug_save_counter += 1
                detected_frame = cv_image.copy()
                cv2.rectangle(detected_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.rectangle(detected_frame, (rx1, ry1), (rx2, ry2), (0, 255, 0), 1)
                cv2.imwrite(
                    os.path.join(
                        DEBUG_DETECTED_DIR, f"{self.debug_save_counter:06d}.png"
                    ),
                    detected_frame,
                )
                cv2.imwrite(
                    os.path.join(
                        DEBUG_CROPPED_DIR, f"{self.debug_save_counter:06d}.png"
                    ),
                    cv_image[y1:y2, x1:x2],
                )

                # Create mask for the shrunk ROI
                mask = np.zeros((h, w), dtype=np.uint8)
                mask[ry1:ry2, rx1:rx2] = 255

                # Project to 3D via PointCloud2
                points_3d = extract_3d_points_from_pc2(mask, pc2_msg)
                if len(points_3d) < 5:
                    continue

                # Fit surface
                result = compute_robust_surface(points_3d)
                if result is None:
                    continue
                centroid, normal = result

                # Transform to map frame
                map_point, map_normal = transform_point_and_normal(
                    centroid, normal, tf_stamped
                )

                # Face Recognition: full YOLO box, not the shrunk depth ROI
                face_crop = cv_image[y1:y2, x1:x2]
                person_name = self._recognize_face(face_crop)

                cam_dist = float(np.linalg.norm(centroid))
                # Feed to tracker
                status, track = self.track_manager.add_observation(
                    map_point,
                    map_normal,
                    cam_dist,
                    rgb_msg.header.stamp,
                    label=person_name,
                )

                if track is None:
                    continue
                if status == "confirmed":
                    self._publish_detection(track, rgb_msg.header.stamp)
                    pos, _, _ = self.track_manager.get_best_estimate(track)
                    track["_last_published_pos"] = pos.copy()
                    track["_update_count_since_publish"] = 0
                elif status == "updated":
                    pos, _, _ = self.track_manager.get_best_estimate(track)
                    last_pos = track.get("_last_published_pos")
                    update_count = track.get("_update_count_since_publish", 0)
                    if (
                        last_pos is None
                        or np.linalg.norm(pos - last_pos) > 0.1
                        or update_count >= 5
                    ):
                        self.get_logger().info("approaching face update")
                        self._publish_detection(track, rgb_msg.header.stamp)

                        track["_last_published_pos"] = pos.copy()
                        track["_update_count_since_publish"] = 0
                    else:
                        track["_update_count_since_publish"] = update_count + 1

                # Draw center point on display
                cx = (rx1 + rx2) // 2
                cy = (ry1 + ry2) // 2
                cv2.circle(cv_image, (cx, cy), 4, (0, 0, 255), -1)

        # Publish annotated image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError:
            pass

    # ------------------------------------------------------------------
    # Publish confirmed detection
    # ------------------------------------------------------------------

    def _publish_detection(self, track, stamp):
        pos, normal, label = self.track_manager.get_best_estimate(track)
        label = "Unknown" if label is None else label
        id = track["id"]
        # Shift the published position to the left (relative to surface normal).
        # Normal points AWAY from the surface. Vector to the left is (ny, -nx).
        nx, ny = normal[0], normal[1]
        pos[0] += ny * self.lateral_offset
        pos[1] += -nx * self.lateral_offset

        self.get_logger().info(
            f"Face #{track['id']} ({label}) at "
            f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        )

        # PoseStamped: position + normal encoded as orientation
        pose = PoseStamped()
        pose.header.frame_id = f"map|{label}|{id}"
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
            pos, normal, label, cam_dist = self.track_manager.get_best_estimate_temp(
                track
            )

            # Shift markers to match the published (shifted) position
            nx, ny = normal[0], normal[1]
            pos[0] += ny * self.lateral_offset
            pos[1] += -nx * self.lateral_offset

            i = track["id"] - 1

            # Sphere
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "faces"
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
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = "face_labels"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pos[0]) + 0.3
            t.pose.position.y = float(pos[1])
            t.pose.position.z = float(pos[2]) + 0.2
            t.pose.orientation.w = 1.0

            t.scale.x = t.scale.y = t.scale.z = 0.12

            # 2. MAKE IT YELLOW: Red + Green light (Full opacity alpha)
            t.color.r = 0.0
            t.color.g = 1.0
            t.color.b = 0.0
            t.color.a = 1.0

            label = "Unknown" if label is None else label
            cam_dist = 0.000 if cam_dist is None else cam_dist
            t.text = f"{label} (ID: {track['id']}) dist: {cam_dist:.2f}"
            t.lifetime.sec = 0
            markers.append(t)

        marker_array.markers = markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    print("Face detection node starting.")
    rclpy.init(args=args)
    node = FaceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
