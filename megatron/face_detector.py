"""YOLO-based face detector with depth image projection and SVD surface fitting.

Subscribes to synced RGB + depth images via message_filters, runs YOLOv8
inference, projects face ROIs to 3D via pinhole model, fits surface normals,
and publishes confirmed detections as PoseStamped (with normal as orientation).
"""

import os
from typing import cast

import cv2
import face_recognition
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
from megatron.tile_anomaly import quad_from_contour, warp_tile

# Side length of the canonical fronto-parallel square that the detected
# picture quad gets warped to before recognition.
RECOGNITION_SIZE = 200

# Debug dump of every face detection, for inspecting why crops are tiny.
DEBUG_DETECTED_DIR = "/home/luka/coding/dis/debug_faces_detected"
DEBUG_CROPPED_DIR = "/home/luka/coding/dis/debug_faces_cropped"

# How far past the YOLO box to look for the picture's actual edge. The
# picture frame is pasted flat on the wall, so its true boundary often
# sits outside YOLO's (loose) face box on one side and inside it on
# another — pad generously and let the contour search find the edge.
PLANE_PAD_FRAC = 0.6


def _detect_picture_quad(
    frame: np.ndarray, x1: int, y1: int, x2: int, y2: int
) -> tuple[np.ndarray | None, tuple[int, int, int, int]]:
    """Find the 4 corners of the picture billboard around a YOLO face box.

    Same recipe as task2_controller._detect_tile: Otsu-threshold the region
    to separate the (textured, brighter) picture from the (flat) wall, then
    pick the contour that looks like a quad. Returns corners in full-frame
    pixel coordinates, or None if nothing quad-like was found, plus the
    padded crop bounds (px1, py1, px2, py2) used for the search.
    """
    h, w = frame.shape[:2]
    bw, bh = x2 - x1, y2 - y1
    pad_x, pad_y = int(bw * PLANE_PAD_FRAC), int(bh * PLANE_PAD_FRAC)
    px1, py1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
    px2, py2 = min(w, x2 + pad_x), min(h, y2 + pad_y)
    if px2 <= px1 or py2 <= py1:
        return None, (px1, py1, px2, py2)

    crop = cv2.cvtColor(frame[py1:py2, px1:px2], cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cy, cx = crop.shape[0] // 2, crop.shape[1] // 2
    if thresh[cy, cx] == 0:
        thresh = 255 - thresh
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    crop_area = crop.shape[0] * crop.shape[1]
    crop_center = np.array([crop.shape[1] / 2, crop.shape[0] / 2])

    best = None
    best_dist = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < crop_area * 0.15 or area > crop_area * 0.95:
            continue
        box = cv2.boxPoints(cv2.minAreaRect(contour))
        bw_, bh_ = (
            float(np.linalg.norm(box[0] - box[1])),
            float(np.linalg.norm(box[1] - box[2])),
        )
        aspect = max(bw_, bh_) / max(min(bw_, bh_), 1.0)
        if aspect > 1.8:
            continue
        dist = float(np.linalg.norm(box.mean(axis=0) - crop_center))
        if best is None or dist < best_dist:
            best, best_dist = contour, dist

    if best is None:
        return None, (px1, py1, px2, py2)

    quad = quad_from_contour(best)
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(best)).astype(np.float32)
    quad = quad + np.array([px1, py1], dtype=np.float32)
    return quad, (px1, py1, px2, py2)


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
        self.declare_parameter("recognition_tolerance", 0.55)
        self.declare_parameter("recognition_margin", 0.08)
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
        self.recognition_tolerance = cast(
            float, self.get_parameter("recognition_tolerance").value
        )
        self.recognition_margin = cast(
            float, self.get_parameter("recognition_margin").value
        )
        self.personnel_dir = cast(str, self.get_parameter("personnel_dir").value)

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-face.pt")

        # Face recognition bank
        self.known_encodings = []
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
                img = face_recognition.load_image_file(filepath)
                encodings = face_recognition.face_encodings(img)
                if encodings:
                    self.known_encodings.append(encodings[0])
                    self.known_names.append(name)
                    self.get_logger().info(f"Loaded personnel: <{name}>")
                else:
                    self.get_logger().warn(f"No face found in {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to load {filename}: {e}")

    def _recognize_face(self, face_crop) -> str:
        """Run face recognition on a BGR image crop. Returns 'Unknown' or name."""
        if face_crop.size == 0 or not self.known_encodings:
            return "Unknown"

        try:
            crop_h, crop_w = face_crop.shape[:2]
            crop_rgb = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)

            # The crop is now a clean warped picture (or, on fallback, a raw
            # YOLO box) — try dlib's own face detector first for proper
            # landmark alignment; only fall back to "whole crop is the
            # face" if it can't find anything (e.g. on a degraded fallback
            # crop).
            face_locs = face_recognition.face_locations(crop_rgb)
            if not face_locs:
                face_locs = [(0, crop_w, crop_h, 0)]

            encodings = face_recognition.face_encodings(
                crop_rgb, known_face_locations=face_locs
            )
            if not encodings:
                self.get_logger().info(
                    f"Recognition: crop {crop_w}x{crop_h} -> no encoding",
                    throttle_duration_sec=1.0,
                )
                return "Unknown"

            distances = face_recognition.face_distance(
                self.known_encodings, encodings[0]
            )
            order = np.argsort(distances)
            best_idx = order[0]
            best_dist = float(distances[best_idx])
            second_dist = float(distances[order[1]]) if len(order) > 1 else None

            name = "Unknown"
            if best_dist < self.recognition_tolerance and (
                second_dist is None
                or second_dist - best_dist >= self.recognition_margin
            ):
                name = self.known_names[best_idx]

            self.get_logger().info(
                f"Recognition: crop {crop_w}x{crop_h} -> "
                + ", ".join(
                    f"{n}:{d:.3f}"
                    for n, d in zip(self.known_names, distances, strict=True)
                )
                + f" => {name}",
                throttle_duration_sec=1.0,
            )
            return name
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

                # Find the picture's true rectangular boundary (the YOLO
                # box is just a rough position) and warp it to a canonical
                # fronto-parallel square — gives recognition a clean, full,
                # correctly-scaled view instead of a YOLO-box crop.
                quad, (px1, py1, px2, py2) = _detect_picture_quad(
                    cv_image, x1, y1, x2, y2
                )
                if quad is not None:
                    # Warp to the quad's own native resolution, capped at
                    # RECOGNITION_SIZE — upsampling small/far detections up
                    # to a fixed 200x200 introduces enough blur to break
                    # dlib's encoder (confirmed: every 200x200 upsampled
                    # crop in testing came back Unknown, while native-size
                    # raw-box fallbacks recognized correctly).
                    qw, qh = cv2.boundingRect(quad.astype(np.int32))[2:4]
                    warp_size = min(RECOGNITION_SIZE, max(qw, qh))
                    # Small positive inset: the detected quad tends to run
                    # slightly past the picture into the wall (visible as a
                    # blue-grey margin in debug_faces_cropped), so shrink
                    # corners inward a bit before warping.
                    recognition_crop = warp_tile(
                        cv_image, quad, size=warp_size, border_inset=0.08
                    )
                else:
                    recognition_crop = cv_image[y1:y2, x1:x2]

                # Debug dump: full frame with raw YOLO box, padded search
                # region, and detected quad drawn; plus the recognition
                # input (warped picture, or the raw box crop on fallback).
                self.debug_save_counter += 1
                detected_frame = cv_image.copy()
                cv2.rectangle(detected_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.rectangle(detected_frame, (px1, py1), (px2, py2), (255, 0, 0), 1)
                if quad is not None:
                    cv2.polylines(
                        detected_frame,
                        [quad.astype(np.int32)],
                        isClosed=True,
                        color=(0, 255, 0),
                        thickness=1,
                    )
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
                    recognition_crop,
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

                # Face Recognition: warped picture plane (or raw box on fallback)
                person_name = self._recognize_face(recognition_crop)

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
