#!/usr/bin/env python3
"""
Cylinder detector.

Consumes visualization_msgs/Marker from cylinder_segmentation C++ (already in
map frame), clusters repeated sightings using the teammate's Cluster algorithm,
confirms barrels, and publishes PoseStamped on /detected_cylinders with
  frame_id = "map|{color}|{orientation}"

Also provides /spill_check (std_srvs/Trigger): counts OAK-D depth cloud points
in a ground-level Z-slice within 0.5 m XY of the most recently observed barrel
centroid. Returns success=True if count ≥ 4000 (spill detected).
"""

import colorsys
import math
from collections import Counter
from statistics import median

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_lib
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from megatron.perception_utils import quaternion_to_rotation_matrix

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS = 0.33  # m — vertical barrel merge radius
CLUSTER_RADIUS_H = 0.70  # m — horizontal barrels vary more with viewing angle
CONFIRM_THRESH = 10  # detections before confirming
COMPACT_MIN = 6     # compact_enough threshold — decoupled from CONFIRM_THRESH because
                    # compact_points only returns N inliers when N ≥ 60% of count, so
                    # compact_enough(10) would effectively require ~17 sightings
COMPACT_INLIER_RADIUS = 0.32
MAX_RAW_PTS = 300
SAME_COLOUR_SUPPRESS_RADIUS = 0.48
SAME_COLOUR_SUPPRESS_RADIUS_H = 1.5
MIN_MARK_DIST = 0.18  # m — physical overlap guard
MAX_DETECTION_DISTANCE = 2.5  # m — ignore markers further than this from the robot

SPILL_Z_LO = 0.005  # m above map ground
SPILL_Z_HI = 0.15  # m above map ground
SPILL_XY_RADIUS = 0.5  # m from barrel centroid XY
SPILL_COUNT_THRESH = 4000  # points to declare a spill

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

_INACTIVE_STATES = frozenset(("NAVIGATE_ROOM2_ENTRY", "FOLLOW_BLUE_LINE", "DONE"))


# ── Colour classification (adapted from teammate's cylinder_localizator) ───────


def _rgb_to_colour_name(r: float, g: float, b: float) -> str:
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    h_deg = h * 360.0
    if v < 0.15:
        return "black"
    if s < 0.25:
        return "unknown"
    if h_deg < 15 or h_deg >= 330:
        return "red"
    if 15 <= h_deg < 45 and v < 0.45:
        return "brown"
    if h_deg < 40:
        return "orange"
    if h_deg < 80:
        return "yellow"
    if h_deg < 165:
        return "green"
    if h_deg < 270:
        return "blue"
    if h_deg < 315:
        return "purple"
    return "unknown"


def _colours_compatible(a: str, b: str) -> bool:
    if a in (None, "unknown") or b in (None, "unknown"):
        return True
    return a == b


# ── Cluster (adapted from teammate's cylinder_localizator) ────────────────────
# Custom Cluster used instead of IncrementalTrackManager because barrel tracking
# requires orientation-dependent merge radii (vertical 0.33 m, horizontal 0.70 m)
# and colour/orientation voting — ITM has neither concept.


class Cluster:
    __slots__ = (
        "points",
        "centroid",
        "colour_votes",
        "orientation_votes",
        "confirmed",
        "suppressed",
        "cluster_id",
    )

    def __init__(self, x, y, z, colour, orientation, cluster_id):
        self.points = [(x, y, z)]
        self.centroid = (x, y, z)
        self.colour_votes = Counter({colour: 1})
        self.orientation_votes = Counter({orientation: 1})
        self.confirmed = False
        self.suppressed = False
        self.cluster_id = cluster_id

    def add(self, x, y, z, colour, orientation):
        if len(self.points) < MAX_RAW_PTS:
            self.points.append((x, y, z))
        n = len(self.points)
        cx, cy, cz = self.centroid
        self.centroid = (cx + (x - cx) / n, cy + (y - cy) / n, cz + (z - cz) / n)
        self.colour_votes[colour] += 1
        self.orientation_votes[orientation] += 1

    @property
    def count(self):
        return len(self.points)

    @property
    def best_colour(self):
        return self.colour_votes.most_common(1)[0][0]

    @property
    def best_orientation(self):
        return self.orientation_votes.most_common(1)[0][0]

    @property
    def compact_points(self):
        xs, ys = [p[0] for p in self.points], [p[1] for p in self.points]
        mx, my = median(xs), median(ys)
        compact = [
            p
            for p in self.points
            if math.sqrt((p[0] - mx) ** 2 + (p[1] - my) ** 2) <= COMPACT_INLIER_RADIUS
        ]
        return (
            compact
            if len(compact) >= max(3, int(0.6 * len(self.points)))
            else self.points
        )

    @property
    def robust_centroid(self):
        pts = self.compact_points
        return (
            median(p[0] for p in pts),
            median(p[1] for p in pts),
            median(p[2] for p in pts),
        )

    def compact_enough(self, min_count: int) -> bool:
        return len(self.compact_points) >= min_count

    def dist2d(self, x, y) -> float:
        pts = self.compact_points if len(self.points) >= CONFIRM_THRESH else self.points
        mx = median(p[0] for p in pts) if len(pts) >= 3 else self.centroid[0]
        my = median(p[1] for p in pts) if len(pts) >= 3 else self.centroid[1]
        return math.sqrt((mx - x) ** 2 + (my - y) ** 2)


# ── Node ──────────────────────────────────────────────────────────────────────


class CylinderDetector(Node):
    def __init__(self):
        super().__init__("cylinder_detector")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Marker, "cylinder_markers", self._marker_cb, SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self._pc2_cb, SENSOR_QOS
        )
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self.detected_pub = self.create_publisher(
            PoseStamped, "/detected_cylinders", 10
        )
        self.marker_array_pub = self.create_publisher(
            MarkerArray, "/detected_barrels_markers", 10
        )
        self.create_service(Trigger, "/spill_check", self._spill_check_cb)
        self.create_subscription(PointStamped, "/spill_target", self._spill_target_cb, 10)

        self.clusters: list[Cluster] = []
        self.cluster_id_counter: int = 0
        self._latest_pc2: PointCloud2 | None = None
        self._spill_target: np.ndarray | None = None
        self.robot_state: str = "INIT"

        self.create_timer(3.0, self._republish_confirmed)
        self.get_logger().info("CylinderDetector ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        self.robot_state = msg.data

    def _spill_target_cb(self, msg: PointStamped):
        self._spill_target = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _pc2_cb(self, msg: PointCloud2):
        self._latest_pc2 = msg

    def _marker_cb(self, msg: Marker):
        if self.robot_state in _INACTIVE_STATES:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            if math.sqrt((x - rx) ** 2 + (y - ry) ** 2) > MAX_DETECTION_DISTANCE:
                return
        except Exception:
            pass  # no TF yet — let the detection through

        colour = _rgb_to_colour_name(msg.color.r, msg.color.g, msg.color.b)
        orientation = msg.text if msg.text in ("vertical", "horizontal") else "vertical"
        radius = CLUSTER_RADIUS_H if orientation == "horizontal" else CLUSTER_RADIUS

        # Find best matching cluster by orientation + colour + 2D distance
        best, best_d = None, float("inf")
        for c in self.clusters:
            if c.best_orientation != orientation:
                continue
            if not _colours_compatible(c.best_colour, colour):
                continue
            d = c.dist2d(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None or best_d > radius: # new Cluster
            best = Cluster(x, y, z, colour, orientation, self.cluster_id_counter)
            self.cluster_id_counter += 1
            self.clusters.append(best)
        else:
            best.add(x, y, z, colour, orientation)

        if best.count >= CONFIRM_THRESH and best.compact_enough(COMPACT_MIN):
            just_confirmed = not best.confirmed
            if just_confirmed:
                best.confirmed = True
                # Check if an already-confirmed cluster nearby owns this region first.
                # This handles the race where both barrel ends reach CONFIRM_THRESH
                # before either suppresses the other.
                cx, cy, cz = best.robust_centroid
                suppress_r = (
                    SAME_COLOUR_SUPPRESS_RADIUS_H
                    if best.best_orientation == "horizontal"
                    else SAME_COLOUR_SUPPRESS_RADIUS
                )
                for c in self.clusters:
                    if c is best or not c.confirmed or c.suppressed:
                        continue
                    if c.best_orientation != best.best_orientation:
                        continue
                    if not _colours_compatible(c.best_colour, best.best_colour):
                        continue
                    if c.dist2d(cx, cy) < suppress_r:
                        best.suppressed = True
                        self.get_logger().info(
                            f"Barrel end suppressed (duplicate of cluster "
                            f"{c.cluster_id}): {best.best_colour}/"
                            f"{best.best_orientation} at ({cx:.2f}, {cy:.2f})"
                        )
                        break
                if not best.suppressed:
                    self._suppress_duplicates(best)
                    self._publish_marker_array()
            if best.suppressed:
                return
            cx, cy, cz = best.robust_centroid
            if just_confirmed:
                self.get_logger().info(
                    f"Barrel confirmed: {best.best_colour}/{best.best_orientation} "
                    f"at ({cx:.2f}, {cy:.2f})"
                )
            self._publish_pose(best)

    def _suppress_duplicates(self, new_cluster: Cluster):
        """Mark same-colour/orientation clusters within suppress radius as confirmed
        (silently) so they don't generate duplicate detections.
        """
        cx, cy, _ = new_cluster.robust_centroid
        orientation = new_cluster.best_orientation
        suppress_r = (
            SAME_COLOUR_SUPPRESS_RADIUS_H
            if orientation == "horizontal"
            else SAME_COLOUR_SUPPRESS_RADIUS
        )
        for c in self.clusters:
            if c is new_cluster or c.confirmed:
                continue
            if c.best_orientation != orientation:
                continue
            if not _colours_compatible(c.best_colour, new_cluster.best_colour):
                continue
            d = c.dist2d(cx, cy)
            if d < MIN_MARK_DIST or d < suppress_r:
                c.confirmed = True   # guard: prevents re-confirmation in _marker_cb
                c.suppressed = True  # guard: prevents republish in _republish_confirmed

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_pose(self, cluster: Cluster):
        cx, cy, cz = cluster.robust_centroid
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"map|{cluster.best_colour}|{cluster.best_orientation}"
        msg.pose.position.x = cx
        msg.pose.position.y = cy
        msg.pose.position.z = cz
        msg.pose.orientation.w = 1.0  # identity — axis yaw not encoded (see plan §J)
        self.detected_pub.publish(msg)

    def _create_marker(self, cluster: Cluster) -> Marker:
        cx, cy, cz = cluster.robust_centroid
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "barrels"
        marker.id = cluster.cluster_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz
        marker.pose.orientation.w = 1.0

        if cluster.best_orientation == "horizontal":
            marker.scale.x = 0.2
            marker.scale.y = 0.4
            marker.scale.z = 0.2
        else:
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.4

        color_map = {
            "red": (1.0, 0.0, 0.0),
            "green": (0.0, 1.0, 0.0),
            "blue": (0.0, 0.0, 1.0),
            "yellow": (1.0, 1.0, 0.0),
            "orange": (1.0, 0.5, 0.0),
            "purple": (0.5, 0.0, 0.5),
            "brown": (0.6, 0.3, 0.3),
            "black": (0.2, 0.2, 0.2),
        }
        r, g, b = color_map.get(cluster.best_colour, (0.7, 0.7, 0.7))
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 0.8
        return marker

    def _create_text_marker(self, cluster: Cluster) -> Marker:
        cx, cy, cz = cluster.robust_centroid
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "orientation_labels"
        marker.id = cluster.cluster_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz + 0.3  # Float above the barrel
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.2  # Text height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "H" if cluster.best_orientation == "horizontal" else "V"
        return marker

    def _publish_marker_array(self):
        markers: list = []
        for c in self.clusters:
            if c.confirmed and not c.suppressed:
                markers.append(self._create_marker(c))
                markers.append(self._create_text_marker(c))
        if markers:
            msg = MarkerArray()
            msg.markers = markers
            self.marker_array_pub.publish(msg)

    def _republish_confirmed(self):
        self._publish_marker_array()
        for c in self.clusters:
            if c.confirmed and not c.suppressed:
                self._publish_pose(c)

    # ── /spill_check service ──────────────────────────────────────────────────

    def _spill_check_cb(self, _request, response: Trigger.Response):
        centroid = self._spill_target
        pc2 = self._latest_pc2

        if centroid is None:
            response.success = False
            response.message = "no spill target set"
            return response
        if pc2 is None:
            response.success = False
            response.message = "no point cloud available"
            return response

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                pc2.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            response.success = False
            response.message = f"TF failed: {e}"
            return response

        R = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = tf.transform.translation
        T = np.array([t.x, t.y, t.z])

        # Read all PC2 points — unorganized iteration is fine for spill check
        try:
            raw = pc2_lib.read_points_numpy(pc2, field_names=["x", "y", "z"])
        except Exception as e:
            response.success = False
            response.message = f"PC2 read failed: {e}"
            return response

        if raw.dtype.names:  # structured array path
            pts_cam = np.column_stack(
                [
                    raw["x"].ravel().astype(np.float64),
                    raw["y"].ravel().astype(np.float64),
                    raw["z"].ravel().astype(np.float64),
                ]
            )
        else:
            pts_cam = raw.reshape(-1, 3).astype(np.float64)

        finite = np.isfinite(pts_cam).all(axis=1)
        pts_cam = pts_cam[finite]
        if len(pts_cam) == 0:
            response.success = False
            response.message = "no finite points in cloud"
            return response

        # Batch-transform camera → map
        pts_map = (R @ pts_cam.T).T + T

        bx, by = centroid[0], centroid[1]
        z_ok = (pts_map[:, 2] >= SPILL_Z_LO) & (pts_map[:, 2] <= SPILL_Z_HI)
        xy_ok = (
            np.sqrt((pts_map[:, 0] - bx) ** 2 + (pts_map[:, 1] - by) ** 2)
            < SPILL_XY_RADIUS
        )
        count = int(np.sum(z_ok & xy_ok))

        leaking = count >= SPILL_COUNT_THRESH
        response.success = True  # call succeeded; check message for result
        response.message = f"{'LEAK' if leaking else 'OK'} count={count} threshold={SPILL_COUNT_THRESH}"
        self.get_logger().info(f"SpillCheck: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CylinderDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
