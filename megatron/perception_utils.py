"""Shared perception utilities for Megatron Task 1.

Provides:
- DepthCameraGeometry: pinhole projection from depth image to 3D points
- compute_robust_surface(): SVD-based surface normal estimation with median depth filtering
- transform_point_and_normal(): TF2 wrapper for camera→map transforms
- IncrementalTrackManager: distance-weighted clustering for detection tracking
"""

import math
from collections import Counter

import numpy as np
import tf2_geometry_msgs as tfg
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs_py import point_cloud2 as pc2_lib

# ---------------------------------------------------------------------------
# Depth → 3D projection
# ---------------------------------------------------------------------------


class DepthCameraGeometry:
    """Extracts 3D points from a depth image using pinhole camera intrinsics.

    Call ``update_intrinsics`` once with a CameraInfo message, then use
    ``extract_3d_points`` on every synced depth frame.
    """

    def __init__(self, max_range: float = 5.0):
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None
        self.max_range = max_range
        self._ready = False

    @property
    def ready(self) -> bool:
        return self._ready

    def update_intrinsics(self, camera_info_msg) -> None:
        """Extract fx, fy, cx, cy from a sensor_msgs/CameraInfo K matrix."""
        k = camera_info_msg.k
        self.fx = k[0]
        self.fy = k[4]
        self.cx = k[2]
        self.cy = k[5]
        self._ready = True

    def extract_3d_points(self, mask: np.ndarray, depth: np.ndarray) -> np.ndarray:
        """Project masked depth pixels to 3D points in the camera optical frame.

        Parameters
        ----------
        mask : (H, W) uint8 array — nonzero where we want 3D points
        depth : (H, W) float32 array — depth in meters (32FC1)

        Returns
        -------
        (N, 3) float64 array of [X, Y, Z] in camera optical frame, or empty (0, 3).
        """
        if not self._ready:
            return np.empty((0, 3), dtype=np.float64)

        if self.cx is None or self.cy is None or self.fx is None or self.fy is None:
            return np.empty((0, 3), dtype=np.float64)

        vs, us = np.nonzero(mask)
        if len(vs) == 0:
            return np.empty((0, 3), dtype=np.float64)

        z = depth[vs, us].astype(np.float64)

        # Filter invalid and out-of-range depths
        valid = (z > 0.0) & (z < self.max_range) & np.isfinite(z)
        vs, us, z = vs[valid], us[valid], z[valid]
        if len(z) == 0:
            return np.empty((0, 3), dtype=np.float64)

        x = (us.astype(np.float64) - self.cx) * z / self.fx
        y = (vs.astype(np.float64) - self.cy) * z / self.fy

        return np.column_stack([x, y, z])


# ---------------------------------------------------------------------------
# PointCloud2 → 3D point extraction (organized cloud)
# ---------------------------------------------------------------------------


def extract_3d_points_from_pc2(
    mask: np.ndarray,
    pc2_msg,
    max_range: float = 5.0,
) -> np.ndarray:
    """Extract 3D points from an organized PointCloud2 at mask pixels.

    The PointCloud2 must be organized (height > 1, same H×W as the image).
    Points in the PC2 are already in the sensor's coordinate frame and have
    correct geometry — no pinhole math required.

    Parameters
    ----------
    mask    : (H, W) uint8 array — nonzero where we want 3D points
    pc2_msg : sensor_msgs/PointCloud2 (organized, height == image height)
    max_range : discard points further than this from the camera origin

    Returns
    -------
    (N, 3) float64 array or empty (0, 3).
    """
    h = pc2_msg.height
    w = pc2_msg.width
    if h <= 1:
        # Unorganized cloud — cannot use pixel-level masks
        return np.empty((0, 3), dtype=np.float64)

    pts = pc2_lib.read_points_numpy(pc2_msg, field_names=["x", "y", "z"])
    pts = pts.reshape((h, w, 3)).astype(np.float64)

    vs, us = np.nonzero(mask)
    if len(vs) == 0:
        return np.empty((0, 3), dtype=np.float64)

    # Clamp indices in case RGB and PC2 dimensions differ slightly
    vs_c = np.clip(vs, 0, h - 1)
    us_c = np.clip(us, 0, w - 1)
    points = pts[vs_c, us_c, :]

    finite = np.isfinite(points).all(axis=1)
    nonzero = ~np.all(points == 0.0, axis=1)
    in_range = np.linalg.norm(points, axis=1) < max_range
    valid = finite & nonzero & in_range

    return points[valid]


# ---------------------------------------------------------------------------
# SVD-based surface fitting
# ---------------------------------------------------------------------------


def compute_robust_surface(
    points_3d: np.ndarray,
    depth_tolerance: float = 0.05,
    min_inliers: int = 5,
):
    """Fit a plane to 3D points using median depth filtering + SVD.

    Parameters
    ----------
    points_3d : (N, 3) array in camera optical frame
    depth_tolerance : max distance from median depth to keep (meters)
    min_inliers : minimum points after filtering

    Returns
    -------
    (centroid, normal) as (3,) arrays, or None if too few inliers.
    The normal points *toward* the camera (away from the surface).
    """
    if len(points_3d) < min_inliers:
        return None

    # Distance of each point from camera origin
    dists = np.linalg.norm(points_3d, axis=1)
    median_dist = np.median(dists)

    # Keep only points close to the median depth (removes wall behind ring hole)
    inlier_mask = np.abs(dists - median_dist) < depth_tolerance
    inliers = points_3d[inlier_mask]

    if len(inliers) < min_inliers:
        return None

    centroid = inliers.mean(axis=0)
    centered = inliers - centroid

    # SVD: the normal is the right singular vector with smallest singular value
    _, s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[2]  # 3rd row = direction of least variance = surface normal

    # Flip normal to point toward the camera (centroid has positive Z in optical frame)
    if np.dot(normal, centroid) > 0:
        normal = -normal

    return centroid, normal


# ---------------------------------------------------------------------------
# TF2 helpers
# ---------------------------------------------------------------------------


def transform_point_and_normal(point, normal, tf_stamped):
    """Transform a 3D point and direction vector from camera to map frame.

    Parameters
    ----------
    point : (3,) array — position in camera frame
    normal : (3,) array — direction in camera frame
    tf_stamped : geometry_msgs/TransformStamped (camera → map)

    Returns
    -------
    (map_point, map_normal) as (3,) numpy arrays.
    """
    # Point (gets rotation + translation)
    ps = PointStamped()
    ps.header.frame_id = tf_stamped.header.frame_id
    ps.header.stamp = tf_stamped.header.stamp
    ps.point.x = float(point[0])
    ps.point.y = float(point[1])
    ps.point.z = float(point[2])
    tp = tfg.do_transform_point(ps, tf_stamped)
    map_point = np.array([tp.point.x, tp.point.y, tp.point.z])

    # Normal (gets rotation only — Vector3 transform ignores translation)
    vs = Vector3Stamped()
    vs.header.frame_id = tf_stamped.header.frame_id
    vs.header.stamp = tf_stamped.header.stamp
    vs.vector.x = float(normal[0])
    vs.vector.y = float(normal[1])
    vs.vector.z = float(normal[2])
    tv = tfg.do_transform_vector3(vs, tf_stamped)
    map_normal = np.array([tv.vector.x, tv.vector.y, tv.vector.z])

    return map_point, map_normal


def quaternion_to_rotation_matrix(q) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion to a 3×3 rotation matrix.

    Accepts any object with .x .y .z .w attributes (TransformStamped rotation).
    """
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )


def normal_to_quaternion(normal_2d):
    """Convert a 2D normal (x, y) into a quaternion facing opposite that normal.

    The resulting orientation faces *toward* the surface (the approach direction).
    Only the XY plane (yaw) is used — the arena has no tilted surfaces, so the
    surface normal's Z component is negligible.
    Returns (x, y, z, w) tuple.
    """
    yaw = math.atan2(-normal_2d[1], -normal_2d[0])
    # quaternion_from_euler(0, 0, yaw) → only Z rotation
    w = math.cos(yaw / 2.0)
    z = math.sin(yaw / 2.0)
    return (0.0, 0.0, z, w)


# ---------------------------------------------------------------------------
# Incremental Track Manager
# ---------------------------------------------------------------------------


class IncrementalTrackManager:
    """Distance-weighted object tracker for static detections.

    Stores all observations per track and uses inverse-distance² weighting
    to compute best position/normal estimates. Designed for O(N) per frame
    where N = number of active tracks (typically < 10).
    """

    def __init__(
        self,
        dedup_distance: float = 0.5,
        confirmation_count: int = 3,
        compact_inlier_radius: float = 0.35,
        compact_min_fraction: float = 0.6,
        max_cam_dist: float | None = None,
    ):
        self.dedup_distance = dedup_distance
        self.confirmation_count = confirmation_count
        self.compact_inlier_radius = compact_inlier_radius
        self.compact_min_fraction = compact_min_fraction
        self.max_cam_dist = max_cam_dist
        self._tracks: list[dict] = []
        self._next_id = 1

    def _compact_points(self, track):
        """Filter observations to those within _inlier_radius_ of the median XY position.

        Returns (list of inlier observations, median_x, median_y).
        """
        obs = track["observations"]
        if len(obs) < 3:
            return obs, None, None
        xs = [o["map_pos"][0] for o in obs]
        ys = [o["map_pos"][1] for o in obs]
        mx, my = np.median(xs), np.median(ys)
        inliers = [
            o
            for o in obs
            if np.sqrt((o["map_pos"][0] - mx) ** 2 + (o["map_pos"][1] - my) ** 2)
            <= self.compact_inlier_radius
        ]
        return inliers, mx, my

    def _is_compact(self, track):
        """True if enough observations are clustered around the median XY position."""
        obs = track["observations"]
        if len(obs) < self.confirmation_count:
            return False
        inliers, _, _ = self._compact_points(track)
        required = max(3, int(self.compact_min_fraction * len(obs)))
        return len(inliers) >= required

    def add_observation(self, map_point, normal, cam_dist, stamp, label=None):
        """Add a new observation and return (status, track).

        Matching: greedy nearest-observation (not centroid), so TF drift
        during sharp turns doesn't split a single object into two tracks.

        Gating: a track only confirms when its observations form a compact
        cluster — prevents two genuinely close objects from being bridged
        and merged into one.

        status is one of:
        - 'confirmed' — track just crossed the confirmation threshold
        - 'updated'   — observation added to an already-confirmed track
        - 'new'       — created a brand-new unconfirmed track
        - 'pending'   — observation added to existing unconfirmed track
        """
        map_point = np.asarray(map_point, dtype=np.float64)
        normal = np.asarray(normal, dtype=np.float64)
        cam_dist = float(cam_dist)

        if self.max_cam_dist is not None and cam_dist > self.max_cam_dist:
            return "ignored", None

        obs = {
            "map_pos": map_point,
            "normal": normal,
            "cam_dist": cam_dist,
            "stamp": stamp,
        }
        if label is not None:
            obs["label"] = label

        best_track = None
        best_dist = float("inf")
        for track in self._tracks:
            for o in track["observations"]:
                d = np.linalg.norm(map_point[:2] - o["map_pos"][:2])
                if d < best_dist:
                    best_dist = d
                    best_track = track

        if best_track is not None and best_dist < self.dedup_distance:
            best_track["observations"].append(obs)
            if best_track["confirmed"]:
                return "updated", best_track
            if len(best_track["observations"]) >= self.confirmation_count:
                if self._is_compact(best_track):
                    best_track["confirmed"] = True
                    return "confirmed", best_track
                return "pending", best_track
            return "pending", best_track

        track = {
            "id": self._next_id,
            "observations": [obs],
            "confirmed": False,
            "label": label,
        }
        self._next_id += 1
        self._tracks.append(track)
        return "new", track

    def get_best_estimate(self, track):
        """Robust estimate: median of compact inliers + weighted normal.

        Returns (position, normal, label).
        """
        observations = track["observations"]
        if not observations:
            return np.zeros(3), np.array([1.0, 0.0, 0.0]), None

        inliers, _, _ = self._compact_points(track)
        pts = inliers if len(inliers) >= 3 else observations

        positions = np.array([o["map_pos"] for o in pts])
        normals = np.array([o["normal"] for o in pts])
        labels = [o.get("label") for o in pts]

        weights = np.array([1.0 / (o["cam_dist"] ** 2 + 0.01) for o in pts])

        pos = np.median(positions, axis=0)
        n = np.average(normals, axis=0, weights=weights)
        n_len = np.linalg.norm(n)
        if n_len > 1e-6:
            n = n / n_len
        else:
            n = np.array([1.0, 0.0, 0.0])

        valid_labels = [lb for lb in labels if lb is not None]
        label = Counter(valid_labels).most_common(1)[0][0] if valid_labels else None

        return pos, n, label

    def get_confirmed_tracks(self):
        return [t for t in self._tracks if t["confirmed"]]

    def get_unconfirmed_tracks(self):
        return [
            t for t in self._tracks if not t["confirmed"] and len(t["observations"]) > 0
        ]
