"""Shared perception utilities for Megatron Task 1.

Contains:
- DepthCameraGeometry: pinhole projection from depth images
- compute_robust_surface: SVD-based centroid + surface normal
- IncrementalTrackManager: distance-weighted observation tracker
- TF2 helper functions (transform_point, transform_vector, normal_to_yaw)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import (
    PointStamped,
    Quaternion,
    Vector3Stamped,
)
from sensor_msgs.msg import CameraInfo

import tf2_geometry_msgs  # noqa: F401  – registers do_transform_* for PointStamped/Vector3Stamped
import tf2_ros


# ---------------------------------------------------------------------------
# TF2 helpers
# ---------------------------------------------------------------------------

def transform_point(point_3d: np.ndarray,
                    transform) -> np.ndarray:
    """Transform a 3-element point using a geometry_msgs TransformStamped."""
    p = PointStamped()
    p.point.x = float(point_3d[0])
    p.point.y = float(point_3d[1])
    p.point.z = float(point_3d[2])
    tp = tf2_geometry_msgs.do_transform_point(p, transform)
    return np.array([tp.point.x, tp.point.y, tp.point.z])


def transform_vector(vector_3d: np.ndarray,
                     transform) -> np.ndarray:
    """Transform a 3-element direction vector (rotation only)."""
    v = Vector3Stamped()
    v.vector.x = float(vector_3d[0])
    v.vector.y = float(vector_3d[1])
    v.vector.z = float(vector_3d[2])
    tv = tf2_geometry_msgs.do_transform_vector3(v, transform)
    return np.array([tv.vector.x, tv.vector.y, tv.vector.z])


def normal_to_yaw(normal_xy: np.ndarray) -> float:
    """Convert a 2-D normal direction (pointing out of the surface) into
    the yaw angle that *faces* the surface (i.e. opposite the normal)."""
    return math.atan2(-normal_xy[1], -normal_xy[0])


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_normal_xy(q: Quaternion) -> np.ndarray:
    """Extract the 2-D outward normal that was packed into a quaternion
    via normal_to_yaw → yaw_to_quaternion.  Returns unit vector in XY."""
    # Recover yaw from quaternion
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    # yaw points *toward* the surface; normal points *away*
    return np.array([-math.cos(yaw), -math.sin(yaw)])


# ---------------------------------------------------------------------------
# DepthCameraGeometry
# ---------------------------------------------------------------------------

class DepthCameraGeometry:
    """Pinhole model: projects 2-D pixel masks into 3-D camera-frame points
    using a 32FC1 depth image (meters).

    Call ``initialise(node)`` once.  The class subscribes to ``camera_info``
    and unsubscribes after the first message.
    """

    CAMERA_INFO_TOPIC = '/oakd/rgb/preview/camera_info'

    def __init__(self) -> None:
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self._sub = None
        self._ready = False

    @property
    def ready(self) -> bool:
        return self._ready

    def initialise(self, node: Node) -> None:
        """Subscribe to CameraInfo.  Unsubscribes after the first message."""
        self._node = node
        self._sub = node.create_subscription(
            CameraInfo,
            self.CAMERA_INFO_TOPIC,
            self._camera_info_cb,
            10,
        )
        node.get_logger().info(
            f'DepthCameraGeometry: waiting for {self.CAMERA_INFO_TOPIC}...')

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        k = msg.k  # 3×3 row-major
        self.fx = k[0]
        self.fy = k[4]
        self.cx = k[2]
        self.cy = k[5]
        self._ready = True
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None
        self._node.get_logger().info(
            f'DepthCameraGeometry ready: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def extract_3d_points(self,
                          mask: np.ndarray,
                          depth_image: np.ndarray) -> np.ndarray:
        """Project masked pixels into 3-D camera-frame coordinates.

        Parameters
        ----------
        mask : (H, W) bool or uint8 array (non-zero = valid)
        depth_image : (H, W) float32 array in *meters* (32FC1)

        Returns
        -------
        points : (N, 3) float64 array of (X, Y, Z) in the camera optical frame.
        """
        assert self._ready, 'DepthCameraGeometry not initialised'

        # Pixel coordinates where the mask is active
        vs, us = np.nonzero(mask)
        if len(vs) == 0:
            return np.empty((0, 3), dtype=np.float64)

        zs = depth_image[vs, us].astype(np.float64)

        # Filter out invalid depths (NaN, zero, negative, too far)
        valid = np.isfinite(zs) & (zs > 0.05) & (zs < 10.0)
        vs, us, zs = vs[valid], us[valid], zs[valid]

        if len(zs) == 0:
            return np.empty((0, 3), dtype=np.float64)
        
        if self.cx is None or self.cy is None or self.fx is None or self.fy is None:
            raise ValueError('Camera intrinsics not set in DepthCameraGeometry')

        xs = (us.astype(np.float64) - self.cx) * zs / self.fx
        ys = (vs.astype(np.float64) - self.cy) * zs / self.fy

        return np.column_stack((xs, ys, zs))


# ---------------------------------------------------------------------------
# SVD-based robust surface estimation
# ---------------------------------------------------------------------------

def compute_robust_surface(
    points_3d: np.ndarray,
    depth_tolerance: float = 0.05,
    min_points: int = 10,
) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """Compute centroid and surface normal from a cloud of 3-D points.

    1. Removes NaN / zero points.
    2. Median-distance filter (keeps points within *depth_tolerance* of the
       median distance to camera origin) — removes background / flying pixels.
    3. Computes centroid as the mean of the filtered points.
    4. SVD on centered points → 3rd principal component = surface normal.
    5. Flips normal toward camera if necessary.

    Returns ``(centroid, normal)`` or ``None`` if too few points survive.
    """
    if points_3d.shape[0] < min_points:
        return None

    # Remove NaN rows
    valid = np.isfinite(points_3d).all(axis=1)
    pts = points_3d[valid]
    if pts.shape[0] < min_points:
        return None

    # Median-distance filter
    dists = np.linalg.norm(pts, axis=1)
    med = np.median(dists)
    mask = np.abs(dists - med) < depth_tolerance
    pts = pts[mask]
    if pts.shape[0] < min_points:
        return None

    centroid = pts.mean(axis=0)
    centered = pts - centroid

    # SVD — the smallest singular value's right-singular vector is the normal
    _U, _S, Vt = np.linalg.svd(centered, full_matrices=False)
    normal = Vt[2]  # 3rd row = direction of least variance = surface normal

    # Ensure normal points toward the camera (origin in camera frame = [0,0,0])
    if np.dot(normal, -centroid) < 0:
        normal = -normal

    # Normalise
    n_len = np.linalg.norm(normal)
    if n_len < 1e-6:
        return None
    normal = normal / n_len

    return centroid, normal


# ---------------------------------------------------------------------------
# Incremental Track Manager
# ---------------------------------------------------------------------------

@dataclass
class Hypothesis:
    """A tracked detection hypothesis."""
    observations: list[dict] = field(default_factory=list)
    label: str = ''
    confirmed: bool = False
    reported: bool = False
    rejected: bool = False

    def obs_count(self) -> int:
        return len(self.observations)

    def weighted_estimate(self) -> dict:
        """Inverse-distance-squared weighted mean of position and normal.

        Close-up observations dominate far-away noisy ones.
        """
        positions = np.array([o['map_pos'] for o in self.observations])
        normals = np.array([o['normal'] for o in self.observations])
        cam_dists = np.array([o['cam_dist'] for o in self.observations])

        # Weights: 1 / d^2, clamped to avoid division by zero
        weights = 1.0 / np.maximum(cam_dists, 0.1) ** 2
        weights /= weights.sum()

        w_pos = (positions * weights[:, None]).sum(axis=0)
        w_normal = (normals * weights[:, None]).sum(axis=0)

        # Re-normalise the normal vector
        n_len = np.linalg.norm(w_normal)
        if n_len > 1e-6:
            w_normal = w_normal / n_len

        return {'map_pos': w_pos, 'normal': w_normal, 'label': self.label}


class IncrementalTrackManager:
    """Distance-thresholded track manager with inverse-distance weighting.

    Replaces the naive running-average confirmation from the old code.
    """

    def __init__(self,
                 dedup_distance: float = 0.5,
                 confirmation_count: int = 3) -> None:
        self.dedup_distance = dedup_distance
        self.confirmation_count = confirmation_count
        self.tracks: list[Hypothesis] = []

    def add_observation(
        self,
        map_pos: np.ndarray,
        normal: np.ndarray,
        cam_dist: float,
        stamp,
        label: str = '',
    ) -> tuple[int, bool]:
        """Add an observation, return ``(track_id, newly_confirmed)``.

        If the observation falls within ``dedup_distance`` of an existing
        track's weighted center, it is merged.  Otherwise a new hypothesis
        is created.
        """
        obs = {
            'map_pos': np.asarray(map_pos, dtype=np.float64),
            'normal': np.asarray(normal, dtype=np.float64),
            'cam_dist': float(cam_dist),
            'stamp': stamp,
        }

        best_idx = -1
        best_dist = self.dedup_distance

        for i, hyp in enumerate(self.tracks):
            if hyp.rejected:
                continue
            est = hyp.weighted_estimate()
            d = np.linalg.norm(map_pos - est['map_pos'])
            if d < best_dist:
                best_dist = d
                best_idx = i

        newly_confirmed = False

        if best_idx >= 0:
            hyp = self.tracks[best_idx]
            hyp.observations.append(obs)
            # Override label with the latest (colors might stabilize)
            if label:
                hyp.label = label
            if not hyp.confirmed and hyp.obs_count() >= self.confirmation_count:
                hyp.confirmed = True
                newly_confirmed = True
            return best_idx, newly_confirmed
        else:
            # New hypothesis
            hyp = Hypothesis(observations=[obs], label=label)
            if self.confirmation_count <= 1:
                hyp.confirmed = True
                newly_confirmed = True
            self.tracks.append(hyp)
            return len(self.tracks) - 1, newly_confirmed

    def get_best_estimate(self, track_id: int) -> dict:
        """Return the inverse-distance-weighted estimate for a track."""
        return self.tracks[track_id].weighted_estimate()

    def is_confirmed(self, track_id: int) -> bool:
        return self.tracks[track_id].confirmed

    def is_reported(self, track_id: int) -> bool:
        return self.tracks[track_id].reported

    def mark_reported(self, track_id: int) -> None:
        self.tracks[track_id].reported = True

    def mark_rejected(self, track_id: int) -> None:
        self.tracks[track_id].rejected = True

    def get_confirmed_unreported(self) -> list[tuple[int, dict]]:
        """Return list of (track_id, estimate) for confirmed but unreported tracks."""
        results = []
        for i, hyp in enumerate(self.tracks):
            if hyp.confirmed and not hyp.reported and not hyp.rejected:
                results.append((i, hyp.weighted_estimate()))
        return results

    def get_unconfirmed_hypotheses(self, min_observations: int = 1) -> list[tuple[int, dict]]:
        """Return unconfirmed, non-rejected hypotheses with enough observations."""
        results = []
        for i, hyp in enumerate(self.tracks):
            if (not hyp.confirmed and not hyp.rejected
                    and hyp.obs_count() >= min_observations):
                results.append((i, hyp.weighted_estimate()))
        return results

    def confirmed_count(self) -> int:
        return sum(1 for h in self.tracks if h.confirmed and not h.rejected)
