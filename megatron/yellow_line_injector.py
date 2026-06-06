#!/usr/bin/env python3
"""
Yellow line injector — publishes yellow ground pixels as PointCloud2
for Nav2's local costmap obstacle layer to consume.

Replaces the cmd_vel override in yellow_avoider2.  Nav2 becomes the
single authority over motion; it simply sees yellow lines as obstacles
and routes around them naturally.  No hysteresis, no state machine, no
cmd_vel fight.

Active in PATROL / APPROACH_TARGET / INTERACT states only.
"""

from typing import cast

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Yellow HSV thresholds — OpenCV scale (H: 0-180, S/V: 0-255)
_H_LO, _H_HI = 18, 35
_S_LO = 100
_V_LO = 80

_ACTIVE_STATES = frozenset(("PATROL", "APPROACH_TARGET", "INTERACT"))

# Structured dtype matching the camera's XYZRGB PointCloud2 (24 bytes/point):
#   x(f32,0) y(f32,4) z(f32,8) pad(u32,12) rgb(u32,16) pad(u32,20)
_CLOUD_DTYPE = np.dtype(
    [
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("_p0", np.uint32),
        ("rgb", np.uint32),  # packed 0x00RRGGBB, little-endian
        ("_p1", np.uint32),
    ]
)


class YellowLineInjector(Node):
    def __init__(self):
        super().__init__("yellow_line_injector")

        self.declare_parameter("min_depth", 0.5)  # metres
        self.declare_parameter("max_depth", 6.0)  # metres
        self.declare_parameter("min_points", 20)  # reject sparse noise

        self._active = False

        self.create_subscription(
            PointCloud2,
            "/top_camera/rgb/preview/depth/points",
            self._cloud_cb,
            SENSOR_QOS,
        )
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self._pub = self.create_publisher(PointCloud2, "/yellow_line_cloud", 10)
        self.get_logger().info("YellowLineInjector ready.")

    def _state_cb(self, msg: String):
        was = self._active
        self._active = msg.data in _ACTIVE_STATES
        if self._active != was:
            self.get_logger().info(
                f"{'Activated' if self._active else 'Deactivated'} (state={msg.data!r})"
            )

    def _cloud_cb(self, msg: PointCloud2):
        if not self._active:
            return

        pts = np.frombuffer(msg.data, dtype=_CLOUD_DTYPE)

        # Keep only points in a sane depth range
        z = pts["z"]
        min_d = cast(float, self.get_parameter("min_depth").value)
        max_d = cast(float, self.get_parameter("max_depth").value)
        valid = np.isfinite(z) & (z > min_d) & (z < max_d)
        if not np.any(valid):
            return

        pts_v = pts[valid]

        # Unpack RGB from little-endian uint32 (0x00RRGGBB)
        rgb = pts_v["rgb"]
        r = ((rgb >> 16) & 0xFF).astype(np.float32)
        g = ((rgb >> 8) & 0xFF).astype(np.float32)
        b = (rgb & 0xFF).astype(np.float32)

        yellow = _is_yellow(r, g, b)
        if int(np.sum(yellow)) < cast(int, self.get_parameter("min_points").value):
            return

        xyz = np.stack(
            [pts_v["x"][yellow], pts_v["y"][yellow], pts_v["z"][yellow]],
            axis=1,
        )
        self._pub.publish(_make_cloud(xyz, msg.header))


def _is_yellow(r: np.ndarray, g: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Vectorised RGB → HSV yellow test; returns boolean mask."""
    r_n, g_n, b_n = r / 255.0, g / 255.0, b / 255.0

    v = np.maximum(np.maximum(r_n, g_n), b_n)
    min_c = np.minimum(np.minimum(r_n, g_n), b_n)
    delta = v - min_c
    d_safe = np.where(delta > 0, delta, 1.0)

    s = np.where(v > 0, delta / v, 0.0)

    # Hue in [0, 6)
    h = np.zeros_like(v)
    m_r = (v == r_n) & (delta > 0)
    m_g = ~m_r & (v == g_n) & (delta > 0)
    m_b = ~m_r & ~m_g & (delta > 0)
    h = np.where(m_r, ((g_n - b_n) / d_safe) % 6.0, h)
    h = np.where(m_g, (b_n - r_n) / d_safe + 2.0, h)
    h = np.where(m_b, (r_n - g_n) / d_safe + 4.0, h)

    # OpenCV convention: H [0,180], S/V [0,255]
    h_cv = h * 30.0  # [0,6) → [0,180)
    s_cv = s * 255.0
    v_cv = v * 255.0

    return (h_cv >= _H_LO) & (h_cv <= _H_HI) & (s_cv >= _S_LO) & (v_cv >= _V_LO)


def _make_cloud(xyz: np.ndarray, header) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = YellowLineInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
