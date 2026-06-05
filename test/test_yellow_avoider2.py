"""Unit tests for yellow_avoider2 line fitting and steering logic.

Tests the standalone _fit_line + _compute_steering replicated from
yellow_avoider2.py.  Each test draws a synthetic yellow mask representing
a specific camera-view scenario and asserts the output direction.
"""

import numpy as np
import cv2
import pytest

W, H = 320, 240  # camera resolution


# ── replicated from yellow_avoider2.py (exact copies, not imports) ──

def _fit_line(scan_mask, min_pixels=30, cfg_margin=0.3):
    contours, _ = cv2.findContours(
        scan_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None

    best_contour = None
    best_bottom = -1
    for cnt in contours:
        if cv2.contourArea(cnt) < min_pixels:
            continue
        max_y = int(cnt[:, 0, 1].max())
        if max_y > best_bottom:
            best_bottom = max_y
            best_contour = cnt

    if best_contour is None:
        return None

    cnt_ys = best_contour[:, 0, 1]
    cnt_top = float(cnt_ys.min())
    cnt_bot = float(cnt_ys.max())

    cnt_mask = np.zeros_like(scan_mask)
    cv2.drawContours(cnt_mask, [best_contour], -1, 255, cv2.FILLED)
    ys, xs = np.where(cnt_mask > 0)
    pts = np.column_stack([xs, ys]).astype(np.float32)

    result = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
    vx = float(result[0][0])
    vy = float(result[1][0])
    x0 = float(result[2][0])
    y0 = float(result[3][0])

    cx = W / 2.0
    if abs(vx) < 1e-6:
        v_cross = y0
    else:
        t = (cx - x0) / vx
        v_cross = y0 + t * vy

    extent = max(cnt_bot - cnt_top, 1.0)
    margin = extent * cfg_margin
    if v_cross < cnt_top - margin or v_cross > cnt_bot + margin:
        return None

    v_cross = float(np.clip(v_cross, 0, H))
    return (vx, vy, x0, y0, v_cross)


def _compute_steering(line_result, kp=30.0):
    """Replicate of v2 _run_avoidance steering direction.
    Uses x_bottom (line's x at image bottom) to decide left vs right."""
    vx, vy, x0, y0, v_cross = line_result
    remaining = H - v_cross

    cx = W / 2.0
    if abs(vy) > 1e-6:
        x_bottom = x0 + (H - y0) * vx / vy
    else:
        x_bottom = x0

    steer_dir = 1.0 if x_bottom > cx else -1.0
    angular = steer_dir * kp / max(remaining, 1.0)
    return angular, steer_dir, remaining, x_bottom


# ── helpers ──

def _draw_line_mask(p1, p2, thickness=8):
    """Draw a thick yellow line on an empty mask."""
    m = np.zeros((H, W), dtype=np.uint8)
    cv2.line(m, p1, p2, 255, thickness)
    return m


def _draw_box_corner(bottom_right, top_left, thickness=8):
    """Draw a short box-corner that doesn't cross the center column."""
    m = np.zeros((H, W), dtype=np.uint8)
    # draw L-shaped yellow patch near the right edge
    bx1, by1 = bottom_right
    bx2, by2 = top_left
    cv2.line(m, (bx1, by1), (bx1, by2), 255, thickness)
    cv2.line(m, (bx1, by2), (bx2, by2), 255, thickness)
    return m


# ── steering direction tests ──

class TestSteeringDirection:
    """Expected steering: positive angular = steer LEFT, negative = steer RIGHT.

    Robot faces forward (+x in base_link, downward in image).
    Line on the LEFT  → steer RIGHT (away from line) → angular < 0.
    Line on the RIGHT → steer LEFT  (away from line) → angular > 0.
    """

    def test_line_bottomright_topleft(self):
        """Line right-of-center near bottom, crosses to left near top.
        At the bottom, the line is on the RIGHT → steer LEFT (positive angular)."""
        mask = _draw_line_mask((260, 220), (60, 80))
        res = _fit_line(mask)
        assert res is not None, "Should detect the line"
        ang, steer_dir, rem, x_bot = _compute_steering(res)
        assert steer_dir > 0, f"steer_dir should be +1 (line on RIGHT), got {steer_dir}"
        assert ang > 0, f"Should steer LEFT (positive), got angular={ang:.2f}"

    def test_line_bottomleft_topright(self):
        """Line left-of-center near bottom, crosses to right near top.
        At the bottom, line is on the LEFT → steer RIGHT (negative angular)."""
        mask = _draw_line_mask((60, 220), (260, 80))
        res = _fit_line(mask)
        assert res is not None, "Should detect the line"
        ang, steer_dir, rem, x_bot = _compute_steering(res)
        assert steer_dir < 0, f"steer_dir should be -1 (line on LEFT), got {steer_dir}"
        assert ang < 0, f"Should steer RIGHT (negative), got angular={ang:.2f}"

    def test_vertical_left(self):
        """Angled line from left-bottom toward center → on LEFT side at bottom.
        Line on LEFT → steer RIGHT (negative angular)."""
        mask = _draw_line_mask((60, 220), (150, 100))
        res = _fit_line(mask)
        assert res is not None, "Should detect the line"
        ang, steer_dir, rem, x_bot = _compute_steering(res)
        assert steer_dir < 0, f"steer_dir should be -1 (line on LEFT), got {steer_dir}"
        assert ang < 0, f"Should steer RIGHT (negative), got angular={ang:.2f}"

    def test_vertical_right(self):
        """Angled line from right-bottom toward center → on RIGHT side at bottom.
        Line on RIGHT → steer LEFT (positive angular)."""
        mask = _draw_line_mask((260, 220), (170, 100))
        res = _fit_line(mask)
        assert res is not None, "Should detect the line"
        ang, steer_dir, rem, x_bot = _compute_steering(res)
        assert steer_dir > 0, f"steer_dir should be +1 (line on RIGHT), got {steer_dir}"
        assert ang > 0, f"Should steer LEFT (positive), got angular={ang:.2f}"

    def test_line_centered_vertical(self):
        """Nearly vertical line that crosses center column.
        x_bottom < cx → steer_dir = -1, ang negative."""
        mask = _draw_line_mask((155, 220), (165, 80))
        res = _fit_line(mask)
        assert res is not None, "Should detect center-crossing line"
        ang, steer_dir, rem, x_bot = _compute_steering(res)
        assert steer_dir == -1.0, f"Expected steer_dir=-1, got {steer_dir}"
        assert isinstance(ang, float)


# ── extrapolation guard tests ──

class TestExtrapolationGuard:
    def test_box_corner_short_line(self):
        """A short yellow segment (box corner) that doesn't cross the
        center column should NOT trigger — fitLine extrapolation is rejected."""
        # Square-ish contour near the right edge, spanning rows 100-160
        mask = np.zeros((H, W), dtype=np.uint8)
        mask[100:160, 220:280] = 255  # 60x60 px block
        res = _fit_line(mask)
        # With extrapolation guard: block is square, fitLine gives arbitrary
        # direction, v_cross may land far outside [100, 160] → return None.
        assert res is None, (
            f"Box corner should be rejected (extrapolation), got {res}"
        )

    def test_long_diagonal_line(self):
        """A line that spans most of the image vertically should pass
        the extrapolation guard."""
        mask = _draw_line_mask((260, 210), (60, 40), thickness=10)
        res = _fit_line(mask)
        assert res is not None, (
            "Long line spanning the frame must not be rejected by extrapolation"
        )

    def test_box_corner_near_bottom(self):
        """Box corner near bottom but NOT crossing center — still rejected."""
        mask = np.zeros((H, W), dtype=np.uint8)
        mask[180:230, 240:290] = 255  # bottom-right 50x50 block
        res = _fit_line(mask)
        assert res is None, (
            f"Bottom-right box corner should be rejected, got {res}"
        )


# ── contour selection tests ──

class TestContourSelection:
    def test_picks_lowest_contour(self):
        """Two disconnected yellow blobs — the one closer to bottom wins."""
        mask = np.zeros((H, W), dtype=np.uint8)
        # Higher blob (far): far-right patch spanning rows 50-80
        mask[50:80, 240:300] = 255
        # Lower blob (near): a diagonal line spanning rows 160-220
        cv2.line(mask, (60, 160), (280, 220), 255, 8)
        res = _fit_line(mask)
        assert res is not None, "Should pick one contour"
        _, _, _, y0, _ = res
        # y0 should be near the lower blob's centroid, not the upper one's
        assert y0 > 100, f"fitLine centroid y0={y0:.0f}, expected >100 (lower blob)"

    def test_small_blobs_filtered(self):
        """Blobs below min_pixels are ignored."""
        mask = np.zeros((H, W), dtype=np.uint8)
        # Tiny noise: 5x5 px
        mask[10:15, 10:15] = 255
        # Real line: thick bottom-reaching
        cv2.line(mask, (100, 220), (220, 220), 255, 12)  # horizontal at bottom
        res = _fit_line(mask, min_pixels=100)
        assert res is not None, "Should pick the thick line, ignore tiny noise"

    def test_no_contours_none(self):
        mask = np.zeros((H, W), dtype=np.uint8)
        assert _fit_line(mask) is None


# ── v_cross computation tests ──

class TestVCross:
    def test_positive_slope_cross(self):
        """Line bottom-left → top-right: crosses center at known row."""
        y1, y2 = 200, 60
        x1, x2 = 40, 280
        mask = _draw_line_mask((x1, y1), (x2, y2), thickness=10)
        res = _fit_line(mask)
        assert res is not None
        v_cross = res[4]
        # The line passes through center x=160 somewhere between y=60 and y=200
        slope = (y2 - y1) / (x2 - x1)
        expected = y1 + slope * (160 - x1)  # linear interpolation
        assert abs(v_cross - expected) < 25, (
            f"v_cross={v_cross:.0f}, expected ~{expected:.0f}"
        )

    def test_negative_slope_cross(self):
        """Line bottom-right → top-left."""
        mask = _draw_line_mask((280, 200), (40, 60), thickness=10)
        res = _fit_line(mask)
        assert res is not None
        v_cross = res[4]
        y1, y2 = 200, 60
        x1, x2 = 280, 40
        slope = (y2 - y1) / (x2 - x1)
        expected = y1 + slope * (160 - x1)
        assert abs(v_cross - expected) < 25, (
            f"v_cross={v_cross:.0f}, expected ~{expected:.0f}"
        )


# ── proportional steering tests ──

class TestProportionalSteering:
    def test_closer_line_harder_steer(self):
        """A line that crosses lower (closer to robot) → larger angular."""
        # Far line: crosses at row 100 (remaining=140)
        mask_far = _draw_line_mask((280, 120), (40, 80), thickness=10)
        # Near line: crosses at row 180 (remaining=60)
        mask_near = _draw_line_mask((280, 200), (40, 160), thickness=10)

        res_far = _fit_line(mask_far)
        res_near = _fit_line(mask_near)
        assert res_far is not None
        assert res_near is not None

        ang_far, _, _, _ = _compute_steering(res_far, kp=30.0)
        ang_near, _, _, _ = _compute_steering(res_near, kp=30.0)

        # Both lines are bottom-right→top-left (steer LEFT, positive angular)
        assert ang_far > 0 and ang_near > 0, "Both should steer LEFT"
        # Closer line → larger |angular|
        assert abs(ang_near) > abs(ang_far), (
            f"Near angular={ang_near:.2f} should be > far angular={ang_far:.2f}"
        )
