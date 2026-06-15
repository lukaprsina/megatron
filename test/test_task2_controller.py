from __future__ import annotations

from megatron.task2_controller import (
    _inspection_completion_reason,
    _inspection_steering,
    _inspection_target_available,
    _inspection_waypoint_index,
)


def test_known_workstations_have_calibrated_inspection_waypoints() -> None:
    assert _inspection_waypoint_index("red") == 0
    assert _inspection_waypoint_index("green") == 1
    assert _inspection_waypoint_index("blue") is None


def test_assigned_red_inspection_does_not_require_detector_lock() -> None:
    assert _inspection_target_available("red", set(), waypoint_count=2)
    assert not _inspection_target_available("red", set(), waypoint_count=0)


def test_belt_follow_keeps_heading_correction_during_lateral_recovery() -> None:
    assert _inspection_steering(0.04, -0.01) == 0.03
    assert _inspection_steering(0.09, 0.04) == 0.10


def test_incomplete_tile_count_does_not_end_before_scan_limit() -> None:
    assert (
        _inspection_completion_reason(
            tile_count=3,
            expected_tiles=4,
            front_hit_count=0,
            scan_distance=1.44,
            max_scan_distance=2.4,
            elapsed=40.0,
        )
        is None
    )


def test_inspection_completion_prioritizes_expected_tile_count() -> None:
    assert (
        _inspection_completion_reason(
            tile_count=4,
            expected_tiles=4,
            front_hit_count=0,
            scan_distance=1.9,
            max_scan_distance=2.4,
            elapsed=45.0,
        )
        == "4 tiles"
    )
