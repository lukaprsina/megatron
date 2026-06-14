from __future__ import annotations

from megatron.task2_controller import (
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
