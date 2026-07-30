"""Unit tests for reference-path configuration and message construction."""

import importlib.util
import os

import pytest


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "reference_path_publisher.py")
)
SPEC = importlib.util.spec_from_file_location("reference_path_publisher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_parse_waypoints_accepts_xyz_triples():
    points = MODULE.parse_waypoints([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    assert points == [(0.0, 1.0, 2.0), (3.0, 4.0, 5.0)]


@pytest.mark.parametrize(
    "values",
    [[], [0.0, 1.0, 2.0], [0.0] * 7, [0.0, 1.0, 2.0, float("nan"), 4.0, 5.0]],
)
def test_parse_waypoints_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        MODULE.parse_waypoints(values)


def test_build_path_message_preserves_coordinates_and_frame():
    from builtin_interfaces.msg import Time

    message = MODULE.build_path_message(
        [(-5.5, 5.5, 0.1), (-5.5, 4.5, 0.3)], "world", Time(sec=12)
    )
    assert message.header.frame_id == "world"
    assert message.header.stamp.sec == 12
    assert len(message.poses) == 2
    assert message.poses[-1].pose.position.y == pytest.approx(4.5)
    assert message.poses[-1].pose.position.z == pytest.approx(0.3)
    assert message.poses[-1].pose.orientation.w == pytest.approx(1.0)
