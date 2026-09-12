import math

import pytest

from pc_trajectory.demo.map_transform import MapTransform, MapTransformError
from pc_trajectory.demo.protocol import (
    MessageType,
    Pose,
    ProtocolError,
    decode_message,
    encode_message,
    make_command,
    shortest_angle_deg,
    wrap_to_180,
)
from pc_trajectory.demo.simulated_robot_link import SimulatedRobotLink
from pc_trajectory.geometry import Point2D


def test_map_transform_has_world_x_right_and_world_y_up():
    transform = MapTransform(320.0, 240.0, 2.0)
    assert transform.pixel_to_world(370.0, 240.0) == Point2D(100.0, 0.0)
    assert transform.pixel_to_world(320.0, 190.0) == Point2D(0.0, 100.0)
    assert transform.world_to_pixel(Point2D(100.0, 0.0)) == (370.0, 240.0)
    assert transform.world_to_pixel(Point2D(0.0, 100.0)) == (320.0, 190.0)


def test_map_transform_rejects_non_positive_scale():
    with pytest.raises(MapTransformError):
        MapTransform(0.0, 0.0, 0.0)


def test_angle_helpers_preserve_continuous_yaw_and_shortest_error():
    assert math.isclose(wrap_to_180(361.8), 1.8, abs_tol=1.0e-9)
    assert math.isclose(shortest_angle_deg(10.0, 350.0), 20.0, abs_tol=1.0e-9)
    assert math.isclose(shortest_angle_deg(350.0, 10.0), -20.0, abs_tol=1.0e-9)


def test_protocol_round_trip_rejects_nan_and_unknown_types():
    command = make_command(MessageType.RESET_POSE, 4, x_mm=0.0, y_mm=0.0, yaw_deg=0.0)
    assert decode_message(encode_message(command)) == command
    with pytest.raises(ProtocolError):
        encode_message({"type": "RESET_POSE", "id": 5, "x_mm": math.nan, "y_mm": 0, "yaw_deg": 0})
    with pytest.raises(ProtocolError):
        decode_message('{"type":"DOES_NOT_EXIST"}')


def test_pose_message_keeps_yaw_continuous():
    pose = Pose(1.0, 2.0, 361.8, timestamp_ms=10.0)
    decoded = Pose.from_message(pose.to_message(8))
    assert decoded.yaw_deg == 361.8


def test_simulated_body_axes_and_rotation():
    link = SimulatedRobotLink()
    link.connect()
    link.drain_events()
    link.inject_body_motion(100.0, 0.0, 1.0)
    assert math.isclose(link.pose.x_mm, 100.0, abs_tol=1.0e-9)
    assert math.isclose(link.pose.y_mm, 0.0, abs_tol=1.0e-9)
    link.inject_body_motion(0.0, 100.0, 1.0)
    assert math.isclose(link.pose.x_mm, 100.0, abs_tol=1.0e-9)
    assert math.isclose(link.pose.y_mm, 100.0, abs_tol=1.0e-9)
    events = link.send(make_command(MessageType.ROTATE_REL, 1, angle_deg=90.0, speed_deg_s=45.0))
    assert any(event["type"] == "ACK" for event in events)
    assert math.isclose(link.pose.yaw_deg, 90.0, abs_tol=1.0e-9)


def test_simulated_estop_blocks_motion_until_cleared():
    link = SimulatedRobotLink()
    link.connect()
    link.drain_events()
    link.send(make_command(MessageType.ESTOP, 1))
    with pytest.raises(RuntimeError):
        link.inject_body_motion(100.0, 0.0, 1.0)
    errors = link.send(make_command(MessageType.ROTATE_REL, 2, angle_deg=90.0, speed_deg_s=45.0))
    assert any(event["type"] == "ERROR" and event["code"] == "ESTOP_ACTIVE" for event in errors)
    link.send(make_command(MessageType.CLEAR_ESTOP, 3))
    link.inject_body_motion(100.0, 0.0, 1.0)
    assert link.pose.x_mm > 0.0
