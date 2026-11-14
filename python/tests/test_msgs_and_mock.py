import math
import struct

import numpy as np
import pytest

from g1d_nav_py import msgs
from g1d_nav_py.chassis_http import EventTracker
from g1d_nav_py.mock_robot import footprint_gap, raycast_boxes
from g1d_nav_py.nav_client import make_goal
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_


def test_cloud_roundtrip():
    pts = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 0.0]], np.float32)
    m = msgs.make_cloud_xyz(pts, "base_link", 12.5)
    assert m.point_step == 12 and m.width == 2 and len(m.data) == 24
    np.testing.assert_array_equal(msgs.read_cloud_xyz(m), pts)
    assert msgs.to_sec(m.header.stamp) == pytest.approx(12.5)


def test_read_cloud_with_extra_fields_and_nan():
    # Livox-style layout: x y z intensity(float32) tag(uint8) ... point_step 18
    fields = [PointField_(name=n, offset=o, datatype=t, count=1)
              for n, o, t in (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("tag", 16, 2))]
    raw = b"".join(struct.pack("<ffffBB", *p) for p in ((1, 2, 3, 9, 1, 0), (float("nan"), 0, 0, 0, 0, 0)))
    m = PointCloud2_(header=msgs.header(0.0, "livox"), height=1, width=2, fields=fields,
                     is_bigendian=False, point_step=18, row_step=36, data=list(raw), is_dense=False)
    np.testing.assert_array_equal(msgs.read_cloud_xyz(m), [[1, 2, 3]])


def test_odometry_yaw_roundtrip():
    m = msgs.make_odometry(1.0, 0.5, -0.2, 2.5, 0.3, -0.1)
    x, y, yaw, v, w = msgs.odom_pose(m)
    assert (x, y, v, w) == pytest.approx((0.5, -0.2, 0.3, -0.1))
    assert yaw == pytest.approx(2.5)


def test_stamp_nanosec_carry():
    t = msgs.stamp(10.9999999999)
    assert t.nanosec < 1000000000


def test_raycast_and_footprint_gap():
    boxes = [(2.0, -1.0, 2.2, 1.0)]
    r = raycast_boxes((0.0, 0.0), np.array([0.0, math.pi, math.atan2(1.0, 2.0)]), boxes, 10.0)
    assert r[0] == pytest.approx(2.0)
    assert math.isinf(r[1])
    assert r[2] == pytest.approx(math.hypot(2.0, 1.0))
    assert footprint_gap((0.0, 0.0, 0.0), boxes) == pytest.approx(2.0 - 0.2785, abs=1e-6)
    assert footprint_gap((1.9, 0.0, 0.0), boxes) < 0.0


def test_event_tracker_reports_only_new_events():
    t = EventTracker()
    assert t.update([{"type": "BUMPER_TRIGGERED", "timestamp": 1}]) == []
    assert t.update([{"type": "BUMPER_TRIGGERED", "timestamp": 1}]) == []
    assert t.update([{"type": "BUMPER_TRIGGERED", "timestamp": 1},
                     {"type": "CLIFF_DETECTED", "timestamp": 2}]) == ["CLIFF_DETECTED"]


def test_make_goal_validation():
    g = make_goal(1.0, 2.0, 0.5, stamp=3.0, max_speed=0.3)
    assert g["frame"] == "base_at_stamp" and g["stamp"] == 3.0 and g["max_speed"] == 0.3
    assert "yaw" not in make_goal(1.0, 0.0)
    with pytest.raises(ValueError):
        make_goal(mode="fly")
    with pytest.raises(ValueError):
        make_goal(frame="map")
