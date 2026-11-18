"""Builders / parsers for the unitree_sdk2py (CycloneDDS) message types used by g1d_nav.

Layouts match src/apps/dds_util.hpp: PointCloud2 x/y/z float32 (point_step 12, little endian);
rt/nav/registered_cloud adds a float32 "dynamic" field (point_step 16).
"""
from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import (Point_, Pose_, PoseWithCovariance_,
                                                      Quaternion_, Twist_,
                                                      TwistWithCovariance_, Vector3_)
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_, String_

FLOAT32 = 7


def stamp(t: float) -> Time_:
    sec = math.floor(t)
    return Time_(sec=int(sec), nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def to_sec(t: Time_) -> float:
    return t.sec + 1e-9 * t.nanosec


def header(t: float, frame: str) -> Header_:
    return Header_(stamp=stamp(t), frame_id=frame)


def twist(v: float, w: float) -> Twist_:
    return Twist_(linear=Vector3_(float(v), 0.0, 0.0), angular=Vector3_(0.0, 0.0, float(w)))


def string(data: str) -> String_:
    return String_(data=data)


def json_msg(obj: Dict[str, Any]) -> String_:
    return String_(data=json.dumps(obj, separators=(",", ":")))


def make_cloud_xyz(pts: np.ndarray, frame: str, t: float) -> PointCloud2_:
    pts = np.ascontiguousarray(np.asarray(pts, dtype="<f4").reshape(-1, 3))
    n = int(pts.shape[0])
    fields = [PointField_(name=c, offset=4 * i, datatype=FLOAT32, count=1)
              for i, c in enumerate("xyz")]
    return PointCloud2_(header=header(t, frame), height=1, width=n, fields=fields,
                        is_bigendian=False, point_step=12, row_step=12 * n,
                        data=list(pts.tobytes()), is_dense=True)


def make_cloud_xyzl(static: np.ndarray, dynamic: np.ndarray, frame: str, t: float) -> PointCloud2_:
    """x, y, z, dynamic float32 (point_step 16) as published by g1d_lio: dynamic = 1 for moving."""
    s = np.asarray(static, dtype="<f4").reshape(-1, 3)
    d = np.asarray(dynamic, dtype="<f4").reshape(-1, 3)
    data = np.empty((s.shape[0] + d.shape[0], 4), "<f4")
    data[:s.shape[0], :3] = s
    data[:s.shape[0], 3] = 0.0
    data[s.shape[0]:, :3] = d
    data[s.shape[0]:, 3] = 1.0
    n = int(data.shape[0])
    fields = [PointField_(name=c, offset=4 * i, datatype=FLOAT32, count=1)
              for i, c in enumerate(("x", "y", "z", "dynamic"))]
    return PointCloud2_(header=header(t, frame), height=1, width=n, fields=fields,
                        is_bigendian=False, point_step=16, row_step=16 * n,
                        data=list(np.ascontiguousarray(data).tobytes()), is_dense=True)


def _read_fields(m: PointCloud2_, names) -> Optional[np.ndarray]:
    off = {f.name: f.offset for f in m.fields if f.datatype == FLOAT32}
    if not all(c in off for c in "xyz") or m.point_step == 0:
        raise ValueError("PointCloud2 without float32 x/y/z")
    if not all(c in off for c in names):
        return None
    raw = bytes(m.data)
    n = len(raw) // m.point_step
    out = np.empty((n, len(names)), np.float32)
    if n == 0:
        return out
    buf = np.frombuffer(raw[:n * m.point_step], dtype=np.uint8).reshape(n, m.point_step)
    for i, c in enumerate(names):
        o = off[c]
        out[:, i] = buf[:, o:o + 4].copy().view("<f4").reshape(n)
    return out


def read_cloud_xyz(m: PointCloud2_) -> np.ndarray:
    """(N, 3) float32 from any PointCloud2 with float32 x, y, z fields; finite points only."""
    out = _read_fields(m, ("x", "y", "z"))
    return out[np.isfinite(out).all(axis=1)]


def read_cloud_xyzl(m: PointCloud2_) -> Tuple[np.ndarray, np.ndarray]:
    """(static (N, 3), dynamic (M, 3)) split by the float32 "dynamic" field (> 0.5 = moving);
    clouds without the field are all static."""
    out = _read_fields(m, ("x", "y", "z", "dynamic"))
    if out is None:
        return read_cloud_xyz(m), np.zeros((0, 3), np.float32)
    out = out[np.isfinite(out[:, :3]).all(axis=1)]
    moving = out[:, 3] > 0.5
    return np.ascontiguousarray(out[~moving, :3]), np.ascontiguousarray(out[moving, :3])


def yaw_of(q: Quaternion_) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_yaw(yaw: float) -> Quaternion_:
    return Quaternion_(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def make_odometry(t: float, x: float, y: float, yaw: float, v: float, w: float,
                  frame: str = "odom", child: str = "base_link") -> Odometry_:
    pose = PoseWithCovariance_(pose=Pose_(position=Point_(x, y, 0.0), orientation=quat_yaw(yaw)),
                               covariance=[0.0] * 36)
    tw = TwistWithCovariance_(twist=twist(v, w), covariance=[0.0] * 36)
    return Odometry_(header=header(t, frame), child_frame_id=child, pose=pose, twist=tw)


def odom_pose(m: Odometry_) -> Tuple[float, float, float, float, float]:
    """(x, y, yaw, v, w) of an Odometry_."""
    p = m.pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation),
            m.twist.twist.linear.x, m.twist.twist.angular.z)


def parse_json(m: String_) -> Optional[Dict[str, Any]]:
    try:
        obj = json.loads(m.data)
    except (ValueError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None
