"""Rigid transforms, LIO pose interpolation and the pinhole camera model."""
from __future__ import annotations

import bisect
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(r: np.ndarray, t: Sequence[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def transform_xyz_rpy(d: dict) -> np.ndarray:
    xyz = d.get("xyz", [0.0, 0.0, 0.0])
    rpy = d.get("rpy", [0.0, 0.0, 0.0])
    return transform(rpy_matrix(*rpy), xyz)


def apply(m: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, np.float64).reshape(-1, 3)
    return pts @ m[:3, :3].T + m[:3, 3]


def inverse(m: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out


def quat_xyzw_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _slerp(q0: np.ndarray, q1: np.ndarray, a: float) -> np.ndarray:
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)


class PoseBuffer:
    """Time-ordered LIO poses T_lio_imu (quaternion x, y, z, w + position) with interpolation."""

    def __init__(self, max_len: int = 400):
        self.t: List[float] = []
        self.q: List[np.ndarray] = []
        self.p: List[np.ndarray] = []
        self.max_len = max_len

    def add(self, t: float, q_xyzw: Sequence[float], p: Sequence[float]) -> None:
        if self.t and t <= self.t[-1]:
            if t == self.t[-1]:
                return
            self.t.clear(); self.q.clear(); self.p.clear()   # clock reset
        q = np.asarray(q_xyzw, np.float64)
        self.t.append(float(t))
        self.q.append(q / np.linalg.norm(q))
        self.p.append(np.asarray(p, np.float64))
        if len(self.t) > self.max_len:
            del self.t[0], self.q[0], self.p[0]

    def at(self, t: float, max_extrapolation: float = 0.05) -> Optional[np.ndarray]:
        """4x4 T_lio_imu at t (linear / slerp between neighbours); None outside the buffer."""
        if not self.t or t < self.t[0] - max_extrapolation or t > self.t[-1] + max_extrapolation:
            return None
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return transform(quat_xyzw_to_matrix(self.q[0]), self.p[0])
        if i >= len(self.t):
            return transform(quat_xyzw_to_matrix(self.q[-1]), self.p[-1])
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        q = _slerp(self.q[i - 1], self.q[i], a)
        p = self.p[i - 1] + a * (self.p[i] - self.p[i - 1])
        return transform(quat_xyzw_to_matrix(q), p)


class PinholeCamera:
    """OpenCV pinhole model (optical frame: x right, y down, z forward), radial-tangential
    distortion k1, k2, p1, p2, k3."""

    def __init__(self, K: Sequence[float], width: int, height: int,
                 dist: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0)):
        k = np.asarray(K, np.float64).reshape(-1)
        self.fx, self.fy, self.cx, self.cy = (k[0], k[4], k[2], k[5]) if k.size == 9 else k[:4]
        self.width, self.height = int(width), int(height)
        d = list(dist) + [0.0] * (5 - len(dist))
        self.k1, self.k2, self.p1, self.p2, self.k3 = d[:5]

    def project(self, pts_cam: np.ndarray, min_depth: float = 0.2
                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(uv (N, 2) float, depth (N,), in_image mask)."""
        pts = np.asarray(pts_cam, np.float64).reshape(-1, 3)
        z = pts[:, 2]
        valid = z > min_depth
        zs = np.where(valid, z, 1.0)
        x, y = pts[:, 0] / zs, pts[:, 1] / zs
        r2 = x * x + y * y
        radial = 1 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2
        xd = x * radial + 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
        yd = y * radial + self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
        u = self.fx * xd + self.cx
        v = self.fy * yd + self.cy
        valid &= (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        # Wide-angle distortion folds points far outside the field of view back in.
        valid &= r2 < 4.0
        return np.c_[u, v], z, valid


# Optical frame (z forward) relative to a body-style camera frame (x forward, z up).
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
