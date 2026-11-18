"""LiDAR points inside person masks, depth gating, per-person boxes and a confirmation tracker."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class PersonObservation:
    points: np.ndarray            # (N, 3) LIO world
    centroid: np.ndarray          # (3,)
    score: float
    mask_pixels: int


def points_in_mask(uv: np.ndarray, depth: np.ndarray, valid: np.ndarray, mask: np.ndarray,
                   *, near_quantile: float = 0.2, depth_band: float = 0.6,
                   front_margin: float = 0.3, min_points: int = 5) -> np.ndarray:
    """Indices of projected points that belong to the masked person.

    Mask edges and gaps let background returns through, so only points within `depth_band`
    behind the near surface (the `near_quantile` depth of all points in the mask) are kept;
    points more than `front_margin` in front of it are occluders (e.g. a table edge)."""
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        return idx
    u = uv[idx, 0].astype(np.int64)
    v = uv[idx, 1].astype(np.int64)
    inside = mask[v, u]
    idx = idx[inside]
    if idx.size < min_points:
        return idx[:0]
    d = depth[idx]
    near = float(np.quantile(d, near_quantile))
    keep = (d <= near + depth_band) & (d >= near - front_margin)
    out = idx[keep]
    return out if out.size >= min_points else out[:0]


def box_around(points: np.ndarray, margin: float) -> Tuple[float, float, float, float, float, float]:
    lo = points.min(axis=0) - margin
    hi = points.max(axis=0) + margin
    return (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), float(lo[2]), float(hi[2]))


def box_corners(box: Sequence[float]) -> np.ndarray:
    x0, x1, y0, y1, z0, z1 = box
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], np.float64)


def floor_safe_box(points_lio: np.ndarray, margin: float, T_odom_lio: Optional[np.ndarray],
                   floor_clearance: float = 0.05) -> Tuple[float, ...]:
    """Map-deletion box in the LIO frame. With T_odom_lio the box is built in the gravity-aligned
    odom frame and starts `floor_clearance` above the floor, so the floor under a person stays in
    the map; its corners are mapped back and enclosed by a LIO-frame box."""
    if T_odom_lio is None:
        return box_around(points_lio, margin)
    r, t = T_odom_lio[:3, :3], T_odom_lio[:3, 3]
    p_odom = points_lio @ r.T + t
    b = list(box_around(p_odom, margin))
    b[4] = max(b[4], floor_clearance)
    if b[5] <= b[4]:
        b[5] = b[4] + 0.05
    corners_lio = (box_corners(b) - t) @ r
    lo, hi = corners_lio.min(axis=0), corners_lio.max(axis=0)
    return (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), float(lo[2]), float(hi[2]))


@dataclass
class _Track:
    centroid: np.ndarray
    last_t: float
    hits: int = 1
    reported_until: float = -1e9
    id: int = 0


@dataclass
class PersonTracker:
    """Nearest-centroid association; a person is confirmed after `confirm_hits` observations
    within `max_gap_s` of each other (single-frame false masks never delete map points)."""
    gate_m: float = 0.7
    max_gap_s: float = 0.6
    confirm_hits: int = 2
    tracks: List[_Track] = field(default_factory=list)
    _next_id: int = 1

    def update(self, t: float, observations: Sequence[PersonObservation]) -> List[Tuple[int, bool]]:
        """Returns (track id, confirmed) per observation."""
        self.tracks = [tr for tr in self.tracks if t - tr.last_t <= self.max_gap_s]
        out = []
        used = set()
        for ob in observations:
            best, best_d = None, self.gate_m
            for tr in self.tracks:
                if id(tr) in used:
                    continue
                d = float(np.linalg.norm(tr.centroid[:2] - ob.centroid[:2]))
                if d < best_d:
                    best, best_d = tr, d
            if best is None:
                best = _Track(ob.centroid.copy(), t, id=self._next_id)
                self._next_id += 1
                self.tracks.append(best)
            else:
                best.centroid = ob.centroid.copy()
                best.last_t = t
                best.hits += 1
            used.add(id(best))
            out.append((best.id, best.hits >= self.confirm_hits))
        return out
