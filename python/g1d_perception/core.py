"""Perception core (no I/O): camera frame + LIO poses + registered scans -> person points and
map-deletion boxes, all in the LIO world frame (odom_lio)."""
from __future__ import annotations

import collections
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from .association import PersonObservation, PersonTracker, floor_safe_box, points_in_mask
from .geometry import (R_BODY_OPTICAL, PinholeCamera, PoseBuffer, apply, inverse, transform,
                       transform_xyz_rpy)
from .segmentation import Detection, SegmentationBackend


@dataclass
class PerceptionParams:
    cloud_window_s: float = 0.25        # registered scans around the image time used for masks
    max_cloud_age_s: float = 0.4
    min_score: float = 0.4
    min_mask_pixels: int = 400
    near_quantile: float = 0.2
    depth_band: float = 0.6
    front_margin: float = 0.3
    min_points: int = 8
    max_range: float = 8.0
    box_margin: float = 0.15
    floor_clearance: float = 0.05       # person points / boxes start this high above the floor
    delete_every_s: float = 0.5         # per confirmed person
    confirm_hits: int = 2

    @staticmethod
    def from_cfg(p: Dict) -> "PerceptionParams":
        out = PerceptionParams()
        for k in out.__dataclass_fields__:
            if k in p:
                setattr(out, k, type(getattr(out, k))(p[k]))
        return out


@dataclass
class FrameResult:
    stamp: float
    detections: int = 0
    persons: List[PersonObservation] = field(default_factory=list)
    person_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), np.float32))
    delete_boxes: List[Tuple[float, ...]] = field(default_factory=list)
    skipped: str = ""
    segment_ms: float = 0.0


def camera_extrinsic(cfg: Dict) -> np.ndarray:
    """T_imu_cam (optical frame) from the project config:
    lio.extrinsic_T/R (LiDAR in IMU), kinematics.head_to_livox, perception.camera.head_to_camera
    (camera body frame: x forward, z up)."""
    lio = cfg.get("lio", {}) or {}
    t_il = transform(np.asarray(lio.get("extrinsic_R", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                                np.float64).reshape(3, 3),
                     lio.get("extrinsic_T", [0.0, 0.0, 0.0]))
    t_hl = transform_xyz_rpy((cfg.get("kinematics", {}) or {}).get("head_to_livox", {}) or {})
    cam = ((cfg.get("perception", {}) or {}).get("camera", {}) or {})
    t_hc = transform_xyz_rpy(cam.get("head_to_camera", {}) or {})
    t_hc_opt = t_hc @ transform(R_BODY_OPTICAL, [0.0, 0.0, 0.0])
    return t_il @ inverse(t_hl) @ t_hc_opt


class PerceptionCore:
    def __init__(self, camera: PinholeCamera, T_imu_cam: np.ndarray,
                 backend: SegmentationBackend, params: PerceptionParams = PerceptionParams()):
        self.camera = camera
        self.T_imu_cam = T_imu_cam
        self.backend = backend
        self.p = params
        self.poses = PoseBuffer()
        self.clouds: Deque[Tuple[float, np.ndarray]] = collections.deque(maxlen=12)
        self.tracker = PersonTracker(confirm_hits=params.confirm_hits)
        self.T_odom_lio: Optional[np.ndarray] = None
        self._last_delete: Dict[int, float] = {}

    # ---- inputs
    def add_pose(self, t: float, q_xyzw, p) -> None:
        self.poses.add(t, q_xyzw, p)

    def add_cloud(self, t: float, points_lio: np.ndarray) -> None:
        self.clouds.append((t, np.asarray(points_lio, np.float32).reshape(-1, 3)))

    def set_T_odom_lio(self, m: Optional[np.ndarray]) -> None:
        self.T_odom_lio = m

    # ---- frame
    def _scan_points(self, t: float) -> Optional[np.ndarray]:
        sel = [pts for (tc, pts) in self.clouds if abs(tc - t) <= self.p.cloud_window_s]
        if not sel:
            newest = max((tc for tc, _ in self.clouds), default=None)
            if newest is None or abs(newest - t) > self.p.max_cloud_age_s:
                return None
            sel = [pts for (tc, pts) in self.clouds if tc == newest]
        return np.concatenate(sel)

    def process(self, stamp: float, image: np.ndarray,
                detections: Optional[List[Detection]] = None) -> FrameResult:
        res = FrameResult(stamp)
        T_lio_imu = self.poses.at(stamp)
        if T_lio_imu is None:
            res.skipped = "no_pose"
            return res
        pts = self._scan_points(stamp)
        if pts is None or pts.shape[0] == 0:
            res.skipped = "no_cloud"
            return res
        if detections is None:
            detections = self.backend(image)
            res.segment_ms = self.backend.last_ms
        detections = [d for d in detections
                      if d.score >= self.p.min_score and int(d.mask.sum()) >= self.p.min_mask_pixels]
        res.detections = len(detections)
        if not detections:
            return res
        T_cam_lio = inverse(T_lio_imu @ self.T_imu_cam)
        pc = apply(T_cam_lio, pts)
        uv, depth, valid = self.camera.project(pc)
        valid &= depth < self.p.max_range
        chunks = []
        for det in detections:
            idx = points_in_mask(uv, depth, valid, det.mask, near_quantile=self.p.near_quantile,
                                 depth_band=self.p.depth_band, front_margin=self.p.front_margin,
                                 min_points=self.p.min_points)
            if idx.size == 0:
                continue
            if self.T_odom_lio is not None:
                # Floor returns at the feet share the person's depth: drop them by height.
                r, t = self.T_odom_lio[:3, :3], self.T_odom_lio[:3, 3]
                z = pts[idx].astype(np.float64) @ r[2] + t[2]
                idx = idx[z >= self.p.floor_clearance]
                if idx.size < self.p.min_points:
                    continue
            p = pts[idx].astype(np.float64)
            res.persons.append(PersonObservation(p, p.mean(axis=0), det.score, int(det.mask.sum())))
            chunks.append(pts[idx])
        if chunks:
            res.person_points = np.concatenate(chunks)
        for ob, (tid, confirmed) in zip(res.persons, self.tracker.update(stamp, res.persons)):
            if confirmed and stamp - self._last_delete.get(tid, -1e9) >= self.p.delete_every_s:
                self._last_delete[tid] = stamp
                res.delete_boxes.append(floor_safe_box(ob.points, self.p.box_margin, self.T_odom_lio,
                                                       self.p.floor_clearance))
        return res
