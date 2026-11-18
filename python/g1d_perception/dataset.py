"""Training / evaluation data: camera frames with the matching LiDAR scan and LIO pose.

Session layout (datasets/<name>/):
    images/NNNNNN.jpg            head camera frame (left image)
    scans/NNNNNN.npz             points_lio (N, 3) float32, dynamic (N,) uint8, T_lio_imu (4, 4)
    frames.csv                   index, stamp, image, scan, has_pose
    labels/NNNNNN.txt            (after tools/sam3_label.py) YOLO-seg polygons, class 0 = person
    camera.json                  K, dist, size, T_imu_cam used while recording
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Dict, List, Optional, Sequence

import numpy as np


class DatasetRecorder:
    def __init__(self, root: str, *, every_s: float = 0.5, jpeg_quality: int = 90,
                 max_frames: int = 20000):
        self.root, self.every_s = root, float(every_s)
        self.jpeg_quality, self.max_frames = int(jpeg_quality), int(max_frames)
        self.dir: Optional[str] = None
        self.n = 0
        self._last_t = -1e9
        self._lock = threading.Lock()

    def start(self, camera_meta: Dict, name: Optional[str] = None) -> str:
        with self._lock:
            self.dir = os.path.join(self.root, name or time.strftime("%Y%m%d-%H%M%S"))
            for sub in ("images", "scans", "labels"):
                os.makedirs(os.path.join(self.dir, sub), exist_ok=True)
            with open(os.path.join(self.dir, "camera.json"), "w", encoding="utf-8") as f:
                json.dump(camera_meta, f, indent=2, default=lambda o: np.asarray(o).tolist())
            with open(os.path.join(self.dir, "frames.csv"), "w", encoding="utf-8") as f:
                f.write("index,stamp,image,scan,has_pose\n")
            self.n, self._last_t = 0, -1e9
            return self.dir

    def add(self, stamp: float, image_bgr: np.ndarray, points_lio: Optional[np.ndarray],
            dynamic: Optional[np.ndarray], T_lio_imu: Optional[np.ndarray]) -> bool:
        with self._lock:
            if self.dir is None or self.n >= self.max_frames or stamp - self._last_t < self.every_s:
                return False
            import cv2
            name = f"{self.n:06d}"
            ok, buf = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                return False
            with open(os.path.join(self.dir, "images", name + ".jpg"), "wb") as f:
                f.write(buf.tobytes())
            scan = ""
            if points_lio is not None:
                scan = name + ".npz"
                np.savez_compressed(
                    os.path.join(self.dir, "scans", scan),
                    points_lio=np.asarray(points_lio, np.float32),
                    dynamic=(np.zeros(len(points_lio), np.uint8) if dynamic is None
                             else np.asarray(dynamic, np.uint8)),
                    T_lio_imu=np.eye(4) if T_lio_imu is None else T_lio_imu)
            with open(os.path.join(self.dir, "frames.csv"), "a", encoding="utf-8") as f:
                f.write(f"{self.n},{stamp:.6f},{name}.jpg,{scan},{int(T_lio_imu is not None)}\n")
            self.n += 1
            self._last_t = stamp
            return True

    def stop(self) -> None:
        with self._lock:
            self.dir = None


# ------------------------------------------------------------------------------- YOLO format
def mask_to_polygons(mask: np.ndarray, min_area: int = 50, epsilon_px: float = 1.5) -> List[np.ndarray]:
    """Outer contours of a binary mask as (K, 2) pixel polygons (OpenCV)."""
    import cv2
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        c = cv2.approxPolyDP(c, epsilon_px, True).reshape(-1, 2)
        if c.shape[0] >= 3:
            out.append(c.astype(np.float64))
    return out


def yolo_seg_lines(masks: Sequence[np.ndarray], cls: int = 0) -> List[str]:
    """Ultralytics segmentation label lines: `cls x1 y1 x2 y2 ...` normalised to [0, 1]."""
    lines = []
    for m in masks:
        h, w = m.shape
        for poly in mask_to_polygons(m):
            xy = poly / np.array([w, h], np.float64)
            lines.append(f"{cls} " + " ".join(f"{v:.5f}" for v in np.clip(xy, 0, 1).reshape(-1)))
    return lines


def polygons_to_mask(lines: Sequence[str], h: int, w: int) -> np.ndarray:
    import cv2
    m = np.zeros((h, w), np.uint8)
    for ln in lines:
        v = np.array([float(x) for x in ln.split()[1:]]).reshape(-1, 2) * [w, h]
        cv2.fillPoly(m, [np.round(v).astype(np.int32)], 1)
    return m.astype(bool)
