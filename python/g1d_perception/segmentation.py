"""Person instance segmentation backends.

On PC2 (Orin NX) the backend is a small YOLO-seg model (YOLO11n-seg / YOLO26n-seg distilled from
SAM 3 labels on the 5090, exported to TensorRT on the robot). Ultralytics is an optional
dependency; tests use CallableBackend.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import numpy as np


@dataclass
class Detection:
    mask: np.ndarray        # (H, W) bool, image resolution
    score: float
    label: str = "person"


class SegmentationBackend:
    name = "base"
    last_ms = 0.0

    def __call__(self, image_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError


class CallableBackend(SegmentationBackend):
    name = "callable"

    def __init__(self, fn: Callable[[np.ndarray], List[Detection]]):
        self.fn = fn

    def __call__(self, image_bgr: np.ndarray) -> List[Detection]:
        t0 = time.perf_counter()
        out = self.fn(image_bgr)
        self.last_ms = (time.perf_counter() - t0) * 1e3
        return out


class YoloSegBackend(SegmentationBackend):
    """Ultralytics YOLO segmentation (.pt, or .engine exported on the Orin for TensorRT)."""
    name = "yolo"

    def __init__(self, model: str, *, conf: float = 0.4, imgsz: int = 640,
                 device: Optional[str] = None, person_class: int = 0, half: bool = True):
        from ultralytics import YOLO  # optional dependency
        self.model = YOLO(model, task="segment")
        self.conf, self.imgsz, self.device = conf, imgsz, device
        self.person_class, self.half = person_class, half

    def __call__(self, image_bgr: np.ndarray) -> List[Detection]:
        t0 = time.perf_counter()
        res = self.model.predict(image_bgr, conf=self.conf, imgsz=self.imgsz, device=self.device,
                                 classes=[self.person_class], half=self.half, retina_masks=True,
                                 verbose=False)[0]
        out: List[Detection] = []
        if res.masks is not None:
            masks = res.masks.data.cpu().numpy() > 0.5
            scores = res.boxes.conf.cpu().numpy()
            h, w = image_bgr.shape[:2]
            for m, s in zip(masks, scores):
                if m.shape != (h, w):
                    m = _resize_nearest(m, h, w)
                out.append(Detection(m, float(s)))
        self.last_ms = (time.perf_counter() - t0) * 1e3
        return out


def _resize_nearest(mask: np.ndarray, h: int, w: int) -> np.ndarray:
    ys = (np.arange(h) * mask.shape[0] / h).astype(np.int64)
    xs = (np.arange(w) * mask.shape[1] / w).astype(np.int64)
    return mask[ys][:, xs]


def make_backend(cfg: dict) -> SegmentationBackend:
    kind = cfg.get("backend", "yolo")
    if kind == "yolo":
        return YoloSegBackend(cfg.get("model", "models/person_seg.engine"),
                              conf=float(cfg.get("conf", 0.4)), imgsz=int(cfg.get("imgsz", 640)),
                              device=cfg.get("device"), half=bool(cfg.get("half", True)))
    raise ValueError(f"unknown segmentation backend {kind!r} (on-robot: yolo)")
