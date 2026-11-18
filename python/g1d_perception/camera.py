"""Camera frame sources: the G1-D head camera via teleimager (ZMQ JPEG stream) and replay."""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterator, List, Optional

import numpy as np


@dataclass
class CameraFrame:
    stamp: float              # host clock (receive time - latency), seconds
    image: np.ndarray         # (H, W, 3) BGR uint8
    seq: int = 0


def crop_side_by_side(image: np.ndarray, which: str) -> np.ndarray:
    """The head camera publishes a binocular frame [left | right]; 'left' / 'right' / 'full'."""
    if which == "full":
        return image
    w = image.shape[1] // 2
    return image[:, :w] if which == "left" else image[:, w:]


def decode_jpeg(data: bytes) -> Optional[np.ndarray]:
    try:
        import cv2
    except ImportError:
        cv2 = None
    if cv2 is not None:
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    try:
        from turbojpeg import TurboJPEG
    except ImportError as e:
        raise RuntimeError("need opencv-python or PyTurboJPEG to decode camera frames") from e
    return TurboJPEG().decode(data)


class TeleimagerSource:
    """Subscribes to a teleimager camera topic (raw JPEG per ZMQ message, latest only).

    The stream carries no capture time: frames are stamped at receipt minus `latency_s`
    (measure it once, e.g. by filming a host clock); the projection tolerates a few 10 ms."""

    def __init__(self, host: str, zmq_port: int, *, latency_s: float = 0.06, crop: str = "left",
                 clock: Callable[[], float] = time.time):
        import zmq  # optional dependency (pyzmq)
        self._zmq = zmq
        self.host, self.port = host, int(zmq_port)
        self.latency_s, self.crop, self.clock = float(latency_s), crop, clock
        self._ctx = zmq.Context.instance()
        self._latest: Optional[CameraFrame] = None
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._seq = 0
        self._thread = threading.Thread(target=self._run, name="teleimager", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        zmq = self._zmq
        s = self._ctx.socket(zmq.SUB)
        s.setsockopt(zmq.CONFLATE, 1)
        s.setsockopt(zmq.RCVHWM, 1)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(f"tcp://{self.host}:{self.port}")
        s.setsockopt_string(zmq.SUBSCRIBE, "")
        poller = zmq.Poller()
        poller.register(s, zmq.POLLIN)
        while not self._stop.is_set():
            if not dict(poller.poll(200)).get(s):
                continue
            data = s.recv()
            t = self.clock() - self.latency_s
            img = decode_jpeg(data)
            if img is None:
                continue
            self._seq += 1
            with self._cond:
                self._latest = CameraFrame(t, crop_side_by_side(img, self.crop), self._seq)
                self._cond.notify_all()
        s.close()

    def next(self, timeout: float = 1.0, after_seq: int = 0) -> Optional[CameraFrame]:
        with self._cond:
            self._cond.wait_for(lambda: self._latest is not None and self._latest.seq > after_seq,
                                timeout)
            f = self._latest
        return f if f is not None and f.seq > after_seq else None

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(1.0)


class ReplaySource:
    """Frames recorded by g1d_perception.dataset (images/NNNNNN.jpg + frames.csv with stamps)."""

    def __init__(self, session_dir: str):
        self.dir = session_dir
        rows: List[tuple] = []
        with open(os.path.join(session_dir, "frames.csv"), encoding="utf-8") as f:
            next(f)
            for line in f:
                idx, stamp, name = line.strip().split(",")[:3]
                rows.append((int(idx), float(stamp), name))
        self.rows = rows

    def __iter__(self) -> Iterator[CameraFrame]:
        for idx, stamp, name in self.rows:
            with open(os.path.join(self.dir, "images", name), "rb") as f:
                img = decode_jpeg(f.read())
            if img is not None:
                yield CameraFrame(stamp, img, idx)
