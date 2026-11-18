"""g1d_perception: head camera person masks + Mid-360S points -> rt/nav/person_points,
rt/nav/map_delete, rt/nav/perception_status (unitree_sdk2py DDS).

    python -m g1d_perception.node [--config config/g1d_nav.yaml] [--record datasets]

Subscribes rt/nav/lio_odom (T_lio_imu), rt/nav/registered_cloud (LIO frame) and rt/nav/debug
(T_odom_lio, for floor-safe deletion boxes). Not part of the safety loop: if it stops, the
costmap still has the raw returns and the M-detector labels.
"""
from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from typing import Optional

import numpy as np

from g1d_nav_py import msgs
from g1d_nav_py.config import get, load_config, net_if

from .camera import TeleimagerSource
from .core import PerceptionCore, PerceptionParams, camera_extrinsic
from .dataset import DatasetRecorder
from .geometry import PinholeCamera, quat_xyzw_to_matrix, transform
from .segmentation import make_backend

log = logging.getLogger("g1d.perception")


def build_core(cfg, backend=None) -> PerceptionCore:
    cam = get(cfg, "perception.camera", {}) or {}
    w, h = cam.get("size", [640, 480])
    camera = PinholeCamera(cam.get("K", [380.0, 0.0, 320.0, 0.0, 380.0, 240.0, 0.0, 0.0, 1.0]),
                           w, h, cam.get("dist", [0.0, 0.0, 0.0, 0.0, 0.0]))
    params = PerceptionParams.from_cfg(get(cfg, "perception.association", {}) or {})
    if backend is None:
        backend = make_backend(get(cfg, "perception.segmentation", {}) or {})
    return PerceptionCore(camera, camera_extrinsic(cfg), backend, params)


class PerceptionNode:
    def __init__(self, cfg, core: PerceptionCore, recorder: Optional[DatasetRecorder] = None):
        self.cfg, self.core, self.recorder = cfg, core, recorder
        self.lock = threading.Lock()
        self.last_scan = None
        self.stats = {"frames": 0, "persons": 0, "deleted_boxes": 0, "segment_ms": 0.0,
                      "skipped": {}}

    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        self.subs = []
        for topic, typ, cb, q in (("rt/nav/lio_odom", Odometry_, self._on_odom, 32),
                                  ("rt/nav/registered_cloud", PointCloud2_, self._on_cloud, 4),
                                  ("rt/nav/debug", String_, self._on_debug, 2)):
            s = ChannelSubscriber(topic, typ)
            s.Init(cb, q)
            self.subs.append(s)
        self.pubs = {}
        for key, topic, typ in (("person", "rt/nav/person_points", PointCloud2_),
                                ("delete", "rt/nav/map_delete", String_),
                                ("status", "rt/nav/perception_status", String_)):
            p = ChannelPublisher(topic, typ)
            p.Init()
            self.pubs[key] = p

    def _on_odom(self, m) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.core.add_pose(msgs.to_sec(m.header.stamp), [q.x, q.y, q.z, q.w], [p.x, p.y, p.z])

    def _on_cloud(self, m) -> None:
        static, dynamic = msgs.read_cloud_xyzl(m)
        pts = np.vstack([static, dynamic])
        t = msgs.to_sec(m.header.stamp)
        with self.lock:
            self.core.add_cloud(t, pts)
            self.last_scan = (t, pts, np.r_[np.zeros(len(static)), np.ones(len(dynamic))])

    def _on_debug(self, m) -> None:
        d = msgs.parse_json(m) or {}
        T = d.get("T_odom_lio") or {}
        if T.get("valid"):
            with self.lock:
                self.core.set_T_odom_lio(transform(quat_xyzw_to_matrix(T["q"]), T["t"]))

    def handle_frame(self, stamp: float, image: np.ndarray) -> None:
        with self.lock:
            res = self.core.process(stamp, image)
            scan = self.last_scan
            pose = self.core.poses.at(stamp)
        self.stats["frames"] += 1
        self.stats["segment_ms"] = round(res.segment_ms, 1)
        if res.skipped:
            self.stats["skipped"][res.skipped] = self.stats["skipped"].get(res.skipped, 0) + 1
        if res.person_points.shape[0]:
            self.pubs["person"].Write(msgs.make_cloud_xyz(res.person_points, "odom_lio", stamp))
            self.stats["persons"] += len(res.persons)
        if res.delete_boxes:
            self.pubs["delete"].Write(msgs.json_msg({"stamp": stamp, "frame": "odom_lio",
                                                     "boxes": [list(b) for b in res.delete_boxes]}))
            self.stats["deleted_boxes"] += len(res.delete_boxes)
        if self.recorder is not None and scan is not None:
            self.recorder.add(stamp, image, scan[1], scan[2], pose)

    def publish_status(self) -> None:
        self.pubs["status"].Write(msgs.json_msg(dict(self.stats, stamp=time.time(),
                                                     backend=self.core.backend.name)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--record", default=None, help="dataset root: save frames + scans for labelling")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(int(get(cfg, "system.dds_domain", 0)), net_if(cfg, args.net_if))
    core = build_core(cfg)
    recorder = None
    if args.record:
        recorder = DatasetRecorder(args.record, every_s=float(get(cfg, "perception.record_every_s", 0.5)))
        recorder.start({"K": get(cfg, "perception.camera.K"), "dist": get(cfg, "perception.camera.dist"),
                        "size": get(cfg, "perception.camera.size"), "T_imu_cam": core.T_imu_cam})
    node = PerceptionNode(cfg, core, recorder)
    node.start()
    cam = get(cfg, "perception.camera", {}) or {}
    src = TeleimagerSource(cam.get("host", "192.168.123.164"), int(cam.get("zmq_port", 55555)),
                           latency_s=float(cam.get("latency_s", 0.06)), crop=cam.get("crop", "left"))
    log.info("running (backend %s)", core.backend.name)
    seq, last_status = 0, 0.0
    try:
        while True:
            f = src.next(timeout=1.0, after_seq=seq)
            if f is not None:
                seq = f.seq
                node.handle_frame(f.stamp, f.image)
            if time.time() - last_status > 1.0:
                last_status = time.time()
                node.publish_status()
    except KeyboardInterrupt:
        src.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
