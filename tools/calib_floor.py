#!/usr/bin/env python3
"""Mid-360S extrinsic check on a flat floor (docs/CALIBRATION.md).

Robot static on a level floor, waist / lift at the navigation posture, >= 2 m of free floor in
view. g1d_lio must run with debug.publish_lidar_cloud: true (or pass --topic for another
LiDAR-frame cloud). Prints the measured roll / pitch / height of the LiDAR and, with
--yaw-from-scan, the yaw that best overlays a horizontal slice on the chassis LiDAR scan.

    build/src/tools/g1d_fk config/g1d_nav.yaml <lift_y>        # -> base->livox xyz / rpy
    python3 tools/calib_floor.py --fk-xyz 0.107 0.000 1.230 --fk-rpy 3.1416 -0.04 0 --yaw-from-scan
"""
import argparse
import math
import threading
import time

import numpy as np

import _bootstrap  # noqa: F401

from g1d_nav_py import msgs
from g1d_nav_py.calib import estimate_yaw, fit_floor, level_rotation, matrix_to_rpy, rpy_to_matrix
from g1d_nav_py.config import get, load_config, net_if


class Collector:
    def __init__(self, topic, n_frames):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        self.frames, self.n, self.done = [], n_frames, threading.Event()
        self.sub = ChannelSubscriber(topic, PointCloud2_)
        self.sub.Init(self.cb, 4)

    def cb(self, m):
        if len(self.frames) < self.n:
            self.frames.append(msgs.read_cloud_xyz(m))
            if len(self.frames) >= self.n:
                self.done.set()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--topic", default="rt/nav/livox_cloud")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--fk-xyz", type=float, nargs=3, required=True, help="base->livox xyz from g1d_fk")
    ap.add_argument("--fk-rpy", type=float, nargs=3, required=True, help="base->livox rpy from g1d_fk")
    ap.add_argument("--yaw-from-scan", action="store_true")
    ap.add_argument("--band", type=float, default=0.03, help="slice half height around the chassis LiDAR")
    args = ap.parse_args()
    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(domain, net_if(cfg, args.net_if))
    lidar = Collector(args.topic, args.frames)
    scan = Collector("rt/nav/chassis_scan", 5) if args.yaw_from_scan else None
    if not lidar.done.wait(20.0):
        raise SystemExit(f"only {len(lidar.frames)} frames on {args.topic} (debug.publish_lidar_cloud?)")
    pts = np.vstack(lidar.frames)
    pts = pts[np.linalg.norm(pts, axis=1) < 6.0]

    r_cfg = rpy_to_matrix(*args.fk_rpy)
    up_hint = r_cfg.T @ np.array([0.0, 0.0, 1.0])
    fit = fit_floor(pts, up_hint)
    if fit is None:
        raise SystemExit("no floor plane found (free floor in view? config rpy roughly right?)")
    n, h, inliers = fit
    r_new = level_rotation(r_cfg, n)
    rpy_new = matrix_to_rpy(r_new)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n, up_hint))))))
    print(f"floor: {inliers} inliers of {pts.shape[0]}, normal in LiDAR frame {np.round(n, 4)}")
    print(f"LiDAR height above floor: measured {h:.4f} m, FK {args.fk_xyz[2]:.4f} m "
          f"(error {h - args.fk_xyz[2]:+.4f} m)")
    print(f"base->livox rpy: config {np.round(args.fk_rpy, 4)} -> levelled {np.round(rpy_new, 4)} "
          f"(tilt error {tilt:.2f} deg)")

    if scan is not None:
        if not scan.done.wait(5.0):
            raise SystemExit("no rt/nav/chassis_scan (is g1d_base_guard running?)")
        scan_xy = np.vstack(scan.frames)[:, :2]
        laser_z = float(get(cfg, "robot.chassis_laser.xyz", [0, 0, 0.1009])[2]) + \
            float(get(cfg, "kinematics.base_to_agv_z", 0.0))
        t = np.array([args.fk_xyz[0], args.fk_xyz[1], h])
        base = pts @ r_new.T + t
        band = base[np.abs(base[:, 2] - laser_z) < args.band][:, :2]
        if band.shape[0] < 50:
            raise SystemExit(f"only {band.shape[0]} LiDAR points at the chassis LiDAR height")
        yaw, resid = estimate_yaw(band, scan_xy, t[:2])
        print(f"yaw correction vs chassis scan: {math.degrees(yaw):+.2f} deg "
              f"(mean residual {resid:.3f} m, {band.shape[0]} slice points)")
        rpy_new = matrix_to_rpy(rpy_to_matrix(0.0, 0.0, yaw) @ r_new)
    print("\nWith the waist at zero (base->head rotation = identity) update kinematics.head_to_livox:")
    print(f"  rpy: [{rpy_new[0]:.4f}, {rpy_new[1]:.4f}, {rpy_new[2]:.4f}]")
    print(f"  z  : current z {h - args.fk_xyz[2]:+.4f} m (or fix base_to_agv_z / lift_offset)")


if __name__ == "__main__":
    main()
