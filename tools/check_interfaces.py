#!/usr/bin/env python3
"""Phase-0 interface probe (design doc §3, §15): listens to every robot / g1d topic for a few
seconds and reports rate, header-stamp offset to the host clock and a decoded sample; checks
writers on rt/cmd_vel_no_limit and the chassis HTTP API. Writes a JSON report.

    python3 tools/check_interfaces.py --net-if eth0 [--duration 5] [--out logs/interfaces.json]
"""
import argparse
import json
import math
import os
import threading
import time

import _bootstrap  # noqa: F401

from g1d_nav_py import msgs
from g1d_nav_py.chassis_http import ChassisHttp
from g1d_nav_py.config import get, load_config, net_if


def summarise(name, m):
    """Short, JSON-friendly digest of one message."""
    if name == "odom":
        x, y, yaw, v, w = msgs.odom_pose(m)
        return {"x": x, "y": y, "yaw": yaw, "v": v, "w": w, "frame": m.header.frame_id,
                "child": m.child_frame_id}
    if name == "scan":
        r = [v for v in m.ranges if math.isfinite(v) and v > 0.0]
        return {"beams": len(m.ranges), "valid": len(r), "angle_min": m.angle_min,
                "angle_increment": m.angle_increment, "range_min": m.range_min,
                "range_max": m.range_max, "min_range": min(r) if r else None,
                "frame": m.header.frame_id}
    if name == "hispeed":
        return {"height_y": m.y, "speed_z": m.z, "x": m.x}
    if name == "lowstate":
        q = [ms.q for ms in m.motor_state]
        return {"waist_yaw_q12": q[12], "arms_q15_28": [round(v, 4) for v in q[15:29]]}
    if name == "waistpitch":
        return {"q": m.q, "dq": m.dq}
    if name == "joy":
        return {"lx": m.lx, "ly": m.ly, "rx": m.rx, "ry": m.ry, "keys": m.keys}
    if name.endswith("cloud") or name in ("chassis_scan", "obstacle_points", "registered_cloud"):
        fields = [(f.name, f.offset, f.datatype) for f in m.fields]
        return {"width": m.width, "height": m.height, "point_step": m.point_step,
                "fields": fields, "frame": m.header.frame_id}
    if name.endswith("imu"):
        a = m.linear_acceleration
        return {"acc": [a.x, a.y, a.z], "acc_norm": math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2),
                "gyr": [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]}
    if hasattr(m, "data") and isinstance(m.data, str):
        return {"data": m.data[:300]}
    return {}


class Probe:
    def __init__(self, name, topic, typ, has_header=True):
        self.name, self.topic, self.has_header = name, topic, has_header
        self.t, self.offsets, self.sample, self.lock = [], [], None, threading.Lock()
        from unitree_sdk2py.core.channel import ChannelSubscriber
        self.sub = ChannelSubscriber(topic, typ)
        self.sub.Init(self.cb, 8)

    def cb(self, m):
        now = time.time()
        with self.lock:
            self.t.append(now)
            if self.has_header and hasattr(m, "header"):
                self.offsets.append(now - msgs.to_sec(m.header.stamp))
            if self.sample is None or len(self.t) % 10 == 0:
                self.sample = m

    def report(self, duration):
        with self.lock:
            n = len(self.t)
            rate = (n - 1) / (self.t[-1] - self.t[0]) if n > 2 else None
            off = sorted(self.offsets)
            r = {"topic": self.topic, "count": n, "rate_hz": round(rate, 2) if rate else None,
                 "stamp_offset_s": round(off[len(off) // 2], 4) if off else None,
                 "stamp_offset_spread_s": round(off[-1] - off[0], 4) if len(off) > 1 else None}
            if self.sample is not None:
                try:
                    r["sample"] = summarise(self.name, self.sample)
                except Exception as e:  # noqa: BLE001
                    r["sample_error"] = repr(e)
        return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--out", default="logs/interfaces.json")
    ap.add_argument("--chassis-url", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, MotorState_

    from g1d_nav_py.dds_discovery import find_writers
    from g1d_nav_py.extra_idl import Imu_
    from g1d_nav_py.slamware_idl import LaserScan_

    ChannelFactoryInitialize(domain, net_if(cfg, args.net_if))
    probes = [
        Probe("odom", "rt/slamware_ros_sdk_server_node/odom", Odometry_),
        Probe("scan", "rt/slamware_ros_sdk_server_node/scan", LaserScan_),
        Probe("hispeed", "rt/hispeed_state", Point32_, False),
        Probe("lowstate", "rt/lowstate", LowState_, False),
        Probe("waistpitch", "rt/waistpitchstate", MotorState_, False),
        Probe("joy", "rt/wirelesscontroller", WirelessController_, False),
        Probe("utlidar_cloud", get(cfg, "lidar.unitree_cloud_topic", "rt/utlidar/cloud_livox_mid360"), PointCloud2_),
        Probe("utlidar_imu", get(cfg, "lidar.unitree_imu_topic", "rt/utlidar/imu_livox_mid360"), Imu_),
        Probe("lio_health", "rt/nav/lio_health", String_, False),
        Probe("nav_status", "rt/nav/status", String_, False),
        Probe("guard_state", "rt/nav/guard_state", String_, False),
        Probe("chassis_scan", "rt/nav/chassis_scan", PointCloud2_),
        Probe("obstacle_points", "rt/nav/obstacle_points", PointCloud2_),
    ]
    print(f"listening {args.duration:.1f} s on domain {domain} ...")
    writers = find_writers("rt/cmd_vel_no_limit", domain, 1.0)
    time.sleep(max(0.0, args.duration - 1.0))

    report = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "domain": domain,
              "topics": {p.name: p.report(args.duration) for p in probes},
              "cmd_vel_no_limit_writers": writers}
    http = ChassisHttp(args.chassis_url or get(cfg, "system.chassis_http", "http://192.168.123.163:1448"),
                       timeout=1.0)
    try:
        ev = http.events()
        report["chassis_http"] = {"ok": True, "events": len(ev), "last": ev[-3:]}
    except Exception as e:  # noqa: BLE001
        report["chassis_http"] = {"ok": False, "error": repr(e)}

    for name, r in report["topics"].items():
        flag = "OK " if r["count"] else "-- "
        print(f"{flag}{r['topic']:<44} n={r['count']:<5} rate={r['rate_hz']} Hz "
              f"stamp_offset={r['stamp_offset_s']} s")
    print(f"writers on rt/cmd_vel_no_limit: {writers or 'none'}")
    print(f"chassis HTTP: {report['chassis_http']}")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"report: {args.out}")


if __name__ == "__main__":
    main()
