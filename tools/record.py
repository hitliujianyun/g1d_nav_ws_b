#!/usr/bin/env python3
"""Records DDS topics to a gzipped JSONL file (one message per line: topic, receipt time, fields).
Byte payloads (PointCloud2.data etc.) are stored base64-encoded.

    python3 tools/record.py --net-if eth0 -o logs/run1.jsonl.gz [--topics scan,odom,cmd,status]
    python3 tools/record.py --list
"""
import argparse
import base64
import dataclasses
import gzip
import json
import threading
import time

import _bootstrap  # noqa: F401

from g1d_nav_py.config import get, load_config, net_if


def topic_table():
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import OccupancyGrid_, Odometry_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, MotorState_

    from g1d_nav_py.slamware_idl import LaserScan_
    return {
        "scan": ("rt/slamware_ros_sdk_server_node/scan", LaserScan_),
        "odom": ("rt/slamware_ros_sdk_server_node/odom", Odometry_),
        "hispeed": ("rt/hispeed_state", Point32_),
        "lowstate": ("rt/lowstate", LowState_),
        "waistpitch": ("rt/waistpitchstate", MotorState_),
        "cmd": ("rt/cmd_vel_no_limit", Twist_),
        "nav_cmd": ("rt/nav/cmd_vel", Twist_),
        "status": ("rt/nav/status", String_),
        "guard": ("rt/nav/guard_state", String_),
        "goal": ("rt/nav/goal", String_),
        "lio_odom": ("rt/nav/lio_odom", Odometry_),
        "lio_health": ("rt/nav/lio_health", String_),
        "base_odom": ("rt/nav/base_odom", Odometry_),
        "obstacles": ("rt/nav/obstacle_points", PointCloud2_),
        "registered_cloud": ("rt/nav/registered_cloud", PointCloud2_),
        "costmap": ("rt/nav/local_costmap", OccupancyGrid_),
    }


DEFAULT = "scan,odom,hispeed,cmd,nav_cmd,status,guard,goal,lio_odom,lio_health,base_odom,obstacles"


def to_jsonable(v):
    if dataclasses.is_dataclass(v):
        return {f.name: to_jsonable(getattr(v, f.name)) for f in dataclasses.fields(v)}
    if isinstance(v, (bytes, bytearray)):
        return {"b64": base64.b64encode(bytes(v)).decode()}
    if isinstance(v, (list, tuple)):
        if len(v) > 64 and all(isinstance(x, int) and 0 <= x < 256 for x in v[:64]):
            return {"b64": base64.b64encode(bytes(v)).decode()}
        return [to_jsonable(x) for x in v]
    if isinstance(v, float) and v != v:
        return None
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--topics", default=DEFAULT)
    ap.add_argument("-o", "--out", default=time.strftime("logs/rec-%Y%m%d-%H%M%S.jsonl.gz"))
    ap.add_argument("--duration", type=float, default=0.0, help="0 = until Ctrl-C")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    table = topic_table()
    if args.list:
        for k, (t, typ) in table.items():
            print(f"{k:<18} {t:<42} {typ.__name__}")
        return
    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    ChannelFactoryInitialize(domain, net_if(cfg, args.net_if))

    lock, counts = threading.Lock(), {}
    out = gzip.open(args.out, "wt", encoding="utf-8")
    subs = []
    for key in [k.strip() for k in args.topics.split(",") if k.strip()]:
        topic, typ = table[key]

        def cb(m, topic=topic):
            line = json.dumps({"topic": topic, "t": time.time(), "msg": to_jsonable(m)})
            with lock:
                out.write(line + "\n")
                counts[topic] = counts.get(topic, 0) + 1
        s = ChannelSubscriber(topic, typ)
        s.Init(cb, 64)
        subs.append(s)
    print(f"recording {len(subs)} topics to {args.out} (Ctrl-C to stop)")
    t0 = time.monotonic()
    try:
        while not args.duration or time.monotonic() - t0 < args.duration:
            time.sleep(1.0)
            with lock:
                print(" ".join(f"{t.split('/')[-1]}={n}" for t, n in sorted(counts.items())), flush=True)
    except KeyboardInterrupt:
        pass
    with lock:
        out.close()
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
