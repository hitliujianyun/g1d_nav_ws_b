#!/usr/bin/env python3
"""Measures rt/cmd_vel_no_limit behaviour (design doc §3.4) from the wheel odometry.

!! Moves the robot. Needs >= 3 m of free space ahead, someone at the e-stop, waist / arms
!! locked, and g1d_base_guard STOPPED (this tool writes rt/cmd_vel_no_limit directly).

Tests (each: accelerate to --speed for --drive-s, then):
  silence  : stop publishing             -> time / distance until the base stops (chassis timeout)
  zero     : publish zeros               -> braking time / distance
  clamp    : command 2x --speed and --wz -> reached speeds (silent clamping?)
  hold     : publish zeros for 5 s       -> manual check: can the base be pushed?

    python3 tools/cmd_vel_probe.py --net-if eth0 --tests silence,zero,clamp --i-know-it-moves
"""
import argparse
import json
import os
import threading
import time

import _bootstrap  # noqa: F401

from g1d_nav_py import msgs
from g1d_nav_py.config import get, load_config, net_if

RATE = 30.0


class Odom:
    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        self.log, self.lock = [], threading.Lock()
        self.sub = ChannelSubscriber("rt/slamware_ros_sdk_server_node/odom", Odometry_)
        self.sub.Init(self.cb, 16)

    def cb(self, m):
        x, y, yaw, v, w = msgs.odom_pose(m)
        with self.lock:
            self.log.append((time.monotonic(), x, y, yaw, v, w))

    def last(self):
        with self.lock:
            return self.log[-1] if self.log else None

    def since(self, t0):
        with self.lock:
            return [r for r in self.log if r[0] >= t0]


def drive(pub, v, w, seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        pub.Write(msgs.twist(v, w))
        time.sleep(1.0 / RATE)


def stop_stats(odom, t0, settle=3.0, v_eps=0.02, w_eps=0.03):
    time.sleep(settle)
    rows = odom.since(t0)
    if not rows:
        return {"error": "no odometry"}
    stopped = next((r for r in rows if abs(r[4]) < v_eps and abs(r[5]) < w_eps), None)
    x0, y0 = rows[0][1], rows[0][2]
    out = {"v_at_release": rows[0][4], "w_at_release": rows[0][5]}
    if stopped:
        out["stop_time_s"] = round(stopped[0] - t0, 3)
        out["stop_distance_m"] = round(((stopped[1] - x0) ** 2 + (stopped[2] - y0) ** 2) ** 0.5, 3)
    else:
        out["stop_time_s"] = None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--tests", default="silence,zero,clamp")
    ap.add_argument("--speed", type=float, default=0.2)
    ap.add_argument("--wz", type=float, default=0.4)
    ap.add_argument("--drive-s", type=float, default=2.0)
    ap.add_argument("--out", default="logs/cmd_vel_probe.json")
    ap.add_argument("--i-know-it-moves", action="store_true")
    args = ap.parse_args()
    if not args.i_know_it_moves:
        ap.error("this tool moves the robot: pass --i-know-it-moves")
    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_

    from g1d_nav_py.dds_discovery import find_writers
    ChannelFactoryInitialize(domain, net_if(cfg, args.net_if))
    others = find_writers("rt/cmd_vel_no_limit", domain, 1.5)
    if others:
        raise SystemExit(f"other writers on rt/cmd_vel_no_limit ({others}): stop g1d_base_guard first")
    odom = Odom()
    pub = ChannelPublisher("rt/cmd_vel_no_limit", Twist_)
    pub.Init()
    time.sleep(1.0)
    if odom.last() is None:
        raise SystemExit("no wheel odometry on rt/slamware_ros_sdk_server_node/odom")

    results = {"speed": args.speed, "wz": args.wz}
    for test in [t.strip() for t in args.tests.split(",") if t.strip()]:
        input(f"\n[{test}] clear space ahead? press Enter to run (Ctrl-C aborts) ")
        if test == "silence":
            drive(pub, args.speed, 0.0, args.drive_s)
            results[test] = stop_stats(odom, time.monotonic())
        elif test == "zero":
            drive(pub, args.speed, 0.0, args.drive_s)
            t0 = time.monotonic()
            threading.Thread(target=drive, args=(pub, 0.0, 0.0, 3.0), daemon=True).start()
            results[test] = stop_stats(odom, t0)
        elif test == "clamp":
            t0 = time.monotonic()
            drive(pub, 2.0 * args.speed, 0.0, args.drive_s)
            v_max = max((abs(r[4]) for r in odom.since(t0)), default=None)
            drive(pub, 0.0, 0.0, 1.5)
            t0 = time.monotonic()
            drive(pub, 0.0, 2.0 * args.wz, args.drive_s)
            w_max = max((abs(r[5]) for r in odom.since(t0)), default=None)
            drive(pub, 0.0, 0.0, 1.5)
            results[test] = {"v_cmd": 2.0 * args.speed, "v_reached": v_max,
                             "w_cmd": 2.0 * args.wz, "w_reached": w_max}
        elif test == "hold":
            print("publishing zeros for 5 s: try to push the robot by hand now")
            drive(pub, 0.0, 0.0, 5.0)
            print("stopped publishing: try to push again")
            locked = input("was the base locked while zeros were published? [y/n] ").strip().lower()
            results[test] = {"locked_while_zero": locked.startswith("y")}
        else:
            print(f"unknown test {test!r}")
            continue
        print(json.dumps(results[test], indent=2))

    if "silence" in results and results["silence"].get("stop_time_s"):
        results["suggested_guard"] = {
            "zero_hold": max(1.0, round(results["silence"]["stop_time_s"] + 0.3, 1)),
            "note": "zero_hold should exceed the chassis' own stop time after silence"}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nreport: {args.out}")


if __name__ == "__main__":
    main()
