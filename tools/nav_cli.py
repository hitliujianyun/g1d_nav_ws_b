#!/usr/bin/env python3
"""Command line for g1d_nav: send goals, rotate, stop, e-stop, watch status / guard state.

    python3 tools/nav_cli.py goal 1.5 0.0 [--yaw 0.0] [--frame odom] [--wait]
    python3 tools/nav_cli.py rotate 1.57 --wait
    python3 tools/nav_cli.py simple 1.0 0.0          # "rotate then drive" baseline
    python3 tools/nav_cli.py stop
    python3 tools/nav_cli.py estop on|off
    python3 tools/nav_cli.py watch [--guard]
"""
import argparse
import json
import time

import _bootstrap  # noqa: F401

from g1d_nav_py import msgs
from g1d_nav_py.config import get, load_config, net_if


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("goal", "simple"):
        g = sub.add_parser(name)
        g.add_argument("x", type=float)
        g.add_argument("y", type=float)
        g.add_argument("--yaw", type=float, default=None)
        g.add_argument("--frame", default="base_at_stamp", choices=("base_at_stamp", "odom"))
        g.add_argument("--max-speed", type=float, default=None)
        g.add_argument("--timeout", type=float, default=None)
        g.add_argument("--wait", action="store_true")
    r = sub.add_parser("rotate")
    r.add_argument("yaw", type=float)
    r.add_argument("--wait", action="store_true")
    sub.add_parser("stop")
    e = sub.add_parser("estop")
    e.add_argument("state", choices=("on", "off"))
    w = sub.add_parser("watch")
    w.add_argument("--guard", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    from g1d_nav_py.nav_client import NavClient
    client = NavClient(domain, net_if(cfg, args.net_if))
    time.sleep(0.5)

    if args.cmd in ("goal", "simple"):
        extra = {}
        if args.max_speed is not None:
            extra["max_speed"] = args.max_speed
        if args.timeout is not None:
            extra["timeout_s"] = args.timeout
        mode = "navigate" if args.cmd == "goal" else "simple"
        gid = client.send_goal(args.x, args.y, args.yaw, frame=args.frame, mode=mode, **extra)
    elif args.cmd == "rotate":
        gid = client.rotate(args.yaw)
    elif args.cmd == "stop":
        gid = client.stop()
    elif args.cmd == "estop":
        pub = ChannelPublisher("rt/nav/estop", String_)
        pub.Init()
        time.sleep(0.3)
        pub.Write(msgs.string(args.state))
        print(f"estop {args.state} sent")
        return
    else:
        topic = "rt/nav/guard_state" if args.guard else "rt/nav/status"
        s = ChannelSubscriber(topic, String_)
        s.Init(lambda m: print(m.data, flush=True), 8)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            return

    print(f"sent {gid}")
    if getattr(args, "wait", False):
        final = client.wait(gid, timeout=300.0)
        print(json.dumps(final, indent=2) if final else "timeout")


if __name__ == "__main__":
    main()
