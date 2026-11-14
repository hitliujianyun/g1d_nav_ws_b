"""g1d_base_guard: the ONLY writer of rt/cmd_vel_no_limit (design doc §6.8).

  rt/nav/cmd_vel (Twist_)                         -> GuardCore -> rt/cmd_vel_no_limit (Twist_)
  rt/slamware_ros_sdk_server_node/scan (LaserScan_) -> reflex + relay rt/nav/chassis_scan
                                                     (PointCloud2_ x/y/z, base_link)
  rt/nav/obstacle_points (PointCloud2_, base_link) -> reflex
  rt/nav/estop (String_ "on" / "off"), rt/wirelesscontroller, chassis HTTP events -> latches
  rt/nav/teleop_cmd (Twist_, web teleop deadman) -> overrides nav while fresh
  rt/nav/param_set (String_ JSON {"target": "guard", "name", "value", "id"}) -> rt/nav/param_state
  rt/nav/guard_state (String_ JSON, 10 Hz), logs/guard-<time>.jsonl

    python -m g1d_nav_py.guard_node [--config config/g1d_nav.yaml] [--net-if eth0] [--domain 0]
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import threading
import time
from typing import Any, Dict, Optional

import numpy as np

from . import msgs
from .chassis_http import ChassisHttp, EventTracker
from .config import get, load_config, net_if
from .guard_core import GuardCore, GuardParams, scan_to_points

log = logging.getLogger("g1d.guard")

T_CMD_OUT = "rt/cmd_vel_no_limit"
T_CMD_IN = "rt/nav/cmd_vel"
T_SCAN = "rt/slamware_ros_sdk_server_node/scan"
T_SCAN_RELAY = "rt/nav/chassis_scan"
T_OBSTACLES = "rt/nav/obstacle_points"
T_ESTOP = "rt/nav/estop"
T_JOY = "rt/wirelesscontroller"
T_STATE = "rt/nav/guard_state"
T_TELEOP = "rt/nav/teleop_cmd"
T_PARAM_SET = "rt/nav/param_set"
T_PARAM_STATE = "rt/nav/param_state"


class BaseGuardNode:
    def __init__(self, cfg: Dict[str, Any], *, use_http: bool = True,
                 chassis_url: Optional[str] = None, log_dir: Optional[str] = None,
                 clock=time.monotonic, wall=time.time):
        self.cfg = cfg
        self.p = GuardParams.from_config(cfg)
        self.clock = clock
        self.wall = wall
        self.core = GuardCore(self.p, clock())
        self.lock = threading.Lock()
        xyz = get(cfg, "robot.chassis_laser.xyz", [0.1871, -0.0161, 0.1009])
        rpy = get(cfg, "robot.chassis_laser.rpy", [0.0, 0.0, 0.0])
        self.laser_xyz = (float(xyz[0]), float(xyz[1]),
                          float(xyz[2]) + float(get(cfg, "kinematics.base_to_agv_z", 0.0)))
        self.laser_yaw = float(rpy[2])
        self.use_http = use_http
        self.http = ChassisHttp(chassis_url or get(cfg, "system.chassis_http", "http://192.168.123.163:1448"))
        self.events = EventTracker()
        self.log_dir = log_dir
        self._log_file = None
        self._stop = threading.Event()
        self._subs = []
        self.pub = None
        self.relay_pub = None
        self.state_pub = None

    # ---- DDS --------------------------------------------------------------------------------
    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

        from .slamware_idl import LaserScan_

        self.pub = ChannelPublisher(T_CMD_OUT, Twist_)
        self.pub.Init()
        self.relay_pub = ChannelPublisher(T_SCAN_RELAY, PointCloud2_)
        self.relay_pub.Init()
        self.state_pub = ChannelPublisher(T_STATE, String_)
        self.state_pub.Init()
        self.param_pub = ChannelPublisher(T_PARAM_STATE, String_)
        self.param_pub.Init()
        for topic, typ, cb in ((T_CMD_IN, Twist_, self._on_cmd), (T_SCAN, LaserScan_, self._on_scan),
                               (T_OBSTACLES, PointCloud2_, self._on_cloud),
                               (T_ESTOP, String_, self._on_estop),
                               (T_JOY, WirelessController_, self._on_joy),
                               (T_TELEOP, Twist_, self._on_teleop),
                               (T_PARAM_SET, String_, self._on_param)):
            s = ChannelSubscriber(topic, typ)
            s.Init(cb, 1)
            self._subs.append(s)
        if self.use_http:
            self.http.abort_current_action()
            threading.Thread(target=self._poll_events, name="guard_events", daemon=True).start()
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            name = time.strftime("guard-%Y%m%d-%H%M%S.jsonl")
            self._log_file = open(os.path.join(self.log_dir, name), "a", encoding="utf-8")

    def _on_cmd(self, m) -> None:
        with self.lock:
            self.core.on_cmd(m.linear.x, m.angular.z, self.clock())

    def _on_scan(self, m) -> None:
        pts = scan_to_points(m.angle_min, m.angle_increment, m.ranges, self.laser_xyz,
                             self.laser_yaw, max(0.05, float(m.range_min)),
                             float(m.range_max) if m.range_max > 0 else 30.0)
        with self.lock:
            self.core.on_scan_points(pts, self.clock())
        self.relay_pub.Write(msgs.make_cloud_xyz(pts, "base_link", self.wall()))

    def _on_cloud(self, m) -> None:
        try:
            pts = msgs.read_cloud_xyz(m)
        except ValueError:
            return
        with self.lock:
            self.core.on_cloud_points(pts, self.clock())

    def _on_estop(self, m) -> None:
        cmd = str(m.data).strip().lower()
        if cmd in ("on", "off"):
            with self.lock:
                self.core.on_estop(cmd == "on", "topic")
            log.warning("estop %s via %s", cmd, T_ESTOP)

    def _on_joy(self, m) -> None:
        with self.lock:
            self.core.on_joystick(m.lx, m.ly, m.rx, m.ry, self.clock())

    def _on_teleop(self, m) -> None:
        with self.lock:
            self.core.on_teleop(m.linear.x, m.angular.z, self.clock())

    def _on_param(self, m) -> None:
        req = msgs.parse_json(m)
        if not req or req.get("target") != "guard":
            return
        with self.lock:
            err = self.core.set_param(str(req.get("name")), req.get("value"))
        if err is None:
            log.info("param %s = %s", req.get("name"), req.get("value"))
        self.publish_params({"id": req.get("id", ""), "name": req.get("name"), "ok": err is None,
                             "error": err or ""})

    def publish_params(self, ack: Optional[Dict[str, Any]] = None) -> None:
        with self.lock:
            msg = {"target": "guard", "params": self.core.params()}
        if ack is not None:
            msg["ack"] = ack
        self.param_pub.Write(msgs.json_msg(msg))

    def _poll_events(self) -> None:
        while not self._stop.is_set():
            try:
                for ev in self.events.update(self.http.events()):
                    log.warning("chassis event %s", ev)
                    with self.lock:
                        self.core.on_event(ev)
            except Exception:  # noqa: BLE001 - chassis HTTP may be unreachable
                pass
            self._stop.wait(0.5)

    # ---- loop -------------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        now = self.clock()
        with self.lock:
            out = self.core.step(now, dt)
        if out.publish:
            self.pub.Write(msgs.twist(out.v, out.w))

    def publish_state(self) -> Dict[str, Any]:
        with self.lock:
            st = self.core.state(self.clock())
        st["stamp"] = self.wall()
        self.state_pub.Write(msgs.json_msg(st))
        if self._log_file:
            self._log_file.write(json.dumps(st) + "\n")
        return st

    def run(self) -> None:
        dt = 1.0 / self.p.rate_hz
        k = 0
        state_every = max(1, int(round(self.p.rate_hz / 10.0)))
        t_next = self.clock()
        while not self._stop.is_set():
            self.step(dt)
            k += 1
            if k % state_every == 0:
                self.publish_state()
            if k % max(1, int(self.p.rate_hz)) == 0:
                self.publish_params()
            t_next += dt
            delay = t_next - self.clock()
            if delay > 0:
                self._stop.wait(delay)
            else:
                t_next = self.clock()
        self.shutdown()

    def stop(self) -> None:
        self._stop.set()

    def shutdown(self) -> None:
        with self.lock:
            was_moving = self.core.v != 0.0 or self.core.w != 0.0
        if was_moving and self.pub is not None:
            for _ in range(3):
                self.pub.Write(msgs.twist(0.0, 0.0))
                time.sleep(0.02)
        if self._log_file:
            self._log_file.close()
            self._log_file = None


def check_single_writer(domain: int, policy: str) -> bool:
    """Design doc §11: refuse to start if another process already writes rt/cmd_vel_no_limit."""
    try:
        from .dds_discovery import find_writers
        writers = find_writers(T_CMD_OUT, domain, 1.5)
    except Exception as e:  # noqa: BLE001
        log.warning("writer discovery unavailable: %s", e)
        return True
    if writers:
        log.error("other writers on %s: %s", T_CMD_OUT, writers)
        return policy != "refuse"
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--no-http", action="store_true", help="do not poll chassis HTTP events")
    ap.add_argument("--chassis-url", default=None)
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--other-writers", choices=("refuse", "warn"), default=None)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    cfg = load_config(args.config)
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))
    nif = net_if(cfg, args.net_if)
    policy = args.other_writers or get(cfg, "guard.other_writers", "refuse")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(domain, nif)
    if not check_single_writer(domain, policy):
        return 2
    node = BaseGuardNode(cfg, use_http=not args.no_http, chassis_url=args.chassis_url,
                         log_dir=args.log_dir)
    node.start()
    signal.signal(signal.SIGINT, lambda *_: node.stop())
    signal.signal(signal.SIGTERM, lambda *_: node.stop())
    log.info("g1d_base_guard running (domain %d, if %s, %.0f Hz)", domain, nif, node.p.rate_hz)
    node.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
