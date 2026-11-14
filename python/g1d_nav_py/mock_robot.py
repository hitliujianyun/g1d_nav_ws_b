"""Mock G1-D chassis on DDS for bench / loopback tests (no hardware).

Subscribes rt/cmd_vel_no_limit; integrates a diff-drive base with acceleration limits and a
command timeout; publishes rt/slamware_ros_sdk_server_node/odom (header stamp lagging the host
clock like SLAMWARE), rt/slamware_ros_sdk_server_node/scan (LaserScan_, 360 x 1 deg ray-cast in
a 2D box world), rt/hispeed_state; optionally rt/nav/obstacle_points (what g1d_nav would send)
and a fake chassis HTTP API (events, abort action).

    python -m g1d_nav_py.mock_robot --domain 1 --world corridor
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional, Sequence, Tuple

import numpy as np

Box2 = Tuple[float, float, float, float]   # x0, y0, x1, y1

SLAMWARE_STAMP_LAG_S = 181.0
CHASSIS_FOOTPRINT = (0.2785, 0.2625)        # half length / width of the 557 x 525 mm chassis


def raycast_boxes(origin: Sequence[float], angles: np.ndarray, boxes: Sequence[Box2],
                  max_range: float) -> np.ndarray:
    """Distance to the first box along each angle (inf if none within max_range)."""
    ox, oy = float(origin[0]), float(origin[1])
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(angles.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for x0, y0, x1, y1 in boxes:
            tx0, tx1 = (x0 - ox) / dx, (x1 - ox) / dx
            ty0, ty1 = (y0 - oy) / dy, (y1 - oy) / dy
            tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
            tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
            hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
            t = np.where(tmin > 0.0, tmin, tmax)
            best = np.where(hit & (t > 0) & (t < best), t, best)
    best[best > max_range] = np.inf
    return best


def footprint_gap(pose: Tuple[float, float, float], boxes: Sequence[Box2],
                  half: Tuple[float, float] = CHASSIS_FOOTPRINT, n: int = 12) -> float:
    """Smallest distance from the chassis rectangle outline to any box (<= 0: collision)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    u = np.linspace(-1.0, 1.0, n)
    edge = np.concatenate([np.c_[u * half[0], np.full(n, half[1])], np.c_[u * half[0], np.full(n, -half[1])],
                           np.c_[np.full(n, half[0]), u * half[1]], np.c_[np.full(n, -half[0]), u * half[1]]])
    wx = x + c * edge[:, 0] - s * edge[:, 1]
    wy = y + s * edge[:, 0] + c * edge[:, 1]
    gap = math.inf
    for x0, y0, x1, y1 in boxes:
        ddx = np.maximum(np.maximum(x0 - wx, 0.0), wx - x1)
        ddy = np.maximum(np.maximum(y0 - wy, 0.0), wy - y1)
        inside = (wx > x0) & (wx < x1) & (wy > y0) & (wy < y1)
        d = np.where(inside, -1.0, np.hypot(ddx, ddy))
        gap = min(gap, float(d.min()))
    return gap


def make_world(name: str) -> List[Box2]:
    walls = [(-2.0, -3.2, 12.0, -3.0), (-2.0, 3.0, 12.0, 3.2), (-2.2, -3.2, -2.0, 3.2), (12.0, -3.2, 12.2, 3.2)]
    if name == "wall":           # wall 3 m ahead
        return walls + [(3.0, -3.0, 3.2, 3.0)]
    if name == "corridor":       # 1.2 m corridor with a box in it
        return walls + [(0.8, 0.6, 10.0, 0.8), (0.8, -0.8, 10.0, -0.6), (5.0, -0.6, 5.4, -0.1)]
    if name == "empty":
        return walls
    raise ValueError(f"unknown world {name!r}")


@dataclass
class MockParams:
    rate_hz: float = 50.0
    scan_hz: float = 10.0
    odom_hz: float = 20.0
    cmd_timeout: float = 0.5          # unitree_sim_isaaclab stops after 0.5 s without commands
    max_acc: float = 1.0
    max_ang_acc: float = 2.0
    v_limit: float = 1.0
    w_limit: float = 1.0
    laser_xyz: Tuple[float, float, float] = (0.1871, -0.0161, 0.1009)
    scan_beams: int = 360
    scan_range: float = 25.0
    scan_noise: float = 0.005
    lift_height: float = 0.0
    obstacle_points: bool = False
    obstacle_range: float = 3.0
    box_height: float = 1.0
    stamp_lag: float = SLAMWARE_STAMP_LAG_S


class MockRobot:
    def __init__(self, boxes: Sequence[Box2], params: Optional[MockParams] = None,
                 start: Tuple[float, float, float] = (0.0, 0.0, 0.0), seed: int = 3):
        self.boxes = list(boxes)
        self.p = params or MockParams()
        self.pose = list(start)
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.t_cmd = -math.inf
        self.cmd_log: List[Tuple[float, float, float]] = []
        self.min_gap = math.inf
        self.collided = False
        self.events: List[dict] = []
        self.aborted_actions = 0
        self.rng = np.random.default_rng(seed)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._http: Optional[ThreadingHTTPServer] = None

    # ---- physics ----------------------------------------------------------------------------
    def on_cmd(self, v: float, w: float, now: float) -> None:
        with self.lock:
            self.cmd = (v, w)
            self.t_cmd = now
            self.cmd_log.append((now, v, w))

    def step(self, now: float, dt: float) -> None:
        with self.lock:
            vt, wt = self.cmd if now - self.t_cmd < self.p.cmd_timeout else (0.0, 0.0)
            vt = max(-self.p.v_limit, min(self.p.v_limit, vt))
            wt = max(-self.p.w_limit, min(self.p.w_limit, wt))
            self.v += max(-self.p.max_acc * dt, min(self.p.max_acc * dt, vt - self.v))
            self.w += max(-self.p.max_ang_acc * dt, min(self.p.max_ang_acc * dt, wt - self.w))
            x, y, th = self.pose
            th_mid = th + 0.5 * self.w * dt
            self.pose = [x + self.v * dt * math.cos(th_mid), y + self.v * dt * math.sin(th_mid),
                         math.atan2(math.sin(th + self.w * dt), math.cos(th + self.w * dt))]
            gap = footprint_gap(tuple(self.pose), self.boxes)
            self.min_gap = min(self.min_gap, gap)
            if gap <= 0.0:
                self.collided = True

    def scan_ranges(self) -> Tuple[float, float, np.ndarray]:
        x, y, th = self.pose
        lx, ly, _ = self.p.laser_xyz
        ox, oy = x + math.cos(th) * lx - math.sin(th) * ly, y + math.sin(th) * lx + math.cos(th) * ly
        inc = 2.0 * math.pi / self.p.scan_beams
        a_min = -math.pi
        angles = th + a_min + inc * np.arange(self.p.scan_beams)
        r = raycast_boxes((ox, oy), angles, self.boxes, self.p.scan_range)
        r = r + self.rng.normal(0.0, self.p.scan_noise, r.shape)
        return a_min, inc, r

    def obstacle_points_base(self) -> np.ndarray:
        """Box outlines sampled every 5 cm at a few heights, in base_link, within obstacle_range."""
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        pts = []
        for x0, y0, x1, y1 in self.boxes:
            nx = max(2, int((x1 - x0) / 0.05) + 1)
            ny = max(2, int((y1 - y0) / 0.05) + 1)
            xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
            outline = np.concatenate([np.c_[xs, np.full(nx, y0)], np.c_[xs, np.full(nx, y1)],
                                      np.c_[np.full(ny, x0), ys], np.c_[np.full(ny, x1), ys]])
            dx, dy = outline[:, 0] - x, outline[:, 1] - y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            keep = np.hypot(bx, by) < self.p.obstacle_range
            for z in (0.2, 0.6, min(1.0, self.p.box_height)):
                pts.append(np.c_[bx[keep], by[keep], np.full(int(keep.sum()), z)])
        return np.vstack(pts).astype(np.float32) if pts else np.zeros((0, 3), np.float32)

    def trigger_event(self, event_type: str) -> None:
        with self.lock:
            self.events.append({"type": event_type, "timestamp": int(time.time() * 1000),
                                "id": len(self.events)})

    # ---- DDS + HTTP -------------------------------------------------------------------------
    def start(self, http_port: Optional[int] = None) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

        from . import msgs
        from .slamware_idl import LaserScan_

        self._msgs = msgs
        self._LaserScan = LaserScan_
        self._Point32 = Point32_
        self.sub = ChannelSubscriber("rt/cmd_vel_no_limit", Twist_)
        self.sub.Init(lambda m: self.on_cmd(m.linear.x, m.angular.z, time.monotonic()), 8)
        self.odom_pub = ChannelPublisher("rt/slamware_ros_sdk_server_node/odom", Odometry_)
        self.odom_pub.Init()
        self.scan_pub = ChannelPublisher("rt/slamware_ros_sdk_server_node/scan", LaserScan_)
        self.scan_pub.Init()
        self.lift_pub = ChannelPublisher("rt/hispeed_state", Point32_)
        self.lift_pub.Init()
        self.obst_pub = None
        if self.p.obstacle_points:
            self.obst_pub = ChannelPublisher("rt/nav/obstacle_points", PointCloud2_)
            self.obst_pub.Init()
        if http_port is not None:
            self._start_http(http_port)
        t = threading.Thread(target=self._run, name="mock_robot", daemon=True)
        t.start()
        self._threads.append(t)

    def _start_http(self, port: int) -> None:
        robot = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # noqa: D401 - silence
                pass

            def _reply(self, obj) -> None:
                body = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                if self.path.startswith("/api/platform/v1/events"):
                    with robot.lock:
                        self._reply(list(robot.events))
                else:
                    self.send_error(404)

            def do_DELETE(self):  # noqa: N802
                if self.path.startswith("/api/core/motion/v1/actions/:current"):
                    robot.aborted_actions += 1
                    self._reply({})
                else:
                    self.send_error(404)

        self._http = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        t = threading.Thread(target=self._http.serve_forever, name="mock_http", daemon=True)
        t.start()
        self._threads.append(t)

    @property
    def http_url(self) -> Optional[str]:
        if self._http is None:
            return None
        return f"http://127.0.0.1:{self._http.server_address[1]}"

    def _run(self) -> None:
        msgs = self._msgs
        dt = 1.0 / self.p.rate_hz
        t_next = time.monotonic()
        next_scan = next_odom = t_next
        while not self._stop.is_set():
            now = time.monotonic()
            self.step(now, dt)
            wall = time.time()
            if now >= next_odom:
                next_odom += 1.0 / self.p.odom_hz
                with self.lock:
                    x, y, th = self.pose
                    v, w = self.v, self.w
                self.odom_pub.Write(msgs.make_odometry(wall - self.p.stamp_lag, x, y, th, v, w,
                                                       "odom", "base_link"))
                self.lift_pub.Write(self._Point32(x=0.0, y=float(self.p.lift_height), z=0.0))
            if now >= next_scan:
                next_scan += 1.0 / self.p.scan_hz
                with self.lock:
                    a_min, inc, r = self.scan_ranges()
                    obst = self.obstacle_points_base() if self.obst_pub else None
                self.scan_pub.Write(self._LaserScan(
                    header=msgs.header(wall - self.p.stamp_lag, "laser"), angle_min=a_min,
                    angle_max=a_min + inc * (len(r) - 1), angle_increment=inc, time_increment=0.0,
                    scan_time=1.0 / self.p.scan_hz, range_min=0.05, range_max=self.p.scan_range,
                    ranges=[float(v) for v in r], intensities=[]))
                if obst is not None:
                    self.obst_pub.Write(msgs.make_cloud_xyz(obst, "base_link", wall))
            t_next += dt
            delay = t_next - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                t_next = time.monotonic()

    def stop(self) -> None:
        self._stop.set()
        if self._http is not None:
            self._http.shutdown()
        for t in self._threads:
            t.join(timeout=1.0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--world", default="corridor", choices=("wall", "corridor", "empty"))
    ap.add_argument("--obstacle-points", action="store_true",
                    help="also publish rt/nav/obstacle_points (stand-in for g1d_nav)")
    ap.add_argument("--http-port", type=int, default=None)
    args = ap.parse_args(argv)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(args.domain, None if args.net_if in (None, "", "auto") else args.net_if)
    robot = MockRobot(make_world(args.world), MockParams(obstacle_points=args.obstacle_points))
    robot.start(args.http_port)
    print(f"mock robot running (domain {args.domain}, world {args.world}); Ctrl-C to stop")
    try:
        while True:
            time.sleep(1.0)
            print(f"pose=({robot.pose[0]:.2f},{robot.pose[1]:.2f},{robot.pose[2]:.2f}) "
                  f"v={robot.v:.2f} w={robot.w:.2f} min_gap={robot.min_gap:.2f} collided={robot.collided}")
    except KeyboardInterrupt:
        robot.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
