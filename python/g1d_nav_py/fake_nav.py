"""Stand-in for g1d_lio + g1d_nav on a development PC (no hardware, no C++ DDS build).

Publishes what the real processes publish -- rt/nav/registered_cloud (ray-cast Mid-360S in a 3D
box world, LIO frame), lio_odom, lio_health, base_odom, status, debug, obstacle_points,
local/global_costmap, param_state -- and executes rt/nav/goal with A* + pure pursuit through
rt/nav/cmd_vel, so g1d_base_guard and g1d_web can be exercised end to end. By default it also
runs the mock chassis (MockRobot) in the same process.

    python -m g1d_nav_py.fake_nav --domain 1 [--world lab] [--no-mock]
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import msgs
from .calib import rpy_to_matrix
from .mock_robot import MockParams, MockRobot

Box3 = Tuple[float, float, float, float, float, float]   # x0 y0 z0 x1 y1 z1

LIDAR_XYZ = np.array([0.107, 0.0, 1.23])                 # base -> Mid-360S (inverted, head)
LIDAR_RPY = (math.pi, -0.04, 0.0)
CEILING = 2.8
Z_BAND = (0.08, 1.85)
INFLATION_R = 0.55
INSCRIBED = 0.30
COST_SCALING = 3.0


def make_world3d(name: str) -> List[Box3]:
    t = 0.2
    walls = [(-2.0 - t, -4.0 - t, 0, 10.0 + t, -4.0, 2.8), (-2.0 - t, 4.0, 0, 10.0 + t, 4.0 + t, 2.8),
             (-2.0 - t, -4.0, 0, -2.0, 4.0, 2.8), (10.0, -4.0, 0, 10.0 + t, 4.0, 2.8)]
    if name == "empty":
        return walls
    if name == "lab":
        return walls + [
            (2.0, -0.5, 0, 2.6, 0.4, 0.8),                     # box on the straight line
            (4.5, 1.5, 0.72, 5.9, 2.9, 0.76),                  # table top (overhang)
            (4.5, 1.5, 0, 4.55, 1.55, 0.72), (5.85, 1.5, 0, 5.9, 1.55, 0.72),
            (4.5, 2.85, 0, 4.55, 2.9, 0.72), (5.85, 2.85, 0, 5.9, 2.9, 0.72),
            (6.5, -2.2, 0, 6.8, -1.9, 2.8),                    # pillar
            (7.5, 0.5, 0, 10.0, 0.8, 1.2),                     # partition
            (0.5, -4.0, 0, 1.5, -3.2, 1.0),                    # cabinet
        ]
    raise ValueError(f"unknown world {name!r}")


def chassis_boxes(boxes: Sequence[Box3], laser_z: float = 0.1009):
    """2D boxes seen by the chassis LiDAR plane (for MockRobot)."""
    return [(b[0], b[1], b[3], b[4]) for b in boxes if b[2] <= laser_z <= b[5]]


def raycast3d(origin: np.ndarray, dirs: np.ndarray, boxes: Sequence[Box3], max_range: float) -> np.ndarray:
    best = np.full(dirs.shape[0], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
        for b in boxes:
            lo = (np.array(b[:3]) - origin) * inv
            hi = (np.array(b[3:]) - origin) * inv
            tmin = np.nanmax(np.minimum(lo, hi), axis=1)
            tmax = np.nanmin(np.maximum(lo, hi), axis=1)
            hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
            best = np.where(hit & (tmin < best), tmin, best)
        for zp in (0.0, CEILING):
            t = (zp - origin[2]) * inv[:, 2]
            best = np.where((t > 0) & (t < best), t, best)
    best[best > max_range] = np.inf
    return best


class StaticCostmap:
    """Inflated cost grid of the known world (Nav2 formula), values 0..254 like Costmap2D."""

    def __init__(self, boxes: Sequence[Box3], res: float = 0.05):
        xs = [b[0] for b in boxes] + [b[3] for b in boxes]
        ys = [b[1] for b in boxes] + [b[4] for b in boxes]
        self.res = res
        self.x0, self.y0 = min(xs) - 1.0, min(ys) - 1.0
        self.w = int(math.ceil((max(xs) + 1.0 - self.x0) / res))
        self.h = int(math.ceil((max(ys) + 1.0 - self.y0) / res))
        occ = np.zeros((self.h, self.w), bool)
        for b in boxes:
            if b[5] < Z_BAND[0] or b[2] > Z_BAND[1]:
                continue
            i0, i1 = self._ix(b[0]), self._ix(b[3])
            j0, j1 = self._iy(b[1]), self._iy(b[4])
            occ[j0:j1 + 1, i0:i1 + 1] = True
        dist = self._distance(occ)
        cost = np.where(dist <= INSCRIBED, 253.0,
                        252.0 * np.exp(-COST_SCALING * (dist - INSCRIBED)))
        cost[dist > INFLATION_R] = 0.0
        cost[occ] = 254.0
        self.cost = cost.astype(np.uint8)

    def _ix(self, x):
        return int(np.clip((x - self.x0) / self.res, 0, self.w - 1))

    def _iy(self, y):
        return int(np.clip((y - self.y0) / self.res, 0, self.h - 1))

    def _distance(self, occ: np.ndarray) -> np.ndarray:
        """Distance (m) to the nearest occupied cell, capped at INFLATION_R (brute force on a
        dilation ring; the world is static so this runs once)."""
        r = int(math.ceil(INFLATION_R / self.res)) + 1
        dist = np.full(occ.shape, np.inf)
        dist[occ] = 0.0
        oy, ox = np.nonzero(occ)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                d = math.hypot(dx, dy) * self.res
                if d > INFLATION_R + self.res:
                    continue
                yy, xx = oy + dy, ox + dx
                ok = (yy >= 0) & (yy < self.h) & (xx >= 0) & (xx < self.w)
                np.minimum.at(dist, (yy[ok], xx[ok]), d)
        return dist

    def at(self, x: float, y: float) -> int:
        return int(self.cost[self._iy(y), self._ix(x)])

    def ros_window(self, cx: float, cy: float, size: float, res: float, seen: np.ndarray):
        """OccupancyGrid-style window (ROS values 0..100, 255 unknown) centred on (cx, cy)."""
        n = int(round(size / res))
        ox, oy = cx - size / 2, cy - size / 2
        gx = ox + (np.arange(n) + 0.5) * res
        gy = oy + (np.arange(n) + 0.5) * res
        ix = np.clip(((gx - self.x0) / self.res).astype(int), 0, self.w - 1)
        iy = np.clip(((gy - self.y0) / self.res).astype(int), 0, self.h - 1)
        c = self.cost[np.ix_(iy, ix)].astype(np.int32)
        ros = np.where(c == 254, 100, np.where(c == 253, 99, np.where(c == 0, 0, 1 + (97 * (c - 1)) // 251)))
        known = seen[np.ix_(iy, ix)]
        inside = ((gx[None, :] > self.x0) & (gx[None, :] < self.x0 + self.w * self.res)
                  & (gy[:, None] > self.y0) & (gy[:, None] < self.y0 + self.h * self.res))
        ros = np.where(known & inside, ros, 255).astype(np.uint8)
        return {"width": n, "height": n, "resolution": res, "origin": (ox, oy), "data": ros.ravel()}

    def astar(self, start: Tuple[float, float], goal: Tuple[float, float], step: int = 2):
        """A* on the cost grid (every `step` cells); returns [(x, y), ...] or None."""
        cost = self.cost[::step, ::step]
        h, w = cost.shape
        res = self.res * step

        def cell(p):
            return (int(np.clip((p[1] - self.y0) / res, 0, h - 1)), int(np.clip((p[0] - self.x0) / res, 0, w - 1)))

        s, g = cell(start), cell(goal)
        if cost[g] >= 253:
            return None
        nbrs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.414), (-1, 1, 1.414),
                (1, -1, 1.414), (1, 1, 1.414)]
        openq = [(0.0, s)]
        came = {s: None}
        gcost = {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for dy, dx, d in nbrs:
                n = (cur[0] + dy, cur[1] + dx)
                if not (0 <= n[0] < h and 0 <= n[1] < w):
                    continue
                c = cost[n]
                if c >= 253 and n != s:
                    continue
                ng = gcost[cur] + d * (1.0 + 3.0 * c / 252.0)
                if ng < gcost.get(n, math.inf):
                    gcost[n] = ng
                    came[n] = cur
                    heapq.heappush(openq, (ng + math.hypot(n[0] - g[0], n[1] - g[1]), n))
        if g not in came:
            return None
        path, cur = [], g
        while cur is not None:
            path.append((self.x0 + (cur[1] + 0.5) * res, self.y0 + (cur[0] + 0.5) * res))
            cur = came[cur]
        path.reverse()
        path[0], path[-1] = tuple(start), tuple(goal)
        return path


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class FakeNav:
    TUNABLE = ("approach_v_max", "simple_v_max", "final_rotation_wmax", "final_rotation_kp",
               "progress_window_s", "progress_min_m", "recovery_wait_s", "recovery_clear_radius_m",
               "recovery_rotate_rad")

    def __init__(self, boxes: Sequence[Box3], *, vx_max: float = 0.4, wz_max: float = 0.6,
                 rays_per_frame: int = 6000, seed: int = 5):
        self.boxes = list(boxes)
        self.static = StaticCostmap(self.boxes)
        self.seen = np.zeros_like(self.static.cost, bool)
        self.vx_max, self.wz_max = vx_max, wz_max
        self.params = {"approach_v_max": 0.2, "simple_v_max": 0.3, "final_rotation_wmax": 0.5,
                       "final_rotation_kp": 1.5, "progress_window_s": 8.0, "progress_min_m": 0.2,
                       "recovery_wait_s": 2.0, "recovery_clear_radius_m": 1.5,
                       "recovery_rotate_rad": 0.52, "max_speed": 0.0}
        self.rng = np.random.default_rng(seed)
        self.n_rays = rays_per_frame
        r = rpy_to_matrix(*LIDAR_RPY)
        self.R_odom_lio = r
        self.t_odom_lio = LIDAR_XYZ.copy()
        self.pose: Optional[Tuple[float, float, float]] = None
        self.vel = (0.0, 0.0)
        self.guard: Dict[str, Any] = {}
        self.goal: Optional[Dict[str, Any]] = None
        self.target = None
        self.state, self.reason = "IDLE", ""
        self.path: List[Tuple[float, float]] = []
        self.path_i = 0
        self.cmd = (0.0, 0.0)
        self.t_goal = 0.0
        self.best_dist, self.best_t = math.inf, 0.0
        self.final_rot = False
        self.finish_t = -1e9
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    # ---- inputs -----------------------------------------------------------------------------
    def on_odom(self, m) -> None:
        x, y, yaw, v, w = msgs.odom_pose(m)
        with self.lock:
            self.pose, self.vel = (x, y, yaw), (v, w)

    def on_guard(self, m) -> None:
        d = msgs.parse_json(m)
        if d:
            self.guard = d

    def on_goal(self, m, now: float) -> None:
        g = msgs.parse_json(m)
        if not g:
            return
        with self.lock:
            if g.get("mode") == "stop":
                if self.state in ("NAVIGATING", "ROTATING", "RECOVERING"):
                    self._finish("CANCELED", "stop", now)
                return
            self.goal = g
            self._start(now)

    def on_param(self, m) -> Optional[Dict[str, Any]]:
        d = msgs.parse_json(m)
        if not d or d.get("target") != "nav":
            return None
        name, err = d.get("name"), ""
        try:
            v = float(d.get("value"))
            if v < 0 or not math.isfinite(v):
                err = "value must be finite and >= 0"
            elif name == "max_speed" and v > self.vx_max:
                err = f"max_speed above mppi.vx_max ({self.vx_max})"
            elif name not in self.params:
                err = f"unknown parameter '{name}'"
            else:
                self.params[name] = v
        except (TypeError, ValueError):
            err = "value must be a number"
        return {"id": d.get("id", ""), "name": name, "ok": not err, "error": err}

    # ---- goal logic -------------------------------------------------------------------------
    def _start(self, now: float) -> None:
        g, p = self.goal, self.pose
        if p is None:
            self._finish("FAILED", "lio_lost", now)
            return
        frame = g.get("frame", "base_at_stamp")
        x, y = float(g.get("x", 0.0)), float(g.get("y", 0.0))
        yaw = g.get("yaw")
        if frame == "odom":
            tx, ty = x, y
            tyaw = float(yaw) if yaw is not None else None
        else:
            c, s = math.cos(p[2]), math.sin(p[2])
            tx, ty = p[0] + c * x - s * y, p[1] + s * x + c * y
            tyaw = wrap(p[2] + float(yaw)) if yaw is not None else None
        mode = g.get("mode", "navigate")
        self.final_rot = False
        self.t_goal = now
        self.best_dist, self.best_t = math.inf, now
        if mode == "rotate":
            self.target = (p[0], p[1], tyaw if tyaw is not None else p[2])
            self.path = []
            self.state, self.reason = "ROTATING", ""
            return
        if self.static.at(tx, ty) >= 253:
            self._finish("FAILED", "goal_in_obstacle", now)
            return
        if mode == "simple":
            self.path = [(p[0], p[1]), (tx, ty)]
        else:
            path = self.static.astar((p[0], p[1]), (tx, ty))
            if path is None:
                self._finish("FAILED", "no_path", now)
                return
            self.path = path
        self.path_i = 0
        self.target = (tx, ty, tyaw)
        self.state, self.reason = "NAVIGATING", ""

    def _finish(self, state: str, reason: str, now: float) -> None:
        self.state, self.reason = state, reason
        self.cmd = (0.0, 0.0)
        self.finish_t = now

    def _speed_cap(self) -> float:
        v = self.vx_max
        if self.params["max_speed"] > 0:
            v = min(v, self.params["max_speed"])
        if self.goal and self.goal.get("max_speed"):
            v = min(v, float(self.goal["max_speed"]))
        if self.goal and self.goal.get("mode") == "simple":
            v = min(v, self.params["simple_v_max"])
        return v

    def step(self, now: float) -> None:
        with self.lock:
            self._step(now)

    def _step(self, now: float) -> None:
        if self.state not in ("NAVIGATING", "ROTATING", "RECOVERING") or self.pose is None:
            self.cmd = (0.0, 0.0)
            return
        g = self.guard
        if g.get("estop"):
            self._finish("FAILED", "guard_estop", now)
            return
        if g.get("manual"):
            self._finish("CANCELED", "manual_override", now)
            return
        if now - self.t_goal > float(self.goal.get("timeout_s", 120.0)):
            self._finish("FAILED", "timeout", now)
            return
        x, y, yaw = self.pose
        kp, wmax = self.params["final_rotation_kp"], self.params["final_rotation_wmax"]
        if self.state == "ROTATING" or self.final_rot:
            err = wrap(self.target[2] - yaw)
            if abs(err) < float(self.goal.get("yaw_tolerance", 0.25 if self.final_rot else 0.05)) \
                    and abs(self.vel[1]) < 0.15:
                self._finish("SUCCEEDED", "", now)
                return
            self.cmd = (0.0, max(-wmax, min(wmax, kp * err)) if abs(kp * err) > 0.08 else math.copysign(0.08, err))
            return
        tx, ty, tyaw = self.target
        dist = math.hypot(tx - x, ty - y)
        if dist < self.best_dist - self.params["progress_min_m"]:
            self.best_dist, self.best_t = dist, now
        elif now - self.best_t > self.params["progress_window_s"]:
            self._finish("FAILED", "blocked", now)
            return
        if dist < float(self.goal.get("xy_tolerance", 0.2)):
            if tyaw is not None:
                self.final_rot = True
                self.cmd = (0.0, 0.0)
            else:
                self._finish("SUCCEEDED", "", now)
            return
        # pure pursuit on the path
        while self.path_i < len(self.path) - 1 and math.hypot(self.path[self.path_i][0] - x,
                                                              self.path[self.path_i][1] - y) < 0.5:
            self.path_i += 1
        lx, ly = self.path[self.path_i]
        alpha = wrap(math.atan2(ly - y, lx - x) - yaw)
        vcap = self._speed_cap()
        if dist < 0.6:
            vcap = min(vcap, self.params["approach_v_max"])
        if abs(alpha) > 0.9:
            self.cmd = (0.0, math.copysign(min(self.wz_max, 0.6), alpha))
            return
        v = vcap * max(0.2, math.cos(alpha))
        look = max(0.3, math.hypot(lx - x, ly - y))
        w = max(-self.wz_max, min(self.wz_max, 2.0 * v * math.sin(alpha) / look))
        self.cmd = (v, w)

    # ---- sensing ----------------------------------------------------------------------------
    def lidar_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        """(points in odom, points in the LIO frame) for one 100 ms Mid-360S frame."""
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        origin = np.array([x + c * LIDAR_XYZ[0] - s * LIDAR_XYZ[1], y + s * LIDAR_XYZ[0] + c * LIDAR_XYZ[1],
                           LIDAR_XYZ[2]])
        az = self.rng.uniform(-math.pi, math.pi, self.n_rays)
        el = np.radians(self.rng.uniform(-52.0, 7.0, self.n_rays))
        dirs = np.c_[np.cos(el) * np.cos(az + yaw), np.cos(el) * np.sin(az + yaw), np.sin(el)]
        r = raycast3d(origin, dirs, self.boxes, 20.0)
        ok = np.isfinite(r) & (r > 0.3)
        pts = origin + dirs[ok] * (r[ok] + self.rng.normal(0, 0.01, int(ok.sum())))[:, None]
        pts_lio = (pts - self.t_odom_lio) @ self.R_odom_lio
        self._mark_seen(origin[0], origin[1])
        return pts.astype(np.float32), pts_lio.astype(np.float32)

    def _mark_seen(self, x: float, y: float, r: float = 8.0) -> None:
        st = self.static
        n = int(r / st.res)
        cx, cy = st._ix(x), st._iy(y)
        yy, xx = np.ogrid[max(0, cy - n):min(st.h, cy + n), max(0, cx - n):min(st.w, cx + n)]
        mask = (yy - cy) ** 2 + (xx - cx) ** 2 <= n * n
        self.seen[max(0, cy - n):min(st.h, cy + n), max(0, cx - n):min(st.w, cx + n)] |= mask

    def obstacle_points_base(self, pts_odom: np.ndarray) -> np.ndarray:
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts_odom[:, 0] - x, pts_odom[:, 1] - y
        b = np.c_[c * dx + s * dy, -s * dx + c * dy, pts_odom[:, 2]]
        keep = (b[:, 2] >= Z_BAND[0]) & (b[:, 2] < Z_BAND[1]) & (np.hypot(b[:, 0], b[:, 1]) < 3.0)
        return b[keep].astype(np.float32)

    # ---- reports ----------------------------------------------------------------------------
    def status(self, now: float) -> Dict[str, Any]:
        p = self.pose or (0.0, 0.0, 0.0)
        tgt = self.target
        active = self.state in ("NAVIGATING", "ROTATING", "RECOVERING")
        st = {"goal_id": (self.goal or {}).get("id", ""), "state": self.state, "reason": self.reason,
              "partial": False, "adjusted": False,
              "distance_remaining": math.hypot(tgt[0] - p[0], tgt[1] - p[1]) if (tgt and active) else 0.0,
              "yaw_error": wrap(tgt[2] - p[2]) if (tgt and active and tgt[2] is not None) else 0.0,
              "goal": {"x": tgt[0], "y": tgt[1], "yaw": tgt[2] or 0.0} if tgt else {"x": 0, "y": 0, "yaw": 0},
              "pose": {"x": p[0], "y": p[1], "yaw": p[2]},
              "velocity": {"v": self.vel[0], "w": self.vel[1]},
              "free_sectors": self.free_sectors(), "lio_ok": self.pose is not None,
              "state_valid": self.pose is not None, "state_source": "lio+wheel", "odom_consistent": True,
              "body_locked": True, "lock_reasons": [],
              "guard": {"blocked": bool(self.guard.get("blocked")), "estop": bool(self.guard.get("estop")),
                        "manual": bool(self.guard.get("manual"))},
              "recoveries": 0, "path_points": len(self.path), "stamp": now}
        return st

    def free_sectors(self) -> Dict[str, bool]:
        if self.pose is None:
            return {}
        x, y, yaw = self.pose
        out = {}
        for deg in range(-90, 91, 30):
            a = yaw + math.radians(deg)
            free = all(self.static.at(x + d * math.cos(a), y + d * math.sin(a)) < 253
                       for d in np.arange(0.3, 1.5, 0.1))
            out[str(deg)] = free
        return out

    def debug(self, now: float) -> Dict[str, Any]:
        q = _quat_from_matrix(self.R_odom_lio)
        active = self.state in ("NAVIGATING", "ROTATING", "RECOVERING")
        traj = []
        if active and self.pose:
            x, y, yaw = self.pose
            v, w = self.cmd
            for _ in range(28):
                x += v * 0.1 * math.cos(yaw)
                y += v * 0.1 * math.sin(yaw)
                yaw += w * 0.1
                traj.append([x, y])
        goal = None
        if self.goal and self.target and self.state != "IDLE":
            goal = {"id": self.goal.get("id"), "mode": self.goal.get("mode", "navigate"),
                    "x": self.target[0], "y": self.target[1], "yaw": self.target[2] or 0.0,
                    "use_yaw": self.target[2] is not None, "adjusted": False,
                    "xy_tolerance": float(self.goal.get("xy_tolerance", 0.2))}
        local = self.path[self.path_i:self.path_i + 20] if active else []
        return {"stamp": now, "state": self.state,
                "T_odom_lio": {"valid": True, "t": self.t_odom_lio.tolist(), "q": q},
                "goal": goal, "global_path": [list(p) for p in self.path] if active else [],
                "local_path": [list(p) for p in local], "mppi_traj": traj,
                "speed_limit": self.params["max_speed"]}

    # ---- DDS runtime ------------------------------------------------------------------------
    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import OccupancyGrid_, Odometry_
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        self._OccupancyGrid = OccupancyGrid_
        pubs = {}
        for key, topic, typ in (("cloud", "rt/nav/registered_cloud", PointCloud2_),
                                ("lio_odom", "rt/nav/lio_odom", Odometry_),
                                ("lio_health", "rt/nav/lio_health", String_),
                                ("base_odom", "rt/nav/base_odom", Odometry_),
                                ("status", "rt/nav/status", String_),
                                ("debug", "rt/nav/debug", String_),
                                ("obst", "rt/nav/obstacle_points", PointCloud2_),
                                ("local", "rt/nav/local_costmap", OccupancyGrid_),
                                ("global", "rt/nav/global_costmap", OccupancyGrid_),
                                ("params", "rt/nav/param_state", String_),
                                ("cmd", "rt/nav/cmd_vel", Twist_)):
            p = ChannelPublisher(topic, typ)
            p.Init()
            pubs[key] = p
        self.pubs = pubs
        self.subs = []
        for topic, typ, cb in (("rt/slamware_ros_sdk_server_node/odom", Odometry_, self.on_odom),
                               ("rt/nav/guard_state", String_, self.on_guard),
                               ("rt/nav/goal", String_, lambda m: self.on_goal(m, time.monotonic())),
                               ("rt/nav/param_set", String_, self._on_param_msg)):
            s = ChannelSubscriber(topic, typ)
            s.Init(cb, 8)
            self.subs.append(s)
        t = threading.Thread(target=self._run, name="fake_nav", daemon=True)
        t.start()
        self._threads.append(t)

    def _on_param_msg(self, m) -> None:
        ack = self.on_param(m)
        if ack is not None:
            self._publish_params(ack)

    def _publish_params(self, ack=None) -> None:
        msg = {"target": "nav", "params": dict(self.params, mppi_vx_max=self.vx_max, mppi_wz_max=self.wz_max)}
        if ack:
            msg["ack"] = ack
        self.pubs["params"].Write(msgs.json_msg(msg))

    def _grid_msg(self, g) -> Any:
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point_, Pose_, Quaternion_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import MapMetaData_
        info = MapMetaData_(map_load_time=msgs.stamp(0.0), resolution=float(g["resolution"]),
                            width=int(g["width"]), height=int(g["height"]),
                            origin=Pose_(position=Point_(float(g["origin"][0]), float(g["origin"][1]), 0.0),
                                         orientation=Quaternion_(0.0, 0.0, 0.0, 1.0)))
        return self._OccupancyGrid(header=msgs.header(time.time(), "odom"), info=info,
                                   data=g["data"].tolist())

    def _run(self) -> None:
        k = 0
        t_next = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            wall = time.time()
            self.step(now)
            pose = self.pose
            active = self.state in ("NAVIGATING", "ROTATING", "RECOVERING") or now - self.finish_t < 1.0
            if active:
                self.pubs["cmd"].Write(msgs.twist(*self.cmd))
            if pose is not None:
                if k % 2 == 0:                              # 10 Hz LIO
                    pts, pts_lio = self.lidar_frame()
                    self.pubs["cloud"].Write(msgs.make_cloud_xyz(pts_lio, "odom_lio", wall))
                    self.pubs["obst"].Write(msgs.make_cloud_xyz(self.obstacle_points_base(pts),
                                                                "base_link", wall))
                    self.pubs["lio_health"].Write(msgs.json_msg(
                        {"stamp": wall, "ok": True, "degenerate": False, "errors": [],
                         "effective_points": int(pts.shape[0]), "input_points": self.n_rays,
                         "residual": 0.01, "normal_eig_min": 0.1, "process_ms": 6.0,
                         "clock_offset": 0.0, "clock_jumps": 0}))
                    self.pubs["base_odom"].Write(msgs.make_odometry(wall, pose[0], pose[1], pose[2],
                                                                    self.vel[0], self.vel[1]))
                if k % 4 == 0:
                    self.pubs["debug"].Write(msgs.json_msg(self.debug(now)))
                if k % 10 == 0:
                    self.pubs["local"].Write(self._grid_msg(
                        self.static.ros_window(pose[0], pose[1], 6.0, 0.05, self.seen)))
                if k % 20 == 0:
                    self.pubs["global"].Write(self._grid_msg(
                        self.static.ros_window(pose[0], pose[1], 20.0, 0.1, self.seen)))
            if k % 2 == 0:
                self.pubs["status"].Write(msgs.json_msg(self.status(now)))
            if k % 20 == 0:
                self._publish_params()
            k += 1
            t_next += 0.05
            delay = t_next - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                t_next = time.monotonic()

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)


def _quat_from_matrix(r: np.ndarray) -> List[float]:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    y = math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2
    z = math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2
    x = math.copysign(x, r[2, 1] - r[1, 2])
    y = math.copysign(y, r[0, 2] - r[2, 0])
    z = math.copysign(z, r[1, 0] - r[0, 1])
    return [x, y, z, w]


def start_sim(world: str = "lab", with_mock: bool = True, http_port: Optional[int] = None):
    """Starts the fake nav stack (+ mock chassis) on an initialised ChannelFactory."""
    boxes = make_world3d(world)
    robot = None
    if with_mock:
        robot = MockRobot(chassis_boxes(boxes), MockParams())
        robot.start(http_port)
    nav = FakeNav(boxes)
    nav.start()
    return nav, robot


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--world", default="lab", choices=("lab", "empty"))
    ap.add_argument("--no-mock", action="store_true", help="do not run the mock chassis in-process")
    ap.add_argument("--http-port", type=int, default=None, help="fake chassis HTTP API port")
    args = ap.parse_args(argv)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(args.domain, None if args.net_if in (None, "", "auto") else args.net_if)
    nav, robot = start_sim(args.world, not args.no_mock, args.http_port)
    print(f"fake nav running (domain {args.domain}, world {args.world}); Ctrl-C to stop", flush=True)
    try:
        while True:
            time.sleep(2.0)
            p = nav.pose
            print(f"state={nav.state} reason='{nav.reason}' pose={p and tuple(round(v, 2) for v in p)}"
                  + (f" collided={robot.collided}" if robot else ""), flush=True)
    except KeyboardInterrupt:
        nav.stop()
        if robot:
            robot.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
