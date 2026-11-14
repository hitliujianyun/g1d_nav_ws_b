"""Lightweight client for rt/nav/goal and rt/nav/status (design doc §7.2 / §7.3, appendix B).

    client = NavClient(domain=0, net_if="eth0")
    gid = client.send_goal(1.8, -0.4, yaw=0.52, stamp=t_img)     # NavGPT relative waypoint
    final = client.wait(gid, timeout=60)                         # status dict, state SUCCEEDED/...
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from typing import Any, Callable, Dict, List, Optional

TERMINAL_STATES = ("SUCCEEDED", "FAILED", "CANCELED")
T_GOAL = "rt/nav/goal"
T_STATUS = "rt/nav/status"


def make_goal(x: float = 0.0, y: float = 0.0, yaw: Optional[float] = None, *,
              frame: str = "base_at_stamp", stamp: Optional[float] = None, mode: str = "navigate",
              goal_id: Optional[str] = None, **extra: Any) -> Dict[str, Any]:
    """Goal JSON. frame "base_at_stamp": (x, y, yaw) relative to base_link at `stamp` (host time;
    default now); frame "odom": absolute. extra: xy_tolerance, yaw_tolerance, max_speed, timeout_s."""
    if mode not in ("navigate", "rotate", "simple", "stop"):
        raise ValueError(f"unknown mode {mode!r}")
    if frame not in ("base_at_stamp", "odom"):
        raise ValueError(f"unknown frame {frame!r}")
    g: Dict[str, Any] = {"id": goal_id or f"g-{uuid.uuid4().hex[:8]}", "mode": mode, "frame": frame,
                         "stamp": time.time() if stamp is None else float(stamp),
                         "x": float(x), "y": float(y)}
    if yaw is not None:
        g["yaw"] = float(yaw)
    g.update(extra)
    return g


class NavClient:
    def __init__(self, domain: int = 0, net_if: Optional[str] = None, init_factory: bool = True):
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize, ChannelPublisher,
                                                 ChannelSubscriber)
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        if init_factory:
            ChannelFactoryInitialize(domain, net_if)
        self._String = String_
        self._pub = ChannelPublisher(T_GOAL, String_)
        self._pub.Init()
        self._cv = threading.Condition()
        self._status: Optional[Dict[str, Any]] = None
        self._history: Dict[str, Dict[str, Any]] = {}
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._sub = ChannelSubscriber(T_STATUS, String_)
        self._sub.Init(self._on_status, 4)

    def _on_status(self, m) -> None:
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        with self._cv:
            self._status = st
            gid = st.get("goal_id")
            if gid:
                self._history[gid] = st
            self._cv.notify_all()
        for cb in list(self._callbacks):
            cb(st)

    def on_status(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        self._callbacks.append(cb)

    def send(self, goal: Dict[str, Any]) -> str:
        self._pub.Write(self._String(data=json.dumps(goal)))
        return goal["id"]

    def send_goal(self, x: float, y: float, yaw: Optional[float] = None, **kw: Any) -> str:
        return self.send(make_goal(x, y, yaw, **kw))

    def rotate(self, yaw: float, **kw: Any) -> str:
        return self.send(make_goal(0.0, 0.0, yaw, mode="rotate", **kw))

    def stop(self) -> str:
        return self.send(make_goal(mode="stop"))

    def status(self) -> Optional[Dict[str, Any]]:
        with self._cv:
            return self._status

    def wait(self, goal_id: str, timeout: float = 60.0) -> Optional[Dict[str, Any]]:
        """Blocks until goal_id reaches a terminal state; None on timeout."""
        t_end = time.monotonic() + timeout
        with self._cv:
            while True:
                st = self._history.get(goal_id)
                if st and st.get("state") in TERMINAL_STATES:
                    return st
                left = t_end - time.monotonic()
                if left <= 0:
                    return None
                self._cv.wait(left)
