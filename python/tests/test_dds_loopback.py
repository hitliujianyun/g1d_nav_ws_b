"""End-to-end on real CycloneDDS (in-process loopback): mock chassis <-> g1d_base_guard <-> a test
"nav" publishing rt/nav/cmd_vel, plus NavClient against a fake nav status responder.

The unitree ChannelFactory is a process-wide singleton (one domain), so everything shares one
fixture; phases run in order inside one test.
"""
import json
import math
import threading
import time

import numpy as np
import pytest

pytest.importorskip("cyclonedds")

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize, ChannelPublisher,  # noqa: E402
                                         ChannelSubscriber)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_  # noqa: E402
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # noqa: E402

from g1d_nav_py import msgs  # noqa: E402
from g1d_nav_py.config import load_config  # noqa: E402
from g1d_nav_py.dds_discovery import find_writers  # noqa: E402
from g1d_nav_py.guard_node import BaseGuardNode  # noqa: E402
from g1d_nav_py.mock_robot import MockParams, MockRobot, make_world  # noqa: E402
from g1d_nav_py.nav_client import NavClient  # noqa: E402

DOMAIN = 57


@pytest.fixture(scope="module")
def dds():
    ChannelFactoryInitialize(DOMAIN)
    return DOMAIN


class Latest:
    def __init__(self, topic, typ):
        self.msg, self.n, self.lock = None, 0, threading.Lock()
        self.sub = ChannelSubscriber(topic, typ)
        self.sub.Init(self._cb, 4)

    def _cb(self, m):
        with self.lock:
            self.msg, self.n = m, self.n + 1


def wait_until(pred, timeout, period=0.02):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(period)
    return False


class Streamer:
    """Publishes rt/nav/cmd_vel at 20 Hz like g1d_nav."""

    def __init__(self):
        self.pub = ChannelPublisher("rt/nav/cmd_vel", Twist_)
        self.pub.Init()
        self.cmd = None
        self._stop = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while not self._stop.is_set():
            if self.cmd is not None:
                self.pub.Write(msgs.twist(*self.cmd))
            time.sleep(0.05)

    def close(self):
        self._stop.set()
        self.t.join(1.0)


def test_guard_closed_loop_on_dds(dds, tmp_path):
    cfg = load_config()
    robot = MockRobot(make_world("wall"), MockParams(obstacle_points=True))
    robot.start(http_port=0)
    guard = BaseGuardNode(cfg, use_http=True, chassis_url=robot.http_url, log_dir=str(tmp_path))
    guard.start()
    gthread = threading.Thread(target=guard.run, daemon=True)
    gthread.start()
    state = Latest("rt/nav/guard_state", String_)
    relay = Latest("rt/nav/chassis_scan", PointCloud2_)
    nav = Streamer()

    def gstate():
        return json.loads(state.msg.data) if state.msg is not None else {}

    try:
        assert robot.aborted_actions == 1, "guard aborts the SLAMWARE action at start-up"
        assert find_writers("rt/cmd_vel_no_limit", dds, 1.0), "discovery sees the guard writer"

        # Phase A: idle -> the guard never writes rt/cmd_vel_no_limit.
        assert wait_until(lambda: state.n > 3 and gstate().get("n_scan", 0) > 2, 3.0)
        assert robot.cmd_log == []

        # Relay: chassis scan republished as base_link points (wall 3 m ahead).
        pts = msgs.read_cloud_xyz(relay.msg)
        ahead = pts[(np.abs(pts[:, 1]) < 0.2) & (pts[:, 0] > 0)]
        assert ahead[:, 0].min() == pytest.approx(3.0, abs=0.05)

        # Phase B: drive at the wall at 0.3 m/s; the reflex layer stops the base.
        nav.cmd = (0.3, 0.0)
        assert wait_until(lambda: robot.v > 0.25, 3.0)
        assert wait_until(lambda: robot.v == 0.0 and gstate().get("blocked") == "front", 15.0)
        time.sleep(0.5)
        gap = 3.0 - (robot.pose[0] + 0.2785)
        print(f"[e2e] stopped {gap:.3f} m before the wall, min gap {robot.min_gap:.3f}")
        assert not robot.collided
        assert 0.38 <= gap <= 0.60

        # Phase C: nav goes silent -> zero held for zero_hold, then the chassis is released.
        nav.cmd = None
        time.sleep(guard.p.cmd_timeout + guard.p.zero_hold + 0.4)
        n = len(robot.cmd_log)
        time.sleep(0.5)
        assert len(robot.cmd_log) == n, "guard stopped writing after zero_hold"
        assert all(v == 0.0 and w == 0.0 for _, v, w in robot.cmd_log[-5:])

        # Phase D: turn in place (no obstacle within the rotation circle).
        yaw0 = robot.pose[2]
        nav.cmd = (0.0, 0.5)
        assert wait_until(lambda: abs(robot.pose[2] - yaw0) > 0.5, 4.0)
        nav.cmd = (0.0, -0.5)
        assert wait_until(lambda: abs(robot.pose[2] - yaw0) < 0.1, 6.0)

        # Phase E: reverse away from the wall, then e-stop via rt/nav/estop (hard stop, latched).
        nav.cmd = (-0.08, 0.0)
        assert wait_until(lambda: robot.v < -0.05, 3.0)
        estop = ChannelPublisher("rt/nav/estop", String_)
        estop.Init()
        estop.Write(msgs.string("on"))
        assert wait_until(lambda: gstate().get("estop") is True, 1.0)
        assert wait_until(lambda: robot.cmd_log[-1][1] == 0.0, 0.5)
        time.sleep(0.5)
        assert robot.cmd_log[-1][1] == 0.0, "latched while nav still streams"
        estop.Write(msgs.string("off"))
        assert wait_until(lambda: robot.v < -0.05, 3.0)

        # Phase F: a new bumper event on the chassis HTTP API latches the e-stop.
        robot.trigger_event("BUMPER_TRIGGERED")
        assert wait_until(lambda: gstate().get("estop_reason") == "bumper_triggered", 2.0)
        assert wait_until(lambda: robot.cmd_log[-1][1] == 0.0, 0.5)
        estop.Write(msgs.string("off"))
        nav.cmd = None
    finally:
        nav.close()
        guard.stop()
        gthread.join(2.0)
        robot.stop()
    logs = list(tmp_path.glob("guard-*.jsonl"))
    assert logs and logs[0].read_text().count("\n") > 10


def test_nav_client_against_fake_nav(dds):
    status_pub = ChannelPublisher("rt/nav/status", String_)
    status_pub.Init()
    received = []

    def on_goal(m):
        g = json.loads(m.data)
        received.append(g)

        def respond():
            for st in ("NAVIGATING", "NAVIGATING", "SUCCEEDED"):
                time.sleep(0.1)
                status_pub.Write(msgs.json_msg({"goal_id": g["id"], "state": st, "reason": ""}))
        threading.Thread(target=respond, daemon=True).start()

    goal_sub = ChannelSubscriber("rt/nav/goal", String_)
    goal_sub.Init(on_goal, 4)
    client = NavClient(dds, init_factory=False)
    time.sleep(0.3)
    gid = client.send_goal(1.5, -0.3, 0.2, stamp=123.0, max_speed=0.3)
    final = client.wait(gid, timeout=5.0)
    assert final is not None and final["state"] == "SUCCEEDED"
    assert received[0]["x"] == 1.5 and received[0]["frame"] == "base_at_stamp"
    assert received[0]["stamp"] == 123.0
    assert client.wait("unknown-goal", timeout=0.2) is None
