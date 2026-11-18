"""g1d_web end to end on real CycloneDDS (in-process): fake_nav + mock chassis + real guard +
web server; a WebSocket client drives it like the browser does."""
import asyncio
import json
import shutil
import threading
import time

import pytest

pytest.importorskip("cyclonedds")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

from g1d_nav_py.config import DEFAULT_CONFIG, load_config  # noqa: E402
from g1d_nav_py.fake_nav import start_sim  # noqa: E402
from g1d_nav_py.guard_node import BaseGuardNode  # noqa: E402
from g1d_web.app import WebApp  # noqa: E402
from g1d_web.hub import Hub  # noqa: E402
from g1d_web.server import HttpServer  # noqa: E402
from ws_client import WsClient, get  # noqa: E402

DOMAIN = 57      # the unitree ChannelFactory is a process-wide singleton: share test_dds_loopback's


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    ChannelFactoryInitialize(DOMAIN)
    tmp = tmp_path_factory.mktemp("web")
    cfg_path = tmp / "g1d_nav.yaml"
    shutil.copy(DEFAULT_CONFIG, cfg_path)
    cfg = load_config(str(cfg_path))
    nav, robot = start_sim("lab", True, http_port=0)
    guard = BaseGuardNode(cfg, use_http=True, chassis_url=robot.http_url, log_dir=str(tmp / "logs"))
    guard.start()
    gt = threading.Thread(target=guard.run, daemon=True)
    gt.start()
    hub = Hub()
    hub.start()
    app = WebApp(cfg, hub, str(cfg_path), str(tmp))
    yield {"app": app, "nav": nav, "robot": robot, "cfg_path": cfg_path, "tmp": tmp}
    guard.stop()
    gt.join(2.0)
    nav.stop()
    robot.stop()


async def wait_for(c, pred, timeout, seen=None):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        m = await asyncio.wait_for(c.recv(), max(0.1, t_end - time.monotonic()))
        if seen is not None:
            seen.append(m)
        if pred(m):
            return m
    raise AssertionError("timeout")


def test_web_full_loop(stack):
    app = stack["app"]

    async def main():
        server = HttpServer(app, "127.0.0.1", 0)
        await server.start()
        loop_task = asyncio.ensure_future(app.run())
        port = server.port
        try:
            page = await get("127.0.0.1", port, "/")
            assert page.startswith(b"HTTP/1.1 200") and b"G1-D" in page
            js = await get("127.0.0.1", port, "/vendor/three/three.module.min.js")
            assert js.startswith(b"HTTP/1.1 200") and b"text/javascript" in js[:300]
            assert (await get("127.0.0.1", port, "/../config/g1d_nav.yaml")).startswith(b"HTTP/1.1 404")

            c = await WsClient.connect("127.0.0.1", port)
            viewer = await WsClient.connect("127.0.0.1", port)
            hello = await wait_for(c, lambda m: m["type"] == "hello", 5)
            assert hello["config"]["footprint"] and len(hello["tests"]) == 6

            # commands need the control lock
            await c.send({"type": "goal", "goal": {"x": 3.5, "y": -1.5}})
            err = await wait_for(c, lambda m: m["type"] == "error", 5)
            assert "控制权" in err["text"]
            await c.send({"type": "control", "action": "acquire"})
            await wait_for(c, lambda m: m["type"] == "state" and m["control"]["you"]
                           and m["pose"] and m["fresh"]["status"], 10)
            await viewer.send({"type": "control", "action": "acquire"})
            assert "控制权" in (await wait_for(viewer, lambda m: m["type"] == "error", 5))["text"]

            # 1. navigate around the box; all streams arrive
            seen = []
            await c.send({"type": "goal", "goal": {"x": 3.5, "y": -1.5, "yaw": 0.0}})
            sent = await wait_for(c, lambda m: m["type"] == "goal_sent", 5)
            gid = sent["goal"]["id"]
            final = await wait_for(c, lambda m: m["type"] == "state" and (m["status"] or {}).get("goal_id") == gid
                                   and m["status"]["state"] in ("SUCCEEDED", "FAILED"), 60, seen)
            assert final["status"]["state"] == "SUCCEEDED", final["status"]
            kinds = {m["kind"] for m in seen if m["type"] == "binary"}
            assert {1, 2, 4, 5, 6} <= kinds, kinds
            debug_paths = [m["debug"]["global_path"] for m in seen if m["type"] == "state" and m.get("debug")]
            assert any(len(p) > 5 for p in debug_paths)
            assert final["map"]["points"] > 10000 and final["map"]["T_valid"]
            assert not stack["robot"].collided

            # 2. two-waypoint mission
            await c.send({"type": "mission", "action": "start",
                          "mission": {"name": "e2e", "waypoints": [{"x": 2.0, "y": 1.6}, {"x": 0.5, "y": 0.0, "yaw": 3.14}]}})
            done = await wait_for(c, lambda m: m["type"] == "state" and m["mission"]["state"]
                                  in ("done", "failed"), 90)
            assert done["mission"]["state"] == "done", done["mission"]
            assert [r["state"] for r in done["mission"]["results"]] == ["SUCCEEDED", "SUCCEEDED"]

            # 3. runtime parameters + write-back
            await c.send({"type": "param", "target": "guard", "name": "stop_m", "value": 0.5})
            pm = await wait_for(c, lambda m: m["type"] == "params" and m["guard"].get("stop_m") == 0.5, 5)
            assert any(a["name"] == "stop_m" and a["ok"] for a in pm["acks"])
            await c.send({"type": "param", "target": "nav", "name": "max_speed", "value": 9.0})
            pm = await wait_for(c, lambda m: m["type"] == "params" and any(
                a["name"] == "max_speed" and not a["ok"] for a in m["acks"]), 5)
            await c.send({"type": "param_save", "target": "guard"})
            saved = await wait_for(c, lambda m: m["type"] == "param_saved", 5)
            assert "stop_m" in saved["changed"]
            assert "  stop_m: 0.5" in stack["cfg_path"].read_text(encoding="utf-8")

            # 4. map export + download
            await c.send({"type": "map", "action": "export"})
            ex = await wait_for(c, lambda m: m["type"] == "exports" and m.get("new"), 15)
            png = await get("127.0.0.1", port, f"/files/maps/{ex['new']}/grid.png")
            assert png.startswith(b"HTTP/1.1 200") and b"\x89PNG" in png

            # 5. web teleop turns the base and is reported as manual / web
            for _ in range(15):
                await c.send({"type": "teleop", "v": 0.0, "w": 0.4})
                await asyncio.sleep(0.1)
            # States queued while sending may predate the guard switching: wait for the switch.
            st = await wait_for(c, lambda m: m["type"] == "state"
                                and (m["guard"] or {}).get("source") == "teleop", 3)
            assert st["guard"]["manual_source"] == "web"

            # 6. e-stop from a read-only client; release needs control
            await viewer.send({"type": "estop", "on": True})
            await wait_for(c, lambda m: m["type"] == "state" and (m["guard"] or {}).get("estop"), 3)
            await viewer.send({"type": "estop", "on": False})
            await wait_for(viewer, lambda m: m["type"] == "error", 3)
            await c.send({"type": "estop", "on": False})
            await wait_for(c, lambda m: m["type"] == "state" and not (m["guard"] or {}).get("estop"), 3)
            c.close()
            viewer.close()
        finally:
            loop_task.cancel()
            server.server.close()

    asyncio.run(main())
