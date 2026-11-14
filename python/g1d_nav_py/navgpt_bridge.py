"""NavGPT-3 -> g1d_nav glue (design doc §6.9, appendix B).

The model returns a relative waypoint (dx, dy, dyaw) in base_link (x forward, y left, CCW
positive) for an image captured at host time t_img; g1d_nav compensates the latency with its pose
history (frame "base_at_stamp"). Failures come back as text for the next prompt.

    bridge = NavGptBridge(NavClient(0, "eth0"), camera_latency_s=0.08)
    result = bridge.step(lambda: navgpt3.infer(image, instruction), t_recv=frame_recv_time)
    prompt += bridge.feedback(result)
"""
from __future__ import annotations

import uuid
from typing import Any, Callable, Dict, Optional, Tuple

Waypoint = Tuple[float, float, Optional[float]]


class NavGptBridge:
    def __init__(self, client, *, camera_latency_s: float = 0.0, max_speed: float = 0.35,
                 xy_tolerance: float = 0.20, yaw_tolerance: float = 0.25, timeout_s: float = 60.0):
        self.client = client
        self.camera_latency_s = camera_latency_s
        self.goal_opts = {"max_speed": max_speed, "xy_tolerance": xy_tolerance,
                          "yaw_tolerance": yaw_tolerance, "timeout_s": timeout_s}
        self.timeout_s = timeout_s

    def send(self, waypoint: Waypoint, t_recv: float) -> str:
        dx, dy, dyaw = waypoint
        return self.client.send_goal(dx, dy, dyaw, frame="base_at_stamp",
                                     stamp=t_recv - self.camera_latency_s,
                                     goal_id=f"navgpt-{uuid.uuid4().hex[:8]}", **self.goal_opts)

    def step(self, infer: Callable[[], Waypoint], t_recv: float,
             wait: bool = True) -> Dict[str, Any]:
        """Runs one model inference, sends the waypoint and (optionally) waits for the result."""
        gid = self.send(infer(), t_recv)
        if not wait:
            return {"goal_id": gid, "state": "SENT"}
        st = self.client.wait(gid, timeout=self.timeout_s + 10.0)
        return st or {"goal_id": gid, "state": "FAILED", "reason": "client_timeout"}

    @staticmethod
    def feedback(result: Dict[str, Any]) -> str:
        """Prompt text for the next NavGPT round ("" when the waypoint was reached)."""
        state = result.get("state")
        if state == "SUCCEEDED":
            reason = result.get("reason") or ""
            return f"\n[导航完成] {reason}" if reason else ""
        if state in ("FAILED", "CANCELED"):
            sectors = result.get("free_sectors") or {}
            free = [k for k, v in sorted(sectors.items(), key=lambda kv: float(kv[0])) if v]
            text = f"\n[导航失败] 原因={result.get('reason', '')}"
            if result.get("partial"):
                text += "（已部分前进）"
            if free:
                text += f" 可通行方向(度,左正)={','.join(free)}"
            return text
        return ""
