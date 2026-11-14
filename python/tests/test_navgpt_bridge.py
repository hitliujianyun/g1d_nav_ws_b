from g1d_nav_py.nav_client import make_goal
from g1d_nav_py.navgpt_bridge import NavGptBridge


class FakeClient:
    def __init__(self, final):
        self.final, self.sent = final, []

    def send_goal(self, x, y, yaw=None, **kw):
        g = make_goal(x, y, yaw, **kw)
        self.sent.append(g)
        return g["id"]

    def wait(self, gid, timeout):
        return dict(self.final, goal_id=gid)


def test_step_sends_latency_compensated_relative_goal():
    c = FakeClient({"state": "SUCCEEDED", "reason": ""})
    b = NavGptBridge(c, camera_latency_s=0.08, max_speed=0.3)
    r = b.step(lambda: (1.2, -0.3, 0.4), t_recv=100.0)
    g = c.sent[0]
    assert g["frame"] == "base_at_stamp" and abs(g["stamp"] - 99.92) < 1e-9
    assert (g["x"], g["y"], g["yaw"], g["max_speed"]) == (1.2, -0.3, 0.4, 0.3)
    assert g["id"].startswith("navgpt-") and r["goal_id"] == g["id"]
    assert b.feedback(r) == ""


def test_feedback_lists_free_sectors_in_order():
    r = {"state": "FAILED", "reason": "blocked", "partial": True,
         "free_sectors": {"30": True, "-90": True, "0": False, "-30": False, "90": False}}
    text = NavGptBridge.feedback(r)
    assert "原因=blocked" in text and "部分前进" in text
    assert text.endswith("=-90,30")
