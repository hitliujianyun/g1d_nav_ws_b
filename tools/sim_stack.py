#!/usr/bin/env python3
"""Runs the whole stack without hardware: fake_nav (+ mock chassis), the real g1d_base_guard and
g1d_web, each as its own process on one DDS domain. Open http://localhost:8080.

    python tools/sim_stack.py [--domain 7] [--port 8080] [--world lab]
"""
import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=7)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--http-port", type=int, default=18081, help="fake chassis HTTP API")
    ap.add_argument("--world", default="lab", choices=("lab", "empty"))
    args = ap.parse_args()
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([os.path.join(ROOT, "python"),
                                         os.path.join(ROOT, "third_party", "unitree_sdk2_python"),
                                         env.get("PYTHONPATH", "")])
    env["PYTHONUNBUFFERED"] = "1"
    py = sys.executable
    d = str(args.domain)
    cmds = [
        ("fake_nav", [py, "-m", "g1d_nav_py.fake_nav", "--domain", d, "--world", args.world,
                      "--http-port", str(args.http_port)]),
        ("guard", [py, "-m", "g1d_nav_py.guard_node", "--domain", d, "--net-if", "auto",
                   "--chassis-url", f"http://127.0.0.1:{args.http_port}", "--log-dir",
                   os.path.join(ROOT, "logs"), "--other-writers", "warn"]),
        ("web", [py, "-m", "g1d_web", "--domain", d, "--net-if", "auto", "--port", str(args.port)]),
    ]
    procs = []
    for name, cmd in cmds:
        procs.append((name, subprocess.Popen(cmd, cwd=ROOT, env=env)))
        time.sleep(1.5)
    print(f"\nsim stack running: open http://localhost:{args.port}  (Ctrl-C to stop)\n", flush=True)
    try:
        while all(p.poll() is None for _, p in procs):
            time.sleep(0.5)
        for name, p in procs:
            if p.poll() is not None:
                print(f"{name} exited with {p.returncode}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _, p in reversed(procs):
            if p.poll() is None:
                p.terminate()
        for _, p in procs:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    main()
