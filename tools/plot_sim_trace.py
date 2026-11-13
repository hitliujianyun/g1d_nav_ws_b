#!/usr/bin/env python3
"""Plot a g1d_sim_demo trace: global costmap, world boxes, planned paths and driven trajectory.

    python tools/plot_sim_trace.py sim_trace.json -o sim_trace.png
"""
import argparse
import json
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()
    with open(args.trace) as f:
        tr = json.load(f)

    fig, ax = plt.subplots(figsize=(11, 8))
    cm = tr.get("global_costmap")
    if cm:
        nx, ny = cm["size"]
        data = np.array(cm["data"], dtype=float).reshape(ny, nx)
        data[data == 255] = np.nan
        ox, oy = cm["origin"]
        res = cm["resolution"]
        ax.imshow(data, origin="lower", cmap="Greys", vmin=0, vmax=254,
                  extent=[ox, ox + nx * res, oy, oy + ny * res], alpha=0.8)

    for b in tr.get("boxes", []):
        x0, y0, z0 = b["min"]
        x1, y1, z1 = b["max"]
        low = z0 < 0.4  # visible to the chassis LiDAR
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                   ec="tab:brown" if low else "tab:purple",
                                   ls="-" if low else "--", lw=1.2))

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, g in enumerate(tr.get("goals", [])):
        c = colors[i % len(colors)]
        if g.get("global_path"):
            p = np.array(g["global_path"])
            ax.plot(p[:, 0], p[:, 1], ls=":", color=c, lw=1.5)
        if g.get("traj"):
            t = np.array(g["traj"])
            ax.plot(t[:, 1], t[:, 2], color=c, lw=2.0)
        goal = g["goal"]
        r = g["result"]
        ax.plot(goal["x"], goal["y"], marker="*", ms=14, color=c)
        if "yaw" in goal:
            ax.arrow(goal["x"], goal["y"], 0.4 * math.cos(goal["yaw"]), 0.4 * math.sin(goal["yaw"]),
                     head_width=0.08, color=c)
        ax.annotate(f'{goal.get("id", i)}: {r["state"]} {r["reason"]}\n'
                    f't={r["time_s"]:.1f}s clear={r["min_clearance"]:.2f}m',
                    (goal["x"], goal["y"]), textcoords="offset points", xytext=(8, 8),
                    fontsize=8, color=c)

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f'g1d_sim_demo: {tr.get("scenario")}  (dotted = global path, solid = driven; '
                 'dashed boxes = above chassis LiDAR)')
    ax.grid(alpha=0.3)
    out = args.out or args.trace.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
