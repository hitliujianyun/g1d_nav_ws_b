"""Per-frame DUFOMap dynamic labels for scene-flow training (P4, 5090 / Thor).

    python -m g1d_perception.tools.flow_frames recordings/<session> --out datasets/flow/<session>

Input: a keyframe recording made with web.keyframes.min_dt = 0 and min_dist = 0 (every scan).
DUFOMap integrates all scans, then labels every point of every scan; the output is one npz per
scan (points (N, 3) world, pose (4, 4) sensor in world, dufo (N,) uint8, mdet (N,) uint8 =
M-detector label recorded online). This is the self-supervision signal SeFlow / DeltaFlow use
(DUFOMap dynamic labels + ego motion); convert to the trainer's own container (e.g. the
OpenSceneFlow h5 layout) with its preprocessing scripts.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from g1d_web import mapping
from g1d_perception.geometry import quat_xyzw_to_matrix, transform


def export(session: str, out: str, resolution: float = 0.1, d_s: float = 0.2, d_p: int = 1) -> dict:
    import dufomap
    files = mapping.list_frames(session)
    os.makedirs(out, exist_ok=True)
    m = dufomap.dufomap(resolution, d_s, d_p, num_threads=0)
    frames = []
    for path in files:
        pts, lab, origin, q = mapping.read_pcd_frame(path)
        frames.append((path, pts, lab, origin, q))
        if pts.shape[0]:
            m.run(pts, [float(v) for v in (*origin, *q)], cloud_transform=False)
    m.oncePropagateCluster(if_propagate=True, if_cluster=False)
    stats = {"frames": len(frames), "points": 0, "dufo_dynamic": 0, "mdet_dynamic": 0, "agree": 0}
    for path, pts, lab, origin, q in frames:
        dufo = np.asarray(m.segment(pts, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                    cloud_transform=False), np.uint8).reshape(-1)
        mdet = (lab > 0.5).astype(np.uint8)
        pose = transform(quat_xyzw_to_matrix([q[1], q[2], q[3], q[0]]), origin)
        name = os.path.splitext(os.path.basename(path))[0]
        np.savez_compressed(os.path.join(out, name + ".npz"), points=pts, pose=pose, dufo=dufo,
                            mdet=mdet)
        stats["points"] += int(pts.shape[0])
        stats["dufo_dynamic"] += int(dufo.sum())
        stats["mdet_dynamic"] += int(mdet.sum())
        stats["agree"] += int((dufo == mdet).sum())
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--out", required=True)
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--d-s", type=float, default=0.2)
    ap.add_argument("--d-p", type=int, default=1)
    args = ap.parse_args(argv)
    print(json.dumps(export(args.session, args.out, args.resolution, args.d_s, args.d_p)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
