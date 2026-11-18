"""Offline map cleaning from recorded keyframes (DUFOMap or free-space carving).

    python -m g1d_web.mapclean --frames recordings/<session> --map maps/<name>/map_raw.pcd \
        --out maps/<name>/map_clean.pcd [--method dufomap] [--resolution 0.1 --d-s 0.2 --d-p 1]

g1d_web runs this in a child process (DUFOMap's native code must not share a process with the
DDS stack); on the 5090 it is the same tool for recordings copied off the robot. With --mask the
keep mask over the input map is written as .npy instead of a cleaned PCD.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

from . import mapping


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", required=True, help="recording session dir (frames/*.pcd)")
    ap.add_argument("--map", required=True, help="binary xyz PCD to clean")
    ap.add_argument("--out", required=True, help="cleaned PCD (or keep mask .npy with --mask)")
    ap.add_argument("--mask", action="store_true")
    ap.add_argument("--method", default="auto", choices=("auto", "dufomap", "carving"))
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--d-s", type=float, default=0.2)
    ap.add_argument("--d-p", type=int, default=1)
    args = ap.parse_args(argv)
    pts = mapping.read_pcd_binary(args.map)
    frames = mapping.list_frames(args.frames)
    keep, info = mapping.clean_map(pts, frames, resolution=args.resolution, d_s=args.d_s,
                                   d_p=args.d_p, method=args.method, isolate=False)
    if args.mask:
        np.save(args.out, keep)
    else:
        mapping.write_pcd(args.out, np.ascontiguousarray(pts[keep]))
    print(json.dumps(info), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
