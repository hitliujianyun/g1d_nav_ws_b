"""Accumulated LIO map of the current run (voxel filtered, odom frame), trajectory and exports.

Exports (maps/<time>/): map.pcd (binary float32 xyz), grid.png + grid.yaml (ROS map_server format,
occupied = map points between grid_z_min and grid_z_max), trajectory.csv, costmap.png (last global
costmap, if any), meta.json. With recorded keyframes the export also writes map_raw.pcd (before
cleaning) and map.pcd / grid.png come from the cleaned map (DUFOMap, or free-space carving when
the dufomap package is missing).

Keyframes (recordings/<session>/frames/NNNNNN.pcd): one LiDAR scan in odom with the sensor pose in
VIEWPOINT (x y z qw qx qy qz) and the M-detector label in the intensity field -- the layout of the
KTH DynamicMap benchmark / DUFOMap tools, so the same files feed offline cleaning on the 5090
and scene-flow pseudo-labelling.
"""
from __future__ import annotations

import collections
import json
import math
import os
import struct
import threading
import time
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1


def voxel_keys(points: np.ndarray, voxel: float) -> np.ndarray:
    k = np.floor(points / voxel).astype(np.int64) + _OFF
    k &= _MASK
    return (k[:, 0] << (2 * _BITS)) | (k[:, 1] << _BITS) | k[:, 2]


class MapBuilder:
    """Thread-safe voxel map: add_cloud() returns only the points of newly occupied voxels."""

    def __init__(self, voxel: float = 0.05, max_points: int = 3_000_000):
        self.lock = threading.Lock()
        self.max_points = int(max_points)
        self.reset(voxel)

    def reset(self, voxel: Optional[float] = None) -> None:
        with self.lock:
            if voxel is not None:
                if not (0.01 <= voxel <= 1.0):
                    raise ValueError("voxel must be in [0.01, 1.0] m")
                self.voxel = float(voxel)
            self._keys: set = set()
            self._chunks: List[np.ndarray] = []
            self._cache: Optional[np.ndarray] = None
            self.n_points = 0
            self.frames = 0
            self.full = False
            self.trajectory: List[Tuple[float, float, float, float]] = []
            self.started = time.time()

    def add_cloud(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        with self.lock:
            self.frames += 1
            if pts.shape[0] == 0 or self.full:
                return np.zeros((0, 3), np.float32)
            keys = voxel_keys(pts, self.voxel)
            uniq, idx = np.unique(keys, return_index=True)
            known = self._keys
            new_idx = [i for k, i in zip(uniq.tolist(), idx.tolist()) if k not in known]
            if not new_idx:
                return np.zeros((0, 3), np.float32)
            room = self.max_points - self.n_points
            if len(new_idx) > room:
                new_idx = new_idx[:room]
                self.full = True
            sel = np.asarray(new_idx, dtype=np.int64)
            known.update(keys[sel].tolist())
            new_pts = np.ascontiguousarray(pts[sel])
            self._chunks.append(new_pts)
            self._cache = None
            self.n_points += new_pts.shape[0]
            return new_pts

    def points(self) -> np.ndarray:
        with self.lock:
            if self._cache is None:
                self._cache = (np.concatenate(self._chunks) if self._chunks
                               else np.zeros((0, 3), np.float32))
                self._chunks = [self._cache] if self._chunks else []
            return self._cache

    def add_pose(self, t: float, x: float, y: float, yaw: float, min_step: float = 0.05) -> bool:
        with self.lock:
            if self.trajectory:
                lt, lx, ly, lyaw = self.trajectory[-1]
                if (math.hypot(x - lx, y - ly) < min_step and abs(yaw - lyaw) < 0.1
                        and t - lt < 5.0):
                    return False
            self.trajectory.append((t, x, y, yaw))
            return True

    def stats(self) -> Dict:
        with self.lock:
            return {"points": self.n_points, "frames": self.frames, "voxel": self.voxel,
                    "full": self.full, "trajectory": len(self.trajectory),
                    "started": self.started}

    def occupancy(self, resolution: float = 0.05, z_min: float = 0.10, z_max: float = 1.80,
                  floor_z: float = 0.08, margin: float = 0.5):
        """2D grid from the map: returns (img uint8 [h, w] with row 0 = +y edge, origin_xy) using
        map_server values occupied 0, free 254, unknown 205; None if the map is empty."""
        pts = self.points()
        with self.lock:
            traj = np.array([(x, y) for _, x, y, _ in self.trajectory], np.float64).reshape(-1, 2)
        if pts.shape[0] == 0 and traj.shape[0] == 0:
            return None
        xy = np.vstack([pts[:, :2].astype(np.float64), traj]) if traj.size else pts[:, :2]
        x0, y0 = xy.min(axis=0) - margin
        x1, y1 = xy.max(axis=0) + margin
        w = int(math.ceil((x1 - x0) / resolution))
        h = int(math.ceil((y1 - y0) / resolution))
        grid = np.full((h, w), 205, np.uint8)

        def cells(p):
            ix = np.clip(((p[:, 0] - x0) / resolution).astype(np.int64), 0, w - 1)
            iy = np.clip(((p[:, 1] - y0) / resolution).astype(np.int64), 0, h - 1)
            return iy, ix

        z = pts[:, 2]
        floor = pts[np.abs(z) < floor_z]
        if floor.shape[0]:
            grid[cells(floor)] = 254
        if traj.shape[0]:
            grid[cells(traj)] = 254
        # Sparse floor returns leave single unknown cells inside free space: one dilation pass.
        free = grid == 254
        grown = free.copy()
        grown[1:, :] |= free[:-1, :]
        grown[:-1, :] |= free[1:, :]
        grown[:, 1:] |= free[:, :-1]
        grown[:, :-1] |= free[:, 1:]
        grid[grown] = 254
        occ = pts[(z >= z_min) & (z <= z_max)]
        if occ.shape[0]:
            grid[cells(occ)] = 0
        return np.flipud(grid), (float(x0), float(y0))


# ------------------------------------------------------------------------------------- writers
def write_pcd(path: str, pts: np.ndarray) -> None:
    pts = np.ascontiguousarray(pts, dtype="<f4").reshape(-1, 3)
    n = pts.shape[0]
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
              f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes())


def read_pcd_binary(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        data = f.read()
    end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    n = int([ln for ln in data[:end].decode().splitlines() if ln.startswith("POINTS")][0].split()[1])
    return np.frombuffer(data[end:end + 12 * n], dtype="<f4").reshape(n, 3)


def png_gray(img: np.ndarray) -> bytes:
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    raw = b"".join(b"\x00" + img[r].tobytes() for r in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + tag + body
                + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def write_png_gray(path: str, img: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(png_gray(img))


def write_map_yaml(path: str, image: str, resolution: float, origin: Sequence[float]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"image: {image}\nmode: trinary\nresolution: {resolution}\n"
                f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")


def costmap_image(width: int, height: int, data: np.ndarray) -> np.ndarray:
    """ROS OccupancyGrid values (0..100, 255 unknown) -> grey image, row 0 = +y edge."""
    d = np.asarray(data, dtype=np.uint8).reshape(height, width)
    img = np.where(d == 255, 205, 254 - (d.astype(np.int32) * 254) // 100).astype(np.uint8)
    return np.flipud(img)


# ------------------------------------------------------------------------------------ keyframes
def write_pcd_frame(path: str, pts: np.ndarray, labels: Optional[np.ndarray],
                    origin: Sequence[float], quat_wxyz: Sequence[float]) -> None:
    pts = np.asarray(pts, dtype="<f4").reshape(-1, 3)
    n = pts.shape[0]
    data = np.empty((n, 4), "<f4")
    data[:, :3] = pts
    data[:, 3] = 0.0 if labels is None else np.asarray(labels, np.float32).reshape(-1)
    vp = " ".join(f"{v:.6f}" for v in (*origin, *quat_wxyz))
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n"
              f"SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT {vp}\n"
              f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(data).tobytes())


def read_pcd_frame(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(points (N, 3), labels (N,), origin (3,), quat_wxyz (4,)) of a write_pcd_frame file."""
    with open(path, "rb") as f:
        data = f.read()
    end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    head = {ln.split()[0]: ln.split()[1:] for ln in data[:end].decode().splitlines()
            if ln and not ln.startswith("#")}
    n = int(head["POINTS"][0])
    nf = len(head["FIELDS"])
    arr = np.frombuffer(data[end:end + 4 * nf * n], dtype="<f4").reshape(n, nf)
    vp = np.asarray([float(v) for v in head.get("VIEWPOINT", [0, 0, 0, 1, 0, 0, 0])])
    labels = arr[:, 3] if nf > 3 else np.zeros(n, np.float32)
    return np.ascontiguousarray(arr[:, :3]), labels, vp[:3], vp[3:7]


class KeyframeRecorder:
    """Writes a LiDAR keyframe whenever the sensor moved `min_dist` / turned `min_yaw` or
    `max_dt` passed (min_dt = 0 and min_dist = 0 records every scan, e.g. for scene flow)."""

    def __init__(self, root: str, min_dist: float = 0.3, min_yaw: float = 0.3,
                 min_dt: float = 0.5, max_dt: float = 5.0, voxel: float = 0.05,
                 max_frames: int = 4000):
        self.root = root
        self.min_dist, self.min_yaw = float(min_dist), float(min_yaw)
        self.min_dt, self.max_dt = float(min_dt), float(max_dt)
        self.voxel, self.max_frames = float(voxel), int(max_frames)
        self.lock = threading.Lock()
        self.dir: Optional[str] = None
        self.frames = 0
        self._last: Optional[Tuple[float, np.ndarray, np.ndarray]] = None

    def start(self, name: Optional[str] = None) -> str:
        with self.lock:
            name = name or time.strftime("%Y%m%d-%H%M%S")
            self.dir = os.path.join(self.root, name)
            os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)
            self.frames = 0
            self._last = None
            with open(os.path.join(self.dir, "poses.csv"), "w", encoding="utf-8") as f:
                f.write("index,t,x,y,z,qw,qx,qy,qz,points,dynamic\n")
            return self.dir

    def stop(self) -> None:
        with self.lock:
            self.dir = None

    @property
    def active(self) -> bool:
        return self.dir is not None

    def _due(self, t: float, origin: np.ndarray, q: np.ndarray) -> bool:
        if self._last is None:
            return True
        lt, lo, lq = self._last
        dt = t - lt
        if dt < self.min_dt:
            return False
        if dt >= self.max_dt:
            return True
        turn = 2.0 * math.acos(min(1.0, abs(float(np.dot(q, lq)))))
        return float(np.linalg.norm(origin - lo)) >= self.min_dist or turn >= self.min_yaw

    def add(self, t: float, static: np.ndarray, dynamic: np.ndarray, origin: np.ndarray,
            quat_wxyz: np.ndarray) -> bool:
        origin = np.asarray(origin, np.float64)
        q = np.asarray(quat_wxyz, np.float64)
        with self.lock:
            if self.dir is None or self.frames >= self.max_frames or not self._due(t, origin, q):
                return False
            # Moving points first: voxel de-duplication keeps the first point, so a voxel with
            # both keeps the moving label (positives for training).
            pts = np.vstack([np.asarray(dynamic, np.float32).reshape(-1, 3),
                             np.asarray(static, np.float32).reshape(-1, 3)])
            labels = np.concatenate([np.ones(len(dynamic), np.float32),
                                     np.zeros(len(static), np.float32)])
            if self.voxel > 0 and pts.shape[0]:
                _, idx = np.unique(voxel_keys(pts, self.voxel), return_index=True)
                pts, labels = pts[idx], labels[idx]
            path = os.path.join(self.dir, "frames", f"{self.frames:06d}.pcd")
            write_pcd_frame(path + ".tmp", pts, labels, origin, q)
            os.replace(path + ".tmp", path)   # exports list only complete frames
            with open(os.path.join(self.dir, "poses.csv"), "a", encoding="utf-8") as f:
                f.write(f"{self.frames},{t:.6f},{origin[0]:.4f},{origin[1]:.4f},{origin[2]:.4f},"
                        f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f},{pts.shape[0]},"
                        f"{int(labels.sum())}\n")
            self.frames += 1
            self._last = (t, origin, q)
            return True

    def stats(self) -> Dict:
        with self.lock:
            return {"active": self.dir is not None, "dir": self.dir, "frames": self.frames}


def list_frames(session_dir: str) -> List[str]:
    d = os.path.join(session_dir, "frames")
    if not os.path.isdir(d):
        return []
    return [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.endswith(".pcd")]


# ---------------------------------------------------------------------------------- map cleaning
def dufomap_available() -> bool:
    try:
        import dufomap  # noqa: F401
    except ImportError:
        return False
    return True


def clean_map(map_pts: np.ndarray, frame_files: Sequence[str], *, resolution: float = 0.1,
              d_s: float = 0.2, d_p: int = 1, method: str = "auto", isolate: bool = True,
              carve_rays: int = 4000, carve_min_free: int = 2,
              timeout_s: float = 900.0) -> Tuple[np.ndarray, Dict]:
    """Labels map points left by moving objects from the recorded keyframes.

    Returns (keep mask over map_pts, info). DUFOMap (Duberg et al., RA-L 2024) marks voxels that
    some scan saw through as void; points inside void voxels are dropped. d_p = 1 instead of the
    paper's 2: at Mid-360 point density d_p = 2 leaves most moving-object voxels "unknown"
    (13 % vs 95 % recall on a synthetic crossing box). The fallback carves free space along
    sampled rays (coarser; used only without the dufomap package).

    With `isolate` DUFOMap runs in a child process (g1d_web.mapclean): inside the g1d_web
    process, next to CycloneDDS, it crashed with an access violation on Windows. If the child
    fails the carving fallback runs in-process."""
    map_pts = np.asarray(map_pts, np.float32).reshape(-1, 3)
    info: Dict = {"frames": len(frame_files), "map_points": int(map_pts.shape[0])}
    if map_pts.shape[0] == 0 or not frame_files:
        info["method"] = "none"
        return np.ones(map_pts.shape[0], bool), info
    use_dufo = method == "dufomap" or (method == "auto" and dufomap_available())
    t0 = time.time()
    if use_dufo and isolate:
        keep, child = _clean_in_child(map_pts, frame_files, resolution, d_s, d_p, timeout_s)
        if keep is not None:
            info.update(child)
            info["isolated"] = True
            return keep, info
        info["fallback_reason"] = child.get("error", "child failed")
        use_dufo = False
    if use_dufo:
        import dufomap
        m = dufomap.dufomap(float(resolution), float(d_s), int(d_p), num_threads=0)
        for path in frame_files:
            pts, _, origin, q = read_pcd_frame(path)
            if pts.shape[0]:
                m.run(pts, [float(v) for v in (*origin, *q)], cloud_transform=False)
        m.oncePropagateCluster(if_propagate=True, if_cluster=False)
        labels = m.segment(map_pts, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], cloud_transform=False)
        keep = np.asarray(labels).reshape(-1) == 0
        info["method"] = "dufomap"
    else:
        keep = _carve(map_pts, frame_files, resolution, carve_rays, carve_min_free)
        info["method"] = "carving"
    info.update({"removed": int((~keep).sum()), "seconds": round(time.time() - t0, 2),
                 "resolution": resolution, "d_s": d_s, "d_p": d_p})
    return keep, info


def _clean_in_child(map_pts: np.ndarray, frame_files: Sequence[str], res: float, d_s: float,
                    d_p: int, timeout_s: float) -> Tuple[Optional[np.ndarray], Dict]:
    import subprocess
    import sys
    import tempfile
    session = os.path.dirname(os.path.dirname(frame_files[0]))
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "map.pcd")
        dst = os.path.join(tmp, "keep.npy")
        write_pcd(src, map_pts)
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = dict(os.environ, PYTHONPATH=pkg_root + os.pathsep + os.environ.get("PYTHONPATH", ""))
        cmd = [sys.executable, "-m", "g1d_web.mapclean", "--frames", session, "--map", src,
               "--out", dst, "--mask", "--method", "dufomap", "--resolution", str(res),
               "--d-s", str(d_s), "--d-p", str(int(d_p))]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s, env=env)
        except subprocess.TimeoutExpired:
            return None, {"error": f"timeout after {timeout_s:.0f} s"}
        if r.returncode != 0 or not os.path.isfile(dst):
            tail = (r.stderr or r.stdout or "").strip().splitlines()[-1:] or [""]
            return None, {"error": f"exit {r.returncode}: {tail[0][:200]}"}
        keep = np.load(dst)
        info = {}
        for line in reversed(r.stdout.strip().splitlines()):
            if line.startswith("{"):
                info = json.loads(line)
                break
        if keep.shape[0] != map_pts.shape[0]:
            return None, {"error": "mask size mismatch"}
        return keep.astype(bool), info


def _carve(map_pts: np.ndarray, frame_files: Sequence[str], res: float, rays: int,
           min_free: int) -> np.ndarray:
    rng = np.random.default_rng(0)
    free: Dict[int, int] = collections.Counter()
    for path in frame_files:
        pts, _, origin, _ = read_pcd_frame(path)
        if pts.shape[0] == 0:
            continue
        if pts.shape[0] > rays:
            pts = pts[rng.choice(pts.shape[0], rays, replace=False)]
        d = pts.astype(np.float64) - origin
        r = np.linalg.norm(d, axis=1)
        ok = r > 4 * res
        d, r = d[ok] / r[ok, None], r[ok]
        # Stop short of the return: 2 voxels or 10 % of the range (grazing floor rays).
        stop = r - np.maximum(2 * res, 0.1 * r)
        steps = np.arange(res, float(stop.max(initial=0.0)), 0.5 * res)
        if steps.size == 0:
            continue
        s = steps[None, :]
        valid = s < stop[:, None]
        samples = origin + d[:, None, :] * s[..., None]
        keys = np.unique(voxel_keys(samples[valid].reshape(-1, 3), res))
        free.update(keys.tolist())
    if not free:
        return np.ones(map_pts.shape[0], bool)
    mk = voxel_keys(map_pts, res)
    counts = np.fromiter((free.get(int(k), 0) for k in mk), np.int64, mk.shape[0])
    return counts < min_free


def export_map(builder: MapBuilder, root: str, *, resolution: float = 0.05,
               z_min: float = 0.10, z_max: float = 1.80, costmap: Optional[Dict] = None,
               extra_meta: Optional[Dict] = None, frame_files: Sequence[str] = (),
               clean: Optional[Dict] = None) -> Dict:
    name = time.strftime("%Y%m%d-%H%M%S")
    out = os.path.join(root, name)
    os.makedirs(out, exist_ok=True)
    pts = builder.points()
    files = []
    meta = {"name": name, "created": time.time(), **builder.stats(),
            "grid_resolution": resolution, "grid_z": [z_min, z_max], "frame": "odom"}
    occ_builder = builder
    if frame_files and clean is not None:
        keep, info = clean_map(pts, frame_files, **clean)
        meta["cleaning"] = info
        write_pcd(os.path.join(out, "map_raw.pcd"), pts)
        files.append("map_raw.pcd")
        pts = np.ascontiguousarray(pts[keep])
        occ_builder = MapBuilder(builder.voxel, max(1, pts.shape[0] + 1))
        occ_builder.add_cloud(pts)
        with builder.lock:
            occ_builder.trajectory = list(builder.trajectory)
        meta["points"] = int(pts.shape[0])
    write_pcd(os.path.join(out, "map.pcd"), pts)
    files.append("map.pcd")
    occ = occ_builder.occupancy(resolution, z_min, z_max)
    if occ is not None:
        img, origin = occ
        write_png_gray(os.path.join(out, "grid.png"), img)
        write_map_yaml(os.path.join(out, "grid.yaml"), "grid.png", resolution, origin)
        files += ["grid.png", "grid.yaml"]
        meta["grid_origin"] = origin
        meta["grid_size"] = [int(img.shape[1]), int(img.shape[0])]
    with builder.lock:
        traj = list(builder.trajectory)
    with open(os.path.join(out, "trajectory.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,yaw\n")
        for t, x, y, yaw in traj:
            f.write(f"{t:.3f},{x:.4f},{y:.4f},{yaw:.4f}\n")
    files.append("trajectory.csv")
    if costmap:
        write_png_gray(os.path.join(out, "costmap.png"),
                       costmap_image(costmap["width"], costmap["height"], costmap["data"]))
        write_map_yaml(os.path.join(out, "costmap.yaml"), "costmap.png", costmap["resolution"],
                       costmap["origin"])
        files += ["costmap.png", "costmap.yaml"]
    meta.update(extra_meta or {})
    meta["files"] = files + ["meta.json"]
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    return meta


def list_exports(root: str) -> List[Dict]:
    out = []
    if not os.path.isdir(root):
        return out
    for name in sorted(os.listdir(root), reverse=True):
        p = os.path.join(root, name, "meta.json")
        if os.path.isfile(p):
            try:
                with open(p, encoding="utf-8") as f:
                    m = json.load(f)
                out.append({"name": name, "points": m.get("points"), "files": m.get("files", []),
                            "created": m.get("created")})
            except (OSError, ValueError):
                continue
    return out
