"""Moving-object handling on the Python side: labelled clouds, keyframes, map cleaning, fake nav."""
import math
import os

import numpy as np
import pytest

from g1d_web import mapping


# ------------------------------------------------------------------------------- cloud format
def test_xyzl_cloud_round_trip_and_xyz_compat():
    from g1d_nav_py import msgs
    s = np.array([[1, 2, 3], [4, 5, 6]], np.float32)
    d = np.array([[7, 8, 9]], np.float32)
    m = msgs.make_cloud_xyzl(s, d, "odom_lio", 3.5)
    assert m.point_step == 16 and m.width == 3
    rs, rd = msgs.read_cloud_xyzl(m)
    np.testing.assert_array_equal(rs, s)
    np.testing.assert_array_equal(rd, d)
    np.testing.assert_array_equal(msgs.read_cloud_xyz(m), np.vstack([s, d]))  # old readers
    rs, rd = msgs.read_cloud_xyzl(msgs.make_cloud_xyz(s, "x", 0.0))             # no label field
    np.testing.assert_array_equal(rs, s)
    assert rd.shape == (0, 3)


# ----------------------------------------------------------------------- synthetic sequences
def _scan(origin, box_y, n, rng):
    """Room +-6 m, floor -1.2 / ceiling 1.6 relative to the sensor, 0.4 m box crossing x = 2.5."""
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    best = np.full(n, 1e9)
    lab = np.zeros(n, bool)
    for axis, val in ((0, 6.0), (0, -6.0), (1, 6.0), (1, -6.0), (2, -1.2), (2, 1.6)):
        with np.errstate(divide="ignore", invalid="ignore"):
            r = (val - origin[axis]) / d[:, axis]
        ok = (r > 0) & (r < best)
        best[ok] = r[ok]
    with np.errstate(divide="ignore", invalid="ignore"):
        r = (2.5 - origin[0]) / d[:, 0]
    q = origin + r[:, None] * d
    hit = ((d[:, 0] > 1e-6) & (np.abs(q[:, 1] - box_y) < 0.2) & (q[:, 2] > -1.2) & (q[:, 2] < 0.5)
           & (r < best))
    best[hit] = r[hit]
    lab[hit] = True
    return (origin + best[:, None] * d).astype(np.float32), lab


def _record(tmp_path, frames=30, n=20000):
    """Static sensor; box walks y = -2 .. 2.35. Returns (recorder dir, map points, truth)."""
    rng = np.random.default_rng(0)
    rec = mapping.KeyframeRecorder(str(tmp_path / "rec"), min_dist=0.0, min_yaw=0.0, min_dt=0.0,
                                   voxel=0.0)
    d = rec.start("s1")
    builder = mapping.MapBuilder(0.05)
    truth = []
    for k in range(frames):
        pts, lab = _scan(np.zeros(3), -2.0 + 0.15 * k, n, rng)
        # The online labeller missed the box: everything goes into the map (worst case).
        assert rec.add(0.1 * k, pts, np.zeros((0, 3), np.float32), np.zeros(3), np.array([1, 0, 0, 0]))
        builder.add_cloud(pts)
        truth.append(pts[lab])
    return d, builder, np.concatenate(truth)


def _box_trail_points(pts):
    """Map points on the box trail that was seen through later (y < 1.7)."""
    return ((np.abs(pts[:, 0] - 2.5) < 0.06) & (pts[:, 1] < 1.7) & (pts[:, 2] > -1.1)
            & (pts[:, 2] < 0.5))


def test_keyframe_pcd_round_trip_and_spacing(tmp_path):
    pts = np.array([[1, 2, 3], [4, 5, 6]], np.float32)
    path = str(tmp_path / "f.pcd")
    mapping.write_pcd_frame(path, pts, np.array([0, 1]), [1.0, 2.0, 3.0], [0.0, 1.0, 0.0, 0.0])
    p, lab, o, q = mapping.read_pcd_frame(path)
    np.testing.assert_array_equal(p, pts)
    assert lab.tolist() == [0, 1] and o.tolist() == [1, 2, 3] and q.tolist() == [0, 1, 0, 0]

    rec = mapping.KeyframeRecorder(str(tmp_path), min_dist=0.3, min_yaw=0.3, min_dt=0.5,
                                   max_dt=5.0, voxel=0.05)
    assert not rec.add(0.0, pts, pts[:0], np.zeros(3), np.array([1, 0, 0, 0]))  # not started
    rec.start("run")
    q0 = np.array([1.0, 0, 0, 0])
    assert rec.add(0.0, pts, pts[:1], np.zeros(3), q0)              # first frame
    assert not rec.add(1.0, pts, pts[:0], np.array([0.1, 0, 0]), q0)  # moved too little
    assert not rec.add(0.2, pts, pts[:0], np.array([1.0, 0, 0]), q0)  # too soon
    assert rec.add(1.2, pts, pts[:0], np.array([0.4, 0, 0]), q0)     # moved enough
    yaw = 0.4
    assert rec.add(2.0, pts, pts[:0], np.array([0.4, 0, 0]),
                   np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))  # turned enough
    assert rec.add(7.5, pts, pts[:0], np.array([0.4, 0, 0]),
                   np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))  # max_dt
    files = mapping.list_frames(rec.stats()["dir"])
    assert len(files) == 4
    _, lab, _, _ = mapping.read_pcd_frame(files[0])
    assert lab.sum() == 1                                            # M-detector label kept
    with open(os.path.join(rec.stats()["dir"], "poses.csv"), encoding="utf-8") as f:
        assert len(f.read().strip().splitlines()) == 5


@pytest.mark.skipif(not mapping.dufomap_available(), reason="dufomap not installed")
@pytest.mark.parametrize("isolate", [False, True])
def test_dufomap_cleaning_removes_trail_keeps_walls(tmp_path, isolate):
    d, builder, truth = _record(tmp_path)
    pts = builder.points()
    keep, info = mapping.clean_map(pts, mapping.list_frames(d), method="dufomap", isolate=isolate)
    trail = _box_trail_points(pts)
    walls = (np.abs(np.abs(pts[:, 0]) - 6.0) < 0.05) | (np.abs(np.abs(pts[:, 1]) - 6.0) < 0.05)
    assert info["method"] == "dufomap" and trail.sum() > 200
    assert info.get("isolated", False) == isolate
    assert (~keep[trail]).mean() > 0.85
    assert keep[walls].mean() > 0.995


@pytest.mark.skipif(not mapping.dufomap_available(), reason="dufomap not installed")
def test_flow_frames_export_labels_every_scan(tmp_path):
    from g1d_perception.tools.flow_frames import export
    # Real Mid-360S density (~20k points / scan); at half density DUFOMap with d_p = 1 leaves
    # most of the box "unknown" in individual scans.
    d, _, _ = _record(tmp_path, frames=20, n=20000)
    stats = export(d, str(tmp_path / "flow"))
    files = sorted(os.listdir(tmp_path / "flow"))
    assert stats["frames"] == 20 and len([f for f in files if f.endswith(".npz")]) == 20
    z = np.load(tmp_path / "flow" / "000005.npz")
    assert z["points"].shape[0] == z["dufo"].shape[0] == z["mdet"].shape[0]
    np.testing.assert_allclose(z["pose"], np.eye(4))
    # Over all scans: box positions vacated later are mostly dynamic (a slowly moving object
    # overlaps its next positions, which DUFOMap keeps as hits), static structure never is.
    box_lab, wall_lab = [], []
    for k in range(20):
        f = np.load(tmp_path / "flow" / f"{k:06d}.npz")
        p, lab = f["points"], f["dufo"]
        y_box = -2.0 + 0.15 * k
        box = (np.abs(p[:, 0] - 2.5) < 0.05) & (p[:, 2] > -1.0) & (y_box < 0.3)
        wall = (np.abs(np.abs(p[:, 0]) - 6.0) < 0.05) | (np.abs(np.abs(p[:, 1]) - 6.0) < 0.05)
        box_lab.append(lab[box])
        wall_lab.append(lab[wall])
    assert np.concatenate(box_lab).mean() > 0.5
    assert np.concatenate(wall_lab).mean() < 0.001


def test_carving_fallback_removes_trail_keeps_walls(tmp_path):
    d, builder, _ = _record(tmp_path, frames=30, n=8000)
    pts = builder.points()
    keep, info = mapping.clean_map(pts, mapping.list_frames(d), method="carving")
    trail = _box_trail_points(pts)
    walls = (np.abs(np.abs(pts[:, 0]) - 6.0) < 0.05) | (np.abs(np.abs(pts[:, 1]) - 6.0) < 0.05)
    floor = np.abs(pts[:, 2] + 1.2) < 0.03
    assert info["method"] == "carving"
    assert (~keep[trail]).mean() > 0.7
    assert keep[walls].mean() > 0.99 and keep[floor].mean() > 0.97


def test_export_with_keyframes_writes_raw_and_clean_maps(tmp_path):
    d, builder, _ = _record(tmp_path, frames=12, n=6000)
    meta = mapping.export_map(builder, str(tmp_path / "maps"), frame_files=mapping.list_frames(d),
                              clean={"method": "carving"})
    out = tmp_path / "maps" / meta["name"]
    raw = mapping.read_pcd_binary(str(out / "map_raw.pcd"))
    clean = mapping.read_pcd_binary(str(out / "map.pcd"))
    assert "map_raw.pcd" in meta["files"] and meta["cleaning"]["removed"] > 0
    assert clean.shape[0] == raw.shape[0] - meta["cleaning"]["removed"] == meta["points"]
    assert (out / "grid.png").exists()
    # No keyframes -> no cleaning, map.pcd only.
    meta2 = mapping.export_map(builder, str(tmp_path / "maps2"))
    assert "cleaning" not in meta2 and "map_raw.pcd" not in meta2["files"]


# ---------------------------------------------------------------------------------- hub / app
def test_hub_sensor_pose_from_lio_odom_and_extrinsic():
    from g1d_web.hub import Hub, matrix_to_quat_wxyz, quat_to_matrix
    hub = Hub()
    hub.json["debug"] = {"T_odom_lio": {"valid": True, "t": [1.0, 0.0, 1.2],
                                        "q": [1.0, 0.0, 0.0, 0.0]}}   # x, y, z, w: roll pi
    hub.t_imu_lidar = np.array([0.0, 0.0, 0.05])
    hub.lio_poses.append((5.0, np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])))
    assert hub.sensor_pose_odom(6.0) is None
    origin, q = hub.sensor_pose_odom(5.0)
    np.testing.assert_allclose(origin, [1.5, 0.0, 1.15], atol=1e-9)
    np.testing.assert_allclose(quat_to_matrix([q[1], q[2], q[3], q[0]]),
                               np.diag([1.0, -1.0, -1.0]), atol=1e-9)
    r = quat_to_matrix([0.1, -0.3, 0.2, 0.9] / np.linalg.norm([0.1, -0.3, 0.2, 0.9]))
    qq = matrix_to_quat_wxyz(r)
    np.testing.assert_allclose(quat_to_matrix([qq[1], qq[2], qq[3], qq[0]]), r, atol=1e-9)


def test_app_keeps_moving_points_out_of_the_map_and_records(tmp_path):
    from g1d_web.app import WebApp
    from g1d_web.hub import Hub
    hub = Hub()
    hub.json["debug"] = {"T_odom_lio": {"valid": True, "t": [0, 0, 0], "q": [0, 0, 0, 1]}}
    hub.lio_poses.append((1.0, np.zeros(3), np.array([0, 0, 0, 1.0])))
    app = WebApp({"web": {"keyframes": {"min_dt": 0.0}}}, hub, str(tmp_path / "c.yaml"),
                 str(tmp_path))
    static = np.array([[1, 0, 0], [2, 0, 0]], np.float32)
    moving = np.array([[3, 0, 1]], np.float32)
    app._on_cloud(1.0, static, moving)
    assert app.builder.n_points == 2
    np.testing.assert_array_equal(app.latest_dynamic, moving)
    rec = app.recorder.stats()
    assert rec["active"] and rec["frames"] == 1
    _, lab, _, _ = mapping.read_pcd_frame(mapping.list_frames(rec["dir"])[0])
    assert sorted(lab.tolist()) == [0, 0, 1]
    app.owner = "c1"

    class C:
        id = "c1"
    app.handle(C(), {"type": "map", "action": "record", "on": False})
    assert not app.recorder.active


# ----------------------------------------------------------------------------------- fake nav
def test_fake_nav_pedestrian_labels():
    from g1d_nav_py.fake_nav import FakeNav, Pedestrian, make_world3d
    ped = Pedestrian((3.0, -1.0), (3.0, 1.0), speed=1.0)
    assert ped.position(0.0).tolist() == [3.0, -1.0]
    assert ped.position(1.0).tolist() == pytest.approx([3.0, 0.0])
    assert ped.position(3.0).tolist() == pytest.approx([3.0, 0.0])     # walking back
    nav = FakeNav(make_world3d("empty"), rays_per_frame=6000, pedestrians=[ped])
    nav.pose = (0.0, 0.0, 0.0)
    pts, _, moving = nav.lidar_frame_labelled(1.0)                       # pedestrian at (3, 0)
    on_ped = (np.abs(pts[:, 0] - 2.8) < 0.05) & (np.abs(pts[:, 1]) < 0.21) & (pts[:, 2] > 0.05)
    assert moving.sum() > 20
    assert moving[on_ped].all() and on_ped[moving].mean() > 0.95
    pp = nav.person_points(pts, moving)
    assert pp.shape[0] == moving.sum()                                   # straight ahead, 3 m
