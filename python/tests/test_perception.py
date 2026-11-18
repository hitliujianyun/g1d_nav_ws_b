"""g1d_perception with synthetic camera / LiDAR data (no GPU, no camera)."""
import math
import os

import numpy as np
import pytest

from g1d_perception.association import floor_safe_box, points_in_mask
from g1d_perception.camera import crop_side_by_side
from g1d_perception.core import PerceptionCore, PerceptionParams, camera_extrinsic
from g1d_perception.geometry import (R_BODY_OPTICAL, PinholeCamera, PoseBuffer, apply, inverse,
                                     quat_xyzw_to_matrix, transform)
from g1d_perception.segmentation import CallableBackend, Detection

CAM = PinholeCamera([300.0, 0, 320.0, 0, 300.0, 240.0, 0, 0, 1], 640, 480)
T_IMU_CAM = transform(R_BODY_OPTICAL, [0.0, 0.0, 0.0])   # camera at the IMU, looking along +x
PERSON = (2.8, 3.2, -0.25, 0.25, -1.2, 0.5)              # x0 x1 y0 y1 z0 z1 (LIO world)


def _ray_scene(origin, dirs):
    """Range to wall x = 6, floor z = -1.2 and the person box; also 'on person' flag."""
    best = np.full(dirs.shape[0], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = (6.0 - origin[0]) / dirs[:, 0]
        best = np.where((r > 0), r, best)
        r = (-1.2 - origin[2]) / dirs[:, 2]
        best = np.where((r > 0) & (r < best), r, best)
        lo = (np.array(PERSON[0::2]) - origin) / dirs
        hi = (np.array(PERSON[1::2]) - origin) / dirs
    tmin = np.nanmax(np.minimum(lo, hi), axis=1)
    tmax = np.nanmin(np.maximum(lo, hi), axis=1)
    hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0) & (tmin < best)
    best = np.where(hit, tmin, best)
    return best, hit


def _lidar_points(n=60000, seed=0):
    rng = np.random.default_rng(seed)
    az = rng.uniform(-math.pi, math.pi, n)
    el = rng.uniform(-0.9, 0.3, n)
    d = np.c_[np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
    origin = np.array([0.0, 0.0, 0.05])
    r, hit = _ray_scene(origin, d)
    ok = np.isfinite(r) & (r < 30)
    return (origin + d[ok] * r[ok, None]).astype(np.float32), hit[ok]


def _person_mask(dilate_px=4):
    v, u = np.mgrid[0:480, 0:640]
    x = (u.ravel() - CAM.cx) / CAM.fx
    y = (v.ravel() - CAM.cy) / CAM.fy
    d_opt = np.c_[x, y, np.ones_like(x)]
    d = d_opt @ R_BODY_OPTICAL.T
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    _, hit = _ray_scene(np.zeros(3), d)
    m = hit.reshape(480, 640)
    for _ in range(dilate_px):      # imperfect mask edges leak background
        g = m.copy()
        g[1:] |= m[:-1]; g[:-1] |= m[1:]; g[:, 1:] |= m[:, :-1]; g[:, :-1] |= m[:, 1:]
        m = g
    return m


def test_pinhole_projection_and_pose_interpolation():
    uv, z, ok = CAM.project(np.array([[0.0, 0.0, 2.0], [0.3, -0.15, 1.5], [0.0, 0.0, -1.0]]))
    assert ok.tolist() == [True, True, False]
    np.testing.assert_allclose(uv[0], [320, 240])
    np.testing.assert_allclose(uv[1], [320 + 300 * 0.2, 240 - 300 * 0.1])
    pb = PoseBuffer()
    pb.add(1.0, [0, 0, 0, 1], [0, 0, 0])
    yaw = math.pi / 2
    pb.add(2.0, [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)], [2, 0, 0])
    m = pb.at(1.5)
    np.testing.assert_allclose(m[:3, 3], [1, 0, 0])
    np.testing.assert_allclose(math.atan2(m[1, 0], m[0, 0]), yaw / 2, atol=1e-9)
    assert pb.at(0.5) is None and pb.at(2.2) is None


def test_camera_extrinsic_chain_from_config():
    cfg = {"lio": {"extrinsic_T": [0.0, 0.0, 0.05]},
           "kinematics": {"head_to_livox": {"xyz": [0.05, 0.0, 0.55], "rpy": [math.pi, 0.0, 0.0]}},
           "perception": {"camera": {"head_to_camera": {"xyz": [0.1, 0.0, 0.0], "rpy": [0, 0, 0]}}}}
    T = camera_extrinsic(cfg)
    # Optical z (forward) of the camera = head +x; seen from the inverted LiDAR / IMU = +x too.
    np.testing.assert_allclose(T[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-9)
    # Camera 0.1 m ahead and 0.55 m below the (inverted) LiDAR -> IMU frame z = +0.55 + 0.05.
    np.testing.assert_allclose(T[:3, 3], [0.05, 0.0, 0.6], atol=1e-9)


def test_mask_points_depth_gating_rejects_leaking_background():
    pts, on_person = _lidar_points()
    pc = apply(inverse(T_IMU_CAM), pts)
    uv, depth, valid = CAM.project(pc)
    mask = _person_mask(dilate_px=6)
    idx = points_in_mask(uv, depth, valid, mask)
    inside = np.flatnonzero(valid & mask[np.clip(uv[:, 1].astype(int), 0, 479),
                                         np.clip(uv[:, 0].astype(int), 0, 639)])
    wall = pts[:, 0] > 5.9
    assert wall[inside].sum() > 20                    # the dilated mask does leak wall points
    assert wall[idx].sum() == 0
    # Floor returns at the feet share the person's depth; the core drops them by height.
    above_floor = idx[pts[idx, 2] > -1.15]
    precision = on_person[above_floor].mean()
    recall = on_person[idx].sum() / on_person[inside].sum()
    assert precision > 0.99 and recall > 0.9


def test_core_publishes_points_and_confirmed_floor_safe_boxes():
    mask = _person_mask(dilate_px=4)
    backend = CallableBackend(lambda img: [Detection(mask, 0.9)])
    core = PerceptionCore(CAM, T_IMU_CAM, backend, PerceptionParams(confirm_hits=2))
    T_odom_lio = transform(np.eye(3), [0.0, 0.0, 1.2])     # odom floor at z = 0
    core.set_T_odom_lio(T_odom_lio)
    img = np.zeros((480, 640, 3), np.uint8)
    assert core.process(10.0, img).skipped == "no_pose"
    for t in (9.9, 10.0, 10.1, 10.2, 10.3):
        core.add_pose(t, [0, 0, 0, 1], [0, 0, 0])
    assert core.process(10.0, img).skipped == "no_cloud"
    pts, on_person = _lidar_points()
    core.add_cloud(10.0, pts)
    r1 = core.process(10.02, img)
    assert len(r1.persons) == 1 and not r1.delete_boxes          # needs confirmation
    on = np.isin(r1.person_points.view([("", np.float32)] * 3), pts[on_person].view([("", np.float32)] * 3))
    assert on.mean() > 0.97 and r1.person_points.shape[0] > 100
    r2 = core.process(10.12, img)
    assert len(r2.delete_boxes) == 1
    x0, x1, y0, y1, z0, z1 = r2.delete_boxes[0]
    assert x0 < 2.85 and x1 > 2.8 and y0 < -0.2 and y1 > 0.2 and z1 > 0.45
    assert z0 >= -1.2 + 0.05 - 1e-6                               # floor stays in the map
    assert not core.process(10.2, img).delete_boxes               # rate limited per person
    # Without T_odom_lio the box simply encloses the points (+ margin).
    b = floor_safe_box(pts[on_person].astype(np.float64), 0.1, None)
    assert b[4] < -1.2


def test_side_by_side_crop():
    img = np.zeros((4, 8, 3), np.uint8)
    img[:, 4:] = 255
    assert crop_side_by_side(img, "left").max() == 0
    assert crop_side_by_side(img, "right").min() == 255
    assert crop_side_by_side(img, "full").shape == (4, 8, 3)


def test_yolo_labels_round_trip_and_teacher_labelling(tmp_path):
    cv2 = pytest.importorskip("cv2")
    from g1d_perception.dataset import polygons_to_mask, yolo_seg_lines
    from g1d_perception.tools.sam3_label import label_session
    from g1d_perception.tools.train_yolo_seg import build_dataset
    mask = np.zeros((120, 160), bool)
    mask[30:100, 50:80] = True
    mask[20:35, 58:72] = True                                   # head
    lines = yolo_seg_lines([mask])
    back = polygons_to_mask(lines, 120, 160)
    iou = (back & mask).sum() / (back | mask).sum()
    assert len(lines) == 1 and lines[0].startswith("0 ") and iou > 0.9

    for s in ("s1", "s2"):
        d = tmp_path / s / "images"
        d.mkdir(parents=True)
        for i in range(10):
            cv2.imwrite(str(d / f"{i:06d}.jpg"), np.full((120, 160, 3), i * 20, np.uint8))
    calls = []

    def fake_teacher(rgb):
        calls.append(rgb.shape)
        return ([mask, np.zeros_like(mask)], [0.9, 0.95])        # empty mask is ignored
    stats = label_session(str(tmp_path / "s1"), fake_teacher)
    assert stats["labelled"] == 10 and stats["persons"] == 10 and calls[0] == (120, 160, 3)
    assert label_session(str(tmp_path / "s1"), fake_teacher)["skipped"] == 10
    data, n_train, n_val = build_dataset([str(tmp_path / "s1"), str(tmp_path / "s2")],
                                         str(tmp_path / "yolo"), link=False)
    assert n_train + n_val == 10 and n_val >= 1                  # s2 has no labels yet
    assert "0: person" in open(data, encoding="utf-8").read()


def test_node_publishes_person_points_and_map_delete():
    pytest.importorskip("unitree_sdk2py")
    from g1d_nav_py import msgs
    from g1d_perception.node import PerceptionNode
    mask = _person_mask()
    core = PerceptionCore(CAM, T_IMU_CAM, CallableBackend(lambda img: [Detection(mask, 0.9)]))
    node = PerceptionNode({}, core)

    class Pub:
        def __init__(self):
            self.msgs = []

        def Write(self, m):
            self.msgs.append(m)
    node.pubs = {"person": Pub(), "delete": Pub(), "status": Pub()}
    for t in (0.9, 1.0, 1.1, 1.2):
        core.add_pose(t, [0, 0, 0, 1], [0, 0, 0])
    pts, _ = _lidar_points()
    node._on_cloud(msgs.make_cloud_xyzl(pts, np.zeros((0, 3)), "odom_lio", 1.0))
    img = np.zeros((480, 640, 3), np.uint8)
    node.handle_frame(1.0, img)
    node.handle_frame(1.1, img)
    assert len(node.pubs["person"].msgs) == 2
    assert msgs.read_cloud_xyz(node.pubs["person"].msgs[0]).shape[0] > 100
    d = msgs.parse_json(node.pubs["delete"].msgs[0])
    assert d["frame"] == "odom_lio" and len(d["boxes"]) == 1 and len(d["boxes"][0]) == 6
    node.publish_status()
    st = msgs.parse_json(node.pubs["status"].msgs[0])
    assert st["frames"] == 2 and st["deleted_boxes"] == 1
