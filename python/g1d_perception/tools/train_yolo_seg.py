"""Distil SAM 3 labels into a small YOLO-seg person model (5090), export for the Orin.

    python -m g1d_perception.tools.train_yolo_seg datasets/s1 datasets/s2 ... \
        --model yolo11n-seg.pt --epochs 100 --imgsz 640 --out runs/person_seg

Builds an Ultralytics dataset (images + labels symlinked/copied into train/val by session-hash
split, data.yaml with the single class "person"), trains, validates and exports ONNX. The
TensorRT engine must be built on the Orin itself (same TensorRT version as at run time):

    yolo export model=best.pt format=engine half=True imgsz=640 device=0      # on PC2

Acceptance (docs/DYNAMIC_OBJECTS_STUDY.md P3): person recall >= 95 % within 3 m, < 30 ms/frame.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
from typing import List, Sequence, Tuple


def build_dataset(sessions: Sequence[str], out: str, val_fraction: float = 0.15,
                  link: bool = True) -> Tuple[str, int, int]:
    """Returns (data.yaml path, train images, val images). Frames are split by a hash of
    session/frame so that re-running gives the same split."""
    counts = {"train": 0, "val": 0}
    for split in ("train", "val"):
        os.makedirs(os.path.join(out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(out, "labels", split), exist_ok=True)
    for s in sessions:
        tag = os.path.basename(os.path.normpath(s))
        lab_dir = os.path.join(s, "labels")
        for name in sorted(os.listdir(os.path.join(s, "images"))):
            stem, ext = os.path.splitext(name)
            lab = os.path.join(lab_dir, stem + ".txt")
            if ext != ".jpg" or not os.path.exists(lab):
                continue
            h = int(hashlib.md5(f"{tag}/{stem}".encode()).hexdigest(), 16) % 1000
            split = "val" if h < val_fraction * 1000 else "train"
            dst_img = os.path.join(out, "images", split, f"{tag}_{name}")
            dst_lab = os.path.join(out, "labels", split, f"{tag}_{stem}.txt")
            src_img = os.path.abspath(os.path.join(s, "images", name))
            if not os.path.exists(dst_img):
                if link:
                    try:
                        os.symlink(src_img, dst_img)
                    except OSError:
                        shutil.copy2(src_img, dst_img)
                else:
                    shutil.copy2(src_img, dst_img)
            shutil.copy2(lab, dst_lab)
            counts[split] += 1
    yaml_path = os.path.join(out, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {os.path.abspath(out)}\ntrain: images/train\nval: images/val\n"
                "names:\n  0: person\n")
    return yaml_path, counts["train"], counts["val"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--model", default="yolo11n-seg.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--out", default="runs/person_seg")
    ap.add_argument("--data-dir", default="datasets/_yolo_person")
    args = ap.parse_args(argv)
    data, n_train, n_val = build_dataset(args.sessions, args.data_dir)
    print(f"dataset {data}: {n_train} train / {n_val} val images")
    from ultralytics import YOLO
    model = YOLO(args.model)
    model.train(data=data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                project=os.path.dirname(args.out) or ".", name=os.path.basename(args.out),
                single_cls=True, exist_ok=True)
    metrics = model.val(data=data, imgsz=args.imgsz)
    print(metrics.results_dict if hasattr(metrics, "results_dict") else metrics)
    model.export(format="onnx", imgsz=args.imgsz, dynamic=False, simplify=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
