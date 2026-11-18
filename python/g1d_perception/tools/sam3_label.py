"""SAM 3 as teacher: person masks for recorded head-camera frames -> YOLO-seg labels.

    python -m g1d_perception.tools.sam3_label datasets/<session> [--prompt person] [--min-score 0.5]

Runs on the 5090 (SAM 3 needs a CUDA GPU, PyTorch >= 2.7 and access to the facebook/sam3
checkpoints: `pip install -e .` in a clone of github.com/facebookresearch/sam3, then
`hf auth login`). Writes labels/NNNNNN.txt (class 0 = person, normalised polygons) and
labels/summary.json. Frames whose label file exists are skipped unless --overwrite.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from typing import Callable, List, Tuple

import numpy as np

from ..dataset import yolo_seg_lines

Segmenter = Callable[[np.ndarray], Tuple[List[np.ndarray], List[float]]]


def sam3_segmenter(prompt: str) -> Segmenter:
    """Returns rgb (H, W, 3) uint8 -> (masks [(H, W) bool], scores)."""
    from PIL import Image
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model
    model = build_sam3_image_model()
    processor = Sam3Processor(model)

    def run(rgb: np.ndarray):
        state = processor.set_image(Image.fromarray(rgb))
        out = processor.set_text_prompt(state=state, prompt=prompt)
        masks = out["masks"]
        scores = out["scores"]
        masks = masks.detach().cpu().numpy() if hasattr(masks, "detach") else np.asarray(masks)
        scores = scores.detach().cpu().numpy() if hasattr(scores, "detach") else np.asarray(scores)
        masks = masks.reshape(-1, *masks.shape[-2:]) > 0.5
        return [m for m in masks], [float(s) for s in scores.reshape(-1)]

    return run


def label_session(session: str, segment: Segmenter, *, min_score: float = 0.5,
                  overwrite: bool = False) -> dict:
    import cv2
    img_dir, lab_dir = os.path.join(session, "images"), os.path.join(session, "labels")
    os.makedirs(lab_dir, exist_ok=True)
    stats = {"images": 0, "labelled": 0, "persons": 0, "skipped": 0, "seconds": 0.0}
    t0 = time.time()
    for name in sorted(os.listdir(img_dir)):
        if not name.endswith(".jpg"):
            continue
        stats["images"] += 1
        out = os.path.join(lab_dir, os.path.splitext(name)[0] + ".txt")
        if os.path.exists(out) and not overwrite:
            stats["skipped"] += 1
            continue
        bgr = cv2.imread(os.path.join(img_dir, name), cv2.IMREAD_COLOR)
        masks, scores = segment(bgr[:, :, ::-1].copy())
        keep = [m for m, s in zip(masks, scores) if s >= min_score and m.any()]
        lines = yolo_seg_lines(keep)
        with open(out, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))   # empty file = no person
        stats["labelled"] += 1
        stats["persons"] += len(keep)
    stats["seconds"] = round(time.time() - t0, 1)
    with open(os.path.join(lab_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--prompt", default="person")
    ap.add_argument("--min-score", type=float, default=0.5)
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args(argv)
    print(json.dumps(label_session(args.session, sam3_segmenter(args.prompt),
                                   min_score=args.min_score, overwrite=args.overwrite)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
