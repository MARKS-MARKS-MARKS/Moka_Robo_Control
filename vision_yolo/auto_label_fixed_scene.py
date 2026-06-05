#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "datasets" / "tea_objects"
IMAGE_DIR = DATASET / "images" / "train"
LABEL_DIR = DATASET / "labels" / "train"
OUTPUT_DIR = ROOT / "output"

DEFAULT_BOXES = {
    "tea_can": [118, 153, 247, 291],
    "cup": [316, 126, 427, 256],
    "water_bottle": [402, 0, 585, 146],
}
CLASS_IDS = {
    "cup": 0,
    "tea_can": 1,
    "water_bottle": 2,
}
COLORS = {
    "cup": (80, 220, 255),
    "tea_can": (80, 255, 140),
    "water_bottle": (255, 180, 80),
}


def clamp_box(box, width, height):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width - 1, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height - 1, y2))
    xa, xb = sorted((x1, x2))
    ya, yb = sorted((y1, y2))
    if xb - xa < 4 or yb - ya < 4:
        raise ValueError(f"box too small after clamp: {box}")
    return xa, ya, xb, yb


def to_yolo_line(class_name, box, width, height):
    x1, y1, x2, y2 = clamp_box(box, width, height)
    bw = x2 - x1
    bh = y2 - y1
    cx = x1 + bw / 2
    cy = y1 + bh / 2
    return f"{CLASS_IDS[class_name]} {cx / width:.6f} {cy / height:.6f} {bw / width:.6f} {bh / height:.6f}"


def label_images(image_dir: Path, label_dir: Path, boxes: dict, overwrite: bool):
    images = sorted(image_dir.glob("*.jpg"))
    if not images:
        raise SystemExit(f"no jpg images found in {image_dir}")
    label_dir.mkdir(parents=True, exist_ok=True)
    written = 0
    first_preview = None
    for image_path in images:
        label_path = label_dir / f"{image_path.stem}.txt"
        if label_path.exists() and not overwrite:
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"[skip] failed to read {image_path}")
            continue
        height, width = image.shape[:2]
        lines = [to_yolo_line(name, box, width, height) for name, box in boxes.items()]
        label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        written += 1

        if first_preview is None:
            preview = image.copy()
            for name, box in boxes.items():
                x1, y1, x2, y2 = clamp_box(box, width, height)
                color = COLORS.get(name, (255, 255, 255))
                cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
                cv2.putText(preview, name, (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
            first_preview = preview

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    preview_path = OUTPUT_DIR / "fixed_scene_auto_labels_preview.jpg"
    if first_preview is not None:
        cv2.imwrite(str(preview_path), first_preview)
    print(f"[auto-label] images: {len(images)}")
    print(f"[auto-label] labels written: {written}")
    print(f"[auto-label] preview: {preview_path}")


def parse_boxes(path: Path | None):
    if not path:
        return DEFAULT_BOXES
    payload = json.loads(path.read_text(encoding="utf-8"))
    boxes = {}
    for name in CLASS_IDS:
        if name in payload:
            boxes[name] = payload[name]
    missing = [name for name in CLASS_IDS if name not in boxes]
    if missing:
        raise SystemExit(f"missing boxes in {path}: {', '.join(missing)}")
    return boxes


def main():
    parser = argparse.ArgumentParser(description="Auto-label current fixed tea scene with constant boxes.")
    parser.add_argument("--images", type=Path, default=IMAGE_DIR)
    parser.add_argument("--labels", type=Path, default=LABEL_DIR)
    parser.add_argument("--boxes-json", type=Path, default=None)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    boxes = parse_boxes(args.boxes_json)
    label_images(args.images, args.labels, boxes, args.overwrite)


if __name__ == "__main__":
    main()

