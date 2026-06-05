#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2

from capture_dataset import capture_frames, ROOT


OUTPUT = ROOT / "output"
TMP = OUTPUT / "capture_tmp"


def main():
    parser = argparse.ArgumentParser(description="Capture one OAK-D frame and run YOLO detection. No robot motion.")
    parser.add_argument("--model", required=True, help="path to trained YOLO model, e.g. runs/detect/tea_objects/weights/best.pt")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise SystemExit(
            "ultralytics is not installed. Install it first:\n"
            "  python3 -m pip install ultralytics\n"
            f"import error: {exc}"
        )

    OUTPUT.mkdir(parents=True, exist_ok=True)
    TMP.mkdir(parents=True, exist_ok=True)
    for old in TMP.glob("*.jpg"):
        old.unlink()
    capture_frames(count=1, interval=0, width=640, height=360, out_dir=TMP)
    image_path = sorted(TMP.glob("*.jpg"))[-1]

    model = YOLO(args.model)
    results = model.predict(source=str(image_path), conf=args.conf, verbose=False)
    result = results[0]
    annotated = result.plot()
    out_image = OUTPUT / "latest_detection.jpg"
    cv2.imwrite(str(out_image), annotated)

    detections = []
    names = result.names
    for box in result.boxes:
        cls = int(box.cls[0])
        xyxy = [float(v) for v in box.xyxy[0].tolist()]
        detections.append(
            {
                "class_id": cls,
                "class_name": names.get(cls, str(cls)),
                "confidence": float(box.conf[0]),
                "xyxy": xyxy,
                "center_px": [(xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2],
            }
        )

    out_json = OUTPUT / "latest_detection.json"
    out_json.write_text(json.dumps({"image": str(image_path), "detections": detections}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[detect] image: {out_image}")
    print(f"[detect] json : {out_json}")
    print(json.dumps(detections, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

