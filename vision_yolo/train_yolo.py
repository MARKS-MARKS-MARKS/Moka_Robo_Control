#!/usr/bin/env python3
import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DATA_YAML = ROOT / "tea_objects.yaml"


def main():
    parser = argparse.ArgumentParser(description="Train YOLO for tea scene objects.")
    parser.add_argument("--model", default="yolov8n.pt", help="base YOLO model")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise SystemExit(
            "ultralytics is not installed. Install it first:\n"
            "  python3 -m pip install ultralytics\n"
            f"import error: {exc}"
        )

    model = YOLO(args.model)
    model.train(
        data=str(DATA_YAML),
        epochs=max(1, args.epochs),
        imgsz=max(64, args.imgsz),
        batch=max(1, args.batch),
        device=args.device,
        project="runs/detect",
        name="tea_objects",
        exist_ok=True,
    )


if __name__ == "__main__":
    main()

