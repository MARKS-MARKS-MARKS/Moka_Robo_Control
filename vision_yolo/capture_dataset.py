#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import depthai as dai


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "datasets" / "tea_objects"
TRAIN_IMAGES = DATASET / "images" / "train"
VAL_IMAGES = DATASET / "images" / "val"


def capture_frames(count: int, interval: float, width: int, height: int, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    VAL_IMAGES.mkdir(parents=True, exist_ok=True)

    with dai.Pipeline(dai.Device()) as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        capability = dai.ImgFrameCapability()
        capability.size.fixed((width, height))
        capability.fps.fixed(20)
        output = cam.requestOutput(capability, True)
        queue = output.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()
        saved = 0
        print(f"[capture] saving {count} images to {out_dir}")
        while saved < count:
            packet = queue.tryGet()
            if packet is None:
                time.sleep(0.02)
                continue
            frame = packet.getCvFrame()
            stamp = time.strftime("%Y%m%d_%H%M%S")
            filename = out_dir / f"tea_{stamp}_{int(time.time() * 1000) % 1000000:06d}.jpg"
            ok = cv2.imwrite(str(filename), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            if ok:
                saved += 1
                print(f"[capture] {saved:03d}/{count:03d} {filename}")
            time.sleep(max(0.0, interval))


def main():
    parser = argparse.ArgumentParser(description="Capture OAK-D RGB frames for tea object YOLO dataset.")
    parser.add_argument("--count", type=int, default=40, help="number of images to capture")
    parser.add_argument("--interval", type=float, default=0.25, help="seconds between saved frames")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--out", type=Path, default=TRAIN_IMAGES)
    args = parser.parse_args()

    capture_frames(
        count=max(1, args.count),
        interval=max(0.0, args.interval),
        width=max(64, args.width),
        height=max(64, args.height),
        out_dir=args.out,
    )


if __name__ == "__main__":
    main()

