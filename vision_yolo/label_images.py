#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "datasets" / "tea_objects"
IMAGE_DIR = DATASET / "images" / "train"
LABEL_DIR = DATASET / "labels" / "train"
CLASSES = ["cup", "tea_can", "water_bottle"]


class LabelSession:
    def __init__(self, image_path: Path):
        self.image_path = image_path
        self.image = cv2.imread(str(image_path))
        if self.image is None:
            raise RuntimeError(f"failed to read image: {image_path}")
        self.boxes = []
        self.drag_start = None
        self.preview_box = None

    @property
    def label_path(self) -> Path:
        return LABEL_DIR / f"{self.image_path.stem}.txt"

    def load_existing(self):
        if not self.label_path.is_file():
            return
        h, w = self.image.shape[:2]
        for line in self.label_path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            cls, cx, cy, bw, bh = [float(v) for v in parts]
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            self.boxes.append([int(cls), x1, y1, x2, y2])

    def save(self):
        LABEL_DIR.mkdir(parents=True, exist_ok=True)
        h, w = self.image.shape[:2]
        lines = []
        for cls, x1, y1, x2, y2 in self.boxes:
            xa, xb = sorted((max(0, x1), min(w - 1, x2)))
            ya, yb = sorted((max(0, y1), min(h - 1, y2)))
            bw = max(1, xb - xa)
            bh = max(1, yb - ya)
            cx = xa + bw / 2
            cy = ya + bh / 2
            lines.append(f"{cls} {cx / w:.6f} {cy / h:.6f} {bw / w:.6f} {bh / h:.6f}")
        self.label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    def draw(self):
        canvas = self.image.copy()
        for cls, x1, y1, x2, y2 in self.boxes:
            color = [(80, 220, 255), (80, 255, 140), (255, 180, 80)][cls % 3]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(canvas, f"{cls}:{CLASSES[cls]}", (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        if self.preview_box:
            x1, y1, x2, y2 = self.preview_box
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (230, 230, 230), 1)
        cv2.putText(canvas, "drag box, press 0 cup / 1 tea_can / 2 water_bottle, n next, u undo, q quit", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return canvas

    def on_mouse(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.preview_box = (x, y, x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start:
            self.preview_box = (*self.drag_start, x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start:
            x1, y1 = self.drag_start
            self.preview_box = (x1, y1, x, y)
            self.drag_start = None

    def add_preview_as_class(self, cls: int):
        if not self.preview_box:
            return
        x1, y1, x2, y2 = self.preview_box
        if abs(x2 - x1) < 5 or abs(y2 - y1) < 5:
            self.preview_box = None
            return
        xa, xb = sorted((x1, x2))
        ya, yb = sorted((y1, y2))
        self.boxes.append([cls, xa, ya, xb, yb])
        self.preview_box = None
        self.save()


def main():
    parser = argparse.ArgumentParser(description="Simple OpenCV YOLO bbox labeler for tea objects.")
    parser.add_argument("--images", type=Path, default=IMAGE_DIR)
    args = parser.parse_args()

    images = sorted([p for p in args.images.glob("*.jpg")])
    if not images:
        raise SystemExit(f"no jpg images found in {args.images}")

    window = "tea object labeler"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    for index, image_path in enumerate(images, start=1):
        session = LabelSession(image_path)
        session.load_existing()
        cv2.setMouseCallback(window, session.on_mouse)
        while True:
            canvas = session.draw()
            cv2.putText(canvas, f"{index}/{len(images)} {image_path.name}", (10, canvas.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("0"), ord("1"), ord("2")):
                session.add_preview_as_class(int(chr(key)))
            elif key == ord("u"):
                if session.boxes:
                    session.boxes.pop()
                    session.save()
            elif key == ord("n"):
                break
            elif key == ord("q"):
                cv2.destroyAllWindows()
                return
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

