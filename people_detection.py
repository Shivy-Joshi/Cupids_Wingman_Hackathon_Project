#!/usr/bin/env python3
"""
yolo_person_webcam.py
YOLOv8n person detection from a video source (default: laptop webcam).

Now: uses your laptop webcam (source=0).
Later: point --source to a Pi stream URL (MJPEG/RTSP/etc).

Examples:
  # Webcam
  python yolo_person_webcam.py

  # Choose a different webcam index
  python yolo_person_webcam.py --source 1

  # Pi MJPEG stream (example)
  python yolo_person_webcam.py --source "http://<PI_IP>:8080/?action=stream"

  # RTSP stream (example)
  python yolo_person_webcam.py --source "rtsp://<PI_IP>:8554/stream"

Install :
  pip install ultralytics opencv-python
"""

import argparse
import time
from collections import deque

import cv2
from ultralytics import YOLO


PERSON_CLASS_ID = 0  # COCO: 0 = person


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="0",
                   help='Video source: webcam index ("0") OR file path OR stream URL.')
    p.add_argument("--conf", type=float, default=0.35, help="Confidence threshold.")
    p.add_argument("--imgsz", type=int, default=640, help="Inference image size (e.g., 320/480/640).")
    p.add_argument("--max-det", type=int, default=10, help="Max detections per frame.")
    p.add_argument("--show", action="store_true", help="Show a window with overlays.")
    p.add_argument("--no-show", dest="show", action="store_false", help="Disable window display.")
    p.set_defaults(show=True)
    p.add_argument("--flip", action="store_true", help="Flip image horizontally (mirror view).")
    p.add_argument("--device", default=None,
                   help='Device override (e.g., "cpu", "0" for GPU). Default: auto.')
    p.add_argument("--every", type=int, default=1,
                   help="Run detection every N frames (use >1 if you need more FPS).")
    return p.parse_args()


def to_source(src_str: str):
    # If it's digits, treat as webcam index
    if src_str.isdigit():
        return int(src_str)
    return src_str


def draw_box(img, xyxy, label, thickness=2):
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), thickness)
    cv2.putText(img, label, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)


def main():
    args = parse_args()

    # Load model (downloads weights on first run)
    model = YOLO("yolov8n.pt")

    # Optional device override
    if args.device is not None:
        model.to(args.device)

    cap = cv2.VideoCapture(to_source(args.source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video source: {args.source}")

    # Small FPS smoother
    fps_hist = deque(maxlen=20)
    frame_idx = 0

    # Track last known target (useful when --every > 1)
    last_target = None  # (cx, cy) in pixels
    last_target_conf = 0.0

    print("Running. Press 'q' to quit.")
    print("Tip: if it’s slow, try: --imgsz 480 --every 2 (or 320).")

    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Frame grab failed (stream ended?)")
            break

        if args.flip:
            frame = cv2.flip(frame, 1)

        h, w = frame.shape[:2]

        run_det = (frame_idx % max(1, args.every) == 0)
        best = None  # (conf, xyxy, (cx, cy))

        if run_det:
            # Run YOLO
            results = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                max_det=args.max_det,
                verbose=False
            )

            r = results[0]
            if r.boxes is not None and len(r.boxes) > 0:
                # Find best "person" box by confidence
                for b in r.boxes:
                    cls = int(b.cls.item())
                    if cls != PERSON_CLASS_ID:
                        continue
                    conf = float(b.conf.item())
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    cx = 0.5 * (x1 + x2)
                    cy = 0.5 * (y1 + y2)
                    if best is None or conf > best[0]:
                        best = (conf, (x1, y1, x2, y2), (cx, cy))

            if best is not None:
                last_target_conf, last_xyxy, last_target = best
            else:
                last_target = None
                last_target_conf = 0.0

        # Overlay + compute normalized look commands (future servo)
        # lookX/lookY in [-1, +1], where (0,0) is screen center.
        lookX = 0.0
        lookY = 0.0
        has_target = last_target is not None

        if has_target:
            cx, cy = last_target
            lookX = (cx - (w / 2)) / (w / 2)
            lookY = (cy - (h / 2)) / (h / 2)
            lookX = max(-1.0, min(1.0, float(lookX)))
            lookY = max(-1.0, min(1.0, float(lookY)))

        # Draw overlays
        if args.show:
            # Center crosshair
            cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

            if has_target:
                draw_box(frame, last_xyxy, f"person {last_target_conf:.2f}")
                cv2.circle(frame, (int(last_target[0]), int(last_target[1])), 6, (0, 255, 255), -1)

            # FPS calc
            dt = time.time() - t0
            fps = 1.0 / max(dt, 1e-6)
            fps_hist.append(fps)
            fps_smooth = sum(fps_hist) / len(fps_hist)

            status = f"FPS {fps_smooth:.1f} | det every {args.every} | lookX {lookX:+.2f} lookY {lookY:+.2f}"
            cv2.putText(frame, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("YOLOv8n Person Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        # Print commands occasionally (so you can wire it to Arduino later)
        # You can change this to send over serial/UDP.
        if frame_idx % 10 == 0:
            if has_target:
                print(f"TARGET lookX={lookX:+.3f} lookY={lookY:+.3f} conf={last_target_conf:.2f}")
            else:
                print("NO_TARGET")

        frame_idx += 1

    cap.release()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    main()
