#!/usr/bin/env python3
"""
people_detection.py
YOLOv8n person detection from a video source (default: laptop webcam).

Now: uses your laptop webcam (source=0) or a phone stream URL (DroidCam, etc).
Later: point --source to a Pi stream URL (MJPEG/RTSP/etc).

Install:
  pip install ultralytics opencv-python

Run examples:
  # Laptop webcam
  python people_detection.py

  # Phone stream (example)
  python people_detection.py --source "http://100.66.11.174:4747/video" --flip

Output (printed every 10 frames):
  - Turn command: 0..100 + L/R  (e.g., 90L)
  - Tilt command: 0..100 + U/D  (e.g., 25U)
Only prints commands if best person conf >= 0.40, otherwise prints NO_TARGET.
"""

import argparse
import time
from collections import deque

import cv2
from ultralytics import YOLO

import socket


PERSON_CLASS_ID = 0  # COCO: 0 = person


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--source",
        default="0",
        help='Video source: webcam index ("0") OR file path OR stream URL.',
    )
    p.add_argument("--conf", type=float, default=0.35, help="Model confidence threshold (YOLO).")
    p.add_argument("--imgsz", type=int, default=640, help="Inference image size (e.g., 320/480/640).")
    p.add_argument("--max-det", type=int, default=10, help="Max detections per frame.")
    p.add_argument("--show", action="store_true", help="Show a window with overlays.")
    p.add_argument("--no-show", dest="show", action="store_false", help="Disable window display.")
    p.set_defaults(show=True)
    p.add_argument("--flip", action="store_true", help="Flip image horizontally (mirror view).")
    p.add_argument(
        "--device",
        default=None,
        help='Device override (e.g., "cpu", "0" for GPU). Default: auto.',
    )
    p.add_argument("--every", type=int, default=1, help="Run detection every N frames (use >1 if slow).")

    # Your robot logic threshold:
    p.add_argument("--robot-conf", type=float, default=0.40, help="Ignore detections below this confidence.")
    p.add_argument("--print-every", type=int, default=10, help="Print commands every N frames.")
    return p.parse_args()


def to_source(src_str: str):
    # If it's digits, treat as webcam index
    if src_str.isdigit():
        return int(src_str)
    return src_str


def draw_box(img, xyxy, label, thickness=2):
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), thickness)
    cv2.putText(
        img,
        label,
        (x1, max(0, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


def lr_ud_commands(cx: float, cy: float, w: int, h: int):
    """
    Compute command strings:
      turn: 0..100 + L/R based on cx vs image center
      tilt: 0..100 + U/D based on cy vs image center

    Percent is normalized relative to half-width/half-height:
      - at center => 0
      - near left/right edge => ~100
      - near top/bottom edge => ~100
    """
    dx = cx - (w / 2.0)   # + right, - left
    dy = cy - (h / 2.0)   # + down,  - up (image coords)

    x_pct = int(round(clamp01(abs(dx) / (w / 2.0)) * 100.0))
    y_pct = int(round(clamp01(abs(dy) / (h / 2.0)) * 100.0))

    turn = f"{x_pct}{'R' if dx >= 0 else 'L'}"
    tilt = f"{y_pct}{'D' if dy >= 0 else 'U'}"
    return turn, tilt


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

    fps_hist = deque(maxlen=20)
    frame_idx = 0

    # Track last known best target
    last_target = None       # (cx, cy)
    last_target_conf = 0.0
    last_xyxy = None         # (x1, y1, x2, y2)

    print("Running. Press 'q' to quit.")
    print("Tip: if it’s slow, try: --imgsz 480 --every 2 (or 320).")

    PI_IP = "100.99.169.90"  # <-- your Pi's IP  ##########################
    PI_PORT = 5005

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
            results = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                max_det=args.max_det,
                verbose=False,
            )

            r = results[0]
            if r.boxes is not None and len(r.boxes) > 0:
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
                last_xyxy = None

        has_target = last_target is not None

        # Compute lookX/lookY in [-1, +1] (still useful for overlays/debug)
        lookX = 0.0
        lookY = 0.0
        if has_target:
            cx, cy = last_target
            lookX = (cx - (w / 2)) / (w / 2)
            lookY = (cy - (h / 2)) / (h / 2)
            lookX = max(-1.0, min(1.0, float(lookX)))
            lookY = max(-1.0, min(1.0, float(lookY)))

        # Display overlays
        if args.show:
            cv2.drawMarker(
                frame,
                (w // 2, h // 2),
                (255, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2,
            )

            if has_target and last_xyxy is not None:
                draw_box(frame, last_xyxy, f"person {last_target_conf:.2f}")
                cv2.circle(frame, (int(last_target[0]), int(last_target[1])), 6, (0, 255, 255), -1)

            dt = time.time() - t0
            fps = 1.0 / max(dt, 1e-6)
            fps_hist.append(fps)
            fps_smooth = sum(fps_hist) / len(fps_hist)

            status = f"FPS {fps_smooth:.1f} | det every {args.every} | lookX {lookX:+.2f} lookY {lookY:+.2f}"
            cv2.putText(
                frame,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("YOLOv8n Person Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        # Print robot commands
        if frame_idx % max(1, args.print_every) == 0:
            if has_target and last_target_conf >= args.robot_conf:
                cx, cy = last_target
                turn_cmd, tilt_cmd = lr_ud_commands(cx, cy, w, h)
                # Format requested: e.g., "90L" and "25U"
                print(f"{turn_cmd} {tilt_cmd}")
                msg = f"{turn_cmd} {tilt_cmd}\n"
                sock.sendto(msg.encode("utf-8"), (PI_IP, PI_PORT))
                print(msg.strip())

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
