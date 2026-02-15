#!/usr/bin/env python3
"""
main.py

Starts:
1) Display server (phone blinking eyes)
2) People detection (CV + UDP commands)

Example:
    python main.py --source "http://100.66.11.174:4747/video" --flip
"""

import threading
import time
import sys
import argparse

import display_server_laptop
import people_detection


def run_display_server():
    display_server_laptop.app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False
    )


def main():
    print("\n=== Starting Robot System ===")

    # Parse arguments here
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0")
    parser.add_argument("--flip", action="store_true")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--robot-conf", type=float, default=0.40)
    parser.add_argument("--every", type=int, default=1)

    args = parser.parse_args()

    # Start display server first
    print("Starting display server...")
    display_thread = threading.Thread(
        target=run_display_server,
        daemon=True
    )
    display_thread.start()

    time.sleep(2)

    print("Starting people detection...")

    # Inject args into people_detection
    sys.argv = [
        "people_detection.py",
        "--source", args.source,
        "--imgsz", str(args.imgsz),
        "--conf", str(args.conf),
        "--robot-conf", str(args.robot_conf),
        "--every", str(args.every),
    ]

    if args.flip:
        sys.argv.append("--flip")

    people_detection.main()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down cleanly.")
        sys.exit(0)
