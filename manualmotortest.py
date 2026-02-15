import time
import serial
import glob

def pick_arduino():
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "arduino" in p.lower():
            return p
    return None

PORT = pick_arduino()
if PORT is None:
    raise RuntimeError("Arduino not found. Run: ls -l /dev/serial/by-id/")

BAUD = 9600

def send(ser, pct, cmd, duration=0.0):
    pct = max(0, min(100, int(pct)))
    cmd = cmd.upper()

    msg = f"{pct}{cmd}"
    print(f"-> {msg}")
    ser.write(msg.encode("ascii"))
    if duration > 0:
        time.sleep(duration)

def auto_demo(ser):
    print("\n=== AUTO DEMO START ===")
    send(ser, 100, 'F', 2)
    send(ser, 0,   'S', 1)
    send(ser, 100, 'B', 2)
    send(ser, 0,   'S', 1)
    send(ser, 70,  'L', 1.5)
    send(ser, 70,  'R', 1.5)
    send(ser, 30,  'F', 2)
    send(ser, 0,   'S', 1)
    print("=== AUTO DEMO DONE ===\n")

def print_help():
    print("\nManual commands:")
    print("  <pct><cmd>   e.g. 80F, 60L, 60R, 40B, 0S")
    print("Commands: F=forward, B=back, L=left, R=right, S=stop")
    print("Other options:")
    print("  demo   -> run automatic demo sequence")
    print("  stop   -> send 0S")
    print("  q      -> quit\n")

def main():
    print(f"Opening Arduino on {PORT} @ {BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    # Arduino resets on open
    time.sleep(2)

    print_help()

    while True:
        try:
            cmd = input("motor> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not cmd:
            continue

        if cmd in ("q", "quit", "exit"):
            send(ser, 0, 'S')
            break

        if cmd in ("h", "help", "?"):
            print_help()
            continue

        if cmd == "stop":
            send(ser, 0, 'S')
            continue

        if cmd == "demo":
            auto_demo(ser)
            continue

        # Parse manual entry like "80f"
        cmd_up = cmd.upper()
        if len(cmd_up) < 2:
            print("Bad command. Example: 80F or 0S")
            continue

        letter = cmd_up[-1]
        number = cmd_up[:-1]

        if letter not in ("F", "B", "L", "R", "S"):
            print("Bad command letter. Use F B L R S")
            continue

        try:
            pct = int(number)
        except:
            print("Bad percent. Example: 80F")
            continue

        send(ser, pct, letter)

    ser.close()

if __name__ == "__main__":
    main()