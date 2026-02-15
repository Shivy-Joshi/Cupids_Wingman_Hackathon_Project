import time
import serial
import glob

# ===== AUTO DETECT ARDUINO PORT =====
def pick_arduino():
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "arduino" in p.lower():
            return p
    return None

PORT = pick_arduino()
if PORT is None:
    raise RuntimeError("Arduino not found. Run: ls -l /dev/serial/by-id/")

BAUD = 9600

def send(ser, pct, cmd, duration=1.5):
    msg = f"{pct}{cmd}"
    print(f"Sending: {msg}")
    ser.write(msg.encode("ascii"))
    time.sleep(duration)

def main():
    print(f"Opening Arduino on {PORT}")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    # Arduino resets on open
    time.sleep(2)

    print("=== MOTOR TEST START ===")

    # Full Forward
    send(ser, 100, 'F', 2)

    # Stop
    send(ser, 0, 'S', 1)

    # Full Backward
    send(ser, 100, 'B', 2)

    # Stop
    send(ser, 0, 'S', 1)

    # Turn Left
    send(ser, 70, 'L', 1.5)

    # Turn Right
    send(ser, 70, 'R', 1.5)

    # Slow Forward
    send(ser, 30, 'F', 2)

    # Stop
    send(ser, 0, 'S', 1)

    print("=== MOTOR TEST DONE ===")
    ser.close()

if __name__ == "__main__":
    main()