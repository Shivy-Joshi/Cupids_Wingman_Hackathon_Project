import time
import serial

# ==== EDIT IF NEEDED ====
SERVO_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0181168-if00"
SERVO_BAUD = 1000000
SERVO_ID = 4
# =======================

# Defaults you can change while running
MOVE_TIME_MS = 300   # 0 = fastest
SPEED = 800          # 0..30000-ish, depends on servo

def checksum(vals):
    return (~(sum(vals) & 0xFF)) & 0xFF

def write_pos(ser, angle_0_4095, move_time_ms, speed, servo_id=4):
    """
    ST3235 position write:
      FF FF ID LEN INST ADDR posL posH timeL timeH speedL speedH chk
    Writes 6 bytes starting at address 0x2A (pos, time, speed)
    """
    angle = max(0, min(4095, int(angle_0_4095)))
    move_time_ms = max(0, min(30000, int(move_time_ms)))
    speed = max(0, min(30000, int(speed)))

    ID = int(servo_id) & 0xFF
    INST = 0x03
    ADDR = 0x2A
    LEN = 0x09

    pos_l, pos_h = angle & 0xFF, (angle >> 8) & 0xFF
    t_l, t_h = move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF
    s_l, s_h = speed & 0xFF, (speed >> 8) & 0xFF

    body = [ID, LEN, INST, ADDR, pos_l, pos_h, t_l, t_h, s_l, s_h]
    chk = checksum(body)

    pkt = bytes([0xFF, 0xFF] + body + [chk])
    ser.write(pkt)

def pct_to_4095(p):
    p = max(0, min(100, int(p)))
    return int(p * 4095 / 100)

def print_help():
    print("\nCommands:")
    print("  0-4095        -> move to raw position")
    print("  p<0-100>      -> move by percent (e.g., p50)")
    print("  t<ms>         -> set move time in ms (e.g., t300)")
    print("  s<val>        -> set speed (e.g., s800)")
    print("  id<n>         -> set servo ID (e.g., id4)")
    print("  c             -> move to center (2048)")
    print("  q             -> quit\n")

def main():
    global MOVE_TIME_MS, SPEED, SERVO_ID

    print(f"Opening servo on {SERVO_PORT} @ {SERVO_BAUD}, ID={SERVO_ID}")
    ser = serial.Serial(SERVO_PORT, SERVO_BAUD, timeout=0.05)
    time.sleep(0.2)

    print_help()
    print(f"Current settings: time={MOVE_TIME_MS}ms, speed={SPEED}, id={SERVO_ID}")

    while True:
        try:
            cmd = input("servo> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not cmd:
            continue

        if cmd in ("q", "quit", "exit"):
            break

        if cmd in ("h", "help", "?"):
            print_help()
            continue

        if cmd == "c":
            angle = 2048
            write_pos(ser, angle, MOVE_TIME_MS, SPEED, SERVO_ID)
            print(f"Moved -> {angle} (center)")
            continue

        if cmd.startswith("t"):
            try:
                MOVE_TIME_MS = int(cmd[1:])
                print(f"Set move time -> {MOVE_TIME_MS} ms")
            except:
                print("Bad time. Example: t300")
            continue

        if cmd.startswith("s"):
            try:
                SPEED = int(cmd[1:])
                print(f"Set speed -> {SPEED}")
            except:
                print("Bad speed. Example: s800")
            continue

        if cmd.startswith("id"):
            try:
                SERVO_ID = int(cmd[2:])
                print(f"Set servo ID -> {SERVO_ID}")
            except:
                print("Bad ID. Example: id4")
            continue

        if cmd.startswith("p"):
            try:
                pct = int(cmd[1:])
                angle = pct_to_4095(pct)
                write_pos(ser, angle, MOVE_TIME_MS, SPEED, SERVO_ID)
                print(f"Moved -> {pct}% ({angle})  time={MOVE_TIME_MS}ms speed={SPEED} id={SERVO_ID}")
            except:
                print("Bad percent. Example: p50")
            continue

        # raw position
        try:
            angle = int(cmd)
            if angle < 0 or angle > 4095:
                print("Out of range. Use 0-4095 or p0-p100.")
                continue
            write_pos(ser, angle, MOVE_TIME_MS, SPEED, SERVO_ID)
            print(f"Moved -> {angle}  time={MOVE_TIME_MS}ms speed={SPEED} id={SERVO_ID}")
        except:
            print("Unknown command. Type 'help'.")

    ser.close()

if __name__ == "__main__":
    main()