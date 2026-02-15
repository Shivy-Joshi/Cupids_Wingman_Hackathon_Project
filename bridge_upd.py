import socket
import time
import serial
import re
import glob

# ====== AUTO-DETECT PORTS (so you stop fighting /dev/serial/by-id names) ======
def pick_port_contains(substr: str):
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if substr.lower() in p.lower():
            return p
    return None

MOTOR_PORT = pick_port_contains("arduino")
SERVO_PORT = pick_port_contains("1a86")

if MOTOR_PORT is None:
    raise RuntimeError("Arduino not found in /dev/serial/by-id/. Run: ls -l /dev/serial/by-id/")
if SERVO_PORT is None:
    raise RuntimeError("1a86 servo adapter not found in /dev/serial/by-id/. Run: ls -l /dev/serial/by-id/")

MOTOR_BAUD = 9600

SERVO_BAUD = 1000000
SERVO_ID = 1

# ====== SERVO SAFE RANGE (your calibrated values) ======
SERVO_MIN = 1800
SERVO_CENTER = 2048
SERVO_MAX = 2300
# =======================================================

# Servo motion tuning
MOVE_TIME_MS = 350
SPEED = 600

# UDP listener
HOST = "0.0.0.0"
PORT = 5005  # must match sender

# Rate limiting + jitter filtering
SERVO_SEND_INTERVAL = 0.05  # seconds (20 Hz)
SERVO_DEADBAND = 4          # counts in 0..4095

def motor_write(ser, percent, cmd):
    ser.write(f"{percent}{cmd}".encode("ascii"))

def checksum(vals):
    return (~(sum(vals) & 0xFF)) & 0xFF

def servo_write_pos(ser, angle_0_4095):
    angle = max(SERVO_MIN, min(SERVO_MAX, int(angle_0_4095)))

    ID = SERVO_ID
    INST = 0x03
    ADDR = 0x2A
    LEN = 0x09

    pos_l, pos_h = angle & 0xFF, (angle >> 8) & 0xFF
    t_l, t_h = MOVE_TIME_MS & 0xFF, (MOVE_TIME_MS >> 8) & 0xFF
    s_l, s_h = SPEED & 0xFF, (SPEED >> 8) & 0xFF

    body = [ID, LEN, INST, ADDR, pos_l, pos_h, t_l, t_h, s_l, s_h]
    chk = checksum(body)
    pkt = bytes([0xFF, 0xFF] + body + [chk])
    ser.write(pkt)
    ser.flush()

def pct_to_servo_angle(pct):
    pct = max(0, min(100, int(pct)))
    return int(SERVO_MIN + (pct / 100.0) * (SERVO_MAX - SERVO_MIN))

def parse_msg(msg: str):
    # Expected: "23R 55D"
    msg = msg.strip()
    parts = msg.split()
    if len(parts) < 2:
        return None

    m_tok, s_tok = parts[0], parts[1]

    mm = re.match(r"^(\d{1,3})([FBLRS])$", m_tok)
    if not mm:
        return None
    motor_pct = max(0, min(100, int(mm.group(1))))
    motor_cmd = mm.group(2)

    sm = re.match(r"^(\d{1,3})([A-Za-z])$", s_tok)
    if not sm:
        return None
    servo_pct = max(0, min(100, int(sm.group(1))))

    return motor_pct, motor_cmd, servo_pct

def open_serial(path, baud, timeout, retries=5):
    last_err = None
    for i in range(retries):
        try:
            return serial.Serial(path, baud, timeout=timeout)
        except Exception as e:
            last_err = e
            time.sleep(0.2)
    raise last_err

def main():
    print("Opening serial ports...")
    print(f"  Motor port: {MOTOR_PORT}")
    print(f"  Servo port: {SERVO_PORT}")

    motor_ser = open_serial(MOTOR_PORT, MOTOR_BAUD, timeout=0.01)
    time.sleep(2)  # UNO resets on open
    servo_ser = open_serial(SERVO_PORT, SERVO_BAUD, timeout=0.01)
    time.sleep(0.2)

    print(f"Motor @ {MOTOR_BAUD}")
    print(f"Servo @ {SERVO_BAUD}  ID={SERVO_ID}")
    print(f"Servo safe range: {SERVO_MIN}..{SERVO_MAX} (center {SERVO_CENTER})")
    print(f"UDP listening on {HOST}:{PORT}")

    # center once
    servo_write_pos(servo_ser, SERVO_CENTER)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))

    last_servo_send = 0.0
    last_servo_angle = None

    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode("utf-8", errors="ignore").strip()

        parsed = parse_msg(msg)
        if not parsed:
            print(f"{addr}: RX '{msg}' (unparsed)")
            continue

        motor_pct, motor_cmd, servo_pct = parsed
        servo_angle = pct_to_servo_angle(servo_pct)

        motor_write(motor_ser, motor_pct, motor_cmd)

        now = time.time()
        if last_servo_angle is None or abs(servo_angle - last_servo_angle) >= SERVO_DEADBAND:
            if (now - last_servo_send) >= SERVO_SEND_INTERVAL:
                servo_write_pos(servo_ser, servo_angle)
                last_servo_send = now
                last_servo_angle = servo_angle

        print(f"{addr}: RX '{msg}' -> motors:{motor_pct}{motor_cmd}  servo:{servo_pct}% -> {servo_angle}")

if __name__ == "__main__":
    main()