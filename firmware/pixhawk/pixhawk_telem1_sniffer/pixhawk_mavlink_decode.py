import time
from pymavlink import mavutil

PORT = "COM13"
BAUD = 115200


def main():
    mav = mavutil.mavlink_connection(PORT, baud=BAUD)
    print(f"Listening on {PORT} @ {BAUD}...")
    while True:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        print(f"[{time.strftime('%H:%M:%S')}] {msg.get_type()} {msg.to_dict()}")


if __name__ == "__main__":
    main()

