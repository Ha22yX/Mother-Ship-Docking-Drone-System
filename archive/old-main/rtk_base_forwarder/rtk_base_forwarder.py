#!/usr/bin/env python3
import argparse
import socket
import subprocess
import sys
import time

import serial
from serial.tools import list_ports

WIFI_SSID = "UAVDocking"
WIFI_PASS = "UAVDocking"
WIFI_INTERFACE = "en0"
AUTO_CONNECT_WIFI = True

DEFAULT_BAUD = 115200
RTCM_MESSAGES = [1005, 1077, 1087, 1097, 1127, 1230]
MAX_TYPE_TRACK = 64

TARGETS = [
    ("192.168.1.2", 14550),
    ("192.168.1.3", 14550),
]
USE_BROADCAST = True
BROADCAST_TARGET = ("192.168.1.255", 14550)

# Base position lock (u-blox F9P)
ENABLE_SURVEY_IN = True
SVIN_MIN_DURATION_S = 120
SVIN_ACC_LIMIT_MM = 5000


def connect_wifi():
    if not AUTO_CONNECT_WIFI:
        return
    try:
        subprocess.run(
            [
                "networksetup",
                "-setairportnetwork",
                WIFI_INTERFACE,
                WIFI_SSID,
                WIFI_PASS,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        pass


def looks_like_rtcm(data: bytes) -> bool:
    # RTCM3 preamble is 0xD3
    if b"\xD3" in data:
        return True
    return False


def parse_rtcm_stream(data: bytes, buf: bytearray, stats: dict):
    buf.extend(data)
    stats["rtcm_preamble"] += data.count(0xD3)
    while True:
        if len(buf) < 3:
            return
        # Find preamble
        if buf[0] != 0xD3:
            idx = buf.find(0xD3)
            if idx == -1:
                buf.clear()
                return
            del buf[:idx]
            if len(buf) < 3:
                return
        # RTCM3 length is 10 bits from bytes 1-2 (lower 6 bits of byte1, full byte2)
        length = ((buf[1] & 0x03) << 8) | buf[2]
        frame_len = 3 + length + 3  # header + payload + CRC
        if len(buf) < frame_len:
            return
        payload = buf[3:3 + length]
        if length >= 2:
            msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF
            stats["types"][msg_type] = stats["types"].get(msg_type, 0) + 1
            stats["total_msgs"] += 1
        del buf[:frame_len]


def ubx_checksum(payload: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def ubx_message(cls_id: int, msg_id: int, payload: bytes) -> bytes:
    length = len(payload)
    header = bytes([0xB5, 0x62, cls_id, msg_id, length & 0xFF, (length >> 8) & 0xFF])
    ck = ubx_checksum(bytes([cls_id, msg_id, length & 0xFF, (length >> 8) & 0xFF]) + payload)
    return header + payload + ck


def cfg_msg_rtcm(msg_num: int, rate_usb: int) -> bytes:
    msg_class = 0xF5
    # u-blox CFG-MSG uses RTCM message number modulo 1000 for msg_id
    # e.g. 1005 -> 0x05, 1077 -> 0x4D, 1230 -> 0xE6
    msg_id = msg_num % 1000
    # rates: I2C, UART1, UART2, USB, SPI, reserved
    payload = bytes([msg_class, msg_id, 0, 0, 0, rate_usb, 0, 0])
    return ubx_message(0x06, 0x01, payload)


def cfg_save() -> bytes:
    clear_mask = 0x00000000
    save_mask = 0x0000FFFF
    load_mask = 0x00000000
    device_mask = 0x17  # BBR + Flash + EEPROM
    payload = (
        clear_mask.to_bytes(4, "little") +
        save_mask.to_bytes(4, "little") +
        load_mask.to_bytes(4, "little") +
        bytes([device_mask])
    )
    return ubx_message(0x06, 0x09, payload)


def cfg_tmode3_survey_in(min_dur_s: int, acc_limit_mm: int) -> bytes:
    version = 0
    reserved1 = 0
    flags = 0x00000001  # survey-in enable
    ecef_x = 0
    ecef_y = 0
    ecef_z = 0
    ecef_hp_x = 0
    ecef_hp_y = 0
    ecef_hp_z = 0
    reserved2 = 0
    fixed_pos_acc = 0
    svin_min_dur = int(min_dur_s)
    svin_acc_limit = int(acc_limit_mm)
    payload = (
        bytes([version, reserved1]) +
        flags.to_bytes(4, "little") +
        int(ecef_x).to_bytes(4, "little", signed=True) +
        int(ecef_y).to_bytes(4, "little", signed=True) +
        int(ecef_z).to_bytes(4, "little", signed=True) +
        int(ecef_hp_x).to_bytes(1, "little", signed=True) +
        int(ecef_hp_y).to_bytes(1, "little", signed=True) +
        int(ecef_hp_z).to_bytes(1, "little", signed=True) +
        bytes([reserved2]) +
        int(fixed_pos_acc).to_bytes(4, "little") +
        int(svin_min_dur).to_bytes(4, "little") +
        int(svin_acc_limit).to_bytes(4, "little")
    )
    return ubx_message(0x06, 0x71, payload)


def try_start_survey_in(ser: serial.Serial):
    if not ENABLE_SURVEY_IN:
        return
    msg = cfg_tmode3_survey_in(SVIN_MIN_DURATION_S, SVIN_ACC_LIMIT_MM)
    ser.write(msg)
    ser.flush()


def try_enable_rtcm(ser: serial.Serial, rate_usb: int):
    for msg_num in RTCM_MESSAGES:
        ser.write(cfg_msg_rtcm(msg_num, rate_usb))
    ser.flush()


def try_save_cfg(ser: serial.Serial):
    ser.write(cfg_save())
    ser.flush()


def port_score(p):
    desc = (p.description or "").lower()
    manuf = (p.manufacturer or "").lower()
    if "u-blox" in desc or "u-blox" in manuf or "ublox" in desc or "ublox" in manuf:
        return 3
    if "f9p" in desc or "f9p" in manuf:
        return 3
    if "gps" in desc or "gps" in manuf:
        return 2
    return 0


def find_base_port(forced_port: str | None = None):
    ports = list(list_ports.comports())
    if not ports:
        return None, None

    ports.sort(key=port_score, reverse=True)

    if forced_port:
        try:
            ser = serial.Serial(forced_port, baudrate=DEFAULT_BAUD, timeout=0.2)
        except Exception:
            return None, None
        return ser, DEFAULT_BAUD

    for p in ports:
        try:
            ser = serial.Serial(p.device, baudrate=DEFAULT_BAUD, timeout=0.2)
        except Exception:
            continue
        return ser, DEFAULT_BAUD
    return None, None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Force serial port (e.g., COM19 or /dev/tty.usbmodem)")
    parser.add_argument("--configure-rtcm", action="store_true", help="Enable RTCM messages on USB")
    parser.add_argument("--rtcm-rate", type=int, default=1, help="RTCM output rate on USB (default 1)")
    parser.add_argument("--no-save", action="store_true", help="Do not save configuration to flash")
    args = parser.parse_args()

    connect_wifi()
    ser, baud = find_base_port(args.port)
    if ser is None:
        print("RTK base not found.")
        sys.exit(1)

    print(f"RTK base: {ser.port} @ {baud}")
    try_start_survey_in(ser)
    if args.configure_rtcm:
        try_enable_rtcm(ser, args.rtcm_rate)
        if not args.no_save:
            try_save_cfg(ser)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    total = 0
    last_total = 0
    last_log = time.time()
    rtcm_buf = bytearray()
    stats = {"types": {}, "total_msgs": 0, "rtcm_preamble": 0, "nmea": 0, "ubx": 0}
    while True:
        data = ser.read(1024)
        if not data:
            continue
        total += len(data)
        # Quick protocol hints
        if b"$" in data:
            stats["nmea"] += data.count(b"$")
        if b"\xB5\x62" in data:
            stats["ubx"] += data.count(b"\xB5\x62")
        parse_rtcm_stream(data, rtcm_buf, stats)
        for host, port in TARGETS:
            sock.sendto(data, (host, port))
        if USE_BROADCAST:
            sock.sendto(data, BROADCAST_TARGET)
        now = time.time()
        if now - last_log > 2.0:
            delta = total - last_total
            last_total = total
            # Show top message types and protocol hints
            top = sorted(stats["types"].items(), key=lambda x: x[1], reverse=True)[:MAX_TYPE_TRACK]
            top_str = " ".join([f"{k}:{v}" for k, v in top])
            print(f"RTCM bytes sent: {total} (+{delta} bytes)")
            print(
                f"RTCM msgs: {stats['total_msgs']}  types: {top_str}  "
                f"rtcm_preamble: {stats['rtcm_preamble']}  nmea: {stats['nmea']}  ubx: {stats['ubx']}"
            )
            last_log = now


if __name__ == "__main__":
    main()

