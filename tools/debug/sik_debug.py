#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SiK Telemetry Radio V3 手动调试脚本
- 打开指定串口（默认 COM28）
- 实时显示收到的数据（ASCII + HEX）
- 交互输入：可发送 AT 命令 / 任意字符串
- 发送 "+++" 进入 AT 模式（需要静默期，脚本已处理）
- 简单检测 MAVLink v1/v2 HEARTBEAT 帧（只做粗检测，便于排错）

用法：
  python sik_debug.py --port COM28 --baud 57600
"""

import argparse
import sys
import time
import threading
from collections import deque

import serial

MAV_V1_STX = 0xFE
MAV_V2_STX = 0xFD
MSG_ID_HEARTBEAT = 0  # MAVLink message id

def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def safe_ascii(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in data)

def try_detect_mavlink(buffer: deque):
    """
    粗略检测 MAVLink v1/v2 帧结构（不做 CRC 校验，只判断格式和 msgid）
    发现 HEARTBEAT 就打印提示，帮助你判断有没有“像样的 MAVLink 流”
    """
    # 把 deque 转成 bytes 方便处理（不太大就行）
    data = bytes(buffer)
    i = 0
    found = []
    while i < len(data):
        stx = data[i]
        if stx == MAV_V1_STX:
            # v1: FE LEN SEQ SYS COMP MSGID ... CRC(2)
            if i + 6 <= len(data):
                plen = data[i + 1]
                frame_len = 6 + plen + 2  # stx..msgid(6 bytes total header) + payload + crc
                if i + frame_len <= len(data):
                    msgid = data[i + 5]
                    if msgid == MSG_ID_HEARTBEAT:
                        found.append(("v1", i))
                    i += frame_len
                    continue
        elif stx == MAV_V2_STX:
            # v2: FD LEN INC FLAGS SEQ SYS COMP MSGID(3) ... CRC(2) [SIG(13) optional]
            if i + 10 <= len(data):
                plen = data[i + 1]
                incompat = data[i + 2]
                # msgid 3 bytes little endian
                msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
                has_sig = (incompat & 0x01) != 0
                frame_len = 10 + plen + 2 + (13 if has_sig else 0)
                if i + frame_len <= len(data):
                    if msgid == MSG_ID_HEARTBEAT:
                        found.append(("v2", i))
                    i += frame_len
                    continue
        i += 1

    # 如果找到了心跳，输出提示（但不刷屏：最多提示一次/秒）
    return found

class RateLimiter:
    def __init__(self, interval_s=1.0):
        self.interval_s = interval_s
        self.last = 0.0
    def ok(self):
        now = time.time()
        if now - self.last >= self.interval_s:
            self.last = now
            return True
        return False

def reader_thread(ser: serial.Serial, stop_evt: threading.Event, show_hex: bool, show_ascii: bool):
    buf = deque(maxlen=4096)
    rl = RateLimiter(1.0)

    while not stop_evt.is_set():
        try:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                buf.extend(chunk)

                out_parts = []
                ts = time.strftime("%H:%M:%S")
                if show_ascii:
                    out_parts.append(safe_ascii(chunk))
                if show_hex:
                    out_parts.append(hexdump(chunk))

                print(f"[{ts}] RX ({len(chunk)} bytes): " + (" | ".join(out_parts) if out_parts else str(chunk)))

                # 粗检测 MAVLink 心跳
                found = try_detect_mavlink(buf)
                if found and rl.ok():
                    kinds = ",".join(sorted(set(k for k, _ in found)))
                    print(f"    ✅ 似乎检测到 MAVLink HEARTBEAT（{kinds}），说明波特率/数据流可能正确。")
            else:
                time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[ERR] 串口读取错误: {e}")
            break

def enter_at_mode(ser: serial.Serial):
    """
    SiK AT 模式通常要求在发送 '+++' 前后保持 1 秒左右静默。
    """
    print("准备进入 AT 模式：静默 1.2s → 发送 '+++' → 静默 1.2s")
    time.sleep(1.2)
    ser.write(b"+++")
    ser.flush()
    time.sleep(1.2)
    print("已发送 '+++'. 如果进入成功，通常会返回 'OK'。")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM28", help="串口，例如 COM28")
    ap.add_argument("--baud", type=int, default=57600, help="波特率，常见 57600/115200")
    ap.add_argument("--timeout", type=float, default=0.1, help="读超时")
    ap.add_argument("--no-hex", action="store_true", help="不显示十六进制")
    ap.add_argument("--no-ascii", action="store_true", help="不显示ASCII")
    args = ap.parse_args()

    show_hex = not args.no_hex
    show_ascii = not args.no_ascii

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
            write_timeout=1.0,
        )
    except Exception as e:
        print(f"无法打开串口 {args.port} @ {args.baud}: {e}")
        sys.exit(1)

    print(f"已打开 {ser.port} @ {ser.baudrate}")
    print("命令：")
    print("  +++        进入 AT 模式（脚本会自动加静默期）")
    print("  at <cmd>   发送 AT 命令，例如: at ATI  /  at AT&V  /  at ATS0? ")
    print("  send <s>   发送任意字符串（原样加 \\r\\n）")
    print("  baud <n>   仅显示提示：你需要重启脚本用 --baud 重新打开")
    print("  exit       退出\n")

    stop_evt = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_evt, show_hex, show_ascii), daemon=True)
    t.start()

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line.lower() in ("exit", "quit", "q"):
                break

            if line == "+++":
                enter_at_mode(ser)
                continue

            if line.lower().startswith("at "):
                cmd = line[3:].strip()
                payload = (cmd + "\r\n").encode("ascii", errors="ignore")
                ser.write(payload)
                ser.flush()
                print(f"TX: {cmd}")
                continue

            if line.lower().startswith("send "):
                s = line[5:]
                payload = (s + "\r\n").encode("utf-8", errors="ignore")
                ser.write(payload)
                ser.flush()
                print(f"TX(raw): {s}")
                continue

            if line.lower().startswith("baud "):
                print("提示：更换波特率需要重新打开串口。请退出脚本后用 --baud <n> 重新运行。")
                continue

            # 默认：原样发送并加 CRLF
            payload = (line + "\r\n").encode("utf-8", errors="ignore")
            ser.write(payload)
            ser.flush()
            print(f"TX: {line}")

    finally:
        stop_evt.set()
        try:
            time.sleep(0.2)
            ser.close()
        except Exception:
            pass
        print("已退出。")

if __name__ == "__main__":
    main()
