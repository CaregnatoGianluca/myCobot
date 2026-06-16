#!/usr/bin/env python3
"""
Minimal, robust TCP bridge for the myCobot 280 Pi.

Runs ON the Raspberry Pi (on the robot). Wraps the WORKING direct-serial
pymycobot path and exposes a dead-simple newline-delimited ASCII protocol over
TCP, so a remote ROS 2 driver node (mycobot_driver_node, on the WSL2 dev
machine) can drive the arm.

This deliberately replaces Elephant Robotics' stock demo/Server.py, whose
text-handshake socket protocol is buggy (returns []/-1, throws 0xfe UTF-8
decode errors). Here we control both ends, so the protocol is trivial.

Protocol (one command per line, ASCII):
    GET_ANGLES\\n                       -> "a1,a2,a3,a4,a5,a6\\n"   (degrees) | "ERR\\n"
    SET_ANGLES a1 a2 a3 a4 a5 a6 spd\\n -> "OK\\n"                   (degrees, spd 0-100)
    GET_COORDS\\n                       -> "x,y,z,rx,ry,rz\\n"        (mm/deg) | "ERR\\n"
    SET_GRIPPER value speed\\n          -> "OK\\n"   (value 0=closed .. 100=open)
    GET_GRIPPER\\n                      -> "value\\n" (0-100) | "ERR\\n"
    RELEASE\\n                          -> "OK\\n"   (relax all servos — free drive)
    FOCUS\\n                            -> "OK\\n"   (re-energize servos — hold pose)
    PING\\n                             -> "PONG\\n"

Prerequisites on the Pi:
    - pymycobot installed and working via direct serial (verified with v3.0.0:
      MyCobot('/dev/ttyAMA0', 1000000).get_angles() returns real data)
    - /dev/ttyAMA0 must be FREE. Elephant's Bluetooth helper
      (mycobot_pi_bluetooth/uart_peripheral_serial.py) holds it open — kill it
      first:  sudo pkill -f uart_peripheral_serial

Usage on the Pi:
    python3 mycobot_tcp_bridge.py
    # optional: --port 9000 --serial /dev/ttyAMA0 --baud 1000000
"""
import argparse
import socket
import sys
import threading
import time

from pymycobot import MyCobot

ARM_JOINTS = 6


class CobotBridge:
    def __init__(self, serial_port, baud):
        print(f"[bridge] opening {serial_port} @ {baud} ...", flush=True)
        self.mc = MyCobot(serial_port, baud)
        # Serialize access — pymycobot / the serial link is not re-entrant.
        self.lock = threading.Lock()

        # The Atom needs a moment after the serial port opens before get_angles()
        # returns valid data; the first read(s) often come back []. Warm up + retry.
        time.sleep(2.0)
        angles = None
        for attempt in range(8):
            angles = self.mc.get_angles()
            if angles and angles != -1:
                break
            print(f"[bridge] warmup get_angles() -> {angles!r}, retrying...", flush=True)
            time.sleep(0.5)
        if not angles or angles == -1:
            raise RuntimeError(
                f"get_angles() returned {angles!r} after retries. Is the arm powered "
                f"and is {serial_port} free? (kill uart_peripheral_serial first)")
        print(f"[bridge] serial OK, current angles: {angles}", flush=True)

        # Only now — after the link is confirmed — enable fresh mode ("always
        # execute the latest command", good for streamed trajectory points).
        # Done last so it can't disrupt the initial handshake; best-effort.
        try:
            self.mc.set_fresh_mode(1)
            time.sleep(0.1)
        except Exception as e:
            print(f"[bridge] set_fresh_mode(1) not available: {e}", flush=True)

    def handle_line(self, line):
        """Process one ASCII command line, return the ASCII response (no newline)."""
        parts = line.split()
        if not parts:
            return "ERR"
        cmd = parts[0].upper()

        if cmd == "PING":
            return "PONG"

        if cmd == "GET_ANGLES":
            with self.lock:
                angles = self.mc.get_angles()
            if not angles or angles == -1:
                return "ERR"
            return ",".join(f"{a:.3f}" for a in angles)

        if cmd == "SET_ANGLES":
            # SET_ANGLES a1 a2 a3 a4 a5 a6 speed
            if len(parts) != ARM_JOINTS + 2:
                return "ERR"
            try:
                angles = [float(x) for x in parts[1:1 + ARM_JOINTS]]
                speed = int(float(parts[1 + ARM_JOINTS]))
            except ValueError:
                return "ERR"
            speed = max(1, min(100, speed))
            with self.lock:
                self.mc.send_angles(angles, speed)
            return "OK"

        if cmd == "GET_COORDS":
            with self.lock:
                coords = self.mc.get_coords()
            if not coords or coords == -1:
                return "ERR"
            return ",".join(f"{c:.3f}" for c in coords)

        if cmd == "SET_GRIPPER":
            # SET_GRIPPER value speed   (value 0=closed .. 100=open)
            if len(parts) != 3:
                return "ERR"
            try:
                value = int(float(parts[1]))
                speed = int(float(parts[2]))
            except ValueError:
                return "ERR"
            value = max(0, min(100, value))
            speed = max(1, min(100, speed))
            with self.lock:
                self.mc.set_gripper_value(value, speed)
            return "OK"

        if cmd == "GET_GRIPPER":
            with self.lock:
                v = self.mc.get_gripper_value()
            if v is None or v == -1:
                return "ERR"
            return str(v)

        if cmd == "RELEASE":
            # Relax all servos so the arm can be moved by hand (drag teaching).
            with self.lock:
                self.mc.release_all_servos()
            return "OK"

        if cmd == "FOCUS":
            # Re-energize servos; the arm holds its current position.
            with self.lock:
                self.mc.power_on()
            return "OK"

        return "ERR"


def serve(bridge, host, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[bridge] listening on {host}:{port} — waiting for connection", flush=True)

    while True:
        conn, addr = srv.accept()
        print(f"[bridge] client connected: {addr}", flush=True)
        try:
            with conn:
                conn_file = conn.makefile("r", encoding="ascii", newline="\n")
                while True:
                    line = conn_file.readline()
                    if line == "":          # client closed
                        break
                    try:
                        resp = bridge.handle_line(line.strip())
                    except Exception as e:  # never die on one bad command
                        print(f"[bridge] error handling {line!r}: {e}", flush=True)
                        resp = "ERR"
                    conn.sendall((resp + "\n").encode("ascii"))
        except Exception as e:
            print(f"[bridge] connection error: {e}", flush=True)
        finally:
            print("[bridge] client disconnected — waiting for connection", flush=True)


def main():
    ap = argparse.ArgumentParser(description="myCobot 280 Pi TCP bridge")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--serial", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()

    try:
        bridge = CobotBridge(args.serial, args.baud)
    except Exception as e:
        print(f"[bridge] FATAL: {e}", file=sys.stderr, flush=True)
        sys.exit(1)

    serve(bridge, args.host, args.port)


if __name__ == "__main__":
    main()
