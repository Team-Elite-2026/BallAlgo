import time

import serial

import config as cfg


def open_robot_serial():
    candidate_ports = [cfg.SERIAL_PORT]
    for fallback in ("/dev/ttyAMA0", "/dev/ttyS0"):
        if fallback not in candidate_ports:
            candidate_ports.append(fallback)

    for port in candidate_ports:
        try:
            ser = serial.Serial(
                port=port,
                baudrate=cfg.SERIAL_BAUD,
                timeout=cfg.SERIAL_TIMEOUT,
            )
            time.sleep(0.1)
            print(f"[SERIAL] Connected on {port}")
            return ser
        except Exception as exc:
            print(f"[SERIAL] Failed to open {port}: {exc}")

    return None


def safe_write(serial_conn, message):
    if serial_conn is None:
        return False
    try:
        serial_conn.write(message.encode("utf-8"))
        return True
    except serial.SerialException as exc:
        print(f"[SERIAL] Write failed: {exc}")
        return False


class RobotHeadingReader:
    """Parses Teensy compass heading messages (e.g. '-45h') on the Pi UART."""

    def __init__(self, default_heading_deg=0.0):
        self._buffer = ""
        self._heading_deg = default_heading_deg

    @property
    def heading_deg(self):
        return self._heading_deg

    def poll(self, serial_conn):
        if serial_conn is None:
            return self._heading_deg

        try:
            waiting = serial_conn.in_waiting
            if waiting <= 0:
                return self._heading_deg
            chunk = serial_conn.read(waiting).decode("utf-8", errors="ignore")
        except serial.SerialException:
            return self._heading_deg

        for ch in chunk:
            if ch == "h":
                if self._buffer:
                    try:
                        self._heading_deg = float(self._buffer)
                    except ValueError:
                        pass
                self._buffer = ""
            elif ch in "0123456789.-":
                self._buffer += ch
            else:
                self._buffer = ""

        return self._heading_deg
