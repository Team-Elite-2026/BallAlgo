import time

import serial

import config as cfg


def open_robot_serial():
    try:
        ser = serial.Serial(
            port=cfg.SERIAL_PORT,
            baudrate=cfg.SERIAL_BAUD,
            timeout=cfg.SERIAL_TIMEOUT,
        )
        time.sleep(0.1)
        print(f"[SERIAL] Connected on {cfg.SERIAL_PORT}")
        return ser
    except Exception as exc:
        print(f"[SERIAL] Failed to open serial: {exc}")
        return None


def safe_write(serial_conn, message):
    if serial_conn is None:
        return
    try:
        serial_conn.write(message.encode("utf-8"))
    except serial.SerialException:
        pass


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
