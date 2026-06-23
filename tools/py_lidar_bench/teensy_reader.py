import struct

import serial

_PROTOCOL_MAGIC = 0xCEFAEDFE
_MSG_TELEMETRY = 0x04
_TELEMETRY_SIZE = 24  # sizeof(TeensyTelemetryPayload)


def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if (crc & 1) else crc >> 1
    return ~crc & 0xFFFFFFFF


class TeensyReader:
    """Reads heading (and other telemetry) from the Teensy over the binary frame protocol.

    Frame layout (little-endian):
        magic    4 bytes  0xCEFAEDFE
        type     1 byte
        plen     2 bytes  payload length
        payload  N bytes
        crc32    4 bytes  CRC32 of the preceding 7+N bytes
    """

    def __init__(self, port: str, baud: int):
        self._serial = None
        self._buf = bytearray()
        self.latest_heading_deg: float | None = None
        try:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.001)
            print(f"[TEENSY] Connected on {port} @ {baud}")
        except Exception as exc:
            print(f"[TEENSY] Failed to open serial: {exc}")

    @property
    def is_connected(self) -> bool:
        return self._serial is not None

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
            print("[TEENSY] Closed")

    def poll(self) -> float | None:
        """Drain serial buffer, parse frames, return latest headingDeg or None."""
        if self._serial is None:
            return None
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._buf.extend(self._serial.read(waiting))
        except Exception:
            return None

        heading = None
        i = 0
        while i + 4 <= len(self._buf):
            magic = struct.unpack_from("<I", self._buf, i)[0]
            if magic != _PROTOCOL_MAGIC:
                i += 1
                continue
            if i + 7 > len(self._buf):
                break
            msg_type = self._buf[i + 4]
            plen = struct.unpack_from("<H", self._buf, i + 5)[0]
            frame_len = 7 + plen + 4
            if i + frame_len > len(self._buf):
                break
            body = bytes(self._buf[i : i + 7 + plen])
            crc = struct.unpack_from("<I", self._buf, i + 7 + plen)[0]
            if _crc32(body) != crc:
                i += 1
                continue
            if msg_type == _MSG_TELEMETRY and plen >= _TELEMETRY_SIZE:
                heading_deg = struct.unpack_from("<f", self._buf, i + 7)[0]
                self.latest_heading_deg = heading_deg
                heading = heading_deg
            i += frame_len

        del self._buf[:i]
        return heading
