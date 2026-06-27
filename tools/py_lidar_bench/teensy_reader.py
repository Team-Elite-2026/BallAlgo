import serial
import time


class TeensyReader:
    """Reads heading-only ASCII telemetry from the Teensy."""

    def __init__(self, port: str, baud: int):
        self._serial = None
        self._rx_text = ""
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
        """Drain serial buffer, parse ``T,heading=...`` lines, return latest heading."""
        if self._serial is None:
            return None
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._rx_text += self._serial.read(waiting).decode("ascii", errors="ignore")
        except Exception:
            return None

        heading = None
        while "\n" in self._rx_text:
            line, self._rx_text = self._rx_text.split("\n", 1)
            line = line.strip()
            if not line.startswith("T,"):
                continue
            for part in line[2:].split(","):
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                if key.strip() != "heading":
                    continue
                try:
                    heading = float(value)
                    self.latest_heading_deg = heading
                except ValueError:
                    pass
        return heading


def main() -> int:
    try:
        from . import config as cfg
    except ImportError:
        import config as cfg

    reader = TeensyReader(cfg.TEENSY_PORT, cfg.TEENSY_BAUD)
    if not reader.is_connected:
        return 1

    print("Reading Teensy heading. Ctrl+C to quit.")
    try:
        while True:
            heading = reader.poll()
            if heading is not None:
                print(f"heading={heading:.2f}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
