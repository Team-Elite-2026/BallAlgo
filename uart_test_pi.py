#!/usr/bin/env python3
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with: python3 -m pip install pyserial")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Simple UART test utility for Raspberry Pi.")
    parser.add_argument("--port", default="/dev/serial0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate")
    parser.add_argument("--message", default="sending on pi uart", help="Message to send")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between sends")
    parser.add_argument("--timeout", type=float, default=0.05, help="Read timeout in seconds")
    parser.add_argument(
        "--read-only",
        action="store_true",
        help="Only read incoming bytes, do not transmit",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=args.timeout,
            write_timeout=args.timeout,
        )
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}")
        return 1

    print(f"Opened {args.port} @ {args.baud} baud")
    print("Press Ctrl+C to stop")

    next_send = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if not args.read_only and now >= next_send:
                payload = f"{args.message}\n".encode("utf-8")
                ser.write(payload)
                print(f"TX: {args.message}")
                next_send = now + args.interval

            if ser.in_waiting:
                data = ser.readline()
                if data:
                    print(f"RX: {data.decode('utf-8', errors='replace').rstrip()}")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())