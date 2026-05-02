import serial
import time

# Open hardware serial port
ser = serial.Serial(
    port="/dev/serial0",   # ALWAYS use serial0
    baudrate=115200,
    timeout=1              # seconds
)

time.sleep(2)  # Give serial time to initialize

print("Serial port opened")

try:
    while True:
        # ---- WRITE ----
        msg = "Hello from Pi\n"
        ser.write(msg.encode("ASCII"))
        print("Sent:", msg.strip())

        # ---- READ ----
        if ser.in_waiting > 0:
            line = ser.readline().decode("ASCII", errors="ignore").strip()
            print("Received:", line)

        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    ser.close()