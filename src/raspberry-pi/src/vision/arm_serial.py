import serial
import threading
import time


class ArmSerial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.serial = serial.Serial(
            port,
            baudrate,
            timeout=0.05,
            write_timeout=0.5
        )

        # Flask can briefly have more than one video stream during a refresh.
        # Keep their serial requests from consuming each other's replies.
        self.lock = threading.Lock()

        # ESP32 may reset when serial connection opens
        time.sleep(2)
        self.serial.reset_input_buffer()


    def send_command(self, command):
        with self.lock:
            self.serial.write((command + "\n").encode())


    def get_state(self):
        with self.lock:
            # Discard boot/debug text so only the new STATE reply is parsed.
            self.serial.reset_input_buffer()
            self.serial.write(b"GET_STATE\n")

            deadline = time.monotonic() + 0.5

            while time.monotonic() < deadline:
                line = self.serial.readline().decode(errors="replace").strip()

                if not line.startswith("STATE "):
                    continue

                parts = line.split()

                if len(parts) != 7:
                    continue

                try:
                    return {
                        "base": float(parts[1]),
                        "x": float(parts[2]),
                        "y": float(parts[3]),
                        "z": float(parts[4]),
                        "pitch": float(parts[5]),
                        "roll": float(parts[6]),
                    }
                except ValueError:
                    continue

        print("Timed out waiting for STATE response")
        return None

    def move_pose(self, x, y, z, pitch, roll):
        command = (
            f"move_pose "
            f"{x:.2f} {y:.2f} {z:.2f} "
            f"{pitch:.2f} {roll:.2f}"
        )

        self.send_command(command)
