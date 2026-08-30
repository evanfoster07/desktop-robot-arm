import serial
import time


class ArmSerial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.serial = serial.Serial(
            port,
            baudrate,
            timeout=1
        )

        # ESP32 may reset when serial connection opens
        time.sleep(2)


    def send_command(self, command):
        self.serial.write((command + "\n").encode())


    def get_state(self):
        self.send_command("GET_STATE")

        while True:
            line = self.serial.readline().decode().strip()

            if not line:
                continue

            if not line.startswith("STATE "):
                continue

            parts = line.split()

            return {
                "base": float(parts[1]),
                "x": float(parts[2]),
                "y": float(parts[3]),
                "z": float(parts[4]),
                "pitch": float(parts[5]),
                "roll": float(parts[6]),
            }

    def move_pose(self, x, y, z, pitch, roll):
        command = (
            f"move_pose "
            f"{x:.2f} {y:.2f} {z:.2f} "
            f"{pitch:.2f} {roll:.2f}"
        )

        self.send_command(command)