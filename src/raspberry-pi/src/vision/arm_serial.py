import math
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
        self.lock = threading.Lock()
        time.sleep(2)
        self.serial.reset_input_buffer()

    def send_command(self, command):
        with self.lock:
            self.serial.write((command + "\n").encode())

    def run_command(self, command, response_window_s=0.20):
        """Send a raw ESP32 command exactly as typed and collect any reply lines."""
        command = command.strip()
        if not command:
            return []

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write((command + "\n").encode())

            lines = []
            deadline = time.monotonic() + response_window_s

            while time.monotonic() < deadline:
                line = self.serial.readline().decode(errors="replace").strip()
                if line:
                    lines.append(line)
                    deadline = time.monotonic() + 0.05

            return lines

    def get_state(self):
        with self.lock:
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

    def move_pose(self, x, y, z, pitch, roll, return_reason=False):
        """
        Send one Cartesian pose to the ESP32.

        By default this keeps the old boolean API. With return_reason=True it
        returns (success, reason), allowing the motion sequence to distinguish
        an explicit POSE_FAIL from a serial timeout.
        """
        command = (
            f"move_pose "
            f"{x:.2f} {y:.2f} {z:.2f} "
            f"{pitch:.2f} {roll:.2f}"
        )

        success = False
        reason = "response_timeout"

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write((command + "\n").encode())
            deadline = time.monotonic() + 0.75

            while time.monotonic() < deadline:
                line = self.serial.readline().decode(errors="replace").strip()

                if line == "POSE_OK":
                    success = True
                    reason = "accepted"
                    break

                if line == "POSE_FAIL":
                    reason = "rejected"
                    print(
                        "ESP32 rejected pose: "
                        f"({x:.1f}, {y:.1f}, {z:.1f}, {pitch:.1f})"
                    )
                    break

        if not success and reason == "response_timeout":
            print("Timed out waiting for POSE_OK / POSE_FAIL")

        if return_reason:
            return success, reason

        return success

    def move_pose_and_wait(
        self,
        x,
        y,
        z,
        pitch,
        roll,
        timeout_s=5.0,
        position_tolerance_mm=3.0,
        pitch_tolerance_deg=2.0,
        stable_samples=3,
        return_reason=False
    ):
        """
        Send a Cartesian pose and wait for the tracked pose to settle.

        return_reason=True returns:
            (True, "complete")
            (False, "rejected")
            (False, "response_timeout")
            (False, "settle_timeout")

        This lets visual tracking safely retry only when the ESP32 explicitly
        rejected a pose. A settle timeout is NOT immediately retried because
        the arm may already have moved partway toward the target.
        """
        accepted, reason = self.move_pose(
            x,
            y,
            z,
            pitch,
            roll,
            return_reason=True
        )

        if not accepted:
            if return_reason:
                return False, reason
            return False

        deadline = time.monotonic() + timeout_s
        stable_count = 0

        while time.monotonic() < deadline:
            state = self.get_state()
            if state is None:
                stable_count = 0
                time.sleep(0.05)
                continue

            position_error = math.sqrt(
                (state["x"] - x) ** 2
                + (state["y"] - y) ** 2
                + (state["z"] - z) ** 2
            )
            pitch_error = abs(state["pitch"] - pitch)

            if (
                position_error <= position_tolerance_mm
                and pitch_error <= pitch_tolerance_deg
            ):
                stable_count += 1
                if stable_count >= stable_samples:
                    if return_reason:
                        return True, "complete"
                    return True
            else:
                stable_count = 0

            time.sleep(0.05)

        print(
            "Timed out waiting for arm motion to settle at "
            f"({x:.1f}, {y:.1f}, {z:.1f}, {pitch:.1f})"
        )

        if return_reason:
            return False, "settle_timeout"

        return False

    def adjust_wrist_pitch(self, delta_angle):
        """Adjust wrist pitch relatively using the ESP32 `p_rel <delta>` command."""
        self.send_command(f"p_rel {int(delta_angle)}")

    def adjust_wrist_pitch_and_wait(self, delta_angle, settle_s=0.5):
        self.adjust_wrist_pitch(delta_angle)
        time.sleep(settle_s)
        return True

    def set_gripper(self, angle):
        self.send_command(f"g {int(angle)}")

    def set_gripper_and_wait(self, angle, settle_s=0.8):
        self.set_gripper(angle)
        time.sleep(settle_s)
        return True

    def close(self):
        with self.lock:
            self.serial.close()
