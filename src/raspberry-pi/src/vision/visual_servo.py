from dataclasses import dataclass

from tracking import (
    camera_correction_to_robot,
    camera_forward_to_robot,
    clamp,
)


@dataclass
class MotionOutcome:
    success: bool
    recoverable: bool = False
    reason: str = ""


class VisualServo:
    def __init__(self, arm, config):
        self.arm = arm
        self.config = config

    def move(self, correction_right, correction_vertical, approach_mm, state_name):
        state = self.arm.get_state()

        if state is None:
            print("Tracking move skipped: failed to get arm state")
            return self._failure("failed to read arm state")

        correction_right = clamp(
            correction_right,
            -self.config.max_lateral_step_mm,
            self.config.max_lateral_step_mm,
        )
        correction_vertical = clamp(
            correction_vertical,
            -self.config.max_vertical_step_mm,
            self.config.max_vertical_step_mm,
        )

        success, reason = self._attempt_pose(
            state,
            correction_right,
            correction_vertical,
            approach_mm,
            0.0,
            "normal",
            state_name,
        )

        if success:
            return MotionOutcome(success=True)

        if reason != "rejected":
            return self._failure(reason)

        # Try nearby pitches if IK rejects the first pose
        success, reason = self._try_pitch_offsets(
            state,
            correction_right,
            correction_vertical,
            approach_mm,
            "pitch-relax",
            state_name,
        )

        if success:
            return MotionOutcome(success=True)

        if reason != "rejected":
            return self._failure(reason)

        retry_state = self.arm.get_state()
        if retry_state is None:
            return self._failure(
                "pose rejected, then failed to reread arm state"
            )

        retry_approach_mm = self._smaller_approach(approach_mm)
        retry_right = correction_right * self.config.retry_correction_scale
        retry_vertical = correction_vertical * self.config.retry_correction_scale

        print(
            "Full tracking target rejected. Retrying smaller XYZ move: "
            f"correction_scale={self.config.retry_correction_scale:.2f}, "
            f"forward={retry_approach_mm:.2f} mm"
        )

        success, retry_reason = self._attempt_pose(
            retry_state,
            retry_right,
            retry_vertical,
            retry_approach_mm,
            0.0,
            "small",
            state_name,
        )

        if success:
            return MotionOutcome(success=True)

        if retry_reason != "rejected":
            return self._failure(retry_reason)

        # Last shot with a smaller move and nearby pitches
        success, final_reason = self._try_pitch_offsets(
            retry_state,
            retry_right,
            retry_vertical,
            retry_approach_mm,
            "small-pitch-relax",
            state_name,
        )

        if success:
            return MotionOutcome(success=True)

        return self._failure(
            f"normal={reason}, small={retry_reason}, final={final_reason}"
        )

    def _try_pitch_offsets(
        self,
        state,
        correction_right,
        correction_vertical,
        approach_mm,
        prefix,
        state_name,
    ):
        last_reason = "rejected"

        for pitch_offset_deg in self.config.pitch_relaxation_steps_deg:
            print(
                "Pose rejected. Trying nearby tool pitch: "
                f"{pitch_offset_deg:+.1f} deg"
            )

            success, reason = self._attempt_pose(
                state,
                correction_right,
                correction_vertical,
                approach_mm,
                pitch_offset_deg,
                f"{prefix} {pitch_offset_deg:+.1f}deg",
                state_name,
            )

            if success:
                return True, "complete"

            last_reason = reason
            if reason != "rejected":
                return False, reason

        return False, last_reason

    def _attempt_pose(
        self,
        state,
        correction_right,
        correction_vertical,
        approach_mm,
        pitch_offset_deg,
        label,
        state_name,
    ):
        target_pitch = state["pitch"] + pitch_offset_deg

        corr_dx, corr_dy, corr_dz = camera_correction_to_robot(
            correction_right,
            correction_vertical,
            state["base"],
            state["pitch"],
        )
        forward_dx, forward_dy, forward_dz = camera_forward_to_robot(
            approach_mm,
            state["base"],
            state["pitch"],
        )

        target_x = state["x"] + corr_dx + forward_dx
        target_y = state["y"] + corr_dy + forward_dy
        target_z = state["z"] + corr_dz + forward_dz

        print(
            f"{state_name} [{label}]: "
            f"camera_right={correction_right:.2f} mm, "
            f"camera_vertical={correction_vertical:.2f} mm, "
            f"forward={approach_mm:.2f} mm, "
            f"pitch_offset={pitch_offset_deg:+.2f} deg, "
            f"xyz_delta=({corr_dx + forward_dx:.2f}, "
            f"{corr_dy + forward_dy:.2f}, "
            f"{corr_dz + forward_dz:.2f}) mm"
        )

        return self.arm.move_pose_and_wait(
            target_x,
            target_y,
            target_z,
            target_pitch,
            state["roll"],
            timeout_s=self.config.move_timeout_s,
            position_tolerance_mm=self.config.position_tolerance_mm,
            pitch_tolerance_deg=self.config.pitch_tolerance_deg,
            return_reason=True,
        )

    def _smaller_approach(self, approach_mm):
        if approach_mm == 0.0:
            return 0.0

        magnitude = min(
            abs(approach_mm),
            self.config.retry_approach_step_mm,
        )
        return magnitude if approach_mm > 0.0 else -magnitude

    @staticmethod
    def _failure(reason):
        return MotionOutcome(
            success=False,
            recoverable=True,
            reason=reason,
        )
