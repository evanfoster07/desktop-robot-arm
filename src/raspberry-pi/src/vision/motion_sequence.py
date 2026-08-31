from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum, auto
import threading

from tracking import (
    camera_correction_to_robot,
    camera_forward_to_robot,
    clamp,
    get_box_error,
    get_box_metrics,
    get_tracking_correction,
)


class SequenceState(Enum):
    IDLE = auto()
    OPEN_GRIPPER = auto()

    CENTER_CREEPER = auto()
    APPROACH_CREEPER = auto()
    FINAL_GRAB = auto()
    GRAB = auto()

    RETURN_TO_VIEW = auto()

    CENTER_GOAL = auto()
    APPROACH_GOAL = auto()
    DROP = auto()

    RETREAT = auto()
    DONE = auto()
    ERROR = auto()


@dataclass
class SequenceConfig:
    # YOLO classes
    creeper_class_id: int = 0
    goal_class_id: int = 1
    min_target_confidence: float = 0.20

    # Desired image locations.
    target_x_norm: float = 0.50
    creeper_target_y_norm: float = 0.85
    goal_target_y_norm: float = 0.75

    # Visual-servo tuning
    tracking_gain_mm: float = 10.0
    tracking_deadband: float = 0.05
    centered_frames_required: int = 3

    # Maximum Cartesian image-space correction per accepted move
    max_lateral_step_mm: float = 5.0
    max_vertical_step_mm: float = 5.0

    # Normally hold the current tool pitch. If an otherwise useful Cartesian
    # correction is rejected by IK, search a few nearby pitch values to give
    # shoulder/elbow/wrist another valid posture to work with
    pitch_relaxation_steps_deg: tuple[float, ...] = (2.0, -2.0, 4.0, -4.0)

    # Closed-loop approach
    approach_step_mm: float = 5.0
    retry_approach_step_mm: float = 2.5
    retry_correction_scale: float = 0.5
    approach_sign: float = 1.0

    # A visual tracking pose rejection is recoverable. Only give up after
    # several consecutive failed tracking attempts.
    max_consecutive_pose_failures: int = 3

    # Close-range thresholds
    # For the Creeper, this is now the PRE-GRASP trigger
    creeper_grab_width_norm: float | None = None
    goal_drop_width_norm: float = 0.55

    # Terminal/open-loop grab calibration.
    # First lurch forward while holding current Cartesian tool pitch,
    # then directly command ONLY the wrist-pitch servo to this angle.
    final_grab_forward_mm: float | None = None
    final_grab_wrist_pitch_delta_deg: int | None = None
    final_grab_wrist_settle_s: float = 0.5

    # Slightly looser than normal tracking for final grab
    final_grab_max_error_x_norm: float = 0.10
    final_grab_max_error_y_norm: float = 0.15

    # Safe `g <angle>` values for gripper
    gripper_open_angle: int = 40
    gripper_closed_angle: int = 60
    gripper_settle_s: float = 0.8

    # Motion-completion polling tolerances
    move_timeout_s: float = 5.0
    position_tolerance_mm: float = 3.0
    pitch_tolerance_deg: float = 2.0


@dataclass
class MotionOutcome:
    success: bool
    recoverable: bool = False
    reason: str = ""


class MotionSequence:
    """High-level autonomous pick-and-place state machine."""

    def __init__(self, arm, config=None):
        self.arm = arm
        self.config = config or SequenceConfig()

        self.state = SequenceState.IDLE
        self.error_message = ""
        self.view_pose = None

        # One worker means arm commands can never overlap.
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.motion_future = None
        self.next_state_after_motion = None

        self.centered_frames = 0
        self.consecutive_pose_failures = 0
        self.last_motion_failure = ""

        self.last_target_box = None
        self.last_target_metrics = None
        self.last_target_point = None
        self.last_error_norm = None

        # Flask can briefly create multiple video streams on refresh
        self.lock = threading.Lock()

    @property
    def active(self):
        return self.state not in {
            SequenceState.IDLE,
            SequenceState.DONE,
            SequenceState.ERROR,
        }

    def start(self):
        """
        Start a new sequence and save the current pose as the baseline/view
        pose. Put the arm in the desired viewing pose before pressing Start.
        """
        with self.lock:
            if self.config.gripper_open_angle is None:
                self._set_error(
                    "Set gripper_open_angle in SequenceConfig before starting."
                )
                return False

            if self.config.gripper_closed_angle is None:
                self._set_error(
                    "Set gripper_closed_angle in SequenceConfig before starting."
                )
                return False

            if self.config.creeper_grab_width_norm is None:
                self._set_error(
                    "Set creeper_grab_width_norm in SequenceConfig before starting."
                )
                return False

            if self.config.final_grab_forward_mm is None:
                self._set_error(
                    "Set final_grab_forward_mm in SequenceConfig before starting."
                )
                return False

            if self.config.final_grab_wrist_pitch_delta_deg is None:
                self._set_error(
                    "Set final_grab_wrist_pitch_delta_deg in SequenceConfig before starting."
                )
                return False

            if self.motion_future is not None and not self.motion_future.done():
                return False

            state = self.arm.get_state()

            if state is None:
                self._set_error("Could not read baseline arm pose.")
                return False

            self.view_pose = {
                "x": state["x"],
                "y": state["y"],
                "z": state["z"],
                "pitch": state["pitch"],
                "roll": state["roll"],
            }

            self.error_message = ""
            self.centered_frames = 0
            self.consecutive_pose_failures = 0
            self.last_motion_failure = ""
            self.last_target_box = None
            self.last_target_metrics = None
            self.last_target_point = None
            self.last_error_norm = None

            self.state = SequenceState.OPEN_GRIPPER
            print("Sequence started. Saved baseline view pose:", self.view_pose)
            return True

    def stop(self):
        """
        Stop issuing future autonomous commands.

        This does not interrupt a servo move already accepted by the ESP32.
        """
        with self.lock:
            self.state = SequenceState.IDLE

            # A command already accepted by the ESP32 cannot be cancelled
            # here, but when its background worker finishes we deliberately
            # remain IDLE instead of advancing the old sequence
            if self.motion_future is not None:
                self.next_state_after_motion = SequenceState.IDLE
            else:
                self.next_state_after_motion = None

            self.centered_frames = 0
            print("Sequence stopped")

    def update(self, detections, frame_width, frame_height):
        """Advance the state machine by at most one decision per frame."""
        with self.lock:
            self._finish_background_motion_if_ready()

            # Never make a new visual decision from an intermediate motion
            # frame. Wait for the accepted pose to settle first
            if self.motion_future is not None:
                return

            if self.state in {
                SequenceState.IDLE,
                SequenceState.ERROR,
                SequenceState.DONE,
            }:
                return

            if self.state == SequenceState.OPEN_GRIPPER:
                self._schedule_gripper(
                    self.config.gripper_open_angle,
                    SequenceState.CENTER_CREEPER
                )
                return

            if self.state == SequenceState.FINAL_GRAB:
                self._schedule_motion(
                    self._final_grab_worker,
                    SequenceState.GRAB
                )
                return

            if self.state == SequenceState.GRAB:
                self._schedule_gripper(
                    self.config.gripper_closed_angle,
                    SequenceState.RETURN_TO_VIEW
                )
                return

            if self.state == SequenceState.RETURN_TO_VIEW:
                self._schedule_view_pose(SequenceState.CENTER_GOAL)
                return

            if self.state == SequenceState.DROP:
                self._schedule_gripper(
                    self.config.gripper_open_angle,
                    SequenceState.RETREAT
                )
                return

            if self.state == SequenceState.RETREAT:
                self._schedule_view_pose(SequenceState.DONE)
                return

            if self.state == SequenceState.CENTER_CREEPER:
                self._update_centering(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.creeper_class_id,
                    self.config.creeper_target_y_norm,
                    SequenceState.APPROACH_CREEPER,
                )
                return

            if self.state == SequenceState.APPROACH_CREEPER:
                self._update_approach(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.creeper_class_id,
                    self.config.creeper_target_y_norm,
                    self.config.creeper_grab_width_norm,
                    SequenceState.GRAB,
                )
                return

            if self.state == SequenceState.CENTER_GOAL:
                self._update_centering(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.goal_class_id,
                    self.config.goal_target_y_norm,
                    SequenceState.APPROACH_GOAL,
                )
                return

            if self.state == SequenceState.APPROACH_GOAL:
                self._update_approach(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.goal_class_id,
                    self.config.goal_target_y_norm,
                    self.config.goal_drop_width_norm,
                    SequenceState.DROP,
                )
                return

    def get_debug_info(self):
        return {
            "state": self.state.name,
            "error": self.error_message,
            "target_box": self.last_target_box,
            "target_metrics": self.last_target_metrics,
            "target_point": self.last_target_point,
            "error_norm": self.last_error_norm,
            "moving": self.motion_future is not None,
            "pose_failures": self.consecutive_pose_failures,
            "last_motion_failure": self.last_motion_failure,
        }

    def _select_target(self, detections, class_id):
        candidates = [
            detection
            for detection in detections
            if (
                detection["class_id"] == class_id
                and detection["confidence"] >= self.config.min_target_confidence
            )
        ]

        if not candidates:
            return None

        return max(candidates, key=lambda detection: detection["confidence"])

    def _get_target_tracking(
        self,
        detection,
        frame_width,
        frame_height,
        target_y_norm
    ):
        target_x = self.config.target_x_norm * frame_width
        target_y = target_y_norm * frame_height

        tracking = get_box_error(
            detection["box"],
            frame_width,
            frame_height,
            target_x=target_x,
            target_y=target_y,
        )

        metrics = get_box_metrics(
            detection["box"],
            frame_width,
            frame_height,
        )

        self.last_target_box = detection["box"]
        self.last_target_metrics = metrics
        self.last_target_point = tracking["target"]
        self.last_error_norm = tracking["error_norm"]

        return tracking, metrics

    def _update_centering(
        self,
        detections,
        frame_width,
        frame_height,
        class_id,
        target_y_norm,
        next_state
    ):
        target = self._select_target(detections, class_id)

        if target is None:
            self.centered_frames = 0
            self.last_target_box = None
            self.last_target_metrics = None
            self.last_error_norm = None
            return

        tracking, _ = self._get_target_tracking(
            target,
            frame_width,
            frame_height,
            target_y_norm,
        )

        error_x, error_y = tracking["error_norm"]

        correction_right, correction_vertical, centered_x, centered_y = (
            get_tracking_correction(
                error_x,
                error_y,
                gain=self.config.tracking_gain_mm,
                deadband=self.config.tracking_deadband,
            )
        )

        if centered_x and centered_y:
            self.centered_frames += 1

            if self.centered_frames >= self.config.centered_frames_required:
                self.centered_frames = 0
                self.state = next_state
                print("Sequence state ->", self.state.name)
            return

        self.centered_frames = 0

        self._schedule_tracking_move(
            error_x,
            error_y,
            correction_right,
            correction_vertical,
            0.0,
            self.state,
        )

    def _update_approach(
        self,
        detections,
        frame_width,
        frame_height,
        class_id,
        target_y_norm,
        size_threshold,
        next_state
    ):
        target = self._select_target(detections, class_id)

        if target is None:
            # Never continue forward if vision loses the target
            self.centered_frames = 0
            self.last_target_box = None
            self.last_target_metrics = None
            self.last_error_norm = None
            return

        tracking, metrics = self._get_target_tracking(
            target,
            frame_width,
            frame_height,
            target_y_norm,
        )

        error_x, error_y = tracking["error_norm"]

        correction_right, correction_vertical, centered_x, centered_y = (
            get_tracking_correction(
                error_x,
                error_y,
                gain=self.config.tracking_gain_mm,
                deadband=self.config.tracking_deadband,
            )
        )

        centered = centered_x and centered_y

        # Creeper: switch to the terminal open-loop grab before the target
        # gets so close that visual servoing becomes unreliable
        if class_id == self.config.creeper_class_id:
            final_grab_aligned = (
                abs(error_x) <= self.config.final_grab_max_error_x_norm
                and abs(error_y) <= self.config.final_grab_max_error_y_norm
            )

            if final_grab_aligned and metrics["width_norm"] >= size_threshold:
                self.state = SequenceState.FINAL_GRAB
                print(
                    "Sequence state -> FINAL_GRAB "
                    f"(target width={metrics['width_norm']:.3f}, "
                    f"error=({error_x:.3f}, {error_y:.3f}))"
                )
                return

        # Goal: keep the existing closed-loop aligned + size trigger
        elif centered and metrics["width_norm"] >= size_threshold:
            self.state = next_state
            print(
                f"Sequence state -> {self.state.name} "
                f"(target width={metrics['width_norm']:.3f})"
            )
            return

        # If target drifts out of alignment, correct without advancing
        # Forward steps happen only while centered
        approach_mm = (
            self.config.approach_sign * self.config.approach_step_mm
            if centered
            else 0.0
        )

        self._schedule_tracking_move(
            error_x,
            error_y,
            correction_right,
            correction_vertical,
            approach_mm,
            self.state,
        )

    def _schedule_tracking_move(
        self,
        error_x,
        error_y,
        correction_right,
        correction_vertical,
        approach_mm,
        next_state
    ):
        self._schedule_motion(
            self._tracking_move_worker,
            next_state,
            error_x,
            error_y,
            correction_right,
            correction_vertical,
            approach_mm,
        )

    def _tracking_move_worker(
        self,
        error_x,
        error_y,
        correction_right,
        correction_vertical,
        approach_mm
    ):
        state = self.arm.get_state()

        if state is None:
            print("Tracking move skipped: failed to get arm state")
            return MotionOutcome(
                success=False,
                recoverable=True,
                reason="failed to read arm state"
            )

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

        # First try the desired right/up/forward camera translation while
        # holding the arm's current absolute tool pitch. Vertical image error
        # is no longer turned directly into a pitch command
        success, reason = self._attempt_tracking_pose(
            state=state,
            correction_right=correction_right,
            correction_vertical=correction_vertical,
            approach_mm=approach_mm,
            pitch_offset_deg=0.0,
            label="normal",
        )

        if success:
            return MotionOutcome(success=True)

        if reason != "rejected":
            return MotionOutcome(
                success=False,
                recoverable=True,
                reason=reason
            )

        # The exact XYZ + pitch combination can be over-constrained near a
        # joint limit. Keep the same desired camera translation but relax
        # absolute pitch a few degrees in either direction. If one is valid,
        # the next camera frame will visually re-center from that new posture
        success, reason = self._try_pitch_relaxations(
            state=state,
            correction_right=correction_right,
            correction_vertical=correction_vertical,
            approach_mm=approach_mm,
            prefix="pitch-relax",
        )

        if success:
            return MotionOutcome(success=True)

        if reason != "rejected":
            return MotionOutcome(
                success=False,
                recoverable=True,
                reason=reason
            )

        # If every full-size target was just outside the valid workspace, try
        # the same visual intent with a smaller translation/approach step
        retry_state = self.arm.get_state()

        if retry_state is None:
            return MotionOutcome(
                success=False,
                recoverable=True,
                reason="pose rejected, then failed to reread arm state"
            )

        retry_approach_mm = 0.0

        if approach_mm != 0.0:
            retry_magnitude = min(
                abs(approach_mm),
                self.config.retry_approach_step_mm
            )
            retry_approach_mm = (
                retry_magnitude if approach_mm > 0.0 else -retry_magnitude
            )

        retry_correction_right = (
            correction_right * self.config.retry_correction_scale
        )
        retry_correction_vertical = (
            correction_vertical * self.config.retry_correction_scale
        )

        print(
            "Full tracking target rejected. Retrying smaller XYZ move: "
            f"correction_scale={self.config.retry_correction_scale:.2f}, "
            f"forward={retry_approach_mm:.2f} mm"
        )

        success, retry_reason = self._attempt_tracking_pose(
            state=retry_state,
            correction_right=retry_correction_right,
            correction_vertical=retry_correction_vertical,
            approach_mm=retry_approach_mm,
            pitch_offset_deg=0.0,
            label="small",
        )

        if success:
            return MotionOutcome(success=True)

        if retry_reason != "rejected":
            return MotionOutcome(
                success=False,
                recoverable=True,
                reason=retry_reason
            )

        # Last recovery attempt: small XYZ translation plus nearby pitch search
        success, final_reason = self._try_pitch_relaxations(
            state=retry_state,
            correction_right=retry_correction_right,
            correction_vertical=retry_correction_vertical,
            approach_mm=retry_approach_mm,
            prefix="small-pitch-relax",
        )

        if success:
            return MotionOutcome(success=True)

        return MotionOutcome(
            success=False,
            recoverable=True,
            reason=(
                f"normal={reason}, small={retry_reason}, "
                f"final={final_reason}"
            )
        )

    def _try_pitch_relaxations(
        self,
        state,
        correction_right,
        correction_vertical,
        approach_mm,
        prefix
    ):
        """Try nearby absolute tool pitches without changing visual intent."""
        last_reason = "rejected"

        for pitch_offset_deg in self.config.pitch_relaxation_steps_deg:
            print(
                "Pose rejected. Trying nearby tool pitch: "
                f"{pitch_offset_deg:+.1f} deg"
            )

            success, reason = self._attempt_tracking_pose(
                state=state,
                correction_right=correction_right,
                correction_vertical=correction_vertical,
                approach_mm=approach_mm,
                pitch_offset_deg=pitch_offset_deg,
                label=f"{prefix} {pitch_offset_deg:+.1f}deg",
            )

            if success:
                return True, "complete"

            last_reason = reason

            if reason != "rejected":
                return False, reason

        return False, last_reason

    def _attempt_tracking_pose(
        self,
        state,
        correction_right,
        correction_vertical,
        approach_mm,
        pitch_offset_deg,
        label
    ):
        """Build and attempt one camera-right/up/forward Cartesian move."""
        target_pitch = state["pitch"] + pitch_offset_deg

        # Both screen axes are now converted directly into Cartesian camera
        # translation. This lets IK redistribute the motion across shoulder,
        # elbow and wrist instead of forcing vertical error into wrist pitch
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
            f"{self.state.name} [{label}]: "
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

    def _final_grab_worker(self):
        """
        Terminal grab routine:
        1. Lurch forward along the current camera/tool-forward axis while
           holding the current overall Cartesian tool pitch
        2. After the lurch settles, command ONLY the wrist-pitch servo to the
           calibrated absolute servo angle
        3. Advance to GRAB, which closes the gripper
        """
        state = self.arm.get_state()

        if state is None:
            print("FINAL_GRAB failed: could not read arm state")
            return False

        forward_mm = self.config.approach_sign * self.config.final_grab_forward_mm

        forward_dx, forward_dy, forward_dz = camera_forward_to_robot(
            forward_mm,
            state["base"],
            state["pitch"],
        )

        target_x = state["x"] + forward_dx
        target_y = state["y"] + forward_dy
        target_z = state["z"] + forward_dz

        print(
            "FINAL_GRAB lurch: "
            f"forward={forward_mm:.2f} mm, "
            f"holding tool pitch={state['pitch']:.2f} deg, "
            f"xyz_delta=({forward_dx:.2f}, {forward_dy:.2f}, {forward_dz:.2f}) mm"
        )

        success, reason = self.arm.move_pose_and_wait(
            target_x,
            target_y,
            target_z,
            state["pitch"],
            state["roll"],
            timeout_s=self.config.move_timeout_s,
            position_tolerance_mm=self.config.position_tolerance_mm,
            pitch_tolerance_deg=self.config.pitch_tolerance_deg,
            return_reason=True,
        )

        if not success:
            print(f"FINAL_GRAB lurch failed: {reason}")
            return False

        print(
            "FINAL_GRAB wrist pitch delta -> "
            f"{self.config.final_grab_wrist_pitch_delta_deg:+d} deg"
        )

        return self.arm.adjust_wrist_pitch_and_wait(
            self.config.final_grab_wrist_pitch_delta_deg,
            self.config.final_grab_wrist_settle_s,
        )

    def _schedule_gripper(self, angle, next_state):
        self._schedule_motion(
            self.arm.set_gripper_and_wait,
            next_state,
            angle,
            self.config.gripper_settle_s,
        )

    def _schedule_view_pose(self, next_state):
        if self.view_pose is None:
            self._set_error("No baseline view pose was saved.")
            return

        self._schedule_motion(
            self.arm.move_pose_and_wait,
            next_state,
            self.view_pose["x"],
            self.view_pose["y"],
            self.view_pose["z"],
            self.view_pose["pitch"],
            self.view_pose["roll"],
            self.config.move_timeout_s,
            self.config.position_tolerance_mm,
            self.config.pitch_tolerance_deg,
        )

    def _schedule_motion(self, function, next_state, *args):
        if self.motion_future is not None:
            return

        self.next_state_after_motion = next_state
        self.motion_future = self.executor.submit(function, *args)

    def _finish_background_motion_if_ready(self):
        if self.motion_future is None or not self.motion_future.done():
            return

        try:
            result = self.motion_future.result()
        except Exception as exc:
            self.motion_future = None
            self.next_state_after_motion = None
            self._set_error(f"Motion worker exception: {exc}")
            return

        next_state = self.next_state_after_motion
        self.motion_future = None
        self.next_state_after_motion = None

        # Tracking workers return MotionOutcome so a rejected visual correction
        # can be treated as recoverable instead of killing the sequence
        if isinstance(result, MotionOutcome):
            if not result.success:
                self.consecutive_pose_failures += 1
                self.last_motion_failure = result.reason

                print(
                    "Recoverable tracking failure "
                    f"{self.consecutive_pose_failures}/"
                    f"{self.config.max_consecutive_pose_failures}: "
                    f"{result.reason}"
                )

                if (
                    self.consecutive_pose_failures
                    >= self.config.max_consecutive_pose_failures
                ):
                    self._set_error(
                        "Tracking pose failed repeatedly: "
                        f"{result.reason}"
                    )

                # Otherwise stay in the current CENTER/APPROACH state and let
                # the next fresh frame recompute the visual correction
                return

            self.consecutive_pose_failures = 0
            self.last_motion_failure = ""

        elif not result:
            # Non-visual motions (gripper, return-to-view, retreat) are still
            # considered fatal if they fail
            self._set_error("Arm motion failed or timed out.")
            return
        else:
            self.consecutive_pose_failures = 0
            self.last_motion_failure = ""

        self.state = next_state
        self.centered_frames = 0
        print("Sequence state ->", self.state.name)

    def _set_error(self, message):
        self.state = SequenceState.ERROR
        self.error_message = message
        print("Sequence ERROR:", message)
