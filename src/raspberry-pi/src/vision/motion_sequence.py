from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum, auto
import threading

from tracking import (
    camera_forward_to_robot,
    get_box_error,
    get_box_metrics,
    get_tracking_correction,
)
from visual_servo import MotionOutcome, VisualServo


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

    # Image targets
    target_x_norm: float = 0.50
    creeper_target_y_norm: float = 0.85
    goal_target_y_norm: float = 0.75

    # Visual servo tuning
    tracking_gain_mm: float = 10.0
    tracking_deadband: float = 0.05
    centered_frames_required: int = 3
    max_lateral_step_mm: float = 5.0
    max_vertical_step_mm: float = 5.0
    pitch_relaxation_steps_deg: tuple[float, ...] = (2.0, -2.0, 4.0, -4.0)

    # Approach tuning
    approach_step_mm: float = 5.0
    retry_approach_step_mm: float = 2.5
    retry_correction_scale: float = 0.5
    approach_sign: float = 1.0
    max_consecutive_pose_failures: int = 3

    # Close range triggers
    creeper_grab_width_norm: float | None = None
    goal_drop_width_norm: float = 0.55

    # Final grab tuning
    final_grab_forward_mm: float | None = None
    final_grab_wrist_pitch_delta_deg: int | None = None
    final_grab_wrist_settle_s: float = 0.5
    final_grab_max_error_x_norm: float = 0.10
    final_grab_max_error_y_norm: float = 0.15

    # Gripper
    gripper_open_angle: int = 40
    gripper_closed_angle: int = 60
    gripper_settle_s: float = 0.8

    # Motion settle checks
    move_timeout_s: float = 5.0
    position_tolerance_mm: float = 3.0
    pitch_tolerance_deg: float = 2.0


class MotionSequence:
    def __init__(self, arm, config=None):
        self.arm = arm
        self.config = config or SequenceConfig()
        self.visual_servo = VisualServo(self.arm, self.config)

        self.state = SequenceState.IDLE
        self.error_message = ""
        self.view_pose = None

        # One worker keeps arm commands from overlapping
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

        # Flask can briefly open more than one stream on refresh
        self.lock = threading.Lock()

    @property
    def active(self):
        return self.state not in {
            SequenceState.IDLE,
            SequenceState.DONE,
            SequenceState.ERROR,
        }

    def start(self):
        with self.lock:
            missing = self._missing_config()
            if missing:
                self._set_error(f"Set {missing} in SequenceConfig before starting")
                return False

            if self.motion_future is not None and not self.motion_future.done():
                return False

            state = self.arm.get_state()
            if state is None:
                self._set_error("Could not read baseline arm pose")
                return False

            self.view_pose = {
                key: state[key]
                for key in ("x", "y", "z", "pitch", "roll")
            }

            self.error_message = ""
            self.centered_frames = 0
            self.consecutive_pose_failures = 0
            self.last_motion_failure = ""
            self._clear_target_debug()

            self.state = SequenceState.OPEN_GRIPPER
            print("Sequence started. Saved baseline view pose:", self.view_pose)
            return True

    def stop(self):
        with self.lock:
            self.state = SequenceState.IDLE

            # Let an accepted move finish but don't advance the old sequence
            if self.motion_future is not None:
                self.next_state_after_motion = SequenceState.IDLE
            else:
                self.next_state_after_motion = None

            self.centered_frames = 0
            print("Sequence stopped")

    def update(self, detections, frame_width, frame_height):
        with self.lock:
            self._finish_background_motion_if_ready()

            # Don't track off a frame captured halfway through a move
            if self.motion_future is not None:
                return

            if self.state in {
                SequenceState.IDLE,
                SequenceState.ERROR,
                SequenceState.DONE,
            }:
                return

            if self._handle_fixed_state():
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
            elif self.state == SequenceState.APPROACH_CREEPER:
                self._update_approach(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.creeper_class_id,
                    self.config.creeper_target_y_norm,
                    self.config.creeper_grab_width_norm,
                    SequenceState.GRAB,
                )
            elif self.state == SequenceState.CENTER_GOAL:
                self._update_centering(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.goal_class_id,
                    self.config.goal_target_y_norm,
                    SequenceState.APPROACH_GOAL,
                )
            elif self.state == SequenceState.APPROACH_GOAL:
                self._update_approach(
                    detections,
                    frame_width,
                    frame_height,
                    self.config.goal_class_id,
                    self.config.goal_target_y_norm,
                    self.config.goal_drop_width_norm,
                    SequenceState.DROP,
                )

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

    def _handle_fixed_state(self):
        if self.state == SequenceState.OPEN_GRIPPER:
            self._schedule_gripper(
                self.config.gripper_open_angle,
                SequenceState.CENTER_CREEPER,
            )
        elif self.state == SequenceState.FINAL_GRAB:
            self._schedule_motion(self._final_grab_worker, SequenceState.GRAB)
        elif self.state == SequenceState.GRAB:
            self._schedule_gripper(
                self.config.gripper_closed_angle,
                SequenceState.RETURN_TO_VIEW,
            )
        elif self.state == SequenceState.RETURN_TO_VIEW:
            self._schedule_view_pose(SequenceState.CENTER_GOAL)
        elif self.state == SequenceState.DROP:
            self._schedule_gripper(
                self.config.gripper_open_angle,
                SequenceState.RETREAT,
            )
        elif self.state == SequenceState.RETREAT:
            self._schedule_view_pose(SequenceState.DONE)
        else:
            return False

        return True

    def _select_target(self, detections, class_id):
        candidates = [
            detection
            for detection in detections
            if detection["class_id"] == class_id
            and detection["confidence"] >= self.config.min_target_confidence
        ]
        return max(candidates, key=lambda d: d["confidence"], default=None)

    def _get_tracking_data(
        self,
        detections,
        frame_width,
        frame_height,
        class_id,
        target_y_norm,
    ):
        target = self._select_target(detections, class_id)
        if target is None:
            self.centered_frames = 0
            self._clear_target_debug()
            return None

        target_x = self.config.target_x_norm * frame_width
        target_y = target_y_norm * frame_height
        tracking = get_box_error(
            target["box"],
            frame_width,
            frame_height,
            target_x=target_x,
            target_y=target_y,
        )
        metrics = get_box_metrics(
            target["box"],
            frame_width,
            frame_height,
        )

        self.last_target_box = target["box"]
        self.last_target_metrics = metrics
        self.last_target_point = tracking["target"]
        self.last_error_norm = tracking["error_norm"]

        error_x, error_y = tracking["error_norm"]
        correction_right, correction_vertical, centered_x, centered_y = (
            get_tracking_correction(
                error_x,
                error_y,
                gain=self.config.tracking_gain_mm,
                deadband=self.config.tracking_deadband,
            )
        )

        return {
            "error_x": error_x,
            "error_y": error_y,
            "correction_right": correction_right,
            "correction_vertical": correction_vertical,
            "centered": centered_x and centered_y,
            "metrics": metrics,
        }

    def _update_centering(
        self,
        detections,
        frame_width,
        frame_height,
        class_id,
        target_y_norm,
        next_state,
    ):
        data = self._get_tracking_data(
            detections,
            frame_width,
            frame_height,
            class_id,
            target_y_norm,
        )
        if data is None:
            return

        if data["centered"]:
            self.centered_frames += 1
            if self.centered_frames >= self.config.centered_frames_required:
                self._set_state(next_state)
            return

        self.centered_frames = 0
        self._schedule_tracking_move(
            data["correction_right"],
            data["correction_vertical"],
            0.0,
        )

    def _update_approach(
        self,
        detections,
        frame_width,
        frame_height,
        class_id,
        target_y_norm,
        size_threshold,
        next_state,
    ):
        data = self._get_tracking_data(
            detections,
            frame_width,
            frame_height,
            class_id,
            target_y_norm,
        )
        if data is None:
            return

        error_x = data["error_x"]
        error_y = data["error_y"]
        width_norm = data["metrics"]["width_norm"]
        centered = data["centered"]

        if class_id == self.config.creeper_class_id:
            final_grab_aligned = (
                abs(error_x) <= self.config.final_grab_max_error_x_norm
                and abs(error_y) <= self.config.final_grab_max_error_y_norm
            )

            if final_grab_aligned and width_norm >= size_threshold:
                self.state = SequenceState.FINAL_GRAB
                print(
                    "Sequence state -> FINAL_GRAB "
                    f"(target width={width_norm:.3f}, "
                    f"error=({error_x:.3f}, {error_y:.3f}))"
                )
                return

        elif centered and width_norm >= size_threshold:
            self.state = next_state
            print(
                f"Sequence state -> {self.state.name} "
                f"(target width={width_norm:.3f})"
            )
            return

        # Only move forward while the target is centered
        approach_mm = (
            self.config.approach_sign * self.config.approach_step_mm
            if centered
            else 0.0
        )
        self._schedule_tracking_move(
            data["correction_right"],
            data["correction_vertical"],
            approach_mm,
        )

    def _schedule_tracking_move(
        self,
        correction_right,
        correction_vertical,
        approach_mm,
    ):
        self._schedule_motion(
            self.visual_servo.move,
            self.state,
            correction_right,
            correction_vertical,
            approach_mm,
            self.state.name,
        )

    def _final_grab_worker(self):
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
            self._set_error("No baseline view pose was saved")
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

        if isinstance(result, MotionOutcome):
            if not result.success:
                self._handle_tracking_failure(result.reason)
                return

            self._reset_motion_failures()
        elif not result:
            self._set_error("Arm motion failed or timed out")
            return
        else:
            self._reset_motion_failures()

        self.state = next_state
        self.centered_frames = 0
        print("Sequence state ->", self.state.name)

    def _handle_tracking_failure(self, reason):
        self.consecutive_pose_failures += 1
        self.last_motion_failure = reason

        print(
            "Recoverable tracking failure "
            f"{self.consecutive_pose_failures}/"
            f"{self.config.max_consecutive_pose_failures}: {reason}"
        )

        if self.consecutive_pose_failures >= self.config.max_consecutive_pose_failures:
            self._set_error(f"Tracking pose failed repeatedly: {reason}")

    def _set_state(self, state):
        self.state = state
        self.centered_frames = 0
        print("Sequence state ->", self.state.name)

    def _reset_motion_failures(self):
        self.consecutive_pose_failures = 0
        self.last_motion_failure = ""

    def _clear_target_debug(self):
        self.last_target_box = None
        self.last_target_metrics = None
        self.last_target_point = None
        self.last_error_norm = None

    def _missing_config(self):
        required = (
            "gripper_open_angle",
            "gripper_closed_angle",
            "creeper_grab_width_norm",
            "final_grab_forward_mm",
            "final_grab_wrist_pitch_delta_deg",
        )
        return next(
            (name for name in required if getattr(self.config, name) is None),
            None,
        )

    def _set_error(self, message):
        self.state = SequenceState.ERROR
        self.error_message = message
        print("Sequence ERROR:", message)
