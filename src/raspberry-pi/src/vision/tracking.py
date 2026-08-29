import math


def get_box_error(
    box,
    frame_width,
    frame_height,
    target_x=None,
    target_y=None
):
    x1, y1, x2, y2 = box

    box_x = (x1 + x2) / 2
    box_y = (y1 + y2) / 2

    if target_x is None:
        target_x = frame_width / 2

    if target_y is None:
        target_y = frame_height / 2

    error_x = box_x - target_x
    error_y = box_y - target_y

    error_x_norm = error_x / (frame_width / 2)
    error_y_norm = error_y / (frame_height / 2)

    return {
        "center": (box_x, box_y),
        "target": (target_x, target_y),
        "error_px": (error_x, error_y),
        "error_norm": (error_x_norm, error_y_norm),
    }


def get_tracking_correction(error_x, error_y, gain=10.0, deadband=0.05):
    dx = 0.0
    dy = 0.0
    centered_x = True
    centered_y = True

    if abs(error_x) > deadband:
        dx = gain * error_x
        centered_x = False

    if abs(error_y) > deadband:
        dy = gain * error_y
        centered_y = False

    return dx, dy, centered_x, centered_y


def get_tool_axes(base_deg, pitch_deg):
    base = math.radians(base_deg)
    pitch = math.radians(pitch_deg)

    forward = (
        math.cos(pitch) * math.cos(base),
        math.cos(pitch) * math.sin(base),
        math.sin(pitch)
    )

    right = (
        -math.sin(base),
        math.cos(base),
        0.0
    )

    up = (
        -math.sin(pitch) * math.cos(base),
        -math.sin(pitch) * math.sin(base),
        math.cos(pitch)
    )

    return forward, right, up

def camera_correction_to_robot(correction_x, correction_y, base_deg, pitch_deg):
    """
    Convert image-space correction into robot XYZ correction.
    """

    # Experimental calibration showed that robot motion must
    # oppose both image-space error directions
    # pose + y 10 mm = creeper move right on screen
    # pose + z 10 mm = creeper move down on screen

    _, camera_right, camera_up = get_tool_axes(base_deg, pitch_deg)

    dx = (
        -correction_x * camera_right[0]
        -correction_y * camera_up[0]
    )

    dy = (
        -correction_x * camera_right[1]
        -correction_y * camera_up[1]
    )

    dz = (
        -correction_x * camera_right[2]
        -correction_y * camera_up[2]
    )

    return dx, dy, dz