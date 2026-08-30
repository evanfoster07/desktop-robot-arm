from flask import Flask, Response
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO
import cv2
import time
from concurrent.futures import ThreadPoolExecutor
from tracking import *
from arm_serial import ArmSerial

app = Flask(__name__)

# Load trained YOLO model once when script starts
model = YOLO("src/vision/runs/detect/creeper-2/weights/best.pt")

# Create camera once
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1280, 720)},
    raw={"size": picam2.sensor_resolution},
    transform=Transform(hflip=1, vflip=1)
)

picam2.configure(config)
picam2.start()

arm = ArmSerial()

# Send at most one correction per interval. This gives the arm time to move
# without permanently disabling continuous tracking or flooding the ESP32.
CORRECTION_INTERVAL_S = 1.0
last_correction_time = 0.0

# Serial communication runs on this single background worker so waiting for
# the ESP32 never prevents the browser stream from receiving its next frame.
control_executor = ThreadPoolExecutor(max_workers=1)
control_future = None


def apply_tracking_correction(error_x, error_y, correction_right):
    """Get the arm state and send one tracking correction in the background."""

    state = arm.get_state()

    if state is None:
        print("Tracking skipped: failed to get arm state")
        return

    # Use wrist pitch for vertical image correction.
    PITCH_GAIN_DEG = 10.0
    MAX_PITCH_STEP_DEG = 3.0

    pitch_step = max(
        -MAX_PITCH_STEP_DEG,
        min(MAX_PITCH_STEP_DEG, -PITCH_GAIN_DEG * error_y)
    )

    target_pitch = state["pitch"] + pitch_step

    # Use XYZ movement only for horizontal image correction.
    robot_dx, robot_dy, robot_dz = camera_correction_to_robot(
        correction_right,
        0.0,
        state["base"],
        state["pitch"]
    )

    MAX_STEP_MM = 5.0

    robot_dx = max(-MAX_STEP_MM, min(MAX_STEP_MM, robot_dx))
    robot_dy = max(-MAX_STEP_MM, min(MAX_STEP_MM, robot_dy))
    robot_dz = max(-MAX_STEP_MM, min(MAX_STEP_MM, robot_dz))

    target_x = state["x"] + robot_dx
    target_y = state["y"] + robot_dy
    target_z = state["z"] + robot_dz

    arm.move_pose(
        target_x,
        target_y,
        target_z,
        target_pitch,
        state["roll"]
    )

    print(
        "Tracking move: "
        f"error=({error_x:.3f}, {error_y:.3f}), "
        f"delta=({robot_dx:.2f}, {robot_dy:.2f}, {robot_dz:.2f}) mm, "
        f"pitch_step={pitch_step:.2f} deg"
    )


def report_control_error(future):
    """Print exceptions raised by the background control worker"""

    exception = future.exception()

    if exception is not None:
        print(f"Tracking control error: {exception}")

def run_inference(frame):
    """
    Runs YOLO inference on one frame

    Return:
        annotated_frame:
            Original camera frame with YOLO boxes/labels drawn on it

        detections:
            List containing useful information for each detection
    """

    global last_correction_time, control_future

    # Run YOLO
    results = model(
        frame,
        imgsz=640,
        verbose=False
    )

    # Only passed one image, so take the first result
    result = results[0]

    detections = []

    # Each detected object has one bounding box
    for box in result.boxes:

        # xyxy format:
        # [left, top, right, bottom]
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        confidence = float(box.conf[0])
        class_id = int(box.cls[0])

        # Calculate center of bounding box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        detections.append({
            "class_id": class_id,
            "confidence": confidence,
            "box": [x1, y1, x2, y2],
            "center": [cx, cy]
        })

    # Choose box colours by class
    # OpenCV uses BGR
    CLASS_COLORS = {
        0: (0, 100, 0),   # creeper - dark green
        1: (0, 0, 255),   # goal - red
    }

    # Start with the original frame
    annotated_frame = frame.copy()

    # Draw each detection manually
    for detection in detections:

        class_id = detection["class_id"]
        confidence = detection["confidence"]

        x1, y1, x2, y2 = map(int, detection["box"])

        color = CLASS_COLORS.get(class_id, (255, 255, 255))
        label = f"{model.names[class_id]} {confidence:.2f}"

        # Bounding box
        cv2.rectangle(
            annotated_frame,
            (x1, y1),
            (x2, y2),
            color,
            3
        )

        # Label
        cv2.putText(
            annotated_frame,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2
        )

    # Track only the highest-confidence creeper 
    # Acting inside the drawing loop could send multiple corrections for one frame
    creepers = [d for d in detections if d["class_id"] == 0]

    if not creepers:
        return annotated_frame, detections

    creeper = max(creepers, key=lambda detection: detection["confidence"])

    tracking = get_box_error(
        creeper["box"],
        frame.shape[1],
        frame.shape[0]
    )

    error_x, error_y = tracking["error_norm"]

    correction_right, _, centered_x, centered_y = get_tracking_correction(
        error_x,
        error_y,
        gain=10.0,
        deadband=0.05
    )

    # Do not request arm state when no move is needed.
    if centered_x and centered_y:
        return annotated_frame, detections

    now = time.monotonic()

    if now - last_correction_time < CORRECTION_INTERVAL_S:
        return annotated_frame, detections

    # Do not queue another correction while the previous serial operation is
    # still running. The video path only schedules work and returns immediately.
    if control_future is None or control_future.done():
        last_correction_time = now

        control_future = control_executor.submit(
            apply_tracking_correction,
            error_x,
            error_y,
            correction_right
        )

        control_future.add_done_callback(report_control_error)


    return annotated_frame, detections


def generate_frames():
    while True:

        # Capture full-FOV camera frame
        frame = picam2.capture_array()

        # Picamera2 gives RGB
        # OpenCV / YOLO plotting works with BGR here
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Run object detection
        annotated_frame, _ = run_inference(frame)

        # Encode annotated image as JPEG for browser streaming
        success, buffer = cv2.imencode(".jpg", annotated_frame)

        if not success:
            continue

        frame_bytes = buffer.tobytes()

        # Send one MJPEG frame to the browser
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes
            + b"\r\n"
        )


@app.route("/")
def index():
    return """
    <html>
        <body>
            <h1>Robot Arm YOLO Inference</h1>
            <img src="/video_feed">
        </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False
    )
