from flask import Flask, Response, redirect, request, url_for
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO
import cv2
import html

from arm_serial import ArmSerial
from motion_sequence import MotionSequence, SequenceConfig


app = Flask(__name__)

# Load your current trained model
MODEL_PATH = "src/vision/runs/detect/creeper-3/weights/best.pt"
model = YOLO(MODEL_PATH)

# Camera
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (1280, 720)},
    raw={"size": picam2.sensor_resolution},
    transform=Transform(hflip=1, vflip=1)
)
picam2.configure(config)
picam2.start()

# Arm connection
arm = ArmSerial()

# Autonomous sequence
sequence_config = SequenceConfig(
    gripper_open_angle=40,
    gripper_closed_angle=60,

    # --- FINAL GRAB CALIBRATION ---
    # Trigger terminal grab at this Creeper box width.
    creeper_grab_width_norm=0.65,

    # Lurch forward this many mm while holding overall Cartesian tool pitch.
    final_grab_forward_mm=14,

    # Then directly command ONLY the wrist-pitch servo to this ABSOLUTE angle.
    final_grab_wrist_pitch_delta_deg=10,

    # Goal still uses normal visual alignment + size threshold.
    goal_drop_width_norm=0.65,

    approach_sign=1.0,
)
sequence = MotionSequence(arm, sequence_config)

# Browser manual-command status
last_manual_command = ""
last_manual_response = ""

CLASS_COLORS = {
    0: (0, 100, 0),
    1: (0, 0, 255),
}


def get_detections(frame):
    results = model(frame, imgsz=640, verbose=False)
    result = results[0]
    detections = []

    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        confidence = float(box.conf[0])
        class_id = int(box.cls[0])

        detections.append({
            "class_id": class_id,
            "confidence": confidence,
            "box": [x1, y1, x2, y2],
            "center": [(x1 + x2) / 2, (y1 + y2) / 2],
        })

    return detections


def draw_detections(frame, detections):
    annotated = frame.copy()

    for detection in detections:
        class_id = detection["class_id"]
        confidence = detection["confidence"]
        x1, y1, x2, y2 = map(int, detection["box"])

        color = CLASS_COLORS.get(class_id, (255, 255, 255))
        label = f"{model.names[class_id]} {confidence:.2f}"

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)
        cv2.putText(
            annotated,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2
        )

    return annotated


def draw_sequence_debug(frame):
    debug = sequence.get_debug_info()

    cv2.putText(
        frame,
        f"STATE: {debug['state']}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    if debug["target_point"] is not None:
        tx, ty = map(int, debug["target_point"])
        cv2.drawMarker(
            frame,
            (tx, ty),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            24,
            2
        )

    metrics = debug["target_metrics"]
    error_norm = debug["error_norm"]
    y = 65

    if metrics is not None:
        cv2.putText(
            frame,
            f"target width={metrics['width_norm']:.3f} area={metrics['area_norm']:.3f}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )
        y += 28

    if error_norm is not None:
        cv2.putText(
            frame,
            f"error=({error_norm[0]:.3f}, {error_norm[1]:.3f})",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )
        y += 28

    if debug["moving"]:
        cv2.putText(
            frame,
            "ARM MOVING",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

    return frame


def run_inference(frame):
    detections = get_detections(frame)

    sequence.update(
        detections,
        frame.shape[1],
        frame.shape[0]
    )

    annotated_frame = draw_detections(frame, detections)
    annotated_frame = draw_sequence_debug(annotated_frame)

    return annotated_frame, detections


def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        annotated_frame, _ = run_inference(frame)

        success, buffer = cv2.imencode(".jpg", annotated_frame)
        if not success:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )


@app.route("/")
def index():
    debug = sequence.get_debug_info()

    command_text = html.escape(last_manual_command)
    response_text = html.escape(last_manual_response)

    return f"""
    <html>
        <body>
            <h1>Robot Arm YOLO Inference</h1>

            <p>Sequence state: <b>{debug['state']}</b></p>

            <form action="/start" method="post" style="display:inline;">
                <button type="submit">Start Pick + Place</button>
            </form>

            <form action="/stop" method="post" style="display:inline;">
                <button type="submit">Stop Sequence</button>
            </form>

            <hr>

            <h3>Manual ESP32 Command</h3>

            <form action="/command" method="post">
                <input
                    type="text"
                    name="command"
                    placeholder="s 90, e 120, g 30, fk, GET_STATE..."
                    style="width: 340px;"
                    autocomplete="off"
                    autofocus
                >
                <button type="submit">Send</button>
            </form>

            <p>
                Same format as the ESP32 serial monitor. Manual commands are
                blocked while the autonomous sequence is active.
            </p>

            <p><b>Last command:</b> {command_text}</p>
            <pre>{response_text}</pre>

            <hr>

            <img src="/video_feed">
        </body>
    </html>
    """


@app.route("/command", methods=["POST"])
def manual_command():
    global last_manual_command, last_manual_response

    command = request.form.get("command", "").strip()

    if not command:
        return redirect(url_for("index"))

    last_manual_command = command

    # Prevent manual commands from fighting the autonomous state machine.
    if sequence.active:
        last_manual_response = (
            "Command blocked: stop the autonomous sequence first."
        )
        return redirect(url_for("index"))

    try:
        response_lines = arm.run_command(command)

        if response_lines:
            last_manual_response = "\n".join(response_lines)
        else:
            last_manual_response = "Command sent (no serial response)."

    except Exception as exc:
        last_manual_response = f"Serial command error: {exc}"

    return redirect(url_for("index"))


@app.route("/start", methods=["POST"])
def start_sequence():
    sequence.start()
    return redirect(url_for("index"))


@app.route("/stop", methods=["POST"])
def stop_sequence():
    sequence.stop()
    return redirect(url_for("index"))


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
        debug=False,
        threaded=True
    )
