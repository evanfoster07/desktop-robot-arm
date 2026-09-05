import cv2
from flask import Flask, Response, redirect, render_template, request, url_for
from libcamera import Transform
from picamera2 import Picamera2
from ultralytics import YOLO

from arm_serial import ArmSerial
from motion_sequence import MotionSequence, SequenceConfig


MODEL_PATH = "src/vision/runs/detect/creeper-3/weights/best.pt"
FRAME_SIZE = (1280, 720)
INFERENCE_SIZE = 640

CLASS_COLORS = {
    0: (0, 100, 0),
    1: (0, 0, 255),
}

SEQUENCE_CONFIG = SequenceConfig(
    gripper_open_angle=40,
    gripper_closed_angle=60,
    creeper_grab_width_norm=0.65,
    final_grab_forward_mm=14,
    final_grab_wrist_pitch_delta_deg=10,
    goal_drop_width_norm=0.65,
    approach_sign=1.0,
)


app = Flask(__name__)
model = YOLO(MODEL_PATH)
arm = ArmSerial()
sequence = MotionSequence(arm, SEQUENCE_CONFIG)

last_manual_command = ""
last_manual_response = ""


# Camera setup
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(
    main={"size": FRAME_SIZE},
    raw={"size": picam2.sensor_resolution},
    transform=Transform(hflip=1, vflip=1),
)
picam2.configure(camera_config)
picam2.start()


def get_detections(frame):
    result = model(frame, imgsz=INFERENCE_SIZE, verbose=False)[0]
    detections = []

    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        detections.append({
            "class_id": int(box.cls[0]),
            "confidence": float(box.conf[0]),
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
        _draw_text(
            annotated,
            label,
            (x1, max(y1 - 10, 20)),
            0.8,
            color,
        )

    return annotated


def draw_sequence_debug(frame):
    debug = sequence.get_debug_info()
    _draw_text(frame, f"STATE: {debug['state']}", (20, 35), 0.8)

    if debug["target_point"] is not None:
        tx, ty = map(int, debug["target_point"])
        cv2.drawMarker(
            frame,
            (tx, ty),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            24,
            2,
        )

    y = 65
    metrics = debug["target_metrics"]
    error_norm = debug["error_norm"]

    if metrics is not None:
        _draw_text(
            frame,
            f"target width={metrics['width_norm']:.3f} "
            f"area={metrics['area_norm']:.3f}",
            (20, y),
        )
        y += 28

    if error_norm is not None:
        _draw_text(
            frame,
            f"error=({error_norm[0]:.3f}, {error_norm[1]:.3f})",
            (20, y),
        )
        y += 28

    if debug["moving"]:
        _draw_text(frame, "ARM MOVING", (20, y))

    return frame


def run_inference(frame):
    detections = get_detections(frame)
    sequence.update(detections, frame.shape[1], frame.shape[0])

    annotated = draw_detections(frame, detections)
    draw_sequence_debug(annotated)
    return annotated, detections


def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        annotated, _ = run_inference(frame)

        success, buffer = cv2.imencode(".jpg", annotated)
        if not success:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )


def _draw_text(
    frame,
    text,
    position,
    scale=0.65,
    color=(255, 255, 255),
):
    cv2.putText(
        frame,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
    )


@app.route("/")
def index():
    return render_template(
        "index.html",
        state=sequence.get_debug_info()["state"],
        last_command=last_manual_command,
        last_response=last_manual_response,
    )


@app.route("/command", methods=["POST"])
def manual_command():
    global last_manual_command, last_manual_response

    command = request.form.get("command", "").strip()
    if not command:
        return redirect(url_for("index"))

    last_manual_command = command

    if sequence.active:
        last_manual_response = (
            "Command blocked: stop the autonomous sequence first."
        )
        return redirect(url_for("index"))

    try:
        response_lines = arm.run_command(command)
        last_manual_response = (
            "\n".join(response_lines)
            if response_lines
            else "Command sent (no serial response)."
        )
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
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=True,
    )
