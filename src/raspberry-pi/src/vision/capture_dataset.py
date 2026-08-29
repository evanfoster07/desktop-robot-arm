from pathlib import Path
from picamera2 import Picamera2
from libcamera import Transform
from flask import Flask, Response
import cv2
import time
import threading


SAVE_DIR = (
    Path.home()
    / "desktop-robot-arm"
    / "src"
    / "raspberry-pi"
    / "dataset"
    / "images_raw"
)

PREFIX = "scene"

app = Flask(__name__)

picam2 = Picamera2()

# Prevent browser streaming and image capture from
# trying to access the camera at exactly the same time.
camera_lock = threading.Lock()


def get_next_image_number():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    existing_numbers = []

    # Look at every JPG regardless of prefix.
    for image_path in SAVE_DIR.glob("*.jpg"):
        try:
            number = int(image_path.stem.split("_")[-1])
            existing_numbers.append(number)
        except ValueError:
            pass

    if not existing_numbers:
        return 1

    return max(existing_numbers) + 1


def generate_frames():
    while True:

        with camera_lock:
            # Grab smaller preview stream
            frame = picam2.capture_array("lores")

        success, buffer = cv2.imencode(".jpg", frame)

        if not success:
            continue

        frame_bytes = buffer.tobytes()

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
            <h1>Dataset Camera</h1>
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


def start_web_server():
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False
    )


def main():
    image_number = get_next_image_number()

    # main  = high-resolution image saved to dataset
    # lores = smaller image used only for browser livestream
    config = picam2.create_still_configuration(
        main={"size": (3280, 2464)},
        lores={"size": (640, 480), "format": "RGB888"},
        transform=Transform(hflip=1, vflip=1)
    )

    picam2.configure(config)
    picam2.start()

    # Give auto exposure / white balance time to settle
    time.sleep(2)

    # Flask needs its own thread because app.run() normally
    # blocks the rest of the Python program.
    web_thread = threading.Thread(
        target=start_web_server,
        daemon=True
    )

    web_thread.start()

    print(f"Saving images to: {SAVE_DIR}")
    print(f"Next image: {PREFIX}_{image_number:03d}.jpg")
    print()
    print("Livestream: http://robotarm.local:5000")
    print()
    print("Press ENTER to capture")
    print("Type q then ENTER to quit")

    try:
        while True:
            command = input("> ").strip().lower()

            if command == "q":
                break

            filename = f"{PREFIX}_{image_number:03d}.jpg"
            output_path = SAVE_DIR / filename

            with camera_lock:
                # Save from HIGH-RES main stream
                picam2.capture_file(
                    str(output_path),
                    name="main"
                )

            print(f"Saved: {output_path}")

            image_number += 1

    finally:
        picam2.stop()


if __name__ == "__main__":
    main()