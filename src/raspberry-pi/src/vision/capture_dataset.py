from pathlib import Path
from picamera2 import Picamera2
import time
from libcamera import Transform


SAVE_DIR = Path.home() / "desktop-robot-arm" / "src" / "raspberry-pi" / "dataset" / "images_raw"
PREFIX = "creeper"


def get_next_image_number():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    existing_numbers = []

    for image_path in SAVE_DIR.glob(f"{PREFIX}_*.jpg"):
        try:
            number = int(image_path.stem.split("_")[-1])
            existing_numbers.append(number)
        except ValueError:
            pass

    if not existing_numbers:
        return 1

    return max(existing_numbers) + 1


def main():
    image_number = get_next_image_number()

    # Create and configure camera once
    picam2 = Picamera2()

    config = picam2.create_still_configuration(
        main={"size": (3280, 2464)},
        transform=Transform(hflip=1, vflip=1)
    )

    picam2.configure(config)
    picam2.start()

    # Give auto exposure / white balance a moment to settle
    time.sleep(2)

    print(f"Saving images to: {SAVE_DIR}")
    print(f"Next image: {PREFIX}_{image_number:03d}.jpg")
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

            picam2.capture_file(str(output_path))

            print(f"Saved: {output_path}")

            image_number += 1

    finally:
        picam2.stop()


if __name__ == "__main__":
    main()