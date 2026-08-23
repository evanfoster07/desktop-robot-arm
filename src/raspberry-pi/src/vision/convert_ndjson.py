import json
from pathlib import Path
import shutil

ndjson_path = Path("dataset/robot-arm-training.ndjson")
labels_root = Path("dataset/labels")

images_source = Path("dataset/images_raw")
images_root = Path("dataset/images")

with open(ndjson_path, "r") as file:
    for line in file:
        data = json.loads(line)

        # Skip the first dataset metadata line
        if data["type"] != "image":
            continue

        # Retrieve name, split and box data
        image_name = data["file"]
        split = data["split"]
        boxes = data["annotations"]["boxes"]

        # Copy images to test/val sets
        image_dir = images_root / split
        image_dir.mkdir(parents=True, exist_ok=True)

        source_image = images_source / image_name
        destination_image = image_dir / image_name

        shutil.copy2(source_image, destination_image)

        # Create corresponding .txt folders and write box data
        label_dir = labels_root / split
        label_dir.mkdir(parents=True, exist_ok=True)

        label_name = Path(image_name).stem + ".txt"
        label_path = label_dir / label_name

        with open(label_path, "w") as label_file:
            for box in boxes:
                class_id, x_center, y_center, width, height = box

                label_file.write(
                    f"{class_id} {x_center} {y_center} {width} {height}\n"
                )