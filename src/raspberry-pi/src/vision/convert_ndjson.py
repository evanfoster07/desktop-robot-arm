import json
from pathlib import Path
from urllib.request import urlretrieve


ndjson_path = Path("dataset/robot-arm-training.ndjson")
labels_root = Path("dataset/labels")
images_root = Path("dataset/images")


with open(ndjson_path, "r") as file:
    for line in file:
        data = json.loads(line)

        # Skip the first dataset metadata line
        if data["type"] != "image":
            continue

        # Retrieve image info, split and box data
        image_name = data["file"]
        image_url = data["url"]
        split = data["split"]

        # Images with no annotations may have no "annotations" field
        annotations = data.get("annotations") or {}
        boxes = annotations.get("boxes", [])

        # Download image into its train/val split
        image_dir = images_root / split
        image_dir.mkdir(parents=True, exist_ok=True)

        destination_image = image_dir / image_name

        print(f"Downloading: {image_name}")
        urlretrieve(image_url, destination_image)

        # Create corresponding label folder and write box data
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
