from ultralytics import YOLO
from pathlib import Path


def main():
    model = YOLO("src/vision/runs/detect/creeper-2/weights/best.pt")

    output_dir = Path(__file__).parent / "runs" / "detect"

    model.train(
        data="dataset/data.yaml",
        epochs=20,
        imgsz=640,
        freeze=10,
        batch=8,
        workers=2,
        project=output_dir,
        name="creeper"
    )
    

if __name__ == "__main__":
    main()