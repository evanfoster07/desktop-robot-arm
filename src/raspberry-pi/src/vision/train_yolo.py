from ultralytics import YOLO
from pathlib import Path


def main():
    model = YOLO("yolo26n.pt")

    output_dir = Path(__file__).parent / "runs" / "detect"

    model.train(
        data="dataset/data.yaml",
        epochs=30,
        imgsz=640,
        freeze=10,
        project=output_dir,
        name="creeper"
    )
    

if __name__ == "__main__":
    main()