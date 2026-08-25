from ultralytics import YOLO


def main():
    model = YOLO("yolo26n.pt")

    model.train(
        data="dataset/data.yaml",
        epochs=20,
        imgsz=640,
        freeze=10,
        project="src/vision/runs/detect",
        name="creeper"
    )


if __name__ == "__main__":
    main()