from ultralytics import YOLO

model = YOLO("yolo26n.pt")

model.train(
    data="dataset/data.yaml",
    epochs=20,
    imgsz=640,
    project="src/vision/runs/detect",
    name="creeper"
)