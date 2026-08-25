import cv2
from ultralytics import YOLO

model = YOLO("src/vision/runs/detect/creeper-3/weights/best.pt")

image = cv2.imread("dataset/images/train/creeper11.jpg")

if image is None:
    print("Failed to load image")
    exit()

print(image.shape)
results = model(image, conf=0.02)

annotated_image = results[0].plot()
cv2.imwrite("dataset/testing/creeper_test_result.jpg", annotated_image)