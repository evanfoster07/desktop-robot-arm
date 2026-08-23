import cv2
from ultralytics import YOLO

model = YOLO("yolo26n.pt")

image = cv2.imread("images/initial_tests/test1.jpg")

if image is None:
    print("Failed to load image")
    exit()

print(image.shape)
results = model(image)