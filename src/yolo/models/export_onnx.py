from ultralytics import YOLO

# Load the YOLOv11n model
model = YOLO("/home/junsu/1.yolo_seg_linux/src/yolo/src/models/1.pt")

# Export the model to ONNX format
model.export(format="onnx")