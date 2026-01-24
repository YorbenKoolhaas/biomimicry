from ultralytics import YOLO

# load YOLOv8 nano model
model = YOLO("yolov8n.pt")

# train
if __name__ == "__main__":
    model = YOLO("yolov8n.pt")
    results = model.train(
        data="C:\\Users\\Riko\\Downloads\\GSD\\GSD-YOLO\\data.yaml", #change path to your data.yaml location
        epochs=100,
        imgsz=640,
        batch=8,
        device=0,
        patience=15
    )
