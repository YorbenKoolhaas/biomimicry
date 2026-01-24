import os

from ultralytics import YOLO

# path to your trained model
model = YOLO("C:\\Users\\Riko\\PycharmProjects\\GripNSnip\\runs\\detect\\train17\\weights\\best.pt")

# run detection on a strawberry image
results = model.predict(source="strawberry3.png", save=True, conf=0.35)

# output written to runs/detect/predictN/ by default
# extract bounding box coordinates
bounding_boxes = []
centers = []
for result in results:
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0]  # extract the coordinates

        # convert tensors to Python floats
        x1, y1, x2, y2 = x1.item(), y1.item(), x2.item(), y2.item()

        # compute center
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        bounding_boxes.append((x1, y1, x2, y2))
        centers.append((cx, cy))

# function to generate a unique filename
def generate_unique_filename(directory, base_name, extension):
    counter = 1
    while True:
        filename = os.path.join(directory, f"{base_name}_{counter}.{extension}")
        if not os.path.exists(filename):
            return filename
        counter += 1

# define the output directory and base name for the bounding box coordinates
output_directory = "C:\\Users\\Riko\\PycharmProjects\\GripNSnip\\runs\\detect\\bounding_box_coordinates"
base_name = "bounding_boxes"

# generate a unique output file path
output_file_path = generate_unique_filename(output_directory, base_name, "txt")

# write the bounding box coordinates to the file
with open(output_file_path, "w") as file:
    for i, ((x1, y1, x2, y2), (cx, cy)) in enumerate(zip(bounding_boxes, centers)):
        file.write(f"Bounding Box {i+1}: ({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f})\n")
        file.write(f"Center: ({cx:.2f}, {cy:.2f})\n\n")


print(f"Bounding box coordinates have been written to {output_file_path}")
