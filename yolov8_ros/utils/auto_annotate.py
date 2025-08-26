# Adapted from Ultralytics YOLO (https://docs.ultralytics.com/reference/data/annotator/#ultralytics.data.annotator.auto_annotate)

from pathlib import Path
from collections import Counter
from ultralytics import YOLO
import cv2
import argparse
import numpy as np

def auto_annotate(data, det_model="yolov8x.pt", device="", output_dir=None, desired_class_id=None, draw=False):
    """
    Automatically annotates images using a YOLO object detection model.

    This function processes all images in a given directory and detects objects using the YOLO model. The resulting annotation is saved as a text file for each image, in the normalized [x_center y_center width height] format.
    
    Args:
        data (str): Path to the folder with the images to be annotated.
        det_model (str | "yolov8x.pt"): Path or name of the pre-trained YOLO model.
        device (str | ""): Device that will process the model (e.g., 'cpu', 'cuda', '0').
        output_dir (str | None): Directory to save the annotated results. If None, a default directory will be created.
        desired_class_id (int | None): ID of the desired class to annotate. If None, all classes will be annotated.
        draw (bool | False): If True, draws the bounding boxes on the original image and saves the annotated image.

    Notes:
        - The function creates a new output directory if one is not specified.
        - Annotation results are saved as text files with the same name as the image file.
        - Each line in the output text file represents a detected object with its class ID and bounding box points, in the format [x_center y_center width height] normalized by the image size.
    """
    det_model = YOLO(det_model)

    # Read data from the input directory and create an output directory if it doesn't exist
    data = Path(data)
    if not output_dir:
        output_dir = data.parent / f"{data.stem}_auto_annotate_labels"
    if data.suffix == ".txt":
        with open(data, "r") as file:
            image_paths = [line.strip() for line in file.readlines()]
        data = image_paths
    Path(output_dir).mkdir(exist_ok=True, parents=True)

    print("🔍 Starting YOLO inference for object detection in the images...")

    # Perform inference on all images
    det_results = det_model(data, device=device)

    # Display detailed results per image
    print("\n🖼️  Results per Image\n")

    print(f"Image        |        Detections")
    print(f"-----------------------------------")
    for result in det_results:
        class_ids = result.boxes.cls.int().tolist()  # noqa
        # Display the filename and a detection summary for each processed image
        class_counts = Counter([det_model.names[c] for c in class_ids])
        summary = ', '.join(f"{v} {k}" for k, v in class_counts.items())
        print(f"{Path(result.path).name} | {summary}")

        if draw == True:
            img = cv2.imread(result.path)

        if len(class_ids):
            boxes = result.boxes.xywhn  # Bounding boxes of an image in normalized xywh format

            filtered_boxes = []
            filtered_class_ids = []

            # Filter the bounding boxes by the desired_class_id if it is not None
            for i in range(len(class_ids)):
                if desired_class_id is None or class_ids[i] == desired_class_id:
                    filtered_boxes.append(boxes[i])
                    filtered_class_ids.append(class_ids[i])

                    if draw == True:
                        box = boxes[i]
                        x_center, y_center, width, height = box
                        x_center *= img.shape[1]
                        y_center *= img.shape[0]
                        width *= img.shape[1]
                        height *= img.shape[0]

                        x1 = int(x_center - width / 2)
                        y1 = int(y_center - height / 2)
                        x2 = int(x_center + width / 2)
                        y2 = int(y_center + height / 2)

                        np.random.seed(class_ids[i])  # Ensures the color is always the same for the same ID
                        color = tuple(int(x) for x in np.random.randint(0, 255, size=3))

                        # Draw the bounding box with the class color
                        img = cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

                        # Draw the class name with the same color
                        img = cv2.putText(img, str(det_model.names[class_ids[i]]), (x1, y1 + 20),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 4)
                        
            if len(filtered_boxes) > 0:
                with open(f"{Path(output_dir) / Path(result.path).stem}.txt", "w") as f:
                    # Write each filtered bounding box to a new line in the text file
                    for i in range(len(filtered_boxes)):
                        b = filtered_boxes[i]
                        if len(b) == 0:
                            continue
                        box = map(str, filtered_boxes[i].reshape(-1).tolist())
                        f.write(f"{filtered_class_ids[i]} " + " ".join(box) + "\n")
                    if draw == True:
                        output_image_path = Path(output_dir) / f"{Path(result.path).stem}_annotated.jpg"
                        cv2.imwrite(str(output_image_path), img)

    print(f"\n💾 Saving annotated images to the directory: {str(output_dir)}") 
    print("\n✅ Auto-annotation completed successfully!")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Auto annotate images using a YOLO model.")
    parser.add_argument("data", type=str, help="Path to the folder containing the images to annotate.")
    parser.add_argument("--det_model", type=str, default="yolov8x.pt", help="Path or name of the YOLO model.")
    parser.add_argument("--device", type=str, default="", help="Device to run the model on (e.g., 'cpu', 'cuda', '0').")
    parser.add_argument("--output_dir", type=str, default=None, help="Directory to save the annotated results.")
    parser.add_argument("--desired_class_id", type=int, default=None, help="ID of the class to annotate. Annotates all classes if not specified.")
    parser.add_argument("--draw", action="store_true", help="Draws the bounding boxes on the original images and saves them.")

    args = parser.parse_args()

    auto_annotate(
        data=args.data,
        det_model=args.det_model,
        device=args.device,
        output_dir=args.output_dir,
        desired_class_id=args.desired_class_id,
        draw=args.draw,
    )