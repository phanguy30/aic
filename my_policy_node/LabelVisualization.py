import cv2
import os
import numpy as np

IMAGE_FOLDER = "aic_dataset1/yolo_dataset/images/"
LABEL_FOLDER = "aic_dataset1/yolo_dataset/labels/"
OUTPUT_FOLDER = "aic_dataset1/visualized/"


def visualize_labels(image_file, label_file):
    image = cv2.imread(image_file)
    if image is None:
        print(f"Failed to load image: {image_file}")
        return None
        
    h, w, _ = image.shape
    
    # Check if label file exists and is not empty
    if os.path.exists(label_file) and os.path.getsize(label_file) > 0:
        labels = np.loadtxt(label_file)
        
        # Handle the case where there is only one label (1D array)
        if labels.ndim == 1:
            labels = np.array([labels])
            
        for label in labels:
            # We take the first 5 elements for the bounding box
            class_id, x_center, y_center, bbox_width, bbox_height = label[:5]
            
            # Convert normalized coordinates to absolute pixel coordinates
            x_center_abs = x_center * w
            y_center_abs = y_center * h
            width_abs = bbox_width * w
            height_abs = bbox_height * h
            
            # Calculate top-left (x1, y1) and bottom-right (x2, y2)
            x1 = int(x_center_abs - width_abs / 2)
            y1 = int(y_center_abs - height_abs / 2)
            x2 = int(x_center_abs + width_abs / 2)
            y2 = int(y_center_abs + height_abs / 2)
            
            # Draw bounding box (Green)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Visualize keypoints
            keypoints = label[5:]
            for i in range(0, len(keypoints), 3):
                px_norm = keypoints[i]
                py_norm = keypoints[i+1]
                
                # Convert keypoint normalized coordinates to absolute pixel coordinates
                px = int(px_norm * w)
                py = int(py_norm * h)
                
                # Draw a small circle for each keypoint (Red)
                cv2.circle(image, (px, py), 3, (0, 0, 255), -1)
                
    return image



# VERY IMPORTANT: Create the output directory if it doesn't exist
# cv2.imwrite fails silently if the target directory is missing
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# Sort all files first, then take the first 10
images_files = sorted([f for f in os.listdir(IMAGE_FOLDER) if f.endswith('.jpg')])
label_files = [f.replace('.jpg', '.txt') for f in images_files]

image_file_paths = [os.path.join(IMAGE_FOLDER, f) for f in images_files]
label_file_paths = [os.path.join(LABEL_FOLDER, f) for f in label_files]

for image_file, label_file in zip(image_file_paths, label_file_paths):
    visualized_image = visualize_labels(image_file, label_file)
    
    if visualized_image is not None:
        # Create output path and save
        output_filename = f"visualized_{os.path.basename(image_file)}"
        output_path = os.path.join(OUTPUT_FOLDER, output_filename)
        
        cv2.imwrite(output_path, visualized_image)
        print(f"Saved: {output_path}")

