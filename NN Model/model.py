import torch
import cv2
import numpy as np
from time import time
import ultralytics
# Load the YOLOv5 model.
# You can specify different model sizes like 'yolov5s', 'yolov5m', etc.
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Function to get the name of the objects
def get_object_names():

    with open('yolov5/data/coco.yaml', 'r') as f:
        coco_classes = f.read().split('\n')[2:]
        return coco_classes

# Open a connection to the webcam. '0' is the default camera.
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Webcam feed started. Press 'q' to exit.")


while True:

    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    start_time = time()
    results = model(frame_rgb)
    end_time = time()


    detections = results.pandas().xyxy[0]
    

    object_names = detections['name'].tolist()
    
 
    for index, row in detections.iterrows():
    
        xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
        
      
        confidence = row['confidence']
    
        class_name = row['name']

        if confidence > 0.5:
           
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            
            
            label = f'{class_name}: {confidence:.2f}'
            
            
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('YOLO Object Identification', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()