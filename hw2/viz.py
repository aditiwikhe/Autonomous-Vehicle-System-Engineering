import cv2
import numpy as np
import math

# class Detector:

#     # return (BOOL(whether person detected or not), HEIGHT, WIDTH) 
#     def detect(self, frame):
#         classes = []
#         with open("coco.names", "r") as f:
#             classes = [line.strip() for line in f.readlines()]

#         height, width, _ = frame.shape
#         blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)
#         net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
#         layer_names = net.getLayerNames()
#         output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        
#         net.setInput(blob)
#         outs = net.forward(output_layers)

#         # Post-process outputs
#         class_ids = []
#         confidences = []
#         boxes = []

#         for out in outs:
#             for detection in out:
#                 scores = detection[5:]
#                 class_id = np.argmax(scores)
#                 confidence = scores[class_id]
#                 if confidence > 0.5 and class_id == 0: # 0 corresponds to person class in COCO dataset
#                     # Convert YOLOv4 Tiny output format to (x, y, w, h) bounding box format
#                     center_x = int(detection[0] * width)
#                     center_y = int(detection[1] * height)
#                     w = int(detection[2] * width)
#                     h = int(detection[3] * height)
#                     x = int(center_x - w/2)
#                     y = int(center_y - h/2)

#                     class_ids.append(class_id)
#                     confidences.append(float(confidence))
#                     boxes.append([x, y, w, h])
#                     return (True, h, w)
                
#                     #Apply non-maximum suppression to remove overlapping boxes
#         indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
#         # Draw boxes and labels on frame
#         for i in indices:
#             # i = i[0]
#             x, y, w, h = boxes[i]
#             cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
#             label = "{}: {:.2f}%".format(classes[class_ids[i]], confidence)
#             cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
#             #print(h)
#             cv2.imshow("Pedestrian Detection", frame)
                 
          
#         return (False, math.inf, math.inf)
        

# d = Detector()
# cap = cv2.VideoCapture(0)
# while True:
#     ret, frame = cap.read()
#     if ret:
#         print(d.detect(frame))
#         cv2.imshow("Pedestrian Detection", frame) 
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#          break





# Load YOLOv4 Tiny weights and configuration files
net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")

# Load class names
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Set input and output layers
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

#Load video feed
cap = cv2.VideoCapture(0)


while True:
    # Read frame from video feed
    ret, frame = cap.read()

    if ret:
        # Resize frame to 416x416 (YOLOv4 Tiny input size)
        height, width, channels = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)

        # Pass frame through network
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Post-process outputs
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == 0: # 0 corresponds to person class in COCO dataset
                    # Convert YOLOv4 Tiny output format to (x, y, w, h) bounding box format
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)

                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])


        #Apply non-maximum suppression to remove overlapping boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        # Draw boxes and labels on frame
        for i in indices:
            # i = i[0]
            x, y, w, h = boxes[i]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = "{}: {:.2f}%".format(classes[class_ids[i]], confidence)
            cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            print(h)

    # Show frame
    cv2.imshow("Pedestrian Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

