import cv2
import numpy as np

# Load the YOLOv3 model
net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')

# Load the COCO class labels
classes = []
with open('coco.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]

# Load the image
img = cv2.imread('stop_sign_image.jpg')

# Define the input size and scaling factor
input_size = 416
scale_factor = 1/255.0

# Create a blob from the image
blob = cv2.dnn.blobFromImage(img, scale_factor, (input_size, input_size), swapRB=True, crop=False)

# Set the input to the network
net.setInput(blob)

# Get the output layer names
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Run the forward pass and get the output
outputs = net.forward(output_layers)

# Process the output to get the class IDs and bounding boxes
class_ids = []
confidences = []
boxes = []
for output in outputs:
    for detection in output:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5 and classes[class_id] == 'stop sign':
            center_x = int(detection[0] * img.shape[1])
            center_y = int(detection[1] * img.shape[0])
            width = int(detection[2] * img.shape[1])
            height = int(detection[3] * img.shape[0])
            x = int(center_x - width / 2)
            y = int(center_y - height / 2)
            class_ids.append(class_id)
            confidences.append(float(confidence))
            boxes.append([x, y, width, height])

# Apply non-maximum suppression to remove overlapping boxes
indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

# Draw the bounding boxes around the detected stop signs
for i in indices:
    #i = i[0]
    box = boxes[i]
    x, y, w, h = box
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

# Display the image with the detected stop signs
cv2.imshow('Stop sign detection', img)
cv2.waitKey(0)
cv2.destroyAllWindows()