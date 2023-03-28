#!/usr/bin/env python3
# import rospy
# from pacmod_msgs.msg import PacmodCmd
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
import cv2
#import imutils
import numpy as np
class Detector:

	def detect(self, frame):
		blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)
		net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
		layer_names = net.getLayerNames()
		output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
		
		net.setInput(blob)
		outs = net.forward(output_layers)

		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax(scores)
				confidence = scores[class_id]
				if confidence > 0.5 and class_id == 0: # 0 corresponds to person class in COCO dataset
					return True
		return False



c = Detector()
while True:
	cap = cv2.VideoCapture(0)
	ret, frame = cap.read()
	
	if ret:
		print(c.detect(frame))

