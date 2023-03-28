
import rospy
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

class Manager:
	def __init__(self):
		self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
		rospy.spin()

	
	def callback(self, image):
		cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
		self.detector = Detector()
		person_exists = self.detector.detect(cvimage)
		print(person_exists)
		print(type(cvimage))
	


if _name_ == '_main_':
	rospy.init_node('sos_node', anonymous=True)
	node = Manager()


# c = Detector()
# while True:
# 	cap = cv2.VideoCapture(0)
# 	ret, frame = cap.read()
	
# 	if ret:
# 		print(c.detect(frame))

