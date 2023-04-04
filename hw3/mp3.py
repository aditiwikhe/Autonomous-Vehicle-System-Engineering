
import rospy
import statistics
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detector:
  
	# return (BOOL(whether person detected or not), HEIGHT, WIDTH) 
	def detect(self, frame):
		height, width, _ = frame.shape
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
					center_x = int(detection[0] * width)
					center_y = int(detection[1] * height)
					w = int(detection[2] * width)
					h = int(detection[3] * height)
					return (True, h, w)
		return (False, -999, -999)

class Manager:
	def __init__(self):
		self.bridge = CvBridge()
		
		self.person_exists = False
		self.height = None
		self.width = None
		self.target = self.height*self.width # to be filled with a bounding box size
		self.cur_bb_size = self.target # also bounding box size. This one will be updated 
		self.prev_err = 0
		self.cum_err = 0
		self.Kp = 0
		self.Ki = 0
		self.Kd = 0
		self.cur_dir = 1 # Forward: 1, Reverse: 0
  
		self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
		self.brake = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size = 1)
		self.accelerate = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 1)
		self.shift = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size = 1)
		
  

	
	def callback(self, image):
		cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
		self.detector = Detector()
		self.person_exists, self.height, self.width = self.detector.detect(cvimage)
		self.curr_bb_size = self.height * self.width
		print(self.person_exists)

		# if not person_exists:
		# 	self.brake.publish(f64_cmd = 0.0)
		# 	self.accelerate.publish(f64_cmd = 0.31)
		# else:
		# 	self.accelerate.publish(f64_cmd = 0.0)
		# 	self.brake.publish(f64_cmd = 0.4)
		
	def run(self):
		while 1:
			bb_size = None # to be updated with bb_size from detector
			err = self.target - self.cur_bb_size
			update = self.Kp*err + self.Ki*self.cum_err + self.Kd*(err - self.prev_err)
			direction = update >= 0

			if direction != 0:
				self.shift.publish(f64_cmd = 1)
			else:
				self.shift.pusblish(f64_cmd = 0)

			if update > 0.5:
				update = 0.5

			self.accelerate.publish(f64_cmd = abs(update))

			self.prev_err = err
			self.cum_err += err




		# while not self.person_exists:
		# 	self.brake.publish(f64_cmd = 0.0)
		# 	self.accelerate.publish(f64_cmd = 0.31)
			
		# self.accelerate.publish(f64_cmd = 0.0)
		# self.brake.publish(f64_cmd = 0.4)

if __name__ == '__main__':
	rospy.init_node('braking_node', anonymous=True)
	node = Manager()
	node.run()
