
import rospy
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import statistics

class Detector:
    def detect(self, frame):
        classes = []
        with open('coco.names', 'r') as f:
            classes = [line.strip() for line in f.readlines()]
        
        blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)
        net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        
        net.setInput(blob)
        outs = net.forward(output_layers)

        class_ids = []
        confidences = []
        boxes = []

        smooth = 5
        smooth_arr = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == classes[class_id] == 'stop sign': # 0 corresponds to person class in COCO dataset
                    center_x = int(detection[0] * frame.shape[1])
                    center_y = int(detection[1] * frame.shape[0])
                    width = int(detection[2] * frame.shape[1])
                    height = int(detection[3] * frame.shape[0])
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    smooth_arr.append(height)
                    if len(smooth_arr) == smooth:
                        return statistics.mean(smooth_arr)
                        smooth_arr = []
        return (-9999)

class Manager:
    def __init__(self):
        self.bridge = CvBridge()
        self.person_exists = False
        self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
        self.brake = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size = 1)
        self.accelerate = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 1)

    
    def callback(self, image):
        cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.detector = Detector()
        self.sign_exists = self.detector.detect(cvimage)
        print(self.sign_exists)

        # if not person_exists:
        # 	self.brake.publish(f64_cmd = 0.0)
        # 	self.accelerate.publish(f64_cmd = 0.31)
        # else:
        # 	self.accelerate.publish(f64_cmd = 0.0)
        # 	self.brake.publish(f64_cmd = 0.4)
        
    # def run(self):
    #     while not self.person_exists:
    #         self.brake.publish(f64_cmd = 0.0)
    #         self.accelerate.publish(f64_cmd = 0.31)
            
    #     self.accelerate.publish(f64_cmd = 0.0)
    #     self.brake.publish(f64_cmd = 0.4)

if __name__ == '__main__':
    rospy.init_node('braking_node', anonymous=True)
    node = Manager()
    # node.run()
