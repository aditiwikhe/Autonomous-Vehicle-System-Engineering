
import rospy
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import statistics
from collections import deque
import scipy.signal as signal

class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted

class Detector:
    
    def __init__():
        self.smooth = deque()
        self.smooth_factor = 4
    
    def detect(self, frame):
        # classes = []
#         with open('coco.names', 'r') as f:
#             classes = [line.strip() for line in f.readlines()]
        
        blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)
        net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        
        net.setInput(blob)
        outs = net.forward(output_layers)

        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.3 and class_id == class_id == 11: # 0 corresponds to person class in COCO dataset
                    center_x = int(detection[0] * frame.shape[1])
                    center_y = int(detection[1] * frame.shape[0])
                    width = int(detection[2] * frame.shape[1])
                    height = int(detection[3] * frame.shape[0])
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    if len(self.smooth) < self.smooth_factor:
                        self.smooth.append(height)
                    else:
                        self.smooth.popleft()
                        self.smooth.append(height)
                    print(self.smooth)
                    return statistics.mean(self.smooth)
        return (None)

class Manager:
    def __init__(self):
        self.bridge = CvBridge()
        self.person_exists = False
        self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.braked = False
        
        self.desired_speed = 0.3  # m/s, reference speed
        self.max_accel     = 0.4 # % of acceleration
        self.pid_speed     = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)
        
        

    
    def callback(self, image):
        cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.detector = Detector()
        self.sign_height = self.detector.detect(cvimage)
        print(self.sign_height)

        
    def run(self):
        
        while 1:
            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)
            output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

            if output_accel > self.max_accel:
                output_accel = self.max_accel

            if output_accel < 0.3:
                output_accel = 0.3

            self.accel_cmd.f64_cmd = output_accel
            #self.steer_cmd.angular_position = np.radians(steering_angle)
            self.accel_pub.publish(self.accel_cmd)
            #self.steer_pub.publish(self.steer_cmd)
            
            if self.sign_height <= 80 and self.braked == False:
                print('Sign detected, sleeping 5 sec')
                time.sleep(5)
                self.brake_pub.publish(self.brake_pub)
                self.braked = True
            
            
            
        # while not self.person_exists:
#             self.brake.publish(f64_cmd = 0.0)
#             self.accelerate.publish(f64_cmd = 0.31)
#
#         self.accelerate.publish(f64_cmd = 0.0)
#         self.brake.publish(f64_cmd = 0.4)

if __name__ == '__main__':
    rospy.init_node('braking_node', anonymous=True)
    node = Manager()
    # node.run()
