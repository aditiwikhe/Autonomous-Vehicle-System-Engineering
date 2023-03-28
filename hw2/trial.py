#!/usr/bin/env python3

#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================
from __future__ import print_function
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#Python Headers
import math
import os

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():

    def __init__(self):

        self.rate = rospy.Rate(2)
        self.bridge = CvBridge()
        self.rate_loop_over = rospy.Rate(5)
        # pacmod_msgs/PacmodCmd || as_rx/turn_cmd
        #Commands the turn signal subsystem to transition to a given state [enum].

        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)
        self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
        self.sos_signal = [2,2,2,0,0,0,2,2,2]
        #self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size = 1)
        #self.steer_cmd = PositionWithSpeed()
        #self.steer_cmd.angular_position = 0.75 # radians, -: clockwise, +: counter-clockwise
        #self.steer_cmd.angular_velocity_limit = 2.0 # radians / second

    def run(self):
        i = 0
        
        #rostopic pub -l /pacmod/as_rx/turn_cmd pacmod_msgs/PacmodCmd “{header: auto, ui16_cmd: 0}”
        while not rospy.is_shutdown():
            if i == 9:
                self.rate_loop_over.sleep()
                i = 0
            self.turn_pub.publish(ui16_cmd = self.sos_signal[i])
            i += 1
            self.rate.sleep()
            #print(self.image_sub)
    def callback(self, image):
        cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
        print(type(cvimage))


if __name__ == '__main__':
    rospy.init_node('sos_node', anonymous=True)
    node = Node()
    node.run()
