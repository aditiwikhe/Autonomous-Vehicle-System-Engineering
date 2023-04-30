#!/usr/bin/env python3


#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================

#Python Headers
import math
import os
import time


# ROS Headers
import rospy


# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import pandas
# import ros_numpy


# class Node():


#         def _init_(self):


#                 self.rate = rospy.Rate(2)
#                 self.rate_loop_over = rospy.Rate(5)
#                 # pacmod_msgs/PacmodCmd || as_rx/turn_cmd
#                 #Commands the turn signal subsystem to transition to a given state [enum].


#                 self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)
#                 self.sos_signal = [2,2,2,0,0,0,2,2,2]
#                 #self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size = 1)
#                 #self.steer_cmd = PositionWithSpeed()
#                 #self.steer_cmd.angular_position = 0.75 # radians, -: clockwise, +: counter-clockwise
#                 #self.steer_cmd.angular_velocity_limit = 2.0 # radians / second


#         def run(self):
#                 i = 0
                
#                 #rostopic pub -l /pacmod/as_rx/turn_cmd pacmod_msgs/PacmodCmd “{header: auto, ui16_cmd: 0}”
#                 while not rospy.is_shutdown():
#                         if i == 9:
#                                 self.rate_loop_over.sleep()
#                                 i = 0
#                         self.turn_pub.publish(ui16_cmd = self.sos_signal[i])
#                         i += 1
#                         self.rate.sleep()

class PC_Manip:
        def __init__(self):
                self.xyz = []
                self.lidar_sub = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, self.lidar_callback)
                
        def lidar_callback(self, pointcloud):
                # print(ros_numpy.point_cloud2.get_xyz_points(pointcloud))
                # self.xyz = ros_numpy.point_cloud2.get_xyz_points(pointcloud)
                for point in sensor_msgs.point_cloud2.read_points(pointcloud, skip_nans=True):
                        pt_x = point[0]
                        pt_y = point[1]
                        pt_z = point[2]
                        # self.xyz.append((pt_x, pt_y, pt_z))
                        f = open('xyz_lidar.csv', 'a')
                        f.write(f'{pt_x}, {pt_y}, {pt_z}\n')
                        f.close()



        def run(self):
                i = 0
                print('here')
                #rostopic pub -l /pacmod/as_rx/turn_cmd pacmod_msgs/PacmodCmd “{header: auto, ui16_cmd: 0}”
                while i<1:
                        time.sleep(1)
                        i+=1


if __name__ == '__main__':
        rospy.init_node('sos_node', anonymous=True)
        node = PC_Manip()
        print('after init')
        node.run()