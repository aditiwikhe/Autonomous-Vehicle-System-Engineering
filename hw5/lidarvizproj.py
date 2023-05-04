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
import pandas as pd
import numpy as np
import copy
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
                self.lidar_sub = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, self.lidar_callback)
                self.points = []
                self.num_x_cells = 20
                self.num_y_cells = 20
                self.look_radius = 20  
                self.z_clip = 1.5     
                
        def lidar_callback(self, pointcloud):
                # print(ros_numpy.point_cloud2.get_xyz_points(pointcloud))
                # self.xyz = ros_numpy.point_cloud2.get_xyz_points(pointcloud)
                for point in sensor_msgs.point_cloud2.read_points(pointcloud, skip_nans=True):
                        pt_x = point[0]
                        pt_y = point[1]
                        pt_z = point[2]
                        self.points.append([pt_x, pt_y, pt_z])

                        #print(len(self.points))

                        # # self.xyz.append((pt_x, pt_y, pt_z))
                        # f = open('xyz_lidar.csv', 'a')
                        # f.write(f'{pt_x}, {pt_y}, {pt_z}\n')
                        # f.close()
                

        def find_squares(self):
                p = copy.deepcopy(self.points)
                print('len',len(self.points))
                # time.sleep(1)
                #print(self.points)


                df = pd.DataFrame(p)
                print(df)
                self.points = []
                
                df.columns = ('x','y','z')
                print(df)

                # clip z co-ordiantes
                df = df[(df['z'] >= - self.z_clip) & (df['z'] <= self.z_clip)]
                
                # clip points based on look radius
                df = df[(df['x'] <= self.look_radius) & ((df['x'] >= (-1) * self.look_radius)) \
                        & (df['y'] <= self.look_radius) & (df['y'] >= (-1) * self.look_radius)]

                        
                num_grids = 20
                        
                # Calculate the grid size based on the number of desired grids
                grid_size = np.ceil(np.max([df['x'].max(), df['y'].max()]) / num_grids)


                # Create a grid by rounding off the x and y coordinates to the nearest multiple of grid_size
                df['x_grid'] = np.floor(df['x'] / grid_size) * grid_size
                df['y_grid'] = np.floor(df['y'] / grid_size) * grid_size

                # Group the data by grid and count the number of points in each grid
                counts = df.groupby(['x_grid', 'y_grid']).size().reset_index(name='count')

                # Calculate the mean x and y coordinates for each grid
                mean_coords = df.groupby(['x_grid', 'y_grid']).agg({'x': 'mean', 'y': 'mean'}).reset_index()

                # Merge the count and mean coordinates dataframes
                result = pd.merge(counts, mean_coords, on=['x_grid', 'y_grid'])

                # Sort the data by the count of points in each grid in descending order
                result = result.sort_values(by='count', ascending=False)
                result = result.head(10)
                
                # return top 10 most dense grid (mean of x and y of all points in grid)
                return list(zip(result['x'], result['y']))




        def run(self):
                print('here')
                time.sleep(0.2)
                #rostopic pub -l /pacmod/as_rx/turn_cmd pacmod_msgs/PacmodCmd “{header: auto, ui16_cmd: 0}”
                print(self.find_squares())

                while not rospy.is_shutdown():
                        pass
                        #time.sleep(0.1)
                        
                        


if __name__ == '__main__':
        rospy.init_node('sos_node', anonymous=True)
        node = PC_Manip()
        print('after init')
        node.run()
