#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/13/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
from sklearn.cluster import KMeans

# ROS Headers
import alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import pandas as pd
import numpy as np
import copy
import time
import matplotlib.pyplot as plt

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


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(10)

        self.look_ahead = 4
        self.wheelbase  = 1.75 # meters
        self.offset     = 0.46 # meters

        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 0.0

        self.olat       = 40.0928563
        self.olon       = -88.2359994

        # read waypoints into the system 
        self.pointcloud_data = PC_Manip()
        self.env_points = self.pointcloud_data.get_env()
        self.goal       = 0            
        self.get_waypoints()

        self.midpoint_reached = False

        self.desired_speed = 0.55  # m/s, reference speed
        self.max_accel     = 0.4 # % of acceleration
        self.pid_speed     = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second
        


    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees

    def speed_callback(self, msg):
        self.speed = round(msg.data, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        if (heading_curr >= 0 and heading_curr < 90):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 90 and heading_curr < 180):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 180 and heading_curr < 270):
            yaw_curr = np.radians(90 - heading_curr)
        else:
            yaw_curr = np.radians(450 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):

        if(f_angle > 35):
            f_angle = 35

        if (f_angle < -35):
            f_angle = -35

        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)

        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0

        return steer_angle

    def reset_origin(self):
        self.olat = self.lat
        self.olon = self.lon

    # def track2midpoint(box1_loc, box2_loc, gem_startloc, num_points=4):
    #     goal = ((box1_loc[0]+box2_loc[0])/2, (box1_loc[1]+box2_loc[1])/2) #midpoint of the two boxes
    #     print(gem_startloc[0])
    #     print(goal)
    #     print( int(goal[0]-gem_startloc[0])//num_points)
    #     track_points_x = np.linspace(gem_startloc[0], goal[0], num_points)
    #     track_points_y = np.linspace(gem_startloc[1], goal[1], num_points)
    #     # track_points_x = np.linspace(gem_startloc[0], goal[0], (goal[0]-gem_startloc[0])/num_points)
    #     # track_points_y = np.linspace(gem_startloc[1], goal[1], (goal[1]-gem_startloc[1])/num_points)
    #     theta = math.atan(goal[0] / goal[1])
    #     track_points_heading = [theta+90 for i in range(len(track_points_x))]        
    #     return track_points_x, track_points_y, track_points_heading

    # def circlepoints(circle_center, gem_startloc, num_points=20):
    #     r = math.sqrt((gem_startloc[0]-circle_center[0])**2 + (gem_startloc[1]-circle_center[1])**2)
    #     print((circle_center[0]/r))
    #     print(np.degrees(math.acos(((gem_startloc[0]-circle_center[0])/r))))
    #     starting_t = np.degrees(math.acos((gem_startloc[0] - circle_center[0])/r))
    #     if gem_startloc[1] < circle_center[1]:
    #         starting_t *= -1
    #     angles = np.linspace(starting_t, starting_t+360, num_points)
    #     circle_points_x = []
    #     circle_points_y = []
    #     circle_points_heading = []
        
    #     for i in range(num_points):
    #         circle_points_x = np.append(circle_points_x, r*np.cos(np.radians(angles[i]))+circle_center[0])
    #         circle_points_y = np.append(circle_points_y, r*np.sin(np.radians(angles[i]))+circle_center[1])
    #         circle_points_heading = np.append(circle_points_heading, angles[i]+90)
    
    #     return circle_points_x, circle_points_y, circle_points_heading
    
    def track2midpoint(self, box1_loc, box2_loc, gem_startloc, num_points=4):
        goal = ((box1_loc[0]+box2_loc[0])/2, (box1_loc[1]+box2_loc[1])/2) #midpoint of the two boxes

        track_points_x = np.linspace(gem_startloc[0], goal[0], num_points)
        track_points_y = np.linspace(gem_startloc[1], goal[1], num_points)

        theta = math.atan(((goal[1]-gem_startloc[1]) / (goal[0]-gem_startloc[0])))

        track_points_heading = [180-(np.degrees(theta)+90) for i in range(len(track_points_x))] #adding 90 so that positive x direction is 90deg (normally positive x direction would be 0deg)
        if gem_startloc[0] > goal[0]:
            track_points_heading = [x + 180 for x in track_points_heading] # accounting for wrapping that results in heading flip at 180deg
        return track_points_x, track_points_y, track_points_heading

    def circlepoints(self, circle_center, gem_startloc, perimeter_point, num_points=20):
        r = math.sqrt((perimeter_point[0]-circle_center[0])**2 + (perimeter_point[1]-circle_center[1])**2)
        starting_t = np.degrees(math.acos((perimeter_point[0] - circle_center[0])/r))
        if perimeter_point[1] < circle_center[1]:
            starting_t *= -1 # accounting for unwrapping that happens at 180deg (aka on the bottom half of a circle)
        
        # Assigning direction, Counterclockwise == 1, clockwise == 0
        if gem_startloc[0] < circle_center[0]:
            if perimeter_point[1] > circle_center[1]:
                direction = 0
            else:
                direction = 1
        else:
            if perimeter_point[1] > circle_center[1]:
                direction = 1
            else:
                direction = 0
        
        if direction:
            angles = np.linspace(starting_t, starting_t+360, num_points)
        else:
            angles = np.linspace(starting_t, starting_t-360, num_points)
        
        circle_points_x = []
        circle_points_y = []
        circle_points_heading = []
        for i in range(num_points):
            circle_points_x = np.append(circle_points_x, r*np.cos(np.radians(angles[i]))+circle_center[0])
            circle_points_y = np.append(circle_points_y, r*np.sin(np.radians(angles[i]))+circle_center[1])

            # +90 for forward x direction being 90, plus another 90 to calculated the tangent
            circle_points_heading = np.append(circle_points_heading, (angles[i]+(direction*180))) 
        
        for i in range(len(circle_points_heading)):
            if circle_points_heading[i] > 360:
                circle_points_heading[i] -= 360
            elif circle_points_heading[i] < 0:
                circle_points_heading[i] += 360

        return circle_points_x, circle_points_y, circle_points_heading
        
    def get_waypoints(self):
        self.reset_origin()
        curr_loc = self.get_gem_state()
        point1 = self.env_points[0]
        for i in range(len(self.env_points)):
            dist = math.sqrt(((self.env_points[i][0] - point1[0])**2 + (self.env_points[i][1] - point1[1])**2))
            if dist > 1:
                 point2 = self.env_points[i]
                 break
        self.path_points_lon_x, self.path_points_lat_y, self.path_points_heading = self.track2midpoint(point1, point2, (curr_loc[0], curr_loc[1]), 20)
        self.circle_points_x, self.circle_points_y, self.circle_points_heading = self.circlepoints(self.env_points[0], (curr_loc[0], curr_loc[1]), (self.path_points_lon_x[-1], self.path_points_lat_y[-1]), 20)
        # self.path_points_lon_x = np.append(self.path_points_lon_x ,circle_points_x)
        # self.path_points_lat_y = np.append(self.path_points_lat_y, circle_points_y)
        # self.path_points_heading = np.append(self.path_points_heading, circle_points_heading)
        plt.plot(self.path_points_lon_x, self.path_points_lat_y)
        plt.plot(self.circle_points_x, self.circle_points_y)
        self.wp_size             = len(self.path_points_lon_x)
        self.dist_arr            = np.zeros(self.wp_size)
        plt.show()

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   

    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def start_pp(self):
        
        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True


            

            
            curr_x, curr_y, curr_yaw = self.get_gem_state()
            # f = open("8figure_final.csv",'w')
            # f.write(f'{curr_x}, {curr_y}, {curr_yaw}\n')
            # f.close()
            dist2goal = math.sqrt((curr_x - self.path_points_lon_x[-1])**2 + (curr_y - self.path_points_lat_y[-1])**2)
            
            print('dist2goal: ', dist2goal)

            if dist2goal > 1.5 and not self.midpoint_reached:
                self.path_points_x = np.array(self.path_points_lon_x)
                self.path_points_y = np.array(self.path_points_lat_y)
            else:
                print('midpoint reached')
                self.midpoint_reached = True
                self.path_points_x = np.array(self.circle_points_x)
                self.path_points_y = np.array(self.circle_points_y)
                self.path_points_heading = self.circle_points_heading
                self.wp_size             = len(self.circle_points_x)
                self.dist_arr            = np.zeros(self.wp_size)


            # finding the distance of each way point from the current position
            for i in range(len(self.path_points_x)):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

            # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
            goal_arr = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

            # finding the goal point which is the last in the set of points less than the lookahead distance
            for idx in goal_arr:
                v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                temp_angle = self.find_angle(v1,v2)
                # find correct look-ahead point by using heading information
                if abs(temp_angle) < np.pi:
                    self.goal = idx
                    break

            # finding the distance between the goal point and the vehicle
            # true look-ahead distance between a waypoint and current position
            L = self.dist_arr[self.goal]

            # find the curvature and the angle 
            alpha = self.heading_to_yaw(self.path_points_heading[self.goal]) - curr_yaw

            # ----------------- tuning this part as needed -----------------
            k       = 0.41 
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L) 
            angle   = angle_i*2
            # ----------------- tuning this part as needed -----------------

            f_delta = round(np.clip(angle, -0.61, 0.61), 3)

            f_delta_deg = np.degrees(f_delta)

            # steering_angle in degrees
            steering_angle = self.front2steer(f_delta_deg)

            if(self.gem_enable == True):
                print("Current index: " + str(self.goal))
                print("Forward velocity: " + str(self.speed))
                ct_error = round(np.sin(alpha) * L, 3)
                print("Crosstrack Error: " + str(ct_error))
                print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
                print("Steering wheel angle: " + str(steering_angle) + " degrees" )
                print("Heading angle: ", list(self.path_points_heading))
                print("\n")

            # if (self.goal >= 625 and self.goal <= 940):
            #     self.desired_speed = 1.5
            # else:
            #     self.desired_speed = 0.7

            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)
            output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

            if output_accel > self.max_accel:
                output_accel = self.max_accel

            if output_accel < 0.3:
                output_accel = 0.3

            self.accel_cmd.f64_cmd = output_accel
            self.steer_cmd.angular_position = np.radians(steering_angle)
            self.accel_pub.publish(self.accel_cmd)
            self.steer_pub.publish(self.steer_cmd)

            self.rate.sleep()

class PC_Manip:
        def __init__(self):
                self.lidar_sub = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, self.lidar_callback)
                self.points = []
                self.num_x_cells = 20
                self.num_y_cells = 20
                self.look_radius = 10
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
                #print(df)
                self.points = []
                
                df.columns = ('x','y','z')
                #print(df)

                #clip z co-ordiantes
                df = df[(df['z'] >= - self.z_clip) & (df['z'] <= self.z_clip)]

                #df = df[(df['x'] >= 1) & (df['y'] >= 1) & (df['x'] <= -1) & (df['y'] <= -1)]
                
                # clip points based on look radius
                df = df[(df['x'] <= self.look_radius) & ((df['x'] >= (-1) * self.look_radius)) \
                        & (df['y'] <= self.look_radius) & (df['y'] >= (-1) * self.look_radius)]
                        
                exclude = df[(df['x'] <= 1) & ((df['x'] >= (-1) * 1)) \
                        & (df['y'] <= 1) & (df['y'] >= (-1) * 1)]

                cond = df['y'].isin(exclude['y'])
                df.drop(df[cond].index, inplace = True)
                cond = df['x'].isin(exclude['x'])
                df.drop(df[cond].index, inplace = True)

               
                


                        
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
                print(result.shape)
                result = result.head(10)

                # kmeans = KMeans(n_clusters=2)
                # kmeans.fit(df)
                # centroids = kmeans.cluster_centers_

                # return top 10 most dense grid (mean of x and y of all points in grid)
                
                fig, ax = plt.subplots(figsize=(7,7))


                # Plot the original points
                ax.scatter(df['x'], df['y'], s=2, alpha=0.5)
                #ax.scatter(exclude['x'], exclude['y'], s=2, alpha=0.5)

                # Plot the mean x and y for each grid
                ax.scatter(result['x'], result['y'], s=50, marker='x', color='red')

                # Set the x and y axis labels
                ax.set_xlabel('X Coordinates')
                ax.set_ylabel('Y Coordinates')


                print(result)
                # Show the plot
                # plt.show()
                
                return list(zip(result['x'], result['y']))
                # return centroids

        def get_env(self):
                time.sleep(1)
                return(self.find_squares())

def pure_pursuit():
    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass





if __name__ == '__main__':    
    pure_pursuit()


