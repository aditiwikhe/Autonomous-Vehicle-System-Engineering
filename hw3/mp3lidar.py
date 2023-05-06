
import rospy
import statistics
import time
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Manager:
	def __init__(self):
        self.plot = True
		#self.bridge = CvBridge()
		self.points = []
		#self.person_exists = False
		#self.height = 275
        self.x_dist = 7
		#self.width = None
		self.target = self.x_dist # to be filled with a bounding box size
		self.cur_bb_size = self.target # also bounding box size. This one will be updated
		self.prev_bb_size = None
		self.prev_err = 0
		self.cum_err = 0
		self.Kp = 0.0035
		self.Ki = 0
		self.Kd = 0
		self.cur_dir = 1 # Forward: 1, Reverse: 0

		self.height_array = []
		self.err_array = []
		self.update_array = []
		self.prev_update = 0
  
		#self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo_raw/image_raw_color', Image, self.callback)
        self.lidar_sub = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, self.lidar_callback)
		self.brake = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size = 1)
		self.accelerate = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 1)
		self.shift = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size = 1)
		
        def lidar_callback(self, pointcloud):
                # print(ros_numpy.point_cloud2.get_xyz_points(pointcloud))
                # self.xyz = ros_numpy.point_cloud2.get_xyz_points(pointcloud)
                for point in sensor_msgs.point_cloud2.read_points(pointcloud, skip_nans=True):
                        pt_x = point[0]
                        pt_y = point[1]
                        pt_z = point[2]
                        self.points.append([pt_x, pt_y, pt_z])

                        # # self.xyz.append((pt_x, pt_y, pt_z))
                        # f = open('xyz_lidar.csv', 'a')
                        # f.write(f'{pt_x}, {pt_y}, {pt_z}\n')
                        # f.close()
                        
        def process_points(self):
            data = np.array(self.)
            # Step 2: Define the four corner points of the rectangle
            corner_points = np.array([[1, -1.5], [10, -1.5], [10, 1.5], [1, 1.5]])

            # Step 3: Filter the points that lie within the rectangle using NumPy boolean indexing
            mask = np.logical_and.reduce((
                data[:, 0] >= corner_points[0, 0],
                data[:, 0] <= corner_points[1, 0],
                data[:, 1] >= corner_points[0, 1],
                data[:, 1] <= corner_points[2, 1]
            ))
            r_points = data[mask]

            print(r_points.shape)
            mean_x, mean_y = r_points[:, 0].mean(), r_points[:, 1].mean()
            print(mean_x, mean_y)
            
            # Step 4: Plot the points and the rectangle using Matplotlib
            if self.plot == True:
                fig, ax = plt.subplots()
                ax.scatter(data[:, 0], data[:, 1],s = 0.1,label = "All points")
                ax.scatter(r_points[:, 0], r_points[:, 1], s = 0.1,label = "Points inside rectangle")
                ax.plot(corner_points[:, 0], corner_points[:, 1], "--", c='r', label = "Rectangle")
                plt.show()
                ax.legend()
                self.plot = False
            
            self.cur_bb_size = self.mean_x

		# if not person_exists:
		# 	self.brake.publish(f64_cmd = 0.0)
		# 	self.accelerate.publish(f64_cmd = 0.31)
		# else:
		# 	self.accelerate.publish(f64_cmd = 0.0)
		# 	self.brake.publish(f64_cmd = 0.4)
		
	def run(self):
		start = time.time()
		while 1:
		# while time.time() - start < 10:
			if not self.cur_bb_size:
				self.cur_bb_size = self.prev_bb_size
			err = self.target - self.cur_bb_size
			print(self.cur_bb_size)
			if self.cur_bb_size:
				self.height_array.append(self.cur_bb_size)
			print(f'error: {err}')
			if err:
				self.err_array.append(err)
			update = self.Kp*err + self.Ki*self.cum_err + self.Kd*(err - self.prev_err)
			direction = update >= 0
			update = abs(update)
			print(direction)

			if direction == True:
				self.shift.publish(ui16_cmd = 3)
			else:
				self.shift.publish(ui16_cmd = 1)

			if update > 0.5:
				update = 0.5

			if update < self.prev_update:
				self.accelerate.publish(f64_cmd = 0)
				self.brake.publish(f64_cm = self.prev_update - update)
			else:
				self.accelerate.publish(f64_cmd = abs(update))

			print(f'update: {update}')
			if update:
				self.update_array.append(update)

			self.prev_err = err
			self.cum_err += err
			if self.cur_bb_size:
				self.prev_bb_size = self.cur_bb_size

		print(f'{max(self.height_array)} {min(self.height_array)}')
		print(f'{max(self.err_array)} {min(self.err_array)}')
		print(f'{max(self.update_array)} {min(self.update_array)}')		


		# while not self.person_exists:
		# 	self.brake.publish(f64_cmd = 0.0)
		# 	self.accelerate.publish(f64_cmd = 0.31)
			
		# self.accelerate.publish(f64_cmd = 0.0)
		# self.brake.publish(f64_cmd = 0.4)

if __name__ == '__main__':
	rospy.init_node('braking_node', anonymous=True)
	node = Manager()
	node.run()
