#! /bin/python3

from atexit import unregister
import pyransac3d as pyrsc
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import os
import message_filters
import copy
from geometry_msgs.msg import PoseStamped
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class cube_detec():
	def __init__(self):

		print("Initialising cube_detec_node")

		self.bridge = CvBridge()
		self.image_queue = [None, None]
		self.image_recieved = False
		self.camera_info_msg = [None, None]

		rospy.init_node('cube_detec_node', anonymous=True)

		self.colour_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
		self.depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw', Image)
		self.colour_info_subOnce = rospy.Subscriber("/camera/rgb/camera_info",CameraInfo, self.camera_info_callback, 0, queue_size=1)
		self.depth_info_subOnce = rospy.Subscriber("/camera/depth_registered/camera_info", CameraInfo, self.camera_info_callback, 1, queue_size=1)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.colour_sub, self.depth_sub], 10, 1)
		self.ts.registerCallback(self.image_callback)

		self.marker_pub = rospy.Publisher('vis_marker',Marker,queue_size=10)
		self.detection_pub = rospy.Publisher('detection',PoseStamped,queue_size=10)
		self.detection_msg = PoseStamped()
		self.detection_msg.header.frame_id = "camera_depth_optical_frame"

		self.cube_pcd_pub = rospy.Publisher('cube_pcd', PointCloud2, queue_size=1)
		

 
		rospy.sleep(2)
		
		print("cube_detec_node ready")
		


	def find_biggest_contour(self, image): #figures out which of the found contours is the biggest
		image = image.copy()
		contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
		biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

		return biggest_contour




	def image_callback(self, img_msg, depth_msg):
		try:
			cv_bgr_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
			cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

			cv_rgb_image = cv2.cvtColor(cv_bgr_image, cv2.COLOR_BGR2RGB)# remove
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

		self.image_queue[0] = cv_rgb_image
		self.image_queue[1] = cv_depth_image

		if self.image_queue[0] is None or self.image_queue[1] is None:
			self.image_recieved = False
			return

		if self.camera_info_msg[0] is None or self.camera_info_msg[1] is None:
			self.image_recieved = False
			return
		self.image_recieved = True





	def camera_info_callback(self,CameraInfo_msg, cameraID):
		if cameraID:
			self.camera_info_msg[1] = CameraInfo_msg
			self.depth_info_subOnce.unregister()
		else:
			self.camera_info_msg[0] = CameraInfo_msg
			self.colour_info_subOnce.unregister()



	def ransac3d(self,pcd):

		points = np.asarray(pcd.points)
		#print("points: ", points.shape,points.size)

		cuboid = pyrsc.Cuboid()
		self.best_eq, self.best_inliers = cuboid.fit(points, thresh=0.001, maxIteration=1000)
		x = 0
		y = 0
		z = 0
		n = 0
		
		new_pcd = o3d.geometry.PointCloud()
		temp_array = []

		for p in self.best_inliers:
			x += points[p][0]
			y += points[p][1]
			z += points[p][2]
			n += 1
			temp_array.append([points[p][0],points[p][1],points[p][2]])

		if n == 0:
			return
		
		new_pcd.points = o3d.utility.Vector3dVector(temp_array)
		x /= n
		y /= n
		z /= n
		self.center_point = [x, y, z]
		
		## visualisation of cropped pcd
		FIELDS_XYZ = [
			PointField(name='x',offset=0, datatype=PointField.FLOAT32, count=1),
			PointField(name='y',offset=4, datatype=PointField.FLOAT32, count=1),
			PointField(name='z',offset=8, datatype=PointField.FLOAT32, count=1)
		]
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "camera_rgb_optical_frame"
		
		self.cube_pcd_pub.publish(pc2.create_cloud(header, FIELDS_XYZ, np.asarray(new_pcd.points)))

	
		#print("best eq",self.best_eq)
		#print("com: %.3f"%x," %.3f" %y," %.3f"%z)
		#print("inliers: ",n)



		return new_pcd




	def deproject_points(self,points):
		K = np.asarray(self.camera_info_msg[1].K).reshape(3,3)
		u = points[:,0]
		v = points[:,1]
		z = points[:,2]
		x_over_z = (u - K[0, 2]) / K[0, 0]
		y_over_z = (v - K[1, 2]) / K[1, 1]
		x = x_over_z * z
		y = y_over_z * z

		return np.array([x,y,z])




	def orientation_calc(self):
			
		plane_normal = R.from_rotvec(self.best_eq[0][0:3])
		quat = plane_normal.as_quat()

		# a = np.asarray(self.best_eq[0:,0:3])

		# b = np.asarray(self.best_eq[0:,3])


		# self.corner = np.linalg.solve(a,b)



		return quat

	
	def display_marker(self):

		marker = Marker()
		marker.header.frame_id = 'camera_rgb_optical_frame'
		marker.header.stamp = rospy.Time.now()
		marker.type = 1

		marker.pose = self.detection_msg.pose

		# marker.pose.position.x = self.corner[0]
		# marker.pose.position.y = self.corner[1]
		# marker.pose.position.z = -self.corner[2]
		# marker.pose.orientation.x = 0
		# marker.pose.orientation.y = 0
		# marker.pose.orientation.z = 0
		# marker.pose.orientation.w = 0


		marker.scale.x = 0.02
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.action = 0
		marker.color.r = 1
		marker.color.g = 0
		marker.color.b = 0
		marker.color.a = 1
		marker.lifetime = rospy.Duration(0)
		# marker.frame_locked = 1
		self.marker_pub.publish(marker)




		return



if __name__ == '__main__':

	cube = cube_detec()

	while not rospy.is_shutdown():
		if cube.image_recieved:
			timer = rospy.Time.now()

			# convert to hsv colorspace
			image_blur = cv2.GaussianBlur(cube.image_queue[0],(7,7), 0)
			hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)


			# lower bound and upper bound for blue color
			lower_bound = np.array([90, 100, 100])
			upper_bound = np.array([140, 255, 255])

			# lower bound and upper bound for skin color
			# lower_bound = np.array([0, 120, 120])
			# upper_bound = np.array([100, 255, 255])

			# lower_bound = np.array([20, 150, 150])
			# upper_bound = np.array([50, 255, 255])


			# find the colors within the boundaries
			mask = cv2.inRange(hsv, lower_bound, upper_bound)

			# define kernel size  
			kernel = np.ones((7,7),np.uint8)
			# Remove unnecessary noise from mask
			mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

			only_blue = cv2.bitwise_and(cube.image_queue[0], cube.image_queue[0], mask=mask)

			
			image = mask.copy()
			contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

			if not len(contour_sizes) > 0:
				print("skip; no contour")
				continue
			biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

			output = cv2.drawContours(only_blue, biggest_contour, -1, (0, 0, 255), 3)

			output2 = cv2.drawContours(cube.image_queue[0], biggest_contour, -1, (0, 0, 255), 3)

			x,y,w,h = cv2.boundingRect(biggest_contour)
			output = cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)





			#find center
			M = cv2.moments(biggest_contour)
			center = int(M["m10"]/M["m00"]), (int(M["m01"]/M["m00"]))

			output2 = cv2.circle(output2,center,1,[255,0,255],2)
			cv2.imshow("biggest segment and center",output)



			# new_biggest = []
			# for i in biggest_contour:
			# 	x = int(i[0][0])
			# 	y = int(i[0][1])
			# 	new_biggest.append([x,y])


			# sorted_big = sorted(copy.deepcopy(new_biggest),key=lambda l:l[0])

			# some_edge = 20

			# small_x = sorted_big[0][0] - some_edge
			# big_x =sorted_big[len(sorted_big)-1][0] + some_edge


			# sorted_big = sorted(copy.deepcopy(new_biggest),key=lambda l:l[1])

			# small_y = sorted_big[0][1] - some_edge
			# big_y = sorted_big[len(sorted_big)-1][1] + some_edge

			# cropped_d_image = cube.image_queue[1][small_y:big_y, small_x:big_x]
			# cropped_c_image = cube.image_queue[0][small_y:big_y, small_x:big_x]

			# if small_y > big_y or small_x > big_x or not cropped_d_image.shape[0] > 0 or not cropped_d_image.shape[1] > 0 :
			# 	print("nope2: ", cropped_d_image.shape)
			# 	print([small_y,big_y],[small_x,big_x])
			# 	cv2.waitKey(5000)
			# 	continue

			# print("nope2: ", cropped_d_image.shape)
			# print([small_y,big_y],[small_x,big_x])


			#cv2.imshow("cropped colour",cropped_c_image)
			# cv2.imshow("cropped depth",cropped_d_image)
			
			rgb_img = o3d.geometry.Image(cube.image_queue[0][y:y+h, x:x+w])
			d_img = o3d.geometry.Image(cube.image_queue[1][y:y+h, x:x+w].astype(np.float32))

			
	
			rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, d_img, depth_scale=1)


			K = np.asarray(cube.camera_info_msg[1].K).reshape(3,3)

			newcameraintrensics = o3d.camera.PinholeCameraIntrinsic()

			#newcameraintrensics.set_intrinsics(cube.camera_info_msg[1].width, cube.camera_info_msg[1].height,K[0,0],K[1,1],K[0,2],K[1,2])
			newcameraintrensics.set_intrinsics(w, h, K[0,0],K[1,1],K[0,2]-x,K[1,2]-y)

			pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,newcameraintrensics)

			# Flip it, otherwise the pointcloud will be upside down
			#pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


			if len(pcd.points) < 10:
				print("skip; no pointcloud")
				continue
			

			new_pcd = cube.ransac3d(pcd)

			quat = cube.orientation_calc()

			cube.detection_msg.pose.position.x = cube.center_point[0]
			cube.detection_msg.pose.position.y = cube.center_point[1]
			cube.detection_msg.pose.position.z = cube.center_point[2]
			cube.detection_msg.pose.orientation.x = quat[0]
			cube.detection_msg.pose.orientation.y = quat[1]
			cube.detection_msg.pose.orientation.z = quat[2]
			cube.detection_msg.pose.orientation.w = quat[3]
			cube.detection_msg.header.stamp = rospy.Time.now()
			print(cube.detection_msg)
			cube.detection_pub.publish(cube.detection_msg)

			cube.display_marker()
			print("Duration: %.4f"% (rospy.Time.now()-timer).to_sec(),"secs")


			cv2.waitKey(50)
			

			#o3d.visualization.draw_geometries([o3d.geometry.TriangleMesh.create_coordinate_frame(),new_pcd])

			
			








