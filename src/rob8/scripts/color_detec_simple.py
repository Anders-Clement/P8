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

class detect_detec():
	def __init__(self):

		print("Initialising detect_detec_node")

		self.bridge = CvBridge()
		self.image_queue = [None, None]
		self.image_recieved = False
		self.camera_info_msg = [None, None]
		self.running_average = np.zeros([10,3])
		self.x_test_pose = [320-5,320-5,320,320+5,320+5]
		self.y_test_pose = [240-5,240+5,240,240-5,240+5]
		self.z_test_pose = [0.5,0.5,0.5,0.5,0.5]

		rospy.init_node('detect_detec_node', anonymous=True)

		self.colour_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
		self.depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw', Image)
		self.colour_info_subOnce = rospy.Subscriber("/camera/rgb/camera_info",CameraInfo, self.camera_info_callback, 0, queue_size=1)
		self.depth_info_subOnce = rospy.Subscriber("/camera/depth_registered/camera_info", CameraInfo, self.camera_info_callback, 1, queue_size=1)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.colour_sub, self.depth_sub], 10, 1)
		self.ts.registerCallback(self.image_callback)

		self.marker_pub = rospy.Publisher('vis_marker',Marker,queue_size=10)
		self.detection_pub = rospy.Publisher('detection',PoseStamped,queue_size=10)
		self.detection_msg = PoseStamped()
		self.detection_msg.header.frame_id = "camera_rgb_optical_frame"

		self.detect_pcd_pub = rospy.Publisher('detect_pcd', PointCloud2, queue_size=1)
		

 
		rospy.sleep(2)
		
		print("detect_detec_node ready")
		


	def find_biggest_contour(self, image): #figures out which of the found contours is the biggest
		image = image.copy()
		contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
		biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

		print(biggest_contour.shape)

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



	def deproject_points(self,point):
		K = np.asarray(self.camera_info_msg[1].K).reshape(3,3)
		u = point[0]
		v = point[1]
		z = point[2]
		x_over_z = (u - K[0, 2]) / K[0, 0]
		y_over_z = (v - K[1, 2]) / K[1, 1]
		x = x_over_z * z
		y = y_over_z * z

		return np.array([x,y,z])


	
	def display_marker(self):

		marker = Marker()
		marker.header.frame_id = 'camera_rgb_optical_frame'
		marker.header.stamp = rospy.Time.now()
		marker.type = 0

		marker.pose = self.detection_msg.pose

		# marker.pose.position.x = self.x_test_pose[id]
		# marker.pose.position.y = self.y_test_pose[id]
		# marker.pose.position.z = self.z_test_pose[id]
		# marker.pose.orientation.x = 0
		# marker.pose.orientation.y = 0
		# marker.pose.orientation.z = 0
		# marker.pose.orientation.w = 1


		marker.scale.x = 0.02
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.action = 0
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 0
		marker.color.a = 1
		marker.lifetime = rospy.Duration(0)
		# marker.frame_locked = 1
		self.marker_pub.publish(marker)




		return



if __name__ == '__main__':

	detect = detect_detec()

	while not rospy.is_shutdown():
		if detect.image_recieved:
			timer = rospy.Time.now()

			# convert to hsv colorspace
			image_blur = cv2.GaussianBlur(detect.image_queue[0],(7,7), 0)
			hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)


			# lower bound and upper bound for blue color
			lower_bound = np.array([90, 100, 100])
			upper_bound = np.array([140, 255, 255])


			# lower bound and upper bound for skin color
			# lower_bound = np.array([0, 120, 120])
			# upper_bound = np.array([100, 255, 255])


			# lower bound and upper bound for yellow color
			# lower_bound = np.array([20, 100, 100])
			# upper_bound = np.array([40, 255, 255])


			# find the colors within the boundaries
			mask = cv2.inRange(hsv, lower_bound, upper_bound)

			# define kernel size  
			kernel = np.ones((7,7),np.uint8)
			# Remove unnecessary noise from mask
			mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

			only_blue = cv2.bitwise_and(detect.image_queue[0], detect.image_queue[0], mask=mask)

			
			image = mask.copy()
			contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

			if not len(contour_sizes) > 0:
				print("no contours found")
				continue
			biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

			#find center
			M = cv2.moments(biggest_contour)
			center = int(M["m10"]/M["m00"]), (int(M["m01"]/M["m00"]))

			# shows the segmented and what segment is biggest
			output = cv2.drawContours(only_blue, biggest_contour, -1, (0, 0, 255), 3)

			# shows the input image and what segment is biggest
			output2 = cv2.drawContours(detect.image_queue[0], biggest_contour, -1, (0, 0, 255), 3)
			output2 = cv2.circle(output2, center, 1, (255, 0, 255), 2)
			# temp = cv2.circle(detect.image_queue[1], (5,5), 1, (255, 0, 255), 2)
			# temp = cv2.circle(temp, center, 1, (255, 0, 255), 2)

			#cv2.imshow("all segments",output)
			cv2.imshow("biggest contour",output2)
			# cv2.imshow("depth img",temp)


			cv2.waitKey(1)

		
			rgb_img = o3d.geometry.Image(detect.image_queue[0])
			d_img = o3d.geometry.Image(detect.image_queue[1].astype(np.float32))

			
	
			rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, d_img, depth_scale=1)

			
			center_point_3d = [center[0],center[1],np.asarray(rgbd_image.depth)[center[1],center[0]]]

			

			if np.isnan(center_point_3d[0]) or np.isnan(center_point_3d[1]) or np.isnan(center_point_3d[2]):
				print("unusable 3d point: ",center_point_3d) #camera coordinates and not xyz
				continue

			deprojected = detect.deproject_points(center_point_3d)

			detect.running_average = np.append(detect.running_average,[deprojected],axis=0)

			detect.running_average = np.delete(detect.running_average,0,0)
		

			if np.count_nonzero(detect.running_average) < 29:
				continue
			#print(detect.running_average)

			x_poses = detect.running_average[:,0]
			y_poses = detect.running_average[:,1]
			z_poses = detect.running_average[:,2]

			x_poses = np.sort(x_poses)
			y_poses = np.sort(y_poses)
			z_poses = np.sort(z_poses)


			detect.detection_msg.pose.position.x = x_poses[4]
			detect.detection_msg.pose.position.y = y_poses[4]
			detect.detection_msg.pose.position.z = z_poses[4]
			detect.detection_msg.pose.orientation.x = 0
			detect.detection_msg.pose.orientation.y = 0
			detect.detection_msg.pose.orientation.z = 0.5
			detect.detection_msg.pose.orientation.w = -0.5
			detect.detection_msg.header.stamp = rospy.Time.now()
			print(detect.detection_msg)
			detect.detection_pub.publish(detect.detection_msg)

			
			

			detect.display_marker()
			print("Duration: %.4f"% (rospy.Time.now()-timer).to_sec(),"secs")

			# for i in range(5):
			# 	detect.display_marker(i)



			
			








