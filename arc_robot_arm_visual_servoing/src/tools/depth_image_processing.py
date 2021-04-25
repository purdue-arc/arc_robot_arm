#!/usr/bin/env python3

import rospy

import sys
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from tf import TransformListener
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs2
from color_detection import get_bbox_color   

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class DepthImageProcessing:
	'''DepthImageProcessing
	Processes depth image and returns target object coordinates
	'''

	def __init__(self):
		image_sub_topic = rospy.get_param("image_topic")
		depth_sub_topic = image_sub_topic.replace("color", "depth") 
		confidence_sub_topic = image_sub_topic.replace("color", "confidence") 
		depth_info_sub_topic = depth_sub_topic.replace("image_raw", "camera_info") 

		self.bridge = CvBridge()
		self.frame = None 
		self.object_center = None 
		self.target_coord_base = None 
		self.intrinsics = None
		self.pix_grade = None
		self.tf_listener = TransformListener()

		self.image_pub = rospy.Publisher("visual_servoing_image", Image, queue_size=1000)
		self.image_sub = rospy.Subscriber(image_sub_topic, Image, self.image_callback)
		self.depth_image_sub = rospy.Subscriber(depth_sub_topic, Image, self.image_depth_callback)
		self.depth_confidence_sub = rospy.Subscriber(confidence_sub_topic, Image, self.depth_confidence_callback)
		self.depth_info_sub = rospy.Subscriber(depth_info_sub_topic, CameraInfo, self.image_depth_info_callback)

	def image_depth_callback(self, data):
		'''ROS callback for image_depth
		
		arguments:	
		data -- message of type sensor_msgs.image (depth)
		'''
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
			line = '\r'

			if self.intrinsics and self.object_center is not None:
				target_pix = [self.object_center[1],self.object_center[0]] # convert (x, y) to (row, col) for indexing
				depth = cv_image[target_pix[1], target_pix[0]]
				target_coord_camera = rs2.rs2_deproject_pixel_to_point(self.intrinsics, target_pix, depth) 
				target_coord_camera = [ dim / 1000 for dim in target_coord_camera] # convert to meters
				line += '  Coordinate: %8.2f %8.2f %8.2f.' % (target_coord_camera[0], target_coord_camera[1], target_coord_camera[2])
				self.target_coord_base = self.camera_to_base_coords(target_coord_camera) #  	
			if (self.pix_grade is not None):
				line += ' Grade: %2d' % self.pix_grade

			if(line):
				line += '\r'
				sys.stdout.write(line)
				sys.stdout.flush()
		except CvBridgeError as e:
			print(e)
			return
		except ValueError as e:
			return

	def depth_confidence_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

			grades = np.bitwise_and(cv_image >> 4, 0x0f)

			if (self.object_center is not None):
				target_pix = [self.object_center[0],self.object_center[1]] # convert (x, y) to (row, col) for indexing
				self.pix_grade = grades[target_pix[0],target_pix[1]] 
		except CvBridgeError as e:
			print(e)
			return

	def image_depth_info_callback(self, camera_info):
		'''ROS Callback for image_depth_info
		
		Arguments:	
		camera_info -- message of type sensor_msgs.CameraInfo
		'''
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs2.intrinsics()
			self.intrinsics.width = camera_info.width
			self.intrinsics.height = camera_info.height
			self.intrinsics.ppx = camera_info.K[2]
			self.intrinsics.ppy = camera_info.K[5]
			self.intrinsics.fx = camera_info.K[0]
			self.intrinsics.fy = camera_info.K[4]
			if camera_info.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs2.distortion.brown_conrady
			elif camera_info.distortion_model == 'equidistant':
				self.intrinsics.model = rs2.distortion.kannala_brandt4
			print(self.intrinsics)
			self.intrinsics.coeffs = [i for i in camera_info.D]
		except CvBridgeError as e:
			print(e)
			return

	def image_callback(self, image_data):
		'''ROS Callback for image_topic
		
		Arguments:	
		image_data -- message of type sensor_msgs.Image
		'''
		try:
			self.frame = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

			# Color range for white chess piece (lower, upper) 
			chess_white = (np.array([15, 61, 33]), np.array([21, 139, 250]))

			bbox = get_bbox_color(self.frame, chess_white);
			if(bbox is not None): 
				height,width,dims = self.frame.shape
				self.frame = cv2.drawMarker(self.frame, (round(width/2),round(height/2)), (0,0,255), 
							markerSize=30, thickness=2, line_type=cv2.MARKER_CROSS)
				x, y, width, height = bbox
				self.object_center = (round(x + width/2), round(y + height/2)) # (x, y) where (0,0) is top left corner
				cv2.rectangle(self.frame, (x, y), (x + width, y + height), (255,0,0), 2)
			else:
				self.object_center = None

			self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
			
		except CvBridgeError as e:
			print(e)

	def camera_to_base_coords(self, pose):
		camera_pose = PoseStamped()
		camera_pose.header.frame_id = "camera_link"
		camera_pose.pose.position.x = pose[2]
		camera_pose.pose.position.y = pose[0]
		camera_pose.pose.position.z = pose[1]
		camera_pose.pose.orientation.w = 1.0    # Neutral orientation
		world_pose = self.tf_listener.transformPose("/base_link", camera_pose)
		return world_pose

	def get_target_coord(self):
		return self.target_coord_base
