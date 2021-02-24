#!/usr/bin/env python

# Import required Python code.
import cv2
import numpy as np
import math

import roslib
import rospy
import yaml
import tf
import message_filters
from sensor_msgs.msg import JointState, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Header
from hast.srv import servtime, ugvugvsensor
from hast.msg import synctime
from geometry_msgs.msg import Vector3

import csv
import itertools

class vicon_logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		try:
			exp_user = rospy.get_param('/hast/user'); str_user = str(exp_user)
			exp_date = rospy.get_param('/hast/date'); str_date = str(exp_date)
			exp_code = rospy.get_param('/hast/exp_code'); str_code = str(exp_code)
			exp_trial = rospy.get_param('/hast/trial'); str_trial = str(exp_trial).zfill(3)
		except:
			exp_date = rospy.get_param('~date'); str_date = str(exp_date)
			exp_trial = rospy.get_param('~trial'); str_trial = str(exp_trial).zfill(3)
			str_user = "benjamin"

		self.file_loggerfilename =('/home/benjamin/ros/data/{0}/{1}/{2}/vicon_{2}.m').format(str_date,str_code,str_trial)
		self.file_logger = open(self.file_loggerfilename, 'w')

		self.gazebo = rospy.get_param('~gazebo')
		print("~~~~ is GAZEBO? {} ~~~~~").format(self.gazebo)
		self.world_origin = rospy.get_param('~world_origin')
		self.ugv1_map_frame = rospy.get_param('~ugv1_map_frame')
		self.ugv2_map_frame = rospy.get_param('~ugv2_map_frame')
		self.start_time_topic = rospy.get_param('~time_sync_topic')
		self.time_service_topic = rospy.get_param('~time_service_topic')
		self.ugvugvsensor_topic = rospy.get_param('~ugvugvsensor_topic')

		self.start_time = rospy.get_time()
		rospy.set_param(self.start_time_topic, self.start_time)
		# self.ros_time_server = rospy.Service('/hast/serve/time', servtime, self.serve_ros_time)
		self.ros_time_server = rospy.Service(self.time_service_topic, servtime, self.serve_ros_time)
		# self.ugvugv_sensor_server = rospy.Service('/hast/ugvugvsensor', ugvugvsensor, self.serve_ugv_in_FOV)
		self.ugvugv_sensor_server = rospy.Service(self.ugvugvsensor_topic, ugvugvsensor, self.serve_ugv_in_FOV)

		self.file_logger.write("%% --------------------\n")
		self.file_logger.write(("  vicon.service.time_start = {};\n").format(self.start_time))
		self.file_logger.write(("  vicon.params.world  =  ['{}'];\n").format(str(self.world_origin)))
		self.file_logger.write(("  vicon.params.ugv1_map_frame =  ['{}'];\n").format(str(self.ugv1_map_frame)))
		self.file_logger.write(("  vicon.params.ugv2_map_frame =  ['{}'];\n").format(str(self.ugv2_map_frame)))
		self.file_logger.write(("  vicon.params.gazebo =  ['{}'];\n").format(str(self.gazebo)))
		if (not self.gazebo):
			print("~~~~ NOT GAZEBO ~~~~~")

		self.file_logger.write(("  vicon.params.time_sync_topic =  ['{}'];\n").format(str(self.start_time_topic)))
		self.file_logger.write(("  vicon.params.time_sync =  ['{}'];\n").format(str(self.start_time)))

		self.metronome_rate = rospy.Rate(10) #10hz
		self.Frame = 1
		self.vicon_bool = True
		self.vicon_home_origin = dict()
		self.vicon_home_origin['x'] = []
		self.vicon_home_origin['y'] = []
		self.vicon_home_origin['z'] = []
		self.vicon_home_origin['yaw'] = []
		self.vicon_home_origin['q0'] = []
		self.vicon_home_origin['q1'] = []
		self.vicon_home_origin['q2'] = []
		self.vicon_home_origin['q3'] = []

		self.ugv_global_position = []
		self.ugv_global_orientation = []

		self.body_names=[]
		self.body_paths=[]
		self.ugv1_dict = dict()
		self.ugv2_dict = dict()
		self.bodies = rospy.get_param('~body_labels')
		for body_label in self.bodies:
			self.body_names.append(body_label['name'])
			self.body_paths.append(body_label['path'])
			if body_label['name'] == "ugv1":
				self.ugv1_dict['path'] = body_label['path']
			if body_label['name'] == "ugv2":
				self.ugv2_dict['path'] = body_label['path']


		for body_label in self.bodies:
			print("%% vicon_logger::logging tf:{} to tf:{}").format(str(self.world_origin), body_label['path'])
			self.file_logger.write(("  vicon.tf_topics.{} =  ['{}'];\n").format(body_label['name'], body_label['path']))

		self.xf_broadcast = tf.TransformBroadcaster()
		self.xf_listener = tf.TransformListener()
		self.xf_ros = tf.TransformerROS(True,rospy.Duration(10.0))
		self.data_bool = self.getUAV2CAM()

		# load tag detections
		tag_array_topic = rospy.get_param('~tag_array_topic')
		self.tag_array_sub = rospy.Subscriber(tag_array_topic, AprilTagDetectionArray, self.tag_array_cb)
		# self.Hvic2uav = None

		self.uav = {'yaw_v':0, 'pos_v':[0, 0, 0]}
		self.april_tags = {	0: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												1: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												2: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												3: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												4: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												5: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												6: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												7: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												8: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												9: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												10: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}, \
												11: {'seq': 0, 'yaw_v':0, 'pos_v':[0, 0, 0]}}

		while not rospy.is_shutdown():
			# rospy.spin()
			# self.getUAV2CAM()
			self.metronome_update()
			self.metronome_rate.sleep()


	# def serve_ros_time(self, req):
	# 	response = Header()
	# 	response.stamp = rospy.get_rostime()
	# 	response.frame_id = "vicon_rostime"
	#
	# 	print("%% vicon_logger::{}/{}.{} requested by {}").format(response.frame_id, response.stamp.secs, response.stamp.nsecs, req.requesting_node)
	# 	self.file_logger.write(("  vicon.service.time_{} = {};\n").format(req.requesting_node, response.stamp.to_sec()))
	# 	# rosservice call /hast/serve/time "test"
	# 	return response

	def serve_ros_time(self, req):
		response = synctime()
		response.frame_id = "vicon_rostime"
		response.node_start = self.start_time
		response.now = rospy.get_rostime()

		print("%%  {} :: node_start={},  now = {}.{},  requested by {}").format(response.frame_id, \
					response.node_start, \
					response.now.secs, response.now.nsecs, \
					req.requesting_node)

		self.file_logger.write(("  vicon.service.{}.node_start_time = {};\n").format(req.requesting_node, self.start_time))
		self.file_logger.write(("  vicon.service.{}.time_now        = {};\n").format(req.requesting_node, response.now.to_sec()))

		# rosservice call /hast/serve/time "test"
		return response

# resp = servicenameResponse()
# resp.firstThingToReturn = 1
# resp.secondThingToReturn = 2
# return resp
# self.ros_time_serverResponse()

		# try:
		#     add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
		#     resp1 = add_two_ints(x, y)
		#     return resp1.sum
		# except rospy.ServiceException as e:
		#     print("Service call failed: %s"%e)
		# hast.srv.servtimeResponse()
		# self.ugv_header = std_msgs.msg.Header()

# rospy.Time.now(), rospy.get_rostime()
#
# Get the current time as either a rospy.Time instance. rospy.Time.now() and rospy.get_rostime() are equivalent.
#
# Toggle line numbers
	# now = rospy.get_rostime()
	# rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
	# rospy.get_time()
# Get the current time in float seconds.
	# seconds = rospy.get_time()


	def serve_ugv_in_FOV(self, req):
		# rosservice call /hast/ugvugvsensor {'ugv1','ugv2'}
		in_FOV = False
		ugv_position_left  = Vector3(); ugv_position_left.x  = 0; ugv_position_left.y  = 0; ugv_position_left.z  = 0
		ugv_position_right = Vector3();	ugv_position_right.x = 0; ugv_position_right.y = 0; ugv_position_right.z = 0

		left_cam_left_marker_position  = Vector3(); left_cam_left_marker_position.x  = 0; left_cam_left_marker_position.y  = 0; left_cam_left_marker_position.z  = 0
		left_cam_right_marker_position = Vector3(); left_cam_right_marker_position.x = 0; left_cam_right_marker_position.y = 0; left_cam_right_marker_position.z = 0
		left_cam_rear_marker_position  = Vector3(); left_cam_rear_marker_position.x  = 0; left_cam_rear_marker_position.y  = 0; left_cam_rear_marker_position.z  = 0

		right_cam_left_marker_position  = Vector3(); right_cam_left_marker_position.x  = 0; right_cam_left_marker_position.y  = 0; right_cam_left_marker_position.z  = 0
		right_cam_right_marker_position = Vector3(); right_cam_right_marker_position.x = 0; right_cam_right_marker_position.y = 0; right_cam_right_marker_position.z = 0
		right_cam_rear_marker_position  = Vector3(); right_cam_rear_marker_position.x  = 0; right_cam_rear_marker_position.y  = 0; right_cam_rear_marker_position.z  = 0

		if (self.gazebo):
			requesting_ugv_baseframe        = "/gazebo/{}/base_footprint".format(req.requesting_ugv)
			requesting_ugv_left_cam_frame   = "/gazebo/{}/stereo_left".format(req.requesting_ugv)
			requesting_ugv_right_cam_frame  = "/gazebo/{}/stereo_right".format(req.requesting_ugv)
			observed_ugv_baseframe          = "/gazebo/{}/base_footprint".format(req.observed_ugv)
			observed_ugv_left_marker_frame  = "/gazebo/{}/left_marker".format(req.observed_ugv)
			observed_ugv_right_marker_frame = "/gazebo/{}/right_marker".format(req.observed_ugv)
			observed_ugv_rear_marker_frame  = "/gazebo/{}/rear_marker".format(req.observed_ugv)

		else:
			requesting_ugv_baseframe        = "/{0}/{0}/base_footprint".format(req.requesting_ugv)
			requesting_ugv_left_cam_frame   = "/???/{}/stereo_left".format(req.requesting_ugv)
			requesting_ugv_right_cam_frame  = "/???/{}/stereo_right".format(req.requesting_ugv)
			observed_ugv_baseframe          = "/{0}/{0}/base_footprint".format(req.observed_ugv)
			observed_ugv_left_marker_frame  = "/vicon/{}/left_marker".format(req.observed_ugv)
			observed_ugv_right_marker_frame = "/vicon/{}/right_marker".format(req.observed_ugv)
			observed_ugv_rear_marker_frame  = "/vicon/{}/rear_marker".format(req.observed_ugv)

		# get locations and orientations for ugv1 and ugv2
		try:
			(xf_t,xf_q) = self.xf_listener.lookupTransform(requesting_ugv_baseframe, observed_ugv_baseframe, rospy.Time(0))
			tf_found = True
		except:
			tf_found = False
			in_FOV = False

		if (tf_found):
			phi = math.atan2(xf_t[1], xf_t[0])
			# print("phi  : {}").format(phi)

			if abs(phi) < 0.25:
				in_FOV = True
				# calculate observed ugv pose in left camera frame
				try:
					(left_xf_t,  left_xf_q)  = self.xf_listener.lookupTransform(requesting_ugv_left_cam_frame,  observed_ugv_left_marker_frame, rospy.Time(0))
					(right_xf_t, right_xf_q) = self.xf_listener.lookupTransform(requesting_ugv_left_cam_frame,  observed_ugv_right_marker_frame, rospy.Time(0))
					(rear_xf_t,  rear_xf_q)  = self.xf_listener.lookupTransform(requesting_ugv_left_cam_frame,  observed_ugv_rear_marker_frame, rospy.Time(0))
					left_cam_left_marker_position.x  = -left_xf_t[1];  left_cam_left_marker_position.y  = -left_xf_t[2];  left_cam_left_marker_position.z  = left_xf_t[0]
					left_cam_right_marker_position.x = -right_xf_t[1]; left_cam_right_marker_position.y = -right_xf_t[2]; left_cam_right_marker_position.z = right_xf_t[0]
					left_cam_rear_marker_position.x  = -rear_xf_t[1];  left_cam_rear_marker_position.y  = -rear_xf_t[2];  left_cam_rear_marker_position.z  = rear_xf_t[0]
					# print("      ugv_position_left =  [{} {} {}]").format(ugv_position_left.x, ugv_position_left.y, ugv_position_left.z)
					left_tf_found = True
				except:
					left_tf_found = False
					# print("      left_tf_found  = False")
				try:
					(left_xf_t,  left_xf_q)  = self.xf_listener.lookupTransform(requesting_ugv_right_cam_frame,  observed_ugv_left_marker_frame, rospy.Time(0))
					(right_xf_t, right_xf_q) = self.xf_listener.lookupTransform(requesting_ugv_right_cam_frame,  observed_ugv_right_marker_frame, rospy.Time(0))
					(rear_xf_t,  rear_xf_q)  = self.xf_listener.lookupTransform(requesting_ugv_right_cam_frame,  observed_ugv_rear_marker_frame, rospy.Time(0))
					right_cam_left_marker_position.x  = -left_xf_t[1];  right_cam_left_marker_position.y  = -left_xf_t[2];  right_cam_left_marker_position.z  = left_xf_t[0]
					right_cam_right_marker_position.x = -right_xf_t[1]; right_cam_right_marker_position.y = -right_xf_t[2]; right_cam_right_marker_position.z = right_xf_t[0]
					right_cam_rear_marker_position.x  = -rear_xf_t[1];  right_cam_rear_marker_position.y  = -rear_xf_t[2];  right_cam_rear_marker_position.z  = rear_xf_t[0]
					# print("      ugv_position_right = [{} {} {}]").format(ugv_position_right.x, ugv_position_right.y, ugv_position_right.z)
					right_tf_found = True
				except:
					right_tf_found = False
					# print("      right_tf_found = False")
			else:
				in_FOV = False

		# print("      left_cam_left_marker_position   = [{} {} {}]").format(left_cam_left_marker_position.x,   left_cam_left_marker_position.y,   left_cam_left_marker_position.z)
		# print("      right_cam_left_marker_position  = [{} {} {}]").format(right_cam_left_marker_position.x,  right_cam_left_marker_position.y,  right_cam_left_marker_position.z)
		# print("      left_cam_right_marker_position  = [{} {} {}]").format(left_cam_right_marker_position.x,  left_cam_right_marker_position.y,  left_cam_right_marker_position.z)
		# print("      right_cam_right_marker_position = [{} {} {}]").format(right_cam_right_marker_position.x, right_cam_right_marker_position.y, right_cam_right_marker_position.z)
		# print("      left_cam_rear_marker_position   = [{} {} {}]").format(left_cam_rear_marker_position.x,   left_cam_rear_marker_position.y,   left_cam_rear_marker_position.z)
		# print("      right_cam_rear_marker_position  = [{} {} {}]").format(right_cam_rear_marker_position.x,  right_cam_rear_marker_position.y,  right_cam_rear_marker_position.z)
		return in_FOV, \
				left_cam_left_marker_position, left_cam_right_marker_position, left_cam_rear_marker_position, \
				right_cam_left_marker_position, right_cam_right_marker_position, right_cam_rear_marker_position


	# Checks if a matrix is a valid rotation matrix.
	def isRotationMatrix(self,R) :
		Rt = np.transpose(R)
		shouldBeIdentity = np.dot(Rt, R)
		I = np.identity(3, dtype = R.dtype)
		n = np.linalg.norm(I - shouldBeIdentity)
		return n < 1e-6

	# Calculates rotation matrix to euler angles
	# The result is the same as MATLAB except the order
	# of the euler angles ( x and z are swapped ).
	def rotationMatrixToEulerAngles(self,R) :
		assert(self.isRotationMatrix(R))
		sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
		singular = sy < 1e-6

		if  not singular :
			x = math.atan2(R[2,1] , R[2,2])
			y = math.atan2(-R[2,0], sy)
			z = math.atan2(R[1,0], R[0,0])
		else :
			x = math.atan2(-R[1,2], R[1,1])
			y = math.atan2(-R[2,0], sy)
			z = 0

		return np.array([x, y, z])

	def getUAV2CAM(self):
		try:
			base_link = "/hast/uav/ardrone_base_link"
			cam_link = "/hast/uav/base_bottomcam"
			(xf_t,xf_q) = self.xf_listener.lookupTransform(base_link, cam_link, rospy.Time(0))
			self.Xform_uav2cam = self.xf_ros.fromTranslationRotation(xf_t, xf_q)
			return True
		except:
			return False

	def tag_array_cb(self,data):
		# using the metronome topic, record all xfs

		if self.getUAV2CAM():
			# self.file_logger.write("\n%% -------- detections------------\n")
			detections = data.detections
			# print("detections = data.detections: {}").format(detections)
			for detection in detections:
				body_label = "april" + str(detection.id).zfill(2)
				self.file_logger.write(("  %% ~~~~ {}\n").format(body_label))

				self.april_tags[detection.id]['seq'] += 1
				seq = self.april_tags[detection.id]['seq']

				# print("{}.seq = {}").format(body_label, seq)

				# pose in camera frame:
				xf_t = [detection.pose.pose.position.x,\
								detection.pose.pose.position.y,\
								detection.pose.pose.position.z]
				xf_q = [detection.pose.pose.orientation.x,\
								detection.pose.pose.orientation.y,\
								detection.pose.pose.orientation.z,\
								detection.pose.pose.orientation.w]

				# 4x4 numpy matrix
				Xform_cam2tag = self.xf_ros.fromTranslationRotation(xf_t, xf_q)
				# print("Xform_cam2tag = self.xf_ros.fromTranslationRotation(xf_t, xf_q)")
				# self.printNumpyH(Xform_cam2tag)

				# 4x4 numpy matrix
				Xform_uav2tag = np.matmul(self.Xform_uav2cam, Xform_cam2tag)
				# print("Xform_uav2tag = np.matmul(self.Xform_uav2cam, Xform_cam2tag)")
				# self.printNumpyH(Xform_uav2tag)

				(roll, pitch, yaw) = euler_from_quaternion (xf_q)


				relative_yaw = self.april_tags[detection.id]['yaw_v'] - self.uav['yaw_v']

				# print("t = {}").format(t)
				# print("t.to_sec() = {}").format(t.to_sec())

				R = np.matrix([\
					[Xform_uav2tag[0][0], Xform_uav2tag[0][1], Xform_uav2tag[0][2]],\
					[Xform_uav2tag[1][0], Xform_uav2tag[1][1], Xform_uav2tag[1][2]],\
					[Xform_uav2tag[2][0], Xform_uav2tag[2][1], Xform_uav2tag[2][2]]])

				(detection_roll, detection_pitch, detection_yaw) = self.rotationMatrixToEulerAngles(R)

				t = detection.pose.header.stamp

				self.file_logger.write(("    vicon.{}.detection.time({},1) = {};\n").format(body_label, seq, t.to_sec()-self.start_time))
				self.file_logger.write(("    vicon.{}.detection.position({},:) = [{},{},{}];\n").format(body_label, seq, xf_t[0],xf_t[1],xf_t[2]))
				self.file_logger.write(("    vicon.{}.detection.orientation({},:) = [{},{},{},{}];\n").format(body_label, seq, xf_q[0],xf_q[1],xf_q[2],xf_q[3]))
				self.file_logger.write(("    vicon.{}.detection.yaw_uav({},1) = {};\n").format(body_label, seq, detection_yaw))
				self.file_logger.write(("    vicon.{}.vicon.yaw_uav({},1) = {};\n").format(body_label, seq, relative_yaw))
				self.file_logger.write(("    vicon.{}.position.uav({},:) = [{},{},{}];\n").format(body_label, seq, Xform_uav2tag[0][3], Xform_uav2tag[1][3], Xform_uav2tag[2][3]))


				# self.file_logger.write(("    vicon.{}.rpy({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,roll, pitch, yaw))
				# self.file_logger.write(("    vicon.{}.position.gl({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,pos_gl[0],pos_gl[1],pos_gl[2]))
				# self.file_logger.write(("    vicon.{}.position.vic({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,xf_t[0],xf_t[1],xf_t[2]))
				# self.file_logger.write(("    vicon.{}.quaternion.vic({},:) = [{},{},{},{}];\n").format(body_label['name'],self.Frame,xf_q[0],xf_q[1],xf_q[2],xf_q[3]))

	def metronome_update(self):
		# use previous 10 measurements to set the global origin
		# print("def metronome_update(self):");
		time_f = rospy.get_time()
		if (self.vicon_bool):
			for body_label in self.bodies:
				if (body_label['name']=='ugv1'):
					if (self.Frame < 10):
						try:
							(xf_t,xf_q) = self.xf_listener.lookupTransform(self.world_origin, body_label['path'], rospy.Time(0))
							(roll, pitch, yaw) = euler_from_quaternion (xf_q)
							self.vicon_home_origin['x'].append(xf_t[0])
							self.vicon_home_origin['y'].append(xf_t[1])
							self.vicon_home_origin['z'].append(xf_t[2])
							self.vicon_home_origin['yaw'].append(yaw)
							self.vicon_home_origin['q0'].append(xf_q[0])
							self.vicon_home_origin['q1'].append(xf_q[1])
							self.vicon_home_origin['q2'].append(xf_q[2])
							self.vicon_home_origin['q3'].append(xf_q[3])
							self.file_logger.write(("  vicon.origin_src.yaw({},1) = {};\n").format(self.Frame ,yaw))
							self.file_logger.write(("  vicon.origin_src.position({},:) = [{},{},{}];\n").format(self.Frame,xf_t[0],xf_t[1],xf_t[2]))
							self.Frame += 1
						except:
							True
					else:
						print("%% vicon_logger::setting trial origin")
						# average previous measurements
						self.vicon_bool = False
						sum_x = sum(self.vicon_home_origin['x'])
						len_x = float(len(self.vicon_home_origin['x']))
						sum_y = sum(self.vicon_home_origin['y'])
						len_y = float(len(self.vicon_home_origin['y']))
						sum_z = sum(self.vicon_home_origin['z'])
						len_z = float(len(self.vicon_home_origin['z']))
						sum_yaw = sum(self.vicon_home_origin['yaw'])
						len_yaw = float(len(self.vicon_home_origin['yaw']))
						sum_q0 = sum(self.vicon_home_origin['q0'])
						sum_q1 = sum(self.vicon_home_origin['q1'])
						sum_q2 = sum(self.vicon_home_origin['q2'])
						sum_q3 = sum(self.vicon_home_origin['q3'])
						len_q = float(len(self.vicon_home_origin['q0']))

						self.ugv_global_position = np.array([sum_x/len_x, sum_y/len_y, sum_z/len_z])
						self.ugv_global_orientation = np.array([sum_q0/len_q, sum_q1/len_q, sum_q2/len_q, sum_q3/len_q])
						self.ugv_global_angle = sum_yaw/len_yaw
						self.Rz_global = np.matrix([\
							[math.cos(self.ugv_global_angle), math.sin(self.ugv_global_angle), 0],\
							[-math.sin(self.ugv_global_angle), math.cos(self.ugv_global_angle), 0],\
							[0, 0, 1]])
						self.file_logger.write(("  vicon.global_origin.yaw = {};\n").format(self.ugv_global_angle))
						self.file_logger.write(("  vicon.global_origin.position = [{},{},{}];\n").format(self.ugv_global_position[0],self.ugv_global_position[1],self.ugv_global_position[2]))
						self.file_logger.write(("  vicon.global_origin.orientation = [{},{},{},{}];\n").format(self.ugv_global_orientation[0],self.ugv_global_orientation[1],self.ugv_global_orientation[2],self.ugv_global_orientation[3]))

						self.negRT = -self.Rz_global*self.ugv_global_position[:, np.newaxis]
						# print("self.negRT: {}").format(self.negRT)
						negRT = np.array(self.negRT).reshape(-1,).tolist()
						# print("negRT: {}").format(negRT)
						self.file_logger.write(("  vicon.global_origin.negRT = [{},{},{}];\n").format(negRT[0],negRT[1],negRT[2]))
						self.file_logger.write("%% --------------------\n")
						self.Frame = 1
		else: # if (not self.vicon_bool):: this means the global origin has been set
			self.file_logger.write("\n%% --------Vicon-----------\n")
			self.file_logger.write(("  vicon.time({},1) = {};\n").format(self.Frame, time_f-self.start_time))
			if (not self.gazebo):
				self.xf_broadcast.sendTransform(self.ugv_global_position, self.ugv_global_orientation, rospy.Time.now(), "/map", "/vicon")

			# log all of the bodies in the experiment
			for body_label in self.bodies:
				try:
					(xf_t,xf_q) = self.xf_listener.lookupTransform(self.world_origin, body_label['path'], rospy.Time(0))
					pos_vic = np.array([xf_t[0],xf_t[1],xf_t[2]])
					# print("pos_vic: {}").format(pos_vic)
					# pos_gl = self.Rz_global*pos_vic[:, np.newaxis]+self.negRT
					pos_gl = self.Rz_global*pos_vic[:, np.newaxis]+self.negRT
					# print("{}.pos_gl: {}").format(body_label['name'], pos_gl)
					pos_gl = np.array(pos_gl).reshape(-1,).tolist()
					# print("pos_gl: {}").format(pos_gl)
					if (body_label['name']=='ugv1'):
						ugv1_t = np.array([xf_t[0],xf_t[1],xf_t[2]])
						(roll, pitch, ugv1_yaw) = euler_from_quaternion (xf_q)
					if (body_label['name']=='ugv2'):
						ugv2_t = np.array([xf_t[0],xf_t[1],xf_t[2]])
						(roll, pitch, ugv2_yaw) = euler_from_quaternion (xf_q)
					if (body_label['name']=='red'):
						red_t = np.array([xf_t[0],xf_t[1],xf_t[2]])
						red_q = xf_q
					if (body_label['name']=='blue'):
						blue_t = np.array([xf_t[0],xf_t[1],xf_t[2]])
						blue_q = xf_q
					if (body_label['name']=='green'):
						green_t = np.array([xf_t[0],xf_t[1],xf_t[2]])
						green_q = xf_q
				except:
					pos_gl = [0,0,0]
					xf_t = [0,0,0]
					xf_q = [0,0,0,1]

				(roll, pitch, yaw) = euler_from_quaternion (xf_q)
				self.file_logger.write(("  %% ~~~~ {}\n").format(body_label['name']))
				self.file_logger.write(("    vicon.{}.yaw.vic({},1) = {};\n").format(body_label['name'],self.Frame,yaw))
				self.file_logger.write(("    vicon.{}.yaw.gl({},1) = {};\n").format(body_label['name'],self.Frame,yaw-self.ugv_global_angle))
				self.file_logger.write(("    vicon.{}.rpy({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,roll, pitch, yaw))
				self.file_logger.write(("    vicon.{}.position.gl({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,pos_gl[0],pos_gl[1],pos_gl[2]))
				self.file_logger.write(("    vicon.{}.position.vic({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,xf_t[0],xf_t[1],xf_t[2]))
				self.file_logger.write(("    vicon.{}.quaternion.vic({},:) = [{},{},{},{}];\n").format(body_label['name'],self.Frame,xf_q[0],xf_q[1],xf_q[2],xf_q[3]))

				if (not self.gazebo):
					self.xf_broadcast.sendTransform([0,0,0], [0,0,0,1], rospy.Time.now(), ("/vicon/{0}/base_footprint").format(body_label['name']), ("/vicon/{0}/{0}").format(body_label['name']))

				if body_label['name'] in {"red", "blue", "green"}:
					try:
						(xf_t_ugv1,xf_q_ugv1) = self.xf_listener.lookupTransform(self.ugv1_dict['path'],body_label['path'], rospy.Time(0))
						# (xf_t_ugv1,xf_q_ugv1) = self.xf_listener.lookupTransform(body_label['path'],self.ugv1_dict['path'], rospy.Time(0))
						pos_ugv1 = np.array([xf_t_ugv1[0],xf_t_ugv1[1],xf_t_ugv1[2]])
					except:
						pos_ugv1 = [0,0,0]
						xf_t_ugv1 = [0,0,0]
						xf_q_ugv1 = [0,0,0,1]
					try:
						(xf_t_ugv2,xf_q_ugv2) = self.xf_listener.lookupTransform(self.ugv2_dict['path'],body_label['path'], rospy.Time(0))
						# (xf_t_ugv2,xf_q_ugv2) = self.xf_listener.lookupTransform(body_label['path'],self.ugv2_dict['path'], rospy.Time(0))
						pos_ugv2 = np.array([xf_t_ugv2[0],xf_t_ugv2[1],xf_t_ugv2[2]])
					except:
						pos_ugv2 = [0,0,0]
						xf_t_ugv2 = [0,0,0]
						xf_q_ugv2 = [0,0,0,1]
					self.file_logger.write(("    vicon.{}.position.ugv1({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,pos_ugv1[0],pos_ugv1[1],pos_ugv1[2]))
					self.file_logger.write(("    vicon.{}.position.ugv2({},:) = [{},{},{}];\n").format(body_label['name'],self.Frame,pos_ugv2[0],pos_ugv2[1],pos_ugv2[2]))

				# try to write vicon data to april dictionary
				try:
					self.april_tags[int(body_label['name'][-2:])]['yaw_v'] = yaw
					self.april_tags[int(body_label['name'][-2:])]['pos_v'] = xf_t
				except:
					dummy = True


			# now use the RGB leds to calculate the UAC pose
			try:
				# print("uav_t.red = [{},{},{}]").format(red_t[0], red_t[1],red_t[2])
				# print("uav_t.red = [{},{},{}]").format(blue_t[0], blue_t[1],blue_t[2])
				# print("uav_t.red = [{},{},{}]").format(green_t[0], green_t[1],green_t[2])
				uav_t = 0.5*(red_t+blue_t)
				# print("uav_t     = [{},{},{}]").format(uav_t[0], uav_t[1],uav_t[2])
				uav_yaw = self.rgb2yaw(red_t, blue_t, green_t)
				uav_gl = self.Rz_global*uav_t[:, np.newaxis]+self.negRT
				# print("uav_gl: {}").format(uav_gl)
				uav_gl = np.array(uav_gl).reshape(-1,).tolist()
				# uav_gl = np.array(uav_gl)[0].tolist()
				# print("uav_gl: {}").format(uav_gl)
				uav_q = blue_q
			except:
				uav_t = [0,0,0]
				uav_q = [0,0,0,1]
				uav_yaw = 0
				uav_gl = [0,0,0]

			# print("self.Hvic2uav = self.computeH(uav_yaw, uav_t)")
			# print("uav_yaw: {}").format(uav_yaw)
			# print("uav_t: {}").format(uav_t)
			# self.Hvic2uav = self.computeH(uav_yaw, uav_t)

			# try to compute the uav in both ugv local frames as well
			# check to see if uav is all zeros
			if all(uav_t):
				# print("uav_t     = [{},{},{}]").format(uav_t[0], uav_t[1],uav_t[2])
				uav_th = np.array([[uav_t[0]],
														[uav_t[1]],
														[uav_t[2]],
														[1]])

				try:
					Hvic2ugv1 = self.computeH(ugv1_yaw, ugv1_t)
					uav_t_in_ugv1 = Hvic2ugv1*uav_th
					uav_t_in_ugv1 = np.array(uav_t_in_ugv1).reshape(-1,).tolist()
					# print("uav_t_in_ugv1: {}").format(uav_t_in_ugv1)
				except:
					uav_t_in_ugv1 = [0,0,0]
					uav_yaw_ugv1 = 0

				try:
					Hvic2ugv2 = self.computeH(ugv2_yaw, ugv2_t)
					uav_t_in_ugv2 = Hvic2ugv2*uav_th
					uav_t_in_ugv2 = np.array(uav_t_in_ugv2).reshape(-1,).tolist()
					# print("uav_t_in_ugv2: {}").format(uav_t_in_ugv2)
				except:
					uav_t_in_ugv2 = [0,0,0]
					uav_yaw_ugv2 = 0

			else:
				uav_t_in_ugv1 = [0,0,0]
				uav_t_in_ugv2 = [0,0,0]
				uav_yaw_ugv1 = 0
				uav_yaw_ugv2 = 0

			self.uav['yaw_v'] = uav_yaw
			self.uav['pos_v'] = uav_t
			self.file_logger.write(("  %% ~~~~ uav\n"))
			self.file_logger.write(("    vicon.uav.yaw.vic({},1) = {};\n").format(self.Frame,uav_yaw))
			self.file_logger.write(("    vicon.uav.yaw.gl({},1)  = {};\n").format(self.Frame,uav_yaw - self.ugv_global_angle))
			self.file_logger.write(("    vicon.uav.position.vic({},:) = [{},{},{}];\n").format(self.Frame,uav_t[0],uav_t[1],uav_t[2]))
			self.file_logger.write(("    vicon.uav.position.gl({},:)  = [{},{},{}];\n").format(self.Frame,uav_gl[0],uav_gl[1],uav_gl[2]))
			self.file_logger.write(("    vicon.uav.position.ugv1({},:) = [{},{},{}];\n").format(self.Frame,uav_t_in_ugv1[0],uav_t_in_ugv1[1],uav_t_in_ugv1[2]))
			try:
				self.file_logger.write(("    vicon.uav.yaw.ugv1({},1) = {};\n").format(self.Frame,uav_yaw - ugv1_yaw))
			except:
				dummy = 0 # just do nothing.

			if (not self.gazebo):
				self.xf_broadcast.sendTransform(uav_t, uav_q, rospy.Time.now(),("/vicon/uav/uav"), self.world_origin)
				self.xf_broadcast.sendTransform([0,0,0], [0,0,0,1], rospy.Time.now(), ("/vicon/uav/base_footprint"), ("/vicon/uav/uav"))

			try:
				self.file_logger.write(("    vicon.uav.position.ugv2({},:) = [{},{},{}];\n").format(self.Frame,uav_t_in_ugv2[0],uav_t_in_ugv2[1],uav_t_in_ugv2[2]))
				self.file_logger.write(("    vicon.uav.yaw.ugv2({},1) = {};\n").format(self.Frame,uav_yaw - ugv2_yaw))
			except:
				ugv2_yaw = 0 # just do nothing.

			self.Frame += 1

	def rgb2yaw(self, red_t, blue_t, green_t):
		BmR = blue_t - red_t
		GmB = green_t - blue_t
		RmG = red_t - green_t
		RmB = red_t - blue_t
		# print("RmB: {}").format(RmB)

		znum = np.cross(GmB, RmB)
		normz = np.linalg.norm(znum)
		normy = np.linalg.norm(BmR)
		# print("znum: {}").format(znum)

		z_axis = (1.0 / normz) * znum
		# print("z_axis: {}").format(z_axis)
		y_axis = (1.0 / normy) * BmR
		# print("y_axis: {}").format(y_axis)
		x_axis = np.cross(y_axis, z_axis)
		# print("x_axis: {}").format(x_axis)
		# print("yaw = math.atan2({}, {})").format(x_axis[1], x_axis[0])
		yaw = math.atan2(x_axis[1], x_axis[0])
		# print("yaw: {}").format(yaw)
		return yaw

	def computeRz(self, phi):
		Rz = np.matrix([\
				[ math.cos(phi), math.sin(phi), 0],\
				[-math.sin(phi), math.cos(phi), 0],\
				[0, 0, 1]])
		return Rz

	def printNumpyH(self, H):
		print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[0][0], H[0][1], H[0][2], H[0][3])
		print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[1][0], H[1][1], H[1][2], H[1][3])
		print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[2][0], H[2][1], H[2][2], H[2][3])
		print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[3][0], H[3][1], H[3][2], H[3][3])

	def computeH(self, phi, t):
		Rz = self.computeRz(phi)
		nRzt = -Rz*t[:,np.newaxis]
		H = np.concatenate((Rz, nRzt),axis=1)
		row4 = np.array([[0,0,0,1]])
		H = np.concatenate((H, row4),axis=0)
		return H

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('vicon_logger')
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		log = vicon_logger()
	except rospy.ROSInterruptException: pass



# >>> np.concatenate((a, b), axis=0)
# array([[1, 2],
#        [3, 4],
#        [5, 6]])
# >>> np.concatenate((a, b.T), axis=1)
# array([[1, 2, 5],
#        [3, 4, 6]])
