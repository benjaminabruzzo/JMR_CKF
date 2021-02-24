#!/usr/bin/env python

# Import required ros libraries code.
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

# import python libraries
import numpy as np
import cv2
import os,sys
import yaml


class recordCal():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# Ros params
		self.date = str(rospy.get_param('~calibration_date'))
		self.world_origin = rospy.get_param('~world_origin')
		self.image_folder_path = rospy.get_param("~image_folder_path","/")
		self.calibration_filename = rospy.get_param("~calibration_filename","calibrationXX")

		self.file_loggerfilename =('{}/{}.m').format(self.image_folder_path, self.calibration_filename)
		self.file_logger = open(self.file_loggerfilename, 'w')

		self.left_image_topic = rospy.get_param("~left_image_topic","/left/image_raw")
		self.right_image_topic = rospy.get_param("~right_image_topic","/right/image_raw")
		self.saveraw_image_topic = rospy.get_param("~saveraw_image_topic","/hast/saveraw")

		self.left_image_sub = rospy.Subscriber(self.left_image_topic, Image, self.left_image_bridge)
		self.right_image_sub = rospy.Subscriber(self.right_image_topic, Image, self.right_image_bridge)
		self.saveraw_image_sub = rospy.Subscriber(self.saveraw_image_topic, Empty, self.save_raw_images)


		self.xf_listener = tf.TransformListener()
		self.xf_ros = tf.TransformerROS(True,rospy.Duration(10.0))
		self.bridge = CvBridge()
		self.record_seq = 0 # number of images saved for calibration

		print("Recording left_image_topic: {} ").format(self.left_image_topic)
		print("Recording right_image_topic: {} ").format(self.right_image_topic)
		print("Saving images to: {}").format(self.image_folder_path)
		print("Saving data to: {}").format(self.file_loggerfilename)

		self.file_logger.write("%% --------------------\n")
		self.file_logger.write(("  vicon.params.world =  ['{}'];\n").format(str(self.world_origin)))
		self.file_logger.write(("  vicon.params.left_image_topic =  ['{}'];\n").format(str(self.left_image_topic)))
		self.file_logger.write(("  vicon.params.right_image_topic =  ['{}'];\n").format(str(self.right_image_topic)))
		self.file_logger.write(("  vicon.params.saveraw_image_topic =  ['{}'];\n").format(str(self.saveraw_image_topic)))

		self.body_names=[]
		self.body_paths=[]
		self.bodies = rospy.get_param('~body_labels')
		for body_label in self.bodies:
			self.body_names.append(body_label['name'])
			self.body_paths.append(body_label['path'])

		for body_label in self.bodies:
			print("%% vicon_logger::logging tf:{} to tf:{}").format(str(self.world_origin), body_label['path'])
			self.file_logger.write(("  vicon.tf_topics.{} =  ['{}'];\n").format(body_label['name'], body_label['path']))

	def left_image_bridge(self,msg):
		try:
			self.left_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

	def right_image_bridge(self,msg):
		try:
			self.right_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

	def save_raw_images(self, msg):
		self.record_seq += 1
		left_image_filename = ("{}/left_raw_{:06d}.png").format(self.image_folder_path, self.record_seq)
		right_image_filename = ("{}/right_raw_{:06d}.png").format(self.image_folder_path, self.record_seq)
		try:
			check_left = cv2.imwrite(left_image_filename, self.left_cv_image)
		except cv2.error as e:
			print(e)

		try:
			check_right = cv2.imwrite(right_image_filename, self.right_cv_image)
		except cv2.error as e:
			print(e)

		for body_label in self.bodies:
			try:
				(xf_t,xf_q) = self.xf_listener.lookupTransform(self.world_origin, body_label['path'], rospy.Time(0))
			except:
				xf_t = [0,0,0]
				xf_q = [0,0,0,1]

			(roll, pitch, yaw) = euler_from_quaternion (xf_q)
			self.file_logger.write(("  %% ~~~~ {}\n").format(body_label['name']))
			self.file_logger.write(("    vicon.{}.rpy({},:) = [{},{},{}];\n").format(body_label['name'],self.record_seq, roll, pitch, yaw))
			self.file_logger.write(("    vicon.{}.position.vic({},:) = [{},{},{}];\n").format(body_label['name'],self.record_seq, xf_t[0], xf_t[1], xf_t[2]))
			self.file_logger.write(("    vicon.{}.quaternion.vic({},:) = [{},{},{},{}];\n").format(body_label['name'],self.record_seq, xf_q[0], xf_q[1], xf_q[2], xf_q[3]))

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('recordCal')

	try:
		image_rec = recordCal()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()


# print("saving image: {}").format(image_filename)
#
# self.record_seq += 1
# self.save_frame_rate.sleep() # regulate the frame rate of the saving process
