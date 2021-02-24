#!/usr/bin/env python

# Import required Python code.
import cv2
import roslib
import rospy
from hast.msg import uavstate, flag

class uav_state_remap():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# read subscriber topic parameters
		self.jointslam_flag_topic = rospy.get_param('~jointslam_state_flag')
		self.ckf_uav_autopilot_state_topic = rospy.get_param('~ckf_uav_autopilot_state_topic')
		self.slam_uav_autopilot_state_topic = rospy.get_param('~slam_uav_autopilot_state_topic')

		# subscribe to topics
		self.jointslam_switch_sub = rospy.Subscriber(self.jointslam_flag_topic, flag, self.jointslam_switch_cb)
		self.slamFlag = False

		self.ckf_uav_autopilot_sub = rospy.Subscriber(self.ckf_uav_autopilot_state_topic, uavstate, self.ckf_uav_autopilot_cb)
		self.slam_uav_autopilot_sub = rospy.Subscriber(self.slam_uav_autopilot_state_topic, uavstate, self.slam_uav_autopilot_cb)

		# publisher
		self.cmd_uav_autopilot_state_topic = rospy.get_param('~cmd_uav_autopilot_state_topic')
		self.cmd_uav_autopilot_pub = rospy.Publisher(self.cmd_uav_autopilot_state_topic, uavstate, queue_size=10)
		self.autopilot_state = uavstate()

		while not rospy.is_shutdown():
			rospy.spin()

	def navdata_cb(self, data):
		# call ardrone toggle cam service here

	# <node pkg="rosservice" type="rosservice" name="togglecam_$(env HOST)" args="call /ardrone/togglecam"/>
	def slam_uav_autopilot_cb(self, data):
		if (self.slamFlag):
			self.cmd_uav_autopilot_pub.publish(data)


	def trial_pub(self, data):
		# trigger flag that trial can start


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('uav_state_remap')
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# c_rmp = cmd_remap()
		p_rmp = uav_state_remap()
	except rospy.ROSInterruptException: pass
