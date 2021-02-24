#!/usr/bin/env python

# Import required Python code.
import rospy
from geometry_msgs.msg import Twist
from hast.msg import posewithheader
from hast.srv import slamswitch


def call_slamswitch():
	rospy.wait_for_service("/hast/service/uav/slamswitch")
	pose = posewithheader()
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 0
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1
	try:
		slamswitch_client = rospy.ServiceProxy("/hast/service/uav/slamswitch", slamswitch)
		response = slamswitch_client(True, pose)
		print("{}").format(response)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('slamswitch_py')
	call_slamswitch()

