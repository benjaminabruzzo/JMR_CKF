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

import csv
import itertools

def printNumpyH(H):
	print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[0][0], H[0][1], H[0][2], H[0][3])
	print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[1][0], H[1][1], H[1][2], H[1][3])
	print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[2][0], H[2][1], H[2][2], H[2][3])
	print("  {: .4f} {: .4f} {: .4f} {: .4f} ").format(H[3][0], H[3][1], H[3][2], H[3][3])


rospy.init_node('test_node')
xf_listener = tf.TransformListener()
xf_ros = tf.TransformerROS(True,rospy.Duration(10.0))

base_link = "/hast/uav/ardrone_base_link"
cam_link = "/hast/uav/base_bottomcam"
id0_base_link = "id00_16h5/base_link"
vicon_origin = "vicon/origin"


(xf_t,xf_q) = xf_listener.lookupTransform(base_link, cam_link, rospy.Time(0))
(id0_t,id0_q) = xf_listener.lookupTransform(vicon_origin, id0_base_link, rospy.Time(0))

Xform_uav2cam = xf_ros.fromTranslationRotation(xf_t, xf_q)
Xform_vic2id = xf_ros.fromTranslationRotation(id0_t, id0_q)



printNumpyH(Xform_vic2id)
printNumpyH(Xform_uav2cam)
xform = np.matmul(Xform_vic2id, Xform_uav2cam)
printNumpyH(xform)
