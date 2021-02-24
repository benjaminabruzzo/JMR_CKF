#!/usr/bin/env python
# import python libraries
import numpy as np
import cv2
import os,sys,math, struct
import yaml
import PyKDL

# Import required ros libraries code.
import rospy
import tf, tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from cv_bridge import CvBridge, CvBridgeError
import std_msgs.msg
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.point_cloud2 import read_points, create_cloud


class test_stereoCal():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# Ros params
		self.queue_size = 10
		self.ugv_n = rospy.get_param('~ugv_n')
		self.date = rospy.get_param('/hast/date')
		self.trial = rospy.get_param('/hast/trial')

		self.FocalLength_topic = rospy.get_param("~FocalLength_topic",("/{}/stereo/FocalLength").format(self.ugv_n))
		self.FocalLength = rospy.get_param(("/{}/stereo/FocalLength").format(self.ugv_n))

		self.Baseline_topic = rospy.get_param("~Baseline_topic",("/{}/stereo/Baseline").format(self.ugv_n))
		self.Baseline = rospy.get_param('/{}/stereo/Baseline'.format(self.ugv_n))

		self.Cx_topic = rospy.get_param("~Cx_topic",("/{}/stereo/Cx").format(self.ugv_n))
		self.Cx = rospy.get_param('/{}/stereo/Cx'.format(self.ugv_n))

		self.Cy_topic = rospy.get_param("~Cy_topic",("/{}/stereo/Cy").format(self.ugv_n))
		self.Cy = rospy.get_param('/{}/stereo/Cy'.format(self.ugv_n))

		self.H_cam2ugv = np.matrix([
			[rospy.get_param('~h00'), rospy.get_param('~h01'), rospy.get_param('~h02'), rospy.get_param('~h03')],
			[rospy.get_param('~h10'), rospy.get_param('~h11'), rospy.get_param('~h12'), rospy.get_param('~h13')],
			[rospy.get_param('~h20'), rospy.get_param('~h21'), rospy.get_param('~h22'), rospy.get_param('~h23')]])

		print("Recording FocalLength: {} = {} ").format(self.FocalLength_topic, self.FocalLength)
		print("Recording Baseline: {} = {} ").format(self.Baseline_topic, self.Baseline)
		print("Recording Cx: {} = {} ").format(self.Cx_topic, self.Cx)
		print("Recording Cy: {} = {} ").format(self.Cy_topic, self.Cy)


		self.world_origin = rospy.get_param('~world_origin')
		self.save_data_path = rospy.get_param("~save_data_path","/benjamin/ros/data/")
		self.save_data_filename = rospy.get_param("~save_data_filename","calibrationXX")

		self.file_loggerfilename =('{0}/{1}/{2}/{3}_{2}.m').format(self.save_data_path, self.date, self.trial, self.save_data_filename)
		self.file_logger = open(self.file_loggerfilename, 'w')
		self.logger_idx = 0

		self.left_image_topic = rospy.get_param("~left_image_topic","/left/image_raw")
		self.right_image_topic = rospy.get_param("~right_image_topic","/right/image_raw")

		self.left_image_sub = rospy.Subscriber(self.left_image_topic, Image, self.left_image_bridge)
		self.right_image_sub = rospy.Subscriber(self.right_image_topic, Image, self.right_image_bridge)

		self.ugv_header = std_msgs.msg.Header()
		self.ugv_header_frame_id = rospy.get_param("~ugv_header_frame_id","vicon/{0}/{0}".format(self.ugv_n))
		self.ugv_header.frame_id = self.ugv_header_frame_id
		self.gator_header = std_msgs.msg.Header()
		self.gator_header_frame_id = rospy.get_param("~gator_header_frame_id","vicon/gatorboard/gatorboard")
		self.gator_header.frame_id = self.gator_header_frame_id
		self.stereo_header = std_msgs.msg.Header()
		self.stereo_header_frame_id = rospy.get_param("~stereo_header_frame_id","vicon/{0}/{0}".format(self.ugv_n))
		self.stereo_header.frame_id = self.stereo_header_frame_id

		self.ugv_points_topic = rospy.get_param("~ugv_points_topic","/ugv_points")
		self.ugv_points_pub = rospy.Publisher(self.ugv_points_topic, PointCloud2, queue_size=10)
		self.gator_points_topic = rospy.get_param("~gator_points_topic","/gator_points")
		self.gator_points_pub = rospy.Publisher(self.gator_points_topic, PointCloud2, queue_size=10)
		self.stereo_points_topic = rospy.get_param("~stereo_points_topic","/stereo_points")
		self.stereo_points_pub = rospy.Publisher(self.stereo_points_topic, PointCloud2, queue_size=10)

		self.gator_cloud_sub = rospy.Subscriber(self.gator_points_topic, PointCloud2, self.gator_cloud_cb)
		self.gator_header_cam = std_msgs.msg.Header()
		self.gator_header_cam.frame_id = self.stereo_header_frame_id
		self.gator_cloud_cam_pub = rospy.Publisher('/gator_cam', PointCloud2, queue_size=10)

		self.pc2_alpha = 255

		self.xf_listener = tf.TransformListener()
		self.bridge = CvBridge()
		self.left_received = False
		self.right_received = False

		print("Recording left_image_topic: {} ").format(self.left_image_topic)
		print("Recording right_image_topic: {} ").format(self.right_image_topic)
		print("Publishing gator_points_topic: {} ").format(self.gator_points_topic)
		print("Publishing stereo_points_topic: {} ").format(self.stereo_points_topic)
		print("Publishing gator_header_frame_id: {} ").format(self.gator_header_frame_id)
		print("Publishing stereo_header_frame_id: {} ").format(self.stereo_header_frame_id)
		print("Saving data to: {}").format(self.file_loggerfilename)

		self.file_logger.write("%% --------------------\n")
		self.file_logger.write(("  test_cal.params.world =  ['{}'];\n").format(str(self.world_origin)))
		self.file_logger.write(("  test_cal.params.left_image_topic =  ['{}'];\n").format(str(self.left_image_topic)))
		self.file_logger.write(("  test_cal.params.right_image_topic =  ['{}'];\n").format(str(self.right_image_topic)))
		self.file_logger.write(("  test_cal.params.gator_points_topic =  ['{}'];\n").format(str(self.gator_points_topic)))
		self.file_logger.write(("  test_cal.params.stereo_points_topic =  ['{}'];\n").format(str(self.stereo_points_topic)))
		self.file_logger.write(("  test_cal.params.gator_header_frame_id =  ['{}'];\n").format(str(self.gator_header_frame_id)))
		self.file_logger.write(("  test_cal.params.stereo_header_frame_id =  ['{}'];\n").format(str(self.stereo_header_frame_id)))

		self.file_logger.write(("  test_cal.params.FocalLength_topic =  ['{}'];\n").format(str(self.FocalLength_topic)))
		self.file_logger.write(("  test_cal.params.Baseline_topic =  ['{}'];\n").format(str(self.Baseline_topic)))
		self.file_logger.write(("  test_cal.params.Cx_topic =  ['{}'];\n").format(str(self.Cx_topic)))
		self.file_logger.write(("  test_cal.params.Cy_topic =  ['{}'];\n").format(str(self.Cy_topic)))

		self.file_logger.write(("  test_cal.params.FocalLength =  ['{}'];\n").format(str(self.FocalLength)))
		self.file_logger.write(("  test_cal.params.Baseline =  ['{}'];\n").format(str(self.Baseline)))
		self.file_logger.write(("  test_cal.params.Cx =  ['{}'];\n").format(str(self.Cx)))
		self.file_logger.write(("  test_cal.params.Cy =  ['{}'];\n").format(str(self.Cy)))
		self.file_logger.write(("  test_cal.params.H_cam2ugv =  [...\n   {}, {}, {}, {}; ...\n   {}, {}, {}, {}; ...\n   {}, {}, {}, {}];\n").format(\
							self.H_cam2ugv.item(0,0), self.H_cam2ugv.item(0,1), self.H_cam2ugv.item(0,2), self.H_cam2ugv.item(0,3), \
							self.H_cam2ugv.item(1,0), self.H_cam2ugv.item(1,1), self.H_cam2ugv.item(1,2), self.H_cam2ugv.item(1,3), \
							self.H_cam2ugv.item(2,0), self.H_cam2ugv.item(2,1), self.H_cam2ugv.item(2,2), self.H_cam2ugv.item(2,3)))
		# self.file_logger.write(("  test_cal.params.H =  [{}, {}, {}, {}; {}, {}, {}, {}; {}, {}, {}, {}];\n").format(\
		# self.H_cam2ugv.item(0,0), self.H_cam2ugv.item(0,1), self.H_cam2ugv.item(0,2), self.H_cam2ugv.item(0,3), \
		# self.H_cam2ugv.item(1,0), self.H_cam2ugv.item(1,1), self.H_cam2ugv.item(1,2), self.H_cam2ugv.item(1,3), \
		# self.H_cam2ugv.item(2,0), self.H_cam2ugv.item(2,1), self.H_cam2ugv.item(2,2), self.H_cam2ugv.item(2,3))))


		self.fields = [PointField('x', 0, PointField.FLOAT32,1),
							PointField('y', 4, PointField.FLOAT32,1),
							PointField('z', 8, PointField.FLOAT32,1),
							PointField('rgba', 12, PointField.UINT32,1),
							]


		self.body_names=[]
		self.body_paths=[]
		self.bodies = rospy.get_param('~body_labels')
		for body_label in self.bodies:
			self.body_names.append(body_label['name'])
			self.body_paths.append(body_label['path'])

		for body_label in self.bodies:
			print("%% test_cal::logging tf:{} to tf:{}").format(str(self.world_origin), body_label['path'])
			self.file_logger.write(("  test_cal.tf_topics.{} =  ['{}'];\n").format(body_label['name'], body_label['path']))

	def left_image_bridge(self,msg):
		try:
			self.left_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.left_received = True
		except CvBridgeError as e:
			print(e)

	def right_image_bridge(self,msg):
		try:
			self.right_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.right_received = True
		except CvBridgeError as e:
			print(e)

	def calculateVec(self, left_pixels, right_pixels):
		f = self.FocalLength
		b = self.Baseline
		Cx = self.Cx
		Cy = self.Cy
		xl = left_pixels[0] - Cx
		yl = left_pixels[1] - Cy
		xr = right_pixels[0] - Cx
		yr = right_pixels[1] - Cy
		d = abs(xl - xr);
		z = b*f/d;
		y = z*(yl+yr)/(2*f);
		x = z*(xl+xr)/(2*f);
		return [x,y,z]

	def gator_cloud_cb(self,msg):
		self.gator_header_cam.stamp = rospy.Time.now()
		try:
			# use this one to map points to 'camera' frame
			# xf_p, xf_q = self.xf_listener.lookupTransform(self.stereo_header_frame_id, self.gator_header_frame_id, rospy.Time())
			# use this one to map points to ugv base frame
			xf_p, xf_q = self.xf_listener.lookupTransform("vicon/{0}/{0}".format(self.ugv_n), self.gator_header_frame_id, rospy.Time())
			self.gator_cloud_bool = True
		except:
			self.gator_cloud_bool = False
		if (self.gator_cloud_bool):
			self.gator_cloud = self.do_transform_cloud(msg, self.gator_header_cam, xf_p, xf_q)
			self.gator_cloud_cam_pub.publish(self.gator_cloud)

	def transform_to_kdl(self, p, q):
		return PyKDL.Frame(PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3]), PyKDL.Vector(p[0], p[1], p[2]))

	def do_transform_cloud(self, cloud, header, position, quaternion):
		t_kdl = self.transform_to_kdl(position, quaternion)
		points_out = []
		pts_1 = []
		idx = 0
		for p_in in pc2.read_points(cloud):
			idx+=1
			p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
			if (idx==1):
				pts_1.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
			points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
		# res = pc2.create_cloud(header, cloud.fields, pts_1)
		res = pc2.create_cloud(header, cloud.fields, points_out)
		return res

	def publish_gator_cloud(self):
		# publish gator points in reality
		self.gator_pt_grid = np.zeros((6*8,3), np.float32)
		self.gator_pt_grid[:,:2] = 0.111*np.mgrid[0:8,0:6].T.reshape(-1,2)
		# self.gator_pt_grid[:,:2] = 0.111*np.mgrid[0:1,0:1].T.reshape(-1,2) # just one point
		gator_rbg = []
		idx = 0
		for point in self.gator_pt_grid:
			color = (255-5*idx,0,5*idx) # blue
			rgb = struct.unpack('I', struct.pack('BBBB', 255-5*idx, 0, 5*idx, self.pc2_alpha))[0]
			idx+=1
			gator_rbg.append([point[0], point[1], point[2], rgb])

		self.gator_header.stamp = rospy.Time.now()
		# self.gator_points_pcl = pc2.create_cloud_xyz32(self.gator_header, self.gator_pt_grid)
		# self.gator_points_pub.publish(self.gator_points_pcl)
		self.gator_points_rbg = pc2.create_cloud(self.gator_header, self.fields, gator_rbg)
		self.gator_points_pub.publish(self.gator_points_rbg)

	def remap_point(self, vec):
		x = np.matrix([[vec[0]], [vec[1]], [vec[2]], [1]])
		p = self.H_cam2ugv*x
		return [p.item(0,0), p.item(1,0), p.item(2,0)]

	def calculate_cloud(self):
		if (self.left_received and self.right_received):
			# calculate points from stereo images:
			left_cv_grey = cv2.cvtColor(self.left_cv_image,cv2.COLOR_BGR2GRAY)
			right_cv_grey = cv2.cvtColor(self.right_cv_image,cv2.COLOR_BGR2GRAY)
			# Find the chess board corners
			left_ret, left_corners = cv2.findChessboardCorners(left_cv_grey, (8,6), None)
			right_ret, right_corners = cv2.findChessboardCorners(right_cv_grey, (8,6), None)
			self.logger_idx += 1
			self.corners3d = []
			self.corners_rgb = []

			corner_idx = 0
			gator_idx = 0
			self.remapped_corners3d = []

			if left_ret and right_ret and self.gator_cloud_bool: #then both images have a checkerboard
				criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)
				left_corners_subpix = cv2.cornerSubPix(left_cv_grey, left_corners, (11, 11), (-1, -1), criteria)
				right_corners_subpix = cv2.cornerSubPix(right_cv_grey, right_corners, (11, 11), (-1, -1), criteria)

				left_circle = self.left_cv_image.copy()
				right_circle = self.right_cv_image.copy()

				radius = 5 # 5 pixel Radius
				thickness = 2

				for left_pixels, right_pixels in zip(left_corners_subpix, right_corners_subpix):
					color = (255-5*corner_idx,0,5*corner_idx) # blue
					rgb = struct.unpack('I', struct.pack('BBBB', 255-5*corner_idx, 0, 5*corner_idx, self.pc2_alpha))[0]
					corner_idx += 1
					corner3d = self.calculateVec(left_pixels[0], right_pixels[0])
					left_circle = cv2.circle(left_circle, (left_pixels[0][0], left_pixels[0][1]), radius, color, thickness)
					right_circle = cv2.circle(right_circle, (right_pixels[0][0], right_pixels[0][1]), radius, color, thickness)

					self.corners3d.append([corner3d[0], corner3d[1], corner3d[2]])
					self.corners_rgb.append([corner3d[0], corner3d[1], corner3d[2], rgb])
					remapped_corner3d = self.remap_point([corner3d[0], corner3d[1], corner3d[2]])
					self.remapped_corners3d.append([remapped_corner3d[0], remapped_corner3d[1], remapped_corner3d[2], rgb])

					self.file_logger.write(("  test_cal.left_pixel({},:,{})  =  [{}, {}];\n").format(corner_idx, self.logger_idx,  left_pixels[0][0]-self.Cx,  left_pixels[0][1]-self.Cy))
					self.file_logger.write(("  test_cal.right_pixel({},:,{}) =  [{}, {}];\n").format(corner_idx, self.logger_idx, right_pixels[0][0]-self.Cx, right_pixels[0][1]-self.Cy))
					self.file_logger.write(("  test_cal.raw.left_pixel({},:,{})  =  [{}, {}];\n").format(corner_idx, self.logger_idx,  left_pixels[0][0],  left_pixels[0][1]))
					self.file_logger.write(("  test_cal.raw.right_pixel({},:,{}) =  [{}, {}];\n").format(corner_idx, self.logger_idx, right_pixels[0][0], right_pixels[0][1]))
					self.file_logger.write(("  test_cal.corner3d({},:,{})   =  [{}, {}, {}];\n").format(corner_idx, self.logger_idx, corner3d[0], corner3d[1], corner3d[2]))
					self.file_logger.write(("  test_cal.remapped3d({},:,{}) =  [{}, {}, {}];\n").format(corner_idx, self.logger_idx, remapped_corner3d[0], remapped_corner3d[1], remapped_corner3d[2]))

				im_circles = cv2.hconcat([left_circle, right_circle])
				cv2.imshow('circles', im_circles)
				cv2.waitKey(1)

				for data in pc2.read_points(self.gator_cloud, skip_nans=False):
					gator_idx += 1
					self.file_logger.write(("  test_cal.gator3d({},:,{})  =  [{}, {}, {}];\n").format(gator_idx, self.logger_idx, data[0], data[1], data[2]))

				gator2ugv_p, gator2ugv_q = self.xf_listener.lookupTransform("vicon/{0}/{0}".format(self.ugv_n), "vicon/gatorboard/gatorboard", rospy.Time())
				self.file_logger.write(  ("  test_cal.gator2ugv_p({},:) =  [{}, {}, {}];\n").format(self.logger_idx, gator2ugv_p[0], gator2ugv_p[1], gator2ugv_p[2]))
				self.file_logger.write(  ("  test_cal.gator2ugv_q({},:) =  [{}, {}, {}, {}];\n").format(self.logger_idx, gator2ugv_q[0], gator2ugv_q[1], gator2ugv_q[2], gator2ugv_q[3]))

				self.ugv_header.stamp = rospy.Time.now()
				# self.ugv_points_pcl = pc2.create_cloud_xyz32(self.ugv_header, self.remapped_corners3d)
				self.ugv_points_pcl = pc2.create_cloud(self.ugv_header, self.fields, self.remapped_corners3d)
				self.ugv_points_pub.publish(self.ugv_points_pcl)

				self.stereo_header.stamp = rospy.Time.now()
				# self.stereo_points_pcl = pc2.create_cloud_xyz32(self.stereo_header, self.corners3d)
				self.stereo_points_pcl = pc2.create_cloud(self.stereo_header, self.fields, self.corners_rgb)
				self.stereo_points_pub.publish(self.stereo_points_pcl)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('test_stereoCal')

	try:
		stereo_test = test_stereoCal()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(10) # 60hz
	while not rospy.is_shutdown():
		stereo_test.publish_gator_cloud()
		stereo_test.calculate_cloud()
		rate.sleep()




# print("saving image: {}").format(image_filename)
#
# self.record_seq += 1
# self.save_frame_rate.sleep() # regulate the frame rate of the saving process
