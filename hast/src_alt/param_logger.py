#!/usr/bin/env python

# Import required Python code.
import cv2
import numpy as np
import math

import roslib
import rospy
import yaml

import csv
import itertools
from string import Template

class param_logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		self.exp_date = rospy.get_param('/hast/date')
		self.str_date = str(self.exp_date)

		self.exp_trial = rospy.get_param('/hast/trial')
		self.str_trial = str(self.exp_trial).zfill(3)

		self.file_loggerfilename = ('/home/benjamin/ros/data/{0}/{1}/param_logger_{1}.tex').format(self.str_date,self.str_trial)
		print("param_logger::logging to {}".format(self.file_loggerfilename))
		self.file_logger = open(self.file_loggerfilename, 'w')

		self.trial_host = rospy.get_param('/trial/host')
		self.file_logger.write(("%% /trial/host = {}\n").format(self.trial_host))

		self.jointSLAM = {'uavQdkScale':rospy.get_param('/jointSLAM/uavQdkScale'), \
							'uavQwScale':rospy.get_param('/jointSLAM/uavQwScale'), \
							'ugvQwScale':rospy.get_param('/jointSLAM/ugvQwScale'), \
							'RkScale':rospy.get_param('/jointSLAM/RkScale'), \
							'RkXYZ':rospy.get_param('/jointSLAM/RkXYZ'), \
							'RkYaw':rospy.get_param('/jointSLAM/RkYaw'), \
							}
		self.file_logger.write("%% --------------------\n")
		self.file_logger.write(("%% /jointSLAM/uavQdkScale = {}\n").format(self.jointSLAM['uavQdkScale']))
		self.file_logger.write(("%% /jointSLAM/uavQwScale = {}\n").format(self.jointSLAM['uavQwScale']))
		self.file_logger.write(("%% /jointSLAM/ugvQwScale = {}\n").format(self.jointSLAM['ugvQwScale']))
		self.file_logger.write(("%% /jointSLAM/RkScale = {}\n").format(self.jointSLAM['RkScale']))
		self.file_logger.write(("%% /jointSLAM/RkXYZ = {}\n").format(self.jointSLAM['RkXYZ']))
		self.file_logger.write(("%% /jointSLAM/RkYaw = {}\n").format(self.jointSLAM['RkYaw']))
		self.file_logger.write("%% --------------------\n")

		ugv_str = "1";
		try:
			ugv_host = rospy.get_param('/trial/ugv{}/host'.format(ugv_str))
			self.ugv1 = {'id': ugv_str, \
							'host':ugv_host, \
							'PixelScale':rospy.get_param("/ugv{}_stereoObs_{}/PixelScale".format(ugv_str, ugv_host)), \
							}
			self.file_logger.write(("%% /trial/ugv{}/host = {}\n").format(self.ugv1['id'], self.ugv1['host']))
		except:
			self.file_logger.write(("%% no ugv{}\n").format(ugv_str))
			pass

		ugv_str = "2";
		try:
			ugv_host = rospy.get_param('/trial/ugv{}/host'.format(ugv_str))
			self.file_logger.write(("%% /trial/ugv{}/host = {}\n").format(ugv_str, ugv_host))
			self.ugv1 = {'host':ugv_host, \
							'PixelScale':rospy.get_param("/ugv{}_stereoObs_{}/PixelScale".format(ugv_str, ugv_host)), \
							}
		except:
			self.file_logger.write(("%% no ugv{}\n").format(ugv_str))
			pass

		self.file_logger.write(("\n\n\input{{{0}/{1}/{1}.tex}}\n\n").format(self.str_date, self.str_trial))

		self.file_logger.write(("\n\n\pagebreak\n\subsection*{{date: {}, trial: {}}}\n").format(self.str_date, self.str_trial))
		self.file_logger.write("\\begin{multicols}{2}\n")

		self.file_logger.write("\t\\noindent The pixel uncertainty used in the stereo triangulation:\n\t\\begin{equation}\n")
		self.file_logger.write(("\t\t\Sigma_s = J_s \\begin{{bmatrix}}{0}&0&0 \\\\ 0&{0}&0 \\\\ 0&0&{0} \end{{bmatrix}} J_s^T\n\t\end{{equation}}\n\n").format(self.ugv1['PixelScale']))

		self.file_logger.write("\t\\noindent The odometry uncertainty of the UGV in motion:\n\t\\begin{equation}\n")
		self.file_logger.write(("\t\tQw_{{ugv}} = J_{{ugv}} \\begin{{bmatrix}}{0}&0&0&0\\\\0&{0}&0&0\\\\0&0&{0}&0\\\\0&0&0&{0}\end{{bmatrix}} J_{{ugv}}^T.\n\t\end{{equation}}\n\n").format(self.jointSLAM['ugvQwScale']))

		self.file_logger.write("\t\\noindent The odometry uncertainty of the UAV in motion:\n\t\\begin{equation}\n")
		self.file_logger.write(("\t\tQw_{{uav}} = J_{{uav}} \\begin{{bmatrix}}{0}&0&0&0\\\\0&{0}&0&0\\\\0&0&{0}&0\\\\0&0&0&{0}\end{{bmatrix}} J_{{uav}}^T.\n\t\end{{equation}}\n\n").format(self.jointSLAM['uavQwScale']))

		self.file_logger.write("\t\\noindent The measurement uncertainty of the UAV observing a landmark:\n\t\\begin{equation}\n")
		self.file_logger.write(("\t\tRk_{{tag}} = {0} \cdot \\begin{{bmatrix}}{1}&0&0&0\\\\0&{1}&0&0\\\\0&0&{1}&0\\\\0&0&0&{2}\end{{bmatrix}}.\n\t\end{{equation}}\n\n").format(\
				self.jointSLAM['RkScale'], \
				self.jointSLAM['RkXYZ'], \
				self.jointSLAM['RkYaw']))

		self.file_logger.write("\\end{multicols}\n")
		self.file_logger.write("\\begin{figure}[h]\n")
		self.file_logger.write("\t\\begin{tabular}{cc}\n")
		self.file_logger.write("\t\t\\begin{subfigure}[b]{0.5\\textwidth}\n")
		self.file_logger.write(("\t\t\t\\includegraphics[width=\\textwidth]{{{0}/{1}/uav_ugv_eigenvalues_{1}.eps}}\n").format(self.str_date, self.str_trial))
		self.file_logger.write("\t\t\t\caption{{\\ UAV/UGV}} \label{{fig:uav_ugv_{}}}\n".format(self.str_trial))
		self.file_logger.write("\t\t\end{subfigure} &\n")
		self.file_logger.write("\t\t\\begin{subfigure}[b]{0.5\\textwidth}\n")
		self.file_logger.write(("\t\t\t\\includegraphics[width=\\textwidth]{{{0}/{1}/uav_tag_eigenvalues_{1}.eps}}\n").format(self.str_date, self.str_trial))
		self.file_logger.write("\t\t\t\caption{{\\ UAV/TAG}} \label{{fig:uav_tag_{}}}\n".format(self.str_trial))
		self.file_logger.write("\t\t\end{subfigure} &\n")
		self.file_logger.write("\t\end{tabular}\n")
		self.file_logger.write("\t\centering\n")
		self.file_logger.write(("\t\t\\includegraphics[width=0.75\\textwidth]{{{0}/{1}/system_eigenvalues_{1}.eps}}\n").format(self.str_date, self.str_trial))
		self.file_logger.write("\t\t\caption{{System eigenvalues (some negative).}}\label{{fig:uav_tag_{}}}\n".format(self.str_trial))
		self.file_logger.write("\end{figure}\n")


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('param_logger')
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		log = param_logger()
	except rospy.ROSInterruptException: pass



# // the measurement uncertainty of the UAV observing a landmark
# // \begin{equation}
# //
# // \end{equation}
