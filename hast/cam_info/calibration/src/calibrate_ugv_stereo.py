#!/bin/python
#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
# call as python stereo_calibrate.py 20180209 004
import numpy as np
import cv2, yaml
import sys, math, glob

def makeHomVec(vec):
	return np.concatenate([vec, [1]])

def column(row):
	column = np.array(row)
	return column[:, np.newaxis]

def wrapH(R, t):
	H = np.concatenate([R, -R.dot(t)], axis=1)
	hom = np.array([0,0,0,1])
	H = np.vstack([H, hom])
	return H

def unwrapH(H):
	R = H[:3, :3].copy()
	H_t = H[:3, 3:4].copy()
	t = -np.transpose(R).dot(H_t)
	return R, t

def invertH(H):
	R = H[:3, :3].copy()
	H_t = H[:3, 3:4].copy()
	return wrapH(np.transpose(R), H_t)

def nxm2mfile(fileservice, data, datastring): # write data to file
	for i in range(0,data.shape[0]):
		line = datastring + "(" + str(i+1) + ",:)=["
		for j in range(0,data.shape[1]):
			line += (str(data[i,j])) + " "
		line += "];\n"
		fileservice.write(line)
	fileservice.write("\n")

def makeHforCalOuts(R,t):
	H = np.concatenate((R, t), axis=1)
	H = np.concatenate((H, np.matrix('0 0 0 1')), axis=0)
	return H

def calibrateUsingImages(images):
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*8,3), np.float32)
	objp[:,:2] = 0.111*np.mgrid[0:8,0:6].T.reshape(-1,2)
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	# print(images)
	for fname in images:
		# fname=images[0]
		# print(fname)
		img = cv2.imread(fname)
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
		if ret == True: # If found, add object points, image points (after refining them)
			# cv2.cornerSubPix
			rt = cv2.cornerSubPix(gray, corners, (11, 11),(-1, -1), criteria)
			objpoints.append(objp)
			imgpoints.append(corners)
		else:
			print("bad grid: {}").format(fname)
	ret, cameraMatrix, distortionCoeffs, rotationVecs, translationVecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
	camera_model = dict([('M', cameraMatrix), ('dist', distortionCoeffs),('rvecs', rotationVecs), ('tvecs', translationVecs), ('ret', ret),
						('img_shape', gray.shape[::-1]), ('objp', objp), ('objpoints', objpoints), ('imgpoints', imgpoints)])
	return camera_model
	# I think ret is the reprojection error

def stereo_calibrate(LeftDict, RightDict):
	# 	I fixed it by changing the stereoCalibrate flags. For example using: cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6
	# Another thing that gave larger RMS error, but at least a correct remap was calibrating intrinsics and extrinsics together. So no cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_INTRINSIC

	flags = 0
	# flags |= cv2.CALIB_FIX_INTRINSIC
	# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
	flags |= cv2.CALIB_USE_INTRINSIC_GUESS
	# flags |= cv2.CALIB_FIX_FOCAL_LENGTH
	# flags |= cv2.CALIB_FIX_ASPECT_RATIO
	flags |= cv2.CALIB_ZERO_TANGENT_DIST
	# flags |= cv2.CALIB_RATIONAL_MODEL
	flags |= cv2.CALIB_SAME_FOCAL_LENGTH
	# flags |= cv2.CALIB_FIX_K3
	# flags |= cv2.CALIB_FIX_K4
	# flags |= cv2.CALIB_FIX_K5
	stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

	Ldist = LeftDict["dist"]
	Rdist = RightDict["dist"]
	LM = LeftDict["M"]
	RM = RightDict["M"]
	objpts = LeftDict["objpoints"]
	img_shape = LeftDict["img_shape"]
	Limgpts = LeftDict["imgpoints"]
	Rimgpts = RightDict["imgpoints"]
	ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(objpts, Limgpts, Rimgpts, LM, Ldist, RM, Rdist, img_shape, criteria=stereocalib_criteria, flags=flags)
	stereo_model = dict([('LM', M1), ('RM', M2), ('Ldist', d1), ('Rdist', d2),
						('Lrvecs', LeftDict["rvecs"]), ('Rrvecs', RightDict["rvecs"]),
						('img_shape', img_shape),
						('R', R), ('T', T),
						('E', E), ('F', F)])
	return stereo_model

def save_calibration(cfg, stereoData, LeftDict, RightDict, LeftImages, RightImages):
	print("open({}, 'w')").format(cfg['path']+cfg['stereo_caldata'])
	f = open(cfg['path']+cfg['stereo_caldata'], 'w')

	# Stereo Data
	nxm2mfile(f,stereoData.left_intrinsics,  "caldata.stereo.left.M")
	nxm2mfile(f,stereoData.right_intrinsics, "caldata.stereo.right.M")
	nxm2mfile(f,stereoData.left_distortion,  "caldata.stereo.left.dist")
	nxm2mfile(f,stereoData.right_distortion, "caldata.stereo.right.dist")
	nxm2mfile(f,stereoData.left_R,  "caldata.stereo.left.R")
	nxm2mfile(f,stereoData.right_R, "caldata.stereo.right.R")
	nxm2mfile(f,stereoData.left_P,  "caldata.stereo.left.P")
	nxm2mfile(f,stereoData.right_P, "caldata.stereo.right.P")
	nxm2mfile(f,stereoData.R, "caldata.stereo.R")
	nxm2mfile(f,stereoData.T, "caldata.stereo.T")
	nxm2mfile(f,stereoData.E, "caldata.stereo.E")
	nxm2mfile(f,stereoData.F, "caldata.stereo.F")
	nxm2mfile(f,stereoData.Q, "caldata.stereo.Q")

	# Left Data
	f.write(("caldata.left.serial = '{}';\n").format(cfg['left_serial']))
	nxm2mfile(f,LeftDict['objp'], "caldata.left.chessboard3Dpoints")
	nxm2mfile(f,LeftDict['M'], "caldata.left.cameraMatrix")
	nxm2mfile(f,LeftDict['dist'], "caldata.left.distortionCoeffs")
	f.write(("caldata.left.ret = {};\n").format(LeftDict['ret']))


	for v,t,img in zip(LeftDict['rvecs'],LeftDict['tvecs'], LeftImages):
		R,J = cv2.Rodrigues(v)
		R = np.matrix(R)
		t = np.matrix(t)
		H = makeHforCalOuts(R,t)
		f.write(("caldata.left.t(:,:,{}) = [{}, {}, {}]';\n").format(int(img[-7:-4]), t.item(0,0), t.item(1,0), t.item(2,0)))
		f.write(("caldata.left.R(:,:,{}) = [{},{},{}; {},{},{}; {},{},{}];\n").format(int(img[-7:-4]), \
												R.item(0,0),R.item(0,1),R.item(0,2),\
												R.item(1,0),R.item(1,1),R.item(1,2),\
												R.item(2,0),R.item(2,1),R.item(2,2)))
		f.write(("caldata.left.H(:,:,{}) = [{},{},{},{}; {},{},{},{}; {},{},{},{}; {},{},{},{}];\n").format(int(img[-7:-4]), \
												H.item(0,0),H.item(0,1),H.item(0,2),H.item(0,3),\
												H.item(1,0),H.item(1,1),H.item(1,2),H.item(1,3),\
												H.item(2,0),H.item(2,1),H.item(2,2),H.item(2,3),\
												H.item(3,0),H.item(3,1),H.item(3,2),H.item(3,3)))

	# Right Data
	f.write(("caldata.right.serial = '{}';\n").format(cfg['right_serial']))
	nxm2mfile(f,RightDict['objp'], "caldata.right.chessboard3Dpoints")
	nxm2mfile(f,RightDict['M'], "caldata.right.cameraMatrix")
	nxm2mfile(f,RightDict['dist'], "caldata.right.distortionCoeffs")
	f.write(("caldata.right.ret = {};\n").format(RightDict['ret']))

	for v,t,img in zip(RightDict['rvecs'],RightDict['tvecs'], RightImages):
		R,J = cv2.Rodrigues(v)
		R = np.matrix(R)
		t = np.matrix(t)
		H = makeHforCalOuts(R,t)
		f.write(("caldata.right.t(:,:,{}) = [{}, {}, {}]';\n").format(int(img[-7:-4]), t.item(0,0), t.item(1,0), t.item(2,0)))
		f.write(("caldata.right.R(:,:,{}) = [{}, {}, {}; {}, {}, {}; {}, {}, {}];\n").format(int(img[-7:-4]), \
												R.item(0,0),R.item(0,1),R.item(0,2),\
												R.item(1,0),R.item(1,1),R.item(1,2),\
												R.item(2,0),R.item(2,1),R.item(2,2)))
		f.write(("caldata.right.H(:,:,{}) = [{},{},{},{}; {},{},{},{}; {},{},{},{}; {},{},{},{}];\n").format(int(img[-7:-4]), \
												H.item(0,0),H.item(0,1),H.item(0,2),H.item(0,3),\
												H.item(1,0),H.item(1,1),H.item(1,2),H.item(1,3),\
												H.item(2,0),H.item(2,1),H.item(2,2),H.item(2,3),\
												H.item(3,0),H.item(3,1),H.item(3,2),H.item(3,3)))
	f.close()

	print("open({}/{}.yaml, 'w')").format(cfg['path'], cfg['left_serial'])
	left_file = open("{}/{}.yaml".format(cfg['path'], cfg['left_serial']), 'w')
	left_file.write(("# {}/{} stereo-left parameters\n").format(cfg['ugv_serial'], cfg['ugv_n']))
	left_file.write(("# CAMERA SERIAL {}\n").format(cfg['left_serial']))
	left_file.write( "# yaml file auto-generated by : calibrate_ugv_stereo.py\n")
	left_file.write(("# yaml file auto-generated on : {}\n").format(cfg['date']))
	left_file.write( "# #### TAB CANNOT BE USED #####\n\n")
	left_file.write(("image_width: {}\n").format(stereoData.img_shape[0]))
	left_file.write(("image_height: {}\n\n").format(stereoData.img_shape[1]))
	left_file.write( "camera_matrix:\n  rows: 3\n  cols: 3\n")
	left_file.write(("  data: [{},{},{}, {},{},{}, {},{},{}]\n\n").format(\
												stereoData.left_intrinsics[0][0], stereoData.left_intrinsics[0][1], stereoData.left_intrinsics[0][2],\
												stereoData.left_intrinsics[1][0], stereoData.left_intrinsics[1][1], stereoData.left_intrinsics[1][2],\
												stereoData.left_intrinsics[2][0], stereoData.left_intrinsics[2][1], stereoData.left_intrinsics[2][2]))
	left_file.write( "distortion_coefficients:\n  rows: 1\n  cols: 5\n")
	left_file.write(("  data: [{}, {}, {}, {}, {}]\n\n").format(\
												stereoData.left_distortion[0][0],\
												stereoData.left_distortion[0][1],\
												stereoData.left_distortion[0][2],\
 												stereoData.left_distortion[0][3],\
												stereoData.left_distortion[0][4]))
	left_file.write( "rectification_matrix:\n  rows: 3\n  cols: 3\n")
	left_file.write(("  data: [{},{},{}, {},{},{}, {},{},{}]\n\n").format(\
												stereoData.left_R[0][0], stereoData.left_R[0][1], stereoData.left_R[0][2],\
												stereoData.left_R[1][0], stereoData.left_R[1][1], stereoData.left_R[1][2],\
												stereoData.left_R[2][0], stereoData.left_R[2][1], stereoData.left_R[2][2]))
	left_file.write( "projection_matrix:\n  rows: 3\n  cols: 4\n")
	left_file.write(("  data: [{},{},{},{}, {},{},{},{}, {},{},{},{}]\n\n").format(\
												stereoData.left_P[0][0], stereoData.left_P[0][1], stereoData.left_P[0][2], stereoData.left_P[0][3],\
												stereoData.left_P[1][0], stereoData.left_P[1][1], stereoData.left_P[1][2], stereoData.left_P[1][3],\
												stereoData.left_P[2][0], stereoData.left_P[2][1], stereoData.left_P[2][2], stereoData.left_P[2][3]))
	left_file.write( "distortion_model: plumb_bob\n")
	left_file.write( "camera_info_url: file:///home/benjamin/.ros/camera_info/${NAME}.yaml\n\n")
	left_file.write( "auto_white_balance: false\n")
	left_file.write( "white_balance_blue: 525\n")
	left_file.write( "white_balance_red: 270\n")
	left_file.write( "enable_trigger: true\n")
	left_file.write( "trigger_mode: mode14\n\n")
	left_file.write( "auto_gain: false\n")
	left_file.write( "gain: 8.375\n")
	left_file.write( "auto_exposure: false\n")
	left_file.write( "exposure: 0.5\n")
	left_file.write( "auto_shutter: false\n")
	left_file.write( "shutter_speed: 0.002\n")
	left_file.close()

	print("open({}/{}.yaml, 'w')").format(cfg['path'], cfg['right_serial'])
	right_file = open("{}/{}.yaml".format(cfg['path'], cfg['right_serial']), 'w')
	right_file.write(("# {}/{} stereo-right parameters\n").format(cfg['ugv_serial'], cfg['ugv_n']))
	right_file.write(("# CAMERA SERIAL {}\n").format(cfg['right_serial']))
	right_file.write( "# yaml file auto-generated by : calibrate_ugv_stereo.py\n")
	right_file.write(("# yaml file auto-generated on : {}\n").format(cfg['date']))
	right_file.write( "# #### TAB CANNOT BE USED #####\n\n")
	right_file.write(("image_width: {}\n").format(stereoData.img_shape[0]))
	right_file.write(("image_height: {}\n\n").format(stereoData.img_shape[1]))
	right_file.write( "camera_matrix:\n  rows: 3\n  cols: 3\n")
	right_file.write(("  data: [{},{},{}, {},{},{}, {},{},{}]\n\n").format(\
												stereoData.right_intrinsics[0][0], stereoData.right_intrinsics[0][1], stereoData.right_intrinsics[0][2],\
												stereoData.right_intrinsics[1][0], stereoData.right_intrinsics[1][1], stereoData.right_intrinsics[1][2],\
												stereoData.right_intrinsics[2][0], stereoData.right_intrinsics[2][1], stereoData.right_intrinsics[2][2]))
	right_file.write( "distortion_coefficients:\n  rows: 1\n  cols: 5\n")
	right_file.write(("  data: [{}, {}, {}, {}, {}]\n\n").format(\
												stereoData.right_distortion[0][0],\
												stereoData.right_distortion[0][1],\
												stereoData.right_distortion[0][2],\
												stereoData.right_distortion[0][3],\
												stereoData.right_distortion[0][4]))
	right_file.write( "rectification_matrix:\n  rows: 3\n  cols: 3\n")
	right_file.write(("  data: [{},{},{}, {},{},{}, {},{},{}]\n\n").format(\
												stereoData.right_R[0][0], stereoData.right_R[0][1], stereoData.right_R[0][2],\
												stereoData.right_R[1][0], stereoData.right_R[1][1], stereoData.right_R[1][2],\
												stereoData.right_R[2][0], stereoData.right_R[2][1], stereoData.right_R[2][2]))
	right_file.write( "projection_matrix:\n  rows: 3\n  cols: 4\n")
	right_file.write(("  data: [{},{},{},{}, {},{},{},{}, {},{},{},{}]\n\n").format(\
												stereoData.right_P[0][0], stereoData.right_P[0][1], stereoData.right_P[0][2], stereoData.right_P[0][3],\
												stereoData.right_P[1][0], stereoData.right_P[1][1], stereoData.right_P[1][2], stereoData.right_P[1][3],\
												stereoData.right_P[2][0], stereoData.right_P[2][1], stereoData.right_P[2][2], stereoData.right_P[2][3]))
	right_file.write( "distortion_model: plumb_bob\n")
	right_file.write( "camera_info_url: file:///home/benjamin/.ros/camera_info/${NAME}.yaml\n\n")
	right_file.write( "auto_white_balance: false\n")
	right_file.write( "white_balance_blue: 525\n")
	right_file.write( "white_balance_red: 270\n")
	right_file.write( "enable_trigger: true\n")
	right_file.write( "trigger_mode: mode14\n\n")
	right_file.write( "auto_gain: false\n")
	right_file.write( "gain: 8.375\n")
	right_file.write( "auto_exposure: false\n")
	right_file.write( "exposure: 0.5\n")
	right_file.write( "auto_shutter: false\n")
	right_file.write( "shutter_speed: 0.002\n")
	right_file.close()

class stereo_class(object):
	def __init__(self, stereoDict):
		self.left_intrinsics = stereoDict['LM']
		self.right_intrinsics = stereoDict['RM']
		self.left_distortion = stereoDict['Ldist']
		self.right_distortion = stereoDict['Rdist']
		self.img_shape = stereoDict['img_shape']
		self.R = stereoDict['R']
		self.T = stereoDict['T']
		self.E = stereoDict['E']
		self.F = stereoDict['F']
		self.left_R = [] # roation applied to image to move into projective plane
		self.right_R = []  # roation applied to image to move into projective plane
		self.left_P = [] # left projection matrix
		self.right_P = [] # right projection matrix
		self.Q = []
		self.left_mapx = [] # maps for computing recitifed images
		self.left_mapy = [] # maps for computing recitifed images

def stereo_rectify(stereo_config):
	stereo_config.left_R, stereo_config.right_R, stereo_config.left_P, stereo_config.right_P, stereo_config.Q, roi1, roi2 = cv2.stereoRectify(
		stereo_config.left_intrinsics, stereo_config.left_distortion,
		stereo_config.right_intrinsics, stereo_config.right_distortion,
		stereo_config.img_shape, stereo_config.R, stereo_config.T, alpha=0)
	return stereo_config

def rectifyImages(cfg, stereoData, LeftImages, RightImages):
	# https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html?highlight=initundistort#initundistortrectifymap
	# cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, size, m1type[, map1[, map2]])
	# Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
	stereoData.left_mapx = np.ndarray(shape=(stereoData.img_shape[1], stereoData.img_shape[0], 1), dtype='float32')
	stereoData.left_mapy = np.ndarray(shape=(stereoData.img_shape[1], stereoData.img_shape[0], 1), dtype='float32')
	stereoData.right_mapx = np.ndarray(shape=(stereoData.img_shape[1], stereoData.img_shape[0], 1), dtype='float32')
	stereoData.right_mapy = np.ndarray(shape=(stereoData.img_shape[1], stereoData.img_shape[0], 1), dtype='float32')
	cv2.initUndistortRectifyMap(stereoData.left_intrinsics, \
															stereoData.left_distortion, \
															stereoData.left_R, \
															stereoData.left_P, \
															stereoData.img_shape,
															cv2.CV_32FC1, \
															stereoData.left_mapx, \
															stereoData.left_mapy)
	cv2.initUndistortRectifyMap(stereoData.right_intrinsics, \
															stereoData.right_distortion, \
															stereoData.right_R, \
															stereoData.right_P, \
															stereoData.img_shape,
															cv2.CV_32FC1, \
															stereoData.right_mapx, \
															stereoData.right_mapy)
	# for all images:
	LeftRectImages = []
	for fname in LeftImages:
		rect_name = ("{}/rect/left_rect_{}").format(fname[:-24], fname[-10:])
		LeftRectImages.append(rect_name)
		left_raw = cv2.imread(fname)
		left_rectified = cv2.remap(left_raw, stereoData.left_mapx, stereoData.left_mapy, cv2.INTER_CUBIC)
		try:
			check_left = cv2.imwrite(rect_name, left_rectified)
		except cv2.error as e:
			print(e)

	RightRectImages = []
	for fname in RightImages:
		rect_name = ("{}/rect/right_rect_{}").format(fname[:-25], fname[-10:])
		RightRectImages.append(rect_name)
		right_raw = cv2.imread(fname)
		right_rectified = cv2.remap(right_raw, stereoData.right_mapx, stereoData.right_mapy, cv2.INTER_CUBIC)
		try:
			check_right = cv2.imwrite(rect_name, right_rectified)
		except cv2.error as e:
			print(e)

	# open file for recording pixel data
	print("rect_file = open(cfg['stereo_rectdata'], 'w') = open({}, 'w')").format(cfg['path']+cfg['stereo_rectdata'])
	rect_file = open(cfg['path']+cfg['stereo_rectdata'], 'w')

	# determine which rectified images can be used to calculate ugv base TF
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)
	test_images = []
	for leftname, rightname in zip(LeftRectImages, RightRectImages):
		# load rectified images
		left_rect_img = cv2.imread(leftname)
		right_rect_img = cv2.imread(rightname)
		left_grey_img = cv2.cvtColor(left_rect_img,cv2.COLOR_BGR2GRAY)
		right_grey_img = cv2.cvtColor(right_rect_img,cv2.COLOR_BGR2GRAY)

		# attempt to find checkerboard in both images
		left_ret, left_corners = cv2.findChessboardCorners(left_grey_img, (8,6),None)
		right_ret, right_corners = cv2.findChessboardCorners(right_grey_img, (8,6),None)


		if left_ret and right_ret: #then both images have a checkerboard
			# save images for later
			try:
				left_savename = leftname[:-26] + "/test_rect/left_rect_" + leftname[-10:]
				# print(left_savename)
				check_left = cv2.imwrite(left_savename, left_rect_img)
			except cv2.error as e:
				print(e)
			try:
				right_savename = rightname[:-27] + "/test_rect/right_rect_" + rightname[-10:]
				# print(right_savename)
				check_left = cv2.imwrite(right_savename, right_rect_img)
			except cv2.error as e:
				print(e)

			# create a list of which images were kept:
			test_images.append(int(leftname[-10:-4]))
			# print(test_images)

			left_corners_subpix = cv2.cornerSubPix(left_grey_img, left_corners, (11, 11),(-1, -1), criteria)
			right_corners_subpix = cv2.cornerSubPix(right_grey_img, right_corners, (11, 11),(-1, -1), criteria)

			i = 0
			for left_pix, right_pix in zip(left_corners_subpix, right_corners_subpix):
				# print(("rectdata.rect_{}.pixels.left.xy(:,{}) = [{}, {}]").format(leftname[-7:-4], i+1, left_corners[i][0][0], left_corners[i][0][1]))
				rect_file.write(("rectdata.rect_{}.pixels.left.xy({},:) = [{}, {}];\n").format(leftname[-7:-4], i+1, left_pix[0][0], left_pix[0][1]))
				rect_file.write(("rectdata.rect_{}.pixels.right.xy({},:) = [{}, {}];\n").format(leftname[-7:-4], i+1, right_pix[0][0], right_pix[0][1]))
				i+=1
			rect_file.write("\n")

	i = 1
	# print(test_images)
	for idx in test_images:
		rect_file.write(("rectdata.rect_indices({},1) = {};\n").format(i, idx))
		i += 1
	rect_file.close()

def main():
	# Setup config
	with open("/home/benjamin/ros/src/metahast/hast/cam_info/calibration/yaml/calibrate_ugv_stereo.yaml", 'r') as ymlfile:
		if sys.version_info[0] > 2:
			cfg = yaml.load(ymlfile, Loader=yaml.FullLoader)
		else:
			cfg = yaml.load(ymlfile)

	# glob into list all of the left and right images
	print("directory of calibration images: {}").format(cfg['path'] + cfg['imgdir'])
	LeftImages = sorted(glob.glob(cfg['path'] + cfg['imgdir'] + '/left*.png'))
	RightImages = sorted(glob.glob(cfg['path'] + cfg['imgdir'] + '/right*.png'))

	# use images to generate camera models
	print("LeftCamera = calibrateUsingImages(LeftImages)")
	LeftCamera = calibrateUsingImages(LeftImages)
	print("RightCamera = calibrateUsingImages(RightImages)")
	RightCamera = calibrateUsingImages(RightImages)

	# use camera models to generate stereo model
	print("stereoModel = stereo_calibrate(LeftCamera, RightCamera)")
	stereoModel = stereo_calibrate(LeftCamera, RightCamera)
	print("stereo_data = stereo_class(stereoModel)")
	stereo_data = stereo_class(stereoModel)
	print("stereo_data = stereo_rectify(stereo_data)")
	stereo_data = stereo_rectify(stereo_data)

	# calculate rectification maps and rectify images
	print("rectifyImages(stereo_data, LeftImages, RightImages)")
	rectifyImages(cfg, stereo_data, LeftImages, RightImages)

	# save data to file
	print("save_calibration(cfg, stereo_data, LeftCamera, RightCamera, LeftImages, RightImages)")
	save_calibration(cfg, stereo_data, LeftCamera, RightCamera, LeftImages, RightImages)


if __name__ == "__main__":
	main()
