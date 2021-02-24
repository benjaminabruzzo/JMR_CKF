#!/bin/python
#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
# call as python stereo_calibrate.py 20180209 004
import numpy as np
import cv2
import glob
import sys
import yaml

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
	flags = 0
	# flags |= cv2.CALIB_FIX_INTRINSIC
	# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
	flags |= cv2.CALIB_USE_INTRINSIC_GUESS
	flags |= cv2.CALIB_FIX_FOCAL_LENGTH
	# flags |= cv2.CALIB_FIX_ASPECT_RATIO
	flags |= cv2.CALIB_ZERO_TANGENT_DIST
	# flags |= cv2.CALIB_RATIONAL_MODEL
	# flags |= cv2.CALIB_SAME_FOCAL_LENGTH
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
		stereo_config.img_shape, stereo_config.R, stereo_config.T, alpha=0.10)
	return stereo_config

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


left_img = cv2.imread(LeftImages[0])
right_img = cv2.imread(RightImages[0])
left_gray = cv2.cvtColor(left_img,cv2.COLOR_BGR2GRAY)
right_gray = cv2.cvtColor(right_img,cv2.COLOR_BGR2GRAY)
# Find the chess board corners
left_ret, left_corners = cv2.findChessboardCorners(left_gray, (8,6),None)
right_ret, right_corners = cv2.findChessboardCorners(right_gray, (8,6),None)
if (left_ret and right_ret): # If found, add object points, image points (after refining them)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)
	left_sub_pix = cv2.cornerSubPix(left_gray, left_corners, (11, 11),(-1, -1), criteria)
	right_sub_pix = cv2.cornerSubPix(right_gray, right_corners, (11, 11),(-1, -1), criteria)



def calculateVec(left_pixels, right_pixels):
	# print("left_pixels  = {}").format(left_pixels)
	# print("right_corner = {}").format(right_pixels)
	f = 1289.9524511199999779
	b = 0.1789203784865196
	Cx = 962.6438407900000129
	Cy = 385.9585990909999964
	xl = left_pixels[0] - Cx
	yl = Cy - left_pixels[1]
	xr = right_pixels[0] - Cx
	yr = Cy - right_pixels[1]
	d = abs(xl - xr);
	z = b*f/d;
	y = z*(yl+yr)/(2*f);
	x = z*(xl+xr)/(2*f);
	return [x,y,z]

i = 0
corners3d = []
for left_pix, right_pix in zip(left_sub_pix, right_sub_pix):
	corner3d = calculateVec(left_pix[0], right_pix[0])
	corners3d.append(corner3d)
	i+=1

print("corners3d = {}").format(corners3d)

	print("left_corner[{}] = ({},{})").format(i, left_pix[0][0], left_pix[0][1])
	print("right_corner[{}] = ({},{})").format(i, right_pix[0][0], right_pix[0][1])
	print("pt_vector[{}] = {}").format(i, corner3d)

# calculate rectification maps and rectify images
# print("rectifyImages(stereo_data, LeftImages, RightImages)")
# rectifyImages(cfg, stereo_data, LeftImages, RightImages)

# save data to file
# print("save_calibration(cfg, stereo_data, LeftCamera, RightCamera, LeftImages, RightImages)")
# save_calibration(cfg, stereo_data, LeftCamera, RightCamera, LeftImages, RightImages)
