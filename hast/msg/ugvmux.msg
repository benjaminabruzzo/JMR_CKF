float64 stamp 	# time of measurement
uint32 seq			# serial number of message

hast/posewithcov ugvEst			# current estimated pose of ugv
hast/vector4     odomDelta  # ugv odom change

# stereo data
hast/posewithcov stereoMeas	# stereo measurement of uav from ugv
hast/posewithcov stereoUGV	# stereo measurement of ugv from ugv
bool stereoUGV_inFOV				# flag for whether the UGV is in the current FOV
