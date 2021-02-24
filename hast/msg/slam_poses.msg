std_msgs/Header header

bool tag_flag # bool flag whether update was ugv or uav based
posewithcov uav_pose		# current estimated pose of uav

uint8 ugv_id # id of observing ugv
posewithcov[] ugv_poses	# current estimated poses of ugvs
