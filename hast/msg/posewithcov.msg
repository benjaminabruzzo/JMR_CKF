std_msgs/Header header
string vehicle_id # name of vehicle
bool observedByUGV # flag whether or not the ugv has been observed by a cooperating ugv

geometry_msgs/Point position	# 3D position vector
matrix3x3 PCov	# 3D Position covariance matrix

float64 yaw 		# float of yaw
float64 yaw_cov	# estimated direction covariance
