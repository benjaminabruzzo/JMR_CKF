#include "apriltagclass.hpp"
#include "genheaders.hpp"

apriltagclass::apriltagclass() // void for construction of KF class
{
	id = 0;

	isgoal = false;
	inFOV = false;
	MeasPosition_cam = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	MeasQuaternion_cam = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	MeasPosition_uav = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	MeasPosition_gl = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	PredictedMeasurement_uav = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	PredictedMeasurement_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	PredictedPhi = 0;

	quaternion_xform = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

	measCount = 0;
	MeasYaw_cam = 0;
	MeasYaw_uav = 0;
	MeasYaw_gl = 0;
	EstYaw_gl = 0; //radians
	slamCount = 0;
	vk = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);

	Rk_uav = 1 * (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);

	P = 1 * (cv::Mat_<double>(4, 4) <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0);

	P_trace = 0;
	trace_count = 0;

	I = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	nI = -1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

	initDKF();
}

// cv::Mat getPosteriorEst() {return dKF.PosteriorEst;}
cv::Mat apriltagclass::getMeasPosition_uav() {return MeasPosition_uav;}
cv::Mat apriltagclass::getMeasPosition_gl() {return MeasPosition_gl;}

void apriltagclass::publishStateTF(ros::Time pubstamp)
{//ROS_INFO("uav void publishUAVtf(ros::Time pubstamp)");
	// mfile.writeString("%% --------------- void publishUAVtf(ros::Time pubstamp)  --------------- \n\n" );
		double cosyaw = cos(EstYaw_gl);
		double sinyaw = sin(EstYaw_gl);
		R_TF_gl.setValue(cosyaw, -sinyaw, 0,
									sinyaw,  cosyaw, 0,
									0,0,1);
		R_TF_gl.getRotation(Q_TF_gl);

		TF_gl.setRotation(Q_TF_gl);
		TF_gl.setOrigin( tf::Vector3(EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0)) );
		TF_broadcast.sendTransform(tf::StampedTransform(TF_gl, pubstamp, s_TF_global, s_TF_tag));
}


void apriltagclass::initDKF()
{
	dkf.counter = 1;

	dkf.zk = (cv::Mat_<double>(4, 1) << 0,0,0,0);
	dkf.Dk = (cv::Mat_<double>(4, 1) << 0,0,0,0);
	dkf.PosteriorEst = (cv::Mat_<double>(4, 1) << 0,0,0,0);

	dkf.I  = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	dkf.H  = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	dkf.Fk = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	dkf.Rk = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	dkf.Qk = 0.1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	dkf.PosteriorCov = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

}
