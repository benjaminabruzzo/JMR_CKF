#include "hastUGV.hpp"

hastUGV::hastUGV()
{
	// ROS comms
		stateMsg_id = 0;
		init_time = 0;

	// Possibly useful constants
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function

	// Initial estimated states of ugv
		EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		EstYaw_gl = 0;
			// cosyaw = cos(Pi * EstYaw_gl / 180);
			// sinyaw = sin(Pi * EstYaw_gl / 180);
			cosyaw = cos( EstYaw_gl );
			sinyaw = sin( EstYaw_gl );
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
						   cosyaw, sinyaw, 0,
						  -sinyaw, cosyaw, 0,
						   0, 0, 1);
			Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
					   cosyaw, sinyaw, 0, 0,
					  -sinyaw, cosyaw, 0, 0,
					   0, 0, 1, 0,
					   0, 0, 0, 1);
			Rlo2gl4x4 = (cv::Mat_<double>(4, 4) <<
							cosyaw, -sinyaw, 0, 0,
							sinyaw, cosyaw, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1);

		// Hgl2lo transforms a point in the global frame into the tb frame
			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

		velStamp = 0;
		velStamplast = 0;
		velCounter = 0;
		estCounter = 0;

		slamtime = 0;
		slamtimelast = 0;

		correction_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		yawCorrection = 0;

		//complementary KF class init
		Qw_scale = 1;
		ckf.Qw = Qw_scale*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); // the last term is zero because the ugv doesnt contribute to the yaw bias
		ckf.Fk =  (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
		ckf.Qdk = (cv::Mat_<double>(4, 4) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);

	/*-----  Initialize Wheel Odometry */
		odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0,0);
		odom_gl = (cv::Mat_<double>(4, 1) << 0,0,0,0);
		wheelTime = 0;
		lastWheelTime = 0;
		wheelcount = 0;
		wheelTimeStamp = ros::Time::now();

		wheelyaw_q = 0;

		// using odom twist message:
			wheel_yaw_rate = 0;
			wheel_twist_linear = (cv::Mat_<double>(3, 1) << 0, 0, 0);
			wheel_twist_angular = (cv::Mat_<double>(3, 1) << 0, 0, 0);

		// reduce datawrite from vel commands
		vel_dt_aggregator = 0;
}

void hastUGV::initDKF()
{
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


void hastUGV::updatePoseFromCKF(cv::Mat ckf_correction_gl, double ckf_yaw_correction)
{
	// remove the prevous correction
	EstYaw_gl      += yawCorrection;
	EstPosition_gl += correction_gl;

	// save the new correction
	yawCorrection = ckf_yaw_correction;
	correction_gl = (cv::Mat_<double>(3, 1) <<
					ckf_correction_gl.at<double>(0,0),
					ckf_correction_gl.at<double>(1,0),
					ckf_correction_gl.at<double>(2,0));

	// apply correction to estimated pose
	EstYaw_gl      -= yawCorrection;
	EstPosition_gl -= correction_gl;

	cosyaw = cos( EstYaw_gl );
	sinyaw = sin( EstYaw_gl );
	Rgl2lo = (cv::Mat_<double>(3, 3) <<
					 cosyaw, sinyaw, 0,
					-sinyaw, cosyaw, 0,
					 0, 0, 1);
	Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
				 cosyaw, sinyaw, 0, 0,
				-sinyaw, cosyaw, 0, 0,
				 0, 0, 1, 0,
				 0, 0, 0, 1);
	Rlo2gl4x4 = (cv::Mat_<double>(4, 4) <<
					cosyaw, -sinyaw, 0, 0,
					sinyaw, cosyaw, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1);

	// Hgl2lo transforms a point in the global frame into the ugv frame
	Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
	Hlo2gl = invertH(Hgl2lo);

	ckf.Qdk = (cv::Mat_<double>(4, 4) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);  // reset after incrementing
}

void hastUGV::openmFile()
{
	ros::Duration(0.1).sleep();
	fprintf (data.filename, "%% --------------- openmFile() --------------- \n" );
	fprintf (data.filename, "%s.init_time = %f;\n", s_data_filename.c_str(), init_time);
	fprintf (data.filename, "  %s.ckfinit.Qw(1,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qw.at<double>(0, 0), ckf.Qw.at<double>(0, 1), ckf.Qw.at<double>(0, 2), ckf.Qw.at<double>(0, 3));
	fprintf (data.filename, "  %s.ckfinit.Qw(2,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qw.at<double>(1, 0), ckf.Qw.at<double>(1, 1), ckf.Qw.at<double>(1, 2), ckf.Qw.at<double>(1, 3));
	fprintf (data.filename, "  %s.ckfinit.Qw(3,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qw.at<double>(2, 0), ckf.Qw.at<double>(2, 1), ckf.Qw.at<double>(2, 2), ckf.Qw.at<double>(2, 3));
	fprintf (data.filename, "  %s.ckfinit.Qw(4,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qw.at<double>(3, 0), ckf.Qw.at<double>(3, 1), ckf.Qw.at<double>(3, 2), ckf.Qw.at<double>(3, 3));

	fprintf (data.filename, "  %s.ckfinit.Qdk(1,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qdk.at<double>(0, 0), ckf.Qdk.at<double>(0, 1), ckf.Qdk.at<double>(0, 2), ckf.Qdk.at<double>(0, 3));
	fprintf (data.filename, "  %s.ckfinit.Qdk(2,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qdk.at<double>(1, 0), ckf.Qdk.at<double>(1, 1), ckf.Qdk.at<double>(1, 2), ckf.Qdk.at<double>(1, 3));
	fprintf (data.filename, "  %s.ckfinit.Qdk(3,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qdk.at<double>(2, 0), ckf.Qdk.at<double>(2, 1), ckf.Qdk.at<double>(2, 2), ckf.Qdk.at<double>(2, 3));
	fprintf (data.filename, "  %s.ckfinit.Qdk(4,:) = [%-10.8f, %-10.8f, %-10.8f, %-10.8f];\n", s_data_filename.c_str(), ckf.Qdk.at<double>(3, 0), ckf.Qdk.at<double>(3, 1), ckf.Qdk.at<double>(3, 2), ckf.Qdk.at<double>(3, 3));
	ros::Duration(0.1).sleep();
}

void hastUGV::posePublisher(double timestamp)
{
	state_msg.stamp = timestamp;
	state_msg.id = ++stateMsg_id;
	state_msg.P.x = EstPosition_gl.at<double>(0, 0);
	state_msg.P.y = EstPosition_gl.at<double>(0, 1);
	state_msg.P.z = EstPosition_gl.at<double>(0, 2);
	state_msg.yaw = EstYaw_gl;
	state_pub.publish(state_msg);

	fprintf (data.filename, "\n%% ---- posePublisher ---- \n" );
	fprintf (data.filename, "  %s.est.time(%d,1) = %-10.8f;\n", s_data_filename.c_str(), ++estCounter, timestamp - init_time);
	fprintf (data.filename, "  %s.est.Position_gl(%u,:) = [%-10.8f %-10.8f %-10.8f];\n", s_data_filename.c_str(), estCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
	fprintf (data.filename, "  %s.est.Yaw(%d,1) = %-10.8f;\n", s_data_filename.c_str(), estCounter, EstYaw_gl); //degrees
}

void hastUGV::WheelOdometry(const nav_msgs::Odometry::ConstPtr& osub)
{

	// if ((dkf.counter > 0) && (s_ugvn.compare("ugv2") == 0)) {fprintf(stdout, (BLUE_TEXT "%s :: void hastUGV::WheelOdometry()\n" COLOR_RESET), s_data_filename.c_str());}

	// ROS_WARN("WheelOdometry Called");
	wheelTimeStamp = osub->header.stamp;
	wheelTime = wheelTimeStamp.toSec();

	if (lastWheelTime != wheelTime)
	// if (lastWheelTime < wheelTime)
	{//new data
		// ROS_INFO("ugvRecorder: wheel update");
			wheel_dt = wheelTime - lastWheelTime;
			if (wheel_dt>0.5)// suppress huge velocity estimates if motion starts from zero
				{wheel_dt=0.05;} // 0.1 is the mean dt of experiments on 20170922

			lastWheelTime = wheelTime;

		// update new yaw measurement
			wheel_twist_linear  = (cv::Mat_<double>(3, 1) << osub->twist.twist.linear.x,  osub->twist.twist.linear.y,  osub->twist.twist.linear.z);
			wheel_twist_angular = (cv::Mat_<double>(3, 1) << osub->twist.twist.angular.x, osub->twist.twist.angular.y, osub->twist.twist.angular.z);
			wheel_yaw_rate = osub->twist.twist.angular.z; // negative for some reason

			double x = osub -> pose.pose.orientation.x;
			double y = osub -> pose.pose.orientation.y;
			double z = osub -> pose.pose.orientation.z;
			double w = osub -> pose.pose.orientation.w;

			wheeldeltayaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)) - wheelyaw_q;
			wheelyaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)); // radians
			// wheeldeltayaw_q = 2*acos(osub -> pose.pose.orientation.w) - wheelyaw_q;
			// wheelyaw_q = 2*acos(osub -> pose.pose.orientation.w); // radians

		//update estimate of ugv yaw
			// EstYaw_gl += wheel_yaw_rate*wheel_dt*180/Pi;  // rad/sec --> deg/sec?
			EstYaw_gl += wheeldeltayaw_q; // *180/Pi;  // rad/sec --> deg/sec?
			// cosyaw = cos(Pi * EstYaw_gl / 180);
			// sinyaw = sin(Pi * EstYaw_gl / 180);
			cosyaw = cos( EstYaw_gl );
			sinyaw = sin( EstYaw_gl );
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
							 cosyaw, sinyaw, 0,
							-sinyaw, cosyaw, 0,
										0,0,1);
			Rlo2gl4x4 = (cv::Mat_<double>(4, 4) <<
							 cosyaw, -sinyaw, 0, 0,
							 sinyaw, cosyaw, 0, 0,
							 0, 0, 1, 0,
							 0, 0, 0, 1);
		// Velocity update
			MeasuredVel_lo = (cv::Mat_<double>(3, 1) << osub->twist.twist.linear.x, 0, 0); // forward/backward velocity only
			MeasuredVel_gl = Rgl2lo.t() * MeasuredVel_lo;
			EstPosition_gl += MeasuredVel_gl * wheel_dt;

			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

			odom_delta.at<double>(0,0) += MeasuredVel_gl.at<double>(0,0)*wheel_dt;
			odom_delta.at<double>(1,0) += MeasuredVel_gl.at<double>(1,0)*wheel_dt;
			odom_delta.at<double>(2,0) += MeasuredVel_gl.at<double>(2,0)*wheel_dt;
			odom_delta.at<double>(3,0) += wheeldeltayaw_q;

			// grow Qdk only when ugv is moving
				ckf.Qw = Qw_scale * (cv::Mat_<double>(4, 4) <<
					std::abs(MeasuredVel_gl.at<double>(0,0)*wheel_dt),0,																								0,											0,
					0,																								std::abs(MeasuredVel_gl.at<double>(1,0)*wheel_dt),0,											0,
					0,																								0,																								0,											0,
					0,																								0,																								0,std::abs(wheeldeltayaw_q));

				ckf.Qdk = ckf.Fk * ckf.Qdk * ckf.Fk.t() + ckf.Qw*wheel_dt;

			posePublisher(wheelTime);

			fprintf (data.filename, "\n%% ----  WheelOdometry ---- \n" );
			fprintf (data.filename, "  %s.wheel.time(%d,1) = %-10.8f;\n", s_data_filename.c_str(), ++wheelcount, wheelTime - init_time);
			fprintf (data.filename, "  %s.wheel.dt(%d,1)   = %-10.8f;\n", s_data_filename.c_str(),   wheelcount, wheel_dt);
			fprintf (data.filename, "  %s.wheel.yaw_q(%d,1)   =  %-10.8f;\n", s_data_filename.c_str(), wheelcount, wheelyaw_q); //radians?
			fprintf (data.filename, "  %s.wheel.P_odom(%d,:)  = [%-10.8f %-10.8f %-10.8f];\n", s_data_filename.c_str(), wheelcount,osub -> pose.pose.position.x,osub -> pose.pose.position.y,osub -> pose.pose.position.z);
			fprintf (data.filename, "  %s.wheel.odom_delta(%d,:) = [%-10.8f %-10.8f %-10.8f %-10.8f];\n", s_data_filename.c_str(), wheelcount,
																	odom_delta.at<double>(0,0), odom_delta.at<double>(1,0), odom_delta.at<double>(2,0), odom_delta.at<double>(3,0));
	}
	// if ((dkf.counter > 0) && (s_ugvn.compare("ugv2") == 0)) {fprintf(stdout, (RED_TEXT  "%s :: void hastUGV::WheelOdometry() ... done\n" COLOR_RESET), s_data_filename.c_str());}
}

void hastUGV::writePrealloc(double waitbar_max)
{
	ros::Duration(0.1).sleep();
	preallocFile = std::fopen (s_prealloc.c_str(), "w");
	ros::Duration(0.2).sleep();
	fprintf (preallocFile, "\n%% --------------- writePrealloc() --------------- \n" );
	fprintf (preallocFile, "  wb = waitbar(0,' Loading %s ...');\n", s_data_filename.c_str());
	fprintf (preallocFile, "  waitbar_max = %f;\n", waitbar_max);
	ros::Duration(0.1).sleep();
}
