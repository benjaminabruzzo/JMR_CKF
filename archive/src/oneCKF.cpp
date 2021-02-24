#include "genheaders.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";



class ckfRecorder
{
	private:

		/*---------  File Recorder ------------- */
			std::string s_oneckfMfile, s_uav_baseframe;
			std::string s_run, s_dotm, s_root, s_date, s_user;
			std::FILE * oneckfMfile;
			double Pi;
			int L2Norm;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
				ros::Subscriber stereoPose_sub, HastShutDown_sub;

			/*----- Client Services */
			ros::ServiceServer StateService_ser;

		/*----- image transport Channels */
			image_transport::ImageTransport it;

		/* COMBINED CKF *************																	******************  COMBINED CKF */
			hastUAV uav;
			hastUGV ugv;
			ckfClass oneCKF;

		/*----- Stereo Variables */
			double stereoObs_stamp, stereoObs_yaw_ugv2uav;
			cv::Mat stereoObs_P, stereoObs_P_global, stereoObs_PCov;
			double stereoObs_yaw_cov;
			uint stereoObs_counter;
			cv::Mat Bluepix, Greenpix, Redpix; // pixel vectors [rX rY lX lY]

			bool ckf_sim;

	public:

	ckfRecorder()
	: it(n)
	{
		/*--------- Math Constants ------------- */
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function

		/*---------  File Recorder Initilizer ------------- */
		if (n.getParam("/hast/user", s_user)) {} else {s_user = "turtlebot";}
		if (n.getParam("/hast/run", s_run)) {} else {s_run = "000";}
		if (n.getParam("/hast/date", s_date)) {} else {s_date = "";}
		if (n.getParam("/hast/kf/sim", ckf_sim)) {} else {ckf_sim = false;}
		uav.ckf_sim = ckf_sim; ugv.ckf_sim = ckf_sim;

		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		ros::Duration(0.5).sleep();

		uav.s_filename = s_root + s_date + "/" + s_run + "/uavRecorder_" + s_run + s_dotm;
		uav.openmFile();

		ugv.s_filename = s_root + s_date + "/" + s_run + "/ugvRecorder_" + s_run + s_dotm;
		ugv.openmFile();

		s_oneckfMfile = s_root + s_date + "/" + s_run + "/ckfRecorder_"  + s_run + s_dotm;
		ROS_INFO("ckfRecorder: %s", s_oneckfMfile.c_str());
		oneckfMfile = std::fopen (s_oneckfMfile.c_str(), "w");
		fprintf (oneckfMfile, "%% %s\n", s_oneckfMfile.c_str());
		fprintf (oneckfMfile, "%%clc; \n%%clear all;\n%%close all;\n\n");

		ROS_INFO("ckfRecorder: .m files opened");

		/*--------- Initialize ROS Communication & Variables ------------- */
		ros::param::get("~ckf_TF_parent",uav.s_TF_gl_parent);
		ros::param::get("~ckf_TF_child",uav.s_TF_gl_child);
		ros::param::get("~odom_topic",ugv.s_wheel_odom_topic);
		ros::param::get("~footprint_topic",ugv.s_base_footprint_topic);
			ROS_INFO("ckfRecorder::ugvClass: %s", ugv.s_wheel_odom_topic.c_str());
		
		ros::param::get("~uav_imu_yawdriftrate",uav.yaw_drift_rate);
			ROS_INFO("ckfRecorder::yaw_drift_rate: %4.4f", uav.yaw_drift_rate);
		
		/*-----  Publishers and Subscribers */
		HastShutDown_sub 	= n.subscribe("/hast/shutdown",	1, &ckfRecorder::nodeShutDown, this);
		stereoPose_sub 		= n.subscribe("/hast/stereo/pose", 1, &ckfRecorder::stereoRead , this);
		uav.navData_sub = n.subscribe("/ardrone/navdata",  5, &hastUAV::inertialUpdate, &uav);
		uav.cmdVel_sub  = n.subscribe("/ardrone/cmd_vel",  5, &hastUAV::readCmdVel, &uav);
		uav.state_pub   = n.advertise<hast::uavstate>("/hast/uav/state", 1);

		ugv.WheelOdom_sub 	= n.subscribe(ugv.s_wheel_odom_topic,	5, &hastUGV::WheelOdometry, &ugv);		
		ugv.state_pub 		= n.advertise<hast::ugvstate>("/hast/ugv/state", 1);
		ugv.posePublisher(ros::Time::now().toSec());

		/*-----  Services and Clients */

		StateService_ser  	= n.advertiseService("/hast/service/uav/state", &ckfRecorder::serveState , this);

		ROS_INFO("ckfRecorder::publishers and subscribers constructed");

		/*----- dual-purpose stereo message variables */
		stereoObs_stamp = 0;
		stereoObs_counter = 0;
		stereoObs_yaw_ugv2uav = 0;
		stereoObs_P = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);

		// oneCKF init
			oneCKF.counter = 0;
			oneCKF.Mech = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.Aiding = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.PosteriorEst = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.Fk = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.PosteriorCov = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
			oneCKF.Qw = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.H = (cv::Mat_<double>(4, 5) <<
											1,0,0,0,0,
											0,1,0,0,0,
											0,0,1,0,0, 
											0,0,0,1,1); // !!! 4x5 measurement matrix
			oneCKF.Rk = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);// !!! 4x4 stereo covariance matrix
			oneCKF.I = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
	}

	void update_oneCKF(double stereotime)
	{//ROS_INFO("void update_oneCKF(double stereotime) open");
		double dt = stereotime - oneCKF.lastCKFtime;
		bool Qdk_flag;
		if (dt > 10)
			{dt = 0.05;} // Ignore first dt or any long duration gaps in ckf updates
			else {oneCKF.lastCKFtime = stereotime;} // reset 'last' time
			

		// convert uav and ugv poses to ugv-centric
		// estimate position of UAV in estimated UGV frame?
		uav.EstPosition_ugv = ugv.Hgl2lo * (cv::Mat_<double>(4, 1) << 
										uav.EstPosition_gl.at<double>(0,0), 
										uav.EstPosition_gl.at<double>(1,0), 
										uav.EstPosition_gl.at<double>(2,0),
										1); //homogeneous coordinates

		uav.EstYaw_ugv = uav.EstYaw_gl - ugv.EstYaw_gl;

		oneCKF.Mech = (cv::Mat_<double>(5, 1) <<
										uav.EstPosition_ugv.at<double>(0, 0),
										uav.EstPosition_ugv.at<double>(1, 0),
										uav.EstPosition_ugv.at<double>(2, 0),
										uav.EstYaw_ugv,
										0); // bias is not mechanized? No, it is. This would work if you used compass yaw above here and the bias estimate here
										// H has the sum of the bias and the yaw value coupled, which is how it gets estimated

		oneCKF.Aiding = (cv::Mat_<double>(5, 1) <<
										stereoObs_P.at<double>(0, 0),
										stereoObs_P.at<double>(1, 0),
										stereoObs_P.at<double>(2, 0),
										stereoObs_yaw_ugv2uav,
										0); // double bias_from_stereo = 0; //stereo cannot measure bias

		oneCKF.Rk = (cv::Mat_<double>(4, 4) <<
										stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2), 0,
										stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2), 0,
										stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2), 0,
										0, 0, 0, stereoObs_yaw_cov);// !!! 4x4 stereo covariance matrix

		// uavckf.Qdk should be rotated into ugv frame
		// oneCKF.Qdk = ugv.Rgl2lo4x4*uav.ckf.Qdk*ugv.Rgl2lo4x4.t() + ugv.ckf.Qdk;	


		oneCKF.Qdk = uav.ckf.Qdk + ugv.ckf.Qdk;	
		// how are these getting added?  They're different sizes? 
		// Actually, these are the same size. Who cares if the fifth state of the ugv Q matrix is always zero?

		// % Increment KF
		oneCKF.incrementKF();

		// This prevents divide by zero error ... 
		Qdk_flag = true;
		if ((oneCKF.Qdk.at<double>(0,0) < 0.00001) ||
			(oneCKF.Qdk.at<double>(1,1) < 0.00001) ||
			(oneCKF.Qdk.at<double>(2,2) < 0.00001) ||
			(oneCKF.Qdk.at<double>(3,3) < 0.00001))
			{ // if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
				Qdk_flag = false;
			}
		// The estimated values are the previous esitmates plus the error adjustement
		// take the Qd fraction of the update and apply that to each vehicle?
		// ugv
		if(Qdk_flag)
		{
			ugv.yawCorrection = oneCKF.PosteriorEst.at<double>(3,0) * (ugv.ckf.Qdk.at<double>(3,3)/oneCKF.Qdk.at<double>(3,3));
		} else {
			ugv.yawCorrection = 0;
		}

		ugv.EstYaw_gl += ugv.yawCorrection;

		ugv.cosyaw = cos(Pi * ugv.EstYaw_gl / 180);
		ugv.sinyaw = sin(Pi * ugv.EstYaw_gl / 180);
		ugv.Rgl2lo = (cv::Mat_<double>(3, 3) <<
										 ugv.cosyaw, ugv.sinyaw, 0,
										-ugv.sinyaw, ugv.cosyaw, 0,
															0,0,1);
		ugv.Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
									   ugv.cosyaw, ugv.sinyaw, 0, 0,
									  -ugv.sinyaw, ugv.cosyaw, 0, 0,
									   0, 0, 1, 0,
									   0, 0, 0, 1);
		if(Qdk_flag)
		{
			ugv.correction_gl = ugv.Rgl2lo.t() * (cv::Mat_<double>(3, 1) <<
				oneCKF.PosteriorEst.at<double>(0,0)* (ugv.ckf.Qdk.at<double>(0,0)/oneCKF.Qdk.at<double>(0,0)),
				oneCKF.PosteriorEst.at<double>(1,0)* (ugv.ckf.Qdk.at<double>(1,1)/oneCKF.Qdk.at<double>(1,1)),
				oneCKF.PosteriorEst.at<double>(2,0)* (ugv.ckf.Qdk.at<double>(2,2)/oneCKF.Qdk.at<double>(2,2)));
		} else {
			ugv.correction_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		}
		
		ugv.EstPosition_gl += ugv.correction_gl;
		ugv.Hgl2lo = wrapH(ugv.Rgl2lo, ugv.EstPosition_gl);
		ugv.Hlo2gl = invertH(ugv.Hgl2lo);

		// uav
		if(Qdk_flag)
		{
			uav.correction_ugv = (cv::Mat_<double>(4, 1) <<
				oneCKF.PosteriorEst.at<double>(0,0)* (uav.ckf.Qdk.at<double>(0,0)/oneCKF.Qdk.at<double>(0,0)),
				oneCKF.PosteriorEst.at<double>(1,0)* (uav.ckf.Qdk.at<double>(1,1)/oneCKF.Qdk.at<double>(1,1)),
				oneCKF.PosteriorEst.at<double>(2,0)* (uav.ckf.Qdk.at<double>(2,2)/oneCKF.Qdk.at<double>(2,2)),
				0);
		} else {
			uav.correction_ugv = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
		}
		uav.EstPosition_ugv += uav.correction_ugv;
		uav.EstPositionHom_g = ugv.Hlo2gl * uav.EstPosition_ugv;
		uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << 
										uav.EstPositionHom_g.at<double>(0,0), 
										uav.EstPositionHom_g.at<double>(1,0), 
										uav.EstPositionHom_g.at<double>(2,0));
		if(Qdk_flag)
		{
			uav.yawCorrection = oneCKF.PosteriorEst.at<double>(3,0) * (uav.ckf.Qdk.at<double>(3,3)/oneCKF.Qdk.at<double>(3,3));
		} else {
			uav.yawCorrection = 0;
		}
		uav.EstYaw_gl += ugv.yawCorrection;
		
		// all of the bias goes to the uav * (uav.ckf.Qdk.at<double>(4,4)/oneCKF.Qdk.at<double>(4,4));
		uav.EstYawBias += oneCKF.PosteriorEst.at<double>(4,0);

		double cosyaw = cos(Pi * uav.EstYaw_gl / 180);
		double sinyaw = sin(Pi * uav.EstYaw_gl / 180);
		uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
								   cosyaw, sinyaw, 0,
								   -sinyaw, cosyaw, 0,
								   0, 0, 1);
		uav.Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
								   cosyaw, sinyaw, 0, 0,
								  -sinyaw, cosyaw, 0, 0,
								   0, 0, 1, 0,
								   0, 0, 0, 1);

		uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
		uav.Hlo2gl = invertH(uav.Hgl2lo);

		uav.observedByStereo = true;
		++oneCKF.counter;
		// write out data
			fprintf (oneckfMfile, "ckfRecorder.time(%u,:) = % -6.14f;\n", oneCKF.counter, stereotime);
			fprintf (oneckfMfile, "ckfRecorder.zk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.zk.at<double>(0,0), oneCKF.zk.at<double>(1,0), oneCKF.zk.at<double>(2,0), oneCKF.zk.at<double>(3,0), oneCKF.zk.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.yk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.yk.at<double>(0,0), oneCKF.yk.at<double>(1,0), oneCKF.yk.at<double>(2,0), oneCKF.yk.at<double>(3,0));
			fprintf (oneckfMfile, "ckfRecorder.Mech(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Mech.at<double>(0,0), oneCKF.Mech.at<double>(1,0), oneCKF.Mech.at<double>(2,0), oneCKF.Mech.at<double>(3,0), oneCKF.Mech.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.Kkyk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Kkyk.at<double>(0,0), oneCKF.Kkyk.at<double>(1,0), oneCKF.Kkyk.at<double>(2,0), oneCKF.Kkyk.at<double>(3,0), oneCKF.Kkyk.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.Aiding(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Aiding.at<double>(0,0), oneCKF.Aiding.at<double>(1,0), oneCKF.Aiding.at<double>(2,0), oneCKF.Aiding.at<double>(3,0),oneCKF.Aiding.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.PriorEst(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.PriorEst.at<double>(0,0), oneCKF.PriorEst.at<double>(1,0), oneCKF.PriorEst.at<double>(2,0), oneCKF.PriorEst.at<double>(3,0), oneCKF.PriorEst.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorEst(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorEst.at<double>(0,0), oneCKF.PosteriorEst.at<double>(1,0), oneCKF.PosteriorEst.at<double>(2,0), oneCKF.PosteriorEst.at<double>(3,0), oneCKF.PosteriorEst.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.ugv.correction_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, ugv.correction_gl.at<double>(0,0), ugv.correction_gl.at<double>(1,0), ugv.correction_gl.at<double>(2,0));
			fprintf (oneckfMfile, "ckfRecorder.ugv.yawCorrection(%u,:) = % -6.14f ;\n", oneCKF.counter, ugv.yawCorrection);
			fprintf (oneckfMfile, "ckfRecorder.uav.correction_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, uav.correction_ugv.at<double>(0,0), uav.correction_ugv.at<double>(1,0), uav.correction_ugv.at<double>(2,0), uav.correction_ugv.at<double>(3,0));
			fprintf (oneckfMfile, "ckfRecorder.uav.yawCorrection(%u,:) = % -6.14f ;\n", oneCKF.counter, uav.yawCorrection);

			// fprintf (oneckfMfile, "ckfRecorder.PriorCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(0, 0), oneCKF.PriorCov.at<double>(0, 1), oneCKF.PriorCov.at<double>(0, 2), oneCKF.PriorCov.at<double>(0, 3), oneCKF.PriorCov.at<double>(0, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PriorCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(1, 0), oneCKF.PriorCov.at<double>(1, 1), oneCKF.PriorCov.at<double>(1, 2), oneCKF.PriorCov.at<double>(1, 3), oneCKF.PriorCov.at<double>(1, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PriorCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(2, 0), oneCKF.PriorCov.at<double>(2, 1), oneCKF.PriorCov.at<double>(2, 2), oneCKF.PriorCov.at<double>(2, 3), oneCKF.PriorCov.at<double>(2, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PriorCov(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(3, 0), oneCKF.PriorCov.at<double>(3, 1), oneCKF.PriorCov.at<double>(3, 2), oneCKF.PriorCov.at<double>(3, 3), oneCKF.PriorCov.at<double>(3, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PriorCov(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(4, 0), oneCKF.PriorCov.at<double>(4, 1), oneCKF.PriorCov.at<double>(4, 2), oneCKF.PriorCov.at<double>(4, 3), oneCKF.PriorCov.at<double>(4, 4));

			// fprintf (oneckfMfile, "ckfRecorder.Rk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(0, 0), oneCKF.Rk.at<double>(0, 1), oneCKF.Rk.at<double>(0, 2), oneCKF.Rk.at<double>(0, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Rk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(1, 0), oneCKF.Rk.at<double>(1, 1), oneCKF.Rk.at<double>(1, 2), oneCKF.Rk.at<double>(1, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Rk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(2, 0), oneCKF.Rk.at<double>(2, 1), oneCKF.Rk.at<double>(2, 2), oneCKF.Rk.at<double>(2, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Rk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(3, 0), oneCKF.Rk.at<double>(3, 1), oneCKF.Rk.at<double>(3, 2), oneCKF.Rk.at<double>(3, 3));

			// fprintf (oneckfMfile, "ckfRecorder.Sk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(0, 0), oneCKF.Sk.at<double>(0, 1), oneCKF.Sk.at<double>(0, 2), oneCKF.Sk.at<double>(0, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Sk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(1, 0), oneCKF.Sk.at<double>(1, 1), oneCKF.Sk.at<double>(1, 2), oneCKF.Sk.at<double>(1, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Sk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(2, 0), oneCKF.Sk.at<double>(2, 1), oneCKF.Sk.at<double>(2, 2), oneCKF.Sk.at<double>(2, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Sk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(3, 0), oneCKF.Sk.at<double>(3, 1), oneCKF.Sk.at<double>(3, 2), oneCKF.Sk.at<double>(3, 3));

			// fprintf (oneckfMfile, "ckfRecorder.Skinv(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(0, 0), oneCKF.Skinv.at<double>(0, 1), oneCKF.Skinv.at<double>(0, 2), oneCKF.Skinv.at<double>(0, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Skinv(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(1, 0), oneCKF.Skinv.at<double>(1, 1), oneCKF.Skinv.at<double>(1, 2), oneCKF.Skinv.at<double>(1, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Skinv(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(2, 0), oneCKF.Skinv.at<double>(2, 1), oneCKF.Skinv.at<double>(2, 2), oneCKF.Skinv.at<double>(2, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Skinv(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(3, 0), oneCKF.Skinv.at<double>(3, 1), oneCKF.Skinv.at<double>(3, 2), oneCKF.Skinv.at<double>(3, 3));

			// fprintf (oneckfMfile, "ckfRecorder.Kk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(0, 0), oneCKF.Kk.at<double>(0, 1), oneCKF.Kk.at<double>(0, 2), oneCKF.Kk.at<double>(0, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Kk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(1, 0), oneCKF.Kk.at<double>(1, 1), oneCKF.Kk.at<double>(1, 2), oneCKF.Kk.at<double>(1, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Kk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(2, 0), oneCKF.Kk.at<double>(2, 1), oneCKF.Kk.at<double>(2, 2), oneCKF.Kk.at<double>(2, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Kk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(3, 0), oneCKF.Kk.at<double>(3, 1), oneCKF.Kk.at<double>(3, 2), oneCKF.Kk.at<double>(3, 3));
			// fprintf (oneckfMfile, "ckfRecorder.Kk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(4, 0), oneCKF.Kk.at<double>(4, 1), oneCKF.Kk.at<double>(4, 2), oneCKF.Kk.at<double>(4, 3));

			// fprintf (oneckfMfile, "ckfRecorder.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(0, 0), oneCKF.Qdk.at<double>(0, 1), oneCKF.Qdk.at<double>(0, 2), oneCKF.Qdk.at<double>(0, 3), oneCKF.Qdk.at<double>(0, 4));
			// fprintf (oneckfMfile, "ckfRecorder.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(1, 0), oneCKF.Qdk.at<double>(1, 1), oneCKF.Qdk.at<double>(1, 2), oneCKF.Qdk.at<double>(1, 3), oneCKF.Qdk.at<double>(1, 4));
			// fprintf (oneckfMfile, "ckfRecorder.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(2, 0), oneCKF.Qdk.at<double>(2, 1), oneCKF.Qdk.at<double>(2, 2), oneCKF.Qdk.at<double>(2, 3), oneCKF.Qdk.at<double>(2, 4));
			// fprintf (oneckfMfile, "ckfRecorder.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(3, 0), oneCKF.Qdk.at<double>(3, 1), oneCKF.Qdk.at<double>(3, 2), oneCKF.Qdk.at<double>(3, 3), oneCKF.Qdk.at<double>(3, 4));
			// fprintf (oneckfMfile, "ckfRecorder.Qdk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(4, 0), oneCKF.Qdk.at<double>(4, 1), oneCKF.Qdk.at<double>(4, 2), oneCKF.Qdk.at<double>(4, 3), oneCKF.Qdk.at<double>(4, 4));

			// fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(0, 0), oneCKF.PosteriorCov.at<double>(0, 1), oneCKF.PosteriorCov.at<double>(0, 2), oneCKF.PosteriorCov.at<double>(0, 3), oneCKF.PosteriorCov.at<double>(0, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(1, 0), oneCKF.PosteriorCov.at<double>(1, 1), oneCKF.PosteriorCov.at<double>(1, 2), oneCKF.PosteriorCov.at<double>(1, 3), oneCKF.PosteriorCov.at<double>(1, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(2, 0), oneCKF.PosteriorCov.at<double>(2, 1), oneCKF.PosteriorCov.at<double>(2, 2), oneCKF.PosteriorCov.at<double>(2, 3), oneCKF.PosteriorCov.at<double>(2, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(3, 0), oneCKF.PosteriorCov.at<double>(3, 1), oneCKF.PosteriorCov.at<double>(3, 2), oneCKF.PosteriorCov.at<double>(3, 3), oneCKF.PosteriorCov.at<double>(3, 4));
			// fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(4, 0), oneCKF.PosteriorCov.at<double>(4, 1), oneCKF.PosteriorCov.at<double>(4, 2), oneCKF.PosteriorCov.at<double>(4, 3), oneCKF.PosteriorCov.at<double>(4, 4));

			fprintf (oneckfMfile, "ckfRecorder.uav.red.left.xy(%u,:)    = [% -6.14f % -6.14f];\n", oneCKF.counter, Redpix.at<double>(0,2), Redpix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.red.right.xy(%u,:)   = [% -6.14f % -6.14f];\n", oneCKF.counter, Redpix.at<double>(0,0), Redpix.at<double>(0,1));
			fprintf (oneckfMfile, "ckfRecorder.uav.blue.left.xy(%u,:)   = [% -6.14f % -6.14f];\n", oneCKF.counter, Bluepix.at<double>(0,2), Bluepix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.blue.right.xy(%u,:)  = [% -6.14f % -6.14f];\n", oneCKF.counter, Bluepix.at<double>(0,0), Bluepix.at<double>(0,1));
			fprintf (oneckfMfile, "ckfRecorder.uav.green.left.xy(%u,:)  = [% -6.14f % -6.14f];\n", oneCKF.counter, Greenpix.at<double>(0,2), Greenpix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.green.right.xy(%u,:) = [% -6.14f % -6.14f];\n", oneCKF.counter, Greenpix.at<double>(0,0), Greenpix.at<double>(0,1));

			// fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(0, 0), uav.ckf.Qdk.at<double>(0, 1), uav.ckf.Qdk.at<double>(0, 2), uav.ckf.Qdk.at<double>(0, 3), uav.ckf.Qdk.at<double>(0, 4));
			// fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(1, 0), uav.ckf.Qdk.at<double>(1, 1), uav.ckf.Qdk.at<double>(1, 2), uav.ckf.Qdk.at<double>(1, 3), uav.ckf.Qdk.at<double>(1, 4));
			// fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(2, 0), uav.ckf.Qdk.at<double>(2, 1), uav.ckf.Qdk.at<double>(2, 2), uav.ckf.Qdk.at<double>(2, 3), uav.ckf.Qdk.at<double>(2, 4));
			// fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(3, 0), uav.ckf.Qdk.at<double>(3, 1), uav.ckf.Qdk.at<double>(3, 2), uav.ckf.Qdk.at<double>(3, 3), uav.ckf.Qdk.at<double>(3, 4));
			// fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(4, 0), uav.ckf.Qdk.at<double>(4, 1), uav.ckf.Qdk.at<double>(4, 2), uav.ckf.Qdk.at<double>(4, 3), uav.ckf.Qdk.at<double>(4, 4));

			// fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(0, 0), ugv.ckf.Qdk.at<double>(0, 1), ugv.ckf.Qdk.at<double>(0, 2), ugv.ckf.Qdk.at<double>(0, 3));
			// fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(1, 0), ugv.ckf.Qdk.at<double>(1, 1), ugv.ckf.Qdk.at<double>(1, 2), ugv.ckf.Qdk.at<double>(1, 3));
			// fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(2, 0), ugv.ckf.Qdk.at<double>(2, 1), ugv.ckf.Qdk.at<double>(2, 2), ugv.ckf.Qdk.at<double>(2, 3));
			// fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(3, 0), ugv.ckf.Qdk.at<double>(3, 1), ugv.ckf.Qdk.at<double>(3, 2), ugv.ckf.Qdk.at<double>(3, 3));

			fprintf (oneckfMfile, "ckfRecorder.uav.Yaw(%d,1)  = % -6.14f;\n", oneCKF.counter, uav.EstYaw_gl);
			fprintf (oneckfMfile, "ckfRecorder.uav.YawBias(%d,1)  = % -6.14f;\n", oneCKF.counter, uav.EstYawBias);
			fprintf (oneckfMfile, "ckfRecorder.uav.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n\n", oneCKF.counter, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));

		//reset Qd
		oneCKF.Qdk  = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		uav.ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		ugv.ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		// ROS_INFO("void update_oneCKF(double stereotime) close");
	}

	void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)
	{//ROS_INFO("void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)");
		// New stereo observation
		// stereoObs_odomBit = stereo_state_msg -> odomBIT;
		// uav.state_msg.odomBIT = stereo_state_msg -> odomBIT;
		// stereoObs_odomMODE = stereo_state_msg -> odomMODE;
		stereoObs_stamp = stereo_state_msg -> stamp;
		stereoObs_P = (cv::Mat_<double>(4, 1) << stereo_state_msg->P.x, stereo_state_msg->P.y, stereo_state_msg->P.z, 1.0);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) <<
						 stereo_state_msg->PCov.row0.x, stereo_state_msg->PCov.row0.y, stereo_state_msg->PCov.row0.z,
						 stereo_state_msg->PCov.row1.x, stereo_state_msg->PCov.row1.y, stereo_state_msg->PCov.row1.z,
						 stereo_state_msg->PCov.row2.x, stereo_state_msg->PCov.row2.y, stereo_state_msg->PCov.row2.z);

		stereoObs_yaw_ugv2uav = stereo_state_msg -> yaw; // uav heading angle in degrees in ugv frame
		stereoObs_yaw_cov = stereo_state_msg -> yaw_cov; // variance of heading angle (in radians)

		stereoObs_P_global = ugv.Hlo2gl * stereoObs_P;

		Redpix = (cv::Mat_<double>(4, 1) << stereo_state_msg->red.xr, stereo_state_msg->red.yr, stereo_state_msg->red.xl, stereo_state_msg->red.yl);
		Bluepix = (cv::Mat_<double>(4, 1) << stereo_state_msg->blue.xr, stereo_state_msg->blue.yr, stereo_state_msg->blue.xl, stereo_state_msg->blue.yl);
		Greenpix = (cv::Mat_<double>(4, 1) << stereo_state_msg->green.xr, stereo_state_msg->green.yr, stereo_state_msg->green.xl, stereo_state_msg->green.yl);

		fprintf (ugv.mFile, "ugvRecorder.stereo.time(%d,1)  = % -6.14f;\n", ++stereoObs_counter, stereoObs_stamp);
		fprintf (ugv.mFile, "ugvRecorder.stereo.yaw_ugv(%d,1)  = % -6.14f;\n", stereoObs_counter, stereoObs_yaw_ugv2uav);
		fprintf (ugv.mFile, "ugvRecorder.stereo.yaw_cov(%d,1)  = % -6.14f;\n", stereoObs_counter, stereoObs_yaw_cov);
		fprintf (ugv.mFile, "ugvRecorder.stereo.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", stereoObs_counter, stereoObs_P_global.at<double>(0,0), stereoObs_P_global.at<double>(1,0), stereoObs_P_global.at<double>(2,0));
		fprintf (ugv.mFile, "ugvRecorder.stereo.Position_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", stereoObs_counter, stereoObs_P.at<double>(0,0), stereoObs_P.at<double>(1,0), stereoObs_P.at<double>(2,0));

		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n", stereoObs_counter, stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2));
		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n", stereoObs_counter, stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2));
		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n\n", stereoObs_counter, stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2));

		update_oneCKF(stereoObs_stamp);
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if (ShutDown->flag)
		{
			ROS_INFO("ckfRecorder: Shutdown requested..");
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
	}

	bool serveState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res)
	{res.state = uav.flightState; return true;}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ckfRecorder");
	ckfRecorder cfkRec;
	ros::spin();
	return 0;
}

