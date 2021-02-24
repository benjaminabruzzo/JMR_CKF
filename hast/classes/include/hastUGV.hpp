#ifndef HAST_UGV_H
#define HAST_UGV_H

#include "genheaders.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
#include "fileops.hpp"
#include "utils.hpp"

class hastUGV
{
	private:
		// Constants which might be useful in the future
		double Pi;
		int L2Norm;
	public:
		// File recording:
			std::FILE * mFile;
			std::FILE * preallocFile;
			std::string s_filename, s_prealloc, s_matlab_field, s_ugvn;

			fileops data;
			std::string s_data_filename;

		// ROS comms
		ros::Publisher state_pub; // ugv publisher
		ros::Subscriber WheelOdom_sub; //, cmdVel_sub;
		// std::string s_ugvCmd_topic;
		std::string s_state_pub_topic;
		hast::ugvstate state_msg;
		uint stateMsg_id; // pose msg counter

		// time and Counters
		double init_time;
		double slamtime, slamtimelast;
		double velStamp, velStamplast;
		uint velCounter, estCounter;

		cv::Mat odom_gl, odom_delta, odom_J;
		cv::Mat EstPosition_gl, EstPosition_ugv, correction_gl;
		cv::Mat MeasuredVel_lo, MeasuredVel_gl;
		double EstYaw_gl, cosyaw, sinyaw, yawCorrection;
		cv::Mat Rgl2lo, Rlo2gl, Rgl2lo4x4, Rlo2gl4x4;
		cv::Mat Hlo2gl, Hgl2lo;

		// create complementary filter for ugv
		ckfClass ckf, ekf;
		DiscreteKF dkf;
		double Qw_scale;

		/*-----  Wheel Odometry */
		std::string s_wheel_odom_topic, s_base_footprint_topic;
		uint wheelcount;

		ros::Time wheelTimeStamp;
		double wheelTime, lastWheelTime, wheel_dt;

		double wheel_yaw_rate, wheelyaw_q, wheeldeltayaw_q;
		cv::Mat wheel_twist_linear, wheel_twist_angular;

		double vel_dt_aggregator;
		bool ckf_sim;

				ros::Publisher dkf_pub;
				ros::Subscriber dkf_sub;
					hast::posewithcov dkf_msg;
					std::string s_dkf_topic;

	// Functions
		hastUGV();
		void openmFile();
		void writePrealloc(double waitbar_max);
		void posePublisher(double timestamp);
		void WheelOdometry(const nav_msgs::Odometry::ConstPtr& osub);
		void updatePoseFromCKF(cv::Mat ckf_correction_gl, double ckf_yaw_correction);

		// functions of intilizaing UGV
		void initDKF();

};

#endif
