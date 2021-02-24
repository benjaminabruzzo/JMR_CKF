#ifndef HAST_JOINTSLAM_C_H
#define HAST_JOINTSLAM_C_H

#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
#include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";

class jointSLAM_c
{
	private:
		/*---------  Assorted Constants ------------- */
			int L2Norm;
		/*---------  File Recorder ------------- */
			std::string s_trial, s_date, s_user;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
			tf::TransformListener listener;

			// April Tag detections
			ros::Subscriber TagDetections_sub;
				std::string s_tagDetections;

			// April tag obstacle publications
			ros::Publisher aprilObstacleCloud_pub; 
				std::string s_aprilObstacleCloud_topic;
				sensor_msgs::PointCloud2 aprilObstacleCloud_msg;
				pcl::PointXYZ aprilCenter_XYZ; // pcl point for center of april tag

			// ugvGoal location using tags
			ros::Subscriber ugvGoalID_sub;
				uint ugvGoalID;
				std::string s_ugvGoalID_sub_topic;

			//Slam On/Off switch service
			ros::ServiceServer slam_onoff_switch_ser;
				std::string s_slam_onoff_switch_ser;
				bool slamSwitch;

			// Shutdown listener
			ros::Subscriber shutdown_sub;
				std::string s_shutdown_topic;

			// Uav inertial measurement
			ros::Subscriber navData_sub;
				std::string s_navData_topic;

			struct slamData
			{
				cv::Mat I4, nI4; // identity and negative negative matrix

				cv::Mat HPHt; // HPHt = H*P*H'
				cv::Mat Sk; // Innovation covariance, Sk = HPHt + Rk_slam
				cv::Mat PHt; // PHtr = P_slam*H' 
				cv::Mat Kk; // Kalman Gain, // K = PHtr * Sk
				cv::Mat vk; // difference between measured and estimated landmark poses
				cv::Mat correction; // correction to apply to augmentedState

				// Trying to do dynamic state and matrix growth
				std::vector<int> landmarksOrdered;  // Landmark ID in order of observation
				std::vector<double> timeOflandmarksOrdered;  // Landmark ID in order of observation
				uint numberOfLandmarksOrdered;
				std::vector<double> augmentedState; // robot plus landmarks : estimated positions
				uint augmentedStateSize;
				std::vector<double> augmentedMeasurementVector; // current measurements of landmarks
				std::vector<double> predictedMeasurementVector; // predicted measurements of landmarks
				cv::Mat augmentedCovariance; // square matrix for all vehicles and landmarks

				cv::Mat uavR, uavt, uavHgl2lo;
				double uavCos, uavSin;

				std::vector<int> firstTimeForID;
				std::vector<int> updateForID;
				std::vector<uint> TagsInFOV;
				uint NumberOfTagsinFOV, registered, slamCount;    
				int downimagecountup; // number of images to ignore before starting slam
			};

			struct vehicleStates
			{
				cv::Mat odom_gl;
				double EstPhi_gl, cosEstPhi_gl, sinEstPhi_gl;
				cv::Mat MeasuredVel_lo;
				cv::Mat Rgl2lo, Hgl2lo, Hlo2gl;

				double deltaPhi, odomPhi;
				double compassYaw, compassYawLast;
				double cos_compassYaw, cos_compassYawLast;
				double sin_compassYaw, sin_compassYawLast;
				
				cv::Mat Qdk, Qw, Fk;

				cv::Mat RdeltaPhi_gl2lo;
				double cosRdeltaPhi, sinRdeltaPhi;

				// tf broadcaster variables
				tf::Transform TF;
				tf::Matrix3x3 R_TF;
				tf::Quaternion Q_TF;

				// state listener variables
				cv::Mat posemsg_P_gl;
				double posemsg_Phi_gl;

				ros::Subscriber est_state_sub; // ugv publisher
				ros::Publisher est_state_pub; // ugv publisher
				std::string s_pose_topic, s_mux_topic;
				hast::posewithcov est_pose_msg;
			};
	public:
		double Pi;
		/*---------  Landmark Definitions ------------- */
			std::vector<apriltags_ros::AprilTagDetection> tagArray; //marker array as detected by image
				uint tagArraySize; // number of tags in array
				ros::Time tagStamp;
				double tagTSpublished;

			std::vector<apriltagclass> april;  // class for storing individual tag information
				int tag_max_id, tag_counter;
				std::string s_markerid;
				int printlength;
				char markerChannel[20];
				cv::Vec<double,3> markerObs;

		/*---------  Navdata/uav variables ------------- */
			uint navDataCounter, flightState, navHeadSeq;
			ros::Time navStamp;
			double navTS, navTSpublished, navdt;
			double echoAlt, echoAltLast, deltaAlt;
			
			// tf variables
			tf::TransformBroadcaster uav_TF_pub;
			std::string s_ref_TF_frame, s_uav_TF_frame;
			// ugv Tf strings
			std::string s_ugv1_TF_frame, s_ugv2_TF_frame;
			// std::string s_ugv_TF_frame;

		/*--------- Class variables ------------- */
			fileops mfile;
			slamData slam;
			vehicleStates uav, ugv1, ugv2;
			double numberOfUGV, numberOfUAV;
			uint numUGV, numUAV;

			hastUAV uav_c; // create uav from class

			double augCovScale, uavQdkScale, uavQwScale, RkScale;

			ros::Publisher slam_stateFlag_pub;
				hast::flag slam_flag;
				std::string s_slam_stateFlag_topic;

	// Class Functions
		jointSLAM_c();
		bool slamOnOffSwitch(hast::slamswitch::Request &req, hast::slamswitch::Response &res);
		void ugvUpdate(const hast::ugvmux::ConstPtr& mux_msg);
		void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags);
		void MarkerSLAM(double scantime);
		void inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata);
		void augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj);
		void growCov(cv::Mat Rj);
		void tfPublisher(ros::Time pubstamp);
		void poseHeaderPublisher(ros::Time pubstamp);
		void nodeShutDown(const hast::flag::ConstPtr& ShutDown);
};


#endif