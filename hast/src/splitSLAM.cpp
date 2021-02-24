#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
// #include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// s_markerid = "tag_" + patch::to_string(newLandmarkID);
namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";

pcl::PointCloud<pcl::PointXYZ>::Ptr aprilCloud_global_ptr (new pcl::PointCloud<pcl::PointXYZ>);

class splitSLAM
{
	private:
	public:
		/*---------  Assorted Constants ------------- */
			double Pi;

		/*---------  File Recorder ------------- */
			std::string s_trial, s_date, s_user, s_matlab_field, s_full_field, s_exp_code;

			struct slamData
			{
				cv::Mat I4, nI4, z4; // identity and negative negative matrix
				cv::Mat H; //measurement matrix
				cv::Mat Hx; // prediction of actual measurements
				cv::Mat x; // vector of states
				cv::Mat Pf; // prediction update to covariance matrix
				cv::Mat HPHt; // HPHt = H*P*H'
				cv::Mat Sk; // Innovation covariance, Sk = HPHt + Rk_slam
				cv::Mat Sk_inv; // Inverse of Innovation covariance, Sk = HPHt + Rk_slam
				cv::Mat PHt; // PHtr = P_slam*H'
				cv::Mat Kk; // Kalman Gain, // K = PHtr * Sk
				cv::Mat KH; // Kalman Gain * H,
				cv::Mat KHI; // Identity matrix with same dimensions as (Kalman Gain * H),
				cv::Mat ImKH; // I - Kk * H,
				cv::Mat vk; // difference between measured and estimated landmark poses
				cv::Mat correction; // correction in uav frame to apply to augmentedState (after being rotated into global frame)
				cv::Mat SLAM_P, POST_P;
				cv::Mat ugvQdk, Qdk, Rk; // odometry uncertainty growth

				// Trying to do dynamic state and matrix growth
				std::vector<int> landmarksOrdered;  // Landmark ID in order of observation
				std::vector<double> timeOflandmarksOrdered;  // Landmark ID in order of observation
				uint numberOfLandmarksOrdered;
				std::vector<double> augmentedState; // robots plus landmarks in global frame
				uint augmentedStateSize;
				cv::Mat zk; // measurement vector
				std::vector<double> augmentedMeasurementVector; // current measurements of landmarks/vehicles in uav frame
				std::vector<double> predictedMeasurementVector; // predicted measurements of landmarks/vehicles in uav frame
				cv::Mat augmentedCovariance; // square matrix for all vehicles and landmarks in uav frame

				cv::Mat uavR, uavt;
				double uavCos, uavSin;

				std::vector<int> firstTimeForID;
				std::vector<int> updateForID;
				std::vector<uint> TagsInFOV;
				uint NumberOfTagsinFOV, registered, slamCount;
				int downimagecountup; // number of images to ignore before starting slam

				// debugging update issues
				uint ugv1count, ugv2count, tagcount;
			}; slamData slam, fullSLAM;

			struct vehicleStates
			{
				cv::Mat odom_gl, odom_delta, odom_J;
				double EstPhi_gl, cosEstPhi_gl, sinEstPhi_gl;
				cv::Mat MeasuredVel_lo;
				cv::Mat Rgl2lo, Hgl2lo, Hlo2gl;
				cv::Mat Htag2gl; // convert the measured position of a tag to the global frame, without adding the vehicle offset

				cv::Mat EstPosition_gl;

				int goal_id;

				double deltaPhi, odomPhi;
				double compassYaw, compassYawLast;
				double cos_compassYaw, cos_compassYawLast;
				double sin_compassYaw, sin_compassYawLast;

				cv::Mat Qdk, Qw, Fk, Rk;
				double QwScale, RkScale;

				cv::Mat RdeltaPhi_gl2lo;
				double cosRdeltaPhi, sinRdeltaPhi;

				cv::Mat P; // state covariance
				double P_trace; // trace of covariance

				// tf broadcaster variables
				tf::Transform TF;
				tf::Matrix3x3 R_TF;
				tf::Quaternion Q_TF;

				// state listener variables
				cv::Mat posemsg_P_gl;
				double posemsg_Phi_gl;

				ros::Subscriber est_state_sub; // ugv subscriber
				ros::Publisher est_state_pub; // ugv publisher
				std::string s_pose_topic, s_mux_topic;
				hast::posewithcov est_pose_msg;
				uint header_seq;

				hast::posewithcov slam_pose_msg;

				// for ugvs onlu:
				uint obsCount;
				bool slamFlag;
			}; vehicleStates uav, ugv1, ugv2;

			/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
			tf::TransformListener listener;

			//Slam On/Off switch service
			ros::ServiceServer slam_onoff_switch_ser;
				std::string s_slam_onoff_switch_ser;
				bool slamSwitch;
				bool UAVonly; // use to do uav-only slam without UGV support

			// estimated states of all vehicles
			ros::Publisher slam_pose_pub; // ugv publisher
				hast::slam_poses slam_msg;
				std::string s_slam_vehicle_poses_topic;

				// state flag: whether or not slam is running
				ros::Publisher slam_stateFlag_pub;
					hast::flag slam_flag;
					std::string s_slam_stateFlag_topic;

			// Shutdown listener
			ros::Subscriber shutdown_sub;
				std::string s_shutdown_topic;

		/*---------  Landmark Definitions ------------- */
			// April Tag detections
			ros::Subscriber TagDetections_sub;
				std::string s_tagDetections;

			// April tag obstacle publications
			ros::Publisher aprilObstacleCloud_pub;
				std::string s_aprilObstacleCloud_topic;
				sensor_msgs::PointCloud2 aprilObstacleCloud_msg;
				pcl::PointXYZ aprilCenter_XYZ; // pcl point for center of april tag

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

			ros::ServiceServer GoalTagLocation_ser;
				std::string s_goal_tag_service_topic;

		/*---------  Navdata/uav variables ------------- */
			uint navDataCounter, flightState, navHeadSeq;
			ros::Time navStamp;
			double navTS, navTSpublished, navdt;
			double echoAlt, echoAltLast, deltaAlt;

			// tf variables
			tf::TransformBroadcaster uav_TF_pub;
			std::string s_ref_TF_frame, s_uav_TF_frame, s_april_cloud_frame;
			// ugv Tf strings
			std::string s_ugv1_TF_frame, s_ugv2_TF_frame;
			// std::string s_ugv_TF_frame;
			// Uav inertial measurement
			ros::Subscriber navData_sub;
				std::string s_navData_topic;
				// uav flight state service
				ros::ServiceServer uavFlightState_ser;
					std::string s_uav_flight_state_topic;

		/*--------- Class variables ------------- */
			fileops mfile, fullfile;


			double numberOfUGV, numberOfUAV;
			uint numUGV, numUAV;

			hastUAV uav_c; // create uav from class

			double uavCovInit, ugvCovInit, aprilCovInit, growCovU, uavQdkScale;
			double RkScale, RkXYZ, RkYaw;

			bool absolute_measurements;
			double init_time;


			struct UGVUGV_SENSOR_struct
			{
				bool use_sensor;
				char s_use_sensor[5];
				int string_dummy;
			};
			UGVUGV_SENSOR_struct ugv_sensor;

	// Class Functions
		splitSLAM()
		{
			Pi = atan(1) * 4; // 3.14159...
			// File params
				n.getParam("/hast/user",  s_user);
				n.getParam("/hast/date",  s_date);
				n.getParam("/hast/trial", s_trial);
				n.getParam("/hast/exp_code", s_exp_code);

				if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}

				ros::param::get("~matlab_field",s_matlab_field);
				ros::param::get("~s_full_field",s_full_field);
				fullfile.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/" + s_full_field + "_" + s_trial + ".m");

			/*---------  Config ROS ------------- */
			// Switch to turn slam on/off
				ros::param::get("~jointslam_switch_topic",s_slam_onoff_switch_ser); fullfile.writeString("  " + s_full_field + ".topics.jointslam_switch_topic = '" + s_slam_onoff_switch_ser + "';\n" );
					slam_onoff_switch_ser    = n.advertiseService(s_slam_onoff_switch_ser, &splitSLAM::slamOnOffSwitch, this);
					slamSwitch = false;
					ugv1.obsCount = ugv2.obsCount = 0;
					ugv1.slamFlag = ugv2.slamFlag = false;

			// Switch to turn slam on/off
				ros::param::get("~jointslam_state_flag",s_slam_stateFlag_topic); fullfile.writeString("  " + s_full_field + ".topics.jointslam_state_flag = '" + s_slam_stateFlag_topic + "';\n" );
					slam_stateFlag_pub 	= n.advertise<hast::flag>(s_slam_stateFlag_topic, 10);
					slam_flag.flag = false;
					slam_stateFlag_pub.publish(slam_flag);

			// publish slam vehicle poses
				ros::param::get("~slam_vehicle_poses_topic",  s_slam_vehicle_poses_topic);
					fullfile.writeString("  " + s_full_field + ".topics.slam_vehicle_poses_topic = '"  + uav.s_pose_topic  + "';\n" );
					slam_pose_pub = n.advertise<hast::slam_poses>(s_slam_vehicle_poses_topic, 1);
					slam_msg.header.seq = 0; slam_msg.header.frame_id = "global";

			// frames for publishing TF to Rviz
				ros::param::get("~april_cloud_frame",s_april_cloud_frame); 		fullfile.writeString("  " + s_full_field + ".topics.s_april_cloud_frame = '"  + s_april_cloud_frame + "';\n" );
				ros::param::get("~ref_TF_frame",s_ref_TF_frame); 		fullfile.writeString("  " + s_full_field + ".topics.s_ref_TF_frame = '"  + s_ref_TF_frame + "';\n" );
				ros::param::get("~uav_TF_frame",s_uav_TF_frame); 		fullfile.writeString("  " + s_full_field + ".topics.s_uav_TF_frame = '"  + s_uav_TF_frame + "';\n" );
				ros::param::get("~ugv1_TF_frame",s_ugv1_TF_frame);	fullfile.writeString("  " + s_full_field + ".topics.s_ugv1_TF_frame = '" + s_ugv1_TF_frame + "';\n" );
				ros::param::get("~ugv2_TF_frame",s_ugv2_TF_frame);	fullfile.writeString("  " + s_full_field + ".topics.s_ugv2_TF_frame = '" + s_ugv2_TF_frame + "';\n" );

			// publish uav estimate
				ros::param::get("~numberOfUAV",numberOfUAV); fullfile.writeString("  " + s_full_field + ".params.numberOfUAV = " + patch::to_string(numberOfUAV) + ";\n" );	numUAV = int(numberOfUAV);
				ros::param::get("~numberOfUGV",numberOfUGV); fullfile.writeString("  " + s_full_field + ".params.numberOfUGV = " + patch::to_string(numberOfUGV) + ";\n" ); numUGV = int(numberOfUGV);

				ros::param::get("~uav_slam_pose_topic", uav_c.s_uav_pose_topic); fullfile.writeString("  " + s_full_field + ".topics.uav_pose_topic = '" + uav_c.s_uav_pose_topic + "';\n" );
					uav_c.pose_pub = n.advertise<hast::posewithheader>(uav_c.s_uav_pose_topic, 1);

				ros::param::get("~slam_uav_autopilot_state_topic", uav_c.s_uav_state_topic); fullfile.writeString("  " + s_full_field + ".topics.slam_uav_autopilot_state_topic = '" + uav_c.s_uav_state_topic + "';\n" );
					uav_c.state_pub   = n.advertise<hast::uavstate>(uav_c.s_uav_state_topic, 1); // publisher for uav autopilot

			// subscribe to nav-data
				ros::param::get("~uav_navdata_topic",s_navData_topic); fullfile.writeString("  "+ s_full_field + ".topics.uav_navdata_topic = '" + s_navData_topic + "';\n" );
					navData_sub 	= n.subscribe(s_navData_topic, 1, &splitSLAM::updateUAVinertial , this);

				ros::param::get("~uav_flight_state_topic", s_uav_flight_state_topic); fullfile.writeString("  " + s_full_field + ".topics.s_uav_flight_state_topic = '"  + s_uav_flight_state_topic + "';\n" );
					uavFlightState_ser = n.advertiseService(s_uav_flight_state_topic, &splitSLAM::uavServeState, this);

			// publish ugv estimated states
				ros::param::get("~ugv1_mux_topic", ugv1.s_mux_topic); fullfile.writeString("  " + s_full_field + ".topics.ugv1_mux_topic = '" + ugv1.s_mux_topic + "';\n" );
				ros::param::get("~ugv2_mux_topic", ugv2.s_mux_topic); fullfile.writeString("  " + s_full_field + ".topics.ugv2_mux_topic = '" + ugv2.s_mux_topic + "';\n" );
					ugv1.est_state_sub = n.subscribe(ugv1.s_mux_topic, 1, &splitSLAM::ugvUpdate, this);
					ugv2.est_state_sub = n.subscribe(ugv2.s_mux_topic, 1, &splitSLAM::ugvUpdate, this);
				ros::param::get("~ugv1_pose_topic", ugv1.s_pose_topic); fullfile.writeString("  " + s_full_field + ".topics.ugv1_pose_topic = '" + ugv1.s_pose_topic + "';\n" );
				ros::param::get("~ugv2_pose_topic", ugv2.s_pose_topic); fullfile.writeString("  " + s_full_field + ".topics.ugv2_pose_topic = '" + ugv2.s_pose_topic + "';\n" );
				ros::param::get("~uav_pose_topic",  uav.s_pose_topic);  fullfile.writeString("  " + s_full_field + ".topics.uav_pose_topic = '"  + uav.s_pose_topic  + "';\n" );
					ugv1.est_state_pub = n.advertise<hast::posewithcov>(ugv1.s_pose_topic, 1);
					ugv2.est_state_pub = n.advertise<hast::posewithcov>(ugv2.s_pose_topic, 1);
					uav.est_state_pub  = n.advertise<hast::posewithcov>(uav.s_pose_topic, 1);

					ugv1.est_pose_msg.header.frame_id = "ugv1"; ugv1.est_pose_msg.header.seq = 0;
					ugv2.est_pose_msg.header.frame_id = "ugv2"; ugv2.est_pose_msg.header.seq = 0;
					uav.est_pose_msg.header.frame_id  = "uav";  uav.est_pose_msg.header.seq  = 0;

				ros::param::get("~use_ugvugv_sensor",ugv_sensor.use_sensor); ROS_INFO("trial: ugv_sensor.use_sensor   = %s", ugv_sensor.use_sensor ? "true" : "false" );
					sprintf (ugv_sensor.s_use_sensor, "%s",  ugv_sensor.use_sensor ? "true" : "false" );
					fullfile.writeString("  " + s_full_field + ".topics.use_ugvugv_sensor = " + ugv_sensor.s_use_sensor  + ";\n" );

			// april messages
				ugv1.goal_id = 1000; ugv2.goal_id = 1000;
				ros::param::get("~max_number_of_tags",tag_max_id); fullfile.writeString("  " + s_full_field + ".params.max_number_of_tags = " + patch::to_string(tag_max_id) + ";\n" );
				ros::param::get("~aprilObstacleCloud_topic",s_aprilObstacleCloud_topic); fullfile.writeString("  " + s_full_field + ".topics.aprilObstacleCloud_topic = '" + s_aprilObstacleCloud_topic + "';\n" );
					aprilObstacleCloud_pub = n.advertise<sensor_msgs::PointCloud2>(s_aprilObstacleCloud_topic, 1);
				ros::param::get("~goal_tag_service_topic",s_goal_tag_service_topic); fullfile.writeString("  " + s_full_field + ".params.goal_tag_service_topic = '" + s_goal_tag_service_topic + "';\n" );
					GoalTagLocation_ser = n.advertiseService(s_goal_tag_service_topic, &splitSLAM::serveGoalLocation , this);
					ROS_INFO("SLAM:: serving tag loactions on %s", s_goal_tag_service_topic.c_str());

			// Shutdown listener
				ros::param::get("~shutdown_topic",s_shutdown_topic); fullfile.writeString("  " + s_full_field + ".topics.s_shutdown_topic = '" + s_shutdown_topic + "';\n" );
					shutdown_sub 	= n.subscribe(s_shutdown_topic,	1, &splitSLAM::nodeShutDown, this);

				ros::param::get("~uavCovInit",uavCovInit);			fullfile.writeString("  " + s_full_field + ".params.uavCovInit = "		+ patch::to_string(uavCovInit) + ";\n" );
				ros::param::get("~ugvCovInit",ugvCovInit);			fullfile.writeString("  " + s_full_field + ".params.ugvCovInit = "		+ patch::to_string(ugvCovInit) + ";\n" );
				ros::param::get("~aprilCovInit",aprilCovInit);	fullfile.writeString("  " + s_full_field + ".params.aprilCovInit = "	+ patch::to_string(aprilCovInit) + ";\n" );
				ros::param::get("~growCovU",growCovU); 					fullfile.writeString("  " + s_full_field + ".params.growCovU = "			+ patch::to_string(growCovU) + ";\n" );
				ros::param::get("~uavQdkScale",uavQdkScale);		fullfile.writeString("  " + s_full_field + ".params.uavQdkScale = "		+ patch::to_string(uavQdkScale) + ";\n" );
				ros::param::get("~uavQwScale",uav.QwScale);			fullfile.writeString("  " + s_full_field + ".params.uav.QwScale = "		+ patch::to_string(uav.QwScale) + ";\n" );
				ros::param::get("~ugvQwScale",ugv1.QwScale);		fullfile.writeString("  " + s_full_field + ".params.ugv1.QwScale = "	+ patch::to_string(ugv1.QwScale) + ";\n" );
				ros::param::get("~ugvQwScale",ugv2.QwScale);		fullfile.writeString("  " + s_full_field + ".params.ugv2.QwScale = "	+ patch::to_string(ugv2.QwScale) + ";\n" );
				ros::param::get("~RkScale",RkScale);						fullfile.writeString("  " + s_full_field + ".params.RkScale = "				+ patch::to_string(RkScale) + ";\n" );
				ros::param::get("~RkXYZ",RkXYZ);								fullfile.writeString("  " + s_full_field + ".params.RkXYZ = "					+ patch::to_string(RkXYZ) + ";\n" );
				ros::param::get("~RkYaw",RkYaw);								fullfile.writeString("  " + s_full_field + ".params.RkYaw = "					+ patch::to_string(RkYaw) + ";\n" );

				ugv1.odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0, 0);
				ugv2.odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0, 0);
				uav.odom_delta  = (cv::Mat_<double>(4, 1) << 0,0,0, 0);

				ugv1.odom_J = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				ugv2.odom_J = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				uav.odom_J  = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

				uav.Qw  = uav.QwScale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				ugv1.Qw  = ugv1.QwScale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				ugv2.Qw  = ugv2.QwScale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

			/*---------  Init UAV ------------- */
				uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.MeasuredVel_lo = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.odom_gl = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.odomPhi = 0;
				uav.EstPhi_gl = 0;
				uav.compassYaw = 0; uav.compassYawLast = 0; uav.deltaPhi = 0;
				uav.cos_compassYaw = 0; uav.cos_compassYawLast = 0;
				uav.sin_compassYaw = 0; uav.sin_compassYawLast = 0;
					uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
					uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
					uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
					uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
					uav.Hlo2gl = invertH(uav.Hgl2lo);

				navTS = 0; navdt = 10000; navTSpublished = 0;
				echoAlt = 0; echoAltLast = 0; deltaAlt = 0;
				navDataCounter = 0;

				uav.Rk  =  RkScale * (cv::Mat_<double>(4, 4) <<
															RkXYZ,  0, 0, 0,
															0,  RkXYZ, 0, 0,
															0,  0, RkXYZ, 0,
															0,  0, 0, RkYaw); // measurement covariance of landmarks

			/*---------  Init slam ------------- */
			// create vector of landmarks
				april.reserve( tag_max_id );
				for(uint i = 0; i != tag_max_id; i++) //init april tags
				{
					apriltagclass obj;
					obj.id = i;
					obj.isgoal=false; // define tag #7 as the goal landmark
					// if (i==7){obj.isgoal = true;} else {obj.isgoal=false;} // define tag #7 as the goal landmark
					obj.Rk_uav =  RkScale * (cv::Mat_<double>(4, 4) <<
															RkXYZ,  0, 0, 0,
															0,  RkXYZ, 0, 0,
															0,  0, RkXYZ, 0,
															0,  0, 0, RkYaw); // measurement covariance of landmarks
					obj.s_TF_global = s_ref_TF_frame;
					obj.s_TF_tag = "hast/jointslam/tag_" + patch::to_string(obj.id);
					april.push_back(obj);
				}
				tag_counter = 0;

				fullSLAM.firstTimeForID.assign(tag_max_id, 0); //slam.firstTimeForID.assign(tag_max_id, 0);
				fullSLAM.updateForID.assign(tag_max_id, 0); //slam.updateForID.assign(tag_max_id, 0);
				fullSLAM.TagsInFOV.assign(tag_max_id, 0); //slam.TagsInFOV.assign(tag_max_id, 0);

				fullSLAM.slamCount = 1; //slam.slamCount = 1;
				fullSLAM.downimagecountup = 0; ///slam.downimagecountup = 0; // number of images to ignore before starting slam
				fullSLAM.NumberOfTagsinFOV = 0; //slam.NumberOfTagsinFOV = 0;
				fullSLAM.numberOfLandmarksOrdered = 0; //slam.numberOfLandmarksOrdered = 0;

				fullSLAM.I4 = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				fullSLAM.nI4 = -1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				fullSLAM.z4 = (cv::Mat_<double>(4, 4) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);

				// the augmented covariance will only grow from this size
				fullSLAM.augmentedCovariance = uavCovInit*fullSLAM.I4; //slam.augmentedCovariance = uavCovInit*slam.I4;

				fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //uav x
				fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //uav y
				fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //uav z
				fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //uav yaw

				for(uint i = 0; i != numberOfUGV; i++) //init UGVs
				{
					fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //ugv x
					fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //ugv y
					fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //ugv z
					fullSLAM.augmentedState.push_back(0); //slam.augmentedState.push_back(0); //ugv yaw

					growCov_highU(fullSLAM.z4);
					cv::Mat initUGVcov = ugvCovInit*fullSLAM.I4;
					initUGVcov.rowRange(0,4).colRange(0,4).copyTo(fullSLAM.augmentedCovariance.rowRange(4*(i+1),4*(i+2)).colRange(4*(i+1),4*(i+2)));
				}

				fullfile.cellVecAugmentedState_nx4(fullSLAM.augmentedState,			"  " + s_full_field + ".startup.augmentedState", fullSLAM.slamCount);
				fullfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance,	"  " + s_full_field + ".startup.augmentedCovariance", fullSLAM.slamCount);

				ros::param::get("~absolute_measurements",absolute_measurements); 	ROS_WARN(":~:~:~:~:~:~:~: fullSLAM.absolute_measurements :: %s", absolute_measurements ? "true" : "false" );
		}

		// serve uav navdata state
		bool uavServeState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res)
		{
			ROS_WARN("uavServeState() : %u", flightState);
			res.state = flightState;
			return true;
		}

		bool slamOnOffSwitch(hast::slamswitch::Request &req, hast::slamswitch::Response &res)
		{ // software control, switch between SLAM and External pose estimates for UAV
			if(req.flip)
			{// True -> UAV pose is estimated via slam
				ROS_INFO("  fullSLAM: Begin using images for slam");
				slamSwitch = true;
				slam_flag.flag = true;
				slam_stateFlag_pub.publish(slam_flag);
				// ROS_INFO("  fullSLAM.topics.s_slamState_topic = ['%s'];",  s_slam_stateFlag_topic.c_str());
				// ROS_INFO("  slamState = ['%s']", slam_flag.flag ? "true" : "false" );

				// GoalTagLocation_ser = n.advertiseService(s_goal_tag_service_topic, &splitSLAM::serveGoalLocation , this);
				// ROS_INFO("SLAM:: serving tag loactions on %s", s_goal_tag_service_topic.c_str());


				// Subscribe to tag detections
					ros::param::get("~tag_detection_topic",s_tagDetections); fullfile.writeString("  "+ s_full_field + ".topics.tag_detection_topic =  '" + s_tagDetections + "';\n" );
					TagDetections_sub 	= n.subscribe(s_tagDetections, 1, &splitSLAM::tagDetections , this);


				double qroll, qpitch, qyaw;
				tf::Vector3 t_uav_pose_gl(req.pose.position.x,req.pose.position.y,req.pose.position.z);
				tf::Quaternion q_uav_pose_gl(req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z,req.pose.orientation.w);
				tf::Transform Xform_uav_pose_gl(q_uav_pose_gl, t_uav_pose_gl);
				tf::Matrix3x3(Xform_uav_pose_gl.getRotation()).getRPY(qroll, qpitch, qyaw);

				fullSLAM.augmentedState[0] = req.pose.position.x; //slam.augmentedState[0] = req.pose.position.x;
				fullSLAM.augmentedState[1] = req.pose.position.y; //slam.augmentedState[1] = req.pose.position.y;
				fullSLAM.augmentedState[2] = req.pose.position.z; //slam.augmentedState[2] = req.pose.position.z;
				fullSLAM.augmentedState[3] = qyaw; //slam.augmentedState[3] = qyaw;

				uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << req.pose.position.x,req.pose.position.y,req.pose.position.z);
					uav.EstPhi_gl = qyaw; //stay in radians where possible

				uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
				uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
				uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
				uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
				uav.Hlo2gl = invertH(uav.Hgl2lo);

				fullfile.writeString("%% --------------- init slam from switch --------------- \n" );
				fprintf(fullfile.filename, "  %s.slam_init.init_time = % -6.4f;\n", s_full_field.c_str(), init_time);
				fprintf(fullfile.filename, "  %s.slam_init.uavt(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n",	s_full_field.c_str(), fullSLAM.slamCount, fullSLAM.augmentedState[0], fullSLAM.augmentedState[1], fullSLAM.augmentedState[2]);
				fprintf(fullfile.filename, "  %s.slam_init.uav.EstPhi_gl(%u,:) = % -6.4f;\n", s_full_field.c_str(), fullSLAM.slamCount, uav.EstPhi_gl);
				fullfile.cellCvMatDoubles(uav.EstPosition_gl, 	  			"  " + s_full_field + ".slam_init.uav.EstPosition_gl",						fullSLAM.slamCount);
				fullfile.writeCvMatDoubles(uav.Hgl2lo,									"  " + s_full_field + ".slam_init.uav.Hgl2lo", 										fullSLAM.slamCount);
				fullfile.writeCvMatDoubles(uav.Hlo2gl,									"  " + s_full_field + ".slam_init.uav.Hlo2gl", 										fullSLAM.slamCount);
				fullfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance,	"  " + s_full_field + ".slam_init.augmentedCovariance", fullSLAM.slamCount);
				fullfile.cellVecAugmentedState_nx4(fullSLAM.augmentedState,			"  " + s_full_field + ".augmentedState", 								fullSLAM.slamCount);

				// mfile.writeString("%% --------------- init slam from switch --------------- \n" );
				// fprintf(mfile.filename, "  %s.slam_init.time(%u,:) = % -6.4f;\n",	s_matlab_field.c_str(), slam.slamCount, ros::Time::now().toSec());
				// fprintf(mfile.filename, "  %s.slam_init.uavt(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n",	s_matlab_field.c_str(), slam.slamCount, slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2]);
				// fprintf(mfile.filename, "  %s.slam_init.uav.EstPhi_gl(%u,:) = % -6.4f;\n", s_matlab_field.c_str(), slam.slamCount, uav.EstPhi_gl);
				// mfile.cellCvMatDoubles(uav.EstPosition_gl, 	  			"  "+ s_matlab_field + ".slam_init.uav.EstPosition_gl",		slam.slamCount);
				// mfile.writeCvMatDoubles(uav.Hgl2lo,									"  "+ s_matlab_field + ".slam_init.uav.Hgl2lo", 					slam.slamCount);
				// mfile.writeCvMatDoubles(uav.Hlo2gl,									"  "+ s_matlab_field + ".slam_init.uav.Hlo2gl", 					slam.slamCount);
				// mfile.cellCvMatDoubles_multline(slam.augmentedCovariance,	"  " + s_matlab_field + ".slam_init.augmentedCovariance", slam.slamCount);
				// mfile.cellVecAugmentedState_nx4(slam.augmentedState,			"  " + s_matlab_field + ".augmentedState", 								slam.slamCount);

				// writeCovTrace(ros::Time::now().toSec());

				res.flop = true;

				// start debugging counter
				fullSLAM.ugv1count = 0; //slam.ugv1count = 0;
				fullSLAM.ugv2count = 0; //slam.ugv2count = 0;
				fullSLAM.tagcount = 0; //slam.tagcount = 0;
			}
			if(!req.flip)
			{// false -> UAV pose is estimated externally
				ROS_INFO("  fullSLAM: deactivate slam");
				slamSwitch = false;
				slam_flag.flag = false;
				slam_stateFlag_pub.publish(slam_flag);
				// ROS_INFO("  fullSLAM.topics.s_slamState_topic = ['%s'];",  s_slam_stateFlag_topic.c_str());
				// ROS_INFO("  slamState = ['%s']", slam_flag.flag ? "true" : "false" );

				res.flop = false;

			}
			return true; //this needs to be true, otherwise the service doesn't send the response.
		}// end slamOnOffSwitch(hast::slamswitch::Request &req, hast::slamswitch::Response &res)

		void augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj)
		{
			// mfile.writeString("%% --------------- void augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj)  --------------- \n\n" );
			// add new measurement of landmark to the augmented state (slam.est)
			fullSLAM.augmentedState.push_back(newLandmarkPos.at<double>(0,0)); 	//slam.augmentedState.push_back(newLandmarkPos.at<double>(0,0)); //x
			fullSLAM.augmentedState.push_back(newLandmarkPos.at<double>(1,0)); 	//slam.augmentedState.push_back(newLandmarkPos.at<double>(1,0)); //y
			fullSLAM.augmentedState.push_back(newLandmarkPos.at<double>(2,0)); 	//slam.augmentedState.push_back(newLandmarkPos.at<double>(2,0)); //z
			fullSLAM.augmentedState.push_back(newLandmarkYaw); 									//slam.augmentedState.push_back(newLandmarkYaw); //yaw

			fullSLAM.landmarksOrdered.push_back(newLandmarkID);
			// slam.landmarksOrdered.push_back(newLandmarkID); //this is useful for tracking the number of landmarks
			++fullSLAM.numberOfLandmarksOrdered;
			// ++slam.numberOfLandmarksOrdered; // this will be useful for determining size of augmented P matrix
			fullSLAM.timeOflandmarksOrdered.push_back(ros::Time::now().toSec());
			// slam.timeOflandmarksOrdered.push_back(ros::Time::now().toSec());
			growCov_highU(Rj);
			// growCov(Rj);
		}

		void growCov(cv::Mat Rj)
		{
			// 1) create new rows
			cv::Mat newrowsA, newcolsA;
				int cols0 = fullSLAM.augmentedCovariance.cols;
				// submatrix are the top 4 rows of the current P matrix, vconcat and hconcat don't follow zero index ideas
				cv::Mat submatrix = fullSLAM.augmentedCovariance.colRange(0,cols0).rowRange(0,4);
				submatrix.copyTo(newrowsA);
				// add top 4 rows to bottom of augmented matrix
				// cv::vconcat(slam.augmentedCovariance, newrowsA, slam.augmentedCovariance);
				cv::vconcat(fullSLAM.augmentedCovariance, newrowsA, fullSLAM.augmentedCovariance);
			// 2) define new columns, then add Cm3 to bottom of new columns
				newcolsA = newrowsA.t();
				cv::Mat CMn = fullSLAM.augmentedCovariance.rowRange(0,4).colRange(0,4)+Rj;
				// cv::Mat CMn = fullSLAM.augmentedCovariance.rowRange(0,4).colRange(0,4)+Rj;
				cv::vconcat(newcolsA, CMn, newcolsA);
			// 3) add new columns to right side of augmented matrix
				// cv::hconcat(slam.augmentedCovariance, newcolsA, slam.augmentedCovariance);
				cv::hconcat(fullSLAM.augmentedCovariance, newcolsA, fullSLAM.augmentedCovariance);
		}

		void growCov_highU(cv::Mat Rj)
		{
			// 1) create new rows
				int cols0 = fullSLAM.augmentedCovariance.cols;
				// add top 4 rows to bottom of augmented matrix
				cv::Mat newrowsA(cv::Size(cols0,4), fullSLAM.augmentedCovariance.type(), cv::Scalar(growCovU));
				// cv::vconcat(slam.augmentedCovariance, newrowsA, slam.augmentedCovariance);
				cv::vconcat(fullSLAM.augmentedCovariance, newrowsA, fullSLAM.augmentedCovariance);
			// 2) define new columns, then add Cmn to bottom of new columns
				cv::Mat newcolsA = newrowsA.t();
				cv::Mat CMn = fullSLAM.augmentedCovariance.colRange(0,4).rowRange(0,4)+Rj;
				cv::vconcat(newcolsA, CMn, newcolsA);
			// 3) add new columns to right side of augmented matrix
				// cv::hconcat(slam.augmentedCovariance, newcolsA, slam.augmentedCovariance);
				cv::hconcat(fullSLAM.augmentedCovariance, newcolsA, fullSLAM.augmentedCovariance);
		}

		void ugvUpdate(const hast::ugvmux::ConstPtr& mux_msg)
		{
			int ugvN_id, ugvK_id;

			std::string s_ugv_id = mux_msg-> ugvEst.vehicle_id;

			// add data import of ugv/ugv stereo measurement

			int mux_seq = mux_msg -> seq;
			double mux_time = mux_msg -> stamp;
			ros::Time mux_time_rxd = ros::Time::now();
			double ugvEst_yaw     = mux_msg-> ugvEst.yaw; // current yaw estimate of measuring ugv
			double ugvEst_yaw_cov = mux_msg-> ugvEst.yaw_cov; // covariance of current yaw estimate
			double stereo_meas_yaw     = mux_msg-> stereoMeas.yaw; //
			double stereo_meas_yaw_cov = mux_msg-> stereoMeas.yaw_cov;
			cv::Mat ugvEst_P = (cv::Mat_<double>(3, 1) << mux_msg-> ugvEst.position.x, mux_msg-> ugvEst.position.y, mux_msg-> ugvEst.position.z);
			cv::Mat odomDelta = (cv::Mat_<double>(4, 1) << mux_msg->odomDelta.x, mux_msg->odomDelta.y, mux_msg->odomDelta.z, mux_msg->odomDelta.w);
			cv::Mat ugv_odomJ = (cv::Mat_<double>(4, 4) << 1,0,0,-mux_msg->odomDelta.y, 0,1,0,mux_msg->odomDelta.x, 0,0,1,0, 0,0,0,1);

			cv::Mat stereo_meas_P = (cv::Mat_<double>(3, 1) << mux_msg-> stereoMeas.position.x, mux_msg-> stereoMeas.position.y, mux_msg-> stereoMeas.position.z);
			cv::Mat stereo_meas_P_cov = 100*(cv::Mat_<double>(3, 3) <<
					mux_msg-> stereoMeas.PCov.row0.x, mux_msg-> stereoMeas.PCov.row0.y, mux_msg-> stereoMeas.PCov.row0.z,
					mux_msg-> stereoMeas.PCov.row1.x, mux_msg-> stereoMeas.PCov.row1.y, mux_msg-> stereoMeas.PCov.row1.z,
					mux_msg-> stereoMeas.PCov.row2.x, mux_msg-> stereoMeas.PCov.row2.y, mux_msg-> stereoMeas.PCov.row2.z);

			if (s_ugv_id.compare("ugv1") == 0) {ugvN_id = 1; ugvK_id = 2;}  // ugvK is used for the ugv-ugv sensor
			if (s_ugv_id.compare("ugv2") == 0) {ugvN_id = 2; ugvK_id = 1;}

			fprintf(fullfile.filename,"\n%% --------------- %s Update --------------- %% \n", s_ugv_id.c_str());
			fprintf(fullfile.filename,"  %% --------------- mux message --------------- %% \n");
			fprintf(fullfile.filename,"    %s.mux_%s.mux_time(%u,:) = % -6.4f;\n", 																				s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_time);
			fprintf(fullfile.filename,"    %s.mux_%s.mux_time_rxd(%u,:) = % -6.4f;\n", 																		s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_time_rxd.toSec() - init_time);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.yaw(%u,:)     = % -6.4f;\n", 																	s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, ugvEst_yaw);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.yaw_cov(%u,:) = % -6.4f;\n", 																	s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, ugvEst_yaw_cov);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.odomDelta(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n",					s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg-> odomDelta.x, mux_msg-> odomDelta.y, mux_msg-> odomDelta.z);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.stereo_meas_yaw(%u,:)     = % -6.4f;\n",											s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, stereo_meas_yaw);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.stereo_meas_yaw_cov(%u,:) = % -6.4f;\n",											s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, stereo_meas_yaw_cov);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.P(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n",									s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg-> ugvEst.position.x, mux_msg-> ugvEst.position.y, mux_msg-> ugvEst.position.z);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.odomDelta_yaw(%u,:) = % -6.14f;\n",														s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg-> odomDelta.w);
			fprintf(fullfile.filename,"    %s.mux_%s.ugvEst.stereo_meas_P(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n",			s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg-> stereoMeas.position.x, mux_msg-> stereoMeas.position.y, mux_msg-> stereoMeas.position.z);
			fullfile.cellCvMatDoubles_multline(stereo_meas_P_cov,	"    " + s_full_field + ".mux_" + s_ugv_id + ".ugvEst.stereo_meas_P_cov", mux_seq);
			fullfile.cellCvMatDoubles_multline(ugv_odomJ,	"    " + s_full_field + ".mux_" + s_ugv_id + ".ugvEst.ugv_odomJ", mux_seq);

			fprintf(fullfile.filename,"  %% ------------ ugv_sensor mux message ---------------  ugv_sensor.inFOV = %s%% \n", mux_msg->stereoUGV_inFOV ? "true" : "false" );
			double stereo_meas_UGV_yaw     = mux_msg-> stereoUGV.yaw;
			double stereo_meas_UGV_yaw_cov = mux_msg-> stereoUGV.yaw_cov;
			cv::Mat stereo_meas_UGV_P = (cv::Mat_<double>(3, 1) << mux_msg-> stereoUGV.position.x, mux_msg-> stereoUGV.position.y, mux_msg-> stereoUGV.position.z);
			cv::Mat stereo_meas_UGV_P_cov = 100*(cv::Mat_<double>(3, 3) <<
					mux_msg-> stereoUGV.PCov.row0.x, mux_msg-> stereoUGV.PCov.row0.y, mux_msg-> stereoUGV.PCov.row0.z,
					mux_msg-> stereoUGV.PCov.row1.x, mux_msg-> stereoUGV.PCov.row1.y, mux_msg-> stereoUGV.PCov.row1.z,
					mux_msg-> stereoUGV.PCov.row2.x, mux_msg-> stereoUGV.PCov.row2.y, mux_msg-> stereoUGV.PCov.row2.z);

			fprintf(fullfile.filename,"    %s.mux_%s.ugv_sensor.inFOV(%u,:) = %s;\n",																s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg->stereoUGV_inFOV ? "true" : "false" );
			fprintf(fullfile.filename,"    %s.mux_%s.ugv_sensor.meas_yaw(%u,:)     = % -6.4f;\n",											s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, stereo_meas_UGV_yaw);
			fprintf(fullfile.filename,"    %s.mux_%s.ugv_sensor.meas_yaw_cov(%u,:) = % -6.14f;\n",										s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, stereo_meas_UGV_yaw_cov);
			fprintf(fullfile.filename,"    %s.mux_%s.ugv_sensor.meas_P(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n",	s_full_field.c_str(), s_ugv_id.c_str(), mux_seq, mux_msg-> stereoUGV.position.x, mux_msg-> stereoUGV.position.y, mux_msg-> stereoUGV.position.z);
			fullfile.cellCvMatDoubles_multline(stereo_meas_UGV_P_cov,	"    " + s_full_field + ".mux_" + s_ugv_id + ".ugv_sensor.meas_P_cov", mux_seq);

			if(!slamSwitch)
			{ //do nothing? still initializing both ugvs
				fullSLAM.augmentedState[4*(ugvN_id)+0] = mux_msg-> ugvEst.position.x;
				fullSLAM.augmentedState[4*(ugvN_id)+1] = mux_msg-> ugvEst.position.y;
				fullSLAM.augmentedState[4*(ugvN_id)+2] = mux_msg-> ugvEst.position.z;
				fullSLAM.augmentedState[4*(ugvN_id)+3] = mux_msg-> ugvEst.yaw;
			} else 	{ // run slam
				// Add the odom delta from the mux message to the state prior
				fullSLAM.augmentedState[4*(ugvN_id)+0] += mux_msg-> odomDelta.x;
				fullSLAM.augmentedState[4*(ugvN_id)+1] += mux_msg-> odomDelta.y;
				fullSLAM.augmentedState[4*(ugvN_id)+2] += mux_msg-> odomDelta.z;
				fullSLAM.augmentedState[4*(ugvN_id)+3] += mux_msg-> odomDelta.w;

				if (s_ugv_id.compare("ugv1") == 0) {
					ugv1.obsCount +=1;
					// slam.ugvQdk = ugv_odomJ * ugv1.Qw * ugv_odomJ.t();
					fullSLAM.ugvQdk = ugv_odomJ * ugv1.Qw * ugv_odomJ.t();
					if (ugv1.obsCount>2){
						ugv1.slamFlag = true;
						if (ugv_sensor.use_sensor && mux_msg->stereoUGV_inFOV) {
							ugv2.slam_pose_msg.observedByUGV = true; // use this flag to reset vehicle odom after slam update
						}
					}
					fprintf(fullfile.filename,"   try waitbar(%f/waitbar_max,wb); end\n", double(ugv1.obsCount));
				}
				if (s_ugv_id.compare("ugv2") == 0) {
					ugv2.obsCount +=1;
					// slam.ugvQdk = ugv_odomJ * ugv2.Qw * ugv_odomJ.t();
					fullSLAM.ugvQdk = ugv_odomJ * ugv2.Qw * ugv_odomJ.t();
					if (ugv2.obsCount>2){
						ugv2.slamFlag = true;
						if (ugv_sensor.use_sensor && mux_msg->stereoUGV_inFOV) {
							ugv1.slam_pose_msg.observedByUGV = true; // use this flag to reset vehicle odom after slam update
						}
					}
				}

				fprintf(fullfile.filename,"\n  %% --------------- Slam data --------------- %% \n");
				fullSLAM.slamCount++; //slam.slamCount++;
				fprintf(fullfile.filename,"  %s.time(%u,:) = % -14.14f;\n",				s_full_field.c_str(), fullSLAM.slamCount, mux_time_rxd.toSec() - init_time);
				fullfile.cellVecAugmentedState_nx4(fullSLAM.augmentedState,				"  " + s_full_field + ".augmentedState",	fullSLAM.slamCount);
				fullfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance,	"  " + s_full_field + ".augmentedCovariance", fullSLAM.slamCount);

				/* ~~~~~ update slam covariance matrix for uav and ugv */
					//Update covariance of UAV from motion:
					uav.odom_J  = (cv::Mat_<double>(4, 4) <<
							1,0,0,-uav.odom_delta.at<double>(1,0),
							0,1,0, uav.odom_delta.at<double>(0,0),
							0,0,1,0,
							0,0,0,1);
				/* ~~~~~ update fullSLAM covariance matrix for uav and ugv */
					cv::Mat Jf = cv::Mat::eye(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a identity matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
							uav.odom_J.copyTo(Jf.colRange(0,4).rowRange(0,4)); // add J_uav
							ugv_odomJ.copyTo(Jf.rowRange(4*(ugvN_id),4*(ugvN_id+1)).colRange(4*(ugvN_id),4*(ugvN_id+1))); // add J_ugvN

					cv::Mat Qdk = cv::Mat::zeros(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a zero matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
							uav.Qdk = uav.odom_J * uav.Qw * uav.odom_J.t();
							uav.Qdk.copyTo(Qdk.colRange(0,4).rowRange(0,4));
							fullSLAM.ugvQdk.copyTo(Qdk.rowRange(4*(ugvN_id),4*(ugvN_id+1)).colRange(4*(ugvN_id),4*(ugvN_id+1)));


				cv::Mat SLAM_Rk;
				// if (ugv_sensor.use_sensor && mux_msg->stereoUGV_inFOV && (s_ugv_id.compare("ugv1") == 0))
				if (ugv_sensor.use_sensor && mux_msg->stereoUGV_inFOV)
				{ // then use both uav and ugv measurements
					fullSLAM.zk = (cv::Mat_<double>(8, 1) <<
						mux_msg->stereoMeas.position.x, mux_msg->stereoMeas.position.y, mux_msg->stereoMeas.position.z, mux_msg->stereoMeas.yaw,
						mux_msg->stereoUGV.position.x,  mux_msg->stereoUGV.position.y,  mux_msg->stereoUGV.position.z,  mux_msg->stereoUGV.yaw );
						// fprintf(fullfile.filename,"  %%     fullSLAM.zk = (cv::Mat_<double>(%i, %i)); \n", fullSLAM.zk.rows, fullSLAM.zk.cols);

					SLAM_Rk = (cv::Mat_<double>(8, 8) <<
						mux_msg-> stereoMeas.PCov.row0.x, mux_msg-> stereoMeas.PCov.row0.y, mux_msg-> stereoMeas.PCov.row0.z, 0, 0,0,0,0,
						mux_msg-> stereoMeas.PCov.row1.x, mux_msg-> stereoMeas.PCov.row1.y, mux_msg-> stereoMeas.PCov.row1.z, 0, 0,0,0,0,
						mux_msg-> stereoMeas.PCov.row2.x, mux_msg-> stereoMeas.PCov.row2.y, mux_msg-> stereoMeas.PCov.row2.z, 0, 0,0,0,0,
						0,0,0,mux_msg-> stereoMeas.yaw_cov, 0,0,0,0,
						0,0,0,0, mux_msg-> stereoUGV.PCov.row0.x, mux_msg-> stereoUGV.PCov.row0.y, mux_msg-> stereoUGV.PCov.row0.z, 0,
						0,0,0,0, mux_msg-> stereoUGV.PCov.row1.x, mux_msg-> stereoUGV.PCov.row1.y, mux_msg-> stereoUGV.PCov.row1.z, 0,
						0,0,0,0, mux_msg-> stereoUGV.PCov.row2.x, mux_msg-> stereoUGV.PCov.row2.y, mux_msg-> stereoUGV.PCov.row2.z, 0,
						0,0,0,0, 0,0,0,mux_msg-> stereoUGV.yaw_cov);

						cv::Mat ugv_senor_Q = 1.0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,10);
						ugv_senor_Q.copyTo(Qdk.rowRange(4*(ugvK_id),4*(ugvK_id+1)).colRange(4*(ugvK_id),4*(ugvK_id+1)));

					// for fullSLAM, the H matrix is the same scale as the whole system.
					fullSLAM.H = cv::Mat::zeros(8, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a zero matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
						// the measurement of [uav] - [ugvN]
						fullSLAM.I4.copyTo( fullSLAM.H.rowRange(0,4).colRange(0,4));
						fullSLAM.nI4.copyTo(fullSLAM.H.rowRange(0,4).colRange(4*(ugvN_id),4*(ugvN_id+1)));
						// the measurement of [ugvK] - [ugvN]
						fullSLAM.nI4.copyTo(fullSLAM.H.rowRange(4,8).colRange(4*(ugvN_id),4*(ugvN_id+1)));
						fullSLAM.I4.copyTo( fullSLAM.H.rowRange(4,8).colRange(4*(ugvK_id),4*(ugvK_id+1)));
				} else {
					// use only uav measurements
					fullSLAM.zk = (cv::Mat_<double>(4, 1) <<
					mux_msg->stereoMeas.position.x, mux_msg->stereoMeas.position.y, mux_msg->stereoMeas.position.z, mux_msg-> stereoMeas.yaw );

					SLAM_Rk = (cv::Mat_<double>(4, 4) <<
					mux_msg-> stereoMeas.PCov.row0.x, mux_msg-> stereoMeas.PCov.row0.y, mux_msg-> stereoMeas.PCov.row0.z, 0,
					mux_msg-> stereoMeas.PCov.row1.x, mux_msg-> stereoMeas.PCov.row1.y, mux_msg-> stereoMeas.PCov.row1.z, 0,
					mux_msg-> stereoMeas.PCov.row2.x, mux_msg-> stereoMeas.PCov.row2.y, mux_msg-> stereoMeas.PCov.row2.z, 0,
					0,0,0,mux_msg-> stereoMeas.yaw_cov);

					// for fullSLAM, the H matrix is the same scale as the whole system.
					fullSLAM.H = cv::Mat::zeros(4, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a zero matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
					fullSLAM.I4.copyTo(fullSLAM.H.rowRange(0,4).colRange(0,4)); // add uav
					fullSLAM.nI4.copyTo(fullSLAM.H.rowRange(0,4).colRange(4*(ugvN_id),4*(ugvN_id+1))); //add ugv // this should write data to either columns [4-8] or [8-12]
					// fullfile.cellCvMatDoubles_multline(fullSLAM.H, "  " + s_full_field + ".slam.H", fullSLAM.slamCount);
				}

					fullSLAM.Pf = Jf * fullSLAM.augmentedCovariance * Jf.t() + Qdk;
					fullfile.cellCvMatDoubles_multline(Qdk, "  " + s_full_field + ".slam.Qdk", fullSLAM.slamCount);
						Qdk.release();
						Jf.release();

					fullfile.cellCvMatDoubles_multline(SLAM_Rk, "  " + s_full_field + ".slam.Rk", fullSLAM.slamCount);
					fullfile.cellCvMatDoubles_multline(fullSLAM.Pf, "  " + s_full_field + ".slam.Pf", fullSLAM.slamCount);

				// update fullSLAM.EKF equations
					// fprintf(fullfile.filename,"  %%     fullSLAM.PHt = fullSLAM.Pf * fullSLAM.H.t() = [%i, %i] * [%i, %i]; \n", fullSLAM.Pf.rows, fullSLAM.Pf.cols, fullSLAM.H.cols, fullSLAM.H.rows);
					fullSLAM.PHt = fullSLAM.Pf * fullSLAM.H.t();
					// fprintf(fullfile.filename,"  %%     fullSLAM.HPHt = fullSLAM.H * fullSLAM.PHt = [%i, %i] * [%i, %i]; \n", fullSLAM.H.rows, fullSLAM.H.cols, fullSLAM.PHt.rows, fullSLAM.PHt.cols);
					fullSLAM.HPHt = fullSLAM.H * fullSLAM.PHt;
					// fprintf(fullfile.filename,"  %%     fullSLAM.Sk = fullSLAM.HPHt + SLAM_Rk = [%i, %i] * [%i, %i]; \n", fullSLAM.HPHt.rows, fullSLAM.HPHt.cols, SLAM_Rk.rows, SLAM_Rk.cols);
					fullSLAM.Sk = fullSLAM.HPHt + SLAM_Rk;
					fullSLAM.Sk_inv = fullSLAM.Sk.inv();
					// fprintf(fullfile.filename,"  %%     fullSLAM.Kk = fullSLAM.PHt * fullSLAM.Sk_inv = [%i, %i] * [%i, %i]; \n", fullSLAM.PHt.rows, fullSLAM.PHt.cols, fullSLAM.Sk_inv.rows, fullSLAM.Sk_inv.cols);
					fullSLAM.Kk = fullSLAM.PHt * fullSLAM.Sk_inv;
					// fprintf(fullfile.filename,"  %%     fullSLAM.KH = fullSLAM.Kk * fullSLAM.H = [%i, %i] * [%i, %i]; \n", fullSLAM.Kk.rows, fullSLAM.Kk.cols, fullSLAM.H.rows, fullSLAM.H.cols);
					fullSLAM.KH = fullSLAM.Kk * fullSLAM.H;
					fullSLAM.x.release(); cv::Mat(fullSLAM.augmentedState).copyTo(fullSLAM.x);
					// fprintf(fullfile.filename,"  %%     fullSLAM.Hx = fullSLAM.H*fullSLAM.x = [%i, %i] * [%i, %i]; \n", fullSLAM.H.rows, fullSLAM.H.cols, fullSLAM.x.rows, fullSLAM.x.cols);
					fullSLAM.Hx = fullSLAM.H*fullSLAM.x;
					// fprintf(fullfile.filename,"  %%     fullSLAM.correction = fullSLAM.Kk * (fullSLAM.zk - fullSLAM.Hx) = [%i, %i] * ([%i, %i] - [%i, %i]); \n", fullSLAM.Kk.rows, fullSLAM.Kk.cols, fullSLAM.zk.rows, fullSLAM.zk.cols, fullSLAM.Hx.rows, fullSLAM.Hx.cols);
					fullSLAM.correction = fullSLAM.Kk * (fullSLAM.zk - fullSLAM.Hx);
					fullSLAM.KHI = cv::Mat::eye(fullSLAM.KH.rows, fullSLAM.KH.cols, CV_64F); //64f == double
					// fprintf(fullfile.filename,"  %%     fullSLAM.POST_P = (fullSLAM.KHI - fullSLAM.Kk*fullSLAM.H) * fullSLAM.Pf = ([%i, %i] - [%i, %i] * [%i, %i]); \n", fullSLAM.KHI.rows, fullSLAM.KHI.cols, fullSLAM.Kk.rows, fullSLAM.Kk.cols, fullSLAM.H.rows, fullSLAM.H.cols);
					fullSLAM.POST_P = (fullSLAM.KHI - fullSLAM.Kk*fullSLAM.H) * fullSLAM.Pf;
					fullSLAM.POST_P.copyTo(fullSLAM.augmentedCovariance);

					// update states
						for(uint state = 0; state != fullSLAM.augmentedState.size(); state++){fullSLAM.augmentedState[state]+=fullSLAM.correction.at<double>(state,0);}
						fullfile.cellCvMatDoubles_multline(fullSLAM.H, 					"  " + s_full_field + ".slam.H", 					fullSLAM.slamCount);
						fullfile.cellMatAugmentedState_nx4(fullSLAM.x, 					"  " + s_full_field + ".slam.x", 					fullSLAM.slamCount);
						fullfile.cellVecAugmentedState_nx4(fullSLAM.zk,					"  " + s_full_field + ".slam.zk", 				fullSLAM.slamCount);
						fullfile.cellMatAugmentedState_nx4(fullSLAM.Hx, 				"  " + s_full_field + ".slam.Hx", 				fullSLAM.slamCount);
						fullfile.cellMatAugmentedState_nx4(fullSLAM.correction, "  " + s_full_field + ".slam.correction", fullSLAM.slamCount);
						fullfile.cellVecAugmentedState_nx4(fullSLAM.augmentedState,	"  " + s_full_field + ".slam.augmentedState", fullSLAM.slamCount);
						fullfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance, 		"  " + s_full_field + ".augmentedCovariance", 		fullSLAM.slamCount);

				// update uav pose
					uavPublishEst();
					publishSLAM(ros::Time::now(), false, ugvN_id);

				if (s_ugv_id.compare("ugv1") == 0)
				{
					ugv1.est_pose_msg.header.seq += 1;
					ugv1.est_pose_msg.header.stamp = ros::Time::now();
					ugv1.est_pose_msg.position.x = fullSLAM.augmentedState[4*(ugvN_id)+0];
					ugv1.est_pose_msg.position.y = fullSLAM.augmentedState[4*(ugvN_id)+1];
					ugv1.est_pose_msg.position.z = fullSLAM.augmentedState[4*(ugvN_id)+2];
					ugv1.est_pose_msg.yaw				 = fullSLAM.augmentedState[4*(ugvN_id)+3];
					// ugv1.est_state_pub.publish(ugv1.est_pose_msg);

					ugv1.EstPosition_gl = (cv::Mat_<double>(3, 1) <<
						fullSLAM.augmentedState[4*(ugvN_id)+0],
						fullSLAM.augmentedState[4*(ugvN_id)+1],
						fullSLAM.augmentedState[4*(ugvN_id)+2]);
					// ugv1.EstPhi_gl = wrapRadians(fullSLAM.augmentedState[4*(ugvN_id)+3]);
					ugv1.EstPhi_gl = (fullSLAM.augmentedState[4*(ugvN_id)+3]);

					fprintf(fullfile.filename,"  %s.%s.est.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n",
							s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.slamCount,
							ugv1.EstPosition_gl.at<double>(0,0),
							ugv1.EstPosition_gl.at<double>(1,0),
							ugv1.EstPosition_gl.at<double>(2,0));

					fprintf(fullfile.filename,"  %s.%s.est.yaw.global(%u,:) = % -16.14f;\n",	s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.slamCount, ugv1.EstPhi_gl);

					fullSLAM.ugv1count+=1; //slam.ugv1count+=1;

					fprintf(fullfile.filename,"  %s.%s.uav_est.time(%u,:) = % -16.14f;\n", 															s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv1count, ugv1.est_pose_msg.header.stamp.toSec()-init_time);
					fprintf(fullfile.filename,"  %s.%s.uav_est.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", 	s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv1count, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.%s.uav_est.yaw.global(%u,:) = % -16.14f;\n", 												s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv1count, uav.EstPhi_gl);
					fprintf(fullfile.filename,"  %s.slam.postP.ugv1_time(%u,1) = % -16.14f;\n", 												s_full_field.c_str(), fullSLAM.ugv1count, ugv1.est_pose_msg.header.stamp.toSec()-init_time);
					// fullfile.cellCvMatDoubles_multline(postP, "  " + s_full_field + ".slam.postP.ugv1", fullSLAM.ugv1count);
				}

				if (s_ugv_id.compare("ugv2") == 0)
				{
					ugv2.est_pose_msg.header.seq += 1;
					ugv2.est_pose_msg.header.stamp = ros::Time::now();
					ugv2.est_pose_msg.position.x = fullSLAM.augmentedState[4*(ugvN_id)+0];
					ugv2.est_pose_msg.position.y = fullSLAM.augmentedState[4*(ugvN_id)+1];
					ugv2.est_pose_msg.position.z = fullSLAM.augmentedState[4*(ugvN_id)+2];
					ugv2.est_pose_msg.yaw				 = fullSLAM.augmentedState[4*(ugvN_id)+3];
					// ugv2.est_state_pub.publish(ugv2.est_pose_msg);

					ugv2.EstPosition_gl = (cv::Mat_<double>(3, 1) <<
						fullSLAM.augmentedState[4*(ugvN_id)+0],
						fullSLAM.augmentedState[4*(ugvN_id)+1],
						fullSLAM.augmentedState[4*(ugvN_id)+2]);
					// ugv2.EstPhi_gl = wrapRadians(fullSLAM.augmentedState[4*(ugvN_id)+3]);
					ugv2.EstPhi_gl = (fullSLAM.augmentedState[4*(ugvN_id)+3]);

					fprintf(fullfile.filename,"  %s.%s.est.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", 		s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.slamCount, ugv2.EstPosition_gl.at<double>(0,0), ugv2.EstPosition_gl.at<double>(1,0),  ugv2.EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.%s.est.yaw.global(%u,:) = % -16.14f;\n", 													s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.slamCount, ugv2.EstPhi_gl);

					fullSLAM.ugv2count+=1;
					fprintf(fullfile.filename,"  %s.%s.uav_est.time(%u,:) = % -16.14f;\n", 														s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv2count, ugv2.est_pose_msg.header.stamp.toSec()-init_time);
					fprintf(fullfile.filename,"  %s.%s.uav_est.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv2count, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.%s.uav_est.yaw.global(%u,:) = % -16.14f;\n", 											s_full_field.c_str(), s_ugv_id.c_str(), fullSLAM.ugv2count, uav.EstPhi_gl);
					fprintf(fullfile.filename,"  %s.slam.postP.ugv2_time(%u,1) = % -16.14f;\n", 												s_full_field.c_str(), fullSLAM.ugv2count, ugv2.est_pose_msg.header.stamp.toSec()-init_time);
					// mfile.cellCvMatDoubles_multline(postP, "  " + s_matlab_field + ".slam.postP.ugv2", fullSLAM.ugv2count);

				}

				uav.odom_delta  = (cv::Mat_<double>(4, 1) << 0,0,0, 0);
				uav.Qdk = 0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

			} //end if(slamSwitch)
		}// end ugvUpdate(const hast::ugvmux::ConstPtr& mux_msg)

		void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags)
		{
			// run update every time there is a new tag detection:
			// mfile.writeString("%% --------------- void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags)  --------------- \n\n" );
			tagArray = tags->detections;
			tagArraySize = tagArray.size();
			fullSLAM.NumberOfTagsinFOV = 0;
			fullSLAM.TagsInFOV.assign(tag_max_id, 0);

			double tagTime = ros::Time::now().toSec();

			// if(slamSwitch && ugv1.slamFlag && ugv2.slamFlag)
			if(slamSwitch && ugv1.slamFlag)
			{
					tag_counter++;
					fprintf(fullfile.filename, "\n%% --------------- tag Message --------------- %% \n");
					fprintf(fullfile.filename,"  %s.april.time(%u,:) = % -6.4f;\n"																						, s_full_field.c_str(), tag_counter, tagTime-init_time);
					fprintf(fullfile.filename,"    %s.april.ArraySize(%u,:) = %i;\n"																				, s_full_field.c_str(), tag_counter, tagArraySize);
					fprintf(fullfile.filename,"    %s.april.uav.PrioriEstPhi_gl(%u,:) = % -6.4f;\n"														, s_full_field.c_str(), tag_counter, uav.EstPhi_gl);
					fprintf(fullfile.filename,"    %s.april.uav.PrioriEstPosition_gl(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n"			, s_full_field.c_str(), tag_counter,
																					uav.EstPosition_gl.at<double>(0,0),
																					uav.EstPosition_gl.at<double>(1,0),
																					uav.EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"    %s.april.uav.PrioriAugmentedState(%u,:) = [% -6.6f, % -6.6f, % -6.6f, % -6.6f];\n", s_full_field.c_str(), tag_counter,
																					fullSLAM.augmentedState[0],
																					fullSLAM.augmentedState[1],
																					fullSLAM.augmentedState[2],
																					fullSLAM.augmentedState[3]);

				if (tagArray.size()>0)
				{
					uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
					uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
					uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);

					uav.Htag2gl = (cv::Mat_<double>(4, 4) <<
							   uav.cosEstPhi_gl, -uav.sinEstPhi_gl, 0, 0,
							   uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0, 0,
								 0, 0, 1, 0,
							   0, 0, 0, 1); // used to roatet relative measurement of tag from uav to global frame

					uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
					uav.Hlo2gl = invertH(uav.Hgl2lo);
				}//end if (tagArray.size()>0)

				std::vector<int> TagsInFOV;
				int id;
				for(int k = 0; k != tagArraySize; k++)
				{
					// fprintf(mfile.filename, "  %% --------------- tagDetections --------------- %% \n");
					// set marker IDs
					id = tagArray[k].id;
					printlength = sprintf(markerChannel, "/april_marker_%u", id);
					s_markerid = "tag_" + patch::to_string(id);

					// marker time stamp
					tagStamp = tagArray[k].pose.header.stamp;
					tagTSpublished = tagStamp.toSec();

					// Camera frame measurement of tags
					if(absolute_measurements){
						/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
							tf::StampedTransform StampedXform_viconuav2tag;
							std::string s_tagID, s_camPath;
							s_camPath = "/vicon/uav/base_bottomcam";
								try {
									s_tagID = "/id" + patch::to_string(id) + "_16h5/base_footprint";
									listener.lookupTransform(s_camPath, s_tagID, ros::Time(0),
											StampedXform_viconuav2tag);
								}
								catch (tf::TransformException ex) {ROS_INFO("tagDetections: tf %s to %s is missing", s_camPath.c_str(), s_tagID.c_str());}
								april[id].MeasPosition_cam = (cv::Mat_<double>(4, 1) <<
										StampedXform_viconuav2tag.getOrigin().x(),
										StampedXform_viconuav2tag.getOrigin().y(),
										StampedXform_viconuav2tag.getOrigin().z(), 1);
								april[id].MeasQuaternion_cam = (cv::Mat_<double>(4, 1) <<
										StampedXform_viconuav2tag.getRotation().x(),
										StampedXform_viconuav2tag.getRotation().y(),
										StampedXform_viconuav2tag.getRotation().z(),
										StampedXform_viconuav2tag.getRotation().w());
								// fprintf(mfile.filename,"    %s.april.%s.obs.MeasPosition_cam(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_matlab_field.c_str(), s_markerid.c_str(), april[id].measCount+1,
								// 		tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z);
								// fprintf(mfile.filename,"    %s.april.%s.vic.MeasPosition_cam(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_matlab_field.c_str(), s_markerid.c_str(), april[id].measCount+1,
								// 		StampedXform_viconuav2tag.getOrigin().x(), StampedXform_viconuav2tag.getOrigin().y(), StampedXform_viconuav2tag.getOrigin().z());
								// fprintf(mfile.filename,"    %s.april.%s.obs.MeasQuaternion_cam(%u,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n", s_matlab_field.c_str(), s_markerid.c_str(), april[id].measCount+1,
								// 		tagArray[k].pose.pose.orientation.x,  tagArray[k].pose.pose.orientation.y, tagArray[k].pose.pose.orientation.z, tagArray[k].pose.pose.orientation.w);
								// fprintf(mfile.filename,"    %s.april.%s.vic.MeasQuaternion_cam(%u,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n", s_matlab_field.c_str(), s_markerid.c_str(), april[id].measCount+1,
								// 		StampedXform_viconuav2tag.getRotation().x(), StampedXform_viconuav2tag.getRotation().y(), StampedXform_viconuav2tag.getRotation().z(), StampedXform_viconuav2tag.getRotation().w());

						/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
					} else {
						// use actual measurements
						april[id].MeasPosition_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z, 1);
						april[id].MeasQuaternion_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.orientation.x,  tagArray[k].pose.pose.orientation.y, tagArray[k].pose.pose.orientation.z, tagArray[k].pose.pose.orientation.w);
					}

					// Games using TF
					double qx,qy,qz,qw, qroll, qpitch;
					tf::StampedTransform StampedXform_uav2cam, StampedXform_april2map;

					tf::Vector3 t_cam2tag(april[id].MeasPosition_cam.at<double>(0,0), april[id].MeasPosition_cam.at<double>(1,0), april[id].MeasPosition_cam.at<double>(2,0));
					tf::Quaternion q_cam2tag(april[id].MeasQuaternion_cam.at<double>(0,0), april[id].MeasQuaternion_cam.at<double>(1,0), april[id].MeasQuaternion_cam.at<double>(2,0), april[id].MeasQuaternion_cam.at<double>(3,0));
					tf::Transform Xform_cam2tag(q_cam2tag, t_cam2tag);
					tf::Matrix3x3(Xform_cam2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_cam_rad);
					// april[id].MeasYaw_cam = toDegrees(april[id].MeasYaw_cam_rad);
					april[id].MeasYaw_cam = april[id].MeasYaw_cam_rad; //stay in radians where possible

					try {listener.lookupTransform("/hast/uav/ardrone_base_link", "/hast/uav/base_bottomcam", ros::Time(0), StampedXform_uav2cam);}
						catch (tf::TransformException ex) {ROS_INFO("tagDetections: /hast/uav/base_bottomcam is missing");}

					tf::Quaternion q_uav2cam(StampedXform_uav2cam.getRotation().x(), StampedXform_uav2cam.getRotation().y(), StampedXform_uav2cam.getRotation().z(), StampedXform_uav2cam.getRotation().w());
					tf::Vector3 t_uav2cam(StampedXform_uav2cam.getOrigin().x(), StampedXform_uav2cam.getOrigin().y(), StampedXform_uav2cam.getOrigin().z());
					tf::Transform Xform_uav2cam(q_uav2cam, t_uav2cam);
					tf::Transform Xform_uav2tag = Xform_uav2cam*Xform_cam2tag;

					qx = Xform_uav2tag.getRotation().x(); qy = Xform_uav2tag.getRotation().y(); qz = Xform_uav2tag.getRotation().z(); qw = Xform_uav2tag.getRotation().w();
					tf::Matrix3x3(Xform_uav2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_uav_rad);

					april[id].measCount++;
					april[id].inFOV = true;
					april[id].MeasPosition_uav = (cv::Mat_<double>(4, 1) << Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z(), 1);
					// april[id].MeasPosition_gl = uav.Hlo2gl * april[id].MeasPosition_uav;
					april[id].MeasRelUAV_gl = - uav.Htag2gl * april[id].MeasPosition_uav; //used for fullSLAM measurement of tag
					april[id].MeasYaw_uav = april[id].MeasYaw_uav_rad; //stay in radians where possible
					april[id].MeasYaw_gl = april[id].MeasYaw_uav + fullSLAM.augmentedState[3]; //stay in radians where possible

					fullSLAM.TagsInFOV[fullSLAM.NumberOfTagsinFOV] = april[id].id;
					TagsInFOV.push_back(id);
					//update the number count of tags in FOV
					fullSLAM.NumberOfTagsinFOV++;
				} // end for(int k = 0; k != tagArraySize; k++)


				// ROS_INFO("Number of tags in FOV %u", fullSLAM.NumberOfTagsinFOV);
				if (fullSLAM.NumberOfTagsinFOV > 0)
				// if (0)
				{ // only run slam if there is a new measurement of tags & the UAVv has been observed at least once
					fullfile.cellVecInts(TagsInFOV,          "    " + s_full_field + ".april.TagsInFOV", fullSLAM.slamCount);
					MarkerSLAM(tagTime);
					// update the map every time there is at least one tag in the FOV
					// publishAprilObstacles();
				} //end if (fullSLAM.NumberOfTagsinFOV > 0)

				// reset inFOV for all markers
				for(uint i = 0; i != tag_max_id; i++){april[i].inFOV = false;}

				// Both of these should be zero from initial augmentation, or should be zeroed at the end of an iteration
				// for (uint i = 0; i < slam.predictedMeasurementVector.size(); ++i)
				// {
				// 	slam.predictedMeasurementVector[i]=0;
				// 	slam.augmentedMeasurementVector[i]=0;
				// }
			} //end if(slamSwitch)
		} // end void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags)

		void MarkerSLAM(double scantime)
		{// mfile.writeString("%% --------------- void MarkerSLAM(double scantime) --------------- \n\n" );
			fullSLAM.firstTimeForID.assign(tag_max_id, 0); //slam.firstTimeForID.assign(tag_max_id, 0);
			fullSLAM.updateForID.assign(tag_max_id, 0); //slam.updateForID.assign(tag_max_id, 0); // reset updating vector to zeros
			int updatingTags = 0; // number of tags that are updating measurements

			std::vector<int> updatingIDs, newIDs;
			int newTags = 0; // number of first time observations
			uint ID; // id of april tag under consideration
			for(uint i = 0; i != fullSLAM.NumberOfTagsinFOV; i++)
			{
				ID = april[fullSLAM.TagsInFOV[i]].id;
				if (april[fullSLAM.TagsInFOV[i]].measCount == (1+fullSLAM.downimagecountup)) // first, check to see if this is the first estimate of the marker:
				{
					// ROS_INFO("MarkerSLAM::fullSLAM.firstTimeForID[%i] = %i;", newTags, ID);
					fullSLAM.firstTimeForID[newTags]=ID; fullSLAM.firstTimeForID[newTags]=ID;
					newIDs.push_back(ID);
					newTags++;
					// ROS_INFO("MarkerSLAM::newTags [%u]", newTags);
				} else {
					if (april[fullSLAM.TagsInFOV[i]].measCount < (1+fullSLAM.downimagecountup))
					{
						// Do nothing
					} else { // measurement >= (1+fullSLAM.downimagecountup), start updating
						fullSLAM.updateForID[updatingTags] = ID; fullSLAM.updateForID[updatingTags] = ID;
						updatingIDs.push_back(ID);
						april[fullSLAM.TagsInFOV[i]].updateOrder = updatingTags;
						updatingTags++; // number of tags getting updated
					}
				}
			} // end for(uint i = 0; i != fullSLAM.NumberOfTagsinFOV; i++)

			fullfile.cellVecUints(fullSLAM.TagsInFOV,      "    " + s_full_field + ".april.InFOV"						, fullSLAM.slamCount);
			fullfile.cellVecInts(fullSLAM.firstTimeForID,  "    " + s_full_field + ".april.firstTimeForID"	, fullSLAM.slamCount);
			fullfile.cellVecInts(fullSLAM.updateForID,     "    " + s_full_field + ".april.updateForID"			, fullSLAM.slamCount);
			fullfile.cellVecInts(fullSLAM.landmarksOrdered,"    " + s_full_field + ".april.landmarksOrdered", fullSLAM.slamCount);
			fullfile.cellVecInts(updatingIDs,          "    " + s_full_field + ".april.updatingIDs"			, fullSLAM.slamCount);

			/* ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~*/
			// ROS_INFO("if (updatingTags>0), %d>0", updatingTags);
			if (updatingTags>0)
			{// now that we know there is an update for at least one april tag:
				uint ID, Ord; //indices for populating matrices

				// fprintf(mfile.filename, "%% --------------- updatingTags --------------- %% \n");
				// slam.slamCount++;
				// fprintf(mfile.filename,"  %s.time(%u,:) = % -6.14f;\n", 						s_matlab_field.c_str(), slam.slamCount, scantime);
				// fprintf(mfile.filename,"    %s.newTags(%u,:) = %i;\n", 							s_matlab_field.c_str(), slam.slamCount, newTags);
				// fprintf(mfile.filename,"    %s.updatingTags(%u,:) = %i;\n", 				s_matlab_field.c_str(), slam.slamCount, updatingTags);
				// fprintf(mfile.filename,"    %s.numberOfTagsObserved(%u,1) = %i;\n", s_matlab_field.c_str(), slam.slamCount, int(slam.landmarksOrdered.size()));

				fprintf(fullfile.filename, "%% --------------- updatingTags --------------- %% \n");
				fullSLAM.slamCount++;
				fprintf(fullfile.filename,"  %s.time(%u,:) = % -6.14f;\n", 										s_full_field.c_str(), fullSLAM.slamCount, scantime-init_time);
				fprintf(fullfile.filename,"    %s.scantime(%u,:) = % -6.14f;\n", 							s_full_field.c_str(), fullSLAM.slamCount, scantime);
				fprintf(fullfile.filename,"    %s.april.newTags(%u,:) = %i;\n", 							s_full_field.c_str(), fullSLAM.slamCount, newTags);
				fprintf(fullfile.filename,"    %s.april.updatingTags(%u,:) = %i;\n", 					s_full_field.c_str(), fullSLAM.slamCount, updatingTags);
				fprintf(fullfile.filename,"    %s.april.numberOfTagsObserved(%u,1) = %i;\n",	s_full_field.c_str(), fullSLAM.slamCount, int(fullSLAM.landmarksOrdered.size()));

				uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
				uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
				uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
				uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
				uav.Hlo2gl = invertH(uav.Hgl2lo);

				// mfile.cellCvMatDoubles(uav.EstPosition_gl,	"    " + s_matlab_field + ".april.slam.uav.EstPosition_gl",	slam.slamCount);
				// mfile.writeCvMatDoubles(uav.Hgl2lo,					"    " + s_matlab_field + ".april.slam.uav.Hgl2lo", 				slam.slamCount);
				// mfile.writeCvMatDoubles(uav.Hlo2gl,					"    " + s_matlab_field + ".april.slam.uav.Hlo2gl", 				slam.slamCount);
				// mfile.cellCvMatDoubles_multline(slam.augmentedCovariance, 			"  " + s_matlab_field + ".slam.prioriCovariance", slam.slamCount);

				// fullfile.cellCvMatDoubles(uav.EstPosition_gl,									"  " + s_full_field + ".april.slam.uav.EstPosition_gl",	fullSLAM.slamCount);
				// fullfile.writeCvMatDoubles(uav.Hgl2lo,												"  " + s_full_field + ".april.slam.uav.Hgl2lo", 				fullSLAM.slamCount);
				// fullfile.writeCvMatDoubles(uav.Hlo2gl,												"  " + s_full_field + ".april.slam.uav.Hlo2gl", 				fullSLAM.slamCount);
				// fullfile.cellCvMatDoubles_multline(slam.augmentedCovariance, 	"  " + s_full_field + ".slam.prioriCovariance", 				fullSLAM.slamCount);

				// cv::Mat SLAM_H = cv::Mat::zeros((updatingTags)*4, (updatingTags+1)*4, CV_64F);
				fullSLAM.H = cv::Mat::zeros((updatingTags)*4, fullSLAM.augmentedState.size(), CV_64F);
				// ROS_INFO("fullSLAM.H = cv::Mat::zeros((updatingTags)*4, fullSLAM.augmentedState.size(), CV_64F); ::~:: (%d, %d)", fullSLAM.H.rows, fullSLAM.H.cols);
				fullSLAM.zk = cv::Mat::zeros(updatingTags*4, 1, CV_64F);
				// ROS_INFO("fullSLAM.zk = cv::Mat::zeros(updatingTags*4, 1, CV_64F); ::~:: (%d, %d)", fullSLAM.zk.rows, fullSLAM.zk.cols);
				fullSLAM.Rk = cv::Mat::zeros(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a identity matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
				// ROS_INFO("fullSLAM.Rk = cv::Mat::zeros(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); ::~:: (%d, %d)", fullSLAM.Rk.rows, fullSLAM.Rk.cols);

				cv::Mat SLAM_Rk = cv::Mat::eye(updatingTags*4, updatingTags*4, CV_64F);

				//Update covariance of UAV from motion:
					uav.odom_J  = (cv::Mat_<double>(4, 4) << 1,0,0,-uav.odom_delta.at<double>(1,0), 0,1,0,uav.odom_delta.at<double>(0,0), 0,0,1,0, 0,0,0,1);
					uav.Qdk = uav.odom_J * uav.Qw * uav.odom_J.t();
					// cv::Mat aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4);  // uav current covariance
					// aux_C = uav.odom_J * slam.augmentedCovariance.colRange(0,4).rowRange(0,4) * uav.odom_J.t() + uav.Qdk;
					// mfile.cellCvMatDoubles_multline(aux_C, "  " + s_matlab_field + ".uav.aux_C", slam.slamCount);
					// aux_C.release();

				/* ~~~~~ update fullSLAM covariance matrix for uav and ugv */
					cv::Mat Jf = cv::Mat::eye(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a identity matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
						// add J_uav
							uav.odom_J.copyTo(Jf.colRange(0,4).rowRange(0,4));
							// fullfile.cellCvMatDoubles_multline(Jf, "  " + s_full_field + ".slam.Jf", fullSLAM.slamCount);

					cv::Mat Qdk = cv::Mat::zeros(fullSLAM.augmentedCovariance.rows, fullSLAM.augmentedCovariance.cols, CV_64F); // this should be a identity matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
							uav.Qdk = uav.odom_J * uav.Qw * uav.odom_J.t();
							uav.Qdk.copyTo(Qdk.colRange(0,4).rowRange(0,4));
							// fullfile.cellCvMatDoubles_multline(Qdk, "  " + s_full_field + ".slam.Qdk", fullSLAM.slamCount);

					fullSLAM.Pf = Jf * fullSLAM.augmentedCovariance * Jf.t() + Qdk;
						Qdk.release();
						Jf.release();

					fullfile.cellCvMatDoubles_multline(fullSLAM.Pf, "  " + s_full_field + ".slam.Pf", fullSLAM.slamCount);

				// Now that H and Rk are defined, we can populate them
				// fprintf(mfile.filename,"%%slam.uav.EstPhi_gl = % -6.4f;\n", uav.EstPhi_gl);


				for(uint i = 0; i != updatingTags; i++)
				{
					april[fullSLAM.updateForID[i]].slamCount++;
					ID = april[fullSLAM.updateForID[i]].id; // the numeric ID of the observed tag
					Ord = april[fullSLAM.updateForID[i]].order; // serial order of observation (1 indexed)
					fullSLAM.I4.copyTo(fullSLAM.H.rowRange(4*i, 4*(i+1)).colRange(0,4)); // does not need to be offset by UGV states
					fullSLAM.nI4.copyTo(fullSLAM.H.rowRange(4*i, 4*(i+1)).colRange(4*(Ord-1+numUGV+numUAV),4*(Ord+numUGV+numUAV)));

					april[ fullSLAM.updateForID[i] ].MeasRelUAV_gl.rowRange(0,3).colRange(0,1).copyTo(fullSLAM.zk.rowRange(4*i, 4*(i+1)-1).colRange(0,1));

					// write measured yaw value to zk vector
					fullSLAM.zk.at<double>(4*(i+1)-1, 0) = - april[ fullSLAM.updateForID[i] ].MeasYaw_uav;

					// check to see if measurmeents are near the +-pi wrapping
					// double uav_phi_Hx = uav_phi - april_phi
					double uav_phi_zk_Hx = (- april[ fullSLAM.updateForID[i] ].MeasYaw_uav) - (fullSLAM.augmentedState[3] - fullSLAM.augmentedState[4*(Ord+numUGV+numUAV)-1]);
					if (uav_phi_zk_Hx > 6.0) {fullSLAM.zk.at<double>(4*(i+1)-1, 0) -= (2*Pi);}
					if (uav_phi_zk_Hx < -6.0) {fullSLAM.zk.at<double>(4*(i+1)-1, 0) += (2*Pi);}

					// Rk is constant for all tag measurements, but must be rotated to the global/map frame eventually
					april[fullSLAM.updateForID[i]].Rk_uav.copyTo(SLAM_Rk.colRange(i*4, (i+1)*4).rowRange(i*4, (i+1)*4));
					april[fullSLAM.updateForID[i]].Rk_uav.copyTo(fullSLAM.Rk.colRange(i*4, (i+1)*4).rowRange(i*4, (i+1)*4));
					s_markerid = "tag_" + patch::to_string(ID);

					fprintf(fullfile.filename,"  %s.april.%s.slam.time(%u,1) = % -6.14f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, scantime - init_time);
					fprintf(fullfile.filename,"    %s.april.%s.slam.MeasYaw_uav(%u,1)      = % -6.14f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].MeasYaw_uav);
					fprintf(fullfile.filename,"    %s.april.%s.slam.PredictedPhi_uav(%u,1) = % -6.14f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, fullSLAM.augmentedState[3] - april[ID].EstYaw_gl);
					fprintf(fullfile.filename,"    %s.april.%s.slam.MeasYaw_gl(%u,1)       = % -6.14f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].MeasYaw_gl);
					fprintf(fullfile.filename,"    %s.april.%s.slam.MeasRelUAV_gl(%u,:)    = [% -6.8f, % -6.8f, % -6.8f];\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].MeasRelUAV_gl.at<double>(0,0), april[ID].MeasRelUAV_gl.at<double>(1,0), april[ID].MeasRelUAV_gl.at<double>(2,0));
					fprintf(fullfile.filename,"    %s.april.%s.slam.MeasPosition_uav(%u,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].MeasPosition_uav.at<double>(0,0), april[ID].MeasPosition_uav.at<double>(1,0), april[ID].MeasPosition_uav.at<double>(2,0));
					// fprintf(fullfile.filename,"    %s.april.%s.slam.MeasPosition_gl(%u,:)      = [% -6.8f, % -6.8f, % -6.8f];\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].MeasPosition_gl.at<double>(0,0), april[ID].MeasPosition_gl.at<double>(1,0), april[ID].MeasPosition_gl.at<double>(2,0));
				}  // end for(uint i = 0; i != updatingTags; i++)



				// update fullSLAM.EKF equations
					fullSLAM.PHt = fullSLAM.Pf * fullSLAM.H.t();
					fullSLAM.HPHt = fullSLAM.H * fullSLAM.PHt;
					fullSLAM.Sk = fullSLAM.HPHt + SLAM_Rk;
					fullSLAM.Sk_inv = fullSLAM.Sk.inv();
					fullSLAM.Kk = fullSLAM.PHt * fullSLAM.Sk_inv;
					fullSLAM.KH = fullSLAM.Kk * fullSLAM.H;
					fullSLAM.x = cv::Mat(fullSLAM.augmentedState);
					fullSLAM.Hx = fullSLAM.H*fullSLAM.x;
					fullSLAM.correction = fullSLAM.Kk * (fullSLAM.zk - fullSLAM.Hx);
					fullSLAM.KHI = cv::Mat::eye(fullSLAM.KH.rows, fullSLAM.KH.cols, CV_64F); //64f == double
					fullSLAM.POST_P = (fullSLAM.KHI - fullSLAM.Kk*fullSLAM.H) * fullSLAM.Pf;
					fullSLAM.POST_P.copyTo(fullSLAM.augmentedCovariance);

					fullfile.cellCvMatDoubles_multline(fullSLAM.H, 					"  " + s_full_field + ".slam.H", 					fullSLAM.slamCount);
					fullfile.cellMatAugmentedState_nx4(fullSLAM.x, 					"  " + s_full_field + ".slam.x", 					fullSLAM.slamCount);
					fullfile.cellVecAugmentedState_nx4(fullSLAM.zk,					"  " + s_full_field + ".slam.zk", 				fullSLAM.slamCount);
					fullfile.cellMatAugmentedState_nx4(fullSLAM.Hx, 				"  " + s_full_field + ".slam.Hx", 					fullSLAM.slamCount);

					// update states
					for(uint state = 0; state != fullSLAM.augmentedState.size(); state++){fullSLAM.augmentedState[state]+=fullSLAM.correction.at<double>(state,0);}
					fullfile.cellMatAugmentedState_nx4(fullSLAM.correction, "  " + s_full_field + ".slam.correction", fullSLAM.slamCount);
					fullfile.cellVecAugmentedState_nx4(fullSLAM.augmentedState,	"  " + s_full_field + ".slam.augmentedState", fullSLAM.slamCount);
					fullfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance, 		"  " + s_full_field + ".augmentedCovariance", 		fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.Rk,					"  " + s_full_field + ".slam.Rk", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.PHt,				"  " + s_full_field + ".slam.PHt", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.HPHt,				"  " + s_full_field + ".slam.HPHt", 			fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.Sk,					"  " + s_full_field + ".slam.Sk", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.Sk_inv,			"  " + s_full_field + ".slam.Sk_inv", 		fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.Kk, 				"  " + s_full_field + ".slam.Kk", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.KH, 				"  " + s_full_field + ".slam.KH", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.KHI, 				"  " + s_full_field + ".slam.KHI", 				fullSLAM.slamCount);
					// fullfile.cellCvMatDoubles_multline(fullSLAM.POST_P, 		"  " + s_full_field + ".slam.POST_P", 		fullSLAM.slamCount);

				// update uav pose
					uavPublishEst();
					publishSLAM(ros::Time::now(), true, 0);

				slam.tagcount+=1;
					fprintf(fullfile.filename,"  %s.april.uav_est.time(%u,1) = % -16.14f;\n", s_full_field.c_str(), slam.tagcount, uav.est_pose_msg.header.stamp.toSec()-init_time);
					fprintf(fullfile.filename,"  %s.april.uav_est.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", s_full_field.c_str(), slam.tagcount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.april.uav_est.yaw.global(%u,:) = % -16.14f;\n", s_full_field.c_str(), slam.tagcount, uav.EstPhi_gl);
					// fprintf(mfile.filename,"  %s.slam.postP.april_time(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.tagcount, uav.est_pose_msg.header.stamp.toSec());
				// 	mfile.cellCvMatDoubles_multline(slam.POST_P, "  " + s_matlab_field + ".slam.postP.april", slam.tagcount);

				//update tag poses
				ros::Time april_time = ros::Time::now();
				for(uint i = 0; i != updatingTags; i++)
				{
					uint ID = april[fullSLAM.updateForID[i]].id; // the numeric ID of the observed tag
					s_markerid = "tag_" + patch::to_string(ID);
					uint Ord = april[fullSLAM.updateForID[i]].order; // serial order of observation (zero indexed?)
					cv::Mat tag_correction_gl = (cv::Mat_<double>(3, 1) <<
						fullSLAM.correction.at<double>(4*(numUGV+Ord)+0,0),
						fullSLAM.correction.at<double>(4*(numUGV+Ord)+1,0),
						fullSLAM.correction.at<double>(4*(numUGV+Ord)+2,0));

					april[ID].EstPosition_gl = (cv::Mat_<double>(3, 1) <<
						fullSLAM.augmentedState[4*(numUGV+Ord)+0],
						fullSLAM.augmentedState[4*(numUGV+Ord)+1],
						fullSLAM.augmentedState[4*(numUGV+Ord)+2]);
					april[ID].EstYaw_gl = fullSLAM.augmentedState[4*(numUGV+Ord)+3];

					april[ID].publishStateTF(april_time);

					fprintf(fullfile.filename,"  %s.april.%s.slam.time(%u,:) = % -6.4f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april_time.toSec()-init_time);
					fprintf(fullfile.filename,"  %s.april.%s.slam.correctionP_gl(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount,
								tag_correction_gl.at<double>(0,0),
								tag_correction_gl.at<double>(1,0),
								tag_correction_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.april.%s.slam.correctionYaw(%u,:) = % -6.4f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, fullSLAM.correction.at<double>(4*(numUGV+Ord)+3,0));
					fprintf(fullfile.filename,"  %s.april.%s.slam.EstPosition_gl(%u,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount,
								april[ID].EstPosition_gl.at<double>(0,0),
								april[ID].EstPosition_gl.at<double>(1,0),
								april[ID].EstPosition_gl.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.april.%s.slam.EstPhi_gl(%u,1) = % -6.4f;\n", s_full_field.c_str(), s_markerid.c_str(), april[ID].slamCount, april[ID].EstYaw_gl);
				}

			} //end if (updatingTags>0)

			// If there are new tags, the whole system must be augmented:
			if (newTags>0)
			{
				cv::Mat newLandmarkPos, obsRk, newLandmarkMeas;
				double newLandmarkYaw;
				int newLandmarkID;
				// ROS_INFO("for(uint i = 0; i != newTags; i++) ");

				for(uint i = 0; i != newTags; i++)
				{
					newLandmarkID = april[fullSLAM.firstTimeForID[i]].id;
					newLandmarkMeas = april[fullSLAM.firstTimeForID[i]].getMeasPosition_uav();
					newLandmarkPos = uav.Hlo2gl * april[fullSLAM.firstTimeForID[i]].getMeasPosition_uav();  // rotate to global
					// newLandmarkYaw = wrapRadians(april[fullSLAM.firstTimeForID[i]].MeasYaw_uav + uav.EstPhi_gl);
					newLandmarkYaw = (april[fullSLAM.firstTimeForID[i]].MeasYaw_uav + uav.EstPhi_gl);
					obsRk = april[fullSLAM.firstTimeForID[i]].Rk_uav;
					augmentSLAM(newLandmarkID, newLandmarkPos, newLandmarkYaw, obsRk);
					april[fullSLAM.firstTimeForID[i]].order = fullSLAM.numberOfLandmarksOrdered; // should not start at zero
					// april[fullSLAM.firstTimeForID[i]].EstPosition_gl = newLandmarkPos;
					newLandmarkPos.copyTo(april[fullSLAM.firstTimeForID[i]].EstPosition_gl);
					april[fullSLAM.firstTimeForID[i]].EstYaw_gl = newLandmarkYaw;
					s_markerid = "tag_" + patch::to_string(newLandmarkID);
					fprintf(fullfile.filename,"  %s.april.%s.order = %i;\n", s_full_field.c_str(), s_markerid.c_str(), fullSLAM.numberOfLandmarksOrdered);
					fprintf(fullfile.filename,"  %s.april.%s.init.EstPosition_gl(1,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_full_field.c_str(), s_markerid.c_str(),
						newLandmarkPos.at<double>(0,0),
						newLandmarkPos.at<double>(1,0),
						newLandmarkPos.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.april.%s.init.EstYaw_gl(1,:) = % -6.4f;\n", s_full_field.c_str(), s_markerid.c_str(), newLandmarkYaw);
					fprintf(fullfile.filename,"  %s.april.%s.init.MeasPosition_uav(1,:) = [% -6.4f, % -6.4f, % -6.4f];\n", s_full_field.c_str(), s_markerid.c_str(),
						newLandmarkMeas.at<double>(0,0),
						newLandmarkMeas.at<double>(1,0),
						newLandmarkMeas.at<double>(2,0));
					fprintf(fullfile.filename,"  %s.april.%s.init.MeasYaw_uav(1,:) = % -6.4f;\n", s_full_field.c_str(), s_markerid.c_str(), april[fullSLAM.firstTimeForID[i]].MeasYaw_uav);
					std::string s_obsRK = "  " + s_full_field + ".april." + s_markerid + ".init.obsRk";
					fullfile.writeCvMatDoubles(obsRk, s_obsRK, 1);
					// mfile.writeCvMatDoubles(uav.Hlo2gl, "  fullSLAM.april." + s_markerid + ".init.Hlo2gl", slam.slamCount);

					april[fullSLAM.firstTimeForID[i]].publishStateTF(ros::Time::now());

				} // end for(uint i = 0; i != newTags; i++)

				// update covariance based on co-visible tags:
				// mfile.cellCvMatDoubles_multline_ndec(slam.augmentedCovariance,	"  " + s_matlab_field + ".slam.augmentedCovariance_ndec", slam.slamCount);
				// fprintf(mfile.filename,"  %% updateNewTagRelations(updatingIDs, newIDs, obsRk);\n");
				updateNewTagRelations(updatingIDs, newIDs, obsRk);
			} //end if (newTags>0)

			updateAprilCloud();

			// reset odom based
			uav.odom_delta  = (cv::Mat_<double>(4, 1) << 0,0,0, 0);
			uav.Qdk = 0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
		}

		void updateAprilCloud()
		{
			// clear point cloud
			// ROS_WARN("clear costmaps");
			// ugv1_clearCostmaps_cli.call(nullcall);
			// ugv2_clearCostmaps_cli.call(nullcall);

			aprilCloud_global_ptr->points.clear();
			// goalCloudXYZ_ptr->points.clear();
			pcl::PointXYZ point_XYZ; // pcl point for center of obstacle
			for(uint i = 0; i != fullSLAM.numberOfLandmarksOrdered; i++)
			{
				// update point clouds
				// ROS_INFO("SLAM:: adding april_point[%i] to cloud", fullSLAM.landmarksOrdered[i]);
				point_XYZ.x = fullSLAM.augmentedState[4*(numUGV+i+1)+0];
				point_XYZ.y = fullSLAM.augmentedState[4*(numUGV+i+1)+1];
				// point_XYZ.z = fullSLAM.augmentedState[4*(numUGV+i+1)+2];
				point_XYZ.z = 0;
				// aprilCloud_global_ptr->points.push_back(point_XYZ);
				if (fullSLAM.landmarksOrdered[i] != ugv1.goal_id && fullSLAM.landmarksOrdered[i] != ugv2.goal_id )
				{// if not goal, update obstacle cloud
					aprilCloud_global_ptr->points.push_back(point_XYZ);
				} else {
					// publish the goal location
					// goalCloudXYZ_ptr->points.push_back(point_XYZ);
				}
			}

			pcl::toROSMsg(*aprilCloud_global_ptr, aprilObstacleCloud_msg);
			aprilObstacleCloud_msg.header.frame_id = s_april_cloud_frame;
			aprilObstacleCloud_pub.publish(aprilObstacleCloud_msg);

			// pcl::toROSMsg(*goalCloudXYZ_ptr, goalCloud2_msg);
			// goalCloud2_msg.header.frame_id = "/map";
			// goalCloud2_pub.publish(goalCloud2_msg);

		}

		void updateNewTagRelations(std::vector<int> updatingIDs, std::vector<int> newIDs, cv::Mat Rj)
		{// this will update the covariances between the newly observed tag and the previously seen tags in the same image
			cv::Mat Cov_tag_tag;
			uint ID, Ord, IDx, Ordx; //indices for populating matrices


			for(uint idx = 0; idx != newIDs.size(); idx++)
			{
				// fprintf(mfile.filename,"\n");
				 IDx = april[newIDs[ idx ]].id; // the numeric ID of the observed tag
				// fprintf(mfile.filename,"  %%   IDx = april[newIDs[ %02d ]].id = %02d; \n", idx, IDx);
				Ordx = april[newIDs[ idx ]].order; // serial order of observation (1 indexed)
				// fprintf(mfile.filename,"  %%     Ordx = april[newIDs[ %02d ]].order = %02d; \n", idx, Ordx);

				// set tag/uav covariance block
				// cv::Mat Cov_UAV_tag = slam.augmentedCovariance.rowRange(0,4).colRange(0,4);
				// cv::Mat Cov_UAV_tag = slam.augmentedCovariance.rowRange(0,4).colRange(0,4)+Rj;
				cv::Mat Cov_UAV_tag = 0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				// fprintf(mfile.filename,"  %%       Cov_UAV_tag.copyTo(slam.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d)));\n",
																														 // 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx), 				 0,4);
				Cov_UAV_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)).colRange(0,4));
				// fprintf(mfile.filename,"  %%       Cov_UAV_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d)));\n",
				// 																										 0,4,					 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx));
				Cov_UAV_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(0,4).colRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)));

				// set tag/tag covariance blocks for previously updated tags
				// fprintf(mfile.filename,"  %%     for(uint i = 0; i != updatingIDs.size(); i++)\n");
				for(uint i = 0; i != updatingIDs.size(); i++)
				{
					Ord = april[updatingIDs[  i ]].order; // serial order of observation (1 indexed)
					// fprintf(mfile.filename,"  %%      Ord = april[updatingIDs[ %02d ]].order = %02d; \n", i, Ord);
					// fprintf(mfile.filename,"  %%       Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d)+Rj;\n",
																															 		// 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord), 					4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord));
					// Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)).colRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord))+Rj;
					// cv::Mat Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(0,4).colRange(0,4) + uav.Rk;
					cv::Mat Cov_tag_tag = 0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
					// fprintf(mfile.filename,"  %%       Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d);\n",
					// 																										 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx), 				 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord));
					Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)).colRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)));
					// fprintf(mfile.filename,"  %%       Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d);\n",
					// 																										 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord), 				 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx));
					Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)).colRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)));
				}
				// set tag/tag covariance blocks for previously added NEW tags
				// fprintf(mfile.filename,"  %%     for(uint i = 0; i != idx; i++)\n");
				for(uint i = 0; i != idx; i++)
				{
					Ord = april[newIDs[  i ]].order; // serial order of observation (1 indexed)
					// fprintf(mfile.filename,"  %%      Ord = april[updatingIDs[ %02d ]].order = %02d; \n", i, Ord);
					// fprintf(mfile.filename,"  %%       Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d)+Rj;\n",
																															 		// 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord), 					4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord));
					// cv::Mat Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)).colRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord))+Rj;
					// cv::Mat Cov_tag_tag = fullSLAM.augmentedCovariance.rowRange(0,4).colRange(0,4) + uav.Rk;
					cv::Mat Cov_tag_tag = 0 * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
					// fprintf(mfile.filename,"  %%       Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d));\n",
					// 																										 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx), 				 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord));
					Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)).colRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)));
					// fprintf(mfile.filename,"  %%       Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d));\n",
					// 																										 4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord), 				 4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx));
					Cov_tag_tag.copyTo(fullSLAM.augmentedCovariance.rowRange(4*(numUAV+numUGV+Ord-1),4*(numUAV+numUGV+Ord)).colRange(4*(numUAV+numUGV+Ordx-1),4*(numUAV+numUGV+Ordx)));
				}
			}
		}

		void updateUAVinertial(const ardrone_autonomy::Navdata::ConstPtr& navdata)
		{
			flightState = navdata->state;
			navHeadSeq = navdata->header.seq;
			navStamp = navdata->header.stamp;
			navTSpublished = navStamp.toSec();

			if ((navTSpublished != navTS) && (slamSwitch))
			{	/*----- Inertial Update */
				++navDataCounter;
				// fprintf(mfile.filename, "%% --------------- updateUAVinertial --------------- \n\n" );
				// mfile.cellCvMatDoubles_multline(fullSLAM.augmentedCovariance, 			"    " + s_matlab_field + ".fullSLAM.prioriCovariance", fullSLAM.slamCount);

				if ( navDataCounter % 10 == 5 )
				{ // publish slam state every tenth message
					slam_stateFlag_pub.publish(slam_flag);
					// ROS_INFO("  fullSLAM.topics.s_slamState_topic = ['%s'];",  s_slam_stateFlag_topic.c_str());
					// ROS_INFO("  slamState = ['%s']", slam_flag.flag ? "true" : "false" );

				}

				// fprintf(mfile.filename,"  fullSLAM.uav.EstPhi_gl_0(%u,:) = % -6.14f;\n", navDataCounter, uav.EstPhi_gl);
				navdt = navTSpublished - navTS;
				navTS = navTSpublished;
				// suppress huge velocity estimates if motion starts from zero
				if (navdt>10){
					navdt=0.006; // 0.006 is the mean dt of an early experiment
					uav.compassYaw = toRadians(navdata->rotZ); // this just prevents computing a potentially huge angular velocity
						uav.cos_compassYaw = cos((uav.compassYaw));
						uav.sin_compassYaw = sin((uav.compassYaw));
						echoAltLast = 0.001 * double(navdata->altd);
						echoAlt = 0.001 * double(navdata->altd);
				}

				// uav.Qdk += uav.Qw*navdt;

				// update altitude measurements
				echoAltLast = echoAlt;
				echoAlt = 0.001 * double(navdata->altd);
				deltaAlt = echoAlt - echoAltLast;

				// Update heading measurements

				uav.compassYawLast = uav.compassYaw;
				uav.compassYaw = toRadians(navdata->rotZ);
					if ((uav.compassYaw - uav.compassYawLast) > 350) {uav.compassYawLast -= 360;}
					if ((uav.compassYaw - uav.compassYawLast) < -350) {uav.compassYawLast += 360;}

					uav.cos_compassYaw = cos((uav.compassYaw));
					uav.sin_compassYaw = sin((uav.compassYaw));

					uav.deltaPhi = uav.compassYaw - uav.compassYawLast;

				// Velocity update
				uav.MeasuredVel_lo = (cv::Mat_<double>(3, 1) <<
									0.001 * (navdata->vx), // not sure why these might be backwards..
									0.001 * (navdata->vy),// not sure why these might be backwards..
									deltaAlt/navdt);

			// update estimated position by half of the delta angle
				uav.cosRdeltaPhi = cos(uav.EstPhi_gl + 0.5*uav.deltaPhi);
				uav.sinRdeltaPhi = sin(uav.EstPhi_gl + 0.5*uav.deltaPhi);

				uav.RdeltaPhi_gl2lo = (cv::Mat_<double>(3, 3) <<
						   uav.cosRdeltaPhi, uav.sinRdeltaPhi, 0,
						  -uav.sinRdeltaPhi, uav.cosRdeltaPhi, 0,
						  0, 0, 1);

				cv::Mat gl_delta = uav.RdeltaPhi_gl2lo.t() * (navdt * uav.MeasuredVel_lo);
				uav.EstPosition_gl   += gl_delta;
				fullSLAM.augmentedState[0] += gl_delta.at<double>(0,0); uav.odom_delta.at<double>(0,0) += gl_delta.at<double>(0,0);
				fullSLAM.augmentedState[1] += gl_delta.at<double>(1,0); uav.odom_delta.at<double>(1,0) += gl_delta.at<double>(1,0);
				fullSLAM.augmentedState[2] += gl_delta.at<double>(2,0); uav.odom_delta.at<double>(2,0) += gl_delta.at<double>(2,0);
				fullSLAM.augmentedState[3] += uav.deltaPhi; 						uav.odom_delta.at<double>(3,0) += uav.deltaPhi;
				uav.EstPhi_gl += uav.deltaPhi;

				double odomcos  = cos(uav.odomPhi + 0.5*uav.deltaPhi);
				double odomsin  = sin(uav.odomPhi + 0.5*uav.deltaPhi);
				cv::Mat odomRgl2lo = (cv::Mat_<double>(3, 3) <<
						   odomcos, odomsin, 0,
						   -odomsin, odomcos, 0,
						   0, 0, 1);

				// fprintf(mfile.filename,"  fullSLAM.uav.navdata.EstPhi_gl(%u,:) = % -6.14f;\n", navDataCounter, uav.EstPhi_gl);
					uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
					uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
					uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
				// uav_c.EstPosition_uav = uav.Rgl2lo * uav.EstPosition_gl;

				uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
				uav.Hlo2gl = invertH(uav.Hgl2lo);

				publishUAVtf(navStamp);
				publishUAVautopilot(navTSpublished);
			}
		}

		/* ----- Publishing ----- */
		void publishUAVautopilot(double pubtime)
		{
			uav_c.state_msg.P.x = uav.EstPosition_gl.at<double>(0, 0);
			uav_c.state_msg.P.y = uav.EstPosition_gl.at<double>(1, 0);
			uav_c.state_msg.P.z = uav.EstPosition_gl.at<double>(2, 0);

			/*----- Velocity Estimate */
			uav_c.MeasuredVel_gl = uav.Rgl2lo.t() * uav.MeasuredVel_lo;

			uav_c.state_msg.V.x = uav_c.MeasuredVel_gl.at<double>(0, 0);
			uav_c.state_msg.V.y = uav_c.MeasuredVel_gl.at<double>(1, 0);
			uav_c.state_msg.V.z = uav_c.MeasuredVel_gl.at<double>(2, 0);

			/*----- Acceleration Measured */
			uav_c.state_msg.A.x = 0;
			uav_c.state_msg.A.y = 0;
			uav_c.state_msg.A.z = 0;

			/*----- Pose Estimate */
			uav_c.state_msg.R.row0.x = uav.Rgl2lo.at<double>(0,0);
			uav_c.state_msg.R.row0.y = uav.Rgl2lo.at<double>(0,1);
			uav_c.state_msg.R.row0.z = 0;
			uav_c.state_msg.R.row1.x = uav.Rgl2lo.at<double>(1,0);
			uav_c.state_msg.R.row1.y = uav.Rgl2lo.at<double>(1,1);
			uav_c.state_msg.R.row1.z = 0;
			uav_c.state_msg.R.row2.x = 0;
			uav_c.state_msg.R.row2.y = 0;
			uav_c.state_msg.R.row2.z = 1;

			/*----- Yaw Estimate */
			uav_c.state_msg.yaw = uav.EstPhi_gl;

			/*----- Time stamp of Estimate */
			uav_c.state_msg.id = ++uav_c.stateMsg_id;
			uav_c.state_msg.stamp = pubtime;
			uav_c.state_pub.publish(uav_c.state_msg);
		}

		void publishUAVtf(ros::Time pubstamp)
		{//ROS_INFO("uav void publishUAVtf(ros::Time pubstamp)");
			// mfile.writeString("%% --------------- void publishUAVtf(ros::Time pubstamp)  --------------- \n\n" );
			if (slam_flag.flag)
			{ //publish when slam has been activated
				uav.R_TF.setValue(uav.cosEstPhi_gl, -uav.sinEstPhi_gl, 0,
													uav.sinEstPhi_gl,  uav.cosEstPhi_gl, 0,
													0,0,1);
				uav.R_TF.getRotation(uav.Q_TF);

				uav.TF.setRotation(uav.Q_TF);
				uav.TF.setOrigin( tf::Vector3(uav.EstPosition_gl.at<double>(0, 0), uav.EstPosition_gl.at<double>(1, 0), uav.EstPosition_gl.at<double>(2, 0)) );
				uav_TF_pub.sendTransform(tf::StampedTransform(uav.TF, pubstamp, s_ref_TF_frame, s_uav_TF_frame));
				publishUAVpose(pubstamp);
			}
		}

		void publishSLAM(ros::Time pubstamp, bool tag_flag, uint ugv_id)
		{//ROS_INFO("UAV void posePublisher(double pubtime)");
			// std_msgs/Header header
				// uint32 seq
				// time stamp
				// string frame_id
			slam_msg.header.seq += 1;
			slam_msg.header.stamp = pubstamp;

			// bool tag_flag # bool flag whether update was ugv or uav based
			// posewithcov uav_pose		# current estimated pose of uav
			slam_msg.tag_flag = tag_flag;
			slam_msg.uav_pose.vehicle_id = "uav";
			slam_msg.uav_pose.position.x = fullSLAM.augmentedState[0];
			slam_msg.uav_pose.position.y = fullSLAM.augmentedState[1];
			slam_msg.uav_pose.position.z = fullSLAM.augmentedState[2];
			slam_msg.uav_pose.PCov.row0.x = fullSLAM.augmentedCovariance.at<double>(0,0);
			slam_msg.uav_pose.PCov.row0.y = fullSLAM.augmentedCovariance.at<double>(0,1);
			slam_msg.uav_pose.PCov.row0.z = fullSLAM.augmentedCovariance.at<double>(0,2);
			slam_msg.uav_pose.PCov.row1.x = fullSLAM.augmentedCovariance.at<double>(1,0);
			slam_msg.uav_pose.PCov.row1.y = fullSLAM.augmentedCovariance.at<double>(1,1);
			slam_msg.uav_pose.PCov.row1.z = fullSLAM.augmentedCovariance.at<double>(1,2);
			slam_msg.uav_pose.PCov.row2.x = fullSLAM.augmentedCovariance.at<double>(2,0);
			slam_msg.uav_pose.PCov.row2.y = fullSLAM.augmentedCovariance.at<double>(2,1);
			slam_msg.uav_pose.PCov.row2.z = fullSLAM.augmentedCovariance.at<double>(2,2);
			slam_msg.uav_pose.yaw = fullSLAM.augmentedState[3];
			slam_msg.uav_pose.yaw_cov = fullSLAM.augmentedCovariance.at<double>(3,3);

			// uint8 ugv_id # id of observing ugv
			slam_msg.ugv_id = ugv_id;
			ugv1.slam_pose_msg.vehicle_id = "ugv1";
			ugv1.slam_pose_msg.header.seq = slam_msg.header.seq;
			ugv1.slam_pose_msg.header.stamp = slam_msg.header.stamp;
			ugv1.slam_pose_msg.header.frame_id = slam_msg.header.frame_id;
			ugv1.slam_pose_msg.position.x  = fullSLAM.augmentedState[4];
			ugv1.slam_pose_msg.position.y  = fullSLAM.augmentedState[5];
			ugv1.slam_pose_msg.position.z  = fullSLAM.augmentedState[6];
			ugv1.slam_pose_msg.PCov.row0.x = fullSLAM.augmentedCovariance.at<double>(4,4);
			ugv1.slam_pose_msg.PCov.row0.y = fullSLAM.augmentedCovariance.at<double>(4,5);
			ugv1.slam_pose_msg.PCov.row0.z = fullSLAM.augmentedCovariance.at<double>(4,6);
			ugv1.slam_pose_msg.PCov.row1.x = fullSLAM.augmentedCovariance.at<double>(5,4);
			ugv1.slam_pose_msg.PCov.row1.y = fullSLAM.augmentedCovariance.at<double>(5,5);
			ugv1.slam_pose_msg.PCov.row1.z = fullSLAM.augmentedCovariance.at<double>(5,6);
			ugv1.slam_pose_msg.PCov.row2.x = fullSLAM.augmentedCovariance.at<double>(6,4);
			ugv1.slam_pose_msg.PCov.row2.y = fullSLAM.augmentedCovariance.at<double>(6,5);
			ugv1.slam_pose_msg.PCov.row2.z = fullSLAM.augmentedCovariance.at<double>(6,6);
			ugv1.slam_pose_msg.yaw = fullSLAM.augmentedState[7];
			ugv1.slam_pose_msg.yaw_cov = fullSLAM.augmentedCovariance.at<double>(7,7);

			ugv2.slam_pose_msg.vehicle_id = "ugv2";
			ugv2.slam_pose_msg.header.seq = slam_msg.header.seq;
			ugv2.slam_pose_msg.header.stamp = slam_msg.header.stamp;
			ugv2.slam_pose_msg.header.frame_id = slam_msg.header.frame_id;
			ugv2.slam_pose_msg.position.x  = fullSLAM.augmentedState[8];
			ugv2.slam_pose_msg.position.y  = fullSLAM.augmentedState[9];
			ugv2.slam_pose_msg.position.z  = fullSLAM.augmentedState[10];
			ugv2.slam_pose_msg.PCov.row0.x = fullSLAM.augmentedCovariance.at<double>(8,8);
			ugv2.slam_pose_msg.PCov.row0.y = fullSLAM.augmentedCovariance.at<double>(8,9);
			ugv2.slam_pose_msg.PCov.row0.z = fullSLAM.augmentedCovariance.at<double>(8,10);
			ugv2.slam_pose_msg.PCov.row1.x = fullSLAM.augmentedCovariance.at<double>(9,8);
			ugv2.slam_pose_msg.PCov.row1.y = fullSLAM.augmentedCovariance.at<double>(9,9);
			ugv2.slam_pose_msg.PCov.row1.z = fullSLAM.augmentedCovariance.at<double>(9,10);
			ugv2.slam_pose_msg.PCov.row2.x = fullSLAM.augmentedCovariance.at<double>(10,8);
			ugv2.slam_pose_msg.PCov.row2.y = fullSLAM.augmentedCovariance.at<double>(10,9);
			ugv2.slam_pose_msg.PCov.row2.z = fullSLAM.augmentedCovariance.at<double>(10,10);
			ugv2.slam_pose_msg.yaw = fullSLAM.augmentedState[11];
			ugv2.slam_pose_msg.yaw_cov = fullSLAM.augmentedCovariance.at<double>(11,11);

			// posewithcov[] ugv_poses	# current estimated poses of ugvs
			slam_msg.ugv_poses.clear();
			slam_msg.ugv_poses.push_back(ugv1.slam_pose_msg);
			slam_msg.ugv_poses.push_back(ugv2.slam_pose_msg);

			slam_pose_pub.publish(slam_msg);

			ugv1.slam_pose_msg.observedByUGV = false;
			ugv2.slam_pose_msg.observedByUGV = false;
		}

		void publishUAVpose(ros::Time pubstamp)
		{//ROS_INFO("UAV void posePublisher(double pubtime)");
			// mfile.writeString("%% --------------- void publishUAVpose(ros::Time pubstamp)  --------------- \n\n" );
			// publish estimated pose for other nodes to use

			hast::posewithheader pose_msg;
			pose_msg.position.x = uav.EstPosition_gl.at<double>(0, 0);
			pose_msg.position.y = uav.EstPosition_gl.at<double>(1, 0);
			pose_msg.position.z = uav.EstPosition_gl.at<double>(2, 0);
			pose_msg.orientation = tf::createQuaternionMsgFromYaw(uav.EstPhi_gl);

			/*----- Time stamp of Estimate */
			pose_msg.header.seq = ++uav_c.pose_msg_seq;
			pose_msg.header.stamp = pubstamp;
			pose_msg.header.frame_id = "/map";

			uav_c.pose_pub.publish(pose_msg);
		}

		void uavPublishEst()
		{
			ros::Time stamp=ros::Time::now();
			uav.est_pose_msg.header.seq += 1;
			uav.est_pose_msg.header.stamp = stamp;
			uav.est_pose_msg.position.x = fullSLAM.augmentedState[0];
			uav.est_pose_msg.position.y = fullSLAM.augmentedState[1];
			uav.est_pose_msg.position.z = fullSLAM.augmentedState[2];
			uav.est_pose_msg.yaw				= fullSLAM.augmentedState[3];
			uav.est_state_pub.publish(uav.est_pose_msg);

			//update uav pose
				uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << fullSLAM.augmentedState[0], fullSLAM.augmentedState[1], fullSLAM.augmentedState[2]);
				// uav.EstPhi_gl = wrapRadians(fullSLAM.augmentedState[3]);
				uav.EstPhi_gl = (fullSLAM.augmentedState[3]);
				uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
				uav.Hlo2gl = invertH(uav.Hgl2lo);

			fprintf(fullfile.filename,"  %s.uav.est.time(%u,:) = % -6.14f;\n", s_full_field.c_str(), uav.est_pose_msg.header.seq, stamp.toSec()-init_time);
			fprintf(fullfile.filename,"  %s.uav.est.p.global(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_full_field.c_str(), uav.est_pose_msg.header.seq,
				uav.EstPosition_gl.at<double>(0,0),
				uav.EstPosition_gl.at<double>(1,0),
				uav.EstPosition_gl.at<double>(2,0));
			fprintf(fullfile.filename,"  %s.uav.est.p.correction(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_full_field.c_str(), uav.est_pose_msg.header.seq,
				fullSLAM.correction.at<double>(0,0),
				fullSLAM.correction.at<double>(1,0),
				fullSLAM.correction.at<double>(2,0));
			fprintf(fullfile.filename,"  %s.uav.est.yaw.global(%u,:)     = % -6.14f;\n", s_full_field.c_str(), uav.est_pose_msg.header.seq, uav.EstPhi_gl);
			fprintf(fullfile.filename,"  %s.uav.est.yaw.correction(%u,:) = % -6.14f;\n", s_full_field.c_str(), uav.est_pose_msg.header.seq, fullSLAM.correction.at<double>(3,0));
		}

		bool serveGoalLocation(hast::ugvgoal::Request &req, hast::ugvgoal::Response &res)
		{
			// first clear the costmaps
			// clearCostmaps_cli.call(nullcall);
			ROS_INFO("SLAM:: %s is requesting tag %i location for goal action", req.ugv_id.c_str(), req.tagID);
			ROS_INFO("SLAM::     (april[%i].id = %i)", req.tagID, april[req.tagID].id);
			ROS_INFO("SLAM::     (april[%i].measCount = %i)", req.tagID, april[req.tagID].measCount);

			if (april[req.tagID].measCount > 0)
			{
				if (req.ugv_id.compare("ugv1") == 0){ugv1.goal_id = req.tagID;}
				if (req.ugv_id.compare("ugv2") == 0){ugv2.goal_id = req.tagID;}
				april[req.tagID].isgoal = true;
				res.goalInMap = true;	// response to verify goal location is in the map
				res.goalP.x = april[req.tagID].EstPosition_gl.at<double>(0,0);	// position of target in map
				res.goalP.y = april[req.tagID].EstPosition_gl.at<double>(1,0);	// position of target in map
				res.goalP.z = april[req.tagID].EstPosition_gl.at<double>(2,0);	// position of target in map
				res.goalYaw = april[req.tagID].EstYaw_gl;	// yaw of target in map
			} else {
				ROS_INFO("requested April tag has not been observed");
				res.goalInMap = false;	// response to verify goal location is in the map
				res.goalP.x = 0;	// position of target in map
				res.goalP.y = 0;	// position of target in map
				res.goalP.z = 0;	// position of target in map
				res.goalYaw = 0;	// yaw of target in map
			}
			res.goalStamped.point.x = res.goalP.x;
			res.goalStamped.point.y = res.goalP.y;
			res.goalStamped.point.z = res.goalP.z;
			res.goalStamped.header.frame_id = "/ugv1/map";
			return true; // always reply true
		}

		void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
		{
			// mfile.writeString("%% --------------- void nodeShutDown(const hast::flag::ConstPtr& ShutDown)  --------------- \n\n" );
			// ROS_WARN("ckfRecorder: nodeShutDown");
			if (ShutDown->flag)
			{
				// ROS_INFO("  fullSLAM: init_pre_fileops requested..");
				fprintf(fullfile.filename,"\n%% --------------- close(wb) ---------------\n" );
				fprintf(fullfile.filename,"    try close(wb); end;\n");
				fullfile.init_pre_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/", s_full_field + "_" + s_trial + ".m", double(ugv1.obsCount));
				ROS_INFO("  fullSLAM: Shutdown requested..");

				ros::Duration(1.5).sleep();
				ros::shutdown();

			}
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "splitSLAM");

	splitSLAM uS;
	ros::spin();
	return 0;
}


/* ----- tag slam -----  */
// cv::Mat subsetP(std::vector<int> updatingIDs, int updatingTags)
// {// this takes the list of updating landmarks and generates the covariance for slam update
// 	// mfile.writeString("%% --------------- cv::Mat subsetP(std::vector<int> updatingIDs, int updatingTags) --------------- \n\n" );
// 	// See page 63 in Jan2019 notebook
// 		cv::Mat SLAM_P;
// 		uint ID, Ord; //indices for populating matrices
// 		SLAM_P = cv::Mat::zeros((1+updatingTags)*4, (1+updatingTags)*4, CV_64F);
//
// 		slam.augmentedCovariance.rowRange(0,4).colRange(0,4).copyTo(SLAM_P.rowRange(0,4).colRange(0,4));
//
// 	//then iterate through the relevant subblocks
// 	for(uint i = 0; i != updatingTags; i++)
// 	{
// 		ID = april[slam.updateForID[  i ]].id; // the numeric ID of the observed tag
// 		// fprintf(mfile.filename,"  %% ID = april[slam.updateForID[%02d]].id = %02d; \n", i, ID);
// 		Ord = april[updatingIDs[  i ]].order; // serial order of observation (1 indexed)
// 		// fprintf(mfile.filename,"  %% Ord = april[updatingIDs[%02d]].order = %02d; \n", i, Ord);
//
// 		// every landmark also has a covariance with the UAV
// 		// fprintf(mfile.filename," slam.augmentedCovariance.rowRange( 0, 4).colRange(%02d,%02d).copyTo(SLAM_P.rowRange( 0, 4).colRange(%02d,%02d));\n",
// 		// 																								4*(Ord+numUGV), 4*(Ord+numUGV+1), 																		 4*(i+1), 4*(i+2));
// 		slam.augmentedCovariance.rowRange(0,4).colRange(4*(Ord+numUGV), 4*(Ord+numUGV+1)).copyTo(SLAM_P.rowRange( 0, 4).colRange(4*(i+1), 4*(i+2)));
//
// 		// fprintf(mfile.filename," slam.augmentedCovariance.rowRange(%02d,%02d).colRange( 0, 4).copyTo(SLAM_P.rowRange(%02d,%02d).colRange( 0, 4));\n",
// 		// 																	4*(Ord+numUGV), 4*(Ord+numUGV+1), 																			4*(i+1), 4*(i+2));
// 		slam.augmentedCovariance.rowRange(4*(Ord+numUGV), 4*(Ord+numUGV+1)).colRange(0,4).copyTo(SLAM_P.rowRange(	4*(i+1), 4*(i+2)).colRange(0,4));
//
//
// 		for(uint idx = 0; idx != updatingTags; idx++)
// 		{ // see page 60 in Jan2019 notebook
// 			uint udx = april[updatingIDs[idx]].order;
// 			// fprintf(mfile.filename," slam.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d).copyTo(SLAM_P.rowRange(%02d,%02d).colRange(%02d,%02d));\n",
// 			// 																	4*(numUGV+Ord),4*(numUGV+Ord+1), 					4*(numUGV+udx),4*(numUGV+udx+1), 												4*(i+1),4*(i+2), 					4*(idx+1), 4*(idx+2));
// 			slam.augmentedCovariance.rowRange(4*(numUGV+Ord),4*(numUGV+Ord+1)).colRange(4*(numUGV+udx),4*(numUGV+udx+1)).copyTo(SLAM_P.rowRange(4*(i+1),4*(i+2)).colRange(4*(idx+1), 4*(idx+2)));
// 		}
// 	}
// 	// mfile.cellCvMatDoubles_multline(SLAM_P, "  fullSLAM.SLAM_P", slam.slamCount);
// 	// ROS_INFO("FINISHED slam.SLAM_P = subsetP(updatingIDs, updatingTags);");
// 	return SLAM_P;
// }

// void expandKk(cv::Mat SLAM_Kk, std::vector<int> updatingIDs, int updatingTags)
// {// see page 62 in Jan2019 notebook
// 	// mfile.writeString("%% --------------- void expandKk(cv::Mat SLAM_Kk, std::vector<int> updatingIDs, int updatingTags) --------------- \n\n" );
// 	// fprintf(mfile.filename,"expandKk(SLAM_Kk, updatingIDs, updatingTags);\n");
// 	cv::Mat aux_Kk;
// 	uint ID, Ord; //indices for populating matrices
// 	for(uint i = 0; i != updatingTags; i++)
// 	{
// 		ID = april[slam.updateForID[i]].id; // the numeric ID of the observed tag
// 		Ord = april[slam.updateForID[i]].order; // serial order of observation (1 indexed)
//
// 		// fprintf(mfile.filename, "%% i : %02d, Ord: %02d\n", i, Ord);
//
// 		// copy UAV Kk block to big Kk block
// 		// fprintf(mfile.filename," %% SLAM_Kk.rowRange( 0, 4).colRange(%02d,%02d).copyTo(slam.Kk.rowRange( 0, 4).colRange(%02d,%02d));\n",
// 		// 																4*(i),4*(i+1), 									 										 4*(Ord+1), 4*(Ord+2));
// 		SLAM_Kk.rowRange(0,4).colRange(	4*(i),4*(i+1)).copyTo(slam.Kk.rowRange(0,4).colRange(4*(Ord+1), 4*(Ord+2)));
//
// 		for(uint idx = 0; idx != updatingTags; idx++)
// 		{
// 			uint udx = april[updatingIDs[idx]].order;
// 			// fprintf(mfile.filename, "%% idx : %02d, udx: %02d\n", idx, udx);
// 			// fprintf(mfile.filename," %% SLAM_Kk.rowRange(%02d,%02d).colRange(%02d,%02d).copyTo(slam.Kk.rowRange(%02d,%02d).colRange(%02d,%02d));\n",
// 												// 4*(i+1), 4*(i+2),						4*(idx),4*(idx+1),												 	4*(udx+numUGV), 4*(udx+numUGV+1), 					4*(Ord+1),4*(Ord+2));
// 			SLAM_Kk.rowRange(	4*(i+1), 4*(i+2)).colRange(	4*(idx),4*(idx+1)).copyTo(slam.Kk.rowRange(	4*(udx+numUGV), 4*(udx+numUGV+1)).colRange(	4*(Ord+1),4*(Ord+2)));
// 		}
// 	}
// 	// ROS_INFO("Finished expandKk(SLAM_Kk, updatingIDs, updatingTags);");
// }

// mfile.writeString("%% --------------- measurement matrix --------------- \n" );
// cv::Mat SLAM_H; // measurement matrix based on one UGV and one UAV
// SLAM_H = cv::Mat::zeros(4,8, CV_64F);
// mfile.cellCvMatDoubles_multline(SLAM_H, "  %% fullSLAM.H", slam.slamCount);

// the first four columns of H are always block diagonals of I:
// slam.I4.copyTo(SLAM_H.rowRange(0,4).colRange(0,4)); // needs to be offset by UAV states
// slam.nI4.copyTo(SLAM_H.rowRange(0,4).colRange(4,8)); // this should write data to SLAM_H



// void expandP(cv::Mat SLAM_Kk, cv::Mat SLAM_H, cv::Mat SLAM_P, std::vector<int> updatingIDs, int updatingTags)
// {//fprintf(mfile.filename,"expandP( SLAM_Kk, SLAM_H, slam.SLAM_P, updatingIDs, updatingTags);\n");
// 	// mfile.writeString("%% --------------- void expandP(cv::Mat SLAM_Kk, cv::Mat SLAM_H, cv::Mat SLAM_P, std::vector<int> updatingIDs, int updatingTags) --------------- \n\n" );
//
// 		// slam.augmentedCovariance = (KHI - KH) * slam.augmentedCovariance;
// 		// Ck = (I-K H) Ck
// 		cv::Mat KH = SLAM_Kk * SLAM_H;
// 		KH.copyTo(slam.KH);
// 		// fprintf(mfile.filename,"%% KH [rows x cols] = [%i %i]\n", KH.rows, KH.cols);
//
// 		cv::Mat KHI = cv::Mat::eye(KH.rows, KH.cols, CV_64F); //64f == double
// 		// fprintf(mfile.filename,"%% slam.augmentedCovariance = (KHI - KH) * slam.augmentedCovariance; ([%i x %i]-[%i x %i]) * [%i x %i]\n", KHI.rows, KHI.cols, KH.rows, KH.cols, slam.augmentedCovariance.rows, slam.augmentedCovariance.cols);
//
// 		slam.ImKH = (KHI - KH);
// 		cv::Mat postP = (KHI - KH) * SLAM_P;
// 		// mfile.cellCvMatDoubles_multline(postP, "  fullSLAM.postP", slam.slamCount);
// 		postP.copyTo(slam.POST_P);
// 		// mfile.cellCvMatDoubles_multline(postP, "  fullSLAM.postP", slam.slamCount);
// 		// now extract the blocks and plug them into slam.augmentedCovariance;
//
// 		// fprintf(mfile.filename,"  %%   slam.augmentedCovariance = [%i x %i]\n", slam.augmentedCovariance.rows, slam.augmentedCovariance.cols);
//
// 		postP.rowRange(0,4).colRange(0,4).copyTo(slam.augmentedCovariance.rowRange(0,4).colRange(0,4));
//
// 		cv::Mat aux_P, aux_C;
// 		uint  ID, Ord, udx; //indices for populating matrices
// 		for(uint i = 0; i != updatingTags; i++)
// 		{
// 			// fprintf(mfile.filename,"\n");
// 			// ID  = april[updatingIDs[  i ]].id; // the numeric ID of the observed tag
// 			// fprintf(mfile.filename,"  %% ID  = april[updatingIDs[%02d]].id = %02d; \n", i, ID);
// 			Ord = april[updatingIDs[  i ]].order; // serial order of observation (1 indexed)
// 			// fprintf(mfile.filename,"  %% Ord = april[updatingIDs[%02d]].order = %02d; \n", i, Ord);
//
// 			// uav/tag blocks
// 			// fprintf(mfile.filename,"  %% uav row block\n");
// 			postP.rowRange( 0, 4).colRange(4*(i+1), 4*(i+2)).copyTo(slam.augmentedCovariance.rowRange(0,4).colRange(4*(Ord+numUAV+numUGV-1), 4*(Ord+numUAV+numUGV)));
//
// 			// fprintf(mfile.filename,"  %% uav col block\n");
// 			// fprintf(mfile.filename,"  %%   postP.rowRange(%02d,%02d).colRange( 0, 4).copyTo(slam.augmentedCovariance.rowRange(%02d,%02d).colRange( 0, 4));\n",
// 			// 								4*(i+1),4*(i+2),																												4*(Ord+numUAV+numUGV-1), 4*(Ord+numUAV+numUGV));
// 			postP.rowRange(	4*(i+1),4*(i+2)).colRange(0,4).copyTo(slam.augmentedCovariance.rowRange(4*(Ord+numUAV+numUGV-1), 4*(Ord+numUAV+numUGV)).colRange(0,4));
//
// 			// tag/tag block (only need one, the second loop will get the other)
// 			//  this will do the row of covariances from one tag to all of the others getting updated
// 			// fprintf(mfile.filename,"  %% tag/tag blocks\n");
// 			for(uint idx = 0; idx != updatingTags; idx++)
// 			{ // see page 60 in Jan2019 notebook
// 				udx = april[updatingIDs[idx]].order;
// 				// fprintf(mfile.filename,"  %% uint udx = april[updatingIDs[%02d]].updateOrder = %02d; \n", idx, udx);
//
// 				// fprintf(mfile.filename,"  %%   postP.rowRange(%02d,%02d).colRange(%02d,%02d).copyTo(slam.augmentedCovariance.rowRange(%02d,%02d).colRange(%02d,%02d));\n",
// 				// 							 4*(i+1),4*(i+2), 				 4*(udx+1), 4*(udx+2),																					4*(numUGV+Ord),4*(numUGV+Ord+1), 				  4*(numUAV+udx),4*(numUAV+udx+1));
// 				postP.rowRange(4*(i+1),4*(i+2)).colRange(4*(idx+1), 4*(idx+2)).copyTo(slam.augmentedCovariance.rowRange(4*(numUGV+Ord),4*(numUGV+Ord+1)).colRange(4*(numUGV+udx),4*(numUGV+udx+1)));
// 			}
// 		}
// 		// fprintf(mfile.filename,"%% ~~~~~~~~~~~~~~~~~~~~~~~\n");
// 		// ROS_INFO("Finished expandP( SLAM_Kk, SLAM_H, slam.SLAM_P, updatingIDs, updatingTags);");
// 		// mfile.cellCvMatDoubles_multline(postP, "  " + s_matlab_field + ".postP", slam.slamCount);
// }

// void writeCovTrace(double time)
// {
// 	fprintf(mfile.filename,"  %s.ugv1.aug.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", s_matlab_field.c_str(), slam.slamCount, slam.augmentedState[4*(1)+0], slam.augmentedState[4*(1)+1], slam.augmentedState[4*(1)+2]);
// 	fprintf(mfile.filename,"  %s.ugv2.aug.p.global(%u,:) = [% -16.14f, % -16.14f, % -16.14f];\n", s_matlab_field.c_str(), slam.slamCount, slam.augmentedState[4*(2)+0], slam.augmentedState[4*(2)+1], slam.augmentedState[4*(2)+2]);
// 	fprintf(mfile.filename,"  %s.ugv1.aug.yaw.global(%u,:) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, slam.augmentedState[4*(1)+3]);
// 	fprintf(mfile.filename,"  %s.ugv2.aug.yaw.global(%u,:) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, slam.augmentedState[4*(2)+3]);
//
// 	// mfile.cellCvMatDoubles_multline_ndec(slam.augmentedCovariance,	"  " + s_matlab_field + ".trace.prioriCovariance", slam.slamCount);
// 	cv::Mat tempMat;
// 	fprintf(mfile.filename,"  %s.trace.time(%u,:) = % -6.14f;\n", s_matlab_field.c_str(), slam.slamCount, time);
// 		fprintf(mfile.filename,"  %s.trace.system.trace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(slam.augmentedCovariance)[0]);
// 		mfile.cellCvMatDoubles_multline(slam.augmentedCovariance, 			"  " + s_matlab_field + ".trace.systemCovariance", slam.slamCount);
//
// 	// UAV trace
// 	slam.augmentedCovariance.rowRange(0,4).colRange(0,4).copyTo(tempMat);
// 		fprintf(mfile.filename,"  %s.trace.uav.trace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 		mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.uav.covmat", slam.slamCount);
// 		tempMat.release();
// 	// UGV1 trace
// 	slam.augmentedCovariance.rowRange(4,8).colRange(4,8).copyTo(tempMat);
// 		fprintf(mfile.filename,"  %s.trace.ugv1.trace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 		mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.ugv1.covmat", slam.slamCount);
// 		tempMat.release();
// 		// UGV1/UAV correlation trace
// 		// slam.augmentedCovariance.rowRange(0,4).colRange(4,8).copyTo(tempMat);
// 		// 	fprintf(mfile.filename,"  %s.trace.ugv1.cctrace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 		// 	mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.ugv1.ccmat", slam.slamCount);
// 		// 	tempMat.release();
//
// 	// UGV2 trace
// 	slam.augmentedCovariance.rowRange(8,12).colRange(8,12).copyTo(tempMat);
// 		fprintf(mfile.filename,"  %s.trace.ugv2.trace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 		mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.ugv2.covmat", slam.slamCount);
// 		tempMat.release();
// 		// UGV2/UAV correlation trace
// 		// slam.augmentedCovariance.rowRange(0,4).colRange(8,12).copyTo(tempMat);
// 		// 	fprintf(mfile.filename,"  %s.trace.ugv2.cctrace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 		// 	mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.ugv2.ccmat", slam.slamCount);
// 		// 	tempMat.release();
//
// 	uint ID, Ord; //indices for populating matrices
// 	for(uint i = 0; i != slam.landmarksOrdered.size(); i++)
// 	{
// 		april[slam.landmarksOrdered[  i ]].trace_count++;
// 		// TAGn trace
// 		slam.augmentedCovariance.rowRange(4*(numUAV+numUGV+i),4*(numUAV+numUGV+i+1)).colRange(4*(numUAV+numUGV+i),4*(numUAV+numUGV+i+1)).copyTo(tempMat);
// 			fprintf(mfile.filename,"  %s.trace.tag_%02d.time(%u,1) = % -16.14f;\n",  s_matlab_field.c_str(), slam.landmarksOrdered[i], april[slam.landmarksOrdered[i]].trace_count, time);
// 			fprintf(mfile.filename,"  %s.trace.tag_%02d.trace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.landmarksOrdered[i], april[slam.landmarksOrdered[i]].trace_count, cv::trace(tempMat)[0]);
// 			mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.tag_" + patch::to_string(slam.landmarksOrdered[i]) + ".covmat", april[slam.landmarksOrdered[  i ]].trace_count);
// 			tempMat.release();
// 			// TAGn/UAV correlation trace
// 			slam.augmentedCovariance.rowRange(0,4).colRange(8,12).copyTo(tempMat);
// 				fprintf(mfile.filename,"  %s.trace.ugv2.cctrace(%u,1) = % -16.14f;\n", s_matlab_field.c_str(), slam.slamCount, cv::trace(tempMat)[0]);
// 				mfile.cellCvMatDoubles_multline(tempMat, "  " + s_matlab_field + ".trace.ugv2.ccmat", slam.slamCount);
// 				tempMat.release();
//
// 	}
// 	// mfile.cellCvMatDoubles_multline_ndec(slam.augmentedCovariance,	"  " + s_matlab_field + ".trace.postCovariance", slam.slamCount);
// }
