#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
// #include "hastSLAM.hpp"
// #include "hastUAV.hpp"
#include "hastUGV.hpp"

namespace enc = sensor_msgs::image_encodings;

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
		// ROS_DEBUG flag
			bool debug_flag;

		/*---------  File Recorder ------------- */
			std::string s_trial, s_dotm, s_root, s_date, s_user, s_ugvn, s_exp_code;

			double Pi;
			int L2Norm;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
				ros::Subscriber stereoPose_sub, HastShutDown_sub;
				std::string s_shutdown_topic;

				ros::Subscriber slamState_sub;
					std::string s_slamState_topic;
					bool slamState;
					bool primaryUGV;

				ros::ServiceClient callTime_cli; // client to request master ros time offset
					hast::servtime time_call;
					std::string calling_node;


			/*----- Client Services */
			ros::ServiceServer StateService_ser;

		/*----- image transport Channels */
			image_transport::ImageTransport it;

			hastUGV ugv; // create ugv from class
			// ckfClass oneCKF, ugvCKF; // create ckf for uav and ugv

			tf::TransformListener listener; // tf listener for UAV slam location

		/*----- Stereo Variables */
			double stereoObs_stamp, stereoObs_yaw_ugv2uav;
			cv::Mat stereoObs_P, stereoObs_P_global, stereoObs_PCov;
			double stereoObs_yaw_cov;
			uint stereoObs_counter;

			std::string s_Pose_sub, s_ugvn_state, s_obstacle_cloud_topic, s_goal_cloud_topic;
	public:

		int framecount; // increment counter for dropping frames
		std::string s_ugv_primary_param;

		bool gazebo;
		double init_time;

		ros::Time calltime, mastertime;

		struct ABS_POSE_struct
		{
			std::string topic;
			ros::ServiceServer service;
			ros::ServiceClient client;
			hast::ugvdrive call;
			cv::Mat position;
			double yaw;
			bool init;
			tf::TransformListener listener;
		};
		ABS_POSE_struct init_hack;

		struct UXV_ckfshare_struct
		{
			cv::Mat EstStates;
			bool call_bool;
			hast::ckfshare call; // service data for getting vehicle location
			std::string client_topic;
			ros::ServiceClient client;
		};

		struct ckfshare_struct
		{
			int call_count;
			std::string serv_topic;
			ros::ServiceServer service;
			UXV_ckfshare_struct uav, ugvn;
			cv::Scalar Qdk_trace;
		};
		ckfshare_struct ckfshare;

		ckfClass splitCKF; // create ckf for uav and both ugvs
		cv::Mat I4, nI4; // identity and negative identity matrices for updating CKF

		struct UGVUGV_SENSOR_struct
		{
			ros::Time obsTime;
			std::string vehicle_id, observed_id; // topic of just ugv sensor data
			std::string s_pose_sub_topic; // topic of just ugv sensor data
			ros::Subscriber stereo_ugvugv_sub; // subscriber of local ugv stereo sensor data
			ros::Subscriber ckf_ugvugv_sub; // subscriber of ugv sensor measurements from other ugv
			ros::Publisher ckf_ugvugv_pub; // Publisher of ugv sensor measurements to other ugv
			hast::ckfugv_sensor meas_msg; //measurement message of ugvugv sensor

			int count;
			double ugvyaw, ugvyaw_cov; // the yaw and cov of a measured stereo pose
			// double ugvyaw_gl, ugvyaw_gl_cov; // the yaw and cov of a measured stereo pose
			double ugvyawest, ugvyawest_cov; // the yaw and cov of a estimated pose
			cv::Mat ugvPobs, ugvPobs_cov; // position and covaraince of measured position of observed ugv
			cv::Mat ugvPobs_gl, ugvPobs_gl_cov; // position and covaraince of measured position of observed ugv
			cv::Mat ugvPest_gl, ugvPest_gl_cov; // global position and covaraince of estimated position of observed ugv
			cv::Mat ugvPest_ugv, ugvPest_ugv_cov; // local/ugv position and covaraince of estimated position of observed ugv

			cv::Mat Rk, Rk_inf; // matrix for holding Rk prior to sending to ckf

		};
		UGVUGV_SENSOR_struct ugv_sensor;


	ckfRecorder()
	: it(n)
	{
		/*---------  File Recorder Initilizer ------------- */
		ros::param::get("~ugv_n",s_ugvn);
		ros::param::get("~ugv_matlab_field",ugv.s_matlab_field);
		ros::param::get("~ugv_data_filename",ugv.s_data_filename);
		ros::param::get("~gazebo", gazebo);
		ros::param::get("~primary",primaryUGV);
		n.getParam("/hast/user",  s_user);
		n.getParam("/hast/date",  s_date);
		n.getParam("/hast/trial", s_trial);
		n.getParam("/hast/exp_code", s_exp_code);

		// ROS_INFO("~~~~~~~~ ugvCKF: init ugvCKF");
		/*--------- Math Constants ------------- */
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function
		debug_flag = false;
		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		ros::Duration(0.5).sleep();

		// if (n.getParam("/hast/init_time", init_time)){
			// callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
			// time_call.request.requesting_node = ugv.s_data_filename;
			// calltime = ros::Time::now();
			// bool call_bool = callTime_cli.call(time_call);
			// ROS_WARN("bool call_bool = %s", call_bool ? "true" : "false");
			// if (call_bool){
			// 	ROS_INFO("Master running.");
			// 	mastertime = time_call.response.header.stamp;
			// 	if (gazebo){
			// 		init_time = 0.0;
			// 	} else {
			// 		ROS_INFO("%s::init_time = %f", time_call.request.requesting_node.c_str(), init_time+=(calltime.toSec() - mastertime.toSec()));
			// 	}
			// 	ugv.init_time = init_time;
			// } else {
			// 	ros::shutdown();
			// }

			if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}
			ugv.init_time = init_time;


		ugv.data.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/" + ugv.s_data_filename + "_" + s_trial + ".m");
		ugv.s_prealloc = "/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/prealloc/pre_" + ugv.s_data_filename + "_" + s_trial + ".m";

		fprintf (ugv.data.filename, "  %s.params.node_start_time = %-12.10f;\n",	ugv.s_data_filename.c_str(), ros::Time::now().toSec());
		fprintf (ugv.data.filename, "  %s.params.init_time = %-12.10f;\n",				ugv.s_data_filename.c_str(), init_time);
		fprintf (ugv.data.filename, "  %s.params.calltime = %-12.10f;\n",					ugv.s_data_filename.c_str(), calltime.toSec());
		fprintf (ugv.data.filename, "  %s.params.mastertime = %-12.10f;\n",				ugv.s_data_filename.c_str(), mastertime.toSec());

		ugv.s_ugvn = s_ugvn;
		ugv.openmFile();

		/*--------- Initialize ROS Communication & Variables ------------- */
		ros::param::get("~odom_topic", ugv.s_wheel_odom_topic);
		ros::param::get("~footprint_topic",ugv.s_base_footprint_topic);

		if(!primaryUGV){
			ros::param::get("~dkf_topic", ugv.s_dkf_topic);

			// uav.state_sub   = n.subscribe(uav.s_uav_state_topic, 1, &hastUAV::updateState_sub , &uav);

			ugv.dkf_pub  	 	= n.advertise<hast::posewithcov>(ugv.s_dkf_topic, 1);
			ugv.initDKF();
		}

		ros::param::get("~shutdown_topic",s_shutdown_topic);
		HastShutDown_sub 	= n.subscribe(s_shutdown_topic,	1, &ckfRecorder::nodeShutDown, this);

		ros::param::get("~Pose_sub",s_Pose_sub);
		stereoPose_sub 		= n.subscribe(s_Pose_sub, 1, &ckfRecorder::stereoRead , this);

		ros::param::get("~slamstate_topic",s_slamState_topic);
		slamState_sub 		= n.subscribe(s_slamState_topic, 1, &ckfRecorder::slamStateRead , this);
		slamState = false;

		ros::param::get("~ugvn_state",s_ugvn_state);
		ugv.state_pub = n.advertise<hast::ugvstate>(s_ugvn_state, 1);
		ugv.posePublisher(ros::Time::now().toSec());
		ugv.WheelOdom_sub 	= n.subscribe(ugv.s_wheel_odom_topic,	5, &hastUGV::WheelOdometry, &ugv);

		ros::param::get("~absolute_init",init_hack.init);
		// ROS_WARN(":~:~:~:~:~:~:~:      %s.absolute_init :: %s", ugv.s_data_filename.c_str(), init_hack.init ? "true" : "false" );

		/*-----  Services and Clients */

		ugv.data.writeString("\n%% --------------- Ros Topics --------------- \n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.primaryUGV = ['" + ( primaryUGV ? "true" : "false") + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.s_slamState_topic = ['" + s_slamState_topic + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.s_Pose_sub = ['" + s_Pose_sub + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.s_wheel_odom_topic = ['" + ugv.s_wheel_odom_topic + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.absolute_init =  ['" + (init_hack.init ? "true" : "false")  + "'];\n" );

		/*----- dual-purpose stereo message variables */
		stereoObs_stamp = 0;
		stereoObs_counter = 0;
		stereoObs_yaw_ugv2uav = 0;
		stereoObs_P = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);

		// ugv-ugv sensor
		if (s_ugvn.compare("ugv1") == 0) {
			ugv_sensor.vehicle_id 	= "ugv1";
			ugv_sensor.observed_id	= "ugv2";
		}
		if (s_ugvn.compare("ugv2") == 0) {
			ugv_sensor.vehicle_id 	= "ugv2";
			ugv_sensor.observed_id	= "ugv1";
		}

		ugv_sensor.count = 0;
		ugv_sensor.ugvyaw = 0;
		ugv_sensor.ugvyawest = 0;
		ugv_sensor.ugvyaw_cov = 1;
		ugv_sensor.ugvyawest_cov = 1;
		ugv_sensor.ugvPobs					= (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		ugv_sensor.ugvPobs_gl				= (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		ugv_sensor.ugvPest_gl				= (cv::Mat_<double>(3, 1) << 0, 0, 0);
		ugv_sensor.ugvPest_ugv			= (cv::Mat_<double>(3, 1) << 0, 0, 0);
		ugv_sensor.ugvPobs_cov			= (cv::Mat_<double>(3, 3) << 1,0,0, 0,1,0, 0,0,1);
		ugv_sensor.ugvPobs_gl_cov		= (cv::Mat_<double>(3, 3) << 1,0,0, 0,1,0, 0,0,1);
		ugv_sensor.ugvPest_gl_cov		= (cv::Mat_<double>(3, 3) << 1,0,0, 0,1,0, 0,0,1);
		ugv_sensor.ugvPest_ugv_cov	= (cv::Mat_<double>(3, 3) << 1,0,0, 0,1,0, 0,0,1);

		ros::param::get("~local_ugv_sensor_pose_sub_topic", ugv_sensor.s_pose_sub_topic);
			ugv.data.writeString("  " + ugv.s_data_filename + ".topics.local_ugv_sensor_pose_sub_topic = ['" + ugv_sensor.s_pose_sub_topic + "'];\n" );
			ugv_sensor.stereo_ugvugv_sub	= n.subscribe(ugv_sensor.s_pose_sub_topic, 1, &ckfRecorder::ugv_sensor_sub , this);

		// this adverises the service of THIS ugv
		ckfshare.serv_topic					= "/ckfshare/" + ugv_sensor.vehicle_id + "_ckf/service";
		ckfshare.uav.client_topic		= "/ckfshare/uav_ckf/service";
		ckfshare.ugvn.client_topic	= "/ckfshare/" + ugv_sensor.observed_id  + "_ckf/service";

		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.ckf_share_serv_topic = ['"   + ckfshare.serv_topic + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.uav_ckf_share_client_topic = ['" + ckfshare.uav.client_topic + "'];\n" );
		ugv.data.writeString("  " + ugv.s_data_filename + ".topics.ugvn_ckf_share_client_topic = ['" + ckfshare.ugvn.client_topic + "'];\n" );

		ckfshare.service  			= n.advertiseService(ckfshare.serv_topic, &ckfRecorder::share_ckf , this);
		ckfshare.uav.client			= n.serviceClient<hast::ckfshare>(ckfshare.uav.client_topic, true);
		ckfshare.ugvn.client		= n.serviceClient<hast::ckfshare>(ckfshare.ugvn.client_topic, true);

		ckfshare.uav.EstStates = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
		ckfshare.ugvn.EstStates = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);

		ckfshare.call_count = 0;

		ugv.Qw_scale = 10;

		I4		=  cv::Mat::eye(4, 4, CV_64F);
		nI4		= -cv::Mat::eye(4, 4, CV_64F);
		splitCKF.Rk_inf = 10000 * cv::Mat::eye(4, 4, CV_64F);
		splitCKF.zk			= cv::Mat::zeros(8,  1, CV_64F); // [Stereo - (Estuav - Estugv); Stereo - (Estugv - Estugv)]
		splitCKF.Mech		= cv::Mat::zeros(8,  1, CV_64F);
		splitCKF.Aiding	= cv::Mat::zeros(8,  1, CV_64F);
		splitCKF.H 			= cv::Mat::zeros(8, 12, CV_64F); // will be populated later

		splitCKF.I			= cv::Mat::eye(12, 12, CV_64F);
		splitCKF.Fk			= cv::Mat::eye(12, 12, CV_64F);
		splitCKF.Qdk		= cv::Mat::zeros(12, 12, CV_64F);
		splitCKF.PosteriorCov = 0.001*cv::Mat::eye(12, 12, CV_64F);
		splitCKF.PosteriorEst = cv::Mat::zeros(12, 1, CV_64F);
		fprintf(ugv.data.filename,"\n%% --------------- init ckf --------------- %% \n");
		fprintf(ugv.data.filename,"  %s.ckf.Qw_scale = %-12.10f;\n",														ugv.s_data_filename.c_str(), ugv.Qw_scale);
		log_ckf(1);
	}

	void log_ckf( int count)
	{
		fprintf(ugv.data.filename,"  %s.ckf.time(%u,:) = %-12.10f;\n",													ugv.s_data_filename.c_str(), count, ros::Time::now().toSec() - init_time);
		fprintf(ugv.data.filename,"    %s.ckf.EstYaw_gl(%u,:) = %-12.10f;\n",										ugv.s_data_filename.c_str(), count, ugv.EstYaw_gl);
		fprintf(ugv.data.filename,"    %s.ckf.EstP_gl(%u,:) = [%-12.10f %-12.10f %-12.10f];\n",	ugv.s_data_filename.c_str(), count, ugv.EstPosition_gl.at<double>(0,0),		ugv.EstPosition_gl.at<double>(1,0),		ugv.EstPosition_gl.at<double>(2,0));
		ugv.data.cellCvMatDoubles_multline(splitCKF.PriorEst,			"    " + ugv.s_data_filename + ".ckf.PriorEst",			count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.PriorCov,			"    " + ugv.s_data_filename + ".ckf.PriorCov",			count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.zk,						"    " + ugv.s_data_filename + ".ckf.zk",						count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.H,						"    " + ugv.s_data_filename + ".ckf.H",						count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Kk,						"    " + ugv.s_data_filename + ".ckf.Kk",						count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Kkyk,					"    " + ugv.s_data_filename + ".ckf.Kkyk",					count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.PosteriorEst,	"    " + ugv.s_data_filename + ".ckf.PosteriorEst",	count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.PosteriorCov,	"    " + ugv.s_data_filename + ".ckf.PosteriorCov",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.zk,						"    " + ugv.s_data_filename + ".ckf.zk",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.PosteriorEst,	"    " + ugv.s_data_filename + ".ckf.correction",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.Mech,					"      " + ugv.s_data_filename + ".ckf.mech.P_gl",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.Aiding,				"    " + ugv.s_data_filename + ".ckf.aiding.P_gl",	count);
	}

	bool share_ckf(hast::ckfshare::Request &req, hast::ckfshare::Response &res)
	{// when this vehicle (ugvi) is observed by another vehicle (ugvn),
		// save observation info from other vehicle

		ckfshare.call_count +=1;
		std::string observing_ugv = req.stereo_obs.vehicle_id;

		fprintf(ugv.data.filename,"\n%% --------------- share_ckf request --------------- %% \n");
		fprintf(ugv.data.filename,"  %s.share_ckf.time(%u,:) = %-12.10f;\n",				ugv.s_data_filename.c_str(), ckfshare.call_count, ros::Time::now().toSec() - init_time);
		fprintf(ugv.data.filename,"  %s.share_ckf.observing_ugv{%u,1} = '%s';\n",		ugv.s_data_filename.c_str(), ckfshare.call_count, observing_ugv.c_str());
		fprintf(ugv.data.filename,"  %s.share_ckf.dkf_update{%u,1}    = '%s';\n",		ugv.s_data_filename.c_str(), ckfshare.call_count, req.ifDKF ? "true" : "false");

		cv::Mat observed_position = (cv::Mat_<double>(3, 1) << req.stereo_obs.pose.position.x, req.stereo_obs.pose.position.y, req.stereo_obs.pose.position.z);
		cv::Mat observed_pos_cov  = (cv::Mat_<double>(3, 3) <<
																	req.stereo_obs.pose.PCov.row0.x, req.stereo_obs.pose.PCov.row0.y, req.stereo_obs.pose.PCov.row0.z,
																	req.stereo_obs.pose.PCov.row1.x, req.stereo_obs.pose.PCov.row1.y, req.stereo_obs.pose.PCov.row1.z,
																	req.stereo_obs.pose.PCov.row2.x, req.stereo_obs.pose.PCov.row2.y, req.stereo_obs.pose.PCov.row2.z);
		double observed_yaw 		= req.stereo_obs.pose.yaw; // make sure this is in radians before it is sent here
		double observed_yaw_cov = req.stereo_obs.pose.yaw_cov;

		cv::Mat ugvEst_pos = (cv::Mat_<double>(3, 1) << req.ugvEst.position.x, req.ugvEst.position.y, req.ugvEst.position.z);
		cv::Mat ugvEst_pos_cov  = (cv::Mat_<double>(3, 3) <<
																	req.ugvEst.PCov.row0.x, req.ugvEst.PCov.row0.y, req.ugvEst.PCov.row0.z,
																	req.ugvEst.PCov.row1.x, req.ugvEst.PCov.row1.y, req.ugvEst.PCov.row1.z,
																	req.ugvEst.PCov.row2.x, req.ugvEst.PCov.row2.y, req.ugvEst.PCov.row2.z);
		double ugvEst_yaw 		= req.ugvEst.yaw;
		double ugvEst_yaw_cov = req.ugvEst.yaw_cov;

		cv::Mat ugvn_Qdk = (cv::Mat_<double>(4, 4) <<
																	req.ugvEst.PCov.row0.x, req.ugvEst.PCov.row0.y, req.ugvEst.PCov.row0.z, 0,
																	req.ugvEst.PCov.row1.x, req.ugvEst.PCov.row1.y, req.ugvEst.PCov.row1.z, 0,
																	req.ugvEst.PCov.row2.x, req.ugvEst.PCov.row2.y, req.ugvEst.PCov.row2.z, 0,
																	0,0,0,req.ugvEst.yaw_cov);

		// respond with current estimate of this vehicle's pose
		res.pose.position.x = ugv.EstPosition_gl.at<double>(0,0);
		res.pose.position.y = ugv.EstPosition_gl.at<double>(1,0);
		res.pose.position.z = ugv.EstPosition_gl.at<double>(2,0);

		res.pose.PCov.row0.x = ugv.ckf.Qdk.at<double>(0,0); res.pose.PCov.row0.y = ugv.ckf.Qdk.at<double>(0,1); res.pose.PCov.row0.z = ugv.ckf.Qdk.at<double>(0,2);
		res.pose.PCov.row1.x = ugv.ckf.Qdk.at<double>(1,0); res.pose.PCov.row1.y = ugv.ckf.Qdk.at<double>(1,1); res.pose.PCov.row1.z = ugv.ckf.Qdk.at<double>(1,2);
		res.pose.PCov.row2.x = ugv.ckf.Qdk.at<double>(2,0); res.pose.PCov.row2.y = ugv.ckf.Qdk.at<double>(2,1); res.pose.PCov.row2.z = ugv.ckf.Qdk.at<double>(2,2);

		res.pose.yaw 			= ugv.EstYaw_gl;
		res.pose.yaw_cov	= ugv.ckf.Qdk.at<double>(3,3);

		fprintf(ugv.data.filename,"    %s.share_ckf.req.obs_yaw(%u,:)    =  %-12.10f;\n",											ugv.s_data_filename.c_str(), ckfshare.call_count, observed_yaw);
		fprintf(ugv.data.filename,"    %s.share_ckf.req.ugvEst_yaw(%u,:) =  %-12.10f;\n",											ugv.s_data_filename.c_str(), ckfshare.call_count, ugvEst_yaw);
		fprintf(ugv.data.filename,"    %s.share_ckf.req.obs_pos(%u,:)    = [%-12.10f %-12.10f %-12.10f];\n",	ugv.s_data_filename.c_str(), ckfshare.call_count, observed_position.at<double>(0,0), observed_position.at<double>(1,0), observed_position.at<double>(2,0));
		fprintf(ugv.data.filename,"    %s.share_ckf.req.ugvEst_pos(%u,:) = [%-12.10f %-12.10f %-12.10f];\n",	ugv.s_data_filename.c_str(), ckfshare.call_count, ugvEst_pos.at<double>(0,0), ugvEst_pos.at<double>(1,0), ugvEst_pos.at<double>(2,0));
		fprintf(ugv.data.filename,"    %s.share_ckf.res.EstYaw_gl(%u,:)  =  %-12.10f;\n",											ugv.s_data_filename.c_str(), ckfshare.call_count, res.pose.yaw);
		fprintf(ugv.data.filename,"    %s.share_ckf.res.EstP_gl(%u,:)    = [%-12.10f %-12.10f %-12.10f];\n",	ugv.s_data_filename.c_str(), ckfshare.call_count, res.pose.position.x, res.pose.position.y, res.pose.position.z);

		// we'll say that the augmented state vector is [UAV; UGVn]
		// so here we're only updating ugv/ugv measurements
		// reset matrices
		splitCKF.Qdk	= cv::Mat::zeros(12, 12, CV_64F);
 		splitCKF.Rk		= cv::Mat::zeros( 8,  8, CV_64F);
		splitCKF.H		= cv::Mat::zeros( 8, 12, CV_64F);
			// only populate the ugv-ugv CKF measurements
			// splitCKF.H		=[	0  0  0
			//									0 -I  I		]
			nI4.copyTo(splitCKF.H.rowRange(4,8).colRange(4, 8));
			 I4.copyTo(splitCKF.H.rowRange(4,8).colRange(8,12));

		cv::Mat ugv_Rk = (cv::Mat_<double>(4, 4) <<
											req.stereo_obs.pose.PCov.row0.x, req.stereo_obs.pose.PCov.row0.y, req.stereo_obs.pose.PCov.row0.z, 0,
											req.stereo_obs.pose.PCov.row1.x, req.stereo_obs.pose.PCov.row1.y, req.stereo_obs.pose.PCov.row1.z, 0,
											req.stereo_obs.pose.PCov.row2.x, req.stereo_obs.pose.PCov.row2.y, req.stereo_obs.pose.PCov.row2.z, 0,
											0,0,0,req.ugvEst.yaw_cov);
		ugv_Rk.copyTo(splitCKF.Rk.rowRange(4,8).colRange(4,8));
		splitCKF.Rk_inf.copyTo(splitCKF.Rk.rowRange(0,4).colRange(0,4));

		cv::Mat ugv_Qdk = (cv::Mat_<double>(4, 4) <<
											ugv.ckf.Qdk.at<double>(0,0), ugv.ckf.Qdk.at<double>(0,1), ugv.ckf.Qdk.at<double>(0,2), ugv.ckf.Qdk.at<double>(0,3),
											ugv.ckf.Qdk.at<double>(1,0), ugv.ckf.Qdk.at<double>(1,1), ugv.ckf.Qdk.at<double>(1,2), ugv.ckf.Qdk.at<double>(1,3),
											ugv.ckf.Qdk.at<double>(2,0), ugv.ckf.Qdk.at<double>(2,1), ugv.ckf.Qdk.at<double>(2,2), ugv.ckf.Qdk.at<double>(2,3),
											ugv.ckf.Qdk.at<double>(3,0), ugv.ckf.Qdk.at<double>(3,1), ugv.ckf.Qdk.at<double>(3,2), ugv.ckf.Qdk.at<double>(3,3));

		// splitCKF.Qdk		=[	Quav	0  		0
		//										0 		Qugv	0
		//										0			0			Qugvn ]
		// ugv_Qdk += ugvn_Qdk; // Qdk_ugvi + Qdk_ugvn
		ugv_Qdk.copyTo(splitCKF.Qdk.colRange(4,8).rowRange(4,8));
		ugvn_Qdk.copyTo(splitCKF.Qdk.colRange(8,12).rowRange(8,12));
		ckfshare.Qdk_trace = cv::trace(splitCKF.Qdk);

		splitCKF.zk = -(cv::Mat_<double>(8, 1) << // aiding - (mech1 - mech2)
											0,0,0,0,
											req.stereo_obs.pose.position.x - ((ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0)) - req.ugvEst.position.x),
											req.stereo_obs.pose.position.y - ((ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0)) - req.ugvEst.position.y),
											req.stereo_obs.pose.position.z - ((ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0)) - req.ugvEst.position.z),
											req.stereo_obs.pose.yaw - ((ugv.EstYaw_gl + ugv.yawCorrection) - req.ugvEst.yaw));

		fprintf(ugv.data.filename,"\n%% --------------- ckf incrementKF --------------- %% \n");
		ugv.data.cellCvMatDoubles_multline(ugv_Qdk,			"    " + ugv.s_data_filename + ".ckf.ckfshare.ugv_Qdk",		ckfshare.call_count);
		ugv.data.cellCvMatDoubles_multline(ugvn_Qdk,		"    " + ugv.s_data_filename + ".ckf.ckfshare.ugvn_Qdk",	ckfshare.call_count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Qdk,"    " + ugv.s_data_filename + ".ckf.ckfshare.Qdk",				ckfshare.call_count);
		ugv.data.cellCvMatDoubles_multline(ugv_Rk,			"    " + ugv.s_data_filename + ".ckf.ckfshare.ugv_Rk", 		ckfshare.call_count);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Rk,	"    " + ugv.s_data_filename + ".ckf.ckfshare.Rk",				ckfshare.call_count);
		fprintf(ugv.data.filename,"    %% %s.ckf.Qdk_CVtrace = %6.14f;\n",	ugv.s_data_filename.c_str(),	ckfshare.Qdk_trace[0]);

		ugv_Qdk.release(); ugvn_Qdk.release(); ugv_Rk.release();

		if (!(ckfshare.Qdk_trace[0] < 0.0001) )		// if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
		{
			splitCKF.zKF();
			ugv.updatePoseFromCKF((cv::Mat_<double>(3, 1) << splitCKF.PosteriorEst.at<double>(4,0), splitCKF.PosteriorEst.at<double>(5,0), splitCKF.PosteriorEst.at<double>(6,0)), splitCKF.PosteriorEst.at<double>(7,0));
			log_ckf(splitCKF.counter);
		}

		// if (s_ugvn.compare("ugv2") == 0) {fprintf(stdout, (RED_TEXT  "%s :: void share_ckf() ... done\n" COLOR_RESET), ugv.s_data_filename.c_str());}
		return true;
	}

	// local stereo obs callback
	void ugv_sensor_sub(const hast::stereougv_sensor::ConstPtr& ugv_meas_pose)
	{ // this is a ugv-ugv-only update
		// read in ugv frame data, log it, and convert to global frame
		ugv_sensor.count += 1;
		ugv_sensor.obsTime = ugv_meas_pose -> header.stamp;
		ugv_sensor.ugvyaw  = ugv_meas_pose -> pose.yaw; //heading angle in degrees in ugv frame in radians
		ugv_sensor.ugvPobs = (cv::Mat_<double>(4, 1) << ugv_meas_pose -> pose.position.x, ugv_meas_pose -> pose.position.y, ugv_meas_pose -> pose.position.z, 1.0);
		ugv_sensor.ugvyaw_cov  = ugv_meas_pose -> pose.yaw_cov;
		ugv_sensor.ugvPobs_cov = (cv::Mat_<double>(3, 3) <<
					 ugv_meas_pose->pose.PCov.row0.x, ugv_meas_pose->pose.PCov.row0.y, ugv_meas_pose->pose.PCov.row0.z,
					 ugv_meas_pose->pose.PCov.row1.x, ugv_meas_pose->pose.PCov.row1.y, ugv_meas_pose->pose.PCov.row1.z,
					 ugv_meas_pose->pose.PCov.row2.x, ugv_meas_pose->pose.PCov.row2.y, ugv_meas_pose->pose.PCov.row2.z);

		// Convert to global frame
			// ugv_sensor.ugvPobs_gl = ugv.Hlo2gl * ugv_sensor.ugvPobs; // this uses the ugv base location
			ugv_sensor.ugvPobs_gl = ugv.Rlo2gl4x4 * ugv_sensor.ugvPobs; // this will just align the measurement to the global frame, without moving it to the global origin
			ugv_sensor.ugvPobs_gl_cov = ugv.Rgl2lo.t()* ugv_sensor.ugvPobs_cov * ugv.Rgl2lo;
			// ugv_sensor.ugvyaw_gl = ugv.EstYaw_gl + ugv_sensor.ugvyaw;
			// ugv_sensor.ugvyaw_gl_cov = ugv.ckf.Qdk.at<double>(3,3)+ugv_sensor.ugvyaw_cov;


		fprintf (ugv.data.filename, "\n%% --------------- Read ugv_sensor --------------- \n" );
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.time(%d,1)   = %-12.10f;\n", 		ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.obsTime.toSec() - init_time);
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvyaw(%d,1) = %-12.10f;\n", 		ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvyaw);
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvyaw_cov(%d,1) = %-12.10f;\n", ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvyaw_cov);
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvPobs(%d,:)    = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvPobs.at<double>(0,0), ugv_sensor.ugvPobs.at<double>(1,0), ugv_sensor.ugvPobs.at<double>(2,0));

		// when ugv observes other ugv, without seeing uav, call the ugv-ugv service/client to trigger updates
		ckfshare.ugvn.call.request.stereo_obs.vehicle_id  = ugv_sensor.vehicle_id; // observing vehicle
		ckfshare.ugvn.call.request.stereo_obs.observed_id = ugv_sensor.observed_id; // observed vehicle
		ckfshare.ugvn.call.request.stereo_obs.pose.position.x  = ugv_sensor.ugvPobs_gl.at<double>(0,0);
		ckfshare.ugvn.call.request.stereo_obs.pose.position.y  = ugv_sensor.ugvPobs_gl.at<double>(1,0);
		ckfshare.ugvn.call.request.stereo_obs.pose.position.z  = ugv_sensor.ugvPobs_gl.at<double>(2,0);
		ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.x = ugv_sensor.ugvPobs_gl_cov.at<double>(0,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.y = ugv_sensor.ugvPobs_gl_cov.at<double>(0,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.z = ugv_sensor.ugvPobs_gl_cov.at<double>(0,2);
		ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.x = ugv_sensor.ugvPobs_gl_cov.at<double>(1,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.y = ugv_sensor.ugvPobs_gl_cov.at<double>(1,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.z = ugv_sensor.ugvPobs_gl_cov.at<double>(1,2);
		ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.x = ugv_sensor.ugvPobs_gl_cov.at<double>(2,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.y = ugv_sensor.ugvPobs_gl_cov.at<double>(2,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.z = ugv_sensor.ugvPobs_gl_cov.at<double>(2,2);
		ckfshare.ugvn.call.request.stereo_obs.pose.yaw = ugv_sensor.ugvyaw;
		ckfshare.ugvn.call.request.stereo_obs.pose.yaw_cov = ugv_sensor.ugvyaw_cov;

		ckfshare.ugvn.call.request.ugvEst.position.x = ugv.EstPosition_gl.at<double>(0,0);
		ckfshare.ugvn.call.request.ugvEst.position.y = ugv.EstPosition_gl.at<double>(1,0);
		ckfshare.ugvn.call.request.ugvEst.position.z = ugv.EstPosition_gl.at<double>(2,0);
		ckfshare.ugvn.call.request.ugvEst.PCov.row0.x = ugv.ckf.Qdk.at<double>(0,0); ckfshare.ugvn.call.request.ugvEst.PCov.row0.y = ugv.ckf.Qdk.at<double>(0,1); ckfshare.ugvn.call.request.ugvEst.PCov.row0.z = ugv.ckf.Qdk.at<double>(0,2);
		ckfshare.ugvn.call.request.ugvEst.PCov.row1.x = ugv.ckf.Qdk.at<double>(1,0); ckfshare.ugvn.call.request.ugvEst.PCov.row1.y = ugv.ckf.Qdk.at<double>(1,1); ckfshare.ugvn.call.request.ugvEst.PCov.row1.z = ugv.ckf.Qdk.at<double>(1,2);
		ckfshare.ugvn.call.request.ugvEst.PCov.row2.x = ugv.ckf.Qdk.at<double>(2,0); ckfshare.ugvn.call.request.ugvEst.PCov.row2.y = ugv.ckf.Qdk.at<double>(2,1); ckfshare.ugvn.call.request.ugvEst.PCov.row2.z = ugv.ckf.Qdk.at<double>(2,2);
		ckfshare.ugvn.call.request.ugvEst.yaw = ugv.EstYaw_gl;
		ckfshare.ugvn.call.request.ugvEst.yaw_cov = ugv.ckf.Qdk.at<double>(3,3);
		ckfshare.ugvn.call_bool = ckfshare.ugvn.client.call(ckfshare.ugvn.call);

		ugv_sensor.ugvyawest  = ckfshare.ugvn.call.response.pose.yaw;
		ugv_sensor.ugvyaw_cov = ckfshare.ugvn.call.response.pose.yaw_cov;
		ugv_sensor.ugvPest_gl  = (cv::Mat_<double>(3, 1) << ckfshare.ugvn.call.response.pose.position.x, ckfshare.ugvn.call.response.pose.position.y, ckfshare.ugvn.call.response.pose.position.z);

		fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvyawest(%d,:)  = %-12.10f ;\n", 									ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvyawest);
		fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvPest_gl(%d,:) = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvPest_gl.at<double>(0,0), ugv_sensor.ugvPest_gl.at<double>(1,0), ugv_sensor.ugvPest_gl.at<double>(2,0));
		fprintf (ugv.data.filename, "    %s.ckf.EstYaw_prior(%u,:)           = %-12.10f;\n",										ugv.s_data_filename.c_str(), ugv_sensor.count, ugv.EstYaw_gl);
		fprintf (ugv.data.filename, "    %s.ckf.EstP_prior(%u,:)             = [%-12.10f %-12.10f %-12.10f];\n",	ugv.s_data_filename.c_str(), ugv_sensor.count, ugv.EstPosition_gl.at<double>(0,0),		ugv.EstPosition_gl.at<double>(1,0),		ugv.EstPosition_gl.at<double>(2,0));

		ugv_sensor.ugvPest_gl_cov  = (cv::Mat_<double>(3, 3) <<
									ckfshare.ugvn.call.response.pose.PCov.row0.x, ckfshare.ugvn.call.response.pose.PCov.row0.y, ckfshare.ugvn.call.response.pose.PCov.row0.z,
									ckfshare.ugvn.call.response.pose.PCov.row1.x, ckfshare.ugvn.call.response.pose.PCov.row1.y, ckfshare.ugvn.call.response.pose.PCov.row1.z,
									ckfshare.ugvn.call.response.pose.PCov.row2.x, ckfshare.ugvn.call.response.pose.PCov.row2.y, ckfshare.ugvn.call.response.pose.PCov.row2.z);

		cv::Mat ugvn_Qdk = (cv::Mat_<double>(4, 4) <<
									ckfshare.ugvn.call.response.pose.PCov.row0.x, ckfshare.ugvn.call.response.pose.PCov.row0.y, ckfshare.ugvn.call.response.pose.PCov.row0.z, 0,
									ckfshare.ugvn.call.response.pose.PCov.row1.x, ckfshare.ugvn.call.response.pose.PCov.row1.y, ckfshare.ugvn.call.response.pose.PCov.row1.z, 0,
									ckfshare.ugvn.call.response.pose.PCov.row2.x, ckfshare.ugvn.call.response.pose.PCov.row2.y, ckfshare.ugvn.call.response.pose.PCov.row2.z, 0,
									0,0,0,ckfshare.ugvn.call.response.pose.yaw_cov);

		// ok, so now I have the the measurement from the local stereo sensor and the response from the other ugv about the current estimate of its position.
		//the next thing to do, is to include these measurements into the ckf
				// so here we're only updating ugv/ugv measurements
				splitCKF.Qdk	= cv::Mat::zeros(12, 12, CV_64F); // this should be a zero matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
		 		splitCKF.Rk		= cv::Mat::zeros( 8,  8, CV_64F);
				splitCKF.H		= cv::Mat::zeros( 8, 12, CV_64F);
					// only populate the ugv-ugv CKF measurements
					// splitCKF.H		=[	0  0  0
					//									0 -I  I		]
					nI4.copyTo(splitCKF.H.rowRange(4,8).colRange(4, 8));
					 I4.copyTo(splitCKF.H.rowRange(4,8).colRange(8,12));

				cv::Mat ugv_Rk = (cv::Mat_<double>(4, 4) <<
															ugv_meas_pose->pose.PCov.row0.x, ugv_meas_pose->pose.PCov.row0.y, ugv_meas_pose->pose.PCov.row0.z, 0,
															ugv_meas_pose->pose.PCov.row1.x, ugv_meas_pose->pose.PCov.row1.y, ugv_meas_pose->pose.PCov.row1.z, 0,
															ugv_meas_pose->pose.PCov.row2.x, ugv_meas_pose->pose.PCov.row2.y, ugv_meas_pose->pose.PCov.row2.z, 0,
															0,0,0,ugv_meas_pose -> pose.yaw_cov);
				ugv_Rk.copyTo(splitCKF.Rk.rowRange(4,8).colRange(4,8));
				splitCKF.Rk_inf.copyTo(splitCKF.Rk.rowRange(0,4).colRange(0,4));

				cv::Mat ugv_Qdk = (cv::Mat_<double>(4, 4) <<
													ugv.ckf.Qdk.at<double>(0,0), ugv.ckf.Qdk.at<double>(0,1), ugv.ckf.Qdk.at<double>(0,2), ugv.ckf.Qdk.at<double>(0,3),
													ugv.ckf.Qdk.at<double>(1,0), ugv.ckf.Qdk.at<double>(1,1), ugv.ckf.Qdk.at<double>(1,2), ugv.ckf.Qdk.at<double>(1,3),
													ugv.ckf.Qdk.at<double>(2,0), ugv.ckf.Qdk.at<double>(2,1), ugv.ckf.Qdk.at<double>(2,2), ugv.ckf.Qdk.at<double>(2,3),
													ugv.ckf.Qdk.at<double>(3,0), ugv.ckf.Qdk.at<double>(3,1), ugv.ckf.Qdk.at<double>(3,2), ugv.ckf.Qdk.at<double>(3,3));

				// splitCKF.Qdk		=[	Quav	0  		0
				//										0 		Qugv	0
				//										0			0			Qugvn ]
				ugv_Qdk.copyTo(splitCKF.Qdk.colRange(4,8).rowRange(4,8));
				ugvn_Qdk.copyTo(splitCKF.Qdk.colRange(8,12).rowRange(8,12));
				ckfshare.Qdk_trace = cv::trace(splitCKF.Qdk);

				splitCKF.zk = (cv::Mat_<double>(8, 1) << // aiding - (mech1 - mech2)
													0,0,0,0,
													ugv_sensor.ugvPobs_gl.at<double>(0,0) - (ckfshare.ugvn.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
													ugv_sensor.ugvPobs_gl.at<double>(1,0) - (ckfshare.ugvn.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
													ugv_sensor.ugvPobs_gl.at<double>(2,0) - (ckfshare.ugvn.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
													ugv_sensor.ugvyaw - (ckfshare.ugvn.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)));


				fprintf(ugv.data.filename,"\n%% --------------- ckf incrementKF --------------- %% \n");
				ugv.data.cellCvMatDoubles_multline(ugv_Qdk,				"    " + ugv.s_data_filename + ".ckf.ugv_sensor.ugv_Qdk",		ugv_sensor.count);
				ugv.data.cellCvMatDoubles_multline(ugvn_Qdk,			"    " + ugv.s_data_filename + ".ckf.ugv_sensor.ugvn_Qdk",	ugv_sensor.count);
				ugv.data.cellCvMatDoubles_multline(splitCKF.Qdk,	"    " + ugv.s_data_filename + ".ckf.ugv_sensor.Qdk",				ugv_sensor.count);
				ugv.data.cellCvMatDoubles_multline(ugv_Rk,	 			"    " + ugv.s_data_filename + ".ckf.ugv_sensor.ugv_Rk",		ugv_sensor.count);
				ugv.data.cellCvMatDoubles_multline(splitCKF.Rk,		"    " + ugv.s_data_filename + ".ckf.ugv_sensor.Rk",				ugv_sensor.count);
				fprintf(ugv.data.filename,"    %% %s.ckf.Qdk_CVtrace = %6.14f;\n",	ugv.s_data_filename.c_str(), ckfshare.Qdk_trace[0]);

				ugv_Qdk.release(); ugvn_Qdk.release();

				if (!(ckfshare.Qdk_trace[0] < 0.0001) ) // if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
				{
					splitCKF.zKF();
					ugv.updatePoseFromCKF((cv::Mat_<double>(3, 1) << splitCKF.PosteriorEst.at<double>(4,0), splitCKF.PosteriorEst.at<double>(5,0), splitCKF.PosteriorEst.at<double>(6,0)), splitCKF.PosteriorEst.at<double>(7,0));
					log_ckf(splitCKF.counter);
				}
	}

	void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)
	{//ROS_INFO("void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)");
		// this updates the uav and the ugv-(if in FOV)
		// if (s_ugvn.compare("ugv2") == 0) {fprintf(stdout, (BLUE_TEXT "%s :: void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)\n" COLOR_RESET), ugv.s_data_filename.c_str());}

		uint stereo_seq = stereo_state_msg -> id;
		stereoObs_stamp = stereo_state_msg -> stamp;
		stereoObs_P = (cv::Mat_<double>(4, 1) << stereo_state_msg->P.x, stereo_state_msg->P.y, stereo_state_msg->P.z, 1.0);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) <<
						 stereo_state_msg->PCov.row0.x, stereo_state_msg->PCov.row0.y, stereo_state_msg->PCov.row0.z,
						 stereo_state_msg->PCov.row1.x, stereo_state_msg->PCov.row1.y, stereo_state_msg->PCov.row1.z,
						 stereo_state_msg->PCov.row2.x, stereo_state_msg->PCov.row2.y, stereo_state_msg->PCov.row2.z);

		stereoObs_yaw_ugv2uav = stereo_state_msg -> yaw; // uav heading angle in degrees in ugv frame
		stereoObs_yaw_cov = stereo_state_msg -> yaw_cov; // variance of heading angle (in radians)

		// stereoObs_P_global = ugv.Hlo2gl * stereoObs_P;  // this uses the ugv base location
		stereoObs_P_global = ugv.Rlo2gl4x4 * stereoObs_P; // this will just align the measurement to the global frame, without moving it to the global origin

		cv::Mat stereoObs_PCov_gl = ugv.Rgl2lo.t() * stereoObs_PCov * ugv.Rgl2lo;
		double stereoObs_yaw_gl = ugv.EstYaw_gl + (Pi * stereoObs_yaw_ugv2uav / 180);
		double stereoObs_yaw_gl_cov = ugv.ckf.Qdk.at<double>(3,3)+stereoObs_yaw_cov;
		// double stereoObs_yaw_gl = ugv.EstYaw_gl + (Pi * stereoObs_yaw_ugv2uav / 180);
		// double stereoObs_yaw_gl_cov = ugv.ckf.Qdk.at<double>(3,3)+stereoObs_yaw_cov;

		fprintf (ugv.data.filename, "\n%% --------------- stereoRead --------------- \n" ); stereoObs_counter +=1;
		fprintf (ugv.data.filename, "  %s.stereo.time(%d,1) = %-12.10f;\n",							ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_stamp - init_time);
		fprintf (ugv.data.filename, "  %s.stereo.slamState{%u,1} = '%s';\n",						ugv.s_data_filename.c_str(), stereoObs_counter, slamState ? "true" : "false" );
		fprintf (ugv.data.filename, "  %s.stereo.ugv_inFOV{%u,1} = '%s';\n",						ugv.s_data_filename.c_str(), stereoObs_counter, stereo_state_msg -> ugv_inFOV ? "true" : "false" );
		fprintf (ugv.data.filename, "  %s.stereo.uav.obsyaw_ugv(%d,1) = %-12.10f;\n", 	ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_yaw_ugv2uav);
		fprintf (ugv.data.filename, "  %s.stereo.uav.obsyaw_gl(%d,1)  = %-12.10f;\n", 	ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_yaw_gl);
		fprintf (ugv.data.filename, "  %s.stereo.uav.obsP_gl(%u,:)  = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_P_global.at<double>(0,0), stereoObs_P_global.at<double>(1,0), stereoObs_P_global.at<double>(2,0));
		fprintf (ugv.data.filename, "  %s.stereo.uav.obsP_ugv(%u,:) = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_P.at<double>(0,0), stereoObs_P.at<double>(1,0), stereoObs_P.at<double>(2,0));

		// fprintf (ugv.data.filename, "  %s.stereo.yaw_cov(%d,1)  = %-12.10f;\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_yaw_cov);
		// fprintf (ugv.data.filename, "%s.stereo.PCov(1,:,%d) = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2));
		// fprintf (ugv.data.filename, "%s.stereo.PCov(2,:,%d) = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2));
		// fprintf (ugv.data.filename, "%s.stereo.PCov(3,:,%d) = [%-12.10f %-12.10f %-12.10f];\n\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2));

		// reset flags
		ckfshare.uav.call_bool = false;
		ckfshare.ugvn.call_bool = false;
		// fprintf(stdout, (BLUE_TEXT  "ckfshare.uav.call_bool  = ckfshare.uav.client.call(ckfshare.uav.call)  = %s; \n" COLOR_RESET), ckfshare.uav.call_bool ? "true" : "false" );
		// fprintf(stdout, (BLUE_TEXT  "ckfshare.ugvn.call_bool = ckfshare.ugvn.client.call(ckfshare.uav.call) = %s; \n" COLOR_RESET), ckfshare.ugvn.call_bool ? "true" : "false" );

		if (!primaryUGV && !slamState) // ignore CKF update for initialization og ugv2
		{
			ckfshare.uav.call.request.ifDKF = true;
		} else {
			ckfshare.uav.call.request.ifDKF = false;
		}

		ckfshare.uav.call.request.stereo_obs.vehicle_id = ugv.s_ugvn;
		ckfshare.uav.call.request.stereo_obs.pose.position.x = stereoObs_P_global.at<double>(0,0);
		ckfshare.uav.call.request.stereo_obs.pose.position.y = stereoObs_P_global.at<double>(1,0);
		ckfshare.uav.call.request.stereo_obs.pose.position.z = stereoObs_P_global.at<double>(2,0);
		ckfshare.uav.call.request.stereo_obs.pose.PCov.row0.x = stereoObs_PCov_gl.at<double>(0,0); ckfshare.uav.call.request.stereo_obs.pose.PCov.row0.y = stereoObs_PCov_gl.at<double>(0,1); ckfshare.uav.call.request.stereo_obs.pose.PCov.row0.z = stereoObs_PCov_gl.at<double>(0,2);
		ckfshare.uav.call.request.stereo_obs.pose.PCov.row1.x = stereoObs_PCov_gl.at<double>(1,0); ckfshare.uav.call.request.stereo_obs.pose.PCov.row1.y = stereoObs_PCov_gl.at<double>(1,1); ckfshare.uav.call.request.stereo_obs.pose.PCov.row1.z = stereoObs_PCov_gl.at<double>(1,2);
		ckfshare.uav.call.request.stereo_obs.pose.PCov.row2.x = stereoObs_PCov_gl.at<double>(2,0); ckfshare.uav.call.request.stereo_obs.pose.PCov.row2.y = stereoObs_PCov_gl.at<double>(2,1); ckfshare.uav.call.request.stereo_obs.pose.PCov.row2.z = stereoObs_PCov_gl.at<double>(2,2);
		ckfshare.uav.call.request.stereo_obs.pose.yaw = stereoObs_yaw_ugv2uav*(Pi/180); // this shouldn't actually be converted to global.  it is already the angular diff between uav & ugv
		ckfshare.uav.call.request.stereo_obs.pose.yaw_cov = stereoObs_yaw_cov;
		// ckfshare.uav.call.request.stereo_obs.pose.yaw = stereoObs_yaw_gl;
		// ckfshare.uav.call.request.stereo_obs.pose.yaw_cov = stereoObs_yaw_gl_cov;

		ckfshare.uav.call.request.ugvEst.position.x = ugv.EstPosition_gl.at<double>(0,0);
		ckfshare.uav.call.request.ugvEst.position.y = ugv.EstPosition_gl.at<double>(1,0);
		ckfshare.uav.call.request.ugvEst.position.z = ugv.EstPosition_gl.at<double>(2,0);
		ckfshare.uav.call.request.ugvEst.PCov.row0.x = ugv.ckf.Qdk.at<double>(0,0); ckfshare.uav.call.request.ugvEst.PCov.row0.y = ugv.ckf.Qdk.at<double>(0,1); ckfshare.uav.call.request.ugvEst.PCov.row0.z = ugv.ckf.Qdk.at<double>(0,2);
		ckfshare.uav.call.request.ugvEst.PCov.row1.x = ugv.ckf.Qdk.at<double>(1,0); ckfshare.uav.call.request.ugvEst.PCov.row1.y = ugv.ckf.Qdk.at<double>(1,1); ckfshare.uav.call.request.ugvEst.PCov.row1.z = ugv.ckf.Qdk.at<double>(1,2);
		ckfshare.uav.call.request.ugvEst.PCov.row2.x = ugv.ckf.Qdk.at<double>(2,0); ckfshare.uav.call.request.ugvEst.PCov.row2.y = ugv.ckf.Qdk.at<double>(2,1); ckfshare.uav.call.request.ugvEst.PCov.row2.z = ugv.ckf.Qdk.at<double>(2,2);
		ckfshare.uav.call.request.ugvEst.yaw = ugv.EstYaw_gl;
		ckfshare.uav.call.request.ugvEst.yaw_cov = ugv.ckf.Qdk.at<double>(3,3);
		ckfshare.uav.call_bool = ckfshare.uav.client.call(ckfshare.uav.call);
		// fprintf(stdout, (BLUE_TEXT  "ckfshare.uav.call_bool = ckfshare.uav.client.call(ckfshare.uav.call) = %s; \n" COLOR_RESET), ckfshare.uav.call_bool ? "true" : "false" );
		fprintf (ugv.data.filename, "  %s.stereo.uav_call_bool{%u,1} = '%s';\n", ugv.s_data_filename.c_str(), stereoObs_counter, ckfshare.uav.call_bool ? "true" : "false" );

		// Need to do UAV & UGVn update
		if (stereo_state_msg -> ugv_inFOV)
		{
			// fprintf(stdout, (BLUE_TEXT  "stereo_state_msg -> ugv_inFOV = %s; \n" COLOR_RESET), stereo_state_msg -> ugv_inFOV ? "true" : "false" );
			ugv_sensor.count += 1;
			ugv_sensor.ugvyaw  = stereo_state_msg -> ugvyaw; //heading angle in degrees in ugv frame in radians
			ugv_sensor.ugvyaw_cov  = stereo_state_msg -> ugvyaw_cov;
			ugv_sensor.ugvPobs = (cv::Mat_<double>(4, 1) << stereo_state_msg->ugvP_obs.x, stereo_state_msg->ugvP_obs.y, stereo_state_msg->ugvP_obs.z, 1.0);
			ugv_sensor.ugvPobs_cov = (cv::Mat_<double>(3, 3) <<
					stereo_state_msg->ugvPCov_obs.row0.x, stereo_state_msg->ugvPCov_obs.row0.y, stereo_state_msg->ugvPCov_obs.row0.z,
					stereo_state_msg->ugvPCov_obs.row1.x, stereo_state_msg->ugvPCov_obs.row1.y, stereo_state_msg->ugvPCov_obs.row1.z,
					stereo_state_msg->ugvPCov_obs.row2.x, stereo_state_msg->ugvPCov_obs.row2.y, stereo_state_msg->ugvPCov_obs.row2.z);
			// convert to global frame
			// ugv_sensor.ugvPobs_gl = ugv.Hlo2gl * ugv_sensor.ugvPobs; // this uses the ugv base location
			ugv_sensor.ugvPobs_gl = ugv.Rlo2gl4x4 * ugv_sensor.ugvPobs; // this will just align the measurement to the global frame, without moving it to the global origin

			ugv_sensor.ugvPobs_gl_cov = ugv.Rgl2lo.t()* ugv_sensor.ugvPobs_cov * ugv.Rgl2lo;
			// ugv_sensor.ugvyaw_gl = ugv.EstYaw_gl + ugv_sensor.ugvyaw;
			// ugv_sensor.ugvyaw_gl_cov = ugv.ckf.Qdk.at<double>(3,3)+ugv_sensor.ugvyaw_cov;

				fprintf (ugv.data.filename, "\n%% --------------- Read ugv_sensor --------------- \n" );
				fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.time(%d,1)   = %-12.10f;\n", 		ugv.s_data_filename.c_str(), ugv_sensor.count, stereoObs_stamp - init_time);
				fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvyaw(%d,1) = %-12.10f;\n", 		ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvyaw);
				fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvyaw_cov(%d,1) = %-12.10f;\n", ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvyaw_cov);
				fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.ugvPobs(%d,:)    = [%-12.10f %-12.10f %-12.10f];\n", ugv.s_data_filename.c_str(), ugv_sensor.count, ugv_sensor.ugvPobs.at<double>(0,0), ugv_sensor.ugvPobs.at<double>(1,0), ugv_sensor.ugvPobs.at<double>(2,0));

			ckfshare.ugvn.call.request.stereo_obs.pose.position.x = ugv_sensor.ugvPobs_gl.at<double>(0,0);
			ckfshare.ugvn.call.request.stereo_obs.pose.position.y = ugv_sensor.ugvPobs_gl.at<double>(1,0);
			ckfshare.ugvn.call.request.stereo_obs.pose.position.z = ugv_sensor.ugvPobs_gl.at<double>(2,0);
			ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.x = ugv_sensor.ugvPobs_gl_cov.at<double>(0,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.y = ugv_sensor.ugvPobs_gl_cov.at<double>(0,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row0.z = ugv_sensor.ugvPobs_gl_cov.at<double>(0,2);
			ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.x = ugv_sensor.ugvPobs_gl_cov.at<double>(1,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.y = ugv_sensor.ugvPobs_gl_cov.at<double>(1,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row1.z = ugv_sensor.ugvPobs_gl_cov.at<double>(1,2);
			ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.x = ugv_sensor.ugvPobs_gl_cov.at<double>(2,0); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.y = ugv_sensor.ugvPobs_gl_cov.at<double>(2,1); ckfshare.ugvn.call.request.stereo_obs.pose.PCov.row2.z = ugv_sensor.ugvPobs_gl_cov.at<double>(2,2);
			ckfshare.ugvn.call.request.stereo_obs.pose.yaw = ugv_sensor.ugvyaw;
			ckfshare.ugvn.call.request.stereo_obs.pose.yaw_cov = ugv_sensor.ugvyaw_cov;

			ckfshare.ugvn.call.request.ugvEst.position.x = ugv.EstPosition_gl.at<double>(0,0);
			ckfshare.ugvn.call.request.ugvEst.position.y = ugv.EstPosition_gl.at<double>(1,0);
			ckfshare.ugvn.call.request.ugvEst.position.z = ugv.EstPosition_gl.at<double>(2,0);
			ckfshare.ugvn.call.request.ugvEst.PCov.row0.x = ugv.ckf.Qdk.at<double>(0,0); ckfshare.ugvn.call.request.ugvEst.PCov.row0.y = ugv.ckf.Qdk.at<double>(0,1); ckfshare.ugvn.call.request.ugvEst.PCov.row0.z = ugv.ckf.Qdk.at<double>(0,2);
			ckfshare.ugvn.call.request.ugvEst.PCov.row1.x = ugv.ckf.Qdk.at<double>(1,0); ckfshare.ugvn.call.request.ugvEst.PCov.row1.y = ugv.ckf.Qdk.at<double>(1,1); ckfshare.ugvn.call.request.ugvEst.PCov.row1.z = ugv.ckf.Qdk.at<double>(1,2);
			ckfshare.ugvn.call.request.ugvEst.PCov.row2.x = ugv.ckf.Qdk.at<double>(2,0); ckfshare.ugvn.call.request.ugvEst.PCov.row2.y = ugv.ckf.Qdk.at<double>(2,1); ckfshare.ugvn.call.request.ugvEst.PCov.row2.z = ugv.ckf.Qdk.at<double>(2,2);
			ckfshare.ugvn.call.request.ugvEst.yaw = ugv.EstYaw_gl;
			ckfshare.ugvn.call.request.ugvEst.yaw_cov = ugv.ckf.Qdk.at<double>(3,3);
			ckfshare.ugvn.call_bool = ckfshare.ugvn.client.call(ckfshare.ugvn.call);
			// fprintf(stdout, (BLUE_TEXT  "ckfshare.ugvn.call_bool = ckfshare.ugvn.client.call(ckfshare.uav.call) = %s; \n" COLOR_RESET), ckfshare.ugvn.call_bool ? "true" : "false" );
			fprintf (ugv.data.filename, "  %s.stereo.ugvn_call_bool{%u,1} = '%s';\n", ugv.s_data_filename.c_str(), stereoObs_counter, ckfshare.ugvn.call_bool ? "true" : "false" );
		}


		fprintf(ugv.data.filename,"\n%% --------------- ckf incrementKF --------------- %% \n");
		// splitCKF.Qdk		=[	Quav	0  		0
		//										0 		Qugv	0
		//										0			0			Qugvn ]
		splitCKF.Qdk	= cv::Mat::zeros(12, 12, CV_64F); // this should be a zero matrix with 4 rows and 4*(#UAV+#UGV+#landmarks)
		splitCKF.Rk		= cv::Mat::zeros( 8,  8, CV_64F);
		splitCKF.H		= cv::Mat::zeros( 8, 12, CV_64F);

		cv::Mat uav_Rk = (cv::Mat_<double>(4, 4) <<
												stereoObs_PCov_gl.at<double>(0,0), stereoObs_PCov_gl.at<double>(0,1), stereoObs_PCov_gl.at<double>(0,2), 0,
												stereoObs_PCov_gl.at<double>(1,0), stereoObs_PCov_gl.at<double>(1,1), stereoObs_PCov_gl.at<double>(1,2), 0,
												stereoObs_PCov_gl.at<double>(2,0), stereoObs_PCov_gl.at<double>(2,1), stereoObs_PCov_gl.at<double>(2,2), 0,
												0,0,0,stereoObs_yaw_gl_cov);
		cv::Mat uav_Qdk = (cv::Mat_<double>(4, 4) <<
												ckfshare.uav.call.response.pose.PCov.row0.x, ckfshare.uav.call.response.pose.PCov.row0.y, ckfshare.uav.call.response.pose.PCov.row0.z, 0,
												ckfshare.uav.call.response.pose.PCov.row1.x, ckfshare.uav.call.response.pose.PCov.row1.y, ckfshare.uav.call.response.pose.PCov.row1.z, 0,
												ckfshare.uav.call.response.pose.PCov.row2.x, ckfshare.uav.call.response.pose.PCov.row2.y, ckfshare.uav.call.response.pose.PCov.row2.z, 0,
												0,0,0,ckfshare.uav.call.response.pose.yaw_cov);
		cv::Mat ugv_Qdk = (cv::Mat_<double>(4, 4) <<
												ugv.ckf.Qdk.at<double>(0,0), ugv.ckf.Qdk.at<double>(0,1), ugv.ckf.Qdk.at<double>(0,2), ugv.ckf.Qdk.at<double>(0,3),
												ugv.ckf.Qdk.at<double>(1,0), ugv.ckf.Qdk.at<double>(1,1), ugv.ckf.Qdk.at<double>(1,2), ugv.ckf.Qdk.at<double>(1,3),
												ugv.ckf.Qdk.at<double>(2,0), ugv.ckf.Qdk.at<double>(2,1), ugv.ckf.Qdk.at<double>(2,2), ugv.ckf.Qdk.at<double>(2,3),
												ugv.ckf.Qdk.at<double>(3,0), ugv.ckf.Qdk.at<double>(3,1), ugv.ckf.Qdk.at<double>(3,2), ugv.ckf.Qdk.at<double>(3,3));

		 uav_Rk.copyTo( splitCKF.Rk.rowRange(0,4).colRange(0,4));
		uav_Qdk.copyTo(splitCKF.Qdk.colRange(0,4).rowRange(0,4));
		ugv_Qdk.copyTo(splitCKF.Qdk.colRange(4,8).rowRange(4,8));

		// fprintf(stdout, (BLUE_TEXT  "if (ckfshare.uav.call_bool && !ckfshare.ugvn.call_bool) = (%s && %s); \n" COLOR_RESET), ckfshare.uav.call_bool ? "true" : "false", !ckfshare.ugvn.call_bool ? "true" : "false" );
		if (ckfshare.uav.call_bool && !ckfshare.ugvn.call_bool)
		{ // just update from UAV observation
			// only populate the ugv-ugv CKF measurements
			// splitCKF.H		=[	I -I  0
			//									0  0  0		]
			 I4.copyTo(splitCKF.H.rowRange(0,4).colRange(0,4));
			nI4.copyTo(splitCKF.H.rowRange(0,4).colRange(4,8));

			splitCKF.zk = (cv::Mat_<double>(8, 1) << // aiding - (mech1 - mech2)
											stereoObs_P_global.at<double>(0,0) - (ckfshare.uav.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
											stereoObs_P_global.at<double>(1,0) - (ckfshare.uav.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
											stereoObs_P_global.at<double>(2,0) - (ckfshare.uav.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
											stereoObs_yaw_ugv2uav*(Pi/180) - (ckfshare.uav.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)),
											0,0,0,0);

			splitCKF.Rk_inf.copyTo(splitCKF.Rk.rowRange(4,8).colRange(4,8)); // ugv2 stereo measurement
		}

		if (ckfshare.uav.call_bool && ckfshare.ugvn.call_bool)
		{ // both UAV and UGV are visible and the service call succeded
			// there are three stereo measurements: uav from ugv1, ugv2 from ugv1, and the difference between the uav and ugv2 in the stereo frame
			// splitCKF.H		=[	I -I  0
			//									0 -I  I		]
			 I4.copyTo(splitCKF.H.rowRange(0,4).colRange(0, 4));
			nI4.copyTo(splitCKF.H.rowRange(0,4).colRange(4, 8));
			nI4.copyTo(splitCKF.H.rowRange(4,8).colRange(4, 8));
			 I4.copyTo(splitCKF.H.rowRange(4,8).colRange(8,12));

			// splitCKF.Rk		=[	Rkuav		0
			//										0 			Rkugv	]

			ugv_sensor.Rk = (cv::Mat_<double>(4, 4) <<
									ugv_sensor.ugvPobs_gl_cov.at<double>(0,0), ugv_sensor.ugvPobs_gl_cov.at<double>(0,1), ugv_sensor.ugvPobs_gl_cov.at<double>(0,2), 0,
									ugv_sensor.ugvPobs_gl_cov.at<double>(1,0), ugv_sensor.ugvPobs_gl_cov.at<double>(1,1), ugv_sensor.ugvPobs_gl_cov.at<double>(1,2), 0,
									ugv_sensor.ugvPobs_gl_cov.at<double>(2,0), ugv_sensor.ugvPobs_gl_cov.at<double>(2,1), ugv_sensor.ugvPobs_gl_cov.at<double>(2,2), 0,
									0,0,0,ugv_sensor.ugvyaw_cov);

			ugv_sensor.Rk.copyTo(splitCKF.Rk.colRange(4,8).rowRange(4,8)); // ugv2 stereo measurement

			cv::Mat ugvn_Qdk = (cv::Mat_<double>(4, 4) <<
									ckfshare.ugvn.call.response.pose.PCov.row0.x, ckfshare.ugvn.call.response.pose.PCov.row0.y, ckfshare.ugvn.call.response.pose.PCov.row0.z, 0,
									ckfshare.ugvn.call.response.pose.PCov.row1.x, ckfshare.ugvn.call.response.pose.PCov.row1.y, ckfshare.ugvn.call.response.pose.PCov.row1.z, 0,
									ckfshare.ugvn.call.response.pose.PCov.row2.x, ckfshare.ugvn.call.response.pose.PCov.row2.y, ckfshare.ugvn.call.response.pose.PCov.row2.z, 0,
									0,0,0,ckfshare.ugvn.call.response.pose.yaw_cov);

			ugvn_Qdk.copyTo(splitCKF.Qdk.colRange(8,12).rowRange(8,12));


			splitCKF.zk = (cv::Mat_<double>(8, 1) << // aiding - (mech1 - mech2)
												stereoObs_P_global.at<double>(0,0) - (ckfshare.uav.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
												stereoObs_P_global.at<double>(1,0) - (ckfshare.uav.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
												stereoObs_P_global.at<double>(2,0) - (ckfshare.uav.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
												stereoObs_yaw_ugv2uav*(Pi/180) - (ckfshare.uav.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)),
												ugv_sensor.ugvPobs_gl.at<double>(0,0) - (ckfshare.ugvn.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
												ugv_sensor.ugvPobs_gl.at<double>(1,0) - (ckfshare.ugvn.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
												ugv_sensor.ugvPobs_gl.at<double>(2,0) - (ckfshare.ugvn.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
												ugv_sensor.ugvyaw - (ckfshare.ugvn.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)));

			// splitCKF.zk = (cv::Mat_<double>(8, 1) << // aiding - (mech1 - mech2)
			// 									stereoObs_P_global.at<double>(0,0) - (ckfshare.uav.call.response.pose.position.x - ugv.EstPosition_gl.at<double>(0,0)),
			// 									stereoObs_P_global.at<double>(1,0) - (ckfshare.uav.call.response.pose.position.y - ugv.EstPosition_gl.at<double>(1,0)),
			// 									stereoObs_P_global.at<double>(2,0) - (ckfshare.uav.call.response.pose.position.z - ugv.EstPosition_gl.at<double>(2,0)),
			// 									stereoObs_yaw_ugv2uav*(Pi/180) - (ckfshare.uav.call.response.pose.yaw - ugv.EstYaw_gl),
			// 									ugv_sensor.ugvPobs_gl.at<double>(0,0) - (ckfshare.ugvn.call.response.pose.position.x - ugv.EstPosition_gl.at<double>(0,0)),
			// 									ugv_sensor.ugvPobs_gl.at<double>(1,0) - (ckfshare.ugvn.call.response.pose.position.y - ugv.EstPosition_gl.at<double>(1,0)),
			// 									ugv_sensor.ugvPobs_gl.at<double>(2,0) - (ckfshare.ugvn.call.response.pose.position.z - ugv.EstPosition_gl.at<double>(2,0)),
			// 									ugv_sensor.ugvyaw - (ckfshare.ugvn.call.response.pose.yaw - ugv.EstYaw_gl));

			ugv.data.cellCvMatDoubles_multline(ugv_sensor.Rk,	"    " + ugv.s_data_filename + ".ckf.stereo.ugvn_Rk",	stereoObs_counter);
			ugv.data.cellCvMatDoubles_multline(ugvn_Qdk,			"    " + ugv.s_data_filename + ".ckf.stereo.ugvn_Qdk",stereoObs_counter);
			// ugv_ugvn_Qdk.release();
			ugvn_Qdk.release();

			// ckf.uxv.EstP_prior = [uav ugv ugvn]
			fprintf (ugv.data.filename, "    %s.ckf.stereo.uxv.EstP_prior(%u,:) = [%-6.4f %-6.4f %-6.4f %-6.4f, %-6.4f %-6.4f %-6.4f %-6.4f, %-6.4f %-6.4f %-6.4f %-6.4f];\n",	ugv.s_data_filename.c_str(), stereoObs_counter,
					ckfshare.uav.call.response.pose.position.x, ckfshare.uav.call.response.pose.position.y, ckfshare.uav.call.response.pose.position.z, ckfshare.uav.call.response.pose.yaw,
					ugv.EstPosition_gl.at<double>(0,0),		ugv.EstPosition_gl.at<double>(1,0),		ugv.EstPosition_gl.at<double>(2,0), ugv.EstYaw_gl,
					ckfshare.ugvn.call.response.pose.position.x, ckfshare.ugvn.call.response.pose.position.y, ckfshare.ugvn.call.response.pose.position.z, ckfshare.ugvn.call.response.pose.yaw);
			fprintf (ugv.data.filename, "    %s.ckf.stereo.uxv.Aid_prior(%u,:)  = [%-6.4f %-6.4f %-6.4f %-6.4f, %-6.4f %-6.4f %-6.4f %-6.4f];\n",	ugv.s_data_filename.c_str(), stereoObs_counter,
					stereoObs_P_global.at<double>(0,0), stereoObs_P_global.at<double>(1,0), stereoObs_P_global.at<double>(2,0), stereoObs_yaw_ugv2uav*(Pi/180),
					ugv_sensor.ugvPobs_gl.at<double>(0,0), ugv_sensor.ugvPobs_gl.at<double>(1,0), ugv_sensor.ugvPobs_gl.at<double>(2,0), ugv_sensor.ugvyaw);
			fprintf (ugv.data.filename, "    %s.ckf.stereo.uxv.Mech_prior(%u,:) = [%-6.4f %-6.4f %-6.4f %-6.4f, %-6.4f %-6.4f %-6.4f %-6.4f];\n",	ugv.s_data_filename.c_str(), stereoObs_counter,
					(ckfshare.uav.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
					(ckfshare.uav.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
					(ckfshare.uav.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
					(ckfshare.uav.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)),
					(ckfshare.ugvn.call.response.pose.position.x - (ugv.EstPosition_gl.at<double>(0,0) + ugv.correction_gl.at<double>(0,0))),
					(ckfshare.ugvn.call.response.pose.position.y - (ugv.EstPosition_gl.at<double>(1,0) + ugv.correction_gl.at<double>(1,0))),
					(ckfshare.ugvn.call.response.pose.position.z - (ugv.EstPosition_gl.at<double>(2,0) + ugv.correction_gl.at<double>(2,0))),
					(ckfshare.ugvn.call.response.pose.yaw - (ugv.EstYaw_gl + ugv.yawCorrection)));
			fprintf (ugv.data.filename, "    %s.ckf.stereo.uxv.zk(%u,:)         = [%-6.4f %-6.4f %-6.4f %-6.4f, %-6.4f %-6.4f %-6.4f %-6.4f];\n",	ugv.s_data_filename.c_str(), stereoObs_counter,
							splitCKF.zk.at<double>(0,0), splitCKF.zk.at<double>(1,0), splitCKF.zk.at<double>(2,0), splitCKF.zk.at<double>(3,0),
							splitCKF.zk.at<double>(4,0), splitCKF.zk.at<double>(5,0), splitCKF.zk.at<double>(6,0), splitCKF.zk.at<double>(7,0));


		}

		ugv.data.cellCvMatDoubles_multline(uav_Rk,				"    " + ugv.s_data_filename + ".ckf.stereo.uav.Rk",	stereoObs_counter);
		ugv.data.cellCvMatDoubles_multline(uav_Qdk,				"    " + ugv.s_data_filename + ".ckf.stereo.uav.Qdk",	stereoObs_counter);
		ugv.data.cellCvMatDoubles_multline(ugv_Qdk,				"    " + ugv.s_data_filename + ".ckf.stereo.ugv.Qdk",	stereoObs_counter);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Rk,		"    " + ugv.s_data_filename + ".ckf.stereo.Rk",			stereoObs_counter);
		ugv.data.cellCvMatDoubles_multline(splitCKF.Qdk,	"    " + ugv.s_data_filename + ".ckf.stereo.Qdk",			stereoObs_counter);

		uav_Rk.release(); uav_Qdk.release(); ugv_Qdk.release();

		if (!primaryUGV && !slamState) // ignore CKF update for initialization og ugv2
		{
			// if (ckfshare.uav.call_bool){update_UGVdkf(ckfshare, stereoObs_stamp, stereo_seq);	}
				update_UGVdkf(ckfshare, stereoObs_stamp, stereo_seq);
		} else {
			ckfshare.Qdk_trace = cv::trace(splitCKF.Qdk);
			fprintf(ugv.data.filename,"   %% %s.ckf.EstYaw_gl = %-12.10f;\n",	ugv.s_data_filename.c_str(), ckfshare.Qdk_trace[0]);
			ugv.data.cellCvMatDoubles_multline(splitCKF.Qdk,   "    " + ugv.s_data_filename + ".ckf.Qdk", 		stereoObs_counter);

			if (!(ckfshare.Qdk_trace[0] < 0.0001) ) // if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
			{// fprintf(stdout, (BLUE_TEXT  "  splitCKF.zKF();\n" COLOR_RESET));
				splitCKF.zKF();
				ugv.updatePoseFromCKF((cv::Mat_<double>(3, 1) << splitCKF.PosteriorEst.at<double>(4,0), splitCKF.PosteriorEst.at<double>(5,0), splitCKF.PosteriorEst.at<double>(6,0)), splitCKF.PosteriorEst.at<double>(7,0));
				log_ckf(splitCKF.counter);
			}
		}

		fprintf (ugv.data.filename, "  try waitbar(%f/waitbar_max,wb); end\n", double(stereoObs_counter));
	}

	void update_UGVdkf(ckfshare_struct ckfshare, double stereotime, int dkf_seq)// used only to initialize non-primary UGVs, after that it is unused.
	{// calculate UGV position based on UAV location and UGV stereo
		// if (s_ugvn.compare("ugv2") == 0) {fprintf(stdout, (BLUE_TEXT "%s :: void update_UGVdkf(ckfshare_struct ckfshare, double stereotime, int dkf_seq)\n" COLOR_RESET), ugv.s_data_filename.c_str());}
		double zk_yaw = 0;
		double uxv_yaw_est = 0;
		double uxv_yaw_stereo = 0;
		cv::Mat uxv_EstPosition_gl = (cv::Mat_<double>(3,1) << 0,0,0); // pre-init Pstereo
		cv::Mat uxv_StereoObs = (cv::Mat_<double>(3,1) << 0,0,0); // pre-init Pstereo
		cv::Mat Pstereo = (cv::Mat_<double>(3,1) << 0,0,0); // pre-init Pstereo
		cv::Mat zk_P = (cv::Mat_<double>(3,1) << 0,0,0); // pre-init Pstereo

		if (dkf_seq>10)
		{ // wait for UAV to be clearly in sight of the primary UGV

			if (ckfshare.ugvn.call_bool)
			{ // just update from UGV observation
				ugv.data.writeString("\n%% --------------- dkf incrementKF (ugv) --------------- %% \n");
				uxv_yaw_est = ckfshare.ugvn.call.response.pose.yaw;
				uxv_yaw_stereo = ugv_sensor.ugvyaw;
				zk_yaw = ckfshare.ugvn.call.response.pose.yaw - ugv_sensor.ugvyaw;

				uxv_EstPosition_gl = (cv::Mat_<double>(3,1) <<
										ckfshare.ugvn.call.response.pose.position.x,
										ckfshare.ugvn.call.response.pose.position.y,
										ckfshare.ugvn.call.response.pose.position.z);

				uxv_StereoObs = (cv::Mat_<double>(3,1) <<
										ugv_sensor.ugvPobs.at<double>(0, 0),
										ugv_sensor.ugvPobs.at<double>(1, 0),
										ugv_sensor.ugvPobs.at<double>(2, 0));

				ugv.dkf.Rk = (cv::Mat_<double>(4, 4) <<
										ckfshare.ugvn.call.response.pose.PCov.row0.x, ckfshare.ugvn.call.response.pose.PCov.row0.y, ckfshare.ugvn.call.response.pose.PCov.row0.z, 0,
										ckfshare.ugvn.call.response.pose.PCov.row1.x, ckfshare.ugvn.call.response.pose.PCov.row1.y, ckfshare.ugvn.call.response.pose.PCov.row1.z, 0,
										ckfshare.ugvn.call.response.pose.PCov.row2.x, ckfshare.ugvn.call.response.pose.PCov.row2.y, ckfshare.ugvn.call.response.pose.PCov.row2.z, 0,
										0,0,0,ckfshare.ugvn.call.response.pose.yaw_cov) + ugv_sensor.Rk;

			} else if (ckfshare.uav.call_bool)
			{ // just update from UAV observation
				ugv.data.writeString("\n%% --------------- dkf incrementKF (uav) --------------- %% \n");
				// global position measurement
				uxv_yaw_est = ckfshare.uav.call.response.pose.yaw;
				uxv_yaw_stereo = (stereoObs_yaw_ugv2uav * Pi/180); // convert stereo to radians
				zk_yaw = ckfshare.uav.call.response.pose.yaw - (stereoObs_yaw_ugv2uav * Pi/180); // convert stereo to radians

				uxv_EstPosition_gl = (cv::Mat_<double>(3,1) <<
										ckfshare.uav.call.response.pose.position.x,
										ckfshare.uav.call.response.pose.position.y,
										ckfshare.uav.call.response.pose.position.z);

				uxv_StereoObs = (cv::Mat_<double>(3,1) <<
										stereoObs_P.at<double>(0, 0),
										stereoObs_P.at<double>(1, 0),
										stereoObs_P.at<double>(2, 0));

				ugv.dkf.Rk = (cv::Mat_<double>(4, 4) <<
											stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2), 0,
											stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2), 0,
											stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2), 0,
											0, 0, 0, stereoObs_yaw_cov)
									 + (cv::Mat_<double>(4, 4) <<
											ckfshare.uav.call.response.pose.PCov.row0.x, ckfshare.uav.call.response.pose.PCov.row0.y, ckfshare.uav.call.response.pose.PCov.row0.z, 0,
											ckfshare.uav.call.response.pose.PCov.row1.x, ckfshare.uav.call.response.pose.PCov.row1.y, ckfshare.uav.call.response.pose.PCov.row1.z, 0,
											ckfshare.uav.call.response.pose.PCov.row2.x, ckfshare.uav.call.response.pose.PCov.row2.y, ckfshare.uav.call.response.pose.PCov.row2.z, 0,
											0,0,0,ckfshare.uav.call.response.pose.yaw_cov);
			}

			if (ugv.dkf.counter == 0)
			{ // then this is the first update, just use the measurement as the estimate
				ugv.dkf.counter = 1;

				ugv.EstYaw_gl = zk_yaw;

				ugv.dkf.cosyaw = cos(ugv.EstYaw_gl);
				ugv.dkf.sinyaw = sin(ugv.EstYaw_gl);
				// ugv.dkf.cosyaw = cos(Pi * ugv.EstYaw_gl / 180);
				// ugv.dkf.sinyaw = sin(Pi * ugv.EstYaw_gl / 180);
				ugv.dkf.Rgl2lo = (cv::Mat_<double>(3, 3) <<
				 ugv.dkf.cosyaw, ugv.dkf.sinyaw, 0,
				-ugv.dkf.sinyaw, ugv.dkf.cosyaw, 0,
				0,0,1);

				Pstereo = ugv.dkf.Rgl2lo.t() * uxv_StereoObs;

				ugv.EstPosition_gl = uxv_EstPosition_gl - Pstereo;

				ugv.dkf.PosteriorEst = (cv::Mat_<double>(4, 1) <<
				ugv.EstPosition_gl.at<double>(0, 0),
				ugv.EstPosition_gl.at<double>(1, 0),
				ugv.EstPosition_gl.at<double>(2, 0),
				ugv.EstYaw_gl);

			} else {
				// secondary updates
				ugv.dkf.cosyaw = cos(zk_yaw);
				ugv.dkf.sinyaw = sin(zk_yaw);
				// ugv.dkf.cosyaw = cos(Pi * zk_yaw / 180);
				// ugv.dkf.sinyaw = sin(Pi * zk_yaw / 180);
				ugv.dkf.Rgl2lo = (cv::Mat_<double>(3, 3) <<
				 ugv.dkf.cosyaw, ugv.dkf.sinyaw, 0,
				-ugv.dkf.sinyaw, ugv.dkf.cosyaw, 0,
				 0,0,1);

				Pstereo = ugv.dkf.Rgl2lo.t() * uxv_StereoObs;

				zk_P = uxv_EstPosition_gl - Pstereo;

				ugv.dkf.zk = (cv::Mat_<double>(4,1) <<
				zk_P.at<double>(0, 0),
				zk_P.at<double>(1, 0),
				zk_P.at<double>(2, 0),
				zk_yaw);

				ugv.dkf.incrementKF();

				ugv.EstPosition_gl = (cv::Mat_<double>(3, 1) <<
				ugv.dkf.PosteriorEst.at<double>(0, 0),
				ugv.dkf.PosteriorEst.at<double>(1, 0),
				ugv.dkf.PosteriorEst.at<double>(2, 0));

				ugv.EstYaw_gl = ugv.dkf.PosteriorEst.at<double>(3, 0);

				ugv.dkf.PosteriorCov.copyTo(ugv.ckf.Qdk);

			}
		} //end if (dkf_seq>10)

			if(init_hack.init){
				/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
					tf::StampedTransform StampedXform_gl2ugv;
					try {listener.lookupTransform("/global", "/vicon/"+ s_ugvn +"/base_TF", ros::Time(0), StampedXform_gl2ugv);}
						catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/origin to /vicon/%s/base_TF is missing", s_ugvn.c_str());}
						double qroll, qpitch;
						tf::Matrix3x3(StampedXform_gl2ugv.getRotation()).getRPY(qroll, qpitch, init_hack.yaw);
						tf::Vector3 t_gl2ugv(StampedXform_gl2ugv.getOrigin().x(), StampedXform_gl2ugv.getOrigin().y(), StampedXform_gl2ugv.getOrigin().z());
						init_hack.position = (cv::Mat_<double>(3, 1) << StampedXform_gl2ugv.getOrigin().x(), StampedXform_gl2ugv.getOrigin().y(), StampedXform_gl2ugv.getOrigin().z());

						ugv.EstPosition_gl = (cv::Mat_<double>(3, 1) << init_hack.position.at<double>(0,0), init_hack.position.at<double>(1,0), init_hack.position.at<double>(2,0));
						ugv.EstYaw_gl = init_hack.yaw;

				/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
			}

			if (ugv.dkf.counter != 0)
			{
				ugv.data.writeDouble(stereotime - init_time,			("  " + ugv.s_data_filename + ".dkf.time"), ugv.dkf.counter);
				ugv.data.writeDouble(uxv_yaw_stereo,							("  " + ugv.s_data_filename + ".dkf.yaw_meas"), ugv.dkf.counter);
				ugv.data.writeDouble(uxv_yaw_est, 								("  " + ugv.s_data_filename + ".dkf.uxv.EstYaw_gl"), ugv.dkf.counter);
				ugv.data.writeDouble(ugv.EstYaw_gl, 							("  " + ugv.s_data_filename + ".dkf.ugv.EstYaw_gl"), ugv.dkf.counter);
				ugv.data.writeDouble(zk_yaw,											("  " + ugv.s_data_filename + ".dkf.zk_yaw"), ugv.dkf.counter);
				ugv.data.writeVecDoubles(uxv_EstPosition_gl,			("  " + ugv.s_data_filename + ".dkf.uxv.EstP_gl"), ugv.dkf.counter);
				ugv.data.writeVecDoubles(ugv.EstPosition_gl,			("  " + ugv.s_data_filename + ".dkf.ugv.EstP_gl"), ugv.dkf.counter);
				ugv.data.writeVecDoubles(uxv_StereoObs,						("  " + ugv.s_data_filename + ".dkf.Pstereo.ugv"), ugv.dkf.counter);
				ugv.data.writeVecDoubles(Pstereo, 								("  " + ugv.s_data_filename + ".dkf.Pstereo.gl"), ugv.dkf.counter);
				ugv.data.writeVecDoubles(ugv.dkf.zk,							("  " + ugv.s_data_filename + ".dkf.zk"), ugv.dkf.counter);
				// ugv.data.writeCvMatDoubles(ugv.dkf.Rk, ("  " + ugv.s_data_filename + ".dkf.Rk"), ugv.dkf.counter);
				ugv.data.writeCvMatDoubles(ugv.dkf.PosteriorCov,	("  " + ugv.s_data_filename + ".dkf.PosteriorCov"), ugv.dkf.counter);

				/*----- Time stamp of Estimate */
				ugv.dkf_msg.header.seq = ugv.dkf.counter;
				ugv.dkf_msg.header.stamp = ros::Time::now();
				ugv.dkf_msg.header.frame_id = s_ugvn;
				ugv.dkf_msg.vehicle_id = s_ugvn;

				ugv.dkf_msg.position.x = ugv.EstPosition_gl.at<double>(0, 0);
				ugv.dkf_msg.position.y = ugv.EstPosition_gl.at<double>(1, 0);
				ugv.dkf_msg.position.z = ugv.EstPosition_gl.at<double>(2, 0);
				ugv.dkf_msg.PCov.row0.x = ugv.ckf.Qdk.at<double>(0,0);
				ugv.dkf_msg.PCov.row1.y = ugv.ckf.Qdk.at<double>(1,1);
				ugv.dkf_msg.PCov.row2.z = ugv.ckf.Qdk.at<double>(2,2);

				ugv.dkf_msg.yaw = ugv.EstYaw_gl;
				ugv.dkf_msg.yaw_cov = ugv.ckf.Qdk.at<double>(3,3);

				ugv.dkf_pub.publish(ugv.dkf_msg);
			}

		}


	void slamStateRead(const hast::flag::ConstPtr& msg)
	{ // this is used to switch off the initilizer DKF
		// if (s_ugvn.compare("ugv2") == 0) {fprintf(stdout, (BLUE_TEXT "%s :: void slamStateRead(const hast::flag::ConstPtr& msg)\n" COLOR_RESET), ugv.s_data_filename.c_str());}
		if (msg->flag && !slamState) {fprintf (ugv.data.filename, "  %s.slam.switchtime  = %-12.10f;\n", ugv.s_data_filename.c_str(), ros::Time::now().toSec());}
		slamState = msg->flag;
		// if (s_ugvn.compare("ugv2") == 0) {fprintf(stdout, (RED_TEXT  "%s :: void slamStateRead() ... done\n" COLOR_RESET), ugv.s_data_filename.c_str());}
	}// ROS_INFO("~~~~~~~~~ %s::slamState  = %s", ugv.s_data_filename.c_str(), slamState ? "true" : "false" );

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if (ShutDown->flag)
		{
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugvCKF_Recorder");
	ckfRecorder cfkRec;
	ros::spin();
	return 0;
}
