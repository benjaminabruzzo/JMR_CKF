#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
// #include "hastSLAM.hpp"
#include "hastUAV.hpp"
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


class uavCKF
{
	private:

		/*---------  File Recorder ------------- */
			std::string s_trial, s_dotm, s_root, s_date, s_user, s_exp_code;

			double Pi;
			int L2Norm;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
				ros::Subscriber HastShutDown_sub;
				std::string s_shutdown_topic;

				ros::ServiceClient callTime_cli; // client to request master ros time offset
					hast::servtime time_call;
					std::string calling_node;

		/*----- image transport Channels */
			image_transport::ImageTransport it;

		/* COMBINED CKF *************																	******************  COMBINED CKF */
			hastUAV uav; // create uav from class
			ros::ServiceServer StateService_ser;
				std::string s_StateService_ser;

			tf::TransformListener listener; // tf listener for UAV slam location

			// april things
			ros::Subscriber TagDetections_sub;
				std::string s_tagDetections;

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

	public:

		bool gazebo;
		double init_time;

		ros::Time calltime, mastertime;

		struct ckfshare_struct
		{
			std::string serv_topic;
			ros::ServiceServer service;
			hast::ckfshare call; // service data for getting tag location
			bool call_bool;
			int call_count;
		};
		ckfshare_struct ckfshare;


		ckfClass splitCKF; // create ckf for uav and both ugvs
		cv::Mat I4, nI4; // identity and negative identity matrices for updating CKF

		cv::Scalar Qdk_CVtrace;

	uavCKF()
	: it(n)
	{
		ros::Duration(1.0).sleep();

		/*---------  File Recorder Initilizer ------------- */
		// ros::param::get("~uav_navdata_file",	uav.s_navdata_file);
		ros::param::get("~uav_data_filename",	uav.s_data_filename);
		ros::param::get("~gazebo", gazebo);
		n.getParam("/hast/user",  s_user);
		n.getParam("/hast/date",  s_date);
		n.getParam("/hast/trial", s_trial);
		n.getParam("/hast/exp_code", s_exp_code);

		/*--------- Math Constants ------------- */
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function
		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		uav.s_parent_node = "uavCKF";


		// if (n.getParam("/hast/init_time", init_time)){
			// callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
			// time_call.request.requesting_node = uav.s_data_filename;
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
			// 	uav.data_init_time = init_time;
			// } else {
			// 	ros::shutdown();
			// }

		if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}
		uav.data_init_time = init_time;

		uav.data.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/" + uav.s_data_filename + "_" + s_trial + ".m");
		uav.s_prealloc = "/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/prealloc/pre_" + uav.s_data_filename + "_" + s_trial + ".m";

		fprintf (uav.data.filename, "  %s.params.calltime = % -6.14f;\n",					uav.s_data_filename.c_str(), calltime.toSec());
		fprintf (uav.data.filename, "  %s.params.init_time = % -6.14f;\n",				uav.s_data_filename.c_str(), init_time);
		fprintf (uav.data.filename, "  %s.params.mastertime = % -6.14f;\n",				uav.s_data_filename.c_str(), mastertime.toSec());
		fprintf (uav.data.filename, "  %s.params.node_start_time = % -6.14f;\n",	uav.s_data_filename.c_str(), ros::Time::now().toSec());
		ros::param::get("~tag_max_id",					tag_max_id);
		fprintf (uav.data.filename, "  %s.params.tag_max_id = %d;\n", 						uav.s_data_filename.c_str(), tag_max_id);

		/*--------- Initialize ROS Communication & Variables ------------- */
		ros::param::get("~TF_oneckf_parent",		uav.s_TF_gl_parent);			uav.data.writeString("  " + uav.s_data_filename + ".topics.TF_oneckf_parent =  ['"	+ uav.s_TF_gl_parent  + "'];\n" );
		ros::param::get("~TF_oneckf_child",			uav.s_TF_gl_child);				uav.data.writeString("  " + uav.s_data_filename + ".topics.TF_oneckf_child =  ['"		+ uav.s_TF_gl_child		+ "'];\n" );
		ros::param::get("~TF_uav_slam_gl",			uav.s_TF_uav_slam_gl);		uav.data.writeString("  " + uav.s_data_filename + ".topics.TF_uav_slam_gl =  ['"		+ uav.s_TF_uav_slam_gl+ "'];\n" );
		ros::param::get("~uav_imu_yawdriftrate",uav.yaw_drift_rate);			// logged elsewhere ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		ros::param::get("~use_compassbias", 		uav.if_useBias); 					uav.data.writeString("  " + uav.s_data_filename + ".topics.if_useBias =  ['" + (uav.if_useBias ? "true" : "false")  + "'];\n" );

		/*-----  Publishers and Subscribers */
		ros::param::get("~navdata_topic",					uav.s_navdata_topic);			uav.data.writeString("  " + uav.s_data_filename + ".topics.s_navdata_topic =      ['"	+ uav.s_navdata_topic + "'];\n" );
		// ros::param::get("~uav_cmdvel_topic",		uav.s_uav_cmdvel_topic);	uav.data.writeString("  " + uav.s_data_filename + ".topics.s_uav_cmdvel_topic = ['" 	+ uav.s_uav_cmdvel_topic + "'];\n" );
		ros::param::get("~uav_state_topic",				uav.s_uav_state_topic);		uav.data.writeString("  " + uav.s_data_filename + ".topics.s_uav_state_topic =    ['" + uav.s_uav_state_topic + "'];\n" );
		ros::param::get("~uav_pose_topic",				uav.s_uav_pose_topic);		uav.data.writeString("  " + uav.s_data_filename + ".topics.s_uav_pose_topic =     ['" + uav.s_uav_pose_topic + "'];\n" );
		ros::param::get("~tag_detection_topic",		s_tagDetections);					uav.data.writeString("  " + uav.s_data_filename + ".topics.tag_detection_topic =  ['" + s_tagDetections + "'];\n" );
		ros::param::get("~shutdown_topic",				s_shutdown_topic);				uav.data.writeString("  " + uav.s_data_filename + ".topics.shutdown_topic =       ['" + s_shutdown_topic + "'];\n" );
		ros::param::get("~uav_states_service",		s_StateService_ser); 			uav.data.writeString("  " + uav.s_data_filename + ".topics.uav_states_service =   ['"	+ s_StateService_ser + "'];\n" );
		ros::param::get("~ckf_share_serv_topic",	ckfshare.serv_topic); 		uav.data.writeString("  " + uav.s_data_filename + ".topics.ckf_share_serv_topic = ['" + ckfshare.serv_topic + "'];\n" );


		uav.navData_sub			= n.subscribe(uav.s_navdata_topic,  	5, &hastUAV::inertialUpdate, &uav); //inertialUpdate
		// uav.cmdVel_sub			= n.subscribe(uav.s_uav_cmdvel_topic, 5, &hastUAV::readCmdVel, &uav); //
		uav.state_pub				= n.advertise<hast::uavstate>(uav.s_uav_state_topic, 1); // advertises uav state for autopilot
		uav.pose_pub  			= n.advertise<hast::posewithheader>(uav.s_uav_pose_topic, 1); // not sure which nodes use this form of the state, but it is redundant with state_pub
		HastShutDown_sub 		= n.subscribe(s_shutdown_topic,	1,	&uavCKF::nodeShutDown, this);
		TagDetections_sub 	= n.subscribe(s_tagDetections, 	1,	&uavCKF::tagDetections , this);
		StateService_ser  	= n.advertiseService(s_StateService_ser, &uavCKF::serveState , this); // this might not get used...
		ckfshare.service  	= n.advertiseService(ckfshare.serv_topic, &uavCKF::share_ckf , this);
			ckfshare.call_count = 0;

		tag_counter = 0;
		april.reserve( tag_max_id );
		for(uint i = 0; i != tag_max_id; i++) //init april tags
		{
			apriltagclass obj;
			obj.id = i;
			april.push_back(obj);
		}

		// ckfcontainters for UAV local Q:
		uav.ckf.Qdk =   cv::Mat::zeros(4, 4, CV_64F);
		uav.ckf.Qw	= 0.1*cv::Mat::eye(4, 4, CV_64F);
		uav.ckf.Fk	=     cv::Mat::eye(4, 4, CV_64F);


		// containters for ckf for uav and both ugvs
		I4		=  cv::Mat::eye(4, 4, CV_64F);
		nI4		= -cv::Mat::eye(4, 4, CV_64F);
		splitCKF.Rk_inf = 10000 * cv::Mat::eye(4, 4, CV_64F);
		splitCKF.I			= cv::Mat::eye(12, 12, CV_64F);
		splitCKF.Fk			= cv::Mat::eye(12, 12, CV_64F);
		splitCKF.Qdk		= cv::Mat::zeros(12, 12, CV_64F);

		splitCKF.zk			= cv::Mat::zeros(8,  1, CV_64F); // [Stereo - (Estuav - Estugv); Stereo - (Estugv - Estugv)]
		splitCKF.H 			= cv::Mat::zeros(8, 12, CV_64F); // will be populated later

		splitCKF.PosteriorCov = 0.001*cv::Mat::eye(12, 12, CV_64F);
		splitCKF.PosteriorEst = cv::Mat::zeros(12, 1, CV_64F);

		fprintf(uav.data.filename,"\n%% --------------- init ckf --------------- %% \n");
		// fprintf(uav.data.filename,"  %s.ckf.Qw_scale = %-12.10f;\n",		uav.s_data_filename.c_str(), ugv.Qw_scale);
		log_ckf(1);

	}

	void log_ckf( int count)
	{
		fprintf(uav.data.filename,"  %s.ckf.time(%u,:) = %-12.10f;\n",													uav.s_data_filename.c_str(), count, ros::Time::now().toSec() - init_time);
		fprintf(uav.data.filename,"    %s.ckf.EstYaw_gl(%u,:) = %-12.10f;\n",										uav.s_data_filename.c_str(), count, uav.EstYaw_gl * (Pi/180));
		fprintf(uav.data.filename,"    %s.ckf.EstP_gl(%u,:) = [%-12.10f %-12.10f %-12.10f];\n",	uav.s_data_filename.c_str(), count, uav.EstPosition_gl.at<double>(0,0),		uav.EstPosition_gl.at<double>(1,0),		uav.EstPosition_gl.at<double>(2,0));
		uav.data.cellCvMatDoubles_multline(splitCKF.Qdk,					"    " + uav.s_data_filename + ".ckf.Qdk",					count);
		uav.data.cellCvMatDoubles_multline(splitCKF.Rk,						"    " + uav.s_data_filename + ".ckf.Rk",						count);
		uav.data.cellCvMatDoubles_multline(splitCKF.PriorEst,			"    " + uav.s_data_filename + ".ckf.PriorEst",			count);
		uav.data.cellCvMatDoubles_multline(splitCKF.PriorCov,			"    " + uav.s_data_filename + ".ckf.PriorCov",			count);
		uav.data.cellCvMatDoubles_multline(splitCKF.zk,						"    " + uav.s_data_filename + ".ckf.zk",						count);
		uav.data.cellCvMatDoubles_multline(splitCKF.H,						"    " + uav.s_data_filename + ".ckf.H",						count);
		uav.data.cellCvMatDoubles_multline(splitCKF.Kk,						"    " + uav.s_data_filename + ".ckf.Kk",						count);
		uav.data.cellCvMatDoubles_multline(splitCKF.Kkyk,					"    " + uav.s_data_filename + ".ckf.Kkyk",					count);
		uav.data.cellCvMatDoubles_multline(splitCKF.PosteriorEst,	"    " + uav.s_data_filename + ".ckf.PosteriorEst",	count);
		uav.data.cellCvMatDoubles_multline(splitCKF.PosteriorCov,	"    " + uav.s_data_filename + ".ckf.PosteriorCov",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.zk,						"    " + ugv.s_data_filename + ".ckf.zk",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.PosteriorEst,	"    " + ugv.s_data_filename + ".ckf.correction",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.Mech,					"      " + ugv.s_data_filename + ".ckf.mech.P_gl",	count);
		// ugv.data.cellCvMatDoubles(splitCKF.Aiding,				"    " + ugv.s_data_filename + ".ckf.aiding.P_gl",	count);
	}

	bool share_ckf(hast::ckfshare::Request &req, hast::ckfshare::Response &res)
	{// when this vehicle is observed by another vehicle,
		// fprintf(stdout, (RED_TEXT "  %s :: bool share_ckf(hast::ckfshare::Request &req, hast::ckfshare::Response &res) \n" COLOR_RESET), uav.s_data_filename.c_str());
		ckfshare.call_count +=1;
		std::string observing_ugv = req.stereo_obs.vehicle_id;
		fprintf(uav.data.filename,"\n%% --------------- share_ckf request --------------- %% \n");
		fprintf(uav.data.filename,"  %s.share_ckf.time(%u,:) = %-12.10f;\n",				uav.s_data_filename.c_str(), ckfshare.call_count, ros::Time::now().toSec() - init_time);
		fprintf(uav.data.filename,"  %s.share_ckf.observing_ugv{%u,1} = '%s';\n",	uav.s_data_filename.c_str(), ckfshare.call_count, observing_ugv.c_str());
		fprintf(uav.data.filename,"  %s.share_ckf.dkf_update{%u,1}    = '%s';\n",	uav.s_data_filename.c_str(), ckfshare.call_count, req.ifDKF ? "true" : "false");
		// save observation info from other vehicle
		cv::Mat observed_position = (cv::Mat_<double>(3, 1) <<
															req.stereo_obs.pose.position.x,
															req.stereo_obs.pose.position.y,
															req.stereo_obs.pose.position.z);
		cv::Mat observed_pos_cov  = (cv::Mat_<double>(3, 3) <<
															req.stereo_obs.pose.PCov.row0.x, req.stereo_obs.pose.PCov.row0.y, req.stereo_obs.pose.PCov.row0.z,
															req.stereo_obs.pose.PCov.row1.x, req.stereo_obs.pose.PCov.row1.y, req.stereo_obs.pose.PCov.row1.z,
															req.stereo_obs.pose.PCov.row2.x, req.stereo_obs.pose.PCov.row2.y, req.stereo_obs.pose.PCov.row2.z);
		double observed_yaw 		= req.stereo_obs.pose.yaw; // make sure this is in global and radians before it is sent here
		double observed_yaw_cov = req.stereo_obs.pose.yaw_cov;

		cv::Mat ugvEst_pos = (cv::Mat_<double>(3, 1) <<
															req.ugvEst.position.x,
															req.ugvEst.position.y,
															req.ugvEst.position.z);
		cv::Mat ugvEst_pos_cov  = (cv::Mat_<double>(3, 3) <<
															req.ugvEst.PCov.row0.x, req.ugvEst.PCov.row0.y, req.ugvEst.PCov.row0.z,
															req.ugvEst.PCov.row1.x, req.ugvEst.PCov.row1.y, req.ugvEst.PCov.row1.z,
															req.ugvEst.PCov.row2.x, req.ugvEst.PCov.row2.y, req.ugvEst.PCov.row2.z);
		double ugvEst_yaw 			= req.ugvEst.yaw;
		double ugvEst_yaw_cov 	= req.ugvEst.yaw_cov;

		cv::Mat ugv_Qdk = (cv::Mat_<double>(4, 4) <<
												req.ugvEst.PCov.row0.x, req.ugvEst.PCov.row0.y, req.ugvEst.PCov.row0.z, 0,
												req.ugvEst.PCov.row1.x, req.ugvEst.PCov.row1.y, req.ugvEst.PCov.row1.z, 0,
												req.ugvEst.PCov.row2.x, req.ugvEst.PCov.row2.y, req.ugvEst.PCov.row2.z, 0,
												0,0,0,req.ugvEst.yaw_cov); // the ugv doesn't directly contribute to the yaw bias

		if (ckfshare.call_count == 1){
			uav.EstYaw_gl = observed_yaw * (180 / Pi);
			uav.EstPosition_gl = (cv::Mat_<double>(3, 1) <<
																req.stereo_obs.pose.position.x,
																req.stereo_obs.pose.position.y,
																req.stereo_obs.pose.position.z);
		}

		// respond with current estimate of this vehicle's pose
		res.pose.position.x = uav.EstPosition_gl.at<double>(0,0);
		res.pose.position.y = uav.EstPosition_gl.at<double>(1,0);
		res.pose.position.z = uav.EstPosition_gl.at<double>(2,0);

		res.pose.PCov.row0.x = uav.ckf.Qdk.at<double>(0,0); res.pose.PCov.row0.y = uav.ckf.Qdk.at<double>(0,1); res.pose.PCov.row0.z = uav.ckf.Qdk.at<double>(0,2);
		res.pose.PCov.row1.x = uav.ckf.Qdk.at<double>(1,0); res.pose.PCov.row1.y = uav.ckf.Qdk.at<double>(1,1); res.pose.PCov.row1.z = uav.ckf.Qdk.at<double>(1,2);
		res.pose.PCov.row2.x = uav.ckf.Qdk.at<double>(2,0); res.pose.PCov.row2.y = uav.ckf.Qdk.at<double>(2,1); res.pose.PCov.row2.z = uav.ckf.Qdk.at<double>(2,2);

		res.pose.yaw 			= uav.EstYaw_gl * Pi / 180; // yaw should be in radians
		res.pose.yaw_cov	= uav.ckf.Qdk.at<double>(3,3);

		fprintf(uav.data.filename,"    %s.share_ckf.req.obs_yaw(%u,:)    =  %-12.10f;\n",												uav.s_data_filename.c_str(), ckfshare.call_count, observed_yaw);
		fprintf(uav.data.filename,"    %s.share_ckf.req.ugvEst_yaw(%u,:) =  %-12.10f;\n",												uav.s_data_filename.c_str(), ckfshare.call_count, ugvEst_yaw);
		fprintf(uav.data.filename,"    %s.share_ckf.req.obs_pos(%u,:)    = [%-12.10f, %-12.10f, %-12.10f];\n",	uav.s_data_filename.c_str(), ckfshare.call_count, observed_position.at<double>(0,0), observed_position.at<double>(1,0), observed_position.at<double>(2,0));
		fprintf(uav.data.filename,"    %s.share_ckf.req.ugvEst_pos(%u,:) = [%-12.10f, %-12.10f, %-12.10f];\n",	uav.s_data_filename.c_str(), ckfshare.call_count, ugvEst_pos.at<double>(0,0), ugvEst_pos.at<double>(1,0), ugvEst_pos.at<double>(2,0));
		fprintf(uav.data.filename,"    %s.share_ckf.res.EstYaw_gl(%u,:)  =  %-12.10f; %% (deg) = (%-12.10f)\n",	uav.s_data_filename.c_str(), ckfshare.call_count, res.pose.yaw, uav.EstYaw_gl);
		fprintf(uav.data.filename,"    %s.share_ckf.res.EstP_gl(%u,:)    = [%-12.10f, %-12.10f, %-12.10f];\n",	uav.s_data_filename.c_str(), ckfshare.call_count, res.pose.position.x, res.pose.position.y, res.pose.position.z);

		if (!req.ifDKF) // only update when not called from a DKF ugv
		{

			// the UAV NEVER observes the UGVs, so the only time the system updatesis when it is obsevred by UGVs
			// we'll say that the augmented state vector is [UAV; UGVn; UGVm]
			splitCKF.Qdk	= cv::Mat::zeros(12, 12, CV_64F);
	 		splitCKF.Rk		= cv::Mat::zeros( 8,  8, CV_64F);
			splitCKF.H		= cv::Mat::zeros( 8, 12, CV_64F);


			uav.ckf.Qdk.copyTo(splitCKF.Qdk.colRange(0,4).rowRange(0,4));
			cv::Mat uav_Rk = (cv::Mat_<double>(4, 4) <<
										req.stereo_obs.pose.PCov.row0.x, req.stereo_obs.pose.PCov.row0.y, req.stereo_obs.pose.PCov.row0.z, 0,
										req.stereo_obs.pose.PCov.row1.x, req.stereo_obs.pose.PCov.row1.y, req.stereo_obs.pose.PCov.row1.z, 0,
										req.stereo_obs.pose.PCov.row2.x, req.stereo_obs.pose.PCov.row2.y, req.stereo_obs.pose.PCov.row2.z, 0,
										0,0,0, req.ugvEst.yaw_cov);

			if (req.stereo_obs.vehicle_id.compare("ugv1") == 0) {
				// splitCKF.H		=[ -I  I  0
				//									0  0  0]
				 I4.copyTo(splitCKF.H.rowRange(0,4).colRange(4,8));
				nI4.copyTo(splitCKF.H.rowRange(0,4).colRange(0,4));

				// splitCKF.Qdk		=[	Quav 0     0
				//										0    Qugv1 0
				//										0    0     0 ]
				ugv_Qdk.copyTo(splitCKF.Qdk.colRange(4,8).rowRange(4,8));

				// splitCKF.Rk		=[ Rkugv1 0
				//									 0      Rkinf]
				uav_Rk.copyTo( splitCKF.Rk.rowRange(0,4).colRange(0,4));
				splitCKF.Rk_inf.copyTo(splitCKF.Rk.rowRange(4,8).colRange(4,8)); // ugv2 stereo measurement

				// splitCKF.zk = (cv::Mat_<double>(8, 1) <<
				// 							req.stereo_obs.pose.position.x - (uav.EstPosition_gl.at<double>(0,0) - req.ugvEst.position.x),
				// 							req.stereo_obs.pose.position.y - (uav.EstPosition_gl.at<double>(1,0) - req.ugvEst.position.y),
				// 							req.stereo_obs.pose.position.z - (uav.EstPosition_gl.at<double>(2,0) - req.ugvEst.position.z),
				// 							req.stereo_obs.pose.yaw - ((uav.EstYaw_gl)*(Pi/180) - req.ugvEst.yaw),
				// 							0,0,0,0);

				splitCKF.zk = (cv::Mat_<double>(8, 1) <<
											req.stereo_obs.pose.position.x - ((uav.EstPosition_gl.at<double>(0,0) + uav.correction_gl.at<double>(0,0)) - req.ugvEst.position.x),
											req.stereo_obs.pose.position.y - ((uav.EstPosition_gl.at<double>(1,0) + uav.correction_gl.at<double>(1,0)) - req.ugvEst.position.y),
											req.stereo_obs.pose.position.z - ((uav.EstPosition_gl.at<double>(2,0) + uav.correction_gl.at<double>(2,0)) - req.ugvEst.position.z),
											req.stereo_obs.pose.yaw - ((uav.EstYaw_gl + uav.yawCorrection)*(Pi/180) - req.ugvEst.yaw),
											0,0,0,0);


			}

			if (req.stereo_obs.vehicle_id.compare("ugv2") == 0) {
				// splitCKF.H		=[	0  0  0
				//								 -I  0  I]
				 I4.copyTo(splitCKF.H.rowRange(4,8).colRange(8,12));
				nI4.copyTo(splitCKF.H.rowRange(4,8).colRange(0,4));

				// splitCKF.Qdk		=[	Quav 0  0
				//										0    0  0
				//										0    0  Qugv2 ]
				ugv_Qdk.copyTo(splitCKF.Qdk.colRange(8,12).rowRange(8,12));

				// splitCKF.Rk		=[ Rkinf 0
				//									 0     Rkugv2]
				uav_Rk.copyTo( splitCKF.Rk.rowRange(4,8).colRange(4,8));
				splitCKF.Rk_inf.copyTo(splitCKF.Rk.rowRange(0,4).colRange(0,4)); // ugv2 stereo measurement

				splitCKF.zk = (cv::Mat_<double>(8, 1) <<
											0,0,0,0,
											req.stereo_obs.pose.position.x - ((uav.EstPosition_gl.at<double>(0,0) + uav.correction_gl.at<double>(0,0)) - req.ugvEst.position.x),
											req.stereo_obs.pose.position.y - ((uav.EstPosition_gl.at<double>(1,0) + uav.correction_gl.at<double>(1,0)) - req.ugvEst.position.y),
											req.stereo_obs.pose.position.z - ((uav.EstPosition_gl.at<double>(2,0) + uav.correction_gl.at<double>(2,0)) - req.ugvEst.position.z),
											req.stereo_obs.pose.yaw - ((uav.EstYaw_gl + uav.yawCorrection)*(Pi/180) - req.ugvEst.yaw));

			}


			fprintf(uav.data.filename,"\n%% --------------- ckf incrementKF  %% \n");
			Qdk_CVtrace = cv::trace(splitCKF.Qdk);
			fprintf(uav.data.filename,"  %% %s.ckf.Qdk_CVtrace = %6.14f;\n",	uav.s_data_filename.c_str(), Qdk_CVtrace[0]);
			// if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
			if (!(Qdk_CVtrace[0] < 0.0001) )
			{
				splitCKF.zKF();
				log_ckf(splitCKF.counter);
				uav.updatePoseFromCKF((cv::Mat_<double>(3, 1) << splitCKF.PosteriorEst.at<double>(0,0), splitCKF.PosteriorEst.at<double>(1,0), splitCKF.PosteriorEst.at<double>(2,0)), splitCKF.PosteriorEst.at<double>(3,0), 0);
			}
		}

		return true;
	}

	void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags)
	{
		tagArray = tags->detections;
		tagArraySize = tagArray.size();

		if (tagArraySize>0)
		{
			tag_counter++;
			double tagTime = ros::Time::now().toSec();
			fprintf(uav.data.filename,"\n%% --------------- tag Message --------------- %% \n");
			fprintf(uav.data.filename,"  %s.april.time(%u,:) = %6.4f;\n"																						, uav.s_data_filename.c_str(), tag_counter, tagTime-init_time);
			fprintf(uav.data.filename,"    %s.april.ArraySize(%u,:) = %i;\n"																				, uav.s_data_filename.c_str(), tag_counter, tagArraySize);

			int id;
			for(int k = 0; k != tagArraySize; k++)
			{
				// fprintf(mfile.filename, "  %% --------------- tagDetections --------------- %% \n");
				// set marker ids
				id = tagArray[k].id;
				// marker time stamp
				tagStamp = tagArray[k].pose.header.stamp;
				tagTSpublished = tagStamp.toSec();

				april[id].MeasPosition_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z, 1);
				april[id].MeasQuaternion_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.orientation.x,  tagArray[k].pose.pose.orientation.y, tagArray[k].pose.pose.orientation.z, tagArray[k].pose.pose.orientation.w);

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
				april[id].MeasPosition_uav = (cv::Mat_<double>(4, 1) << Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z(), 1);
				april[id].MeasPosition_gl = uav.Hlo2gl * april[id].MeasPosition_uav;
				// april[id].MeasRelUAV_gl = - uav.Htag2gl * april[id].MeasPosition_uav; //used for fullSLAM measurement of tag
				april[id].MeasYaw_uav = april[id].MeasYaw_uav_rad; //stay in radians where possible
				april[id].MeasYaw_gl = april[id].MeasYaw_uav + Pi*(uav.EstYaw_gl)/180; //stay in radians where possible

				if (april[id].dkf.counter == 0)
				{ // then this is the first update, just use the measurement as the estimate
					april[id].dkf.PosteriorEst = (cv::Mat_<double>(4, 1) <<
					april[id].MeasPosition_gl.at<double>(0, 0),
					april[id].MeasPosition_gl.at<double>(1, 0),
					april[id].MeasPosition_gl.at<double>(2, 0),
					april[id].MeasYaw_gl);
				} else {
					april[id].dkf.zk = (cv::Mat_<double>(4, 1) <<
					april[id].MeasPosition_gl.at<double>(0, 0),
					april[id].MeasPosition_gl.at<double>(1, 0),
					april[id].MeasPosition_gl.at<double>(2, 0),
					april[id].MeasYaw_gl);

					april[id].dkf.incrementKF();
				}

				april[id].EstPosition_gl = (cv::Mat_<double>(3, 1) <<
				april[id].dkf.PosteriorEst.at<double>(0, 0),
				april[id].dkf.PosteriorEst.at<double>(1, 0),
				april[id].dkf.PosteriorEst.at<double>(2, 0));

				april[id].EstYaw_gl = april[id].dkf.PosteriorEst.at<double>(3, 0);
			}

			fprintf(uav.data.filename,"    %% uav.EstYaw_gl = %6.14f;\n", Pi* (uav.EstYaw_gl)/180);
			for(int k = 0; k != tagArraySize; k++)
			{
				id = tagArray[k].id;
				s_markerid = "tag_" + patch::to_string(id);

				fprintf(uav.data.filename,"    %s.april.%s.time(%u,:) = % -6.14f;\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].measCount, tagTSpublished - init_time);
				fprintf(uav.data.filename,"      %s.april.%s.EstYaw_gl(%u,:)   = %6.14f;\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].dkf.counter-1, april[id].EstYaw_gl);
				fprintf(uav.data.filename,"      %s.april.%s.MeasYaw_gl(%u,:)   = %6.14f;\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_gl);
				fprintf(uav.data.filename,"      %s.april.%s.MeasYaw_uav(%u,:)  = %6.14f;\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_uav);
				fprintf(uav.data.filename,"      %s.april.%s.EstPosition_gl(%u,:)  = [%12.10f, %12.10f, %12.10f];\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].dkf.counter-1, april[id].EstPosition_gl.at<double>(0,0), april[id].EstPosition_gl.at<double>(1,0), april[id].EstPosition_gl.at<double>(2,0));
				fprintf(uav.data.filename,"      %s.april.%s.MeasPosition_gl(%u,:)  = [%12.10f, %12.10f, %12.10f];\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].measCount, april[id].MeasPosition_gl.at<double>(0,0), april[id].MeasPosition_gl.at<double>(1,0), april[id].MeasPosition_gl.at<double>(2,0));
				fprintf(uav.data.filename,"      %s.april.%s.MeasPosition_uav(%u,:) = [%12.10f, %12.10f, %12.10f];\n", uav.s_data_filename.c_str(), s_markerid.c_str(), april[id].measCount, april[id].MeasPosition_uav.at<double>(0,0), april[id].MeasPosition_uav.at<double>(1,0), april[id].MeasPosition_uav.at<double>(2,0));
			}  // end for(uint i = 0; i != updatingTags; i++)
		}

	}

	bool serveState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res)
	{
		fprintf(stdout, (RED_TEXT "%s :: bool serveState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res) \n" COLOR_RESET), uav.s_data_filename.c_str());
		res.state = uav.flightState; return true;
	}

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
	ros::init(argc, argv, "uavCKF");
	uavCKF cfkRec;
	ros::spin();
	return 0;
}
