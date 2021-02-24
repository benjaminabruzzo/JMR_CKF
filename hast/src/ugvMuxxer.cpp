#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "ckfClass.hpp"
// #include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"

// s_markerid = "tag_" + patch::to_string(newLandmarkID);
namespace patch
{
	template < typename T > std::string to_2string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
	template < typename T > std::string to_1string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";

class ugvMuxxer
{
	private:
		/*---------  File Recorder ------------- */
			// fileops mfile;
			std::string s_trial, s_dotm, s_root, s_date, s_user, s_ugvn, s_exp_code;
			double Pi, L2Norm;
			int i_ugvn;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
				ros::Subscriber stereoMeas_sub, HastShutDown_sub;
				std::string s_shutdown_topic;
				int stereoUGVcount;

			bool primaryUGV, gazebo;
			bool slambool, dkfbool, ckfbool;


			hastUGV ugv; // create ugv from class
			double ugv_Qw_scale;

		/*----- Stereo Variables */
			cv::Mat stereoObs_P, stereoObs_P_global, stereoObs_PCov;
			double stereoObs_yaw_cov;
			uint stereoObs_counter;
			bool stereo_data;
			ros::Time stereo_time_rx;

			std::string s_Meas_sub, s_ugvn_state, s_obstacle_cloud_topic, s_goal_cloud_topic;

			ros::Publisher ugv_mux_pub;
				hast::ugvmux ugv_mux_msg;
				std::string s_ugv_mux_topic;

			ros::Subscriber dkf_sub;
				std::string s_dkf_topic;

			ros::Subscriber slam_poses_sub, ugv_pose_sub;
				std::string s_slam_poses_topic, s_ugv_pose_topic;
				std::vector<hast::posewithcov> ugvArray; //vector of ugvs in slam system
				uint ugvArraySize; // number of ugvs in slam system

		cv::Mat EstPosition_dkf, EstPosition_ckf, EstPosition_slam;
		double EstYaw_dkf, EstYaw_ckf, EstYaw_slam;
		double msgTime_dkf, msgTime_ckf, msgTime_slam;
		uint slam_count;
		bool ckf_update, dkf_update;
		tf::Quaternion dkf_q;
		geometry_msgs::Quaternion dkf_q_msg;

		// Ground truth hack
		tf::TransformListener listener;
		bool absolute_measurements;
		bool absolute_init;

	public:

		ros::ServiceClient callTime_cli; // client to request master ros time offset
			hast::servtime time_call;
			std::string calling_node;

		double init_time;

		ros::Time pose_time;
		int pose_count;

		hast::posewithcov slampose_msg;

		// map tf broadcaster
		tf::TransformBroadcaster ugv_map_bc;
		tf::Transform map_tf;
		tf::Matrix3x3 ugvR_TF;
		tf::Quaternion ugvQ_TF;

		ros::ServiceServer MuxMapService_ser;
			std::string ugv_map_mux_topic, ugv_map_root_topic, s_ugv_mux_map_serve_topic;

		struct killtopics
		{
			ros::ServiceServer ugv_running_ser;
			std::string s_ugv_running_ser;
		};
		killtopics endTrial;


		struct driver_status_struct
		{
			std::string topic;
			ros::Publisher pub;
			ros::Subscriber sub;
			hast::ugvmuxstatus msg;
			bool local_cmd_override;
			double yaw_threshold;
		};
		driver_status_struct driver_status;

		struct ugv_pose_struct
		{
			std::string topic;
			ros::ServiceServer service;
			ros::ServiceClient client;
			hast::ugvdrive call;
			cv::Mat position;
			double yaw;
		};
		ugv_pose_struct driver_goal, init_hack;

		struct geometry_msgs_twist_struct
		{
			std::string topic;
			ros::Publisher pub;
			ros::Subscriber sub;
			geometry_msgs::Twist msg;
		};
		geometry_msgs_twist_struct move_base_cmd_vel, driver_cmd_vel;

		ros::Time node_start_time;

	ugvMuxxer()
	{
		node_start_time = ros::Time::now();
		/*---------  File Recorder Initilizer ------------- */
		ros::param::get("~ugv_n",s_ugvn);

		ros::param::get("~ugv_data_filename",ugv.s_data_filename);

		ros::param::get("~gazebo", gazebo);
		n.getParam("/hast/user",  s_user);
		n.getParam("/hast/trial", s_trial);
		n.getParam("/hast/date",  s_date);
		n.getParam("/hast/exp_code", s_exp_code);

		// if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}
		// if (n.getParam("/hast/init_time", init_time)){
		// 	callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
		// 	time_call.request.requesting_node = ugv.s_data_filename;
		// 	ros::Time calltime = ros::Time::now();
		// 	callTime_cli.call(time_call);
		// 	ros::Time mastertime = time_call.response.header.stamp;
		// 	if (gazebo){
		// 		init_time = 0.0;
		// 	} else {
		// 		ROS_INFO("%s::init_time = %f", time_call.request.requesting_node.c_str(), init_time+=(calltime.toSec() - mastertime.toSec()));
		// 	}
		// } else {init_time = 0.0;}

		if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}
		ugv.init_time = init_time;

		/*--------- Math Constants ------------- */
		L2Norm = 4; // Frobenius norm for CV norm() function
		Pi  = atan(1)*4; // 3.14159...

		ugv.data.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/" + ugv.s_data_filename + "_" + s_trial + ".m");

		if (s_ugvn.compare("ugv1") == 0){i_ugvn = 1;}
		if (s_ugvn.compare("ugv2") == 0){i_ugvn = 2;}

		ugv.s_filename = "/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/" + ugv.s_data_filename + "_" + s_trial + ".m";
		ugv.s_prealloc = "/home/" + s_user + "/ros/data/" + s_date + "/" + s_exp_code + "/" + s_trial + "/prealloc/pre_" + ugv.s_data_filename + "_" + s_trial + ".m";
		ugv.s_ugvn = s_ugvn+"_mux";
		// ROS_INFO("~~~~~~~~ MUX : opening %s ", ugv.s_data_filename.c_str());
		ugv.openmFile();
		fprintf (ugv.data.filename, "\n%% --------------- Load Params --------------- \n" );

		ros::param::get("~Qw_scale",ugv.Qw_scale); fprintf (ugv.data.filename, "  %s.topics.Qw_scale = %4.6f;\n", ugv.s_data_filename.c_str(), ugv.Qw_scale);
		ugv.ekf.Qw = ugv.Qw_scale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

		ros::param::get("~primary",primaryUGV); //	ROS_WARN(":~:~:~:~:~:~:~: ugv_n.primary :: %s.%s", s_ugvn.c_str(), primaryUGV ? "true" : "false" );
		if(primaryUGV){
			// do nothing
			EstYaw_dkf = 0;
			EstPosition_dkf = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		} else {
			ros::param::get("~dkf_topic", ugv.s_dkf_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
				fprintf (ugv.data.filename, "  %s.topics.dkf_topic = ['%s'];\n", ugv.s_data_filename.c_str(), ugv.s_dkf_topic.c_str());
				ugv.dkf_sub = n.subscribe(ugv.s_dkf_topic,	1, &ugvMuxxer::DKFread , this);
		}

		ros::param::get("~ugv_mux_topic",s_ugv_mux_topic); //mfile.writeString("%% ugv_mux_topic: " + s_ugv_mux_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.ugv_mux_topic = ['%s'];\n", ugv.s_data_filename.c_str(), s_ugv_mux_topic.c_str());
			ugv_mux_pub = n.advertise<hast::ugvmux>(s_ugv_mux_topic, 1);
			ugv_mux_msg.ugvEst.vehicle_id = s_ugvn;
			ugv_mux_msg.seq = 1;
			ugv_mux_msg.ugvEst.PCov.row0.x = 0; ugv_mux_msg.ugvEst.PCov.row0.y = 0; ugv_mux_msg.ugvEst.PCov.row0.z = 0;
			ugv_mux_msg.ugvEst.PCov.row1.x = 0; ugv_mux_msg.ugvEst.PCov.row1.y = 0; ugv_mux_msg.ugvEst.PCov.row1.z = 0;
			ugv_mux_msg.ugvEst.PCov.row2.x = 0; ugv_mux_msg.ugvEst.PCov.row2.y = 0; ugv_mux_msg.ugvEst.PCov.row2.z = 0;

		ros::param::get("~shutdown_topic",s_shutdown_topic); //mfile.writeString("%% s_shutdown_topic: " + s_shutdown_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.shutdown_topic = ['%s'];\n", ugv.s_data_filename.c_str(), s_shutdown_topic.c_str());
			HastShutDown_sub 	= n.subscribe(s_shutdown_topic,	1, &ugvMuxxer::nodeShutDown, this);

		ros::param::get("~stereoMeas_sub",s_Meas_sub); //mfile.writeString("%% stereoMeas_sub: " + s_Meas_sub + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.stereoMeas_sub = ['%s'];\n", ugv.s_data_filename.c_str(), s_Meas_sub.c_str());
			stereoMeas_sub 		= n.subscribe(s_Meas_sub, 1, &ugvMuxxer::stereoRead , this);
			stereoUGVcount = 0;

		ros::param::get("~odom_topic", ugv.s_wheel_odom_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.odom_topic = ['%s'];\n", ugv.s_data_filename.c_str(), ugv.s_wheel_odom_topic.c_str());
			ugv.WheelOdom_sub = n.subscribe(ugv.s_wheel_odom_topic,	5, &hastUGV::WheelOdometry, &ugv);

		// advertised location of ugv given previous updates and odometry
		ros::param::get("~ugv_state_pub_topic", ugv.s_state_pub_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.state_pub_topic = ['%s'];\n", ugv.s_data_filename.c_str(), ugv.s_state_pub_topic.c_str());
			ugv.s_base_footprint_topic = "/hast/ugv1_mux/base_footprint";
			ugv.state_pub = n.advertise<hast::ugvstate>(ugv.s_state_pub_topic, 1);

		// pose read by by ckf method
		ros::param::get("~ugv_pose_topic",s_ugv_pose_topic); //mfile.writeString("%% ugv_pose_topic: " + s_ugv_pose_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.ugv_pose_topic = ['%s'];\n", ugv.s_data_filename.c_str(), s_ugv_pose_topic.c_str());
			ugv_pose_sub		= n.subscribe(s_ugv_pose_topic, 1, &ugvMuxxer::CKFread , this);

		// pose read by slam method
		ros::param::get("~slam_poses_topic",s_slam_poses_topic); //mfile.writeString("%% slam_poses_topic: " + s_slam_poses_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.slam_poses_topic = ['%s'];\n", ugv.s_data_filename.c_str(), s_slam_poses_topic.c_str());
			slam_poses_sub	= n.subscribe(s_slam_poses_topic, 1, &ugvMuxxer::SLAMposes , this);
			fprintf (ugv.data.filename, "\n");
			slam_count = 0;

		ros::param::get("~ugv_map_mux_topic", ugv_map_mux_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.ugv_map_mux_topic = ['%s'];\n", ugv.s_data_filename.c_str(), ugv_map_mux_topic.c_str());

		ros::param::get("~ugv_map_root_topic", ugv_map_root_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
			fprintf (ugv.data.filename, "  %s.topics.ugv_map_root_topic = ['%s'];\n", ugv.s_data_filename.c_str(), ugv_map_root_topic.c_str());

		ros::param::get("~ugv_mux_map_serve_topic", s_ugv_mux_map_serve_topic); //mfile.writeString("%% odom_topic: " + ugv.s_wheel_odom_topic + "\n" );
			MuxMapService_ser  	= n.advertiseService(s_ugv_mux_map_serve_topic, &ugvMuxxer::muxMapServer , this);

		ros::param::get("~ugv_running_topic", endTrial.s_ugv_running_ser); fprintf (ugv.data.filename, "  %s.topics.ugv_running_topic = ['%s'];\n", ugv.s_data_filename.c_str(), endTrial.s_ugv_running_ser.c_str());
			endTrial.ugv_running_ser 		= n.advertiseService(endTrial.s_ugv_running_ser, 	&ugvMuxxer::is_running,  this);

		ros::param::get("~move_base_cmd_topic", move_base_cmd_vel.topic); fprintf (ugv.data.filename, "  %s.topics.move_base_cmd_topic = ['%s'];\n", ugv.s_data_filename.c_str(), move_base_cmd_vel.topic.c_str());
			move_base_cmd_vel.sub		= n.subscribe(move_base_cmd_vel.topic, 1, &ugvMuxxer::move_base_cmd_vel_read , this);

		ros::param::get("~driver_cmd_topic", driver_cmd_vel.topic); fprintf (ugv.data.filename, "  %s.topics.driver_cmd_topic = ['%s'];\n", ugv.s_data_filename.c_str(), driver_cmd_vel.topic.c_str());
			driver_cmd_vel.pub = n.advertise<geometry_msgs::Twist>(driver_cmd_vel.topic, 1);
			driver_cmd_vel.msg.linear.x = 0;  driver_cmd_vel.msg.linear.y = 0;  driver_cmd_vel.msg.linear.z = 0;
			driver_cmd_vel.msg.angular.x = 0; driver_cmd_vel.msg.angular.y = 0; driver_cmd_vel.msg.angular.z = 0;

		// topic for service that sets a goal pose for the ugv
		ros::param::get("~driver_goal_topic", driver_goal.topic); 	fprintf (ugv.data.filename, "  %s.topics.driver_goal_topic = ['%s'];\n", ugv.s_data_filename.c_str(), driver_goal.topic.c_str());
				driver_goal.service 		= n.advertiseService(driver_goal.topic, 	&ugvMuxxer::set_desired,  this);

		// topic for publishing ugv driver status
		ros::param::get("~driver_status_topic", driver_status.topic); 	fprintf (ugv.data.filename, "  %s.topics.driver_status_topic = ['%s'];\n", ugv.s_data_filename.c_str(), driver_status.topic.c_str());
				driver_status.pub 		= n.advertise<hast::ugvmuxstatus>(driver_status.topic, 1);
				driver_status.local_cmd_override = false;
				driver_status.yaw_threshold = 0.025;
				driver_status.msg.header.seq = 0;
				driver_status.msg.header.frame_id = driver_status.topic;
				driver_status.msg.header.stamp = ros::Time::now();
				driver_status.msg.at_yaw = false;
				driver_status.msg.at_pos = false;

		/*----- dual-purpose stereo message variables */
		pose_count = 0;
		stereoObs_counter = 0;
		stereoObs_P = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

		ros::param::get("~absolute_init",absolute_init);
				ROS_WARN(":~:~:~:~:~:~:~:      %s.absolute_init :: %s",								ugv.s_data_filename.c_str(), absolute_init ? "true" : "false" );
				fprintf (ugv.data.filename, "  %s.topics.absolute_init = ['%s'];\n",	ugv.s_data_filename.c_str(), absolute_init ? "true" : "false" );
		ros::param::get("~absolute_measurements",absolute_measurements);
				ROS_WARN(":~:~:~:~:~:~:~:      %s.absolute_measurements :: %s", 							ugv.s_data_filename.c_str(), absolute_measurements ? "true" : "false" );
				fprintf (ugv.data.filename, "  %s.topics.absolute_measurements = ['%s'];\n",	ugv.s_data_filename.c_str(), absolute_measurements ? "true" : "false" );
		fprintf (ugv.data.filename, "  %s.params.init_time = % -6.14f;\n", ugv.s_data_filename.c_str(), init_time);
		fprintf (ugv.data.filename, "  %s.params.node_start_time = % -6.14f;\n", ugv.s_data_filename.c_str(), ros::Time::now().toSec());
	}

	void move_base_cmd_vel_read(const geometry_msgs::Twist::ConstPtr& msg)
	{
		driver_status.local_cmd_override = false;
		//right now, just pass through the message until I implement the local autopilot
		driver_cmd_vel.pub.publish(msg);
	}

	bool is_running(hast::servtime::Request &req, hast::servtime::Response &res)
	{
		// res.header.seq = 0;
		// res.header.frame_id = s_ugvn;
		// res.header.stamp = ros::Time::now();

		// res.header.seq = 0;
		res.sync_msg.frame_id = s_ugvn;
		res.sync_msg.node_start = node_start_time.toSec();
		res.sync_msg.now = ros::Time::now();


		return true;
	}

	bool set_desired(hast::ugvdrive::Request &req, hast::ugvdrive::Response &res)
	{
		driver_goal.position = (cv::Mat_<double>(3, 1) <<req.goalP.x, req.goalP.y, req.goalP.z);
		driver_goal.yaw = req.goalYaw;
		driver_status.local_cmd_override = true;

		// res.at_yaw = false; res.at_pos = false;
		driver_status.msg.at_yaw = false; driver_status.msg.at_pos = false;
		driver_status.msg.header.stamp = ros::Time::now();
		driver_status.pub.publish(driver_status.msg);

		// ROS_INFO(" MUX:: set_desired()");
		// ROS_INFO(" MUX:: driver_status.msg.at_yaw : '%s';", driver_status.msg.at_yaw ? "true" : "false" );
		// ROS_INFO(" MUX:: driver_status.msg.at_pos : '%s';", driver_status.msg.at_pos ? "true" : "false" );
		// ROS_INFO(" MUX::              req.goalYaw : %6.4f", req.goalYaw);
		// ROS_INFO(" MUX::          driver_goal.yaw : %6.4f", driver_goal.yaw);
		// ROS_INFO(" MUX::                req.goalP : [%6.4f, %6.4f, %6.4f]", req.goalP.x, req.goalP.y, req.goalP.z);
		// ROS_INFO(" MUX::     driver_goal.position : [%6.4f, %6.4f, %6.4f]", driver_goal.position.at<double>(0,0), driver_goal.position.at<double>(1,0), driver_goal.position.at<double>(2,0) );

		return true;
	}

	bool muxMapServer(hast::servmuxmap::Request &req, hast::servmuxmap::Response &res)
	{
		dkf_q =	ToQuaternion(EstYaw_dkf, 0.0, 0.0); quaternionTFToMsg(dkf_q , dkf_q_msg);
		res.map_pose.orientation = dkf_q_msg;
		res.map_pose.position.x = EstPosition_dkf.at<double>(0,0);
		res.map_pose.position.y = EstPosition_dkf.at<double>(1,0);
		res.map_pose.position.z = EstPosition_dkf.at<double>(2,0);
		res.map_yaw = EstYaw_dkf;


		fprintf (ugv.data.filename, "\n%% --------------- muxMapServer --------------- \n" );
		fprintf (ugv.data.filename, "  %s.map_pose.position = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), res.map_pose.position.x, res.map_pose.position.y, res.map_pose.position.z);
		fprintf (ugv.data.filename, "  %s.map_pose.orientation = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), res.map_pose.orientation.x, res.map_pose.orientation.y, res.map_pose.orientation.z, res.map_pose.orientation.w);


		return true;
	}

	void updatePose()
	{
		if (slam_count <1){ // use c/dkf for position
			if(ckf_update){
				EstPosition_ckf.copyTo(ugv.EstPosition_gl);
				ugv.EstYaw_gl = EstYaw_ckf;
				ugv.lastWheelTime = msgTime_ckf;
			}
			if(dkf_update){
				EstPosition_dkf.copyTo(ugv.EstPosition_gl);
				ugv.EstYaw_gl = EstYaw_dkf;
				ugv.lastWheelTime = msgTime_dkf;
			}
		} else {//use slam for position
			if (stereo_data)
			{
				EstPosition_slam.copyTo(ugv.EstPosition_gl);
				ugv.EstYaw_gl = EstYaw_slam;
				ugv.lastWheelTime = msgTime_slam;
				ugv.odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0,0);
				stereo_data = false;
			} else {
				// Do nothing, keep using odom as the basis
			}
		}
		pose_time = ros::Time::now();
		fprintf (ugv.data.filename, "\n%% --------------- pose update --------------- \n" );
		fprintf (ugv.data.filename, "  %s.pose.time(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), ++pose_count, pose_time.toSec() - init_time);
		fprintf (ugv.data.filename, "  %s.pose.yaw_gl(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_count, ugv.EstYaw_gl);
		fprintf (ugv.data.filename, "  %s.pose.position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), pose_count, ugv.EstPosition_gl.at<double>(0,0), ugv.EstPosition_gl.at<double>(1,0), ugv.EstPosition_gl.at<double>(2,0));

		// if (!gazebo){
			if(primaryUGV)
			{
				// do nothing
				map_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
				map_tf.setRotation(ToQuaternion(0.0, 0.0, 0.0));
			} else {
				// broadcast tf based on last DKF measurement
				map_tf.setOrigin( tf::Vector3(EstPosition_dkf.at<double>(0,0), EstPosition_dkf.at<double>(1,0), EstPosition_dkf.at<double>(2,0)));
				map_tf.setRotation(ToQuaternion(EstYaw_dkf, 0.0, 0.0));
			}
			ugv_map_bc.sendTransform(tf::StampedTransform(map_tf, pose_time, ugv_map_root_topic, ugv_map_mux_topic));
		// }

		if (driver_status.local_cmd_override)
		{// then run local control loop
			rotate_to_goal();
		}
	}

	void SLAMposes(const hast::slam_poses::ConstPtr& slam_msg)
	{
		if (slam_msg->header.seq != 0)
		{
			// ROS_INFO("SLAMposes");
			ros::Time msg_time = slam_msg->header.stamp;
			hast::posewithcov ugv_pose;
			ugv_pose = slam_msg->ugv_poses[i_ugvn - 1];
			fprintf (ugv.data.filename, "\n%% --------------- SLAMposes --------------- \n" );
			fprintf (ugv.data.filename, "  %s.slam.time(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), slam_msg->header.seq, msg_time.toSec() - init_time);
			fprintf (ugv.data.filename, "  %s.slam.yaw_gl(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), slam_msg->header.seq, ugv_pose.yaw);
			fprintf (ugv.data.filename, "  %s.slam.position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), slam_msg->header.seq, ugv_pose.position.x, ugv_pose.position.y, ugv_pose.position.z);

			EstPosition_slam = (cv::Mat_<double>(3, 1) << ugv_pose.position.x, ugv_pose.position.y, ugv_pose.position.z);
			EstYaw_slam = ugv_pose.yaw;
			msgTime_slam = msg_time.toSec();
			slam_count = slam_msg->header.seq;

			if (ugv_pose.observedByUGV) {
				// reset odom delta because ugv was observed by other ugv
				ugv.odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0,0);
			}

			updatePose();
		}
	}

	void rotate_to_goal()
	{
			// ROS_INFO(" MUX:: rotate_to_goal()");
			// ROS_INFO(" MUX::                  driver_status.msg.at_yaw: '%s';", driver_status.msg.at_yaw ? "true" : "false" );
			// ROS_INFO(" MUX::                  driver_status.msg.at_pos: '%s';", driver_status.msg.at_pos ? "true" : "false" );
			// ROS_INFO(" MUX::                             ugv.EstYaw_gl: %6.4f", ugv.EstYaw_gl);
			// ROS_INFO(" MUX::                           driver_goal.yaw: %6.4f", driver_goal.yaw);
			// ROS_INFO(" MUX:: std::abs(ugv.EstYaw_gl - driver_goal.yaw): %6.4f", std::abs(ugv.EstYaw_gl - driver_goal.yaw));
			// ROS_INFO(" MUX::               driver_status.yaw_threshold: %6.4f\n", driver_status.yaw_threshold);

		driver_status.msg.header.seq += 1;

		// if angle below threshold
		if (std::abs(ugv.EstYaw_gl - driver_goal.yaw)<driver_status.yaw_threshold)
		{
			// signal @ goal yaw status
			driver_status.msg.at_yaw = true;
			driver_cmd_vel.msg.angular.x = 0; driver_cmd_vel.msg.angular.y = 0; driver_cmd_vel.msg.angular.z = 0;
		} else {
			driver_status.msg.at_yaw = false;
			double v[20] = {-0.25, -0.2, -0.15, -0.1, -0.05,
											 0.05,  0.1,  0.15,  0.2,  0.25}; // set of possible angular velocities (rad/sec)
			int n = 10; // size of v?
			double dt = 0.25; // time into the future to look (sec)

			double v_opt = 0; // optimal velocity to use
			double future_angle = ugv.EstYaw_gl; // assume no motion of ugv
			double future_error = future_angle - driver_goal.yaw;

			// for {set of angular velocities}
			// ROS_WARN("\n  MUX :: v_0 = % 6.4f, future_angle = % 6.4f, future_error = % 6.4f", v_opt, future_angle, future_error);
			for(int i=0;i<n;i++)
			{
				// calculate future body angle
				future_angle = ugv.EstYaw_gl + dt * v[i];
				// compare future angle to goal angle
				// printf(    "       v[%i] = % 6.4f, future_angle = % 6.4f, future_error = % 6.4f\n", i, v[i], ugv.EstYaw_gl + dt * v[i], (future_angle - driver_goal.yaw));
				if( std::abs(future_angle - driver_goal.yaw) < std::abs(future_error))
				{
					// choose velocity which has smallest future angle error
					v_opt=v[i];
					future_error = future_angle - driver_goal.yaw;
				}
			}
			driver_cmd_vel.msg.angular.x = 0; driver_cmd_vel.msg.angular.y = 0; driver_cmd_vel.msg.angular.z = v_opt;

		}
		// publish that velocity
		driver_cmd_vel.msg.linear.x = 0;  driver_cmd_vel.msg.linear.y = 0;  driver_cmd_vel.msg.linear.z = 0;
		driver_cmd_vel.pub.publish(driver_cmd_vel.msg);

		driver_status.msg.header.stamp = ros::Time::now();
		driver_status.pub.publish(driver_status.msg);
	}

	tf::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
	{
	    // Abbreviations for the various angular functions
	    double cy = cos(yaw * 0.5);
	    double sy = sin(yaw * 0.5);
	    double cp = cos(pitch * 0.5);
	    double sp = sin(pitch * 0.5);
	    double cr = cos(roll * 0.5);
	    double sr = sin(roll * 0.5);

			tf::Quaternion q(
				cy * cp * sr - sy * sp * cr,
				sy * cp * sr + cy * sp * cr,
				sy * cp * cr - cy * sp * sr,
				cy * cp * cr + sy * sp * sr);

	    return q;
	}

	void DKFread(const hast::posewithcov::ConstPtr& pose_msg)
	{
		if (pose_msg->header.seq != 0)
		{
			// ROS_INFO("DKFread");
			ros::Time msg_time = pose_msg->header.stamp;
			fprintf (ugv.data.filename, "\n%% --------------- dkf pose --------------- \n" );
			fprintf (ugv.data.filename, "  %s.dkf.time(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, msg_time.toSec()-init_time);
			fprintf (ugv.data.filename, "  %s.dkf.yaw_gl(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, pose_msg->yaw);
			fprintf (ugv.data.filename, "  %s.dkf.position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, pose_msg->position.x, pose_msg->position.y, pose_msg->position.z);

			EstPosition_dkf = (cv::Mat_<double>(3, 1) << pose_msg->position.x,  pose_msg->position.y, pose_msg->position.z);
			EstYaw_dkf = pose_msg->yaw;

			if(absolute_init){
				/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
						tf_getUGVpose();
						EstPosition_dkf = (cv::Mat_<double>(3, 1) << init_hack.position.at<double>(0,0), init_hack.position.at<double>(1,0), init_hack.position.at<double>(2,0));
						EstYaw_dkf = init_hack.yaw;
						fprintf (ugv.data.filename, "  %s.dkf.hack_yaw_gl(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, init_hack.yaw);
						fprintf (ugv.data.filename, "  %s.dkf.hack_pos_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, init_hack.position.at<double>(0,0), init_hack.position.at<double>(1,0), init_hack.position.at<double>(2,0));

				/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
			}

			msgTime_dkf = msg_time.toSec();
			ckf_update = false; dkf_update = true;
			updatePose();
		}
	}

	void tf_getUGVpose()
	{
		tf::StampedTransform StampedXform_gl2ugv;
		try {listener.lookupTransform("/global", "/vicon/"+ s_ugvn +"/base_TF", ros::Time(0), StampedXform_gl2ugv);}
			catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/origin to /vicon/%s/base_TF is missing", s_ugvn.c_str());}
			double qroll, qpitch, ugv_yaw;
			tf::Matrix3x3(StampedXform_gl2ugv.getRotation()).getRPY(qroll, qpitch, init_hack.yaw);
			tf::Vector3 t_gl2ugv(StampedXform_gl2ugv.getOrigin().x(), StampedXform_gl2ugv.getOrigin().y(), StampedXform_gl2ugv.getOrigin().z());
			init_hack.position = (cv::Mat_<double>(3, 1) << StampedXform_gl2ugv.getOrigin().x(), StampedXform_gl2ugv.getOrigin().y(), StampedXform_gl2ugv.getOrigin().z());
	}

	void CKFread(const hast::posewithcov::ConstPtr& pose_msg)
	{
		if (pose_msg->header.seq != 0)
		{
			// ROS_INFO("CKFread");
			// fprintf (ugv.data.filename, "%% void CKFread(const hast::posewithcov::ConstPtr& pose_msg) \n");
			ros::Time msg_time = pose_msg->header.stamp;
			fprintf (ugv.data.filename, "\n%% --------------- CKFread --------------- \n" );
			fprintf (ugv.data.filename, "  %s.slam.time(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, msg_time.toSec()-init_time);
			fprintf (ugv.data.filename, "  %s.slam.yaw_gl(%d,1)  = % -6.14f;\n", ugv.s_data_filename.c_str(), pose_msg->header.seq, pose_msg->yaw);
			fprintf (ugv.data.filename, "  %s.slam.position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), pose_msg->header.seq,
								pose_msg->position.x, pose_msg->position.y, pose_msg->position.z);

			EstPosition_ckf = (cv::Mat_<double>(3, 1) << pose_msg->position.x,  pose_msg->position.y, pose_msg->position.z);
			EstYaw_ckf = pose_msg->yaw;
			msgTime_ckf = msg_time.toSec();
			ckf_update = true; dkf_update = false;
			updatePose();

		}
	}

	void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)
	{
		stereo_time_rx = ros::Time::now();
		stereoObs_P = (cv::Mat_<double>(4, 1) << stereo_state_msg->P.x, stereo_state_msg->P.y, stereo_state_msg->P.z, 1.0);
		cv::Mat stereoObsUGV_P = (cv::Mat_<double>(4, 1) << stereo_state_msg->ugvP_obs.x, stereo_state_msg->ugvP_obs.y, stereo_state_msg->ugvP_obs.z, 1.0); // observed ugv in FOV
		stereo_data = true;

		double cosyaw = cos(ugv.EstYaw_gl);
		double sinyaw = sin(ugv.EstYaw_gl);
		cv::Mat Rlo2gl4x4 = (cv::Mat_<double>(4, 4) <<
						 cosyaw, -sinyaw, 0, 0,
						 sinyaw,  cosyaw, 0, 0,
						 0, 0, 1, 0,
						 0, 0, 0, 1);

	 // vehicle relative stereo measurement in estimated global frame
		cv::Mat stereoObsUGV_P_global = Rlo2gl4x4 * stereoObsUGV_P; // + (cv::Mat_<double>(4, 1) << ugv.EstPosition_gl.at<double>(0,0), ugv.EstPosition_gl.at<double>(1,0), ugv.EstPosition_gl.at<double>(2,0), 0);
						stereoObs_P_global    = Rlo2gl4x4 * stereoObs_P; // + (cv::Mat_<double>(4, 1) << ugv.EstPosition_gl.at<double>(0,0), ugv.EstPosition_gl.at<double>(1,0), ugv.EstPosition_gl.at<double>(2,0), 0);

		cv::Mat uavPCov = Rlo2gl4x4 *(cv::Mat_<double>(4, 4) <<
			stereo_state_msg->PCov.row0.x,stereo_state_msg->PCov.row0.y,stereo_state_msg->PCov.row0.z,0,
			stereo_state_msg->PCov.row1.x,stereo_state_msg->PCov.row1.y,stereo_state_msg->PCov.row1.z,0,
			stereo_state_msg->PCov.row2.x,stereo_state_msg->PCov.row2.y,stereo_state_msg->PCov.row2.z,0,
			0,0,0,stereo_state_msg->yaw_cov) * Rlo2gl4x4.t();


		ugv_mux_msg.seq += 1;
		ugv_mux_msg.stamp = stereo_time_rx.toSec()-init_time;
		ugv_mux_msg.stereoMeas.position.x = stereoObs_P_global.at<double>(0,0);
		ugv_mux_msg.stereoMeas.position.y = stereoObs_P_global.at<double>(1,0);
		ugv_mux_msg.stereoMeas.position.z = stereoObs_P_global.at<double>(2,0);
		ugv_mux_msg.stereoMeas.PCov.row0.x = uavPCov.at<double>(0,0); ugv_mux_msg.stereoMeas.PCov.row0.y = uavPCov.at<double>(0,1); ugv_mux_msg.stereoMeas.PCov.row0.z = uavPCov.at<double>(0,2);
		ugv_mux_msg.stereoMeas.PCov.row1.x = uavPCov.at<double>(1,0); ugv_mux_msg.stereoMeas.PCov.row1.y = uavPCov.at<double>(1,1); ugv_mux_msg.stereoMeas.PCov.row1.z = uavPCov.at<double>(1,2);
		ugv_mux_msg.stereoMeas.PCov.row2.x = uavPCov.at<double>(2,0); ugv_mux_msg.stereoMeas.PCov.row2.y = uavPCov.at<double>(2,1); ugv_mux_msg.stereoMeas.PCov.row2.z = uavPCov.at<double>(2,2);

		ugv_mux_msg.stereoMeas.yaw = stereo_state_msg->yaw * Pi / 180; // stereoObs yaw is in degrees, convert to radians
		ugv_mux_msg.stereoMeas.yaw_cov = uavPCov.at<double>(3,3);
		/*  ~~~~~~~~~~~~~		(why is this still in ugv frame?:
		** it is in the local frame because of the SLAM update of Kk * (zk - Hx).
		** fullSLAM.correction = fullSLAM.Kk * (fullSLAM.zk - fullSLAM.Hx);
		** (Hx) is the difference between the uav & ugv heading angle, which is exaclty the value calculated by the stereo measurement. ** */


		ugv_mux_msg.stereoUGV_inFOV = stereo_state_msg->ugv_inFOV;

		cv::Mat UGVPCov_gl = Rlo2gl4x4 *(cv::Mat_<double>(4, 4) <<
			stereo_state_msg->ugvPCov_obs.row0.x,stereo_state_msg->ugvPCov_obs.row0.y,stereo_state_msg->ugvPCov_obs.row0.z,0,
			stereo_state_msg->ugvPCov_obs.row1.x,stereo_state_msg->ugvPCov_obs.row1.y,stereo_state_msg->ugvPCov_obs.row1.z,0,
			stereo_state_msg->ugvPCov_obs.row2.x,stereo_state_msg->ugvPCov_obs.row2.y,stereo_state_msg->ugvPCov_obs.row2.z,0,
			0,0,0,stereo_state_msg->ugvyaw_cov) * Rlo2gl4x4.t();


		ugv_mux_msg.stereoUGV.position.x	= stereoObsUGV_P_global.at<double>(0,0);
		ugv_mux_msg.stereoUGV.position.y	= stereoObsUGV_P_global.at<double>(1,0);
		ugv_mux_msg.stereoUGV.position.z	= stereoObsUGV_P_global.at<double>(2,0);
		ugv_mux_msg.stereoUGV.PCov.row0.x = UGVPCov_gl.at<double>(0,0); ugv_mux_msg.stereoUGV.PCov.row0.y = UGVPCov_gl.at<double>(0,1); ugv_mux_msg.stereoUGV.PCov.row0.z = UGVPCov_gl.at<double>(0,2);
		ugv_mux_msg.stereoUGV.PCov.row1.x = UGVPCov_gl.at<double>(1,0); ugv_mux_msg.stereoUGV.PCov.row1.y = UGVPCov_gl.at<double>(1,1); ugv_mux_msg.stereoUGV.PCov.row1.z = UGVPCov_gl.at<double>(1,2);
		ugv_mux_msg.stereoUGV.PCov.row2.x = UGVPCov_gl.at<double>(2,0); ugv_mux_msg.stereoUGV.PCov.row2.y = UGVPCov_gl.at<double>(2,1); ugv_mux_msg.stereoUGV.PCov.row2.z = UGVPCov_gl.at<double>(2,2);

		ugv_mux_msg.stereoUGV.yaw     = stereo_state_msg->ugvyaw; // stereoObsUGV yaw is in radians
		ugv_mux_msg.stereoUGV.yaw_cov = UGVPCov_gl.at<double>(3,3);

		ugv_mux_msg.ugvEst.vehicle_id = s_ugvn;
		ugv_mux_msg.ugvEst.position.x = ugv.EstPosition_gl.at<double>(0,0);
		ugv_mux_msg.ugvEst.position.y = ugv.EstPosition_gl.at<double>(1,0);
		ugv_mux_msg.ugvEst.position.z = ugv.EstPosition_gl.at<double>(2,0);
		ugv_mux_msg.ugvEst.yaw = ugv.EstYaw_gl;

		ugv_mux_msg.odomDelta.x = ugv.odom_delta.at<double>(0,0);
		ugv_mux_msg.odomDelta.y = ugv.odom_delta.at<double>(1,0);
		ugv_mux_msg.odomDelta.z = ugv.odom_delta.at<double>(2,0);
		ugv_mux_msg.odomDelta.w = ugv.odom_delta.at<double>(3,0);

		if(absolute_measurements){
			/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
					tf_getUAVpose();
					stereoObs_P = (cv::Mat_<double>(4, 1) <<
						ugv_mux_msg.stereoMeas.position.x,
						ugv_mux_msg.stereoMeas.position.y,
						ugv_mux_msg.stereoMeas.position.z, 1.0);
					stereoObs_P_global = ugv.Hlo2gl * stereoObs_P;
			/* HACK IN ABSOULTE MEASUREMENTS FROM VICON/GAZEBO*/
		}

		ugv_mux_pub.publish(ugv_mux_msg);

		fprintf (ugv.data.filename, "\n%% --------------- stereoRead ---------------\n" );
		fprintf (ugv.data.filename, "  %s.stereo.time(%d,1)         =  % -6.14f;\n", ugv.s_data_filename.c_str(), ++stereoObs_counter, ugv_mux_msg.stamp);
		fprintf (ugv.data.filename, "  %s.stereo.posetime(%d,1)     =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoObs_counter, ugv_mux_msg.stamp);
		fprintf (ugv.data.filename, "  %s.stereo.yaw_ugv(%d,1)      =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoObs_counter, ugv_mux_msg.stereoMeas.yaw);
		fprintf (ugv.data.filename, "  %s.stereo.yaw_gl_est(%d,1)   =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoObs_counter, ugv_mux_msg.stereoMeas.yaw + ugv_mux_msg.ugvEst.yaw);
		fprintf (ugv.data.filename, "  %s.stereo.Position_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_P.at<double>(0,0), stereoObs_P.at<double>(1,0), stereoObs_P.at<double>(2,0));
		fprintf (ugv.data.filename, "  %s.stereo.Position_gl(%u,:)  = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), stereoObs_counter, stereoObs_P_global.at<double>(0,0), stereoObs_P_global.at<double>(1,0), stereoObs_P_global.at<double>(2,0));


		fprintf (ugv.data.filename, "  %% --------------- ugv_sensor.inFOV = %s%% \n", ugv_mux_msg.stereoUGV_inFOV ? "true" : "false" );
		if (ugv_mux_msg.stereoUGV_inFOV)
		{
			stereoUGVcount += 1;
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.yaw(%d,1)      =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoUGVcount, ugv_mux_msg.stereoUGV.yaw);
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.yaw_cov(%d,1)  =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoUGVcount, ugv_mux_msg.stereoUGV.yaw_cov);
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.Position_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, stereoObsUGV_P.at<double>(0,0), stereoObsUGV_P.at<double>(1,0), stereoObsUGV_P.at<double>(2,0));
			fprintf (ugv.data.filename, "  %s.stereo.ugv_sensor.Position_gl(%u,:)  = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, stereoObsUGV_P_global.at<double>(0,0), stereoObsUGV_P_global.at<double>(1,0), stereoObsUGV_P_global.at<double>(2,0));
			fprintf (ugv.data.filename, "  %s.ugv_est.position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, ugv.EstPosition_gl.at<double>(0,0), ugv.EstPosition_gl.at<double>(1,0), ugv.EstPosition_gl.at<double>(2,0));
			fprintf (ugv.data.filename, "  %s.ugv_est.yaw_gl(%u,1)      =  % -6.14f;\n", ugv.s_data_filename.c_str(), stereoUGVcount, ugv.EstYaw_gl);
			// fprintf (ugv.data.filename, "  %s.ugv_est.Rlo2gl4x4(%u, 1,:) = [% -6.14f, % -6.14f, % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, Rlo2gl4x4.at<double>(0, 0), Rlo2gl4x4.at<double>(0, 1), Rlo2gl4x4.at<double>(0, 2));
			// fprintf (ugv.data.filename, "  %s.ugv_est.Rlo2gl4x4(%u, 2,:) = [% -6.14f, % -6.14f, % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, Rlo2gl4x4.at<double>(1, 0), Rlo2gl4x4.at<double>(1, 1), Rlo2gl4x4.at<double>(1, 2));
			// fprintf (ugv.data.filename, "  %s.ugv_est.Rlo2gl4x4(%u, 3,:) = [% -6.14f, % -6.14f, % -6.14f];\n", ugv.s_data_filename.c_str(), stereoUGVcount, Rlo2gl4x4.at<double>(2, 0), Rlo2gl4x4.at<double>(2, 1), Rlo2gl4x4.at<double>(2, 2));
		}


		fprintf (ugv.data.filename, "   try waitbar(%f/waitbar_max,wb); end\n", double(stereoObs_counter));

			// reset odom contrainers
			ugv.odom_delta = (cv::Mat_<double>(4, 1) << 0,0,0,0);
	}

	void tf_getUAVpose() // only used with absolute measurements
	{
		tf::StampedTransform StampedXform_ugv2red, StampedXform_ugv2blue, StampedXform_ugv2green, StampedXform_gl2ugv;
		try {listener.lookupTransform("/vicon/origin", "/vicon/"+ s_ugvn +"/base_TF", ros::Time(0), StampedXform_gl2ugv);}
			catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/origin to /vicon/%s/base_TF is missing", s_ugvn.c_str());}
			double qroll, qpitch, ugv_yaw;
			tf::Matrix3x3(StampedXform_gl2ugv.getRotation()).getRPY(qroll, qpitch, ugv_yaw);
			tf::Vector3 t_gl2ugv(StampedXform_gl2ugv.getOrigin().x(), StampedXform_gl2ugv.getOrigin().y(), StampedXform_gl2ugv.getOrigin().z());

		try {listener.lookupTransform("/vicon/"+ s_ugvn +"/base_TF", "/vicon/uav/ardrone_red_led", ros::Time(0), StampedXform_ugv2red);}
			catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/%s/base_TF to /vicon/uav/ardrone_red_led is missing", s_ugvn.c_str());}
			tf::Vector3 t_ugv2red(StampedXform_ugv2red.getOrigin().x(), StampedXform_ugv2red.getOrigin().y(), StampedXform_ugv2red.getOrigin().z());

		try {listener.lookupTransform("/vicon/"+ s_ugvn +"/base_TF", "/vicon/uav/ardrone_blue_led", ros::Time(0), StampedXform_ugv2blue);}
			catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/%s/base_TF to /vicon/uav/ardrone_blue_led is missing", s_ugvn.c_str());}
			tf::Vector3 t_ugv2blue(StampedXform_ugv2blue.getOrigin().x(), StampedXform_ugv2blue.getOrigin().y(), StampedXform_ugv2blue.getOrigin().z());

		try {listener.lookupTransform("/vicon/"+ s_ugvn +"/base_TF", "/vicon/uav/ardrone_green_led", ros::Time(0), StampedXform_ugv2green);}
			catch (tf::TransformException ex) {ROS_INFO("tf_getUAVpose: /vicon/%s/base_TF to /vicon/uav/ardrone_green_led is missing", s_ugvn.c_str());}
			tf::Vector3 t_ugv2green(StampedXform_ugv2green.getOrigin().x(), StampedXform_ugv2green.getOrigin().y(), StampedXform_ugv2green.getOrigin().z());


		cv::Matx<double,4,1> GmB, RmB, BmR, RmG, UAV_p_ugv;
			UAV_p_ugv = 0.5* cv::Matx41d(t_ugv2red.x() + t_ugv2blue.x(), t_ugv2red.y() + t_ugv2blue.y(), t_ugv2red.z() + t_ugv2blue.z(), 0);
			GmB = cv::Matx41d(t_ugv2green.x() - t_ugv2blue.x(), t_ugv2green.y() - t_ugv2blue.y(), t_ugv2green.z() - t_ugv2blue.z(), 0);
			RmB = cv::Matx41d(t_ugv2red.x() - t_ugv2blue.x(), t_ugv2red.y() - t_ugv2blue.y(), t_ugv2red.z() - t_ugv2blue.z(), 0);
			BmR = cv::Matx41d(t_ugv2blue.x() - t_ugv2red.x(), t_ugv2blue.y() - t_ugv2red.y(), t_ugv2blue.z() - t_ugv2red.z(), 0);
			RmG = cv::Matx41d(t_ugv2red.x() - t_ugv2green.x(), t_ugv2red.y() - t_ugv2green.y(), t_ugv2red.z() - t_ugv2green.z(), 0);

		cv::Matx<double,4,1> znum, x_axis, y_axis, z_axis; //Calculated UAV axes from Circle centers
			znum = Cross(GmB, RmB);
			double normz = norm(znum, L2Norm);
			double normy = norm(BmR, L2Norm);
			z_axis = (1.0 / normz) * znum;
			y_axis = (1.0 / normy) * BmR;
			x_axis = Cross(y_axis, z_axis);

			ugv_mux_msg.stereoMeas.position.x = UAV_p_ugv(0,0);
			ugv_mux_msg.stereoMeas.position.y = UAV_p_ugv(1,0);
			ugv_mux_msg.stereoMeas.position.z = UAV_p_ugv(2,0);
			ugv_mux_msg.stereoMeas.yaw = atan2(x_axis(0, 1), x_axis(0, 0)); // * 180 / Pi; //heading angle in degrees in ugv frame
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if (ShutDown->flag)
		{
			ros::Duration(1.5).sleep();
			ugv.writePrealloc(double(stereoObs_counter));
			ros::Duration(0.5).sleep();
			fprintf (ugv.data.filename, "\n%% --------------- close(wb) ---------------\n" );
			fprintf (ugv.data.filename, "    try close(wb); end;\n");
			ros::shutdown();
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugv_muxxer");
	ugvMuxxer ugvMux;
	ros::spin();
	return 0;
}
