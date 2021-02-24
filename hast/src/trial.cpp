#include "genheaders.hpp"
// #include "utils.hpp"

#include <actionlib/client/simple_action_client.h>
#include <hast/ugvgoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr uavWaypoint_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ugvPicket_ptr (new pcl::PointCloud<pcl::PointXYZ>);

class uavClass
{
	private:
	public:
		/* -------- Clock Times */
		ros::Time navStamp;
		uint navHeadSeq;
		double navTS, navTSpublished, navdt; //, KFdt;

		double Pi;
		int L2Norm;

		/* ugv position variables*/
		cv::Mat EstPosition_gl;
		double EstYaw_gl, cosyaw, sinyaw;
		cv::Mat Rgl2lo, Hlo2gl, Hgl2lo;

		ros::ServiceClient slamswitch_cli;
			hast::slamswitch slamswitch_call;
			hast::posewithheader pose_msg;
			uint pose_msg_seq;

			struct ROSTOPICS_struc
			{
				std::string s_uav_waypoint_cloud_pub;
				std::string s_uav_pose_state_sub;
				std::string s_uav_control_cli; // control on/off/direct-tilt
				std::string s_slam_switch_cli; // to turn on slam?
				std::string s_uav_flight_state_cli;
			}; ROSTOPICS_struc topics;

		ros::Subscriber state_sub;
			bool state_bool;
		ros::ServiceClient control_cli;
			hast::uavcontrolstate control_call;
		ros::ServiceClient navState_cli;
			hast::uavnavstate navState_call;
			uint navStateUint;
		ros::Publisher Land_pub, TakeOff_pub, Reset_pub, imuRecal_pub, flatTrim_pub;
			std_msgs::Empty null;

		double DesiredYaw, flytime;
		cv::Mat Waypoint0;
		cv::Mat WaypointA, WaypointB, WaypointC, WaypointD, WaypointE, WaypointF, WaypointG, WaypointH, WaypointI;
		double WaypointYaw0, WaypointYawA, WaypointYawB, WaypointYawC, WaypointYawD, WaypointYawE, WaypointYawF, WaypointYawG, WaypointYawH, WaypointYawI;

		double liftoff_time;
		double init_time;
		int retryTakeoffCount;
		bool takeoffError;

		ros::Publisher Waypoint_pub;
			sensor_msgs::PointCloud2 uavWaypoint_msg;

	// uav class init
		uavClass() // void for construction of class
		{
			// Possibly useful constants
			pose_msg_seq = 0;
			Pi = atan(1) * 4; // 3.14159...
			L2Norm = 4; // Frobenius norm for CV norm() function

			navStateUint = 0;

			// Initial estimated states of ugv
			EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
			EstYaw_gl = 0;
			cosyaw = cos( EstYaw_gl );
			sinyaw = sin( EstYaw_gl );
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
								cosyaw, sinyaw, 0,
								-sinyaw, cosyaw, 0,
								0, 0, 1);
			DesiredYaw = 0;
			flytime = 5;
			liftoff_time = 0;
			state_bool = false;

			WaypointA = (cv::Mat_<double>(3,1) << 3.15, 0.80, 1.50); WaypointYawA = 0.00;
			WaypointB = (cv::Mat_<double>(3,1) << 2.25, 0.70, 1.40); WaypointYawB = 0.00;
			WaypointC = (cv::Mat_<double>(3,1) << 1.60, 0.60, 1.30); WaypointYawC = 0.00;
			WaypointD = (cv::Mat_<double>(3,1) << 1.60, 0.00, 1.30); WaypointYawD = 0.00;
			WaypointE = (cv::Mat_<double>(3,1) << 3.80, 0.00, 1.50); WaypointYawE = 0.00;
			WaypointF = (cv::Mat_<double>(3,1) << 3.15,-0.80, 1.50); WaypointYawF = 0.00;
			WaypointG = (cv::Mat_<double>(3,1) << 2.25,-0.70, 1.40); WaypointYawB = 0.00;
			WaypointH = (cv::Mat_<double>(3,1) << 1.60,-0.60, 1.30); WaypointYawG = 0.00;
			// waypoint 0?
			Waypoint0 = (cv::Mat_<double>(3,1) << 2.50, 0.00, 1.50); WaypointYaw0 = 0.75; // yaw converted to degrees before sending to uav uavAutopilot
		}

		void readState(const hast::uavstate::ConstPtr& uavState_msg)
		{
			EstPosition_gl = (cv::Mat_<double>(3,1) << uavState_msg->P.x, uavState_msg->P.y, uavState_msg->P.z);
			EstYaw_gl = uavState_msg->yaw;
			state_bool = true;
			// ROS_INFO("trial: uav.readState::EstPosition_gl: [% -6.8f % -6.8f % -6.8f]", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		}

		void hoverFor(double span)
		{
			double tstart = ros::Time::now().toSec();
			double elapsed = 0;
			while(elapsed < span)
			{
				// mac ROS_INFO("trial: Drone Hovering for (% -6.8f)", (span-elapsed));
				hover();
				spin_sleep(0.25); // sleep for 'x' second(s).
				elapsed = (ros::Time::now().toSec() - tstart);
			}
		}

		void hover()
		{
			ros::spinOnce();
			// // mac ROS_INFO("trial: Drone Flight Service Called: Hover");
			control_call.request.flip = false;
			control_call.request.directtilt = false; // true: use tilt.x.y.z for control, false : use PV control
			control_call.request.dP_g.x = 0;
			control_call.request.dP_g.y = 0;
			control_call.request.dP_g.z = 0;
			control_call.request.dYaw = 0;
			control_call.request.YawRate = 0;
			control_call.request.tilt.x = 0;
			control_call.request.tilt.y = 0;
			control_call.request.tilt.z = 0;
			control_cli.call(control_call);
		}

		void callState()
		{
			takeoffError = false;
			navState_cli.call(navState_call);
			navStateUint = navState_call.response.state;
			ROS_INFO("callState() : %u", navStateUint);
			spin_sleep(1); // sleep for 'x' second(s).
		}

	/* UAV utility functions*/
		void liftoff()
		{ROS_INFO("uav.liftoff(); ...");
			/*--- Zero commanded tilts before liftoff */
			for(int b = 0; b < 2; ++b)
				{reCal();hover();ros::spinOnce();}
			/*--------- Drone Takeoff ------------- */
			callState();
			// if((navStateUint==2)||(navStateUint==3)||(navStateUint==4))
			if(navStateUint==2)
			{
				TakeOff_pub.publish(null);
				spin_sleep(1.5); // sleep for 'x' second(s).
				liftoff_time = ros::Time::now().toSec();
			}
			else{
				retryTakeoff();
			}

			callState();
			if (navStateUint==0){retryTakeoff();}

			spin_sleep(1.0); // sleep for 'x' second(s).
			hoverFor(2);

			if (navStateUint==0)
			{takeoffError = true;}
			else
			{ROS_INFO("uav.liftoff(); ... complete.");}
		}

		void retryTakeoff()
		{
			retryTakeoffCount = 0;
			do
			{
				++retryTakeoffCount;
				Reset_pub.publish(null);
				// ros::Duration(2).sleep(); // sleep for 'x' second(s).
				spin_sleep(2); // sleep for 'x' second(s).
				TakeOff_pub.publish(null);
	        callState();
	        callState();
        liftoff_time = ros::Time::now().toSec();
			}while(navStateUint==0 && retryTakeoffCount<10);
		}

		void setDesiredPosition(double NewX, double NewY, double NewZ, double NewYaw)
		{
			// update desired positions in controller
			// fprintf(stdout, (BLUE_TEXT "trial: uav.setDesiredPosition(% -4.4f, % -4.4f, % -4.4f; % -4.4f)\n" COLOR_RESET ), NewX, NewY, NewZ, NewYaw );
			control_call.request.flip = true; // true: use PV control instead of dumb hover
			control_call.request.directtilt = false; // true: use tilt.x.y.z for control false = use PV control
			control_call.request.dP_g.x = NewX;
			control_call.request.dP_g.y = NewY;
			control_call.request.dP_g.z = NewZ;
			control_call.request.dYaw = NewYaw * 180 / Pi; // convert control to degrees
			control_cli.call(control_call);

			publishUAVwaypoint(NewX, NewY, NewZ);
		}

		void publishUAVwaypoint(double NewX, double NewY, double NewZ)
		{
			pcl::PointXYZ pcl_XYZ; // pcl point for center of obstacle
				pcl_XYZ.x = NewX;
				pcl_XYZ.y = NewY;
				pcl_XYZ.z = NewZ;

			uavWaypoint_ptr->points.clear();
			uavWaypoint_ptr->points.push_back(pcl_XYZ);
			pcl::toROSMsg(*uavWaypoint_ptr, uavWaypoint_msg);
			uavWaypoint_msg.header.frame_id = "/map";
			Waypoint_pub.publish(uavWaypoint_msg);
		}

		void reCal()
		{
			// ROS_INFO("reCal(): Publishing Flat-Trim Command...");
				flatTrim_pub.publish(null);
				ros::Duration(0.25).sleep(); // sleep for 'x' second(s).

			// ROS_INFO("reCal(): Publishing IMU reCal Command...");
				imuRecal_pub.publish(null);
				ros::Duration(0.25).sleep(); // sleep for 'x' second(s).

			// ROS_INFO("reCal(): ...done");
		}

		void spin_sleep(double sleep_time)
		{
			double sleepStartTime = ros::Time::now().toSec();
			while((ros::Time::now().toSec() - sleepStartTime) < sleep_time){ros::spinOnce();}
		}

};

class ugvClass
{
	private:
		// Constants which might be useful in the future
		double Pi;
		int L2Norm;

	public:
		double init_time;

		/* ugv position variables*/
		cv::Mat EstPosition_gl;
		double EstYaw_gl, cosyaw, sinyaw;
		cv::Mat Rgl2lo, Hlo2gl, Hgl2lo;

		ros::Subscriber pose_sub;
		uint stateMsg_id, stateMsg_idLast; // pose msg counter
		double msgStamp, msgStampLast;
		uint posecount;

		/* ugv client/Services variables*/
		std::string id;
		int goal_tag_id;
		double yawAtGoal_gl;

		std::string s_move_goal_topic, s_clearcostmap_topic, s_goal_tag_topic;
		std::string s_movebase_status_topic, s_goal_header_frame_id, s_map_frame;
		std::string s_local_plan_topic, s_global_plan_topic;
		std::string s_pose_topic;

		ros::Subscriber state_sub, move_base_status_sub;
		ros::ServiceClient clearCostmaps_cli; hast::null nullcall; // clears obstacle costmaps
		ros::Publisher goalpose_pub; //publisher for setting move base goal of ugv
			geometry_msgs::Pose2D goal_for_ugv_msg; //pose message for uav goal location
		ros::ServiceClient GoalTagLocation_cli; // client to request goal tag location
			hast::ugvgoal goal_call; // service data for getting tag location
			bool goal_call_set;


		// plan variables
		nav_msgs::Path global_plan, local_plan;
			ros::Subscriber global_plan_sub, local_plan_sub;
			double yaw_at_end_radians, yaw_at_end_degrees; //wheelyaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)); // radians

			bool local_plan_bool;
			std::vector<geometry_msgs::PoseStamped> globalPoses, localPoses; //marker array as detected by image
			int local_path_count, global_path_count, picket_count, action_count, turn_count;

		cv::Mat picket_offset, picket_end, picket_mid, path_end_gl; // offset vector for uav pose relative to ugv
		double yaw_end, yaw_mid;
			bool holdsPicket, plan_available;
			int actionStatus;
			std::vector<actionlib_msgs::GoalStatus> goal_status;

		tf::TransformBroadcaster tf_bc;
			tf::Transform pose_TF;
			tf::Quaternion ugv_q;
			tf::Matrix3x3 ugvR_TF;
			tf::Quaternion ugvQ_TF;
			std::string s_TF_mux, s_TF_odom, s_TF_global;

			ros::Time bc_tf_time;
			move_base_msgs::MoveBaseGoal goal; // for move_base goal setting, might not be used...
			geometry_msgs::PointStamped goalPoint_gl, goalPoint_map; // might not be used...

			tf::StampedTransform xf_stamped;


			tf::Vector3 goalP_gl_tf, goalP_odom_tf; // goal point locations in global and local frame


		// Alternatives to using TF
		cv::Mat map_origin, map_Rgl2odom, map_H_gl2odom, map_H_odom2gl;
		double map_yaw;

		ros::ServiceClient callMuxMap_cli; // client to request master ros time offset
			hast::servmuxmap muxmap_call;
			std::string s_ugv_mux_map_serve_topic;
			tf::Transform map_tf;
			geometry_msgs::Pose map_tf_pose;

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
			hast::ugvdrive call; // call message
			bool call_bool; // boolean flag used for testing whether the client called the service properly
			int call_fails; //counter for number of failed calls (a robistifier)
			cv::Mat position; // matrix fo 3d position of goal location
			double yaw;
		};
		ugv_pose_struct driver_goal;

		ros::Publisher PicketPCL_pub;
			sensor_msgs::PointCloud2 PicketPCL_msg;

	ugvClass() // void for construction of class
	{
		// ROS comms
		stateMsg_id = 0;
		stateMsg_idLast = 0;
		posecount = 0;

		// Possibly useful constants
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function

		// Initial estimated states of ugv
		EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		EstYaw_gl = 0;
		cosyaw = cos( EstYaw_gl );
		sinyaw = sin( EstYaw_gl );
		Rgl2lo = (cv::Mat_<double>(3, 3) <<
							cosyaw, sinyaw, 0,
							-sinyaw, cosyaw, 0,
							0, 0, 1);

		Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
		Hlo2gl = invertH(Hgl2lo);

		goal_call.request.tagID = 1;
		goal_call.request.ugv_id = "ugv0";
		holdsPicket = false;
		plan_available = false;
		local_plan_bool = false;

		// pre-load to prevent empty vector operations
		picket_end = (cv::Mat_<double>(3, 1) << 1.5, 0.0, 0.8);
		picket_end.copyTo(picket_mid);
		picket_end.copyTo(picket_offset);
		picket_end.copyTo(path_end_gl);

		actionStatus = 0;
		local_path_count = 0;
		global_path_count = 0;
		picket_count = 0;
		action_count = 0;
		turn_count = 0;

		map_yaw = 0;
		map_origin = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		map_Rgl2odom = (cv::Mat_<double>(3, 3) <<
							1, 0, 0,
							0, 1, 0,
							0, 0, 1);

		map_H_gl2odom = wrapH(map_Rgl2odom, map_origin);
		map_H_odom2gl = invertH(map_H_gl2odom);

	}

	void turnTo(double angle_rad)
	{
		// ROS_INFO("void turnTo(double angle_rad)");
		driver_status.msg.at_yaw = false;
		driver_status.msg.at_pos = false;

		driver_goal.call_fails = 0;
		driver_goal.call.request.goalYaw = angle_rad;
		ROS_INFO("trial: %s.driver_goal.call.request.goalYaw: %6.4f", id.c_str(), driver_goal.call.request.goalYaw);

		ROS_INFO("trial: calling %s.driver_goal.client", id.c_str());
		driver_goal.call_bool = driver_goal.client.call(driver_goal.call);
		// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
		while (!driver_goal.call_bool && driver_goal.call_fails < 5)
		{
			driver_goal.call_fails +=1;
			driver_goal.call_bool = driver_goal.client.call(driver_goal.call);
			// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
			spin_sleep(0.1);
		}

		// if (driver_goal.call_bool)
		// {
		// 	double MoveStartTime = ros::Time::now().toSec();
		// 	spin_sleep(0.1);
		// 	double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
		// 	while ((!driver_status.msg.at_yaw) && (ElapsedTime < 600) )  { // ugv are still moving
		// 		ROS_INFO(" trial:: %s.driver_status.msg.at_yaw : '%s';", id.c_str(), driver_status.msg.at_yaw ? "true" : "false" );
		// 		spin_sleep(0.25);
		// 		ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
		// 	}
		// }
	}

	void dummy_goal(double x, double y, double theta)
	{
		goal_for_ugv_msg.x = x;
		goal_for_ugv_msg.y = y;
		goal_for_ugv_msg.theta = theta; // set the ugv goal yaw to match tag yaw, for some reason
		goalpose_pub.publish(goal_for_ugv_msg);

	}

	bool call_tag_location(double tag_num, bool use_move_base)
	{
		goal_call.request.tagID = tag_num;
		clearCostmaps_cli.call(nullcall);
		bool call_bool = GoalTagLocation_cli.call(goal_call);
		bool call_map_dkf = callMuxMap_cli.call(muxmap_call); // get location of ugv map relative to global

		spin_sleep(1);
		if (call_bool && call_map_dkf){
			goal_call_set = goal_call.response.goalInMap;
			if (goal_call.response.goalInMap){
				// get goal location in global/map frame
				goalPoint_gl = goal_call.response.goalStamped; // 			geometry_msgs::PointStamped goalPoint_gl

				try {
					// create tf from global/map frame pose
					map_tf_pose  = muxmap_call.response.map_pose;
					map_yaw = muxmap_call.response.map_yaw;

					map_tf.setOrigin( tf::Vector3(map_tf_pose.position.x, map_tf_pose.position.y, map_tf_pose.position.z));
					map_tf.setRotation( tf::Quaternion(map_tf_pose.orientation.x, map_tf_pose.orientation.y, map_tf_pose.orientation.z, map_tf_pose.orientation.w));

					map_H_odom2gl = (cv::Mat_<double>(4, 4) <<
									 cos( -map_yaw ), sin( -map_yaw ), 0, map_tf_pose.position.x,
									-sin( -map_yaw ), cos( -map_yaw ), 0, map_tf_pose.position.y,
									0, 0, 1, 0,
									0, 0, 0, 1);

					goalP_gl_tf = tf::Point(goal_call.response.goalP.x, goal_call.response.goalP.y, goal_call.response.goalP.z);
					goalP_odom_tf = map_tf.inverse() * goalP_gl_tf;

					// ROS_INFO("goal point in global frame      ::   [x, y, phi] : [% -4.4f, % -4.4f, % -4.4f]", goalP_gl_tf.x(), goalP_gl_tf.y(), goal_call.response.goalYaw);
					// ROS_INFO("goal point in local frame?      ::   [x, y, phi] : [% -4.4f, % -4.4f, % -4.4f]", goalP_odom_tf.x(), goalP_odom_tf.y(), goal_call.response.goalYaw - EstYaw_gl);
					goal_for_ugv_msg.x = goalP_odom_tf.x();
					goal_for_ugv_msg.y = goalP_odom_tf.y();
					goal_for_ugv_msg.theta = goal_call.response.goalYaw - EstYaw_gl; // set the ugv goal yaw to match tag yaw, for some reason
					if (use_move_base){
						ROS_INFO("goalpose_pub.publish(goal_for_ugv_msg);");
						goalpose_pub.publish(goal_for_ugv_msg);
					}

					return true;
				} catch (tf::TransformException ex) {
					ROS_ERROR("%s", ex.what());
					spin_sleep(1);
					return false;
				}
				}
		  } else {
		    ROS_INFO("TRIAL:: (%s) Failed to call service GoalTagLocation_cli", id.c_str());
				return false;
		  }
	}

	void spin_sleep(double sleep_time)
	{
		double sleepStartTime = ros::Time::now().toSec();
		while((ros::Time::now().toSec() - sleepStartTime) < sleep_time){ros::spinOnce();}
	}

	void driver_status_read(const hast::ugvmuxstatus::ConstPtr& status_msg)
	{
		driver_status.msg.header = status_msg->header;
		driver_status.msg.at_yaw = status_msg->at_yaw;
		driver_status.msg.at_pos = status_msg->at_pos;
	}

	void move_base_status_read(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg)
	{
		// ROS_INFO("move_base_status_read(); ...");
		// actionStatus = status_msg -> status_list[0].status;
		// ROS_INFO("goal_status = status_msg -> status_list");
		goal_status = status_msg -> status_list;
		// ROS_INFO("actionStatus = goal_status[0].status;");
		int size_list = goal_status.size();
		if (size_list>0){
			actionStatus = goal_status[0].status;
		}
		//
		// uint8 PENDING         = 0   # The goal has yet to be processed by the action server
		// uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
		// uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
		//                             #   and has since completed its execution (Terminal State)
		// uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
		// uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
		//                             #    to some failure (Terminal State)
		// uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
		//                             #    because the goal was unattainable or invalid (Terminal State)
		// uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
		//                             #    and has not yet completed execution
		// uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
		//                             #    but the action server has not yet confirmed that the goal is canceled
		// uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
		//                             #    and was successfully cancelled (Terminal State)
		// uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
		//                             #    sent over the wire by an action server

	}

	void readState(const hast::ugvstate::ConstPtr& ugvState_msg)
	{
		// ROS_INFO("ugv readState(); ...");
		msgStamp = ugvState_msg-> stamp;

		if (msgStamp!=msgStampLast)
		{
			++posecount;
			msgStampLast = ugvState_msg-> stamp;
			EstPosition_gl = (cv::Mat_<double>(3, 1) <<
				ugvState_msg -> P.x,
				ugvState_msg -> P.y,
				ugvState_msg -> P.z);
			EstYaw_gl = ugvState_msg -> yaw;
			stateMsg_id = ugvState_msg -> id;

			cosyaw = cos( EstYaw_gl );
			sinyaw = sin( EstYaw_gl );
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
								cosyaw, sinyaw, 0,
								-sinyaw, cosyaw, 0,
								0, 0, 1);

			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);


			// ROS_INFO("pose_TF.setOrigin( tf::Vector3(ugvState_msg ->P.x, ugvState_msg ->P.y, ugvState_msg ->P.z) );");
			pose_TF.setOrigin( tf::Vector3(ugvState_msg ->P.x, ugvState_msg ->P.y, ugvState_msg ->P.z) );
			// ROS_INFO("ugv_q.setRPY(0, 0, ugvState_msg ->yaw);");
			ugv_q.setRPY(0, 0, ugvState_msg ->yaw);
			// ROS_INFO("pose_TF.setRotation(ugv_q);");
			pose_TF.setRotation(ugv_q);
			// ROS_INFO("ros::Time nowStamp = ros::Time::now();");
			bc_tf_time = ros::Time::now();
			// ROS_INFO("tf_bc.sendTransform(tf::StampedTransform(pose_TF, nowStamp, s_TF_odom, s_TF_mux));");
			tf_bc.sendTransform(tf::StampedTransform(pose_TF, bc_tf_time, s_TF_global, s_TF_mux));
		}
	}

	cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
	{
		//ROS_INFO("uav cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)");
		// H = [R -R*t(1:3,1); 0 0 0 1];
		cv::Mat H, nRt, t;
		t = (cv::Mat_<double>(3, 1) << t_in.at<double>(0,0),t_in.at<double>(1,0),t_in.at<double>(2,0));
		nRt = -R_in * t;
		H = (cv::Mat_<double>(4, 4) <<
			R_in.at<double>(0,0), R_in.at<double>(0,1), R_in.at<double>(0,2), nRt.at<double>(0,0),
			R_in.at<double>(1,0), R_in.at<double>(1,1), R_in.at<double>(1,2), nRt.at<double>(1,0),
			R_in.at<double>(2,0), R_in.at<double>(2,1), R_in.at<double>(2,2), nRt.at<double>(2,0),
			0,0,0,1);
		return H;
	}

	cv::Mat invertH(cv::Mat H)
	{
		// Hinv = [R' -R'*t; 0 0 0 1];
		cv::Mat Hinv, RT, R, t, nRTt;
		t = (cv::Mat_<double>(3, 1) << H.at<double>(0,3),H.at<double>(1,3),H.at<double>(2,3));
		R = (cv::Mat_<double>(3, 3) << 	H.at<double>(0,0), H.at<double>(0,1), H.at<double>(0,2),
																		H.at<double>(1,0), H.at<double>(1,1), H.at<double>(1,2),
																		H.at<double>(2,0), H.at<double>(2,1), H.at<double>(2,2));
		RT = R.t();
		nRTt = -RT*t;
		Hinv = (cv::Mat_<double>(4, 4) <<
			RT.at<double>(0,0), RT.at<double>(0,1), RT.at<double>(0,2), nRTt.at<double>(0,0),
			RT.at<double>(1,0), RT.at<double>(1,1), RT.at<double>(1,2), nRTt.at<double>(1,0),
			RT.at<double>(2,0), RT.at<double>(2,1), RT.at<double>(2,2), nRTt.at<double>(2,0),
			0,0,0,1);
		return Hinv;
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
	};

};

class hastTrial
{
	private:
	public:
		/*---------  File Recorder ------------- */
		std::string s_filename, s_trial, s_dotm, s_root, s_handle, s_date, s_user;
		std::string s_exp_code;
		std::FILE * pFile;
		std::string s_ugv_basefootprint;
		std::string s_ugvCmd_topic;
		double Pi;
		int L2Norm;

		/*--------- Flight Containers ------------- */
		bool FlightFlag;
		uint FlyToCount;
		double FlyToYaw, FlyToYawLast;
			cv::Mat FlyTo_g;

		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;

		/*----- Hast Channels */
 		bool ON, OFF;
			// Shutdown hast nodes

		uavClass uav; // create uav from class
		ugvClass ugv; // create ugv from class
		ugvClass ugv1, ugv2;
		ros::Subscriber uav_state_sub;

		/*----- experiment types */
		struct killtopics
		{
			hast::flag Kill_msg;
				std_msgs::Empty Null_msg;
				geometry_msgs::Twist DroneCmd_dr_msg;

				ros::Publisher HastKill, DroneLand, DroneCmd_dr_pub;
				std::string s_killswitch_topic, s_uav_land_topic, s_uav_zerocmd_topic;

			ros::ServiceServer trial_running_ser;
				std::string s_trial_running_ser;
		}; killtopics endTrial;

		struct rostopics
		{
			ros::ServiceClient slamswitch_cli;
				hast::slamswitch slamswitch_call;
				hast::posewithheader pose_msg;
				uint pose_msg_seq;
				std::string s_slam_switch_cli;

		}; rostopics jointSLAM;

		ros::Subscriber KillHast_sub, uav_pose_sub;

		// ros::Publisher HastShutDown_pub;
			hast::flag ShutDown;

		tf::TransformListener listener;
		ros::ServiceClient callTime_cli; // client to request master ros time offset
			hast::servtime time_call;
			std::string calling_node;
			bool gazebo;

			bool iflift, slamtest;

		struct EXPERIMENT_FLAGS_struct
		{
			bool oneUGV; // one or two ugv in experiment
			bool ugv1_w_hover, ugv2_w_hover;
			bool ugv1_picket, ugv2_picket; //uav is a picket robot for ugv2
			bool both_w_hover; //both ugvs move with uav hovering at goal?
			bool ugv1_watchugv2, ugv2_watchugv1; // flag for ugv1 watching ugv 2 as it moves
			bool ugv1_lookatgoal, ugv2_lookatgoal; // bool flag for ugv1 looking at goal

		}; EXPERIMENT_FLAGS_struct exp;

		double start_wait;
		double init_time;

		ros::Time node_start_time;

	hastTrial()
	{
			node_start_time = ros::Time::now();
		/*---------  matlab File Recorder Initilizer ------------- */
			s_handle = "trial";
			if(n.getParam("/hast/user", s_user))				{ROS_INFO("/hast/user: %s", s_user.c_str());}					else {s_user = "benjamin";}
			if(n.getParam("/hast/date", s_date))				{ROS_INFO("/hast/date: %s", s_date.c_str());}					else {s_date = "20181211";}
			if(n.getParam("/hast/trial", s_trial))			{ROS_INFO("/hast/trial: %s", s_trial.c_str());} 			else {s_trial = "001";}
			if(n.getParam("/hast/exp_code", s_exp_code)){ROS_INFO("/hast/exp_code: %s", s_exp_code.c_str());}	else {s_exp_code = "A";}

			s_root = "/home/" + s_user + "/ros/data/";
			s_dotm = ".m";
			s_filename = s_root + s_date + "/" + s_exp_code + "/" + s_trial + "/" + s_handle + "_" + s_trial + s_dotm;
			ROS_INFO("trial: %s", s_filename.c_str());

			pFile = std::fopen (s_filename.c_str(),"w");
			fprintf (pFile,"%% %s\n",s_filename.c_str());
			fprintf (pFile,"\n%%clc; \n%%clear all;\n%%close all;\n\n");

			if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}

			// callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
			// time_call.request.requesting_node = "trial";
			// bool call_bool = callTime_cli.call(time_call);
			// ros::Time calltime = ros::Time::now();
			// if (call_bool){
			// 	ROS_INFO("Master running.");
			// 	ros::Time mastertime = time_call.response.header.stamp;
			//
			// 	if (gazebo){
			// 		init_time = 0.0;
			// 	} else {
			// 		ROS_INFO("%s::init_time = %f", time_call.request.requesting_node.c_str(), init_time+=(calltime.toSec() - mastertime.toSec()));
			// 		ROS_INFO("%s::init_time offset = %f", time_call.request.requesting_node.c_str(), (calltime.toSec() - mastertime.toSec()));
			// 		fprintf (pFile,"trial.init_time  = % -6.8f;\n", init_time);
			// 		fprintf (pFile,"trial.init_time_offset  = % -6.8f;\n", (calltime.toSec() - mastertime.toSec()));
			// 	}
			// 	// } else {init_time = 0.0;}
			// } else {
			// 	ros::shutdown();
			// }

			uav.init_time = init_time;
			ugv1.init_time = init_time;
			ugv2.init_time = init_time;

			ugv1.id = "ugv1";
			ugv2.id = "ugv2";


		/*--------- Initialize ROS Communication ------------- */
		/*-----  Publishers and Subscribers */
			// rostopic pub -1 /ardrone/flattrim std_msgs/Empty
			// rostopic pub -1 /ardrone/imu_recalib std_msgs/Empty
		/*----- UAV Channels */
			if(ros::param::get("~flytime",uav.flytime)){} else {uav.flytime = 10;} //time (seconds) for uav to make and keep a waypoint
			uav.imuRecal_pub	= n.advertise<std_msgs::Empty>("/ardrone/imu_recalib", 1000);
			uav.flatTrim_pub	= n.advertise<std_msgs::Empty>("/ardrone/flattrim", 1000);
			uav.Land_pub    	= n.advertise<std_msgs::Empty>("/ardrone/land", 10);
			uav.TakeOff_pub 	= n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
			uav.Reset_pub 		= n.advertise<std_msgs::Empty>("/ardrone/reset", 10);

			// publishers
			ros::param::get("~uav_pose_state_topic",		uav.topics.s_uav_pose_state_sub);			fprintf (pFile,"trial.topics.uav_pose_state_topic   = '%s';\n",	uav.topics.s_uav_pose_state_sub.c_str());
			ros::param::get("~uav_waypoint_cloud_pub",	uav.topics.s_uav_waypoint_cloud_pub);	fprintf (pFile,"trial.topics.uav_waypoint_cloud_pub = '%s';\n",	uav.topics.s_uav_waypoint_cloud_pub.c_str());
			ros::param::get("~uav_control_cli",					uav.topics.s_uav_control_cli);				fprintf (pFile,"trial.topics.uav_control_cli        = '%s';\n",	uav.topics.s_uav_control_cli.c_str()); // frame for uav detecting april tags
			ros::param::get("~uav_slam_switch_topic",		uav.topics.s_slam_switch_cli);				fprintf (pFile,"trial.topics.uav_slam_switch_topic  = '%s';\n",	uav.topics.s_slam_switch_cli.c_str());
			ros::param::get("~uav_flight_state_topic",	uav.topics.s_uav_flight_state_cli);		fprintf (pFile,"trial.topics.uav_flight_state_topic = '%s';\n",	uav.topics.s_uav_flight_state_cli.c_str()); // frame for uav detecting april tags
			ros::param::get("~jointslam_switch_topic",	jointSLAM.s_slam_switch_cli); 				fprintf (pFile,"trial.topics.jointslam_switch_topic = '%s';\n",	jointSLAM.s_slam_switch_cli.c_str());

			ROS_INFO("  trial::uav.topics.s_uav_pose_state_sub  = %s",		uav.topics.s_uav_pose_state_sub.c_str());
			ROS_INFO("  trial::uav.topics.s_uav_control_cli  = %s",				uav.topics.s_uav_control_cli.c_str());
			ROS_INFO("  trial::uav.topics.s_slam_switch_cli  = '%s'", 		uav.topics.s_slam_switch_cli.c_str());
			ROS_INFO("  trial::jointSLAM.s_slam_switch_cli  = '%s'", 			jointSLAM.s_slam_switch_cli.c_str());

			uav.state_sub				= n.subscribe(uav.topics.s_uav_pose_state_sub, 1, &uavClass::readState, &uav); // read the estimate state of the uav
			uav.Waypoint_pub		= n.advertise<sensor_msgs::PointCloud2>(uav.topics.s_uav_waypoint_cloud_pub, 1);
			uav.control_cli			= n.serviceClient<hast::uavcontrolstate>	(uav.topics.s_uav_control_cli, true);
			uav.navState_cli		= n.serviceClient<hast::uavnavstate>			(uav.topics.s_uav_flight_state_cli, true);
			uav.slamswitch_cli	= n.serviceClient<hast::slamswitch>				(uav.topics.s_slam_switch_cli, true);
			jointSLAM.slamswitch_cli	= n.serviceClient<hast::slamswitch>	(jointSLAM.s_slam_switch_cli, true);


		/*----- UGV Channels */
			ros::param::get("~ugv1_TF_mux",ugv1.s_TF_mux);				fprintf (pFile,"trial.topics.ugv1.s_TF_mux    = '%s';\n",ugv1.s_TF_mux.c_str());
			ros::param::get("~ugv2_TF_mux",ugv2.s_TF_mux);				fprintf (pFile,"trial.topics.ugv2.s_TF_mux    = '%s';\n",ugv2.s_TF_mux.c_str());
			ros::param::get("~ugv1_TF_odom",ugv1.s_TF_odom);			fprintf (pFile,"trial.topics.ugv1.s_TF_odom   = '%s';\n",ugv1.s_TF_odom.c_str());
			ros::param::get("~ugv2_TF_odom",ugv2.s_TF_odom); 			fprintf (pFile,"trial.topics.ugv2.s_TF_odom   = '%s';\n",ugv2.s_TF_odom.c_str());
			ros::param::get("~ugv1_TF_global",ugv1.s_TF_global);	fprintf (pFile,"trial.topics.ugv1.s_TF_global = '%s';\n",ugv1.s_TF_global.c_str());
			ros::param::get("~ugv2_TF_global",ugv2.s_TF_global);	fprintf (pFile,"trial.topics.ugv2.s_TF_global = '%s';\n",ugv2.s_TF_global.c_str());

			ros::param::get("~ugv1_pose_sub",ugv1.s_pose_topic); fprintf (pFile,"trial.topics.ugv1.pose_topic   = '%s';\n",ugv1.s_pose_topic.c_str());
			ros::param::get("~ugv2_pose_sub",ugv2.s_pose_topic); fprintf (pFile,"trial.topics.ugv2.pose_topic   = '%s';\n",ugv2.s_pose_topic.c_str());
				ugv1.state_sub = n.subscribe(ugv1.s_pose_topic, 1, &ugvClass::readState, &ugv1); // read the estimate state of the ugv
				ugv2.state_sub = n.subscribe(ugv2.s_pose_topic, 1, &ugvClass::readState, &ugv2); // read the estimate state of the ugv

			ros::param::get("~ugv1_map_frame",ugv1.s_map_frame); fprintf (pFile,"trial.topics.ugv1.map_frame = '%s';\n",ugv1.s_map_frame.c_str());
			ros::param::get("~ugv2_map_frame",ugv2.s_map_frame); fprintf (pFile,"trial.topics.ugv2.map_frame = '%s';\n",ugv2.s_map_frame.c_str());
			ros::param::get("~ugv1_april_goal_id",ugv1.goal_tag_id); fprintf (pFile,"trial.topics.ugv1.april_goal_id = '%i';\n",ugv1.goal_tag_id);
			ros::param::get("~ugv2_april_goal_id",ugv2.goal_tag_id); fprintf (pFile,"trial.topics.ugv2.april_goal_id = '%i';\n",ugv2.goal_tag_id);
				ugv1.goal_call.request.ugv_id = "ugv1"; ugv1.goal_call.request.tagID = ugv1.goal_tag_id;
				ugv2.goal_call.request.ugv_id = "ugv2"; ugv2.goal_call.request.tagID = ugv2.goal_tag_id;
			// goal_tag_topic gets the position of the goal tag for th eugv from the slam/ckf node
			ros::param::get("~ugv1_goal_tag_topic",ugv1.s_goal_tag_topic); fprintf (pFile,"trial.topics.ugv1.goal_tag_topic = '%s';\n",ugv1.s_goal_tag_topic.c_str());
			ros::param::get("~ugv2_goal_tag_topic",ugv2.s_goal_tag_topic); fprintf (pFile,"trial.topics.ugv2.goal_tag_topic = '%s';\n",ugv2.s_goal_tag_topic.c_str());
				ugv1.GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>(ugv1.s_goal_tag_topic, true);
				ugv2.GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>(ugv2.s_goal_tag_topic, true);
			// ugv1_move_goal_topic sets the goal position of the ugv for move_base
			ros::param::get("~ugv1_move_goal_topic",ugv1.s_move_goal_topic); fprintf (pFile,"trial.topics.ugv1.move_goal_topic = '%s';\n",ugv1.s_move_goal_topic.c_str());
			ros::param::get("~ugv2_move_goal_topic",ugv2.s_move_goal_topic); fprintf (pFile,"trial.topics.ugv2.move_goal_topic = '%s';\n",ugv2.s_move_goal_topic.c_str());
				ugv1.goalpose_pub	= n.advertise<geometry_msgs::Pose2D>(ugv1.s_move_goal_topic.c_str(), 10);
				ugv2.goalpose_pub	= n.advertise<geometry_msgs::Pose2D>(ugv2.s_move_goal_topic.c_str(), 10);
			// clears the costmap of each vehicle
			ros::param::get("~ugv1_clearcostmap_topic",ugv1.s_clearcostmap_topic); fprintf (pFile,"trial.topics.ugv1.clearcostmap_topic = '%s';\n",ugv1.s_clearcostmap_topic.c_str());
			ros::param::get("~ugv2_clearcostmap_topic",ugv2.s_clearcostmap_topic); fprintf (pFile,"trial.topics.ugv2.clearcostmap_topic = '%s';\n",ugv2.s_clearcostmap_topic.c_str());
				ugv1.clearCostmaps_cli = n.serviceClient<hast::null>(ugv1.s_clearcostmap_topic, true);
				ugv2.clearCostmaps_cli = n.serviceClient<hast::null>(ugv2.s_clearcostmap_topic, true);
			// ugv1_goal_header_frame_id sets the goal header_frame_id of the ugv for move_base
			ros::param::get("~ugv1_goal_header_frame_id",ugv1.s_goal_header_frame_id); fprintf (pFile,"trial.topics.ugv1.goal_header_frame_id = '%s';\n",ugv1.s_goal_header_frame_id.c_str());
			ros::param::get("~ugv2_goal_header_frame_id",ugv2.s_goal_header_frame_id); fprintf (pFile,"trial.topics.ugv2.goal_header_frame_id = '%s';\n",ugv2.s_goal_header_frame_id.c_str());
				ugv1.goal.target_pose.header.frame_id = ugv1.s_goal_header_frame_id;
				ugv2.goal.target_pose.header.frame_id = ugv2.s_goal_header_frame_id;

			ros::param::get("~ugv1_movebase_status_topic",ugv1.s_movebase_status_topic); fprintf (pFile,"trial.topics.ugv1.ugv1_movebase_status_topic = '%s';\n",ugv1.s_movebase_status_topic.c_str());
			ros::param::get("~ugv2_movebase_status_topic",ugv2.s_movebase_status_topic); fprintf (pFile,"trial.topics.ugv2.ugv2_movebase_status_topic = '%s';\n",ugv2.s_movebase_status_topic.c_str());
				ugv1.move_base_status_sub = n.subscribe(ugv1.s_movebase_status_topic, 	1, &ugvClass::move_base_status_read, &ugv1);
				ugv2.move_base_status_sub = n.subscribe(ugv2.s_movebase_status_topic, 	1, &ugvClass::move_base_status_read, &ugv2);

			ros::param::get("~ugv1_local_plan_topic", ugv1.s_local_plan_topic); fprintf (pFile,"trial.topics.ugv1.s_local_plan_topic = '%s';\n",ugv1.s_local_plan_topic.c_str());
			ros::param::get("~ugv2_local_plan_topic", ugv2.s_local_plan_topic); fprintf (pFile,"trial.topics.ugv2.s_local_plan_topic = '%s';\n",ugv2.s_local_plan_topic.c_str());
				ugv1.local_plan_sub = n.subscribe(ugv1.s_local_plan_topic, 	1, &hastTrial::ugv1_updateLocalPath, this);
				ugv2.local_plan_sub = n.subscribe(ugv2.s_local_plan_topic, 	1, &hastTrial::ugv2_updateLocalPath, this);

			ros::param::get("~ugv1_global_plan_topic", ugv1.s_global_plan_topic); fprintf (pFile,"trial.topics.ugv1.s_global_plan_topic = '%s';\n",ugv1.s_global_plan_topic.c_str());
			ros::param::get("~ugv2_global_plan_topic", ugv2.s_global_plan_topic); fprintf (pFile,"trial.topics.ugv2.s_global_plan_topic = '%s';\n",ugv2.s_global_plan_topic.c_str());
				ugv1.global_plan_sub = n.subscribe(ugv1.s_global_plan_topic, 	1, &hastTrial::ugv1_updateGlobalPath, this);
				ugv2.global_plan_sub = n.subscribe(ugv2.s_global_plan_topic, 	1, &hastTrial::ugv2_updateGlobalPath, this);

			ros::param::get("~ugv1_mux_map_serve_topic", ugv1.s_ugv_mux_map_serve_topic); fprintf (pFile,"trial.topics.ugv1.s_ugv_mux_map_serve_topic = '%s';\n",ugv1.s_ugv_mux_map_serve_topic.c_str());
			ros::param::get("~ugv2_mux_map_serve_topic", ugv2.s_ugv_mux_map_serve_topic); fprintf (pFile,"trial.topics.ugv2.s_ugv_mux_map_serve_topic = '%s';\n",ugv2.s_ugv_mux_map_serve_topic.c_str());
				ugv1.callMuxMap_cli = n.serviceClient<hast::servmuxmap>(ugv1.s_ugv_mux_map_serve_topic, true);
				ugv2.callMuxMap_cli = n.serviceClient<hast::servmuxmap>(ugv2.s_ugv_mux_map_serve_topic, true);

			ros::param::get("~ugv1_driver_goal_topic", ugv1.driver_goal.topic); fprintf (pFile,"trial.topics.ugv1.driver_goal_topic = '%s';\n",ugv1.driver_goal.topic.c_str());
			ros::param::get("~ugv2_driver_goal_topic", ugv2.driver_goal.topic); fprintf (pFile,"trial.topics.ugv2.driver_goal_topic = '%s';\n",ugv2.driver_goal.topic.c_str());
				ugv1.driver_goal.client = n.serviceClient<hast::ugvdrive>(ugv1.driver_goal.topic, true);
				ugv2.driver_goal.client = n.serviceClient<hast::ugvdrive>(ugv2.driver_goal.topic, true);
				ugv1.driver_goal.call.request.goalP.x = 0; ugv1.driver_goal.call.request.goalP.y = 0; ugv1.driver_goal.call.request.goalP.z = 0;
				ugv2.driver_goal.call.request.goalP.x = 0; ugv2.driver_goal.call.request.goalP.y = 0; ugv2.driver_goal.call.request.goalP.z = 0;
				ugv1.driver_goal.call.request.goalYaw = 0; ugv2.driver_goal.call.request.goalYaw = 0;
				ugv1.driver_goal.call_fails = 0; ugv2.driver_goal.call_fails = 0;

			ros::param::get("~ugv1_driver_status_topic", ugv1.driver_status.topic); fprintf (pFile,"trial.topics.ugv1.driver_status_topic = '%s';\n",ugv1.driver_status.topic.c_str());
			ros::param::get("~ugv2_driver_status_topic", ugv2.driver_status.topic); fprintf (pFile,"trial.topics.ugv2.driver_status_topic = '%s';\n",ugv2.driver_status.topic.c_str());
				ugv1.driver_status.sub = n.subscribe(ugv1.driver_status.topic, 	1, &ugvClass::driver_status_read, &ugv1);
				ugv2.driver_status.sub = n.subscribe(ugv2.driver_status.topic, 	1, &ugvClass::driver_status_read, &ugv2);
				ugv1.driver_status.msg.at_yaw = false; ugv1.driver_status.msg.at_pos = false;
				ugv2.driver_status.msg.at_yaw = false; ugv2.driver_status.msg.at_pos = false;

		/*----- trial Channels */
			// HastShutDown_pub 	= n.advertise<hast::flag>("/hast/shutdown", 10);
			KillHast_sub	 	= n.subscribe("/hast/shutdown",1, &hastTrial::killhast, this);

			ros::param::get("~trial_running_topic", endTrial.s_trial_running_ser); fprintf (pFile,"trial.topics.endTrial.s_trial_running_ser = '%s';\n",endTrial.s_trial_running_ser.c_str());
			endTrial.trial_running_ser 		= n.advertiseService(endTrial.s_trial_running_ser, 	&hastTrial::is_running,  this);

			ON = true;
			OFF = false;
			ShutDown.flag = ON;

			endTrial.Kill_msg.flag = true;

			endTrial.DroneCmd_dr_msg.linear.x = 0.0;	endTrial.DroneCmd_dr_msg.linear.y = 0.0;	endTrial.DroneCmd_dr_msg.linear.z = 0.0;
			endTrial.DroneCmd_dr_msg.angular.z = 0.0; endTrial.DroneCmd_dr_msg.angular.x = 0.0; endTrial.DroneCmd_dr_msg.angular.y = 0.0;

			ros::param::get("~killswitch_topic", 	endTrial.s_killswitch_topic);		fprintf (pFile,"trial.killswitch_topic = '%s';\n",	endTrial.s_killswitch_topic.c_str());
			ros::param::get("~uav_land_topic",		endTrial.s_uav_land_topic);			fprintf (pFile,"trial.uav_land_topic = '%s';\n",		endTrial.s_uav_land_topic.c_str());
			ros::param::get("~uav_zerocmd_topic", endTrial.s_uav_zerocmd_topic);	fprintf (pFile,"trial.uav_zerocmd_topic = '%s';\n",	endTrial.s_uav_zerocmd_topic.c_str());

			endTrial.HastKill				 = n.advertise<hast::flag>(endTrial.s_killswitch_topic, 1000);
			endTrial.DroneLand			 = n.advertise<std_msgs::Empty>(endTrial.s_uav_land_topic, 1000);
			endTrial.DroneCmd_dr_pub = n.advertise<geometry_msgs::Twist>(endTrial.s_uav_zerocmd_topic, 1000);

		/*--------- Initialize Flight Variables ------------- */
		// uav.EstPosition_gl = (cv::Mat_<double>(3,1) << 2.5,0,0);
			FlightFlag = true;
			FlyToCount = 0;
			FlyToYaw = 0;
			FlyToYawLast = 0;

		/*--------- trial parameters ------------- */
		// wtf
			if(ros::param::get("~start_wait",start_wait)){} 									else {start_wait = 13;}
			if(ros::param::get("~gazebo",gazebo)){} 													else {gazebo = false;}
			if(ros::param::get("~iflift",iflift)){} 													else {iflift = false;}							fprintf (pFile,"trial.exp.iflift = '%s';\n",					iflift ? "true" : "false" );							ROS_INFO("trial.exp.         iflift = %s", iflift ? "true" : "false" );
			if(ros::param::get("~slamtest",slamtest)){} 											else {slamtest = false;};						fprintf (pFile,"trial.exp.slamtest = '%s';\n",				slamtest ? "true" : "false" );						ROS_INFO("trial.exp.       slamtest = %s", slamtest ? "true" : "false" );
			if(ros::param::get("~exp/oneUGV",					exp.oneUGV)){} 					else {exp.oneUGV = false;}					fprintf (pFile,"trial.exp.oneUGV = '%s';\n",					exp.oneUGV ? "true" : "false" );					ROS_INFO("trial.exp.         oneUGV = %s", exp.oneUGV ? "true" : "false" );
			if(ros::param::get("~exp/ugv1_lookatgoal",exp.ugv1_lookatgoal)){}	else {exp.ugv1_lookatgoal = false;}	fprintf (pFile,"trial.exp.ugv1_lookatgoal = '%s';\n",	exp.ugv1_lookatgoal ? "true" : "false" );	ROS_INFO("trial.exp.ugv1_lookatgoal = %s", exp.ugv1_lookatgoal ? "true" : "false" );
			if(ros::param::get("~exp/ugv2_lookatgoal",exp.ugv2_lookatgoal)){}	else {exp.ugv2_lookatgoal = false;}	fprintf (pFile,"trial.exp.ugv2_lookatgoal = '%s';\n",	exp.ugv2_lookatgoal ? "true" : "false" );	ROS_INFO("trial.exp.ugv2_lookatgoal = %s", exp.ugv2_lookatgoal ? "true" : "false" );
			if(ros::param::get("~exp/ugv1_watchugv2",	exp.ugv1_watchugv2)){}	else {exp.ugv1_watchugv2 = false;}	fprintf (pFile,"trial.exp.ugv1_watchugv2  = '%s';\n",	exp.ugv1_watchugv2 ? "true" : "false" );	ROS_INFO("trial.exp. ugv1_watchugv2 = %s", exp.ugv1_watchugv2 ? "true" : "false" );
			if(ros::param::get("~exp/ugv2_watchugv1",	exp.ugv2_watchugv1)){}	else {exp.ugv2_watchugv1 = false;}	fprintf (pFile,"trial.exp.ugv2_watchugv1  = '%s';\n",	exp.ugv2_watchugv1 ? "true" : "false" );	ROS_INFO("trial.exp. ugv2_watchugv1 = %s", exp.ugv2_watchugv1 ? "true" : "false" );
			if(ros::param::get("~exp/ugv1_picket",		exp.ugv1_picket)){}			else {exp.ugv1_picket = false;}			fprintf (pFile,"trial.exp.ugv1_picket     = '%s';\n",	exp.ugv1_picket ? "true" : "false" );			ROS_INFO("trial.exp.    ugv1_picket = %s", exp.ugv1_picket ? "true" : "false" );
			if(ros::param::get("~exp/ugv2_picket",		exp.ugv2_picket)){}			else {exp.ugv2_picket = false;}			fprintf (pFile,"trial.exp.ugv2_picket     = '%s';\n",	exp.ugv2_picket ? "true" : "false" );			ROS_INFO("trial.exp.    ugv2_picket = %s", exp.ugv2_picket ? "true" : "false" );
			if(ros::param::get("~exp/ugv1_w_hover",		exp.ugv1_w_hover)){}		else {exp.ugv1_w_hover = false;}		fprintf (pFile,"trial.exp.ugv1_w_hover    = '%s';\n",	exp.ugv1_w_hover ? "true" : "false" );		ROS_INFO("trial.exp.   ugv1_w_hover = %s", exp.ugv1_w_hover ? "true" : "false" );
			if(ros::param::get("~exp/ugv2_w_hover",		exp.ugv2_w_hover)){}		else {exp.ugv2_w_hover = false;}		fprintf (pFile,"trial.exp.ugv2_w_hover    = '%s';\n",	exp.ugv2_w_hover ? "true" : "false" );		ROS_INFO("trial.exp.   ugv2_w_hover = %s", exp.ugv2_w_hover ? "true" : "false" );
			if(ros::param::get("~exp/both_w_hover",		exp.both_w_hover)){}		else {exp.both_w_hover = false;}		fprintf (pFile,"trial.exp.both_w_hover    = '%s';\n",	exp.both_w_hover ? "true" : "false" );		ROS_INFO("trial.exp.   both_w_hover = %s", exp.both_w_hover ? "true" : "false" );

			ROS_WARN("trial.exp.   s_exp_code = %s", s_exp_code.c_str() );
	}

	/*####################        Start Experiment       #################### */
	void RunExperiment()
	{
		/*---------------------- Pre-Flight Preparations ---------------------- */
		spin_sleep(start_wait);
		/*----------------------  Begin Experiment ---------------------- */
		ROS_INFO("trial: ! ! ! Starting trial ! ! !");
		fprintf (pFile,"  trial.complete = false;\n");

		fprintf (pFile,"trial.start_time = %6.10f;\n", ros::Time::now().toSec()-init_time);

		uav.liftoff();
		if (uav.takeoffError)	{endExperiment();}

		while (!uav.state_bool){
			// fprintf(stdout, (BLUE_TEXT "trial: uav.state_bool  = %s\n" COLOR_RESET ), uav.state_bool ? "true" : "false");
			ROS_INFO("trial: uav.state_bool  = %s", uav.state_bool ? "true" : "false" );
			spin_sleep(0.25);
			if ((uav.navStateUint==0) || (uav.navStateUint==2)){uav.state_bool=false;} else {uav.state_bool=true;}
		}

		if (exp.oneUGV && (s_exp_code == "A")) {experiment_ugv1_w_hover();}
		if (exp.oneUGV && (s_exp_code == "B")) {experiment_ugv1_picket();}
		if (exp.ugv1_w_hover) {experiment_ugv1_w_hover();}
		if (exp.ugv2_w_hover) {experiment_ugv2_w_hover();}
		if (exp.ugv1_picket)  {experiment_ugv1_picket();}
		if (exp.ugv2_picket)  {experiment_ugv2_picket();}
		if (exp.both_w_hover) {experiment_both_w_hover();}

		/*----------------------  End Experiment  ---------------------- */
		endExperiment();
	}

	void ugv1_turn()
	{fprintf (pFile,"%% ~~~ ugv1_turn()\n");	ROS_INFO("trial: ugv1_turn()");
		ugv1.driver_goal.call_fails = 0;
		ugv1.driver_status.msg.at_yaw = false;
		ugv1.driver_status.msg.at_pos = false;

		bool ugv1_response = ugv1.call_tag_location(ugv2.goal_tag_id, false); // tag location for UGV2because that is where UGV1 will look
		fprintf (pFile,"  %% ugv1_p = [%6.10f %6.10f];\n", ugv1.EstPosition_gl.at<double>(0,0), ugv1.EstPosition_gl.at<double>(1,0));
		fprintf (pFile,"  %% goal_p = [%6.10f %6.10f];\n", ugv1.goal_call.response.goalP.x, ugv1.goal_call.response.goalP.y);

		ugv1.driver_goal.call.request.goalYaw = atan2(ugv1.goal_call.response.goalP.y - ugv1.EstPosition_gl.at<double>(1,0), ugv1.goal_call.response.goalP.x - ugv1.EstPosition_gl.at<double>(0,0));

		ROS_INFO("trial: calling ugv1.driver_goal.client");
		ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call);
		// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
		while (!ugv1.driver_goal.call_bool && ugv1.driver_goal.call_fails < 10)
		{
			ugv1.driver_goal.call_fails +=1;
			ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call);
			// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
			spin_sleep(0.1);
		}

		if (ugv1.driver_goal.call_bool)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			while ((!ugv1.driver_status.msg.at_yaw) && (ElapsedTime < 600) )  { // ugv are still moving
				fprintf (pFile,"  trial.ugv1.turn.time (%i,1) = %6.10f;\n", ++ugv1.turn_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv1.turn.goal (%i,1) = %6.10f;\n", ugv1.turn_count, ugv1.driver_goal.call.request.goalYaw);
				spin_sleep(0.25);
				ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			}
		}
	}

	void ugv2_turn()
	{fprintf (pFile,"%% ~~~ ugv2_turn()\n");	ROS_INFO("trial: ugv2_turn()");
		ugv2.driver_goal.call_fails = 0;
		ugv2.driver_status.msg.at_yaw = false;
		ugv2.driver_status.msg.at_pos = false;

		bool ugv2_response = ugv2.call_tag_location(ugv1.goal_tag_id, false); // tag location for UGV2because that is where UGV1 will look
		fprintf (pFile,"  %% ugv2_p = [%6.10f %6.10f];\n", ugv2.EstPosition_gl.at<double>(0,0), ugv2.EstPosition_gl.at<double>(1,0));
		fprintf (pFile,"  %% goal_p = [%6.10f %6.10f];\n", ugv2.goal_call.response.goalP.x, ugv2.goal_call.response.goalP.y);

		ugv2.driver_goal.call.request.goalYaw = atan2(ugv2.goal_call.response.goalP.y - ugv2.EstPosition_gl.at<double>(1,0), ugv2.goal_call.response.goalP.x - ugv2.EstPosition_gl.at<double>(0,0));
		// ROS_INFO("trial: ugv1.driver_goal.call.request.goalYaw: %6.4f", ugv1.driver_goal.call.request.goalYaw);

		ROS_INFO("trial: calling ugv1.driver_goal.client");
		ugv2.driver_goal.call_bool = ugv2.driver_goal.client.call(ugv2.driver_goal.call);
		// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
		while (!ugv2.driver_goal.call_bool && ugv2.driver_goal.call_fails < 10)
		{
			ugv2.driver_goal.call_fails +=1;
			ugv2.driver_goal.call_bool = ugv2.driver_goal.client.call(ugv2.driver_goal.call);
			// fprintf (pFile,"%% ugv1.driver_goal.call_bool = ugv1.driver_goal.client.call(ugv1.driver_goal.call) = '%s';\n", ugv1.driver_goal.call_bool ? "true" : "false" );
			spin_sleep(0.1);
		}

		if (ugv2.driver_goal.call_bool)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			while ((!ugv2.driver_status.msg.at_yaw) && (ElapsedTime < 600) )  { // ugv are still moving
				fprintf (pFile,"  trial.ugv2.turn.time (%i,1) = %6.10f;\n", ++ugv2.turn_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv2.turn.goal (%i,1) = %6.10f;\n",   ugv2.turn_count, ugv2.driver_goal.call.request.goalYaw);
				spin_sleep(0.25);
				ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			}
		}
	}

	void UAV_search_path()
	{

		if (!gazebo){ //vicon uav waypoints
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
		} else { // gazebo uav waypoints
			// should be roughly here ... << 1.75, 0.0, 1.10), uav.EstYaw_gl* uav.Pi / 180, 0.00, uav.flytime);
			uav.flytime = 10;
			uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, 0.0);
			// uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, ugv1.goal_call.response.goalYaw);
			UAVstepTo((cv::Mat_<double>(3,1) << 1.50,  0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 1.50, -0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 2.25, -0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 2.25,  0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 3.00,  0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 3.00, -0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 3.75, -0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 3.75,  0.5, 1.10), 0.0, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 2.00,  0.0, 1.10), 0.0, uav.flytime);
		}
	}

	void experimentSlamTest()
	{fprintf (pFile,"%% ~~~ experimentSlamTest()\n");	ROS_INFO("trial: experimentSlamTest()");
		if (!gazebo){ //vicon uav waypoints
			uav.flytime = 8;
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
		} else { // gazebo uav waypoints
			uav.flytime = 12;
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
		}

		slamSwitch();

		if (!gazebo){ //vicon uav waypoints
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.50, 0.0, 1.00), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
		} else { // gazebo uav waypoints
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.50, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
		}
	}

	void experiment_ugv1_picket()
	{fprintf (pFile,"%% ~~~ experiment_ugv1_picket()\n");	ROS_INFO("trial: experiment_ugv1_picket()");
		if (!gazebo){ //vicon uav waypoints
			uav.flytime = 8;
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180 , 0.00, uav.flytime);
		} else { // gazebo uav waypoints
			uav.flytime = 12;
			UAVstepTo((cv::Mat_<double>(3,1) << 1.750,  0.0, 1.10), 0.0, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl* uav.Pi / 180, 0.00, uav.flytime);
		}

		slamSwitch();
		UAV_search_path();

		//ugv1 turn towards goal location
		if(exp.ugv2_lookatgoal) {ugv2_turn();}

		//ugv now can move
		bool ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
		fprintf (pFile,"%% ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_response ? "true" : "false" );

		int ugv1_fails = 0;
		while ((!ugv1_response && ugv1_fails < 10))
		{
			if (!ugv1_response && ugv1_fails < 10)
			{
				ugv1_fails +=1;
				ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
				fprintf (pFile,"%% ugv1_response(%i) = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_fails, ugv1_response ? "true" : "false" );
				spin_sleep(0.1);
			}
		}

		fprintf (pFile,"  trial.ugv1.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), ugv1.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv1.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goal_for_ugv_msg.x, ugv1.goal_for_ugv_msg.y, ugv1.goal_for_ugv_msg.theta);

		ugv2.holdsPicket = false; ugv1.holdsPicket = true;
		// preseed picket_end to prevent crashing
		ugv1.picket_end = (cv::Mat_<double>(3, 1) << uav.control_call.request.dP_g.x, uav.control_call.request.dP_g.y, uav.control_call.request.dP_g.z);

		if (ugv1_response)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;

			ROS_INFO("while (ugv1.actionStatus == 0 || ugv1.actionStatus == 1) {");
			double picket_yaw;
			cv::Mat picket_goal;
			while ((ugv1.actionStatus == 0 || ugv1.actionStatus == 1 ) && (ElapsedTime < 600) )  { // ugv is still moving
				spin_sleep(0.25);
				picket_yaw = ugv1.EstYaw_gl + ugv1.yaw_mid;
				picket_goal = (cv::Mat_<double>(3,1) << ugv1.picket_mid.at<double>(0,0), ugv1.picket_mid.at<double>(1,0), ugv1.picket_mid.at<double>(2,0));
				uav.setDesiredPosition(picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0), 0.0);
				// uav.setDesiredPosition(picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0), picket_yaw);
				if (exp.ugv2_watchugv1 && ugv1.local_plan_bool) {	ugv2.turnTo(atan2(ugv1.path_end_gl.at<double>(1,0) - ugv2.EstPosition_gl.at<double>(1,0),
																																						ugv1.path_end_gl.at<double>(0,0) - ugv2.EstPosition_gl.at<double>(0,0)));}
				ugv1.picket_count += 1;
				fprintf (pFile,"\n%% ~~~~ ugv1 picket  ~~~~ \n");
				fprintf (pFile,"  trial.ugv1.picket_time(%d,1) = %f;\n", ugv1.picket_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv1.actionStatus(%d,1)  = %i;\n", ugv1.picket_count, ugv1.actionStatus);
				fprintf (pFile,"  trial.ugv1.picket_yaw(%d,1)  = %f;\n", ugv1.picket_count, picket_yaw);
				fprintf (pFile,"  trial.ugv1.uav_goal_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.picket_count, picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0));
				fprintf (pFile,"  trial.ugv1.path_end_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.picket_count, ugv1.path_end_gl.at<double>(0,0), ugv1.path_end_gl.at<double>(1,0), ugv1.path_end_gl.at<double>(2,0));
			}
		}
	}

	void experiment_ugv2_picket()
	{fprintf (pFile,"%% ~~~ experiment_ugv2_picket()\n");	ROS_INFO("trial: experiment_ugv2_picket()");
		if (!gazebo){ //vicon uav waypoints
			uav.flytime = 8;
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180 , 0.00, uav.flytime);
		} else { // gazebo uav waypoints
			uav.flytime = 12;
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.50, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 2.00, 0.0, 1.20), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl* uav.Pi / 180, 0.00, uav.flytime);
			UAVstepTo((cv::Mat_<double>(3,1) << 1.75,  0.0, 1.10), 0.0, uav.flytime);
		}

		slamSwitch();
		UAV_search_path();

		//ugv1 turn towards goal location
		if(exp.ugv1_lookatgoal) {ugv1_turn();}

		//ugv now can move
		bool ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
		fprintf (pFile,"%% ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_response ? "true" : "false" );

		int ugv2_fails = 0;
		while ((!ugv2_response && ugv2_fails < 10))
		{
			if (!ugv2_response && ugv2_fails < 10)
			{
				ugv2_fails +=1;
				ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
				fprintf (pFile,"%% ugv2_response(%i) = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_fails, ugv2_response ? "true" : "false" );
				spin_sleep(0.1);
			}
		}

		fprintf (pFile,"  trial.ugv2.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goalP_gl_tf.x(), ugv2.goalP_gl_tf.y(), ugv2.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv2.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goal_for_ugv_msg.x, ugv2.goal_for_ugv_msg.y, ugv2.goal_for_ugv_msg.theta);

		ugv2.holdsPicket = true; ugv1.holdsPicket = false;
		// preseed picket_end to prevent crashing
		ugv2.picket_end = (cv::Mat_<double>(3, 1) << uav.control_call.request.dP_g.x, uav.control_call.request.dP_g.y, uav.control_call.request.dP_g.z);

		if (ugv2_response)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;

			ROS_INFO("while (ugv2.actionStatus == 0 || ugv2.actionStatus == 1) {");
			double picket_yaw;
			cv::Mat picket_goal;
			while ((ugv2.actionStatus == 0 || ugv2.actionStatus == 1 ) && (ElapsedTime < 600) )  { // ugv is still moving
				spin_sleep(0.25);
				picket_yaw = ugv2.EstYaw_gl + ugv2.yaw_mid;
				picket_goal = (cv::Mat_<double>(3,1) << ugv2.picket_mid.at<double>(0,0), ugv2.picket_mid.at<double>(1,0), ugv2.picket_mid.at<double>(2,0));
				uav.setDesiredPosition(picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0), 0.0);
				// uav.setDesiredPosition(picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0), picket_yaw);
				if (exp.ugv1_watchugv2 && ugv2.local_plan_bool) {	ugv1.turnTo(atan2(ugv2.path_end_gl.at<double>(1,0) - ugv1.EstPosition_gl.at<double>(1,0),
																																						ugv2.path_end_gl.at<double>(0,0) - ugv1.EstPosition_gl.at<double>(0,0)));}
				ugv2.picket_count += 1;
				fprintf (pFile,"\n%% ~~~~ ugv2 picket  ~~~~ \n");
				fprintf (pFile,"  trial.ugv2.picket_time(%d,1) = %f;\n", ugv2.picket_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv2.actionStatus(%d,1)  = %i;\n", ugv2.picket_count, ugv2.actionStatus);
				fprintf (pFile,"  trial.ugv2.picket_yaw(%d,1)  = %f;\n", ugv2.picket_count, picket_yaw);
				fprintf (pFile,"  trial.ugv2.uav_goal_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.picket_count, picket_goal.at<double>(0,0), picket_goal.at<double>(1,0), picket_goal.at<double>(2,0));
				fprintf (pFile,"  trial.ugv2.ugv1_est_p(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.picket_count, ugv1.EstPosition_gl.at<double>(0,0), ugv1.EstPosition_gl.at<double>(1,0), ugv1.EstPosition_gl.at<double>(2,0));
				fprintf (pFile,"  trial.ugv2.path_end_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.picket_count, ugv2.path_end_gl.at<double>(0,0), ugv2.path_end_gl.at<double>(1,0), ugv2.path_end_gl.at<double>(2,0));
			}
		}
	}

	void experiment_ugv1_w_hover()
	{fprintf (pFile,"%% ~~~ experiment_ugv1_w_hover()\n");	ROS_INFO("trial: experiment_ugv1_w_hover()");
		if (!gazebo) //vicon uav waypoints
		{ uav.flytime = 8;	linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);	}
		else // gazebo uav waypoints
		{ uav.flytime = 12;	UAVstepTo((cv::Mat_<double>(3,1) << 1.750,  0.0, 1.10), 0.0, uav.flytime);}

		slamSwitch();
		UAV_search_path();

		//ugv1 turn towards goal location
		if(exp.ugv2_lookatgoal) {ugv2_turn();}

		//ugv now can move
		bool ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
		fprintf (pFile,"%% ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_response ? "true" : "false" );

		int ugv1_fails = 0;
		while ((!ugv1_response && ugv1_fails < 10))
		{
			if (!ugv1_response && ugv1_fails < 10)
			{
				ugv1_fails +=1;
				ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
				fprintf (pFile,"%% ugv1_response(%i) = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_fails, ugv1_response ? "true" : "false" );
				spin_sleep(0.1);
			}
		}

		fprintf (pFile,"  trial.ugv1.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goalP_gl_tf.x(), 		ugv1.goalP_gl_tf.y(),			ugv1.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv1.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goal_for_ugv_msg.x,	ugv1.goal_for_ugv_msg.y,	ugv1.goal_for_ugv_msg.theta);

		ugv2.holdsPicket = false; ugv1.holdsPicket = false;
		uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, 0.0);
		// uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, ugv1.goal_call.response.goalYaw);
		if (ugv1_response)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			ugv1.action_count = 0;
			while ((ugv1.actionStatus == 0 || ugv1.actionStatus == 1 ) && (ElapsedTime < 600) )  { // ugv is still moving
				spin_sleep(0.25);
				ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
				if (exp.ugv2_watchugv1 && ugv1.local_plan_bool) {	ugv2.turnTo(atan2(ugv1.path_end_gl.at<double>(1,0) - ugv2.EstPosition_gl.at<double>(1,0),
																																						ugv1.path_end_gl.at<double>(0,0) - ugv2.EstPosition_gl.at<double>(0,0)));}
				ugv1.action_count += 1;
				fprintf (pFile,"\n%% ~~~~ ugv1 action  ~~~~ \n");
				fprintf (pFile,"  trial.ugv1.action_time(%d,1) = %f;\n", ugv1.action_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv1.actionStatus(%d,1)  = %i;\n", ugv1.action_count, ugv1.actionStatus);

			}
		}

	}

	void experiment_ugv2_w_hover()
	{fprintf (pFile,"%% ~~~ experiment_ugv2_w_hover()\n");	ROS_INFO("trial: experiment_ugv2_w_hover()");
		if (!gazebo) //vicon uav waypoints
		{ uav.flytime = 8;	linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);	}
		else // gazebo uav waypoints
		{ uav.flytime = 12;	UAVstepTo((cv::Mat_<double>(3,1) << 1.750,  0.0, 1.10), 0.0, uav.flytime);}
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl  * uav.Pi / 180, 0.00, uav.flytime);	}

		slamSwitch();
		UAV_search_path();

		//ugv1 turn towards goal location
		if(exp.ugv1_lookatgoal) {ugv1_turn();}

		//ugv now can move
		bool ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
		fprintf (pFile,"%% ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_response ? "true" : "false" );

		int ugv2_fails = 0;
		while ((!ugv2_response && ugv2_fails < 10))
		{
			if (!ugv2_response && ugv2_fails < 10)
			{
				ugv2_fails +=1;
				ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
				fprintf (pFile,"%% ugv2_response(%i) = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_fails, ugv2_response ? "true" : "false" );
				spin_sleep(0.1);
			}
		}

		fprintf (pFile,"  trial.ugv2.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goalP_gl_tf.x(), ugv2.goalP_gl_tf.y(), ugv2.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv2.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goal_for_ugv_msg.x,  ugv2.goal_for_ugv_msg.y,  ugv2.goal_for_ugv_msg.theta);

		ugv2.holdsPicket = false; ugv1.holdsPicket = false;
		uav.setDesiredPosition(ugv2.goalP_gl_tf.x(), ugv2.goalP_gl_tf.y(), 1.1, 0.0);
		// uav.setDesiredPosition(ugv2.goalP_gl_tf.x(), ugv2.goalP_gl_tf.y(), 1.1, ugv2.goal_call.response.goalYaw);
		if (ugv2_response)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			ugv2.action_count = 0;
			while ((ugv2.actionStatus == 0 || ugv2.actionStatus == 1 ) && (ElapsedTime < 600) )  { // ugv is still moving
				spin_sleep(0.25);
				ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
				if (exp.ugv1_watchugv2 && ugv2.local_plan_bool) {	ugv1.turnTo(atan2(ugv2.path_end_gl.at<double>(1,0) - ugv1.EstPosition_gl.at<double>(1,0),
																																						ugv2.path_end_gl.at<double>(0,0) - ugv1.EstPosition_gl.at<double>(0,0)));}
				ugv2.action_count += 1;
				fprintf (pFile,"\n%% ~~~~ ugv2 action  ~~~~ \n");
				fprintf (pFile,"  trial.ugv2.action_time(%d,1) = %f;\n", ugv2.action_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv2.actionStatus(%d,1)  = %i;\n", ugv2.action_count, ugv2.actionStatus);

			}
		}

	}

	void experiment_both_w_hover() // both ugvs move simultaneously
	{fprintf (pFile,"%% ~~~ experiment_both_w_hover()\n");	ROS_INFO("trial: experiment_both_w_hover()");
		if (!gazebo) //vicon uav waypoints
		{ uav.flytime = 8;	linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);	}
		else // gazebo uav waypoints
		{ uav.flytime = 12;	UAVstepTo((cv::Mat_<double>(3,1) << 1.750,  0.0, 1.10), 0.0, uav.flytime);}
			// linearTrajTo(uav.EstPosition_gl, (cv::Mat_<double>(3,1) << 1.75, 0.0, 1.10), uav.EstYaw_gl * uav.Pi / 180, 0.00, uav.flytime);	}

		slamSwitch();
		UAV_search_path();

		bool ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
		bool ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
		fprintf (pFile,"%% ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_response ? "true" : "false" );
		fprintf (pFile,"%% ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_response ? "true" : "false" );

		int ugv1_fails = 0; int ugv2_fails = 0;
		while ((!ugv1_response && ugv1_fails < 10) || (!ugv2_response && ugv2_fails < 10))
		{
			if (!ugv1_response && ugv1_fails < 10)
			{
				ugv1_fails +=1;
				ugv1_response = ugv1.call_tag_location(ugv1.goal_tag_id, true);
				fprintf (pFile,"%% ugv1_response(%i) = ugv1.call_tag_location(ugv1.goal_tag_id, true) = '%s';\n", ugv1_fails, ugv1_response ? "true" : "false" );
				spin_sleep(0.1);
			}
			if (!ugv2_response && ugv2_fails < 10)
			{
				ugv2_fails +=1;
				ugv2_response = ugv2.call_tag_location(ugv2.goal_tag_id, true);
				fprintf (pFile,"%% ugv2_response(%i) = ugv2.call_tag_location(ugv2.goal_tag_id, true) = '%s';\n", ugv2_fails, ugv2_response ? "true" : "false" );
				spin_sleep(0.1);
			}
		}

		fprintf (pFile,"  trial.ugv1.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), ugv1.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv1.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.goal_for_ugv_msg.x,  ugv1.goal_for_ugv_msg.y,  ugv1.goal_for_ugv_msg.theta);
		fprintf (pFile,"  trial.ugv2.goal_gl   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goalP_gl_tf.x(), ugv2.goalP_gl_tf.y(), ugv2.goal_call.response.goalYaw);
		fprintf (pFile,"  trial.ugv2.goal_odom = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.goal_for_ugv_msg.x,  ugv2.goal_for_ugv_msg.y,  ugv2.goal_for_ugv_msg.theta);

		ugv2.holdsPicket = false; ugv1.holdsPicket = false;
		// preseed picket_end to prevent crashing
		uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, 0.0);
		// uav.setDesiredPosition(ugv1.goalP_gl_tf.x(), ugv1.goalP_gl_tf.y(), 1.1, ugv1.goal_call.response.goalYaw);

		// ROS_INFO("while (ugv2.actionStatus == 0 || ugv2.actionStatus == 1) {");
		if (ugv1_response && ugv2_response)
		{
			double MoveStartTime = ros::Time::now().toSec();
			spin_sleep(0.1);
			double ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
			ugv1.action_count = 0; ugv2.action_count = 0;
			while ((ugv1.actionStatus == 0 || ugv1.actionStatus == 1 || ugv2.actionStatus == 0 || ugv2.actionStatus == 1 ) && (ElapsedTime < 600) )  { // ugv are still moving
				spin_sleep(0.25);
				ElapsedTime = ros::Time::now().toSec() - MoveStartTime;
				ugv1.action_count += 1; ugv2.action_count += 1;
				fprintf (pFile,"\n%% ~~~~ ugv action  ~~~~ \n");
				fprintf (pFile,"  trial.ugv1.action_time(%d,1) = %f;\n", ugv1.action_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv1.actionStatus(%d,1)  = %i;\n", ugv1.action_count, ugv1.actionStatus);
				fprintf (pFile,"  trial.ugv2.action_time(%d,1) = %f;\n", ugv2.action_count, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.ugv2.actionStatus(%d,1)  = %i;\n", ugv2.action_count, ugv2.actionStatus);

			}
		}

	}

	void slamSwitch()
	{ROS_WARN("slamswitch(); ...");

		uav.pose_msg.position.x = uav.EstPosition_gl.at<double>(0, 0);
		uav.pose_msg.position.y = uav.EstPosition_gl.at<double>(1, 0);
		uav.pose_msg.position.z = uav.EstPosition_gl.at<double>(2, 0);
		uav.pose_msg.orientation = tf::createQuaternionMsgFromYaw(uav.Pi * uav.EstYaw_gl / 180);
		uav.pose_msg.header.seq = ++uav.pose_msg_seq;
		uav.pose_msg.header.stamp = ros::Time::now();
		uav.pose_msg.header.frame_id = "/map";

		uav.slamswitch_call.request.flip = true; // Set to activate slam
		jointSLAM.slamswitch_call.request.flip = true; // Set to activate slam

		uav.slamswitch_call.request.pose = uav.pose_msg;
		jointSLAM.slamswitch_call.request.pose = uav.pose_msg;

		fprintf (pFile,"\n%% ~~~~ slamSwitch() ~~~~ \n");
		fprintf (pFile,"  trial.slam.uav.EstPosition_gl(1,:) = [%6.10f %6.10f %6.10f];\n",uav.EstPosition_gl.at<double>(0, 0),uav.EstPosition_gl.at<double>(1, 0),uav.EstPosition_gl.at<double>(2, 0));
		fprintf (pFile,"  trial.slam.uav.EstYaw_gl(1,:) = %6.10f;\n", uav.EstYaw_gl);
		jointSLAM.slamswitch_cli.call(jointSLAM.slamswitch_call);
		uav.slamswitch_cli.call(uav.slamswitch_call);

		ROS_INFO("slamswitch() complete.");
	}

	void endExperiment()
	{
		/*  Sends landing command and shutdown messages */
		// ROS_WARN("Experiment ended. Press [enter] to land.\n");
		while (ros::ok())
		{
			// std::cin.ignore();
			ROS_INFO("Killing Hast Nodes...");
			ROS_INFO("Stopping Drone...");
			ROS_INFO("Landing Drone...");
			for(int b = 1; b < 15; ++b)
			{
				endTrial.HastKill.publish(endTrial.Kill_msg);
				endTrial.DroneCmd_dr_pub.publish(endTrial.DroneCmd_dr_msg);
				endTrial.DroneLand.publish(endTrial.Null_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}

			ROS_INFO("Resetting Hast Killswitch...");
			for(int b = 1; b < 15; ++b)
			{
				endTrial.Kill_msg.flag = false;
				endTrial.HastKill.publish(endTrial.Kill_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}
			ROS_INFO("...done\n");
			fprintf (pFile,"  trial.complete = true;\n");
			ros::shutdown();
		}
	}

	// path functions
		void ugv1_updateGlobalPath(const nav_msgs::Path::ConstPtr& path_msg)
		{
			// ROS_INFO("ugv1_updateGlobalPath...");
			ugv1.global_path_count += 1;
			double qx, qy, qz, qw, yaw;
			ugv1.globalPoses.clear();
			// ugv1.clearCostmaps_cli.call(ugv1.nullcall);
			ugv1.globalPoses = path_msg->poses;
			uint numposes = ugv1.globalPoses.size();

			std::string frame_id = path_msg->header.frame_id;
			fprintf (pFile,"\n%% ~~~~ ugv1_updateGlobalPath ~~~~ \n");
			fprintf (pFile,"  trial.ugv1.global_path.time (%i,1) = %6.10f;\n", ugv1.global_path_count, ros::Time::now().toSec()-init_time);
			fprintf (pFile,"  trial.ugv1.global_path.frame_id {%i} = '%s';\n", ugv1.global_path_count, frame_id.c_str());

			// for(int b = 0; b < pathPoseLength; ++b)
			cv::Mat pose_n;
			for(int b = 0; b < int(ugv1.globalPoses.size()); ++b)
			{
				qx = ugv1.globalPoses[b].pose.orientation.x;
				qy = ugv1.globalPoses[b].pose.orientation.y;
				qz = ugv1.globalPoses[b].pose.orientation.z;
				qw = ugv1.globalPoses[b].pose.orientation.w;
				// yaw = atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz));
				pose_n = ugv1.Hlo2gl * (cv::Mat_<double>(4, 1) <<
									ugv1.globalPoses[b].pose.position.x,
									ugv1.globalPoses[b].pose.position.y,
									ugv1.globalPoses[b].pose.position.z, 1);
				fprintf (pFile,"  trial.ugv1.global_path.p{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv1.global_path_count, 1+b, pose_n.at<double>(0,0), pose_n.at<double>(1,0), pose_n.at<double>(2,0) );
				fprintf (pFile,"  trial.ugv1.global_path.q{%i}(%i, :) = [%6.10f, %6.10f, %6.10f, %6.10f];\n", ugv1.global_path_count, 1+b, qx, qy, qz, qw);
				fprintf (pFile,"  trial.ugv1.global_path.yaw{%i}(%i) = %6.10f;\n", ugv1.global_path_count, 1+b, atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz)));
			}
		}

		void ugv2_updateGlobalPath(const nav_msgs::Path::ConstPtr& path_msg)
		{
			// ROS_INFO("ugv2_updateGlobalPath...");
			ugv2.global_path_count += 1;
			double qx, qy, qz, qw, yaw;
			ugv2.globalPoses.clear();
			ugv2.globalPoses = path_msg->poses;
			std::string frame_id = path_msg->header.frame_id;

			fprintf (pFile,"\n%% ~~~~ ugv2_updateGlobalPath ~~~~ \n");
			fprintf (pFile,"  trial.ugv2.global_path.time (%i,1) = %6.10f;\n", ugv2.global_path_count, ros::Time::now().toSec()-init_time);
			fprintf (pFile,"  trial.ugv2.global_path.frame_id {%i} = '%s';\n", ugv2.global_path_count, frame_id.c_str());
			fprintf (pFile,"  %%trial.ugv2.pose.p(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.global_path_count, ugv2.EstPosition_gl.at<double>(0,0),ugv2.EstPosition_gl.at<double>(1,0),ugv2.EstPosition_gl.at<double>(2,0));
			fprintf (pFile,"  %%trial.ugv2.pose.yaw(%i,1) = %6.10f;\n", ugv2.global_path_count, ugv2.EstYaw_gl);
			fprintf (pFile,"  trial.ugv2.Hlo2gl(1,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", ugv2.Hlo2gl.at<double>(0,0), ugv2.Hlo2gl.at<double>(0,1), ugv2.Hlo2gl.at<double>(0,2), ugv2.Hlo2gl.at<double>(0,3));
			fprintf (pFile,"  trial.ugv2.Hlo2gl(2,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", ugv2.Hlo2gl.at<double>(1,0), ugv2.Hlo2gl.at<double>(1,1), ugv2.Hlo2gl.at<double>(1,2), ugv2.Hlo2gl.at<double>(1,3));
			fprintf (pFile,"  trial.ugv2.Hlo2gl(3,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", ugv2.Hlo2gl.at<double>(2,0), ugv2.Hlo2gl.at<double>(2,1), ugv2.Hlo2gl.at<double>(2,2), ugv2.Hlo2gl.at<double>(2,3));
			fprintf (pFile,"  trial.ugv2.Hlo2gl(4,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", ugv2.Hlo2gl.at<double>(3,0), ugv2.Hlo2gl.at<double>(3,1), ugv2.Hlo2gl.at<double>(3,2), ugv2.Hlo2gl.at<double>(3,3));

			// for(int b = 0; b < pathPoseLength; ++b)
			cv::Mat pose_n;
			for(int b = 0; b < int(ugv2.globalPoses.size()); ++b)
			{
				qx = ugv2.globalPoses[b].pose.orientation.x; qy = ugv2.globalPoses[b].pose.orientation.y; qz = ugv2.globalPoses[b].pose.orientation.z; qw = ugv2.globalPoses[b].pose.orientation.w;
				pose_n = ugv2.Hlo2gl * (cv::Mat_<double>(4, 1) << ugv2.globalPoses[b].pose.position.x, ugv2.globalPoses[b].pose.position.y, ugv2.globalPoses[b].pose.position.z, 1);

				fprintf (pFile,"  trial.ugv2.global_path.p_gl{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv2.global_path_count, 1+b, pose_n.at<double>(0,0), pose_n.at<double>(1,0), pose_n.at<double>(2,0) );
				fprintf (pFile,"  trial.ugv2.global_path.p_lo{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv2.global_path_count, 1+b, ugv2.globalPoses[b].pose.position.x, ugv2.globalPoses[b].pose.position.y, ugv2.globalPoses[b].pose.position.z );
				fprintf (pFile,"  trial.ugv2.global_path.q{%i}(%i, :) = [%6.10f, %6.10f, %6.10f, %6.10f];\n", ugv2.global_path_count, 1+b, qx, qy, qz, qw);
				fprintf (pFile,"  trial.ugv2.global_path.yaw{%i}(%i) = %6.10f;\n", ugv2.global_path_count, 1+b, atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz)));
			}
		}

		void ugv1_updateLocalPath(const nav_msgs::Path::ConstPtr& path_msg)
		{
			fprintf (pFile,"\n%% ~~~~ ugv1_updateLocalPath ~~~~ \n");
			std::string frame_id = path_msg->header.frame_id;
			ugv1.local_path_count += 1;
			ugv1.localPoses.clear();
			ugv1.localPoses = path_msg->poses;
			uint numposes = ugv1.localPoses.size();
			ugv1.local_plan_bool = true;
			fprintf (pFile,"  trial.ugv1.local_path.frame_id{%i} = '%s';\n", ugv1.local_path_count, frame_id.c_str());
			fprintf (pFile,"  trial.ugv1.local_path.time(%i,1)   = %6.10f;\n", ugv1.local_path_count, ros::Time::now().toSec()-init_time);
			fprintf (pFile,"  trial.ugv1.pose_gl.p(%d,:)         = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, ugv1.EstPosition_gl.at<double>(0,0),ugv1.EstPosition_gl.at<double>(1,0),ugv1.EstPosition_gl.at<double>(2,0));
			fprintf (pFile,"  trial.ugv1.pose_gl.yaw(%i,1)       = %6.10f;\n", ugv1.local_path_count, ugv1.EstYaw_gl);

			double x_end = ugv1.localPoses[numposes-1].pose.position.x;
			double y_end = ugv1.localPoses[numposes-1].pose.position.y;
			double z_end = ugv1.localPoses[numposes-1].pose.position.z;

			double qx_end = ugv1.localPoses[numposes-1].pose.orientation.x;
			double qy_end = ugv1.localPoses[numposes-1].pose.orientation.y;
			double qz_end = ugv1.localPoses[numposes-1].pose.orientation.z;
			double qw_end = ugv1.localPoses[numposes-1].pose.orientation.w;
			ugv1.yaw_end = atan2(2*(qx_end*qy_end-qz_end*qw_end),1-2*(qy_end*qy_end+qz_end*qz_end)); // radians
			cv::Mat H_end2odom = (cv::Mat_<double>(4, 4) <<
							 cos( ugv1.yaw_end ), sin( ugv1.yaw_end ), 0, x_end,
							-sin( ugv1.yaw_end ), cos( ugv1.yaw_end ), 0, y_end,
							0, 0, 1, 0,
							0, 0, 0, 1);

			cv::Mat picket_end_odom = H_end2odom * (cv::Mat_<double>(4, 1) << ugv1.picket_offset.at<double>(0,0), ugv1.picket_offset.at<double>(1,0), ugv1.picket_offset.at<double>(2,0), 1);
			ugv1.picket_end  = ugv1.map_H_odom2gl * picket_end_odom;
			ugv1.path_end_gl = ugv1.map_H_odom2gl * H_end2odom * (cv::Mat_<double>(4, 1) << 0,0,0,1);


			// use a point 1/2 of the distance along the local trajectory for uav goal loaction
			double x_mid = ugv1.localPoses[numposes/2].pose.position.x;
			double y_mid = ugv1.localPoses[numposes/2].pose.position.y;
			double z_mid = ugv1.localPoses[numposes/2].pose.position.z;

			double qx_mid = ugv1.localPoses[numposes/2].pose.orientation.x;
			double qy_mid = ugv1.localPoses[numposes/2].pose.orientation.y;
			double qz_mid = ugv1.localPoses[numposes/2].pose.orientation.z;
			double qw_mid = ugv1.localPoses[numposes/2].pose.orientation.w;
			ugv1.yaw_mid = atan2(2*(qx_mid*qy_mid-qz_mid*qw_mid),1-2*(qy_mid*qy_mid+qz_mid*qz_mid)); // radians
			cv::Mat H_mid2odom = (cv::Mat_<double>(4, 4) <<
							 cos( ugv1.yaw_mid ), sin( ugv1.yaw_mid ), 0, x_mid,
							-sin( ugv1.yaw_mid ), cos( ugv1.yaw_mid ), 0, y_mid,
							0, 0, 1, 0,
							0, 0, 0, 1);

			cv::Mat picket_mid_odom = H_mid2odom * (cv::Mat_<double>(4, 1) << ugv1.picket_offset.at<double>(0,0), ugv1.picket_offset.at<double>(1,0), ugv1.picket_offset.at<double>(2,0), 1);
			ugv1.picket_mid = ugv1.map_H_odom2gl * picket_mid_odom;

			fprintf (pFile,"  trial.ugv1.local_path.p_end(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, x_end, y_end, z_end);
			fprintf (pFile,"  trial.ugv1.local_path.p_mid(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, x_mid, y_mid, z_mid);
			fprintf (pFile,"  trial.ugv1.local_path.yaw_end(%i,1) = [%6.10f];\n", ugv1.local_path_count, ugv1.yaw_end);
			fprintf (pFile,"  trial.ugv1.local_path.yaw_mid(%i,1) = [%6.10f];\n", ugv1.local_path_count, ugv1.yaw_mid);
			fprintf (pFile,"  trial.ugv1.local_path.picket_end_odom(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, picket_end_odom.at<double>(0,0), picket_end_odom.at<double>(1,0), picket_end_odom.at<double>(2,0));
			fprintf (pFile,"  trial.ugv1.local_path.picket_mid_odom(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, picket_mid_odom.at<double>(0,0), picket_mid_odom.at<double>(1,0), picket_mid_odom.at<double>(2,0));
			fprintf (pFile,"  trial.ugv1.local_path.picket_end_gl(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, ugv1.picket_end.at<double>(0,0), ugv1.picket_end.at<double>(1,0), ugv1.picket_end.at<double>(2,0));
			fprintf (pFile,"  trial.ugv1.local_path.picket_mid_gl(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv1.local_path_count, ugv1.picket_mid.at<double>(0,0), ugv1.picket_mid.at<double>(1,0), ugv1.picket_mid.at<double>(2,0));

			cv::Mat path_pose_gl;
			double yaw_path, qx, qy, qz, qw;
			for(int b = 0; b < int(ugv1.localPoses.size()); ++b)
			{
				path_pose_gl = ugv1.map_H_odom2gl * (cv::Mat_<double>(4, 1) << ugv1.localPoses[b].pose.position.x, ugv1.localPoses[b].pose.position.y, ugv1.localPoses[b].pose.position.z, 1);
				qx = ugv1.localPoses[b].pose.orientation.x;
				qy = ugv1.localPoses[b].pose.orientation.y;
				qz = ugv1.localPoses[b].pose.orientation.z;
				qw = ugv1.localPoses[b].pose.orientation.w;
				yaw_path = atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz));

				fprintf (pFile,"  trial.ugv1.local_path.yaw_lo{%i}(%i, :) = [%6.10f];\n", ugv1.local_path_count, 1+b, yaw_path );
				fprintf (pFile,"  trial.ugv1.local_path.p_lo{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv1.local_path_count, 1+b, ugv1.localPoses[b].pose.position.x, ugv1.localPoses[b].pose.position.y, ugv1.localPoses[b].pose.position.z );
				fprintf (pFile,"  trial.ugv1.local_path.p_gl{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv1.local_path_count, 1+b, path_pose_gl.at<double>(0,0), path_pose_gl.at<double>(1,0), path_pose_gl.at<double>(2,0));
			}
		}

		void ugv2_updateLocalPath(const nav_msgs::Path::ConstPtr& path_msg)
		{
			fprintf (pFile,"\n%% ~~~~ ugv2_updateLocalPath ~~~~ \n");
			std::string frame_id = path_msg->header.frame_id;
			ugv2.local_path_count += 1;
			ugv2.localPoses.clear();
			ugv2.localPoses = path_msg->poses;
			uint numposes = ugv2.localPoses.size();
			ugv2.local_plan_bool = true;
			fprintf (pFile,"  trial.ugv2.local_path.frame_id{%i} = '%s';\n", ugv2.local_path_count, frame_id.c_str());
			fprintf (pFile,"  trial.ugv2.local_path.time(%i,1)   = %6.10f;\n", ugv2.local_path_count, ros::Time::now().toSec()-init_time);
			fprintf (pFile,"  trial.ugv2.pose_gl.p(%d,:)         = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, ugv2.EstPosition_gl.at<double>(0,0),ugv2.EstPosition_gl.at<double>(1,0),ugv2.EstPosition_gl.at<double>(2,0));
			fprintf (pFile,"  trial.ugv2.pose_gl.yaw(%i,1)       = %6.10f;\n", ugv2.local_path_count, ugv2.EstYaw_gl);

			double x_end = ugv2.localPoses[numposes-1].pose.position.x;
			double y_end = ugv2.localPoses[numposes-1].pose.position.y;
			double z_end = ugv2.localPoses[numposes-1].pose.position.z;

			double qx_end = ugv2.localPoses[numposes-1].pose.orientation.x;
			double qy_end = ugv2.localPoses[numposes-1].pose.orientation.y;
			double qz_end = ugv2.localPoses[numposes-1].pose.orientation.z;
			double qw_end = ugv2.localPoses[numposes-1].pose.orientation.w;
			ugv2.yaw_end = atan2(2*(qx_end*qy_end-qz_end*qw_end),1-2*(qy_end*qy_end+qz_end*qz_end)); // radians
			cv::Mat H_end2odom = (cv::Mat_<double>(4, 4) <<
							 cos( ugv2.yaw_end ), sin( ugv2.yaw_end ), 0, x_end,
							-sin( ugv2.yaw_end ), cos( ugv2.yaw_end ), 0, y_end,
							0, 0, 1, 0,
							0, 0, 0, 1);

			cv::Mat picket_end_odom = H_end2odom * (cv::Mat_<double>(4, 1) << ugv2.picket_offset.at<double>(0,0), ugv2.picket_offset.at<double>(1,0), ugv2.picket_offset.at<double>(2,0), 1);
			ugv2.picket_end  = ugv2.map_H_odom2gl * picket_end_odom;
			ugv2.path_end_gl = ugv2.map_H_odom2gl * H_end2odom * (cv::Mat_<double>(4, 1) << 0,0,0,1);


			// use a point 1/2 of the distance along the local trajectory for uav goal loaction
			double x_mid = ugv2.localPoses[numposes/2].pose.position.x;
			double y_mid = ugv2.localPoses[numposes/2].pose.position.y;
			double z_mid = ugv2.localPoses[numposes/2].pose.position.z;

			double qx_mid = ugv2.localPoses[numposes/2].pose.orientation.x;
			double qy_mid = ugv2.localPoses[numposes/2].pose.orientation.y;
			double qz_mid = ugv2.localPoses[numposes/2].pose.orientation.z;
			double qw_mid = ugv2.localPoses[numposes/2].pose.orientation.w;
			ugv2.yaw_mid = atan2(2*(qx_mid*qy_mid-qz_mid*qw_mid),1-2*(qy_mid*qy_mid+qz_mid*qz_mid)); // radians
			cv::Mat H_mid2odom = (cv::Mat_<double>(4, 4) <<
							 cos( ugv2.yaw_mid ), sin( ugv2.yaw_mid ), 0, x_mid,
							-sin( ugv2.yaw_mid ), cos( ugv2.yaw_mid ), 0, y_mid,
							0, 0, 1, 0,
							0, 0, 0, 1);

			cv::Mat picket_mid_odom = H_mid2odom * (cv::Mat_<double>(4, 1) << ugv2.picket_offset.at<double>(0,0), ugv2.picket_offset.at<double>(1,0), ugv2.picket_offset.at<double>(2,0), 1);
			ugv2.picket_mid = ugv2.map_H_odom2gl * picket_mid_odom;

			fprintf (pFile,"  trial.ugv2.local_path.p_end(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, x_end, y_end, z_end);
			fprintf (pFile,"  trial.ugv2.local_path.p_mid(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, x_mid, y_mid, z_mid);
			fprintf (pFile,"  trial.ugv2.local_path.yaw_end(%i,1) = [%6.10f];\n", ugv2.local_path_count, ugv2.yaw_end);
			fprintf (pFile,"  trial.ugv2.local_path.yaw_mid(%i,1) = [%6.10f];\n", ugv2.local_path_count, ugv2.yaw_mid);
			fprintf (pFile,"  trial.ugv2.local_path.picket_end_odom(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, picket_end_odom.at<double>(0,0), picket_end_odom.at<double>(1,0), picket_end_odom.at<double>(2,0));
			fprintf (pFile,"  trial.ugv2.local_path.picket_mid_odom(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, picket_mid_odom.at<double>(0,0), picket_mid_odom.at<double>(1,0), picket_mid_odom.at<double>(2,0));
			fprintf (pFile,"  trial.ugv2.local_path.picket_end_gl(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, ugv2.picket_end.at<double>(0,0), ugv2.picket_end.at<double>(1,0), ugv2.picket_end.at<double>(2,0));
			fprintf (pFile,"  trial.ugv2.local_path.picket_mid_gl(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ugv2.local_path_count, ugv2.picket_mid.at<double>(0,0), ugv2.picket_mid.at<double>(1,0), ugv2.picket_mid.at<double>(2,0));

			cv::Mat path_pose_gl;
			double yaw_path, qx, qy, qz, qw;
			for(int b = 0; b < int(ugv2.localPoses.size()); ++b)
			{
				path_pose_gl = ugv2.map_H_odom2gl * (cv::Mat_<double>(4, 1) << ugv2.localPoses[b].pose.position.x, ugv2.localPoses[b].pose.position.y, ugv2.localPoses[b].pose.position.z, 1);
				qx = ugv2.localPoses[b].pose.orientation.x;
				qy = ugv2.localPoses[b].pose.orientation.y;
				qz = ugv2.localPoses[b].pose.orientation.z;
				qw = ugv2.localPoses[b].pose.orientation.w;
				yaw_path = atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz));

				fprintf (pFile,"  trial.ugv2.local_path.yaw_lo{%i}(%i, :) = [%6.10f];\n", ugv2.local_path_count, 1+b, yaw_path );
				fprintf (pFile,"  trial.ugv2.local_path.p_lo{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv2.local_path_count, 1+b, ugv2.localPoses[b].pose.position.x, ugv2.localPoses[b].pose.position.y, ugv2.localPoses[b].pose.position.z );
				fprintf (pFile,"  trial.ugv2.local_path.p_gl{%i}(%i, :) = [%6.10f, %6.10f, %6.10f];\n", ugv2.local_path_count, 1+b, path_pose_gl.at<double>(0,0), path_pose_gl.at<double>(1,0), path_pose_gl.at<double>(2,0));
			}
		}

	// uav Functions
		void UAVstepTo(cv::Mat ToPos, double ToYaw, double timeSpan)
		{
			ROS_INFO("UAVstepTo: ToPos = [% -4.2f % -4.2f % -4.2f] [% -4.2f], FlyTime = [% -4.2f]", ToPos.at<double>(0,0), ToPos.at<double>(1,0), ToPos.at<double>(2,0), ToYaw, timeSpan);

			double TrajStartTime = ros::Time::now().toSec();
			double NewX, NewY, NewZ;
			// ROS_INFO("trial: delta = NewPos - LastPos = [% -6.8f % -6.8f % -6.8f]",delta.at<double>(0,0), delta.at<double>(1,0), delta.at<double>(2,0) );

			while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
			{
				 // run trajectory path
				//update states & calculate desired states
				ros::spinOnce();
				NewX = ToPos.at<double>(0,0);
				NewY = ToPos.at<double>(1,0);
				NewZ = ToPos.at<double>(2,0);
				++FlyToCount;
				// ROS_INFO("trial.uav.FlyTime(%d)  = %f;", FlyToCount, ros::Time::now().toSec());
				// ROS_INFO("trial.uav.DesiredState_gl(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];", FlyToCount, NewX, NewY, NewZ, ToYaw);
				// ROS_INFO("trial.uav.Waypoint(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
				// ROS_INFO("trial.uav.CurrentState_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);

				fprintf (pFile,"\n%% ~~~~ UAVstepTo ~~~~ \n");
				fprintf (pFile,"  trial.uav.FlyTime(%d,1)  = %f;\n", FlyToCount, ros::Time::now().toSec()-init_time);
				fprintf (pFile,"  trial.uav.DesiredState_gl(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, NewX, NewY, NewZ, ToYaw);
				fprintf (pFile,"  trial.uav.Waypoint(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
				fprintf (pFile,"  trial.uav.CurrentState_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
				uav.setDesiredPosition(NewX, NewY, NewZ, ToYaw);
				// ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
				spin_sleep(0.25);
			}
		}

		void linearTrajTo(cv::Mat OriginalPos, cv::Mat ToPos, double OriginalYaw, double ToYaw, double timeSpan)
		{
			ROS_INFO("linearTrajTo: ToPos = [% -4.2f % -4.2f % -4.2f] [% -4.2f], FlyTime = [% -4.2f]", ToPos.at<double>(0,0), ToPos.at<double>(1,0), ToPos.at<double>(2,0), ToYaw, timeSpan);
			// mac ROS_INFO("trial: uav.DesiredPos_gl: [% -6.8f % -6.8f % -6.8f : % -6.8f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
			// mac ROS_INFO("trial: uav.EstPosition_gl: [% -6.8f % -6.8f % -6.8f : % -6.8f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);

			double TrajStartTime = ros::Time::now().toSec();
			cv::Mat mdelta = (ToPos - OriginalPos)/timeSpan;
			double mYaw = (ToYaw - OriginalYaw)/timeSpan;
			double NewX, NewY, NewZ, NewYaw;
			// ROS_INFO("trial: delta = NewPos - LastPos = [% -6.8f % -6.8f % -6.8f]",delta.at<double>(0,0), delta.at<double>(1,0), delta.at<double>(2,0) );

			while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
			{
				 // run trajectory path
				//update states & calculate desired states
				ros::spinOnce();
				double deltaTime = ros::Time::now().toSec() - TrajStartTime;
				NewX = OriginalPos.at<double>(0,0) + mdelta.at<double>(0,0) * deltaTime;
				NewY = OriginalPos.at<double>(1,0) + mdelta.at<double>(1,0) * deltaTime;
				// NewZ = OriginalPos.at<double>(2,0) + mdelta.at<double>(2,0) * deltaTime;
				NewZ = ToPos.at<double>(2,0);
				NewYaw = OriginalYaw + mYaw * deltaTime;
				fprintf (pFile,"\n%% ~~~~ linearTrajTo ~~~~ \n");
				fprintf (pFile,"  trial.uav.FlyTime(%d,1)  = %f;\n", ++FlyToCount, deltaTime+TrajStartTime - init_time);
				fprintf (pFile,"  trial.uav.DesiredState_gl(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, NewX, NewY, NewZ, NewYaw);
				fprintf (pFile,"  trial.uav.Waypoint(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
				fprintf (pFile,"  trial.uav.CurrentState_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
				uav.setDesiredPosition(NewX, NewY, NewZ, NewYaw);
				// // mac ROS_INFO("trial: uav.DesiredPos_gl: [% -6.8f % -6.8f % -6.8f : % -6.8f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
				// // mac ROS_INFO("trial: uav.EstPosition_gl: [% -6.8f % -6.8f % -6.8f : % -6.8f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);
				ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
			}
		}

	/* ROS level utility functions */
		void killhast(const hast::flag::ConstPtr& shutdownflag)
		{ // node cleanup for end of trial
			if(shutdownflag->flag)
			{
				ros::Duration(0.5).sleep();
				uav.Land_pub.publish(uav.null);
				fprintf (pFile,"trial.stop_time = %6.10f;\n", ros::Time::now().toSec() - init_time);
				// mac ROS_INFO("trial: Flight Ended");
				ros::Duration(2).sleep(); // sleep for 'x' second(s).
				ros::shutdown();
			}
		}

		void spin_sleep(double sleep_time)
		{
			double sleepStartTime = ros::Time::now().toSec();
			while((ros::Time::now().toSec() - sleepStartTime) < sleep_time){ros::spinOnce();}
		}

		bool is_running(hast::servtime::Request &req, hast::servtime::Response &res)
		{
			// res.header.seq = 0;
			res.sync_msg.frame_id = "trial";
			res.sync_msg.node_start = node_start_time.toSec();
			res.sync_msg.now = ros::Time::now();

			return true;
		}

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hasttrial");
	hastTrial hE;
	ros::Duration(1).sleep(); // sleep for 'x' second(s).
	hE.RunExperiment();


	return 0;
}
