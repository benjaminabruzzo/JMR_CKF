#include "genheaders.hpp"

// Move library
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
		int goal_tag_id;
		cv::Mat goalP_gl;
		double yawAtGoal_gl;

		std::string s_move_goal_topic, s_clearcostmap_topic, s_goal_tag_topic;
		std::string s_movebaseresult_topic, s_goal_header_frame_id, s_map_frame;
		std::string s_ugvn_ckfstate_topic;

		ros::ServiceClient clearCostmaps_cli; hast::null nullcall; // clears obstacle costmaps
		ros::Publisher goalpose_pub; //publisher for setting move base goal of ugv
			geometry_msgs::Pose2D goal_for_ugv_msg; //pose message for uav goal location
		ros::ServiceClient GoalTagLocation_cli; // client to request goal tag location
			hast::ugvgoal goal_call; // service data for getting tag location
			bool goal_call_set;

			move_base_msgs::MoveBaseGoal goal; // for move_base goal setting, might not be used...
			geometry_msgs::PointStamped goalPoint_ckf, goalPoint_map; // might not be used...

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
			cosyaw = cos(Pi * EstYaw_gl / 180);
			sinyaw = sin(Pi * EstYaw_gl / 180);
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
								cosyaw, sinyaw, 0,
								-sinyaw, cosyaw, 0,
								0, 0, 1);

			goal_call.request.tagID = 1;
			goal_call.request.ugv_id = "ugv0";
		}

		void call_tag_location(double tag_num)
		{
			goal_call.request.tagID = tag_num;
			clearCostmaps_cli.call(nullcall);
			bool call_bool = GoalTagLocation_cli.call(goal_call);
			spin_sleep(1);

			if (call_bool){
					goal_call_set = goal_call.response.goalInMap;
					if (goal_call.response.goalInMap){
						ROS_INFO("TRIAL:: uav found goal tag, setting location as goal for ugv");
						ROS_INFO("TRIAL::     [x y phi] = [% -4.4f % -4.4f % -4.4f ]", goal_call.response.goalP.x, goal_call.response.goalP.y, goal_call.response.goalYaw);
						goal_for_ugv_msg.x = goal_call.response.goalP.x;
						goal_for_ugv_msg.y = goal_call.response.goalP.y;
						goal_for_ugv_msg.y = goal_call.response.goalP.y;
						goal_for_ugv_msg.theta = goal_call.response.goalYaw; // set the ugv goal yaw to match tag yaw, for some reason
						goalpose_pub.publish(goal_for_ugv_msg);
					}
			  } else {
			    ROS_ERROR("TRIAL:: Failed to call service GoalTagLocation_cli");
			  }
		}

		void spinsleep(int dwell_seconds)
		{
			for(int b = 0; b < dwell_seconds; ++b)
			{
				ros::spinOnce();
				ros::Duration(1).sleep(); // sleep for 'x' second(s).
			}
		}

		void spin_sleep(double sleep_time)
		{
			double sleepStartTime = ros::Time::now().toSec();

			while((ros::Time::now().toSec() - sleepStartTime) < sleep_time)
			{
				clearCostmaps_cli.call(nullcall);
				ros::spinOnce();
			}
		}

		void updatePose()
		{
			// mac ROS_INFO("ugvClass: updatePose");
			do
			{
				stateMsg_idLast = stateMsg_id;
				ros::spinOnce();
			}
			while(stateMsg_idLast!=stateMsg_id);
			// mac ROS_INFO("trial: UGV Pose updated.");
			// mac ROS_INFO("trial: EstPosition_gl: [%6.4f %6.4f %6.4f]", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		}

		void readState(const hast::ugvstate::ConstPtr& ugvState_msg)
		{
			msgStamp = ugvState_msg-> stamp;

			if (msgStamp!=msgStampLast)
			{
				++posecount;
				msgStampLast = ugvState_msg-> stamp;
				Rgl2lo = (cv::Mat_<double>(3,3) <<
					ugvState_msg -> R.row0.x, ugvState_msg -> R.row0.y, ugvState_msg -> R.row0.z,
					ugvState_msg -> R.row1.x, ugvState_msg -> R.row1.y, ugvState_msg -> R.row1.z,
					ugvState_msg -> R.row2.x, ugvState_msg -> R.row2.y, ugvState_msg -> R.row2.z);
				EstPosition_gl = (cv::Mat_<double>(3, 1) <<
					ugvState_msg -> P.x,
					ugvState_msg -> P.y,
					ugvState_msg -> P.z);
				EstYaw_gl = ugvState_msg -> yaw;
				stateMsg_id = ugvState_msg -> id;
			}
		}
};

class goalPublisher
{
	public:
		ros::NodeHandle n;
		ros::Subscriber HastShutDown_sub;
		hast::flag Kill_msg;
		ugvClass ugv1, ugv2;
	// rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

	goalPublisher()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
			/*-----  Publishers and Subscribers */
			HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &goalPublisher::nodeShutDown, this);

		/*----- UGV Channels */
			ros::param::get("~ugv1_map_frame",ugv1.s_map_frame); //fprintf (pFile,"trial.topics.ugv1_map_frame = '%s';\n",ugv1.s_map_frame.c_str());
			ros::param::get("~ugv2_map_frame",ugv2.s_map_frame); //fprintf (pFile,"trial.topics.ugv2_map_frame = '%s';\n",ugv2.s_map_frame.c_str());
			ros::param::get("~ugv1_april_goal_id",ugv1.goal_tag_id); //fprintf (pFile,"trial.topics.ugv1_april_goal_id = '%i';\n",ugv1.goal_tag_id);
			ros::param::get("~ugv2_april_goal_id",ugv2.goal_tag_id); //fprintf (pFile,"trial.topics.ugv2_april_goal_id = '%i';\n",ugv2.goal_tag_id);
				ugv1.goal_call.request.ugv_id = "ugv1"; ugv1.goal_call.request.tagID = ugv1.goal_tag_id;
				ugv2.goal_call.request.ugv_id = "ugv2"; ugv2.goal_call.request.tagID = ugv2.goal_tag_id;
			// goal_tag_topic gets the position of the goal tag for th eugv from the slam/ckf node
			ros::param::get("~ugv1_goal_tag_topic",ugv1.s_goal_tag_topic); //fprintf (pFile,"trial.topics.ugv1_goal_tag_topic = '%s';\n",ugv1.s_goal_tag_topic.c_str());
			ros::param::get("~ugv2_goal_tag_topic",ugv2.s_goal_tag_topic); //fprintf (pFile,"trial.topics.ugv2_goal_tag_topic = '%s';\n",ugv2.s_goal_tag_topic.c_str());
				ugv1.GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>(ugv1.s_goal_tag_topic, true);
				ugv2.GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>(ugv2.s_goal_tag_topic, true);
			// ugv1_move_goal_topic sets the goal position of the ugv for move_base
			ros::param::get("~ugv1_move_goal_topic",ugv1.s_move_goal_topic); //fprintf (pFile,"trial.topics.ugv1_move_goal_topic = '%s';\n",ugv1.s_move_goal_topic.c_str());
			ros::param::get("~ugv2_move_goal_topic",ugv2.s_move_goal_topic); //fprintf (pFile,"trial.topics.ugv2_move_goal_topic = '%s';\n",ugv2.s_move_goal_topic.c_str());
				ugv1.goalpose_pub	= n.advertise<geometry_msgs::Pose2D>(ugv1.s_move_goal_topic.c_str(), 10);
				ugv2.goalpose_pub	= n.advertise<geometry_msgs::Pose2D>(ugv2.s_move_goal_topic.c_str(), 10);
			// clears the costmap of each vehicle
			ros::param::get("~ugv1_clearcostmap_topic",ugv1.s_clearcostmap_topic); //fprintf (pFile,"trial.topics.ugv1_clearcostmap_topic = '%s';\n",ugv1.s_clearcostmap_topic.c_str());
			ros::param::get("~ugv2_clearcostmap_topic",ugv2.s_clearcostmap_topic); //fprintf (pFile,"trial.topics.ugv2_clearcostmap_topic = '%s';\n",ugv2.s_clearcostmap_topic.c_str());
				ugv1.clearCostmaps_cli = n.serviceClient<hast::null>(ugv1.s_clearcostmap_topic, true);
				ugv2.clearCostmaps_cli = n.serviceClient<hast::null>(ugv2.s_clearcostmap_topic, true);
			// ugv1_goal_header_frame_id sets the goal header_frame_id of the ugv for move_base
			ros::param::get("~ugv1_goal_header_frame_id",ugv1.s_goal_header_frame_id); //fprintf (pFile,"trial.topics.ugv1_goal_header_frame_id = '%s';\n",ugv1.s_goal_header_frame_id.c_str());
			ros::param::get("~ugv2_goal_header_frame_id",ugv2.s_goal_header_frame_id); //fprintf (pFile,"trial.topics.ugv2_goal_header_frame_id = '%s';\n",ugv2.s_goal_header_frame_id.c_str());
				ugv1.goal.target_pose.header.frame_id = ugv1.s_goal_header_frame_id;
				ugv2.goal.target_pose.header.frame_id = ugv2.s_goal_header_frame_id;

		ROS_INFO("goalPublisher created."); ros::Duration(3).sleep(); // sleep for 'x' second(s).

		ROS_INFO("Calling ugv1.call_tag_location.");
		ugv1.call_tag_location(ugv1.goal_tag_id);

		ROS_INFO("Calling ugv1.call_tag_location.");
		ugv2.call_tag_location(ugv2.goal_tag_id);

		ros::Duration(3).sleep();

		ROS_INFO("goalPublisher closing.");

		ros::shutdown();

	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
		{if(ShutDown->flag){ros::shutdown();}}

	void spin_sleep(double sleep_time)
		{double sleepStartTime = ros::Time::now().toSec();
		while((ros::Time::now().toSec() - sleepStartTime) < sleep_time){ros::spinOnce();}}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goalPublisher");
	goalPublisher gP;
	ros::spin();
	return 0;
}
