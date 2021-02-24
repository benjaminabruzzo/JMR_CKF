#include "genheaders.hpp"

class wait_for_finish
{
	public:
		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
			std::string s_date, s_trial, s_exp_code;

		ros::Subscriber  HastShutDown_sub;
		bool sd_flag;

		ros::ServiceClient callTime_cli, trial_isRunning_cli, ugv1_isRunning_cli, ugv2_isRunning_cli; // client to request trial is running
			std::string s_trial_running_topic, s_ugv1_running_topic, s_ugv2_running_topic;
			bool call_bool, trial_bool, ugv1_bool, ugv2_bool;
			hast::servtime time_call;
			std::string calling_node;

			double init_time;


	wait_for_finish()
	{
		n.getParam("/hast/date",			s_date);			ROS_INFO("/hast/date:     %s", s_date.c_str());
		n.getParam("/hast/trial",			s_trial);			ROS_INFO("/hast/trial:    %s", s_trial.c_str());
		n.getParam("/hast/exp_code",	s_exp_code);	ROS_INFO("/hast/exp_code: %s", s_exp_code.c_str());

		HastShutDown_sub 	= n.subscribe("/hast/shutdown",		1, &wait_for_finish::nodeShutDown		, this);

		callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
		ros::param::get("~ugv1_running_topic",	s_ugv1_running_topic);		ugv1_isRunning_cli	= n.serviceClient<hast::servtime>(s_ugv1_running_topic, true);
		ros::param::get("~ugv2_running_topic",	s_ugv2_running_topic);		ugv2_isRunning_cli	= n.serviceClient<hast::servtime>(s_ugv2_running_topic, true);
		ros::param::get("~trial_running_topic",	s_trial_running_topic);		trial_isRunning_cli	= n.serviceClient<hast::servtime>(s_trial_running_topic, true);
		ros::Duration(1).sleep();
		ROS_INFO("Checking on Nodes");
		time_call.request.requesting_node = "wait_for_finish";
		call_bool = callTime_cli.call(time_call);					if (call_bool){ROS_INFO("Master running.");}		else {ROS_INFO("Master not responding.");	ros::shutdown();}
		ugv1_bool = ugv1_isRunning_cli.call(time_call); 	if (ugv1_bool){ROS_INFO("ugv1 is running.");}		else {ROS_INFO("ugv1 not responding.");		ros::shutdown();}

		if ((s_exp_code.compare("A") == 0) || (s_exp_code.compare("B") == 0) )	{ROS_INFO("no ugv2 to check.");} else
		{ugv2_bool = ugv2_isRunning_cli.call(time_call);		if (ugv2_bool){ROS_INFO("ugv2 is running.");}		else {ROS_INFO("ugv2 not responding.");		ros::shutdown();}}

		trial_bool = trial_isRunning_cli.call(time_call);	if (trial_bool){ROS_INFO("Trial is running.");}	else {ROS_INFO("Trial not responding.");	ros::shutdown();}
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{
				ROS_INFO("wait_for_finish: Trial Complete, Shutting Down...");
				ros::shutdown();
			}
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wait_for_finish");
	wait_for_finish wff;
	ros::spin();
	return 0;
}
