#include "genheaders.hpp"

class rostimeCaller
{
	private:

	public:
		ros::NodeHandle n;

		ros::Subscriber HastShutDown_sub;
		hast::flag Kill_msg;

		ros::ServiceClient callTime_cli; // client to request goal tag location
			hast::servtime time_call;
			std::string calling_node;

	rostimeCaller()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &rostimeCaller::nodeShutDown, this);
		callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
		calling_node = "rostimeCaller";

		ROS_INFO("rostimeCaller created.");
		spinsleep(2);

		ROS_INFO("time_call.request.requesting_node = %s;", calling_node.c_str());
		time_call.request.requesting_node = calling_node;
		ROS_INFO("callTime_cli.call(time_call);");
		ros::Time calltime = ros::Time::now();
		callTime_cli.call(time_call);
		ros::Time servicetime = time_call.response.header.stamp;
		ros::Time responsetime = ros::Time::now();

		ROS_INFO("calltime     :: %f", calltime.toSec());
		ROS_INFO("responsetime :: %f", responsetime.toSec());
		ROS_INFO("servicetime  :: %f", servicetime.toSec());

	}

	void spinsleep(int dwell_seconds)
	{
		for(int b = 0; b < dwell_seconds; ++b)
		{
			ros::spinOnce();
			ros::Duration(1).sleep(); // sleep for 'x' second(s).
		}
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
		{ros::shutdown();}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rostimeCaller");
	rostimeCaller rC;
	ros::spin();
	return 0;
}
