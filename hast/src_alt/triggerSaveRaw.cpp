#include "genheaders.hpp"

std_msgs::Empty Null_msg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HastKill");
	ros::NodeHandle n;
		/*-----  Publishers and Subscribers */
		std::string pub_topic;
		ros::param::get("~pub_topic",pub_topic);
		ros::Publisher SaveTrigger_pub 	= n.advertise<std_msgs::Empty>(pub_topic.c_str(), 1000);

	while (ros::ok())
	{
		ROS_INFO("Ready to capture image.. \n");
		std::cin.ignore();

		SaveTrigger_pub.publish(Null_msg);
		ros::spinOnce();

		ROS_INFO("Resetting camera switch...");

		ros::Duration(1).sleep(); // sleep for 'x' second(s).
		
	}
	ROS_INFO("...done\n");
	ros::shutdown();

	return 0;
}
