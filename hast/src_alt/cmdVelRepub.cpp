#include "genheaders.hpp"

class cmdVelRepub
{
	private:
		ros::Subscriber HastShutDown_sub;
		hast::flag Kill_msg;
		std_msgs::Empty Null_msg;
		geometry_msgs::Twist DroneCmd_dr_msg;

	public:
		ros::NodeHandle n;

	cmdVelRepub()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &shutdownListener::nodeShutDown, this);
		ROS_INFO("Shutdown Listener Constructed");
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{ros::shutdown();}
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugvDrive");
	shutdownListener tC;
	ros::spin();
	return 0;
}
