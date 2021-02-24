#include "genheaders.hpp"


	hast::flag Kill_msg;
	std_msgs::Empty Null_msg;
	geometry_msgs::Twist DroneCmd_dr_msg;

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "HastKill");
		ros::NodeHandle n;
			/*-----  Publishers and Subscribers */
			ros::Publisher HastKill		= n.advertise<hast::flag>("/hast/shutdown", 1000);
			ros::Publisher DroneLand	= n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
			ros::Publisher DroneCmd_dr_pub 	= n.advertise<geometry_msgs::Twist>	("/hast/uav/cmd_vel", 1000);

			DroneCmd_dr_msg.linear.x = 0.0;
			DroneCmd_dr_msg.linear.y = 0.0;
			DroneCmd_dr_msg.linear.z = 0.0;
			DroneCmd_dr_msg.angular.z = 0.0;
			DroneCmd_dr_msg.angular.x = 0.0;
			DroneCmd_dr_msg.angular.y = 0.0;
		while (ros::ok())
		{
			Kill_msg.flag = true;
			Kill_msg.calling_node = ros::this_node::getName();
			std::string this_node = ros::this_node::getName();
			ROS_INFO("%s:: Ready to kill Hast Nodes.\n", this_node.c_str());
			std::cin.ignore();
			ROS_INFO("Killing Hast Nodes...");
			ROS_INFO("Stopping Drone...");
			ROS_INFO("Landing Drone...");
			for(int b = 1; b < 15; ++b)
			{
				HastKill.publish(Kill_msg);
				DroneCmd_dr_pub.publish(DroneCmd_dr_msg);
				DroneLand.publish(Null_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}

			ROS_INFO("Resetting Hast Killswitch...");
			for(int b = 1; b < 15; ++b)
			{
				Kill_msg.flag = false;
				HastKill.publish(Kill_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}
			ROS_INFO("...done\n");
			ros::shutdown();
		}

		return 0;
	}
