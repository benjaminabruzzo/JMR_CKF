#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include "genheaders.hpp"

// class countdown
// {
// 	public:
// 		/*--------- ROS Communication Containers ------------- */
// 		ros::NodeHandle n;
//
// 	countdown()
// 	{
// 		double t = 240;
// 		double n = 20;
//
// 		while (t>0)
// 		{
// 			ROS_INFO("countdown: waiting for (%6.2f) seconds...", t);
// 			t -= n;
// 			ros::Duration(n).sleep(); // sleep for 'x' second(s).
// 		}
//
// 		ROS_INFO("countdown: Timeout, Shutting Down...");
//
// 		ros::shutdown();
// 	}
//
//
// };

int main()
{
	// ros::init(argc, argv, "countdown");
	// ros::NodeHandle n;
	double t = 240;
	double dt = 20;

	while (t>0)
	{
		printf("countdown: waiting for (%6.2f) seconds...\n", t);
		t -= dt;
		sleep(dt);
		// ros::Duration(dt).sleep(); // sleep for 'x' second(s).
	}

	printf("countdown: Timeout, Shutting Down...");
	return 0;
}
