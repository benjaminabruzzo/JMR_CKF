#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "ckfClass.hpp"
// #include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";


class uavMuxxer
{
	private:
		/*---------  File Recorder ------------- */
		fileops mfile;
			std::string s_trial, s_dotm, s_root, s_date, s_user;
			double Pi;

		ros::NodeHandle n;
			ros::Subscriber HastShutDown_sub;
			std::string s_shutdown_topic;


		std::vector<apriltags_ros::AprilTagDetection> tagArray; //marker array as detected by image
			ros::Subscriber TagDetections_sub;
				std::string s_tagDetections;
			uint tagArraySize; // number of tags in array
			ros::Time tagStamp;

		hastUAV uav; // create uav from class
			ros::Publisher uav_mux_pub;
				hast::uavmux uav_mux_msg;
				std::string s_uav_mux_topic;


		ros::Subscriber slam_poses_sub;
			std::string s_slam_poses_topic;


	public:


	uavMuxxer()
	{
		/*--------- Math Constants ------------- */
		Pi = atan(1) * 4; // 3.14159...

		/*---------  File Recorder Initilizer ------------- */
		ros::param::get("~user",s_user);
		ros::param::get("~date",s_date);
		ros::param::get("~trial",s_trial);
		mfile.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_trial + "/uav_mux_"  + s_trial + ".m");
		uav.s_filename = "/home/" + s_user + "/ros/data/" + s_date + "/" + s_trial + "/uav_MuxRecorder_" + s_trial + ".m";
		uav.openmFile();

		ROS_INFO("~~~~~~~~ uav_Muxxer: .m files opened");

		ros::param::get("~uav_mux_topic",s_uav_mux_topic); mfile.writeString("%% uav_mux_topic: " + s_uav_mux_topic + "\n" );
			uav_mux_pub = n.advertise<hast::uavmux>(s_uav_mux_topic, 1);
			uav_mux_msg.seq = 0;

		ros::param::get("~shutdown_topic",s_shutdown_topic); mfile.writeString("%% shutdown_topic: " + s_shutdown_topic + "\n" );
			HastShutDown_sub 	= n.subscribe(s_shutdown_topic,	1, &uavMuxxer::nodeShutDown, this);

		// Subscribe to tag detections
		ros::param::get("~tag_detection_topic",s_tagDetections); mfile.writeString("%% tag_detection_topic: " + s_tagDetections + "\n" );
			TagDetections_sub 	= n.subscribe(s_tagDetections, 1, &uavMuxxer::tagDetections , this);

		ros::param::get("~slam_poses_topic",s_slam_poses_topic); mfile.writeString("%% slam_poses_topic: " + s_slam_poses_topic + "\n" );
			slam_poses_sub 		= n.subscribe(s_slam_poses_topic, 1, &uavMuxxer::SLAMread , this);

		ros::param::get("~navdata_topic",uav.s_navdata_topic);; mfile.writeString("%% navdata_topic: " + uav.s_navdata_topic + "\n" );
			uav.navData_sub = n.subscribe(uav.s_navdata_topic,  	5, &hastUAV::inertialUpdateJoint, &uav);

		ros::param::get("~uav_pose_topic",uav.s_uav_pose_topic); mfile.writeString("%% uav_pose_topic: " + uav.s_uav_pose_topic + "\n" );
			uav.pose_pub = n.advertise<hast::posewithheader>(uav.s_uav_pose_topic, 1);

	}

	void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags)
	{

		tagArray= tags->detections;

		uav_mux_msg.seq += 1;
		uav_mux_msg.stamp = ros::Time::now().toSec();
		uav_mux_msg.tags = tagArray;

		uav_mux_msg.uavEst.position.x = uav.EstPosition_gl.at<double>(0,0);
		uav_mux_msg.uavEst.position.y = uav.EstPosition_gl.at<double>(1,0);
		uav_mux_msg.uavEst.position.z = uav.EstPosition_gl.at<double>(2,0);
		uav_mux_msg.uavEst.PCov.row0.x = uav.ckf.Qdk.at<double>(0,0);
		uav_mux_msg.uavEst.PCov.row1.y = uav.ckf.Qdk.at<double>(1,1);
		uav_mux_msg.uavEst.PCov.row2.z = uav.ckf.Qdk.at<double>(2,2);
		uav_mux_msg.uavEst.yaw = uav.EstYaw_gl;
		uav_mux_msg.uavEst.yaw_cov = uav.ckf.Qdk.at<double>(3,3);

		uav_mux_pub.publish(uav_mux_msg);


	}

	void SLAMread(const hast::slam_poses::ConstPtr& slam_msg)
	{
		uav.EstPosition_gl = (cv::Mat_<double>(3, 1) <<
			slam_msg->uav_pose.position.x,
			slam_msg->uav_pose.position.y,
			slam_msg->uav_pose.position.z);
		uav.EstYaw_gl = slam_msg->uav_pose.yaw;
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
	ros::init(argc, argv, "uav_muxxer");
	uavMuxxer uavMux;
	ros::spin();
	return 0;
}
