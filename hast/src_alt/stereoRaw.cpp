#include "genheaders.hpp"
#include "utils.hpp"

namespace enc = sensor_msgs::image_encodings;

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(5) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}

class stereoRaw
{
private:
	/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
		/*----- image transport Channels */
		image_transport::ImageTransport it;
		image_transport::Subscriber LeftImage_sub, LeftRaw_sub;
		image_transport::Subscriber RightImage_sub, RightRaw_sub;
		ros::Subscriber SaveTrigger_sub;
			std_msgs::Empty Null_msg;

		ros::Subscriber  HastShutDown_sub;

	/*---------  File Recorder ------------- */
		std::string s_filename, s_prealloc, s_trial, s_dotm, s_root, s_handle, s_date, s_user, s_ugv_n;
		std::FILE * pFile;
		std::string s_color_param_file;

		/*--------- Image Containers ------------- */
		ros::Time leftStamp_raw, rightStamp_raw;
		std::string s_imString;

		std::string s_left_raw_topic, s_right_raw_topic, s_raw_trigger_topic;
		int leftraw_count, rightraw_count;

		cv::Mat LeftRawIamge, RightRawIamge;
		double RawSaveTime;

public:
	stereoRaw()
	: it(n)
	{
		ros::param::get("~ugv_n",s_ugv_n);
		if (n.getParam("/hast/user",  s_user)) {} else  {s_user = "benjamin";}
		if (n.getParam("/hast/date",  s_date)) {} else  {s_date = "20200103";}
		if (n.getParam("/hast/trial", s_trial)) {} else {s_trial = "001";}

		/*---------  File Recorder Initilizer ------------- */
		s_handle = s_ugv_n + "Stereo";
		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		s_filename = s_root + s_date + "/" + s_trial + "/" + s_handle + "_" + s_trial + s_dotm;
		ROS_INFO("stereoRaw: %s", s_filename.c_str());
		pFile = std::fopen (s_filename.c_str(),"w");

		/*-----  Publishers and Subscribers */
		ros::param::get("~color_param_file",s_color_param_file); fprintf (pFile,"%% s_color_param_file: %s\n",s_color_param_file.c_str());
		ros::param::get("~left_raw_topic",s_left_raw_topic); fprintf (pFile,"%% s_left_raw_topic: %s\n",s_left_raw_topic.c_str());
		ros::param::get("~right_raw_topic",s_right_raw_topic); fprintf (pFile,"%% s_right_raw_topic: %s\n",s_right_raw_topic.c_str());
		ros::param::get("~raw_trigger_topic",s_raw_trigger_topic); fprintf (pFile,"%% s_raw_trigger_topic: %s\n",s_raw_trigger_topic.c_str());
		LeftRaw_sub		 = it.subscribe(s_left_raw_topic,   1,  &stereoRaw::leftCvBridge_raw, this);
		RightRaw_sub	 = it.subscribe(s_right_raw_topic,  1,  &stereoRaw::rightCvBridge_raw, this);
		SaveTrigger_sub	 = n.subscribe(s_raw_trigger_topic,  1,  &stereoRaw::SaveRaw_wait, this);
		HastShutDown_sub = n.subscribe("/hast/shutdown",           10, &stereoRaw::nodeShutDown, this);

		ROS_INFO("stereoRaw: Watching..");
	}//close constructor

/* ################## --------- ROS Communication Functions --------- ################## */

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{ // node cleanup for end of experiment
		if(ShutDown->flag)
			{
				ROS_INFO("stereoRaw: Stereo Obs Shutting Down...");
				ros::shutdown();
			}
	}

	void leftCvBridge_raw(const sensor_msgs::ImageConstPtr& msg)
	{
		leftStamp_raw = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			LeftRawIamge = cv_ptr->image;
		}//end try
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}//end catch
	} // end void Cbl

	void rightCvBridge_raw(const sensor_msgs::ImageConstPtr& msg)
	{
		rightStamp_raw = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			RightRawIamge = cv_ptr->image;
			// showImage(LeftRawIamge, "right");
		}//end try
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}//end catch
	} // end void CBr

	void SaveRaw_wait(const std_msgs::EmptyConstPtr& msg)
	{ // Null_msg;
		ROS_INFO("stereoRaw: Saving Raw images %i", leftraw_count+1);

		RawSaveTime = ros::Time::now().toSec();
		s_imString = s_root + s_date + "/" + s_trial + "/original/left_raw_" + patch::to_string(++leftraw_count) + ".png";
		imwrite(s_imString.c_str(), LeftRawIamge);
		// showImage(LeftRawIamge, "left");
		// fprintf (pFile,"%s.leftraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), leftraw_count, RawSaveTime-init_time); // this one needs to be here for plotting purposes
		s_imString = s_root + s_date + "/" + s_trial + "/original/right_raw_" + patch::to_string(++rightraw_count) + ".png";
		imwrite(s_imString.c_str(), RightRawIamge);
		// showImage(RightRawIamge, "right");
		// fprintf (pFile,"%s.rightraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), rightraw_count, RawSaveTime-init_time); // this one needs to be here for plotting purposes
	}




/* ################## --------- Image Tool Functions --------- ################## */

	void showImage(cv::Mat image, const char* name)
	{
		 cv::namedWindow(name, 1);
		 cv::imshow(name, image);
		 cv::waitKey(5);
	}

	void saveImage(cv::Mat image, const char* name)
	{
		std::string s_savename;
		s_savename =  s_root + s_date + "/" + name + ".png";
		imwrite(s_savename.c_str(), image);//, compression_params);
	}


}; //end stereoRaw class

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereoLocalizer");
	stereoRaw dP;
	ros::spin();
	return 0;
}
