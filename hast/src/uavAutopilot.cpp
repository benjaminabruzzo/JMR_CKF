#include "genheaders.hpp"

#include <robot_plugins/pid_controller.h>
#include <robot_plugins/pid_controllers.h>

class PIDController {
	public:
		double gain_p;
		double gain_i;
		double gain_d;
		double time_constant;
		double limit;

		double input;
		double dinput;
		double u_cmd;
		double p, i, d;


		PIDController()
		{
			gain_p = 0.0;
			gain_d = 0.0;
			gain_i = 0.0;
			time_constant = 0.0;
			limit = -1.0;

			pid_msg.errors.p = 0;
			pid_msg.errors.i = 0;
			pid_msg.errors.d = 0;
			pid_msg.gains.p = gain_p;
			pid_msg.gains.i = gain_i;
			pid_msg.gains.d = gain_d;

			pid_msg.input = 0;
			pid_msg.dinput = 0;
			pid_msg.time_constant = time_constant;
			pid_msg.limit = limit;
			pid_msg.output = 0;

		}

		~PIDController(){}


		robot_plugins::pid_controller pid_msg;
		std::string name;

		void updatePID2(double set_point, double x)
		{
			pid_msg.state = x;
			double state_error = set_point - x;
			double last_p = p;

			p  = gain_p * state_error; // state error control
			i += gain_i * state_error;
			d  = gain_d*(p - last_p); // control the change in control-gain

			u_cmd = p + i + d; // + gain_d * d + gain_i * i;

			// limit command
			if (fabs(u_cmd) > limit)
			{
				u_cmd = (u_cmd < 0 ? -1.0 : 1.0)*limit; // check if u_cmd is positive or negative, then apply that sign to the limit
			}

		}

		void updatePID(double new_input, double x, double dx, double dt)
		{
			// filter command
			if (dt > 0.0) {
				dinput = (new_input - input) / (dt);
				input  = (dt * new_input ) / (dt);
			}

			// update proportional, differential and integral errors
			pid_msg.state = x;
			p = input - x;
			d = dinput - dx;
			i = i + dt * p;

			// update control output
			u_cmd = gain_p * p + gain_d * d + gain_i * i;

			// limit command
			if (fabs(u_cmd) > limit)
			{
				u_cmd = (u_cmd < 0 ? -1.0 : 1.0)*limit; // check if u_cmd is positive or negative, then apply that sign to the limit
			}
		}

		void reset()
		{
			input = dinput = 0;
			p = i = d = u_cmd = 0;
		}

		void publishPID(double pub_time)
		{
			pid_msg.name = name;

			pid_msg.errors.p = p;
			pid_msg.errors.i = i;
			pid_msg.errors.d = d;
			pid_msg.gains.p = gain_p;
			pid_msg.gains.i = gain_i;
			pid_msg.gains.d = gain_d;

			pid_msg.input = input;
			pid_msg.dinput = dinput;
			pid_msg.time_constant = time_constant;
			pid_msg.limit = limit;
			pid_msg.output = u_cmd;
		}

		void initFileRecorder()
		{

		}

		void writeToFile()
		{

		}



  };

class uavCon
{
	private:
		struct ugvData
		{
			std::string s_EstimatedState_sub;
			ros::Subscriber EstimatedState_sub;
				double StateTS, LastStateTS; // timestamp of ugv messages

			cv::Mat CurrentP_g; // most current global position of ugv
			cv::Mat CurrentR_g2ugv; // most current estimated rotation of global to ugv frame
			double CurrentYaw;  // most current estimated yaw of ugv in global frame
			cv::Mat DesiredP_ugv, DesiredV_ugv; // desired P and V of uav relative to ugv
			double DesiredYaw_ugv; // desired yaw of uav relative to ugv
			double cosyaw, sinyaw; // sin and cosine variables to update rotation matrix for ugv

			bool holdsPicket;

		};

	public:
		double Pi;

		/*---------  File Recorder ------------- */
		std::string s_filename, s_prealloc, s_trial, s_dotm, s_root, s_handle, s_date, s_user, s_exp_code;
		std::FILE * pFile;
		std::FILE * allocFile;


		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;

		std::string s_PicketService_ser, s_ControlService_ser;
		ros::ServiceClient callTime_cli; // client to request master ros time offset
			hast::servtime time_call;
			bool gazebo;

		ros::Publisher DroneCmd_dr_pub;
			geometry_msgs::Twist DroneCmd_dr_msg;
			double YawRateCommand;
			std::string s_uavcmd_topic;

		ros::Publisher AtPosition_pub;
			hast::flag AtPosition_msg;
			bool atPos;

		ros::Subscriber StateVector_sub, HastShutDown_sub;
		std::string s_StateVector_sub;
			double StateTS, LastStateTS;

			/*----- Client Services */
		ros::ServiceServer PicketService_ser;
		ros::ServiceServer ControlService_ser;
			bool ControlIsOn, DirectTiltBit, StateodomBIT, useBackstepping;
			double u_cmd_scale;
			int cmd_count;

		/*---------  PID Variables ------------- */
			cv::Mat CurrentP_g, CurrentV_g, CurrentA_g, CurrentR_g2dr;
			cv::Mat DesiredP_g, DesiredV_g, CmdRate;
			cv::Mat ErrorP_g, ErrorV_g;

			cv::Mat CurrentP_dr, CurrentV_dr, CurrentA_dr;
			cv::Mat DesiredP_dr, DesiredV_dr;
			cv::Mat ErrorP_dr, ErrorV_dr;  //error is the only vector that must be in drone frame

			double DesiredYaw, CurrentYaw, ErrorYaw, YawLimit;
			double cosyaw, sinyaw;

			cv::Mat PControl, VControl, PVControl;
			cv::Mat PControl_g, VControl_g, PVControl_g;
			cv::Mat PVthresh, PVctrl;
			cv::Vec<double,3> PD, TiltLimits;
			double ARDrone_Max_Euler;
			uint ctrlCount, hoverCount, msg_count;

		// ugv variables
			ros::Subscriber ugv1EstimatedState_sub, ugv2EstimatedState_sub;
				double ugv1StateTS, ugv1LastStateTS; // timestamp of ugv messages
				double ugv2StateTS, ugv2LastStateTS; // timestamp of ugv messages

			cv::Mat ugvCurrentP_g; // most current global position of ugv
			cv::Mat CurrentR_g2ugv; // most current estimated rotation of global to ugv frame
			double ugvCurrentYaw;  // most current estimated yaw of ugv in global frame
			cv::Mat DesiredP_ugv, DesiredV_ugv; // desired P and V of uav relative to ugv
			double DesiredYaw_ugv; // desired yaw of uav relative to ugv
			double ugvcosyaw, ugvsinyaw; // sin and cosine variables to update rotation matrix for ugv

			bool isPicketing; // flag for guidance mode
			uint picketChanges; // counter for picket service

		// PIDController
			PIDController pos_X_PID, pos_Y_PID;
			PIDController vel_X_PID, vel_Y_PID;
			PIDController pitch_PID, roll_PID;
			double dt;

		double init_time;


		ugvData ugv1, ugv2;

	uavCon()
	{
		/*---------  File Recorder Initilizer ------------- */
		// s_root = "/home/turtlebot/BitSync/hast/Data/";
		// if(ros::param::get("~user",s_user)){} else {s_user = "BIIP";}
		// if(ros::param::get("~date",s_date)){} else {s_date = "BiiP";}
		// if(ros::param::get("~trial",s_trial)){} else {s_trial = "biip";}
		n.getParam("/hast/user",  s_user);
		n.getParam("/hast/date",  s_date);
		n.getParam("/hast/trial", s_trial);
		n.getParam("/hast/exp_code", s_exp_code);
		ros::param::get("~gazebo", gazebo);

		// if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}

		// ROS_WARN("uavAutopilot::BEEP");
		s_handle = "uavCon";
		ros::Duration(2).sleep();
		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		s_filename = s_root + s_date + "/" + s_exp_code + "/" + s_trial + "/" + s_handle + "_" + s_trial + s_dotm;
		ROS_INFO("uavCon: %s", s_filename.c_str());
		pFile = std::fopen (s_filename.c_str(),"w");
		fprintf (pFile,"%%clc; \n%%clear all;\n%%close all;\n\n");

		if (n.getParam("/hast/init_time", init_time)){} else {init_time = 0.0;}

		// if (n.getParam("/hast/init_time", init_time)){
		// 	callTime_cli = n.serviceClient<hast::servtime>("/hast/serve/time", true);
		// 	time_call.request.requesting_node = "uavCon";
		// 	ros::Time calltime = ros::Time::now();
		// 	callTime_cli.call(time_call);
		// 	ros::Time mastertime = time_call.response.header.stamp;
		// 	if (gazebo){
		// 		init_time = 0.0;
		// 	} else {
		// 		ROS_INFO("%s::init_time = %f", time_call.request.requesting_node.c_str(), init_time+=(calltime.toSec() - mastertime.toSec()));
		// 	}
		// } else {init_time = 0.0;}
		fprintf (pFile,"uavCon.init_time  = % -6.8f;\n", init_time);


		Pi = atan(1) * 4; // 3.14159...
		/*--------- Initialize ROS Communication & Variables ------------- */
		HastShutDown_sub 	= n.subscribe("/hast/shutdown",		1, &uavCon::nodeShutDown		, this); fprintf (pFile,"%s.topics.shutdown = '/hast/shutdown';\n", s_handle.c_str());

		ros::param::get("~uav_state_sub",				s_StateVector_sub); 					fprintf (pFile,"%s.topics.uav_state_sub = '%s';\n", 					s_handle.c_str(),	s_StateVector_sub.c_str());
		ros::param::get("~uav_cmd_topic",				s_uavcmd_topic); 							fprintf (pFile,"%s.topics.uav_cmd_topic = '%s';\n", 					s_handle.c_str(), s_uavcmd_topic.c_str());
		ros::param::get("~PicketService_ser", 	s_PicketService_ser);					fprintf (pFile,"%s.topics.PicketService_ser = '%s';\n",				s_handle.c_str(), s_PicketService_ser.c_str());
		ros::param::get("~ControlService_ser",	s_ControlService_ser);				fprintf (pFile,"%s.topics.ControlService_ser = '%s';\n",			s_handle.c_str(), s_ControlService_ser.c_str());
		ros::param::get("~ugv1EstimatedState_sub",ugv1.s_EstimatedState_sub);	fprintf (pFile,"%s.topics.ugv1.EstimatedState_sub = '%s';\n",	s_handle.c_str(), ugv1.s_EstimatedState_sub.c_str());
		ros::param::get("~ugv2EstimatedState_sub",ugv2.s_EstimatedState_sub);	fprintf (pFile,"%s.topics.ugv2.EstimatedState_sub = '%s';\n",	s_handle.c_str(), ugv2.s_EstimatedState_sub.c_str());

		/*-----  Publishers and Subscribers */
		StateVector_sub 				= n.subscribe(s_StateVector_sub,	1, &uavCon::ReadEstimatedState, this);
		DroneCmd_dr_pub 				= n.advertise<geometry_msgs::Twist>	(s_uavcmd_topic, 1);
		ugv1.EstimatedState_sub = n.subscribe(ugv1.s_EstimatedState_sub,	1,	&uavCon::ugv1ReadState, this);
		ugv2.EstimatedState_sub = n.subscribe(ugv2.s_EstimatedState_sub,	1,	&uavCon::ugv2ReadState, this);

		/*-----  Servers and Clients */
		PicketService_ser 		= n.advertiseService(s_PicketService_ser, 	&uavCon::changePicket, this);
		ControlService_ser 		= n.advertiseService(s_ControlService_ser, 	&uavCon::changeState,  this);

		cosyaw = 0;
		sinyaw = 0;
		StateTS = 0;
		LastStateTS = 0;

		/*--------- Initialize PID Variables ------------- */
		CurrentP_g		= (cv::Mat_<double>(3,1) << 0,0,0.5);
		CurrentP_dr		= (cv::Mat_<double>(3,1) << 0,0,0.5);
		CurrentV_g		= (cv::Mat_<double>(3,1) << 0,0,0);
		CurrentV_dr		= (cv::Mat_<double>(3,1) << 0,0,0);
		CurrentR_g2dr	= (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
		CurrentYaw 		= 0;
		DesiredP_g 		= (cv::Mat_<double>(3,1) << 2,0,1.5);
		CmdRate 			= (cv::Mat_<double>(4,1) << 0,0,0,0);
		DesiredP_dr	 	= (cv::Mat_<double>(3,1) << 0,0,1.5);
		DesiredV_g 		= (cv::Mat_<double>(3,1) << 0,0,0);
		DesiredV_dr 	= (cv::Mat_<double>(3,1) << 0,0,0);
		DesiredYaw 		= 0;
		ErrorP_g		= (cv::Mat_<double>(3,1) << 0,0,0);
		ErrorP_dr		= (cv::Mat_<double>(3,1) << 0,0,0);
		ErrorV_g		= (cv::Mat_<double>(3,1) << 0,0,0);
		ErrorV_dr		= (cv::Mat_<double>(3,1) << 0,0,0);
		ErrorYaw 		= 0;
		PControl		= (cv::Mat_<double>(3,1) << 0,0,0);
		VControl		= (cv::Mat_<double>(3,1) << 0,0,0);
		PVControl		= (cv::Mat_<double>(3,1) << 0,0,0);
		PControl_g		= (cv::Mat_<double>(3,1) << 0,0,0);
		VControl_g		= (cv::Mat_<double>(3,1) << 0,0,0);
		PVControl_g 	= (cv::Mat_<double>(3,1) << 0,0,0);
		PVthresh		= (cv::Mat_<double>(3,1) << 0,0,0);
		PVctrl			= (cv::Mat_<double>(3,1) << 0,0,0);

		// ugv variables
		CurrentR_g2ugv	= (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
		ugvcosyaw = 0;
		ugvsinyaw = 0;

		DesiredP_ugv	= (cv::Mat_<double>(3,1) << 2,0,1);
		DesiredYaw_ugv  = 0;
		DesiredV_ugv 	= (cv::Mat_<double>(3,1) << 0,0,0);

		ugvCurrentYaw = 0;
		ugv1StateTS = 0;
		ugv2StateTS = 0;
		ugv1LastStateTS = 0;
		ugv2LastStateTS = 0;

		cmd_count = 0; // total number of commands
		ctrlCount = 0; 	// number of control commands
		hoverCount = 0; // number of hover commands
		msg_count = 0; //number of received messages


		ARDrone_Max_Euler = 12.0;	// Max tilt angle for roll and pitch in degrees

		// YawLimit = 0.3; //upped for tilt testing to 0.75, 0.3 nominal
		DirectTiltBit = false;


		ControlIsOn = false; // off by default?
		isPicketing = false;
		ugv1.holdsPicket = false;
		ugv2.holdsPicket = false;
		picketChanges = 0;

		// ROS_WARN("uavAutopilot::BEEP BEEP BEEP");

		if(ros::param::get("~u_cmd_scale",u_cmd_scale)){} else {u_cmd_scale = 1.0;}

		// more than 0.4 is too aggressive
		if(ros::param::get("~Kp",PD[0])){} else {PD[0] = 0.2;} // proportional gain on position control
		if(ros::param::get("~Kv",PD[1])){} else {PD[1] = 0.15;} // proportional gain on velocity damper
		if(ros::param::get("~Ky",PD[2])){} else {PD[2] = 0.02;} // yaw control // 0.05 is TOO HIGH for angular control
		if(ros::param::get("~MinTilt",TiltLimits[0])){} else {TiltLimits[0] = 0.0001;} // proportional gain on position control
		if(ros::param::get("~MaxTilt",TiltLimits[1])){} else {TiltLimits[1] = 0.25;} // proportional gain on velocity damper
		if(ros::param::get("~MaxYawRate",TiltLimits[2])){} else {TiltLimits[2] = 0.4;} // yaw control // 0.05 is TOO HIGH for angular control

		// TiltLimits[0] = 0.0001;	// min threshold for tilt on any one axis in degrees
		// TiltLimits[1] = 0.2;	// max threshold for tilt on any one axis in degrees
		//0.005 is too minimal as a max angle.  it over saturates
		// TiltLimits[2] = 0.3;		//  yaw limit
		// YawLimit = 0.3; // Normal for PV controller
		ROS_INFO("PV: [% -6.8f % -6.8f % -6.8f]", PD[0], PD[1], PD[2]);
		ROS_INFO("TiltLimits: [% -6.8f % -6.8f % -6.8f]", TiltLimits[0], TiltLimits[1], TiltLimits[2]);

		fprintf (pFile,"uavCon.PD  = [% -6.8f, % -6.8f, % -6.8f];\n", PD[0], PD[1], PD[2]);
		fprintf (pFile,"uavCon.TiltLimits  = [% -6.8f, % -6.8f, % -6.8f];\n", TiltLimits[0], TiltLimits[1], TiltLimits[2]);
		fprintf (pFile,"uavCon.ARDrone_Max_Euler  = % -6.8f;\n", ARDrone_Max_Euler);

		if(ros::param::get("~useBackstepping",useBackstepping)){} 		else {useBackstepping = false;}

			if(ros::param::get("~posXY_gain_p",pos_X_PID.gain_p)){} 		else {pos_X_PID.gain_p = 5.0;}
			if(ros::param::get("~posXY_gain_i",pos_X_PID.gain_i)){} 		else {pos_X_PID.gain_i = 0.0;}
			if(ros::param::get("~posXY_gain_d",pos_X_PID.gain_d)){} 		else {pos_X_PID.gain_d = 1.0;}
			if(ros::param::get("~posXY_limit" ,pos_X_PID.limit)){} 			else {pos_X_PID.limit = 2.0;}
			if(ros::param::get("~posXY_time_k",pos_X_PID.time_constant)){} 	else {pos_X_PID.time_constant = 0.0;}

			if(ros::param::get("~posXY_gain_p",pos_Y_PID.gain_p)){} 		else {pos_Y_PID.gain_p = 5.0;}
			if(ros::param::get("~posXY_gain_i",pos_Y_PID.gain_i)){} 		else {pos_Y_PID.gain_i = 0.0;}
			if(ros::param::get("~posXY_gain_d",pos_Y_PID.gain_d)){} 		else {pos_Y_PID.gain_d = 1.0;}
			if(ros::param::get("~posXY_limit" ,pos_Y_PID.limit)){} 			else {pos_Y_PID.limit = 2.0;}
			if(ros::param::get("~posXY_time_k",pos_Y_PID.time_constant)){} 	else {pos_Y_PID.time_constant = 0.0;}

			if(ros::param::get("~velXY_gain_p",vel_X_PID.gain_p)){} 		else {vel_X_PID.gain_p = 5.0;}
			if(ros::param::get("~velXY_gain_i",vel_X_PID.gain_i)){} 		else {vel_X_PID.gain_i = 0.0;}
			if(ros::param::get("~velXY_gain_d",vel_X_PID.gain_d)){} 		else {vel_X_PID.gain_d = 1.0;}
			if(ros::param::get("~velXY_limit" ,vel_X_PID.limit)){} 			else {vel_X_PID.limit = 2.0;}
			if(ros::param::get("~velXY_time_k",vel_X_PID.time_constant)){} 	else {vel_X_PID.time_constant = 0.0;}

			if(ros::param::get("~velXY_gain_p",vel_Y_PID.gain_p)){} 		else {vel_Y_PID.gain_p = 5.0;}
			if(ros::param::get("~velXY_gain_i",vel_Y_PID.gain_i)){} 		else {vel_Y_PID.gain_i = 0.0;}
			if(ros::param::get("~velXY_gain_d",vel_Y_PID.gain_d)){} 		else {vel_Y_PID.gain_d = 1.0;}
			if(ros::param::get("~velXY_limit" ,vel_Y_PID.limit)){} 			else {vel_Y_PID.limit = 2.0;}
			if(ros::param::get("~velXY_time_k",vel_Y_PID.time_constant)){} 	else {vel_Y_PID.time_constant = 0.0;}

			if(ros::param::get("~RP_gain_p",roll_PID.gain_p)){} 		else {roll_PID.gain_p = 10.0;}
			if(ros::param::get("~RP_gain_i",roll_PID.gain_i)){} 		else {roll_PID.gain_i = 0.0;}
			if(ros::param::get("~RP_gain_d",roll_PID.gain_d)){} 		else {roll_PID.gain_d = 5.0;}
			if(ros::param::get("~RP_limit" ,roll_PID.limit)){} 			else {roll_PID.limit = 0.5;}
			if(ros::param::get("~RP_time_k",roll_PID.time_constant)){} 	else {roll_PID.time_constant = 0.0;}

			if(ros::param::get("~RP_gain_p",pitch_PID.gain_p)){} 		else {pitch_PID.gain_p = 10.0;}
			if(ros::param::get("~RP_gain_i",pitch_PID.gain_i)){} 		else {pitch_PID.gain_i = 0.0;}
			if(ros::param::get("~RP_gain_d",pitch_PID.gain_d)){} 		else {pitch_PID.gain_d = 5.0;}
			if(ros::param::get("~RP_limit" ,pitch_PID.limit)){} 		else {pitch_PID.limit = 0.5;}
			if(ros::param::get("~RP_time_k",pitch_PID.time_constant)){} else {pitch_PID.time_constant = 0.0;}

			// ROS_WARN("pos_X_PID: [% -6.8f % -6.8f % -6.8f]", pos_X_PID.gain_p, pos_X_PID.gain_i, pos_X_PID.gain_d);
			// ROS_WARN("pos_Y_PID: [% -6.8f % -6.8f % -6.8f]", pos_Y_PID.gain_p, pos_Y_PID.gain_i, pos_Y_PID.gain_d);
			// ROS_WARN("vel_X_PID: [% -6.8f % -6.8f % -6.8f]", vel_X_PID.gain_p, vel_X_PID.gain_i, vel_X_PID.gain_d);
			// ROS_WARN("vel_Y_PID: [% -6.8f % -6.8f % -6.8f]", vel_Y_PID.gain_p, vel_Y_PID.gain_i, vel_Y_PID.gain_d);
			// ROS_WARN("roll_PID:  [% -6.8f % -6.8f % -6.8f]", roll_PID.gain_p, roll_PID.gain_i, roll_PID.gain_d);
			// ROS_WARN("pitch_PID: [% -6.8f % -6.8f % -6.8f]", pitch_PID.gain_p, pitch_PID.gain_i, pitch_PID.gain_d);

			fprintf (pFile,"uavCon.pos_X_PID.gains  = [% -6.8f, % -6.8f, % -6.8f];\n", pos_X_PID.gain_p, pos_X_PID.gain_i, pos_X_PID.gain_d);
			fprintf (pFile,"uavCon.pos_Y_PID.gains  = [% -6.8f, % -6.8f, % -6.8f];\n", pos_Y_PID.gain_p, pos_Y_PID.gain_i, pos_Y_PID.gain_d);
			fprintf (pFile,"uavCon.vel_X_PID.gains  = [% -6.8f, % -6.8f, % -6.8f];\n", vel_X_PID.gain_p, vel_X_PID.gain_i, vel_X_PID.gain_d);
			fprintf (pFile,"uavCon.vel_Y_PID.gains  = [% -6.8f, % -6.8f, % -6.8f];\n", vel_Y_PID.gain_p, vel_Y_PID.gain_i, vel_Y_PID.gain_d);
			fprintf (pFile,"uavCon.roll_PID.gains   = [% -6.8f, % -6.8f, % -6.8f];\n", roll_PID.gain_p, roll_PID.gain_i, roll_PID.gain_d);
			fprintf (pFile,"uavCon.pitch_PID.gains  = [% -6.8f, % -6.8f, % -6.8f];\n", pitch_PID.gain_p, pitch_PID.gain_i, pitch_PID.gain_d);
	}

	bool changeState(hast::uavcontrolstate::Request &req, hast::uavcontrolstate::Response &res)
	{
		if (req.flip)
		{
			DesiredP_g = (cv::Mat_<double>(3,1) << req.dP_g.x,req.dP_g.y,req.dP_g.z);
			CmdRate = (cv::Mat_<double>(4,1) << req.tilt.x,req.tilt.y,req.tilt.z, req.YawRate);
			DirectTiltBit = req.directtilt;
			// DesiredYaw = req.dYaw;
			ControlIsOn = true;
			res.onoff = true;
			// ROS_INFO("uavCon: ControlIsOn : ? = %s", ControlIsOn ? "true" : "false" );
			// ROS_INFO("uavCon: DesiredP_g: [% -6.8f % -6.8f % -6.8f ]", req.dP_g.x,req.dP_g.y,req.dP_g.z);
			// ROS_INFO("uavCon: DirectTiltBit : ? = %s", DirectTiltBit ? "true" : "false" );
			if (DirectTiltBit)
				{ROS_INFO("uavCon: DC_CMDRate: [% -6.8f % -6.8f % -6.8f % -6.8f ]", req.tilt.x,req.tilt.y,req.tilt.z, req.YawRate);}

			// ROS_INFO("uavCon: /*--------- run controller ------------- */");
		}
		else
		{
			ControlIsOn = false;
			res.onoff = false;

			// ROS_INFO("uavCon: /*--------- stop controller ------------- */");
		}
		isPicketing = false;

		// ROS_INFO("ControlIsOn 	= %s", ControlIsOn 	 ? "true" : "false" );
		// ROS_INFO("DirectTiltBit = %s", DirectTiltBit ? "true" : "false" );
		// ROS_INFO("isPicketing 	= %s", isPicketing   ? "true" : "false" );

		return true;
	}

	bool changePicket(hast::uavpicket::Request &req, hast::uavpicket::Response &res)
	{
		if (req.flip)
		{
			// turn on control and set response
			ControlIsOn = true;
			res.onoff = true;
			DesiredP_ugv = (cv::Mat_<double>(3,1) << req.xyz.x,req.xyz.y,req.xyz.z);
			// DesiredYaw_ugv = req.yaw;
			// ROS_INFO("uavCon: ControlIsOn : ? = %s", ControlIsOn ? "true" : "false" );
			// ROS_INFO("uavCon: DirectTiltBit : ? = %s", DirectTiltBit ? "true" : "false" );
			// ROS_INFO("uavCon: /*--------- run controller ------------- */");
			ROS_WARN("uavCon: changePicket: [% -6.2f % -6.2f % -6.2f % -6.2f ]", req.xyz.x,req.xyz.y,req.xyz.z,req.yaw);
			fprintf (pFile,"\nuavCon.picket.time(%d)  = [% -6.8f];\n", ++picketChanges, ros::Time::now().toSec()-init_time);
			fprintf (pFile,"  uavCon.picket.DesiredP_ugv(%d, :) = [% -6.8f % -6.8f % -6.8f];\n", picketChanges, req.xyz.x,req.xyz.y,req.xyz.z);
			fprintf (pFile,"  uavCon.picket.DesiredYaw_ugv(%d, :) = [% -6.8f];\n", picketChanges, req.yaw);
		}
		else
		{
			ControlIsOn = false;
			res.onoff = false;
			// ROS_INFO("uavCon: /*--------- stop controller ------------- */");
		}
		isPicketing = true;
		if (req.ugv_id.compare("ugv1") == 0) {ugv1.holdsPicket = true; ugv2.holdsPicket = false;}
		if (req.ugv_id.compare("ugv2") == 0) {ugv2.holdsPicket = true; ugv1.holdsPicket = false;}

		// ROS_INFO("uavCon: ControlIsOn : ? = %s", ControlIsOn ? "true" : "false" );
		// ROS_INFO("uavCon: isPicketing : ? = %s", isPicketing ? "true" : "false" );
		return true;
	}

	void ugv1ReadState(const hast::ugvstate::ConstPtr& ugvState_msg)
	{
		/*--------- Has new data been published? ------------- */

		ugv1.StateTS = ugvState_msg -> stamp;
		if ((ugv1.StateTS!=ugv1.LastStateTS))
		{ /*----- yes */
			ugv1.LastStateTS = ugv1.StateTS;
			// ROS_INFO("uavCon:ugvReadState: triggered");
			// ROS_WARN("uavCon: ControlIsOn : ? = %s", ControlIsOn ? "true" : "false" );
			// ROS_WARN("uavCon: isPicketing : ? = %s", isPicketing ? "true" : "false" );
			ugv1.CurrentP_g = (cv::Mat_<double>(3,1) << ugvState_msg->P.x, ugvState_msg->P.y, ugvState_msg->P.z);
			ugv1.CurrentYaw = ugvState_msg->yaw; // make sure in radians
				ugv1.cosyaw = cos(ugv1.CurrentYaw); // make sure in radians
				ugv1.sinyaw = sin(ugv1.CurrentYaw); // make sure in radians
			ugv1.CurrentR_g2ugv	= (cv::Mat_<double>(3,3) <<
					           ugv1.cosyaw, ugv1.sinyaw, 0,
					          -ugv1.sinyaw, ugv1.cosyaw, 0,
					           0, 0, 1);

			if(ugv1.holdsPicket)
			{/*----- yes : run controller */
				DesiredP_g = ugv1.CurrentR_g2ugv.t() * DesiredP_ugv + ugv1.CurrentP_g;
				ROS_WARN("uavCon: DesiredP_g: [% -6.2f % -6.2f % -6.2f ]", DesiredP_g.at<double>(0,0), DesiredP_g.at<double>(1,0), DesiredP_g.at<double>(2,0));
				DesiredV_g = CurrentR_g2ugv * DesiredV_ugv;
				// DesiredYaw = DesiredYaw_ugv + ugv1.CurrentYaw;
			}
		}
	}

	void ugv2ReadState(const hast::ugvstate::ConstPtr& ugvState_msg)
	{
		/*--------- Has new data been published? ------------- */

		ugv2.StateTS = ugvState_msg -> stamp;
		if ((ugv2.StateTS!=ugv2.LastStateTS))
		{ /*----- yes */
			ugv2.LastStateTS = ugv2.StateTS;
			// ROS_INFO("uavCon:ugvReadState: triggered");
			// ROS_WARN("uavCon: ControlIsOn : ? = %s", ControlIsOn ? "true" : "false" );
			// ROS_WARN("uavCon: isPicketing : ? = %s", isPicketing ? "true" : "false" );
			ugv2.CurrentP_g = (cv::Mat_<double>(3,1) << ugvState_msg->P.x, ugvState_msg->P.y, ugvState_msg->P.z);
			ugv2.CurrentYaw = ugvState_msg->yaw; // make sure in radians
				ugv2.cosyaw = cos(ugv2.CurrentYaw); // make sure in radians
				ugv2.sinyaw = sin(ugv2.CurrentYaw); // make sure in radians
			ugv2.CurrentR_g2ugv	= (cv::Mat_<double>(3,3) <<
					           ugv2.cosyaw, ugv2.sinyaw, 0,
					          -ugv2.sinyaw, ugv2.cosyaw, 0,
					           0, 0, 1);

			if(ugv2.holdsPicket)
			{/*----- yes : run controller */
				DesiredP_g = ugv2.CurrentR_g2ugv.t() * DesiredP_ugv + ugv2.CurrentP_g;
				ROS_WARN("uavCon: DesiredP_g: [% -6.2f % -6.2f % -6.2f ]", DesiredP_g.at<double>(0,0), DesiredP_g.at<double>(1,0), DesiredP_g.at<double>(2,0));
				DesiredV_g = CurrentR_g2ugv * DesiredV_ugv;
				// DesiredYaw = DesiredYaw_ugv + ugv2.CurrentYaw;
			}
		}
	}

	void ReadEstimatedState(const hast::uavstate::ConstPtr& uavState_msg)
	{//updates uav control each time the filter is updated regardless of whether or not picket is being used

		/*--------- Has new data been published? ------------- */
		StateTS = uavState_msg -> stamp;
		if ((StateTS!=LastStateTS))
		{ /*----- yes */
		/*------- Update Estmated State*/
		dt = StateTS - LastStateTS;
		if (dt > 0.25){dt = 0.05;}
		LastStateTS = StateTS;
		CurrentYaw = uavState_msg->yaw;
		CurrentR_g2dr = (cv::Mat_<double>(3,3) << 	uavState_msg->R.row0.x, uavState_msg->R.row0.y, uavState_msg->R.row0.z,
																								uavState_msg->R.row1.x, uavState_msg->R.row1.y, uavState_msg->R.row1.z,
																								uavState_msg->R.row2.x, uavState_msg->R.row2.y, uavState_msg->R.row2.z);
		CurrentP_g = (cv::Mat_<double>(3,1) << uavState_msg->P.x, uavState_msg->P.y, uavState_msg->P.z);
		CurrentV_g = (cv::Mat_<double>(3,1) << uavState_msg->V.x, uavState_msg->V.y, 0);
		CurrentA_g = (cv::Mat_<double>(3,1) << uavState_msg->A.x, uavState_msg->A.y, uavState_msg->A.z);

		msg_count +=1;
		fprintf (pFile,"\nuavCon.msg.time(%d,:) = % -6.8f;\n", msg_count, ros::Time::now().toSec()-init_time);
		fprintf (pFile,"  uavCon.msg.Current.Yaw(%d,:)  = % -6.8f;\n", msg_count, CurrentYaw);
		fprintf (pFile,"  uavCon.msg.Current.Position_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", msg_count, CurrentP_g.at<double>(0,0), CurrentP_g.at<double>(1,0), CurrentP_g.at<double>(2,0));

			// StateodomBIT = uavState_msg -> odomBIT;
			// if(ControlIsOn && StateodomBIT)
			if(ControlIsOn)
			{/*----- yes : run controller */
				// ROS_INFO("uavCon:ControlIsOn: DesiredP_g: [% -6.2f % -6.2f % -6.2f ]", DesiredP_g.at<double>(0,0), DesiredP_g.at<double>(1,0), DesiredP_g.at<double>(2,0));
				// ROS_INFO("uavCon: Drone Flight Service Called: Waiting for new data");

					// ROS_INFO("uavCon: New data, calculating errors:");
					++ctrlCount;

					CurrentP_dr	= CurrentR_g2dr * CurrentP_g;
					CurrentV_dr = CurrentR_g2dr * CurrentV_g;
					CurrentA_dr = CurrentR_g2dr * CurrentA_g;

					DesiredV_dr = (cv::Mat_<double>(3,1) <<  DroneCmd_dr_msg.linear.x, DroneCmd_dr_msg.linear.y, DroneCmd_dr_msg.linear.z);

					// use stereo altitude
					CurrentP_dr.at<double>(2,0) = uavState_msg->P.z;
					DesiredP_dr	= CurrentR_g2dr * DesiredP_g;

					/*------- Update Controller */
					ErrorP_g 	= DesiredP_g - CurrentP_g;
					ErrorV_g 	= DesiredV_g - CurrentV_g;
					ErrorP_dr	= CurrentR_g2dr * ErrorP_g;
					ErrorV_dr	= CurrentR_g2dr * ErrorV_g;

					DesiredYaw = 0; // just force it right now
					ErrorYaw 	= DesiredYaw - CurrentYaw;

					fprintf (pFile,"\nuavCon.time(%d,:) = % -6.8f;\n", ctrlCount, ros::Time::now().toSec()-init_time);
					// fprintf (pFile,"  uavCon.CmdRate(%d,:)  = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CmdRate.at<double>(0,0), CmdRate.at<double>(1,0), CmdRate.at<double>(2,0), CmdRate.at<double>(3,0));
					fprintf (pFile,"  uavCon.Desired.Position_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DesiredP_g.at<double>(0,0), DesiredP_g.at<double>(1,0), DesiredP_g.at<double>(2,0));
					fprintf (pFile,"  uavCon.Current.Position_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentP_g.at<double>(0,0), CurrentP_g.at<double>(1,0), CurrentP_g.at<double>(2,0));
					// fprintf (pFile,"  uavCon.Desired.Position_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DesiredP_dr.at<double>(0,0), DesiredP_dr.at<double>(1,0), DesiredP_dr.at<double>(2,0));
					// fprintf (pFile,"  uavCon.Current.Position_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentP_dr.at<double>(0,0), CurrentP_dr.at<double>(1,0), CurrentP_dr.at<double>(2,0));
					// // fprintf (pFile,"  uavCon.Desired.Position_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DesiredP_ugv.at<double>(0,0), DesiredP_ugv.at<double>(1,0), DesiredP_ugv.at<double>(2,0));
					// // fprintf (pFile,"  uavCon.Current.Position_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentP_ugv.at<double>(0,0), CurrentP_dr.at<double>(1,0), CurrentP_dr.at<double>(2,0));
					fprintf (pFile,"  uavCon.Current.Yaw(%d,:) = % -6.8f;\n", ctrlCount, CurrentYaw);
					fprintf (pFile,"  uavCon.Desired.Yaw(%d,:) = % -6.8f;\n", ctrlCount, DesiredYaw);
					fprintf (pFile,"  uavCon.Error.Yaw(%d,:)   = % -6.8f;\n", ctrlCount, ErrorYaw);
					//
					// fprintf (pFile,"  uavCon.Desired.Velocity_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DesiredV_g.at<double>(0,0), DesiredV_g.at<double>(1,0), DesiredV_g.at<double>(2,0));
					// fprintf (pFile,"  uavCon.Current.Velocity_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentV_g.at<double>(0,0), CurrentV_g.at<double>(1,0), CurrentV_g.at<double>(2,0));
					// fprintf (pFile,"  uavCon.Desired.Velocity_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DesiredV_dr.at<double>(0,0), DesiredV_dr.at<double>(1,0), DesiredV_dr.at<double>(2,0));
					// fprintf (pFile,"  uavCon.Current.Velocity_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentV_dr.at<double>(0,0), CurrentV_dr.at<double>(1,0), CurrentV_dr.at<double>(2,0));
					//
					// // fprintf (pFile,"uavCon.Error.P_g(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, ErrorP_g.at<double>(0,0), ErrorP_g.at<double>(1,0), ErrorP_g.at<double>(2,0));
					// // fprintf (pFile,"uavCon.Error.V_g(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, ErrorV_g.at<double>(0,0), ErrorV_g.at<double>(1,0), ErrorV_g.at<double>(2,0));
					// // fprintf (pFile,"uavCon.Error.P_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, ErrorP_dr.at<double>(0,0), ErrorP_dr.at<double>(1,0), ErrorP_dr.at<double>(2,0));
					// // fprintf (pFile,"uavCon.Error.V_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, ErrorV_dr.at<double>(0,0), ErrorV_dr.at<double>(1,0), ErrorV_dr.at<double>(2,0));

					// fprintf (pFile,"  uavCon.Current.R_g2dr(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentR_g2dr.at<double>(0,0), CurrentR_g2dr.at<double>(0,1), CurrentR_g2dr.at<double>(0,2));
					// fprintf (pFile,"  uavCon.Current.R_g2dr(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentR_g2dr.at<double>(1,0), CurrentR_g2dr.at<double>(1,1), CurrentR_g2dr.at<double>(1,2));
					// fprintf (pFile,"  uavCon.Current.R_g2dr(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, CurrentR_g2dr.at<double>(2,0), CurrentR_g2dr.at<double>(2,1), CurrentR_g2dr.at<double>(2,2));

					// Make Z control proportional only
					ErrorV_dr.at<double>(2,0) = 0;

					dronePV();

					if (DirectTiltBit) {
						DroneCmd_dr_msg.linear.x = CmdRate.at<double>(0,0);
						DroneCmd_dr_msg.linear.y = CmdRate.at<double>(1,0);
						DroneCmd_dr_msg.linear.z = CmdRate.at<double>(2,0);
						DroneCmd_dr_msg.angular.z = CmdRate.at<double>(3,0);
					} else {
						DroneCmd_dr_msg.linear.x = PVControl.at<double>(0,0);
						DroneCmd_dr_msg.linear.y = PVControl.at<double>(1,0);
						DroneCmd_dr_msg.linear.z = 2*PVControl.at<double>(2,0);
						DroneCmd_dr_msg.angular.x = 0.0;
						DroneCmd_dr_msg.angular.y = 0.0;
						DroneCmd_dr_msg.angular.z = YawRateCommand;
					}

					if (useBackstepping)
					{
						uavPID();
						DroneCmd_dr_msg.linear.x = pos_X_PID.u_cmd * u_cmd_scale;
						DroneCmd_dr_msg.linear.y = pos_Y_PID.u_cmd * u_cmd_scale;
						// DroneCmd_dr_msg.linear.x = vel_X_PID.u_cmd;
						// DroneCmd_dr_msg.linear.y = vel_Y_PID.u_cmd;
					}


					// Convert control signal to ugv frame for postprocessing
					PVControl_g 	= CurrentR_g2dr.t() * (cv::Mat_<double>(3,1) << DroneCmd_dr_msg.linear.x, DroneCmd_dr_msg.linear.y, DroneCmd_dr_msg.linear.z);

					// fprintf (pFile,"  uavCon.cmd.linear_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DroneCmd_dr_msg.linear.x , DroneCmd_dr_msg.linear.y, DroneCmd_dr_msg.linear.z);
					// fprintf (pFile,"  uavCon.cmd.angular_dr(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DroneCmd_dr_msg.angular.x , DroneCmd_dr_msg.angular.y, DroneCmd_dr_msg.angular.z);
					// fprintf (pFile,"  uavCon.cmd.linear_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PVControl_g.at<double>(0,0), PVControl_g.at<double>(1,0), PVControl_g.at<double>(2,0));
					// fprintf (pFile,"  uavCon.cmd.angular_g(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, DroneCmd_dr_msg.angular.x , DroneCmd_dr_msg.angular.y, DroneCmd_dr_msg.angular.z);
					// fprintf (pFile,"  uavCon.cmd.yawrate(%d,1)    =  % -6.8f;\n", ctrlCount, YawRateCommand);


					// ROS_INFO("uavCon: Publishing Commands");
					// DroneCmd_dr_pub.publish(DroneCmd_dr_msg);
					cmdUAV(DroneCmd_dr_msg);
				}
			}	else{// hover
				// ROS_INFO("uavCon: Drone Flight Service Called: Hover");
				DroneCmd_dr_msg.linear.x = 0.0;
				DroneCmd_dr_msg.linear.y = 0.0;
				DroneCmd_dr_msg.linear.z = 0.0;
				DroneCmd_dr_msg.angular.z = 0.0;
				DroneCmd_dr_msg.angular.x = 0.0;
				DroneCmd_dr_msg.angular.y = 0.0;
				cmdUAV(DroneCmd_dr_msg);
				// DroneCmd_dr_pub.publish(DroneCmd_dr_msg);
				// fprintf (pFile,"\nuavCon.hover.Time(%d,:) = % -6.8f;\n", ++hoverCount, ros::Time::now().toSec());
				// fprintf (pFile,"  uavCon.hover.Cmd(%d,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", hoverCount, 0.0,0.0,0.0,0.0 );
			}
	}

	void uavPID()
	{
		// pos_X_PID.updatePID(DesiredP_dr.at<double>(0,0), CurrentP_dr.at<double>(0,0), CurrentV_dr.at<double>(0,0), dt);
		// pos_Y_PID.updatePID(DesiredP_dr.at<double>(1,0), CurrentP_dr.at<double>(1,0), CurrentV_dr.at<double>(1,0), dt);
		// vel_X_PID.updatePID(pos_X_PID.u_cmd, CurrentV_dr.at<double>(0,0), CurrentA_dr.at<double>(0,0), dt);
		// vel_Y_PID.updatePID(pos_Y_PID.u_cmd, CurrentV_dr.at<double>(1,0), CurrentA_dr.at<double>(1,0), dt);

		pos_X_PID.updatePID2(DesiredP_dr.at<double>(0,0), CurrentP_dr.at<double>(0,0));
		pos_Y_PID.updatePID2(DesiredP_dr.at<double>(1,0), CurrentP_dr.at<double>(1,0));
		vel_X_PID.updatePID2(pos_X_PID.u_cmd, CurrentV_dr.at<double>(0,0));
		vel_Y_PID.updatePID2(pos_Y_PID.u_cmd, CurrentV_dr.at<double>(1,0));


		// fprintf (pFile,"  uavCon.pos_PID.u_cmd(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, pos_X_PID.u_cmd, pos_Y_PID.u_cmd, 0.0);
		// fprintf (pFile,"  uavCon.vel_PID.u_cmd(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, vel_X_PID.u_cmd, vel_Y_PID.u_cmd, 0.0);
	}

	void dronePV()
	{
		// ROS_INFO("uavCon: dronePV()");
		PControl = PD[0] * ErrorP_dr;
		VControl = PD[1] * ErrorV_dr;
		PVControl = PControl + VControl;
		// Convert control signal to tb frame for postflight inspection
		PControl_g		= CurrentR_g2dr.t() * PControl;
		VControl_g		= CurrentR_g2dr.t() * VControl;
		PVControl_g 	= CurrentR_g2dr.t() * PVControl;

		// fprintf (pFile,"\n%% --- dronePV --- \n");
		// fprintf (pFile,"  uavCon.raw.PControl_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PControl.at<double>(0,0), PControl.at<double>(1,0), PControl.at<double>(2,0));
		// fprintf (pFile,"  uavCon.raw.VControl_dr(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, VControl.at<double>(0,0), VControl.at<double>(1,0), VControl.at<double>(2,0));
		// fprintf (pFile,"  uavCon.raw.PVControl_dr(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PVControl.at<double>(0,0), PVControl.at<double>(1,0), PVControl.at<double>(2,0));
		// fprintf (pFile,"  uavCon.raw.PControl_g(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PControl_g.at<double>(0,0), PControl_g.at<double>(1,0), PControl_g.at<double>(2,0));
		// fprintf (pFile,"  uavCon.raw.VControl_g(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, VControl_g.at<double>(0,0), VControl_g.at<double>(1,0), VControl_g.at<double>(2,0));
		// fprintf (pFile,"  uavCon.raw.PVControl_g(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PVControl_g.at<double>(0,0), PVControl_g.at<double>(1,0), PVControl_g.at<double>(2,0));

		// Prevent OverLarge Maneuvers
		if (PVControl.at<double>(0,0) >  TiltLimits[1]) {PVControl.at<double>(0,0) =  TiltLimits[1];}
		if (PVControl.at<double>(0,0) < -TiltLimits[1]) {PVControl.at<double>(0,0) = -TiltLimits[1];}

		if (PVControl.at<double>(1,0) >  TiltLimits[1]) {PVControl.at<double>(1,0) =  TiltLimits[1];}
		if (PVControl.at<double>(1,0) < -TiltLimits[1]) {PVControl.at<double>(1,0) = -TiltLimits[1];}

		// make altitude more aggressive
		if (PVControl.at<double>(2,0) >  2*TiltLimits[1]) {PVControl.at<double>(2,0) =  2*TiltLimits[1];}
		if (PVControl.at<double>(2,0) < -2*TiltLimits[1]) {PVControl.at<double>(2,0) = -2*TiltLimits[1];}

		// fprintf (pFile,"uavCon.PVCtrlTopLimit_dr(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PVControl.at<double>(0,0), PVControl.at<double>(1,0), PVControl.at<double>(2,0));

		// Prevent UnderSmall Maneuvers
		if (fabs(PVControl.at<double>(0,0)) < TiltLimits[0]) {PVControl.at<double>(0,0) = 0;}
		if (fabs(PVControl.at<double>(1,0)) < TiltLimits[0]) {PVControl.at<double>(1,0) = 0;}
		// fprintf (pFile,"uavCon.PVCtrlBotLimit_dr(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", ctrlCount, PVControl.at<double>(0,0), PVControl.at<double>(1,0), PVControl.at<double>(2,0));

		// If there is a null desired location
		// if (DesiredP_g(0,0) == 0.0) 	{PVthresh	= (cv::Mat_<double>(3,1) << 0,0,0);} // just hover

		// Increase Z-action
		// PVControl.at<double>(2,0) = PVControl.at<double>(2,0)*2;

		//Decrease Z-action
		// PVControl.at<double>(2,0) = PVControl.at<double>(2,0)*0.5;

		// ROS_INFO("uavCon: Lateral gains computed, updating yaw..");
		YawRateCommand = PD[2]*ErrorYaw; // 0.05 is TOO HIGH for angular control

		if (YawRateCommand >  TiltLimits[2]) {YawRateCommand =  TiltLimits[2];}
		if (YawRateCommand < -TiltLimits[2]) {YawRateCommand = -TiltLimits[2];}
	}

	void cmdUAV(geometry_msgs::Twist cmd_vel)
	{
		DroneCmd_dr_pub.publish(cmd_vel);
		cmd_count += 1;
		fprintf (pFile,"\nuavCon.cmd.time(%d,:) = % -6.8f;\n", cmd_count, ros::Time::now().toSec()-init_time);
		fprintf (pFile,"  uavCon.cmd.linear(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
		fprintf (pFile,"  uavCon.cmd.angular(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
		fprintf (pFile,"  uavCon.cmd.Desired.Position_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, DesiredP_g.at<double>(0,0), DesiredP_g.at<double>(1,0), DesiredP_g.at<double>(2,0));
		fprintf (pFile,"  uavCon.cmd.Current.Position_g(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, CurrentP_g.at<double>(0,0), CurrentP_g.at<double>(1,0), CurrentP_g.at<double>(2,0));
		fprintf (pFile,"  uavCon.cmd.Current.Yaw(%d,:)  = % -6.8f;\n", cmd_count, CurrentYaw);
		fprintf (pFile,"  uavCon.cmd.Desired.Yaw(%d,:)  = % -6.8f;\n", cmd_count, DesiredYaw);

		fprintf (pFile,"  try waitbar(%f/waitbar_max,wb); end\n", double(cmd_count));
	}


	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{
				s_prealloc = s_root + s_date + "/" + s_exp_code + "/" + s_trial + "/prealloc/pre_" + s_handle + "_" + s_trial + s_dotm;
				ROS_INFO("uavCon: s_prealloc.c_str() = %s", s_prealloc.c_str());
				allocFile = std::fopen (s_prealloc.c_str(), "w");

				fprintf (allocFile, "%% %s \n", s_prealloc.c_str());
				fprintf (allocFile, "%%clc; \n%%clear all;\n%%close all;\n\n");
				fprintf (allocFile, "  waitbar_max = %f;\n", double(cmd_count));
				fprintf (allocFile, "  wb = waitbar(0,' Loading uavCon ...');\n");

				if (ctrlCount>=1){
					fprintf (allocFile,"uavCon.time = zeros(%d,1);\n", ctrlCount);
					fprintf (allocFile,"uavCon.CmdRate = zeros(%d,4);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Desired.Position_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Current.Position_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Desired.Position_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Current.Position_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Current.Yaw = zeros(%d,1);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Desired.Yaw = zeros(%d,1);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.P_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.P_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.V_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.P_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.V_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Error.Yaw = zeros(%d,1);\n", ctrlCount);
					fprintf (allocFile,"uavCon.Current.R_g2dr = zeros(3,3,%d);\n\n", ctrlCount);

					// fprintf (allocFile,"uavCon.raw.PControl_dr = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.raw.VControl_dr = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.raw.PVControl_dr = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.raw.PControl_g = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.raw.VControl_g = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.raw.PVControl_g = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.PVCtrlTopLimit_dr = zeros(%d,3);\n", ctrlCount);
					// fprintf (allocFile,"uavCon.PVCtrlBotLimit_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.cmd.linear_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.cmd.angular_dr = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.cmd.yawrate = zeros(%d,1);\n", ctrlCount);
					fprintf (allocFile,"uavCon.cmd.linear_g = zeros(%d,3);\n", ctrlCount);
					fprintf (allocFile,"uavCon.cmd.angular_g = zeros(%d,3);\n", ctrlCount);
				}
				if (hoverCount>=1){
					fprintf (allocFile,"uavCon.hover.Time = zeros(%d,1);\n", hoverCount);
					fprintf (allocFile,"uavCon.hover.Cmd = zeros(%d,4);\n\n", hoverCount);
				}

				ROS_INFO("uavCon: Shutting Down...");
				fprintf (pFile, "\n%% --------------- close(wb) ---------------\n" );
				fprintf (pFile, "    try close(wb); end;\n");

				ros::shutdown();
			}
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uavCon");
	uavCon dC;
	ros::spin();
	return 0;
}
