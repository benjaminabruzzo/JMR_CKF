#include "hastUAV.hpp"

hastUAV::hastUAV() // void for construction of class
{
	// default
	s_parent_node = "default";

	// slamdata
	slamflag = false;

	// use compass bias term?
	if_useBias = false;

	// ROS comms
	stateMsg_id = 0;
	pose_msg_seq = 0;

	// Possibly useful constants
	Pi = atan(1) * 4; // 3.14159...
	L2Norm = 4; // Frobenius norm for CV norm() function

	// Initial estimated states of uav
	EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	EstPositionHom_g = (cv::Mat_<double>(4, 1) << EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0),1);

	correction_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	yawCorrection = 0;

	EstYaw_gl = 0;
	EstYawBias = 0;
		cosyaw = cos(Pi * EstYaw_gl / 180);
		sinyaw = sin(Pi * EstYaw_gl / 180);
		Rgl2lo = (cv::Mat_<double>(3, 3) <<
					   cosyaw, sinyaw, 0,
					  -sinyaw, cosyaw, 0,
					   0, 0, 1);
		Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
				   cosyaw, sinyaw, 0, 0,
				  -sinyaw, cosyaw, 0, 0,
				   0, 0, 1, 0,
				   0, 0, 0, 1);

		H_cam2uav = (cv::Mat_<double>(4, 4) <<
							 0,-1, 0, 0.125,
							-1, 0, 0, 0,
							 0, 0,-1, 0,
							 0, 0, 0, 1);

		// Hgl2lo transforms a point in the global frame into the tb frame
		Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
		Hlo2gl = invertH(Hgl2lo);

		EstPositionHom_lo = Hgl2lo * EstPositionHom_g;
		EstPosition_lo = (cv::Mat_<double>(3, 1) << EstPositionHom_lo.at<double>(0,0), EstPositionHom_lo.at<double>(1,0), EstPositionHom_lo.at<double>(2,0));

	// Navdata variables
	flightState = 0;
	liftoffSwitch = false;
	positionFromTags = false;

	/*----- Altitude Conatiners*/
	echoAlt 	= 0;
	echoAltLast = 0;
	deltaAlt 	= 0;

	compassYaw = 0;
	compassYawLast = 0;
	compassYawDelta = 0;
	compassCounter = 0;
	yaw_drift = 0;
	yaw_drift_rate = 0;
	// yaw_drift_rate = 0.3;

	/*-----  Initialize Time Variables */
	navStamp = ros::Time::now();
	navTS = navStamp.toSec();
	navTSpublished 	= navStamp.toSec();
	navHeadSeq = 0;
	navDataCounter = 0;
	estCounter = 0;
	cmdcount = 0;

	init_time = ros::Time::now();

}

void hastUAV::openmFile()
{
	ros::Duration(0.1).sleep();
	mFile = std::fopen (s_filename.c_str(), "w");

	// ROS_INFO("ckfRecorder::uavClass: %s", s_filename.c_str());
	ros::Duration(0.2).sleep();
	init_time = ros::Time::now();
	fprintf (mFile, "%s.init_time = %f;\n", s_navdata_file.c_str(), init_time.toSec());

	// fprintf (mFile, "%s.slamRead.time(%d,:) = % -16.8f;\n", s_navdata_file.c_str(), msg_seq, msg_time);

	fprintf (mFile, "%s.ckfinit.yaw_drift_rate = % -6.14f;\n", s_navdata_file.c_str(), yaw_drift_rate);
	// fprintf (mFile, "%s.ckfinit.Mech = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", s_navdata_file.c_str(), ckf.Mech.at<double>(0,0), ckf.Mech.at<double>(1,0), ckf.Mech.at<double>(2,0), ckf.Mech.at<double>(3,0), ckf.Mech.at<double>(4,0));
	// fprintf (mFile, "%s.ckfinit.Aiding = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", s_navdata_file.c_str(), ckf.Aiding.at<double>(0,0), ckf.Aiding.at<double>(1,0), ckf.Aiding.at<double>(2,0), ckf.Aiding.at<double>(3,0), ckf.Aiding.at<double>(4,0));

	// fprintf (mFile, "%s.ckfinit.Qdk(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qdk.at<double>(0, 0), ckf.Qdk.at<double>(0, 1), ckf.Qdk.at<double>(0, 2), ckf.Qdk.at<double>(0, 3), ckf.Qdk.at<double>(0, 4));
	// fprintf (mFile, "%s.ckfinit.Qdk(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qdk.at<double>(1, 0), ckf.Qdk.at<double>(1, 1), ckf.Qdk.at<double>(1, 2), ckf.Qdk.at<double>(1, 3), ckf.Qdk.at<double>(1, 4));
	// fprintf (mFile, "%s.ckfinit.Qdk(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qdk.at<double>(2, 0), ckf.Qdk.at<double>(2, 1), ckf.Qdk.at<double>(2, 2), ckf.Qdk.at<double>(2, 3), ckf.Qdk.at<double>(2, 4));
	// fprintf (mFile, "%s.ckfinit.Qdk(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qdk.at<double>(3, 0), ckf.Qdk.at<double>(3, 1), ckf.Qdk.at<double>(3, 2), ckf.Qdk.at<double>(3, 3), ckf.Qdk.at<double>(3, 4));
	// fprintf (mFile, "%s.ckfinit.Qdk(5,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qdk.at<double>(4, 0), ckf.Qdk.at<double>(4, 1), ckf.Qdk.at<double>(4, 2), ckf.Qdk.at<double>(4, 3), ckf.Qdk.at<double>(4, 4));
	//
	// fprintf (mFile, "%s.ckfinit.Qw(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qw.at<double>(0, 0), ckf.Qw.at<double>(0, 1), ckf.Qw.at<double>(0, 2), ckf.Qw.at<double>(0, 3), ckf.Qw.at<double>(0, 4));
	// fprintf (mFile, "%s.ckfinit.Qw(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qw.at<double>(1, 0), ckf.Qw.at<double>(1, 1), ckf.Qw.at<double>(1, 2), ckf.Qw.at<double>(1, 3), ckf.Qw.at<double>(1, 4));
	// fprintf (mFile, "%s.ckfinit.Qw(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qw.at<double>(2, 0), ckf.Qw.at<double>(2, 1), ckf.Qw.at<double>(2, 2), ckf.Qw.at<double>(2, 3), ckf.Qw.at<double>(2, 4));
	// fprintf (mFile, "%s.ckfinit.Qw(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qw.at<double>(3, 0), ckf.Qw.at<double>(3, 1), ckf.Qw.at<double>(3, 2), ckf.Qw.at<double>(3, 3), ckf.Qw.at<double>(3, 4));
	// fprintf (mFile, "%s.ckfinit.Qw(5,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), ckf.Qw.at<double>(4, 0), ckf.Qw.at<double>(4, 1), ckf.Qw.at<double>(4, 2), ckf.Qw.at<double>(4, 3), ckf.Qw.at<double>(4, 4));
	//
	// fprintf (mFile, "%s.ckfinit.Rk(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.Rk.at<double>(0, 0),ckf.Rk.at<double>(0, 1),ckf.Rk.at<double>(0, 2),ckf.Rk.at<double>(0, 3));
	// fprintf (mFile, "%s.ckfinit.Rk(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.Rk.at<double>(1, 0),ckf.Rk.at<double>(1, 1),ckf.Rk.at<double>(1, 2),ckf.Rk.at<double>(1, 3));
	// fprintf (mFile, "%s.ckfinit.Rk(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.Rk.at<double>(2, 0),ckf.Rk.at<double>(2, 1),ckf.Rk.at<double>(2, 2),ckf.Rk.at<double>(2, 3));
	// fprintf (mFile, "%s.ckfinit.Rk(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.Rk.at<double>(3, 0),ckf.Rk.at<double>(3, 1),ckf.Rk.at<double>(3, 2),ckf.Rk.at<double>(3, 3));
	//
	// fprintf (mFile, "%s.ckfinit.H(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.H.at<double>(0, 0),ckf.H.at<double>(0, 1),ckf.H.at<double>(0, 2),ckf.H.at<double>(0, 3),ckf.H.at<double>(0, 4));
	// fprintf (mFile, "%s.ckfinit.H(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.H.at<double>(1, 0),ckf.H.at<double>(1, 1),ckf.H.at<double>(1, 2),ckf.H.at<double>(1, 3),ckf.H.at<double>(1, 4));
	// fprintf (mFile, "%s.ckfinit.H(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), ckf.H.at<double>(2, 0),ckf.H.at<double>(2, 1),ckf.H.at<double>(2, 2),ckf.H.at<double>(2, 3),ckf.H.at<double>(2, 4));
	// fprintf (mFile, "%s.ckfinit.H(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n",s_navdata_file.c_str(), ckf.H.at<double>(3, 0),ckf.H.at<double>(3, 1),ckf.H.at<double>(3, 2),ckf.H.at<double>(3, 3),ckf.H.at<double>(3, 4));

	fprintf (mFile, "%s.H_cam2uav(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), H_cam2uav.at<double>(0, 0),H_cam2uav.at<double>(0, 1),H_cam2uav.at<double>(0, 2),H_cam2uav.at<double>(0, 3));
	fprintf (mFile, "%s.H_cam2uav(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), H_cam2uav.at<double>(1, 0),H_cam2uav.at<double>(1, 1),H_cam2uav.at<double>(1, 2),H_cam2uav.at<double>(1, 3));
	fprintf (mFile, "%s.H_cam2uav(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",s_navdata_file.c_str(), H_cam2uav.at<double>(2, 0),H_cam2uav.at<double>(2, 1),H_cam2uav.at<double>(2, 2),H_cam2uav.at<double>(2, 3));
	fprintf (mFile, "%s.H_cam2uav(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n",s_navdata_file.c_str(), H_cam2uav.at<double>(3, 0),H_cam2uav.at<double>(3, 1),H_cam2uav.at<double>(3, 2),H_cam2uav.at<double>(3, 3));

	ros::Duration(0.1).sleep();
}

void hastUAV::readCmdVel(const geometry_msgs::Twist::ConstPtr& vsub)
{//ROS_INFO("uav void readCmdVel(const geometry_msgs::Twist::ConstPtr& vsub)");
	velStamp = ros::Time::now().toSec();
	cmdTwist = (cv::Mat_<double>(4, 1) <<
								vsub -> linear.x,
								vsub -> linear.y,
								vsub -> linear.z,
								vsub -> angular.z);
	cmdcount++;
	// fprintf (mFile,"%s.cmd.time(%d,1) = % -6.14f;\n", cmdcount, velStamp);
	// fprintf (mFile,"%s.cmd.velocity(%d,:) = [% -6.14f,% -6.14f,% -6.14f,% -6.14f];\n\n", cmdcount,
	// 	cmdTwist.at<double>(0, 0), cmdTwist.at<double>(0, 1), cmdTwist.at<double>(0, 2), cmdTwist.at<double>(0, 3));
}

void hastUAV::inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata)
{//ROS_INFO("uav void inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata)");
	++navDataCounter;
	// ROS_WARN("ckfRecorder: inertialUpdate");
	flightState = navdata->state;
	/*--------- KF - Constant Tilt Model------------- */
	navHeadSeq = navdata->header.seq;
	navStamp = navdata->header.stamp;
	navTSpublished = navStamp.toSec();

	if (navTSpublished != navTS)
	{	/*----- Inertial Update */
		// findCompassDrift(); // before liftoff, calculate the yaw drift rate

		/*----- Update Clock */
		navdt = navTSpublished - navTS;
		if (navdt>1000)// suppress huge velocity estimates if motion starts from zero
			{navdt=0.006;} // 0.006 is the mean dt of an early experiment
			ckf.Qdk = ckf.Fk * ckf.Qdk * ckf.Fk.t() + ckf.Qw*navdt;

		if (navDataCounter<2){ckf.Qdk = (cv::Mat_<double>(4, 4) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);}

		navTS = navTSpublished;
		/* ---- Read in message values ---- */
		/*----- Yaw values */
		if (compassYaw==0 && compassYawLast == 0) {compassYaw = navdata->rotZ;} // set this to prevent large intial first update

		yaw_drift += navdt*yaw_drift_rate;
		compassYawLast = compassYaw;
		compassYaw = navdata->rotZ;

		deltaPhi = compassYaw - compassYawLast;
		if (deltaPhi > 350) {compassYaw -= 360;} // 350?
		if (deltaPhi < -350) {compassYaw += 360;} // 350?

		if (if_useBias)
		{EstYaw_gl = compassYaw + EstYawBias  - yaw_drift;}  // use yaw_drift only when UAV is suspended
		else
		{EstYaw_gl += deltaPhi;	}

		cosyaw = cos(Pi * EstYaw_gl / 180);
		sinyaw = sin(Pi * EstYaw_gl / 180);

		Rgl2lo = (cv::Mat_<double>(3, 3) <<
					   cosyaw, sinyaw, 0,
					   -sinyaw, cosyaw, 0,
					   0, 0, 1);

		Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
						   cosyaw, sinyaw, 0, 0,
						  -sinyaw, cosyaw, 0, 0,
						   0, 0, 1, 0,
						   0, 0, 0, 1);

		// Altitude update from sonar
		echoAltLast = echoAlt;
		echoAlt = 0.001 * double(navdata->altd);
		deltaAlt = echoAlt - echoAltLast;

		// Velocity update
		MeasuredVel_lo = (cv::Mat_<double>(3, 1) <<
							0.001 * (navdata->vx),
							0.001 * (navdata->vy),
							deltaAlt/0.05); // should this be deltaAlt/navdt?

		MeasuredVel_gl = Rgl2lo.t() * MeasuredVel_lo;
		EstPosition_gl += MeasuredVel_gl * navdt;

		MeasuredAcc_lo = (cv::Mat_<double>(3, 1) <<
							(navdata->ax),
							(navdata->ay),
							(navdata->az));
		MeasuredAcc_gl= Rgl2lo.t() * MeasuredAcc_lo;

		Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
		Hlo2gl = invertH(Hgl2lo);

		posePublisher(navTS);
		tfPublisher(navStamp);

		/*----- Write Data */
		if (data.init_active)
		{
			fprintf(data.filename,"\n%% --------------- navdata  --------------- %% \n");
			fprintf(data.filename,"  %s.navdata.time(%d,:)    = % -6.8f;\n",			s_data_filename.c_str(), navDataCounter, navTS-init_time.toSec());
			fprintf(data.filename,"    %s.navdata.navdt(%d,:) = % -6.8f;\n", 			s_data_filename.c_str(), navDataCounter, navdt);
			fprintf(data.filename,"    %s.navdata.Alt(%d,:)   = % -6.8f;\n",			s_data_filename.c_str(), navDataCounter, echoAlt);
			fprintf(data.filename,"    %s.navdata.HeadSeq(%d,:)  = %d;\n", 				s_data_filename.c_str(), navDataCounter, navHeadSeq);
			fprintf(data.filename,"    %s.navdata.Battery(%d,:)  = % -6.8f;\n",		s_data_filename.c_str(), navDataCounter, navdata->batteryPercent);
			fprintf(data.filename,"    %s.navdata.DeltaAlt(%d,:) = % -6.8f;\n", 	s_data_filename.c_str(), navDataCounter, deltaAlt);
			fprintf(data.filename,"    %s.navdata.deltaPhi(%d,1) = % -6.8f;\n", 	s_data_filename.c_str(), navDataCounter, deltaPhi);
			fprintf(data.filename,"    %s.navdata.yaw_drift(%d,:) = % -6.8f;\n",	s_data_filename.c_str(), navDataCounter, yaw_drift);
			fprintf(data.filename,"    %s.navdata.CompassYaw(%d,:)= % -6.8f; %% CompassYaw_deg = % -6.8f\n", s_data_filename.c_str(), navDataCounter, compassYaw*(Pi/180), compassYaw);
			fprintf(data.filename,"    %s.navdata.EstYaw_gl(%d,:) = % -6.8f; %% EstYaw_gl_deg  = % -6.8f\n", s_data_filename.c_str(), navDataCounter, EstYaw_gl*(Pi/180), EstYaw_gl);
			fprintf(data.filename,"    %s.navdata.YawBias(%d,1)   = % -6.8f; %% YawBias_deg    = % -6.8f\n", s_data_filename.c_str(), navDataCounter, EstYawBias*(Pi/180), EstYawBias);
			fprintf(data.filename,"    %s.navdata.V(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n",	s_data_filename.c_str(), navDataCounter, navdata->vx, navdata->vy, navdata->vz);
			fprintf(data.filename,"    %s.navdata.RPY(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n",	s_data_filename.c_str(), navDataCounter, navdata->rotY, navdata->rotX, navdata->rotZ);
			fprintf(data.filename,"    %s.navdata.Position_gl(%u,:)    = [% -6.8f % -6.8f % -6.8f];\n", s_data_filename.c_str(), navDataCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
			fprintf(data.filename,"    %s.navdata.MeasuredVel_gl(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_data_filename.c_str(), navDataCounter, MeasuredVel_gl.at<double>(0,0), MeasuredVel_gl.at<double>(1,0), MeasuredVel_gl.at<double>(2,0));
			fprintf(data.filename,"    %s.navdata.MeasuredVel_lo(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_data_filename.c_str(), navDataCounter, MeasuredVel_lo.at<double>(0,0), MeasuredVel_lo.at<double>(1,0), MeasuredVel_lo.at<double>(2,0));
			fprintf(data.filename,"    %s.navdata.ckf.Qdk(%d,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_data_filename.c_str(), navDataCounter, ckf.Qdk.at<double>(0,0), ckf.Qdk.at<double>(1,1), ckf.Qdk.at<double>(2,2), ckf.Qdk.at<double>(3,3));
		}
		// else {
		// 	// fprintf (mFile, "%s.est.time(%d,1) = % -6.14f;\n", ++estCounter, navTS);
		// 	// fprintf (mFile, "%s.est.Yaw(%d,1) = % -6.14f;\n", estCounter, EstYaw_gl);
		// 	// fprintf (mFile, "%s.est.YawBias(%d,1) = % -6.14f;\n", estCounter, EstYawBias);
		//
		//
		// 	fprintf (mFile, "%s.navdata.time(%d,:) = % -6.14f;\n", 											s_navdata_file.c_str(), navDataCounter, navTS-init_time.toSec());
		// 	fprintf (mFile, "  %s.navdata.navdt(%d,:) = % -6.14f;\n", 										s_navdata_file.c_str(), navDataCounter, navdt);
		// 	fprintf (mFile, "  %s.navdata.HeadSeq(%d,:) = %d;\n", 												s_navdata_file.c_str(), navDataCounter, navHeadSeq);
		// 	fprintf (mFile, "  %s.navdata.Alt(%d,:) = % -6.14f;\n", 											s_navdata_file.c_str(), navDataCounter, echoAlt);
		// 	fprintf (mFile, "  %s.navdata.Battery(%d,:) = % -6.14f;\n", 									s_navdata_file.c_str(), navDataCounter, navdata->batteryPercent);
		// 	fprintf (mFile, "  %s.navdata.DeltaAlt(%d,:) = % -6.14f;\n", 									s_navdata_file.c_str(), navDataCounter, deltaAlt);
		// 	fprintf (mFile, "  %s.navdata.YawBias(%d,1)   = % -6.14f;\n", 								s_navdata_file.c_str(), navDataCounter, EstYawBias);
		// 	fprintf (mFile, "  %s.navdata.deltaPhi(%d,1)  = % -6.14f;\n", 								s_navdata_file.c_str(), navDataCounter, deltaPhi);
		// 	fprintf (mFile, "  %s.navdata.yaw_drift(%d,:) = % -6.14f;\n", 								s_navdata_file.c_str(), navDataCounter, yaw_drift);
		// 	fprintf (mFile, "  %s.navdata.CompassYaw(%d,:)= % -6.14f;\n", 								s_navdata_file.c_str(), navDataCounter, compassYaw);
		// 	fprintf (mFile, "  %s.navdata.V(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", 	s_navdata_file.c_str(), navDataCounter, navdata->vx, navdata->vy, navdata->vz);
		// 	fprintf (mFile, "  %s.navdata.RPY(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), navDataCounter, navdata->rotY, navdata->rotX, navdata->rotZ);
		// 	fprintf (mFile, "  %s.navdata.MeasuredVel_gl(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), navDataCounter, MeasuredVel_gl.at<double>(0,0), MeasuredVel_gl.at<double>(1,0), MeasuredVel_gl.at<double>(2,0));
		// 	fprintf (mFile, "  %s.navdata.MeasuredVel_lo(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), navDataCounter, MeasuredVel_lo.at<double>(0,0), MeasuredVel_lo.at<double>(1,0), MeasuredVel_lo.at<double>(2,0));
		// 	fprintf (mFile, "  %s.navdata.ckf.Qdk(%d,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", s_navdata_file.c_str(), navDataCounter, ckf.Qdk.at<double>(0,0), ckf.Qdk.at<double>(1,1), ckf.Qdk.at<double>(2,2), ckf.Qdk.at<double>(3,3), ckf.Qdk.at<double>(4,4));
		// 	fprintf (mFile, "  %s.navdata.EstYaw_gl(%d,:) = % -6.14f;\n", 								s_navdata_file.c_str(), navDataCounter, EstYaw_gl);
		// 	fprintf (mFile, "  %s.navdata.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", navDataCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		// }

	}
}

void hastUAV::updateState_sub(const hast::uavstate::ConstPtr& state_msg)
{ // this topic is used to share the estimate form ugv1 to ugv2

	EstYaw_gl = state_msg -> yaw;; //stay in radians where possible
	EstPosition_gl = (cv::Mat_<double>(3, 1) << state_msg -> P.x, state_msg -> P.y, state_msg -> P.z);

		cosyaw = cos( EstYaw_gl );
		sinyaw = sin( EstYaw_gl );
		Rgl2lo = (cv::Mat_<double>(3, 3) <<
						 cosyaw, sinyaw, 0,
						-sinyaw, cosyaw, 0,
						 0, 0, 1);

		// Hgl2lo transforms a point in the global frame into the ugv frame
		Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
		Hlo2gl = invertH(Hgl2lo);

}

void hastUAV::updatePoseFromCKF(cv::Mat ckf_correction_gl, double ckf_yaw_correction, double ckf_yawBias_correction)
{
	// remove the prevous correction
	EstYaw_gl      += yawCorrection;
	EstPosition_gl += correction_gl;

	// save the new correction
	yawCorrection = ckf_yaw_correction * (180/Pi);
	correction_gl = (cv::Mat_<double>(3, 1) <<
					ckf_correction_gl.at<double>(0,0),
					ckf_correction_gl.at<double>(1,0),
					ckf_correction_gl.at<double>(2,0));

	// apply correction to estimated pose
	EstYaw_gl      -= yawCorrection;
	EstPosition_gl -= correction_gl;
	EstYawBias     -= ckf_yawBias_correction  * (180/Pi);

	cosyaw = cos( EstYaw_gl * Pi/180);
	sinyaw = sin( EstYaw_gl * Pi/180);
	Rgl2lo = (cv::Mat_<double>(3, 3) <<
					 cosyaw, sinyaw, 0,
					-sinyaw, cosyaw, 0,
					 0, 0, 1);

	// Hgl2lo transforms a point in the global frame into the ugv frame
	Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
	Hlo2gl = invertH(Hgl2lo);

	ckf.Qdk = (cv::Mat_<double>(4, 4) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0); // reset after incrementing

	fprintf(data.filename,"     %% %s.updateFromCKF.ckf_correction_gl  = [%-12.10f, %-12.10f, %-12.10f];\n",	s_data_filename.c_str(), ckf_correction_gl.at<double>(0,0), ckf_correction_gl.at<double>(1,0), ckf_correction_gl.at<double>(2,0));
	fprintf(data.filename,"     %% %s.updateFromCKF.correction_gl      = [%-12.10f, %-12.10f, %-12.10f];\n",	s_data_filename.c_str(), correction_gl.at<double>(0,0), correction_gl.at<double>(1,0), correction_gl.at<double>(2,0));
	fprintf(data.filename,"     %% %s.updateFromCKF.EstPosition_gl     = [%-12.10f, %-12.10f, %-12.10f];\n",	s_data_filename.c_str(), EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
	fprintf(data.filename,"     %% %s.updateFromCKF.ckf_yaw_correction(deg) =  %-12.10f;  (%-12.10f)  \n",		s_data_filename.c_str(), ckf_yaw_correction, yawCorrection);
	fprintf(data.filename,"     %% %s.updateFromCKF.EstYaw_gl(deg)          =  %-12.10f;  (%-12.10f)  \n",		s_data_filename.c_str(), EstYaw_gl*Pi/180, EstYaw_gl);

}

void hastUAV::tfPublisher(ros::Time pubstamp)
{//ROS_INFO("uav void tfPublisher(ros::Time pubstamp)");
	// ROS_INFO("uavRecorder: -------Pose_msg --------------------------");
	// ROS_INFO("uavRecorder: EstPosition_gl  = [%6.4f %6.4f %6.4f]  ", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
	// ROS_INFO("uavRecorder: [00 01 02] = [%6.4f %6.4f %6.4f]  ",  cosyaw, sinyaw, 0.0);
	// ROS_INFO("uavRecorder: [10 11 12] = [%6.4f %6.4f %6.4f]  ", -sinyaw, cosyaw, 0.0);
	// ROS_INFO("uavRecorder: [20 21 22] = [%6.4f %6.4f %6.4f]  ",  0.0,0.0,1.0);

	TF_gl.setOrigin( tf::Vector3(EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0)) );
	R_TF_gl.setValue(cosyaw, -sinyaw, 0,
					 sinyaw, cosyaw, 0,
					  0,0,1);
	R_TF_gl.getRotation(Q_TF_gl);
	TF_gl.setRotation(Q_TF_gl);
	TF_gl_pub.sendTransform(tf::StampedTransform(TF_gl, pubstamp, s_TF_gl_parent, s_TF_gl_child));
	poseHeaderPublisher(pubstamp);
	// ROS_WARN("oneCKF::tfPublisher");
	// ROS_INFO("  parent: %s, ", s_TF_gl_parent.c_str());
	// ROS_INFO("  child:  %s ", s_TF_gl_child.c_str());
	// ROS_INFO("  TF_gl.setOrigin : [%6.4f %6.4f %6.4f]", EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0));
}

void hastUAV::poseHeaderPublisher(ros::Time pubstamp)
{//ROS_INFO("%s::hastUAV::poseHeaderPublisher START", s_parent_node.c_str());
	// publish estimated pose for other nodes to use

	hast::posewithheader pose_msg;
	pose_msg.position.x = EstPosition_gl.at<double>(0, 0);
	pose_msg.position.y = EstPosition_gl.at<double>(1, 0);
	pose_msg.position.z = EstPosition_gl.at<double>(2, 0);
	pose_msg.orientation = tf::createQuaternionMsgFromYaw(Pi * EstYaw_gl / 180);

	/*----- Time stamp of Estimate */
	pose_msg.header.seq = ++pose_msg_seq;
	pose_msg.header.stamp = pubstamp;
	pose_msg.header.frame_id = "/map";

	pose_pub.publish(pose_msg);
}

void hastUAV::posePublisher(double pubtime)
{
	// ROS_INFO("hastUAV::posePublisher(double pubtime)");
	// publish estimated pose for other nodes to use
	/*----- Position Estimate */
	state_msg.P.x = EstPosition_gl.at<double>(0, 0);
	state_msg.P.y = EstPosition_gl.at<double>(1, 0);
	state_msg.P.z = EstPosition_gl.at<double>(2, 0);

	/*----- Velocity Estimate */
	state_msg.V.x = MeasuredVel_gl.at<double>(0, 0);
	state_msg.V.y = MeasuredVel_gl.at<double>(1, 0);
	state_msg.V.z = MeasuredVel_gl.at<double>(2, 0);

	/*----- Acceleration Measured */
	state_msg.A.x = MeasuredAcc_gl.at<double>(0, 0);
	state_msg.A.y = MeasuredAcc_gl.at<double>(1, 0);
	state_msg.A.z = MeasuredAcc_gl.at<double>(2, 0);

	/*----- Pose Estimate */
	state_msg.R.row0.x = Rgl2lo.at<double>(0,0);
	state_msg.R.row0.y = Rgl2lo.at<double>(0,1);
	state_msg.R.row0.z = 0;
	state_msg.R.row1.x = Rgl2lo.at<double>(1,0);
	state_msg.R.row1.y = Rgl2lo.at<double>(1,1);
	state_msg.R.row1.z = 0;
	state_msg.R.row2.x = 0;
	state_msg.R.row2.y = 0;
	state_msg.R.row2.z = 1;

	/*----- Yaw Estimate */
	state_msg.yaw = EstYaw_gl;

	/*----- Time stamp of Estimate */
	state_msg.id = ++stateMsg_id;
	state_msg.stamp = pubtime;
	state_pub.publish(state_msg);

	// fprintf (mFile, "%s.est.time(%d,1) = % -6.14f;\n", ++estCounter, pubtime);
	// fprintf (mFile, "%s.est.Yaw(%d,1) = % -6.14f;\n", estCounter, EstYaw_gl);
	// fprintf (mFile, "%s.est.YawBias(%d,1) = % -6.14f;\n", estCounter, EstYawBias);
	// fprintf (mFile, "%s.est.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n\n", estCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
}

void hastUAV::slamRead(const hast::posewithheader::ConstPtr& pose_msg)
{ // this might not be doing anything anymore
	if(slamflag)
	{// wait for slam to be turned on
		tf::Quaternion q_from_msg(pose_msg -> orientation.x, pose_msg -> orientation.y, pose_msg -> orientation.z, pose_msg -> orientation.w);
		tf::Vector3 p_from_msg(pose_msg -> position.x, pose_msg -> position.y, pose_msg -> position.z);
		tf::Transform Xform_from_msg(q_from_msg, p_from_msg);
		double qroll, qpitch, qyaw;
		tf::Matrix3x3(Xform_from_msg.getRotation()).getRPY(qroll, qpitch, qyaw);
		EstYaw_gl = qyaw; //stay in radians where possible
		EstPosition_gl = (cv::Mat_<double>(3, 1) << pose_msg -> position.x, pose_msg -> position.y, pose_msg -> position.z);

		int msg_seq = pose_msg->header.seq;
		ros::Time msg_stamp = pose_msg->header.stamp;
		double msg_time = msg_stamp.toSec();

		if (data.init_active)
		{
			fprintf(data.filename,"  %s.slamRead.time(%d,:) = % -6.8f;\n", 			s_data_filename.c_str(), msg_seq, msg_time);
			fprintf(data.filename,"  %s.slamRead.EstYaw_gl(%d,:) = % -6.8f;\n", s_data_filename.c_str(), msg_seq, EstYaw_gl);
			fprintf(data.filename,"  %s.slamRead.EstPosition_gl(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_data_filename.c_str(), msg_seq, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		// } else {
			// fprintf (mFile, "%s.slamRead.time(%d,:) = % -16.8f;\n", s_navdata_file.c_str(), msg_seq, msg_time);
			// fprintf (mFile, "%s.slamRead.EstYaw_gl(%d,:) = % -6.14f;\n", s_navdata_file.c_str(), msg_seq, EstYaw_gl);
			// fprintf (mFile, "%s.slamRead.EstPosition_gl(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_navdata_file.c_str(), msg_seq, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		}

	}
}
