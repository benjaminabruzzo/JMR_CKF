#include "utils.hpp"

	cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
	{//ROS_INFO("uav cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)");
	// H = [R -R*t(1:3,1); 0 0 0 1];
		cv::Mat H, nRt, t;
		t = (cv::Mat_<double>(3, 1) << t_in.at<double>(0,0),t_in.at<double>(1,0),t_in.at<double>(2,0));
		nRt = -R_in * t;
		H = (cv::Mat_<double>(4, 4) <<
			R_in.at<double>(0,0), R_in.at<double>(0,1), R_in.at<double>(0,2), nRt.at<double>(0,0),
			R_in.at<double>(1,0), R_in.at<double>(1,1), R_in.at<double>(1,2), nRt.at<double>(1,0),
			R_in.at<double>(2,0), R_in.at<double>(2,1), R_in.at<double>(2,2), nRt.at<double>(2,0),
			0,0,0,1);
		return H;
	}

	cv::Mat invertH(cv::Mat H)
	{// Hinv = [R' -R'*t; 0 0 0 1];
		cv::Mat Hinv, RT, R, t, nRTt;
		t = (cv::Mat_<double>(3, 1) << H.at<double>(0,3),H.at<double>(1,3),H.at<double>(2,3));
		R = (cv::Mat_<double>(3, 3) << 	H.at<double>(0,0), H.at<double>(0,1), H.at<double>(0,2),
																		H.at<double>(1,0), H.at<double>(1,1), H.at<double>(1,2),
																		H.at<double>(2,0), H.at<double>(2,1), H.at<double>(2,2));
		RT = R.t();
		nRTt = -RT*t;
		Hinv = (cv::Mat_<double>(4, 4) <<
			RT.at<double>(0,0), RT.at<double>(0,1), RT.at<double>(0,2), nRTt.at<double>(0,0),
			RT.at<double>(1,0), RT.at<double>(1,1), RT.at<double>(1,2), nRTt.at<double>(1,0),
			RT.at<double>(2,0), RT.at<double>(2,1), RT.at<double>(2,2), nRTt.at<double>(2,0),
			0,0,0,1);
		return Hinv;
	}

	double wrapDegrees(double angle)
	{ // Force angle to stay between +- 180 degrees
		double toplimit = 180;
		double botlimit = -180;
		while(angle > toplimit){angle -= (toplimit - botlimit);}
		while(angle < botlimit){angle += (toplimit - botlimit);}
		return angle;
	}

	double wrapRadians(double angle)
	{ // Force angle to stay between +- 180 degrees
		double toplimit = 1.57079632679;
		double botlimit = -1.57079632679;
		while(angle > toplimit){angle -= (toplimit - botlimit);}
		while(angle < botlimit){angle += (toplimit - botlimit);}
		return angle;
	}


	double toDegrees(double radians){return radians*57.29577951;} //180/Pi = 57.295779513082323;
	double toRadians(double degrees){return degrees*00.01745329;} //Pi/180 = 00.017453292519943;


	cv::Matx<double,4,1> Cross(cv::Matx<double,4,1> A, cv::Matx<double,4,1> B)
	{//computes A x B
		cv::Matx<double,4,1> C;
		C(0,0) =   ((A(1,0) * B(2,0)) - (A(2,0) * B(1,0)));
		C(1,0) = - ((A(0,0) * B(2,0)) - (A(2,0) * B(0,0)));
		C(2,0) =   ((A(0,0) * B(1,0)) - (A(1,0) * B(0,0)));
		C(3,0) = 0;
		return C;
	}


	cv::Matx<double,3,3> wedgevector(cv::Matx<double,4,1> V)
	{
		cv::Matx33d Vx;
		Vx = cv::Matx33d(
						0,  V(2,0), -V(1,0),// row 1
			-V(2,0),       0,  V(0,0), // row 2
			 V(1,0), -V(0,0),      0); // row 3
			return Vx;
	}

	cv::Matx<double,3,3> makeRz(double theta) //in radians
	{
		cv::Matx33d R;
		R = cv::Matx33d(
			 cos(theta), sin(theta), 0,// row 1
			-sin(theta), cos(theta), 0, // row 2
								0, 					0, 1); // row 3
			return R;
	}

	cv::Mat matRz(double theta) //in radians
	{
		cv::Mat R;
		R = (cv::Mat_<double>(3, 3) <<
			 cos(theta), sin(theta), 0,// row 1
			-sin(theta), cos(theta), 0, // row 2
								0, 					0, 1); // row 3
			return R;
	}


	cv::Matx<double,3,3> makeRx(double theta) //in radians
	{
		cv::Matx33d R;
		R = cv::Matx33d(
			 1, 				 0, 				 0, // row 1
			 0, cos(theta), sin(theta), // row 2
			 0,-sin(theta), cos(theta));// row 3
			return R;
	}

	cv::Matx<double,3,3> jacobianOfNorm(cv::Matx<double,4,1> p)
	{
		double x, y, z, A;
		x = p(0,0);
		y = p(1,0);
		z = p(2,0);
		A = 1 / pow(x*x + y*y + z*z, 1.5);

		cv::Matx33d B;
		B = cv::Matx33d(
			y*y + z*z,    -x*y,    -x*z,
					 -x*y, x*x+z*z,    -y*z,
					 -x*z,    -y*z, x*x+y*y);
	 return A*B;
	}

	cv::Matx<double,1,2> jacobianOfAtan2(double y, double x)
	{
		double A = 1 / (x*x+y*y);
		cv::Matx12d px;
		px = A*cv::Matx12d(-y,x);
		return px;
	}

	std::string type2str(int type)
	{
		std::string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch ( depth ) {
			case CV_8U:  r = "CV_8U"; break;
			case CV_8S:  r = "CV_8S"; break;
			case CV_16U: r = "CV_16U"; break;
			case CV_16S: r = "CV_16S"; break;
			case CV_32S: r = "CV_32S"; break;
			case CV_32F: r = "CV_32F"; break;
			case CV_64F: r = "CV_64F"; break;
			default:     r = "User"; break;
		}
		r += "C";
		r += (chans+'0');

  return r;
}
