#ifndef HAST_DKF_H
#define HAST_DKF_H

#include "genheaders.hpp"

class DiscreteKF
{
	private:
	public:
	//Inputs
	cv::Mat zk, Dk, PosteriorEst;
	cv::Mat I, H, Fk, Qk, Rk, PosteriorCov;

	//Intermediates
	cv::Mat PriorEst, PriorCov;
	cv::Mat Sk, yk, Kk;
	cv::Mat Kkyk;

	//increment index
	double lastDKFtime;
	uint counter;

	// rotation matrices
	cv::Mat Rgl2lo;
	double sinyaw, cosyaw;

	DiscreteKF();

	void incrementKF();
};


#endif
