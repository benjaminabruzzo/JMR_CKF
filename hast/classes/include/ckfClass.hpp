#ifndef HAST_CKF_H
#define HAST_CKF_H

#include "genheaders.hpp"

class ckfClass
{
	private:
	public:
	// time
	double lastCKFtime;
	uint counter;
	//Inputs
	cv::Mat Mech, Aiding, PosteriorEst;
	cv::Mat Fk, PosteriorCov, Qdk, Qw, H, I;
	cv::Mat Rk, Rk_inf;

	//Intermediates
	cv::Mat PriorEst, PriorCov;
	cv::Mat zk, Sk, Skinv, yk, Kk;
	cv::Mat Kkyk, IKkH;

	// Reduced order (L) variables:
	// o: observable states
	// u: unobservable states
	// cv::Mat Ino, Iuo;
	// cv::Mat Poo, Puo, Puu;  // covariance sub-matrices
	// cv::Lk;

	ckfClass();

	void incrementKF();
	void zKF();
	// void LKF();

};

#endif
