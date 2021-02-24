#include "ckfClass.hpp"

ckfClass::ckfClass()
{
	lastCKFtime = 0;
	counter = 0;
} // void for construction of KF class

void ckfClass::incrementKF()
{
	++counter;

	// % Prediction
	PriorEst = Fk * PosteriorEst;
	PriorCov = Fk * PosteriorCov * Fk.t() + Qdk;

// % Innovation
	zk = H * (Aiding - Mech);
	yk = zk - H * PriorEst;
	Sk = H * PriorCov * H.t() + Rk;
	Skinv = Sk.inv();
	Kk = PriorCov * H.t() * Skinv;

// % Estimate
	Kkyk = Kk * yk;
	PosteriorEst = PriorEst + Kk * yk;
	IKkH = (I - Kk * H);
	PosteriorCov = IKkH * PriorCov * IKkH.t() + Kk * Rk * Kk.t();

}

void ckfClass::zKF()
{
	// requires setting zk directly rather than zk = H * (Aiding - Mech);
	++counter;

	// % Prediction
	PriorEst = Fk * PosteriorEst;
	PriorCov = Fk * PosteriorCov * Fk.t() + Qdk;

// % Innovation
	// zk = H * (Aiding - Mech);
	yk = zk - H * PriorEst;
	Sk = H * PriorCov * H.t() + Rk;
	Skinv = Sk.inv();
	Kk = PriorCov * H.t() * Skinv;

// % Estimate
	Kkyk = Kk * yk;
	PosteriorEst = PriorEst + Kk * yk;
	IKkH = (I - Kk * H);
	PosteriorCov = IKkH * PriorCov * IKkH.t() + Kk * Rk * Kk.t();
}

// void ckfClass::LKF()
// {
// 	// requires setting zk directly rather than zk = H * (Aiding - Mech);
// 	++counter;
//
// 	// % Prediction
// 	PriorEst = Fk * PosteriorEst;
// 	PriorCov = Fk * PosteriorCov * Fk.t() + Qdk;
//
// // % Innovation
// 	// zk = H * (Aiding - Mech);
// 	yk = zk - H * PriorEst;
// 	Sk = H * PriorCov * H.t() + Rk;
// 	Skinv = Sk.inv();
// 	Kk = PriorCov * H.t() * Skinv;
//
// // % Estimate
// 	Kkyk = Kk * yk;
// 	PosteriorEst = PriorEst + Kk * yk;
// 	IKkH = (I - Kk * H);
// 	PosteriorCov = IKkH * PriorCov * IKkH.t() + Kk * Rk * Kk.t();
// }
