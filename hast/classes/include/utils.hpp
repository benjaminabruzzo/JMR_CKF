// "extern" means that this is not a variable declaration:

#ifndef HAST_UTILS_H
#define HAST_UTILS_H

#include "genheaders.hpp"

cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in);
cv::Mat invertH(cv::Mat H);
double wrapDegrees(double angle);
double wrapRadians(double angle);
double toDegrees(double radians);
double toRadians(double degrees);

cv::Matx<double,4,1> Cross(cv::Matx<double,4,1> A, cv::Matx<double,4,1> B);
cv::Matx<double,3,3> wedgevector(cv::Matx<double,4,1> V);
cv::Matx<double,3,3> makeRz(double theta); //in radians
cv::Mat matRz(double theta); //in radians
cv::Matx<double,3,3> makeRx(double theta); //in radians
cv::Matx<double,3,3> jacobianOfNorm(cv::Matx<double,4,1> p);
cv::Matx<double,1,2> jacobianOfAtan2(double y, double x);
std::string type2str(int type);


#endif