
function J = jacobianOfAtan2(y,x)
% 	cv::Matx<double,1,2> jacobianOfAtan2(double y, double x)
% 	{
% 		double A = 1 / (x*x+y*y);
% 		cv::Matx12d px;
% 		px = A*cv::Matx12d(-y,x);
% 		return px;
% 	}

  A = 1/ (x*x+y*y);
  J = A * [-y, x];

end
