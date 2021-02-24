function J = jacobianOfNorm(p)
% 	cv::Matx<double,3,3> jacobianOfNorm(cv::Matx<double,4,1> p)
% 	{
% 		double x, y, z, A;
% 		x = p(0,0);
% 		y = p(1,0);
% 		z = p(2,0);
% 		A = 1 / pow(x*x + y*y + z*z, 1.5);
% 
% 		cv::Matx33d B;
% 		B = cv::Matx33d(
% 			y*y + z*z,    -x*y,    -x*z,
% 					 -x*y, x*x+z*z,    -y*z,
% 					 -x*z,    -y*z, x*x+y*y);
% 	 return A*B;
% 	}

  x = p(1,1);
  y = p(2,1);
  z = p(3,1);
  A = (x*x + y*y + z*z)^(1.5);

  B = [	y*y + z*z,    -x*y,    -x*z; ...
					   -x*y, x*x+z*z,    -y*z; ...
					   -x*z,    -y*z,  x*x+y*y ];
  
  J = A*B;
end

 