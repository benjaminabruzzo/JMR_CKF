function Vx = wedgevector(V)
% 	cv::Matx<double,3,3> wedgevector(cv::Matx<double,4,1> V)
% 	{
% 		cv::Matx33d Vx;
% 		Vx = cv::Matx33d(
% 						0,  V(2,0), -V(1,0),// row 1
% 			-V(2,0),       0,  V(0,0), // row 2
% 			 V(1,0), -V(0,0),      0); // row 3
% 			return Vx;
% 	}


Vx = [      0,  V(3,1), -V(2,1); ...
      -V(3,1),       0,  V(1,1); ... % row 2
       V(2,1), -V(1,1),      0]; % row 3
end
