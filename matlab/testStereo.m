function ugv_sensor = testStereo()
%% init from experiment
  ugv_sensor.left_marker.pixels_left   = [ 279.589691,  397.293264];
  ugv_sensor.left_marker.pixels_right  = [ 190.688809,  397.293264];
  ugv_sensor.right_marker.pixels_left  = [ 383.032872,  397.212384];
  ugv_sensor.right_marker.pixels_right = [ 296.305139,  397.212384];
  ugv_sensor.rear_marker.pixels_left   = [ 363.835727,  397.418741];
  ugv_sensor.rear_marker.pixels_right  = [ 271.563481,  397.418741];


  ugv_sensor.Baseline =  0.15240000; 
  ugv_sensor.FocalLength =  1082.46358758;
  ugv_sensor.Cx = [ 644.50000000];
  ugv_sensor.Cy = [ 482.50000000];
  ugv_sensor.cam2ugv(1,:) = [ 0.00000000,  0.34202014,  0.93969262,  0.00000000];
  ugv_sensor.cam2ugv(2,:) = [-1.00000000,  0.00000000,  0.00000000,  0.00000000];
  ugv_sensor.cam2ugv(3,:) = [ 0.00000000, -0.93969262,  0.34202014,  0.00000000];
  ugv_sensor.cam2ugv(4,:) = [ 0.00000000,  0.00000000,  0.00000000,  1.00000000];

  Baseline = ugv_sensor.Baseline;
  FocalLength =  ugv_sensor.FocalLength;
  cam2ugv33 = ugv_sensor.cam2ugv(1:3,1:3);
  Cx = ugv_sensor.Cx;
  Cy = ugv_sensor.Cy;

%% calc camera frame positions:
% markers
  ugv_sensor.left_marker.P_cam  = calcvec( ugv_sensor.left_marker, Baseline, FocalLength, Cx);
  ugv_sensor.right_marker.P_cam = calcvec(ugv_sensor.right_marker, Baseline, FocalLength, Cx);
  ugv_sensor.rear_marker.P_cam  = calcvec( ugv_sensor.rear_marker, Baseline, FocalLength, Cx);
% ugv2 centroid
  ugv_sensor.ugv2.P_cam = 0.5 * (ugv_sensor.left_marker.P_cam + ugv_sensor.right_marker.P_cam);

%% calc ugv frame values
% markers
  ugv_sensor.left_marker.P_ugv   = ugv_sensor.cam2ugv * ugv_sensor.left_marker.P_cam ;
  ugv_sensor.right_marker.P_ugv  = ugv_sensor.cam2ugv * ugv_sensor.right_marker.P_cam ;
  ugv_sensor.rear_marker.P_ugv   = ugv_sensor.cam2ugv * ugv_sensor.rear_marker.P_cam ;
% ugv2 centroid
  ugv_sensor.ugv2.P_ugv = 0.5 * (ugv_sensor.left_marker.P_ugv + ugv_sensor.right_marker.P_ugv);

%% compute axes and yaw
  RightmLeft = ugv_sensor.right_marker.P_ugv - ugv_sensor.left_marker.P_ugv;
  RearmLeft  = ugv_sensor.rear_marker.P_ugv  - ugv_sensor.left_marker.P_ugv;
  RightmRear = ugv_sensor.right_marker.P_ugv - ugv_sensor.rear_marker.P_ugv;
  LeftmRight = ugv_sensor.left_marker.P_ugv  - ugv_sensor.right_marker.P_ugv;
  
  ugv_sensor.z_axis = CROSS(RearmLeft, RightmLeft) / norm(CROSS(RearmLeft, RightmLeft));
  ugv_sensor.y_axis = LeftmRight / norm(LeftmRight);
  ugv_sensor.x_axis = CROSS(ugv_sensor.y_axis, ugv_sensor.z_axis);
  ugv_sensor.yaw_ugv = atan2(ugv_sensor.x_axis(2), ugv_sensor.x_axis(1));
  
%% propegate covariances
clc
  ugv_sensor.left_marker.J_cam   = calcJ(  ugv_sensor.left_marker, Baseline, FocalLength);
  ugv_sensor.right_marker.J_cam  = calcJ( ugv_sensor.right_marker, Baseline, FocalLength);
  ugv_sensor.rear_marker.J_cam   = calcJ(  ugv_sensor.rear_marker, Baseline, FocalLength);

  ugv_sensor.left_marker.JJt_cam  =  ugv_sensor.left_marker.J_cam * (1000*eye(3)) * ugv_sensor.left_marker.J_cam';
  ugv_sensor.right_marker.JJt_cam = ugv_sensor.right_marker.J_cam * (1000*eye(3)) * ugv_sensor.right_marker.J_cam';
  ugv_sensor.rear_marker.JJt_cam  =  ugv_sensor.rear_marker.J_cam * (1000*eye(3)) * ugv_sensor.rear_marker.J_cam';

  ugv_sensor.left_marker.JJt_ugv  = cam2ugv33 * ugv_sensor.left_marker.JJt_cam * cam2ugv33';
  ugv_sensor.right_marker.JJt_ugv = cam2ugv33 * ugv_sensor.right_marker.JJt_cam * cam2ugv33';
  ugv_sensor.rear_marker.JJt_ugv  = cam2ugv33 * ugv_sensor.rear_marker.JJt_cam * cam2ugv33';

  RightandLeft_JJt = ugv_sensor.right_marker.JJt_ugv + ugv_sensor.left_marker.JJt_ugv;
  RearandLeft_JJt  = ugv_sensor.rear_marker.JJt_ugv  + ugv_sensor.left_marker.JJt_ugv;

  ugv_sensor.covP_ugv = 0.5 * RightandLeft_JJt * 0.5; % scaled both vectors by 1/2
  covP_ugv = 0.5 * RightandLeft_JJt * 0.5 % scaled both vectors by 1/2

  ugv_sensor.z_JJt = wedgevector(LeftmRight) * RearandLeft_JJt * wedgevector(LeftmRight)' + wedgevector(RearmLeft) * RightandLeft_JJt * wedgevector(RearmLeft)';
  z_JJt = wedgevector(LeftmRight) * RearandLeft_JJt * wedgevector(LeftmRight)' + wedgevector(RearmLeft) * RightandLeft_JJt * wedgevector(RearmLeft)'
%   jacobianOfNorm_z = jacobianOfNorm(CROSS(RearmLeft, RightmLeft))
  ugv_sensor.z_JJt = jacobianOfNorm_z * ugv_sensor.z_JJt * jacobianOfNorm_z';
  
  jacobianOfNorm_y = jacobianOfNorm(LeftmRight);
  ugv_sensor.y_JJt = jacobianOfNorm_y * RightandLeft_JJt * jacobianOfNorm_y';
  y_JJt = jacobianOfNorm_y * RightandLeft_JJt * jacobianOfNorm_y'
  
  ugv_sensor.x_JJt = wedgevector(-ugv_sensor.z_axis) * ugv_sensor.y_JJt * wedgevector(-ugv_sensor.z_axis)' + wedgevector(ugv_sensor.y_axis) * ugv_sensor.z_JJt * wedgevector(ugv_sensor.y_axis)';
  JJt2x2_x = ugv_sensor.x_JJt(1:2, 1:2);
  x_JJt = wedgevector(-ugv_sensor.z_axis) * ugv_sensor.y_JJt * wedgevector(-ugv_sensor.z_axis)' + wedgevector(ugv_sensor.y_axis) * ugv_sensor.z_JJt * wedgevector(ugv_sensor.y_axis)'
  
  jacobianOfAtan2_x = jacobianOfAtan2(ugv_sensor.x_axis(2),ugv_sensor.x_axis(1));
  JJt_atan2 = jacobianOfAtan2_x * JJt2x2_x * jacobianOfAtan2_x';
  ugv_sensor.yaw_JJt = JJt_atan2(1,1);
  
  
end

% disp(''); disp()
% disp(''); disp()
% disp(''); disp()
% disp(''); disp()


function C = CROSS(A, B)
% cv::Matx<double,4,1> Cross(cv::Matx<double,4,1> A, cv::Matx<double,4,1> B)
% 	{//computes A x B
% 		cv::Matx<double,4,1> C;
% 		C(0,0) =   ((A(1,0) * B(2,0)) - (A(2,0) * B(1,0)));
% 		C(1,0) = - ((A(0,0) * B(2,0)) - (A(2,0) * B(0,0)));
% 		C(2,0) =   ((A(0,0) * B(1,0)) - (A(1,0) * B(0,0)));
% 		C(3,0) = 0;
% 		return C;
% 	}

  C(1,1) =   ((A(2,1) * B(3,1)) - (A(3,1) * B(2,1)));
  C(2,1) = - ((A(1,1) * B(3,1)) - (A(3,1) * B(1,1)));
  C(3,1) =   ((A(1,1) * B(2,1)) - (A(2,1) * B(1,1)));
  C(4,1) = 0;

end


