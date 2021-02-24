function data = cal_func(data)
%% calculated camera-to-ugv transformation
% known: vicon-to-ugv and vicon-to-gatorboard
% known: grid points on gatorboard
% observable: grid points in raw and rectified images
% how to convert observed grid points to UGV transformation?
% (1) rectify raw images - done in python script
% (2) generate pixel locations for grid intersections - done in python script

%% (3) triangulate grid locations in center camera space - done here:
%   data.caldata.stereo.right.M_inv = inv(data.caldata.stereo.right.M);
%   M = data.caldata.stereo.right.M_inv*data.caldata.stereo.right.P;
%   Baseline = abs(M(1,4));
%   FocalLength = data.caldata.stereo.right.P(1,1);
%   cx = data.caldata.stereo.right.P(1,3);
%   cy = data.caldata.stereo.right.P(2,3);
	A = [];
	B = [];
	for idx = 1:length(data.trial_numbers)
		data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).cam = ...
			mean(data.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).corner3d,3);
		
		data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).ugv = ...
			mean(data.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).gator3d,3);
		
		[data.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).R_cam2ugv_mean, ...
			data.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).t_cam2ugv_mean] = ...
			rigid_transform_3D(data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).cam',...
			data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).ugv');
		
		A = [A data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).cam'];
		B = [B data.check_points.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))).ugv'];
	end
	

  [data.R_cam2ugv, data.t_cam2ugv] = rigid_transform_3D(A,B);

	data.H_cam2ugv = [data.R_cam2ugv data.t_cam2ugv];

	
	disp([data.meta.calibration_folder data.param_file.H_filename])
	H_yaml = fopen([data.meta.calibration_folder data.param_file.H_filename],'w');
	fprintf(H_yaml, 'h00: %6.16f \n',		data.H_cam2ugv(1,1)); 
	fprintf(H_yaml, 'h01: %6.16f \n',		data.H_cam2ugv(1,2)); 
	fprintf(H_yaml, 'h02: %6.16f \n',		data.H_cam2ugv(1,3)); 
	fprintf(H_yaml, 'h03: %6.16f \n\n', data.H_cam2ugv(1,4)); 
	fprintf(H_yaml, 'h10: %6.16f \n',		data.H_cam2ugv(2,1)); 
	fprintf(H_yaml, 'h11: %6.16f \n',		data.H_cam2ugv(2,2)); 
	fprintf(H_yaml, 'h12: %6.16f \n',		data.H_cam2ugv(2,3)); 
	fprintf(H_yaml, 'h13: %6.16f \n\n', data.H_cam2ugv(2,4)); 
	fprintf(H_yaml, 'h20: %6.16f \n',		data.H_cam2ugv(3,1)); 
	fprintf(H_yaml, 'h21: %6.16f \n',		data.H_cam2ugv(3,2)); 
	fprintf(H_yaml, 'h22: %6.16f \n',		data.H_cam2ugv(3,3)); 
	fprintf(H_yaml, 'h23: %6.16f \n\n', data.H_cam2ugv(3,4)); 

	disp('data.H_cam2ugv = [data.R_cam2ugv data.t_cam2ugv];')
	disp(data.H_cam2ugv)
	
  return

%%
% 	load point vectors in camera frame
	data.check_points.cam = mean(data.test_cal.corner3d,3);
	data.check_points.ugv = mean(data.test_cal.gator3d,3);
	
	for i=0:5
		indices = sprintf('%u -> %u', (1+8*i), (8*(i+1))); 
		data.check_points.ugv2(1+8*i:8*(i+1),:) = flipud(data.check_points.ugv(1+8*i:8*(i+1),:));
	end
	
	[data.rectdata.R_cam2ugv_mean, data.rectdata.t_cam2ugv_mean] = rigid_transform_3D(data.check_points.cam', data.check_points.ugv2');
	
	R = data.rectdata.R_cam2ugv_mean
	t = data.rectdata.t_cam2ugv_mean
% 	H = wrapH(data.rectdata.R_cam2ugv_mean, data.rectdata.t_cam2ugv_mean)
	H = [R t]
	p = [data.check_points.cam(1,:) 1]'
	data.check_points.ugv2(1,:)
	p_ugv = H * p

	disp(data.param_file.H_filename)
	H_yaml = fopen(data.param_file.H_filename,'w');
	fprintf(H_yaml, 'h00: %6.16f \n', H(1,1)); 
	fprintf(H_yaml, 'h01: %6.16f \n', H(1,2)); 
	fprintf(H_yaml, 'h02: %6.16f \n', H(1,3)); 
	fprintf(H_yaml, 'h03: %6.16f \n\n', H(1,4)); 
	fprintf(H_yaml, 'h10: %6.16f \n', H(2,1)); 
	fprintf(H_yaml, 'h11: %6.16f \n', H(2,2)); 
	fprintf(H_yaml, 'h12: %6.16f \n', H(2,3)); 
	fprintf(H_yaml, 'h13: %6.16f \n\n', H(2,4)); 
	fprintf(H_yaml, 'h20: %6.16f \n', H(3,1)); 
	fprintf(H_yaml, 'h21: %6.16f \n', H(3,2)); 
	fprintf(H_yaml, 'h22: %6.16f \n', H(3,3)); 
	fprintf(H_yaml, 'h23: %6.16f \n\n', H(3,4)); 
  
  return

%% (4) convert grid points to 3d points in ugv space:
% convert points to homogenous form  
%   data.check_points.gat = [data.caldata.left.chessboard3Dpoints ones(length(data.caldata.left.chessboard3Dpoints),1)];

%   for i = 1:size(data.vicon.gatorboard.quaternion.vic,1)
%   for mdx = 1:length(data.rectdata.rect_indices)
%     i = data.rectdata.rect_indices(mdx);
%     % calculate the transformation from gator board to vicon origin
%     data.vicon.Rvic2gat(:,:,i) = q2mat(data.vicon.gatorboard.quaternion.vic(i,:));
%     data.vicon.Hvic2gat(:,:,i) = wrapH(data.vicon.Rvic2gat(:,:,i), data.vicon.gatorboard.position.vic(i,:)');
%     data.vicon.Hgat2vic(:,:,i) = invertH(data.vicon.Hvic2gat(:,:,i));
%     
%     % calculate the transformation from vicon origin to ugv base  
%     data.vicon.Rvic2ugv(:,:,i) = q2mat(data.vicon.ugv.quaternion.vic(i,:));
%     data.vicon.Hvic2ugv(:,:,i) = wrapH(data.vicon.Rvic2ugv(:,:,i), data.vicon.ugv.position.vic(i,:)');
%     
%     % calculate the transformation from gator board to ugv base  
%     data.vicon.Hgat2ugv(:,:,i) = data.vicon.Hvic2ugv(:,:,i) * data.vicon.Hgat2vic(:,:,i);
%     
%     % remap gator board points into ugv frame
%     for idx = 1:length(data.check_points.gat)
%       check_points(idx,:) = (data.vicon.Hgat2ugv(:,:,i) * data.check_points.gat(idx,:)')';
%     end; clear idx
%     
%     data.check_points.ugv(:,:,mdx) = check_points; clear check_points
%   end; clear i mdx
	
	
%% (5) calculate tranformation for points
for i = 1:size(data.check_points.cam,3)
  A = data.check_points.ugv(:,1:3,i)';
  B = data.check_points.cam(:,1:3,i)';
  [data.rectdata.R_cam2ugv(:,:,i), data.rectdata.t_cam2ugv(:,:,i)] = rigid_transform_3D(A, B);
end

%%

	A = [];
	B = [];
	for i = 1:length(data.rectdata.rect_indices)
		A = [A data.check_points.ugv(:,1:3,i)'];
		B = [B data.check_points.cam(:,1:3,i)'];
	end
	[data.rectdata.R_cam2ugv_combined, data.rectdata.t_cam2ugv_combined] = rigid_transform_3D(A, B);
  
  data.rectdata.H_cam2ugv_combined = wrapH(data.rectdata.R_cam2ugv_combined, data.rectdata.t_cam2ugv_combined);
  
%% try using just the corner point of the chackerboard


	A = [];
	B = [];
	for i = 1:length(data.rectdata.rect_indices)
		A = [A data.check_points.ugv(1,1:3,i)'];
		B = [B data.check_points.cam(1,1:3,i)'];
	end
  [data.rectdata.R_cam2ugv_combined, data.rectdata.t_cam2ugv_combined] = rigid_transform_3D(A, B);
  data.rectdata.H_cam2ugv_combined = wrapH(data.rectdata.R_cam2ugv_combined, data.rectdata.t_cam2ugv_combined);
	
	%% write data to files
	data.param_file.Baseline = Baseline;
	data.param_file.FocalLength = FocalLength;
	data.param_file.Cx = cx;
  data.param_file.Cy = cy;
	data.param_file.RightOffset = data.caldata.stereo.right.P(1,4);
	data.param_file.camXOffset = 0.005240;
	data.param_file.camYOffset = 0.000000;
	data.param_file.camZOffset = 0.233270; 
	
	data = writeParamFile(data);
	

	
end

function [] = sandBoxing()
%% Sandboxing
clc
w = 20 * pi/180
Hcam_ugv = [Rz(-pi/2)*Rx(-(pi/2-w)) [0 0 0]'; [0 0 0 1]]

for mdx = 1:size(data.check_points.cam,3)
  for i = 1:size(data.check_points.cam,1)
    p_test = data.check_points.cam(i,:,mdx)'
    data.check_points.test(i,:,mdx) = (Hcam_ugv * p_test)';
  end
end



%%
clc
R_gat2ugv = q2mat([0.653, -0.502, 0.349, -0.447])
t_gat2ugv = [0.413, 1.854, -1.703]'
H_gat2ugv = wrapH(R_gat2ugv, t_gat2ugv)
% R = q2mat([0.653, -0.502, 0.349, 0.447])	
% rosrun tf tf_echo vicon/gatorboard/gatorboard vicon/ugv1/ugv1
% At time 1579294580.119
% - Translation: [0.413, 1.854, -1.703]
% - Rotation: in Quaternion [0.653, -0.502, 0.349, 0.447]
%             in RPY (radian) [2.564, -1.130, -0.939]
%             in RPY (degree) [146.916, -64.751, -53.803]

R_ugv2gat = q2mat([0.653, -0.502, 0.349, -0.447])
% rosrun tf tf_echo vicon/ugv1/ugv1 vicon/gatorboard/gatorboard
% At time 1579294846.505
% - Translation: [2.074, 0.975, 1.120]
% - Rotation: in Quaternion [0.653, -0.502, 0.349, -0.447]
%             in RPY (radian) [-1.936, -0.007, -1.316]
%             in RPY (degree) [-110.953, -0.386, -75.404]

R_vic2gat = q2mat([0.583, 0.578, -0.397, -0.410])
% rosrun tf tf_echo vicon vicon/gatorboard/gatorboard
% At time 1579294943.844
% - Translation: [-0.189, -5.628, 1.134]
% - Rotation: in Quaternion [0.583, 0.578, -0.397, -0.410]
%             in RPY (radian) [-1.927, -0.011, 1.555]
%             in RPY (degree) [-110.399, -0.643, 89.117]

R_vic2ugv = q2mat([0.005, -0.001, 0.991, 0.135])
% rosrun tf tf_echo vicon vicon/ugv1/ugv1
% At time 1579294965.882
% - Translation: [2.060, -5.236, -0.005]
% - Rotation: in Quaternion [0.005, -0.001, 0.991, 0.135]
%             in RPY (radian) [-0.002, -0.010, 2.871]
%             in RPY (degree) [-0.088, -0.587, 164.520]

R_vic2ugv'*R_vic2gat

% 	r = data.rectdata.R_cam2ugv_combined
% 	
% 	alpha = atan2(r(2,1), r(1,1));
% 	beta  = atan2(-r(3,1), sqrt(r(3,2)*r(3,2) + r(3,3)*r(3,3)));
% 	gamma = atan2(r(3,2), r(3,3));
% 	
% 	sprintf('args="0.0 0.0 0.0 %2.6f %2.6f %2.6f ',alpha, beta, gamma)
% 	
% 	args="0.005240 0.000000 0.233270 -1.57079632679 0 -1.22173047640 
	
	
	
% $\displaystyle \alpha = \tan^{-1} (r_{21}/r_{11}) ,$	(3.44)
% $\displaystyle \beta = \tan^{-1} \Big(-r_{31} \big/ \sqrt{r^2_{32}+r^2_{33}}\Big) ,$	(3.45)
% $\displaystyle \gamma = \tan^{-1} (r_{32}/r_{33}).$
%% converting vicon quaternion to axis-angle:
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
% angle = 2 * acos(qw) = 2 * 90 degrees = 180 degrees (or -180 degrees which is equivalent)
% x = qx / sqrt(1-qw*qw) = qx
% y = qy / sqrt(1-qw*qw) = qy
% z = qz / sqrt(1-qw*qw) = qw




%% 

root = '/home/benjamin/ros/data/calibrations/20200117/ugv1/';

left_images = dir([root 'test_rect/left*.png']);
right_images = dir([root 'test_rect/right*.png']);

idx = 2;
left_image = imread([data.meta.data_folder '/test_rect/' left_images(idx).name]);
right_image = imread([data.meta.data_folder '/test_rect/' right_images(idx).name]);

left_xy = data.rectdata.(matlab.lang.makeValidName( sprintf('rect_%03d', data.rectdata.rect_indices(idx,1)) )).pixels.left.xy;
right_xy = data.rectdata.(matlab.lang.makeValidName( sprintf('rect_%03d', data.rectdata.rect_indices(idx,1)) )).pixels.right.xy;


figure(10); clf; 
  imshow(left_image)
  hold on;
  try plot(left_xy(:,1), left_xy(:,2), 'r.', 'MarkerSize', 15); catch; end
  title(sprintf('left rect %03d', data.rectdata.rect_indices(idx,1)))

figure(11); clf; 
  imshow(right_image)
  hold on;
  try plot(right_xy(:,1), right_xy(:,2), 'r.', 'MarkerSize', 15); catch; end
  title(sprintf('right rect %03d', data.rectdata.rect_indices(idx,1)))

  

%% 
% absolute distance from camera to checker points: 
abs_points_cam = sqrt(sum(data.check_points.cam(:,1:3,2).*data.check_points.cam(:,1:3,2), 2));

% absolute distance from camera to checker points: 
abs_points_ugv = sqrt(sum(data.check_points.ugv(:,1:3,2).*data.check_points.ugv(:,1:3,2),2));

[abs_points_cam abs_points_ugv [abs_points_cam-abs_points_ugv]]


end

function data = writeParamFile(data)
% open file for writing
	Stereo.yamlID = fopen([data.param_file.path data.param_file.filename],'w');
	fprintf(Stereo.yamlID,'# %s stereo camera calibration parameters\n', data.meta.ugv_n);
	fprintf(Stereo.yamlID,'# Left SERIAL %s\n', data.caldata.left.serial);
	fprintf(Stereo.yamlID,'# Right SERIAL %s\n', data.caldata.right.serial);
	fprintf(Stereo.yamlID,'# yaml file auto-generated on : %s\n', data.meta.date);
	fprintf(Stereo.yamlID,'#### TAB CANNOT BE USED #####\n\n');

	% write data to file for publishing rosparams
	fprintf(Stereo.yamlID,'FocalLength: %6.16f\n',data.param_file.FocalLength);
	fprintf(Stereo.yamlID,'Baseline: %6.16f\n',data.param_file.Baseline);
	fprintf(Stereo.yamlID,'RightOffset: %6.16f\n',data.caldata.stereo.right.P(1,4));
	fprintf(Stereo.yamlID,'camXOffset: %6.6f\n',data.param_file.camXOffset);
	fprintf(Stereo.yamlID,'camYOffset: %6.6f\n',data.param_file.camYOffset);
	fprintf(Stereo.yamlID,'camZOffset: %6.6f\n',data.param_file.camZOffset);
	fprintf(Stereo.yamlID,'Cx: %6.16f\n',data.param_file.Cx);
	fprintf(Stereo.yamlID,'Cy: %6.16f\n',data.param_file.Cy);
	fprintf(Stereo.yamlID,'dRx: 0.00\n');
	fprintf(Stereo.yamlID,'dRz: 0.00');

	disp([data.param_file.path data.param_file.filename ' created']);
	disp(' ');disp(' ');
	
end

function H = wrapH(R,t)
%% 	cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
% 	{//ROS_INFO("uav cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)");
% 	// H = [R -R*t(1:3,1); 0 0 0 1];
% 		cv::Mat H, nRt, t;
% 		t = (cv::Mat_<double>(3, 1) << t_in.at<double>(0,0),t_in.at<double>(1,0),t_in.at<double>(2,0));
% 		nRt = -R_in * t;
% 		H = (cv::Mat_<double>(4, 4) <<
% 			R_in.at<double>(0,0), R_in.at<double>(0,1), R_in.at<double>(0,2), nRt.at<double>(0,0),
% 			R_in.at<double>(1,0), R_in.at<double>(1,1), R_in.at<double>(1,2), nRt.at<double>(1,0),
% 			R_in.at<double>(2,0), R_in.at<double>(2,1), R_in.at<double>(2,2), nRt.at<double>(2,0),
% 			0,0,0,1);
% 		return H;
% 	}
    H = [R -R*t(1:3,1); 0 0 0 1];
end

function Hinv = invertH(H)
% cv::Mat invertH(cv::Mat H)
% 	{// Hinv = [R' -R'*t; 0 0 0 1];
% 		cv::Mat Hinv, RT, R, t, nRTt;
% 		t = (cv::Mat_<double>(3, 1) << H.at<double>(0,3),H.at<double>(1,3),H.at<double>(2,3));
% 		R = (cv::Mat_<double>(3, 3) << 	H.at<double>(0,0), H.at<double>(0,1), H.at<double>(0,2),
% 																		H.at<double>(1,0), H.at<double>(1,1), H.at<double>(1,2),
% 																		H.at<double>(2,0), H.at<double>(2,1), H.at<double>(2,2));
% 		RT = R.t();
% 		nRTt = -RT*t;
% 		Hinv = (cv::Mat_<double>(4, 4) <<
% 			RT.at<double>(0,0), RT.at<double>(0,1), RT.at<double>(0,2), nRTt.at<double>(0,0),
% 			RT.at<double>(1,0), RT.at<double>(1,1), RT.at<double>(1,2), nRTt.at<double>(1,0),
% 			RT.at<double>(2,0), RT.at<double>(2,1), RT.at<double>(2,2), nRTt.at<double>(2,0),
% 			0,0,0,1);
% 		return Hinv;
% 	}
  R = H(1:3,1:3);
  t = H(1:3,4);
  Hinv = [R' -R'*t; 0 0 0 1];
end

function R = q2mat(q)

  qx = q(1); qy = q(2); qz = q(3); qw = q(4);
%   normalize quaternion
  n = 1.0 /sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
  qx = qx * n;
  qy = qy * n;
  qz = qz * n;
  qw = qw * n;
  
%   Then you can create your matrix:

R =[1.0 - 2.0*qy*qy - 2.0*qz*qz, 2.0*qx*qy - 2.0*qz*qw, 2.0*qx*qz + 2.0*qy*qw; ...
    2.0*qx*qy + 2.0*qz*qw, 1.0 - 2.0*qx*qx - 2.0*qz*qz, 2.0*qy*qz - 2.0*qx*qw; ...
    2.0*qx*qz - 2.0*qy*qw, 2.0*qy*qz + 2.0*qx*qw, 1.0 - 2.0*qx*qx - 2.0*qy*qy];
end

function [R] = cv2rodrigues(rvec)
% rvec should be a 3x1
    s1 = size(rvec,1);
    s2 = size(rvec,2);
    if (s1==1) && (s2==3)
        rvec = rvec';
    elseif (s1==3) && (s2==1)
    %     do nothing
    else
        return
    end

    theta = norm(rvec);
    r = rvec/theta;
    R1 = cos(theta)*eye(3);
    R2 = (1-cos(theta))*r*r';
    R3 = sin(theta)*crossmatrix(r);
    R = R1+R2+R3;
    

end

function vec = calcVec(xl,yl,xr,yr,b,f)
% cv::Matx<double,4,1> calcVec(cv::Vec<double,4> Pix)
%   Vecpix(3,0) = 1;
%   Vecpix(2,0) = (Baseline*FocalLength) / disparity;
%   Vecpix(1,0) = Vecpix(2,0) * (Pix[1] + Pix[3] ) / ( 2 * FocalLength );
%   Vecpix(0,0) = Vecpix(2,0) * (Pix[0] + Pix[2] ) / ( 2 * FocalLength );

  d = abs(xl - xr);

  w = 1;
  z = b*f/d;
  y = z*(yl+yr)/(2*f);
  x = z*(xl+xr)/(2*f);
  
  vec = [x y z w];

end

