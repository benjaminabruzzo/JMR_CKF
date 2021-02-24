
%%
clc
clear all
close all

root = '/home/benjamin/ros/data/calibrations/20200117/ugv1/';

left_images = dir([root 'raw/left/*.png']);
right_images = dir([root 'raw/right/*.png']);

for idx = 1:length(left_images)
% load images
%   idx = 2;
  left_image = imread([left_images(idx).folder '/' left_images(idx).name]);
  right_image = imread([right_images(idx).folder '/' right_images(idx).name]);

  left_image_histeq = histeq(rgb2gray(left_image));
  right_image_histeq = histeq(right_image);


%  blur and sharpen to clean up digitization
  windowWidth = 11; % Whatever you want.  More blur for larger numbers.
  kernel = ones(windowWidth) / windowWidth ^ 2;
  blurred_left = imfilter(left_image_histeq, kernel); % Blur the image.
  sharp_left = imsharpen(blurred_left); % sharpen the image
  sharp_left = imsharpen(blurred_left); % sharpen the image


  imwrite(sharp_left,[root 'cleaned/left/' left_images(idx).name])
  imwrite(right_image_histeq, [root 'cleaned/right/' right_images(idx).name])
end

%%
stereoCameraCalibrator([root 'cleaned/left/'],[root 'cleaned/right/'],111)

%%

left.K = stereoParams.CameraParameters1.IntrinsicMatrix';
left.P = cameraMatrix(stereoParams.CameraParameters1,eye(3,3),zeros(3,1))';
left.radial = stereoParams.CameraParameters1.RadialDistortion;
left.tangential = stereoParams.CameraParameters1.TangentialDistortion;
left.d = [left.radial(1:2) left.tangential left.radial(3)];
left.R = eye(3);
left.image_width = stereoParams.CameraParameters1.ImageSize(2);
left.image_height = stereoParams.CameraParameters1.ImageSize(1);

right.K = stereoParams.CameraParameters2.IntrinsicMatrix';
right.P = cameraMatrix(stereoParams.CameraParameters2,stereoParams.RotationOfCamera2,stereoParams.TranslationOfCamera2*0.001)';
right.radial = stereoParams.CameraParameters2.RadialDistortion;
right.tangential = stereoParams.CameraParameters2.TangentialDistortion;
right.d = [right.radial(1:2) right.tangential right.radial(3)];
right.R = stereoParams.RotationOfCamera2;
right.image_width = stereoParams.CameraParameters2.ImageSize(2);
right.image_height = stereoParams.CameraParameters2.ImageSize(1);


%%   figure(1); clf;
%     title('original left image')
%     imshow(left_image)

%   figure(2); clf; 
%     title('histeq left image')
%     imshow(left_image_histeq)
% 

%     figure(3); clf; 
%       title('blurred left image')
%       imshow(blurred_left)

%     figure(4); clf; 
%       title('sharp left image')
%       imshow(sharp_left)

%   blurred_left = imfilter(sharp_left, kernel); % Blur the image.
%     figure(5); clf; 
%       title('blurred left image')
%       imshow(blurred_left)
  
%     figure(6); clf; 
%       title('sharp left image')
%       imshow(sharp_left)


%% Find checkerboards
  % left_image_mod = histeq(rgb2gray(left_image+10));
  % left_image_mod = histeq(imsharpen(rgb2gray(left_image)));
  left_image_mod = histeq((sharp_left));
  right_image_mod = histeq(rgb2gray(right_image));

  [left_imagePoints, left_boardSize] = detectCheckerboardPoints(left_image_mod);
  [right_imagePoints, right_boardSize] = detectCheckerboardPoints(right_image_mod);

%%

figure(10); clf; 
  title('histeq left image')
  imshow(left_image_mod)
  hold on;
  try plot(left_imagePoints(:,1), left_imagePoints(:,2), 'r.', 'MarkerSize', 15); catch; end

figure(11); clf; 
  title('histeq right image')
  imshow(right_image_mod)
  hold on;
  try plot(right_imagePoints(:,1), right_imagePoints(:,2), 'r.', 'MarkerSize', 15); catch; end

  
