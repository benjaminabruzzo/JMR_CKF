%%
disp('1->8')
[data.check_points.cam(1:8,:) data.check_points.ugv(1:8,:)]

%%
disp('9->16')
[data.check_points.cam(9:16,:) flipud(data.check_points.ugv2(9:16,:))]



%% 
for i=0:5
	indices = sprintf('%u -> %u', (1+8*i), (8*(i+1))); 
 	data.check_points.ugv2(1+8*i:8*(i+1),:) = flipud(data.check_points.ugv(1+8*i:8*(i+1),:));
end



%% 
% 		// Wedge = 20; // tilt of cameras above the ugv body horizon
% 		// fprintf (pFile,"%s.pgryaml.Wedge = %6.4f;\n", s_handle.c_str(), Wedge);
% 		// cam2ugv33 = makeRz(-(90)*Pi/180)*makeRx(-(90-Wedge)*Pi/180);

clc

Wedge = 20 * pi/180;

Rz((-90)*pi/180);
Rx(-(90-Wedge)*pi/180);

cam2ugv33 = Rz(-pi/2) * Rx(-(90-Wedge)*pi/180);
t = [0.005240 0.000000 0.233270]';
cam2ugv44 = [cam2ugv33 t; 0 0 0 1]


H_ugv1 = [0.103287206644015, -0.332198424864955, 0.937537177641996, 0.252912336109118; ...
  0.994651016634708, 0.035493860371883, -0.097002788534366, 0.177479629991165; ...
  -0.001052640118050, 0.942541453938841, 0.334087562707201, 0.255987099679203; ...
  0 0 0 1]

H_ugv2 = [-0.0857677551068528, -0.3695933699489570, 0.9252268008838179, -0.0608008068202746; ...
      0.9962886188649279, -0.0250375953981567, 0.0823535472028513, -0.3318726468332660; ...
     -0.0072718707458930,  0.9288562104580297, 0.3703690864386544,  0.1343118833215894; ...
      0,0,0,1]


for i = 1:size(data.ugv1Stereo.Blue.P_cam,1)
  data.ugv1Stereo.Red.P_ugvA(i,:) = (cam2ugv44 * [data.ugv1Stereo.Red.P_cam(i,:) 1]')';
  data.ugv1Stereo.Red.P_ugvB(i,:) = (H_ugv1 * [data.ugv1Stereo.Red.P_cam(i,:) 1]')';
  data.ugv1Stereo.Blue.P_ugvA(i,:) = (cam2ugv44 * [data.ugv1Stereo.Blue.P_cam(i,:) 1]')';
  data.ugv1Stereo.Blue.P_ugvB(i,:) = (H_ugv1 * [data.ugv1Stereo.Blue.P_cam(i,:) 1]')';
  data.ugv1Stereo.Green.P_ugvA(i,:) = (cam2ugv44 * [data.ugv1Stereo.Green.P_cam(i,:) 1]')';
  data.ugv1Stereo.Green.P_ugvB(i,:) = (H_ugv1 * [data.ugv1Stereo.Green.P_cam(i,:) 1]')';
  data.ugv1Stereo.uav.P_ugvA(i,:) = 0.5 * (data.ugv1Stereo.Red.P_ugvA(i,:) + data.ugv1Stereo.Blue.P_ugvA(i,:));
  data.ugv1Stereo.uav.P_ugvB(i,:) = 0.5 * (data.ugv1Stereo.Red.P_ugvB(i,:) + data.ugv1Stereo.Blue.P_ugvB(i,:));

end

