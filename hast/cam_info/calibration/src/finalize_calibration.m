%% set some params
	clc	
	clear all
	close all
	data.meta.ugv_n = 'ugv2';
	data.meta.date = '20200129';
	data.meta.data_folder = '/home/benjamin/ros/data';
	data.meta.calibration_folder = '~/ros/src/metahast/hast/cam_info/calibration';

	data.param_file.path = [data.meta.data_folder];
	data.param_file.H_filename = sprintf('/%s_H_cam2ugv_%s.yaml', data.meta.ugv_n, data.meta.date);
% 	data.param_file.H_filename = ['~/ros/src/metahast/hast/cam_info/calibration/' data.meta.ugv_n '_H_cam2ugv.yaml'];
	
% load data
% 	data.trial_numbers = {'001', '002','003'};
% 	data.trial_numbers = {'004','005','006','007','008'};
% 	data.trial_numbers = {'010','011','012','013','014'};
% 	data.trial_numbers = {'005'};
% 	data.trial_numbers = {'018','019','020','021','022'};
	data.trial_numbers = {'004'};
	for idx = 1:length(data.trial_numbers)
		data_path = sprintf('%s/%s/%s/', data.meta.data_folder, data.meta.date, data.trial_numbers{idx}); 
		data_file = sprintf('%s_test_cal_%s', data.meta.ugv_n, data.trial_numbers{idx}); 
		cd(data_path)
		eval(data_file)
		data.(matlab.lang.makeValidName(sprintf('trial_%s', data.trial_numbers{idx}))) = test_cal;
% 		clear test_cal
	end

% finalize calibration
	cd([data.meta.calibration_folder '/src'])
	data = cal_func(data);

% 	[mean(test_cal.raw.left_pixel,3) mean(test_cal.left_pixel,3)]


%%
% Sandboxing

err = [mean(test_cal.remapped3d,3) - mean(test_cal.gator3d,3) ];
err_squared = sum(err.*err,2);
rms = sqrt(err_squared)


% ugv1Stereo.time(1,1) = 24.587393;
% ugv1Stereo.Red.right.xy(1,:) = [-48.62956238 -464.04523849];
% ugv1Stereo.Red.right.rawxy(1,:) = [ 723.26922607  922.11535645];
% ugv1Stereo.Red.right.radius(1,:) =  8.75803375;
% ugv1Stereo.Red.left.xy(1,:) = [-110.86033630 -464.92988205];
% ugv1Stereo.Red.left.rawxy(1,:) = [ 785.50000000  923.00000000];
% ugv1Stereo.Red.left.radius(1,:) =  8.73222446;
% ugv1Stereo.Red.left.disp(1,:) = -62.23077393;
% ugv1Stereo.Blue.right.xy(1,:) = [ 114.63966370 -472.42988205];
% ugv1Stereo.Blue.right.rawxy(1,:) = [ 560.00000000  930.50000000];
% ugv1Stereo.Blue.right.radius(1,:) =  7.15901089;
% ugv1Stereo.Blue.left.xy(1,:) = [ 55.63966370 -473.42988205];
% ugv1Stereo.Blue.left.rawxy(1,:) = [ 619.00000000  931.50000000];
% ugv1Stereo.Blue.left.radius(1,:) =  7.15901089;
% ugv1Stereo.Blue.left.disp(1,:) = -59.00000000;
% ugv1Stereo.Green.right.xy(1,:) = [ 59.78096008 -473.58204269];
% ugv1Stereo.Green.right.rawxy(1,:) = [ 614.85870361  931.65216064];
% ugv1Stereo.Green.right.radius(1,:) =  10.51351738;
% ugv1Stereo.Green.left.xy(1,:) = [-5.05326843 -474.09522629];
% ugv1Stereo.Green.left.rawxy(1,:) = [ 679.69293213  932.16534424];
% ugv1Stereo.Green.left.radius(1,:) =  10.19674683;
% ugv1Stereo.Green.left.disp(1,:) = -64.83422852;
% ugv1Stereo.Red.P_cam(1,:)   = [-0.17961235, -1.04618167,  2.25970497];
% ugv1Stereo.Blue.P_cam(1,:)  = [ 0.20226379, -1.12352559,  2.38344388];
% ugv1Stereo.Green.P_cam(1,:) = [ 0.05915766, -1.02438767,  2.16896526];
% ugv1Stereo.uav.Obs_cam(1,:) = [ 0.01132572, -1.08485363,  2.32157443];
% ugv1Stereo.Red.P_ugv(1,:)   = [ 2.70045800, -0.25750269,  0.02504590];
% ugv1Stereo.Blue.P_ugv(1,:)  = [ 2.88160428,  0.10758255, -0.00691629];
% ugv1Stereo.Green.P_ugv(1,:) = [ 2.63280810, -0.01043429,  0.01502130];
% ugv1Stereo.uav.P_ugv(1,:)   = [ 2.79103114, -0.07496007,  0.00906480];
% ugv1Stereo.Red.J_cam(1,:,1) = [ 0.00088003, -0.00200620,  0.00000000];
% ugv1Stereo.Red.J_cam(2,:,1) = [ 0.01682733, -0.01682733, -0.00225234];
% ugv1Stereo.Red.J_cam(3,:,1) = [-0.03631170,  0.03631170,  0.00000000];
% ugv1Stereo.Blue.J_cam(1,:,1) = [-0.00230802,  0.00112018,  0.00000000];
% ugv1Stereo.Blue.J_cam(2,:,1) = [ 0.01906294, -0.01906294, -0.00237567];
% ugv1Stereo.Blue.J_cam(3,:,1) = [-0.04039735,  0.04039735,  0.00000000];
% ugv1Stereo.Green.J_cam(1,:,1) = [-0.00099670, -0.00008425,  0.00000000];
% ugv1Stereo.Green.J_cam(2,:,1) = [ 0.01580866, -0.01580866, -0.00216189];
% ugv1Stereo.Green.J_cam(3,:,1) = [-0.03345402,  0.03345402,  0.00000000];
% ugv1Stereo.Red.JJt_cam(1,:,1) = [ 0.00004799,  0.00048568, -0.00104804];
% ugv1Stereo.Red.JJt_cam(2,:,1) = [ 0.00048568,  0.00571391, -0.01222058];
% ugv1Stereo.Red.JJt_cam(3,:,1) = [-0.00104804, -0.01222058,  0.02637079];
% ugv1Stereo.Blue.JJt_cam(1,:,1) = [ 0.00006582, -0.00065352,  0.00138490];
% ugv1Stereo.Blue.JJt_cam(2,:,1) = [-0.00065352,  0.00732435, -0.01540185];
% ugv1Stereo.Blue.JJt_cam(3,:,1) = [ 0.00138490, -0.01540185,  0.03263892];
% ugv1Stereo.Green.JJt_cam(1,:,1) = [ 0.00001000, -0.00014425,  0.00030525];
% ugv1Stereo.Green.JJt_cam(2,:,1) = [-0.00014425,  0.00504501, -0.01057726];
% ugv1Stereo.Green.JJt_cam(3,:,1) = [ 0.00030525, -0.01057726,  0.02238342];
% ugv1Stereo.Red.JJt_ugv(1,:,1) = [ 0.03118622, -0.00438668, -0.00295958];
% ugv1Stereo.Red.JJt_ugv(2,:,1) = [-0.00438668,  0.00062350,  0.00041583];
% ugv1Stereo.Red.JJt_ugv(3,:,1) = [-0.00295958,  0.00041583,  0.00032298];
% ugv1Stereo.Blue.JJt_ugv(1,:,1) = [ 0.03940464, -0.00256562, -0.00398839];
% ugv1Stereo.Blue.JJt_ugv(2,:,1) = [-0.00256562,  0.00017413,  0.00026046];
% ugv1Stereo.Blue.JJt_ugv(3,:,1) = [-0.00398839,  0.00026046,  0.00045032];
% ugv1Stereo.Green.JJt_ugv(1,:,1) = [ 0.02688890, -0.00245818, -0.00274547];
% ugv1Stereo.Green.JJt_ugv(2,:,1) = [-0.00245818,  0.00023062,  0.00025126];
% ugv1Stereo.Green.JJt_ugv(3,:,1) = [-0.00274547,  0.00025126,  0.00031892];
% ugv1Stereo.uav.p_cov(1,:,1) = [ 0.01764771, -0.00173808, -0.00173699];
% ugv1Stereo.uav.p_cov(2,:,1) = [-0.00173808,  0.00019941,  0.00016907];
% ugv1Stereo.uav.p_cov(3,:,1) = [-0.00173699,  0.00016907,  0.00019332];
% ugv1Stereo.uav.z_axis(1,:) = [ 0.06079257  0.05707851  0.99651709];
% ugv1Stereo.uav.z_JJt(1,:,1) = [ 0.00662575, -0.00432080, -0.00015672];
% ugv1Stereo.uav.z_JJt(2,:,1) = [-0.00432080,  0.00428362,  0.00001823];
% ugv1Stereo.uav.z_JJt(3,:,1) = [-0.00015672,  0.00001823,  0.00000852];
% ugv1Stereo.uav.y_axis(1,:) = [ 0.44311003  0.89305137 -0.07818414];
% ugv1Stereo.uav.y_JJt(1,:,1) = [ 0.29759019, -0.14982938, -0.02481494];
% ugv1Stereo.uav.y_JJt(2,:,1) = [-0.14982938,  0.07543948,  0.01254004];
% ugv1Stereo.uav.y_JJt(3,:,1) = [-0.02481494,  0.01254004,  0.00259836];
% ugv1Stereo.uav.x_axis(1,:) = [ 0.89440358 -0.44631973 -0.02899883];
% ugv1Stereo.uav.x_JJt(1,:,1) = [ 0.07353234,  0.14816007, -0.01238578];
% ugv1Stereo.uav.x_JJt(2,:,1) = [ 0.14816007,  0.29856838, -0.02558936];
% ugv1Stereo.uav.x_JJt(3,:,1) = [-0.01238578, -0.02558936,  0.01183317];
% ugv1Stereo.uav.yaw_ugv(1,1) = -26.51982910;
% ugv1Stereo.uav.yaw_jacobianOfAtan2(1,:) = [ 0.44669537,  0.89515635];
% ugv1Stereo.uav.UAV_yaw_var(1,1) =  0.37240393;
% ugv1Stereo.uav.GmB(1,:) = [-0.24879618 -0.11801684  0.02193759];
% ugv1Stereo.uav.BmR(1,:) = [ 0.18114628  0.36508524 -0.03196219];
% ugv1Stereo.uav.RmG(1,:) = [ 0.06764990 -0.24706840  0.01002460];
% 
% 
% H*[ugv1Stereo.Red.P_cam 1]'