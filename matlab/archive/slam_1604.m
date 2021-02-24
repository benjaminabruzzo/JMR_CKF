disp('running clas.m')
% set(0, 'DefaultFigureVisible', 'off');
% set(0, 'DefaultFigureVisible', 'on');

%% meta.date = '20190313/'; % gazebo testing uavSLAM
    meta.date = '20190826/'; % 
    meta.run = '003'; 
    
    %% define meta data
% meta.dataroot = '/media/benjamin/devsdb/hast/data/';
% meta.dataroot = '/Users/benjamin/hast/data/';
meta.dataroot = '/home/benjamin/ros/data/';
meta.saveplots = false;
meta.saveirosplots = false;
meta.saveplotroot = ['/home/benjamin/ros/data/' [meta.date meta.run] '/'];

%% data = loadData(meta);
meta.ugv_leader = "ugv1_";
meta.ugv_leader_stereo = "ugv1Stereo";
% data = loadSlamData(meta);
data = loadSlamData_newformat(meta);

%% plotclas
% [data, meta] = plotclas_func(data, meta);
