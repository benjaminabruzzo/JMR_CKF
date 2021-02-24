disp('batching clas.m')
set(0, 'DefaultFigureVisible', 'off');

meta.date = '20180905/'; % gazebo experiment data

jstr = {...
%     '003';...
%     '006';...
%     '010';...
%     '016';...
%     '018';...
%     '019';...
%     '020';...
%     '022';...
%     '023';...
%     '024';...
%     '025';...
%     '026';...
%     '028';...
%     '034';...
%     '042';...
%     '046';...
    '048';...
    '049';...
    '051';...
    '052';...
    '053';...
    '055';...
    };
meta.dataroot = '/home/benjamin/ros/data/';
meta.saveplots = false;
meta.saveirosplots = false;

for run = 1:length(jstr)
    clc
    meta.run = jstr{run};
    disp(['meta.run : ' meta.run])
    try 
        data = loadData(meta); 

    catch; end
    clear data
end
