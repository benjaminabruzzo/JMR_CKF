function [data, meta] = plotslam_func(data, meta)
default_colors = getpref('display','default_colors');

%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,1),'bs', 'displayname', 'uav x slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,1),'gs', 'displayname', 'uav x oneCKF est'); catch; end
        
%         try plot(data.uavSLAM.time, data.uavSLAM.uav.aug.p.global(:,1),'bx', 'displayname', 'uav x slam aug'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav x gazebo mocap'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,1), ...
%                 'g.', 'displayname', 'uav x stereo'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,1), 'r.', 'displayname', 'uav inertial x'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,1),'m.', 'displayname', 'uav inertial odom x, global'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,1), 'mo', 'displayname', 'uav augmentedState x'); catch; end
        
    hold off; grid on
    title(['uav x position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('x [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -1.5 3.5];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
   
    if meta.saveplots
      
          meta.savefilename = ['uavSLAM_x_' meta.run];
          try_save_plot(meta);
    end
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,2),'bs', 'displayname', 'uav y slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,2),'gs', 'displayname', 'uav y oneCKF est'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav y gazebo mocap'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,2), ...
%                 'g.', 'displayname', 'uav y stereo'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,2), 'r.', 'displayname', 'uav inertial y'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,2),'m.', 'displayname', 'uav inertial odom y, global'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,2), 'mo', 'displayname', 'uav augmentedState y'); catch; end
    hold off; grid on
    title(['uav y position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('y [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -2 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
    if meta.saveplots
          meta.savefilename = ['uavSLAM_y_' meta.run];
          try_save_plot(meta);
    end
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,3),'bs', 'displayname', 'uav z slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,3),'gs', 'displayname', 'uav z oneCKF est'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,3), ...
%                 'go', 'displayname', 'uav z stereo'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,3),'k.', 'displayname', 'uav z gazebo mocap'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,3), 'r.', 'displayname', 'uav inertial z'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,3), 'mo', 'displayname', 'uav augmentedState z'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,3),'m.', 'displayname', 'uav inertial odom z, global'); catch; end
    hold off; grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -1 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end    
    if meta.saveplots
          meta.savefilename = ['uavSLAM_z_' meta.run];
          try_save_plot(meta);
    end
%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.global,'bs', 'displayname', 'uav yaw slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.Yaw*pi/180,'gs', 'displayname', 'uav yaw oneCKF est'); catch; end
        
        try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.radians,'k.', 'displayname', 'uav yaw gazebo mocap'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPhi_gl, 'r.', 'displayname', 'uav inertial phi'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odomPhi, 'm.', 'displayname', 'uav inertial phi odom'); catch; end
% %         try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv*pi/180, 'go', 'displayname', 'uav stereo yaw'); catch; end
        
        
    hold off; grid on
    title(['uav yaw, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('yaw [rad]')
    legend('toggle');legend('Location', 'SouthEast')
    
    if meta.saveplots
          meta.savefilename = ['uavSLAM_yaw_' meta.run];
          try_save_plot(meta);
    end
%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    ii = 0; 
    for i=1:length(data.uavSLAM.april.tagsDetected)
        
        try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,1), ...
                 data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,2), ...
                 'o', 'Color', default_colors{i}, 'MarkerFaceColor', default_colors{i}, 'displayname', data.uavSLAM.april.tagsDetected{i}); 
               ii = ii + 1; legend_str{ii} = h1.DisplayName;
               
        catch; end
    end

    for i=1:length(data.vicon.april_list)
        try
            xyz_avg = mean(data.vicon.(matlab.lang.makeValidName(data.vicon.april_list{i})).P.global);
            h2 = plot(xyz_avg(1), xyz_avg(2), 'ks', 'MarkerFaceColor', [0,0,0]);
        catch
            continue
        end
        clear xyz_avg
    end
    
    
    hold off; grid on
    title(['landmark estimated position, global frame ' [meta.date meta.run]])
    xlabel('X [m]'); ylabel('Y [m]')
    try columnlegend(1,legend_str); catch; end; clear legend_str

%     hold on
%         for i=1:length(data.uavSLAM.april.tagsDetected)
%             try plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
%                      data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
%                      '.', 'displayname', data.uavSLAM.april.tagsDetected{i} ); catch; end
%         end
%     hold off;

    if meta.saveplots
          meta.savefilename = ['uavSLAM_landmarks_' meta.run];
          try_save_plot(meta);
    end
%% figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,1), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['x meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,1), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['x pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,1), ...
                    's', 'Color', default_colors{3}, 'displayname', ['x est P']); catch; end

    end
  hold off
  grid on
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -2 5];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
    
catch; end
%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,2), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['y meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,2), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['y pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,2), ...
                    's', 'Color', default_colors{3}, 'displayname', ['y est P']); catch; end

    end
  hold off
  grid on
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -2 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
    
catch; end
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,3), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['z meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,3), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['z pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,3), ...
                    's', 'Color', default_colors{3}, 'displayname', ['z est P']); catch; end

    end
  hold off
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end   
catch; end
%% figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,1),'bx', 'displayname', 'uav x slam'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
catch; end
%% figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,2),'bx', 'displayname', 'uav y slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
catch; end
%% figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,3),'bx', 'displayname', 'uav z slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')    
catch; end
%% figure(28); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(28); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.correction(:,1),'bx', 'displayname', 'uav yaw slam correction'); catch; end
    try plot(data.uavSLAM.time, data.uavSLAM.uav.aug.yaw.correction(:,1),'bx', 'displayname', 'uav yaw slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
catch; end
%% figure(51-65); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try     for i=1:length(data.uavSLAM.april.tagsDetected)
        figure(50+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
            hold on
            try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,1), ...
                     data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,2), ...
                     'o', 'displayname', data.uavSLAM.april.tagsDetected{i}); catch; end
            try plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
                     data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
                     '.', 'displayname', data.uavSLAM.april.tagsDetected{i} ); catch; end
            hold off; grid on
            title(['landmark estimated position, global frame ' [meta.date meta.run]])
            xlabel('X [m]'); ylabel('Y [m]')
            legend('toggle');legend('Location', 'SouthEast')

%             try %current axis
%                 current_limits = axis; current_axes = gca; 
%                 axes = [-2 2 -2 2];
%                 current_axes.XTick = [axes(1):0.5:axes(2)];
%                 current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%                 current_axes.YTick = [axes(3):0.5:axes(4)];
%                 current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%             end
            
    end
catch; end
%% figure(66-80); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    
try 
  for i=1:length(data.uavSLAM.april.tagsDetected)
        figure(65+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
            hold on
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
                    'o', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' x']);
            catch; end
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
                    'o', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' y']);
            catch; end
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,3), ...
                    'x', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' z']);
            catch; end
            hold off; grid on
            title(['landmark estimated position, global frame ' [meta.date meta.run]])
            xlabel('time [s]'); ylabel('X/Y/Z [m]')
            legend('toggle');legend('Location', 'SouthEast')

%             try %current axis
%                 current_limits = axis; current_axes = gca; 
%                 axes = [-2 2 -2 2];
%                 current_axes.XTick = [axes(1):0.5:axes(2)];
%                 current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%                 current_axes.YTick = [axes(3):0.5:axes(4)];
%                 current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%             end
            
    end
catch; end
%% figure(201); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(201); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  grid on
  hold on
    try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.EstPosition_gl(:,1), 'r.', 'displayname', 'est x pos wheel'); catch; end
    try plot(data.vicon.ugv1_.time, data.vicon.ugv1_.P.global(:,1), 'k.', 'displayname', 'vicon ugv1 x pos'); catch; end
    try plot(data.ugv1CKF.time, data.ugv1CKF.EstPosition_gl(:,1), 'bs', 'displayname', 'ckf ugv1 x pos'); catch; end
  hold off
  title(['ugv x position, global measurement frame ' [meta.date meta.run]])
  legend('toggle')
%   legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end   
catch; end
%% figure(202); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(202); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  grid on
  hold on
    try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.EstPosition_gl(:,2), 'r.', 'displayname', 'est y pos wheel'); catch; end
    try plot(data.vicon.ugv1_.time, data.vicon.ugv1_.P.global(:,2), 'k.', 'displayname', 'vicon ugv1 y pos'); catch; end
    try plot(data.ugv1CKF.time, data.ugv1CKF.EstPosition_gl(:,2), 'bs', 'displayname', 'ckf ugv1 y pos'); catch; end

  hold off
  title(['ugv y position, global measurement frame ' [meta.date meta.run]])
  legend('toggle')
%   legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end   
catch; end
%% figure(203); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(203); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  grid on
  hold on
    try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.EstPosition_gl(:,3), 'r.', 'displayname', 'est pos wheel'); catch; end
    try plot(data.vicon.ugv1_.time, data.vicon.ugv1_.P.global(:,3), 'k.', 'displayname', 'vicon ugv1 z pos'); catch; end
    try plot(data.ugv1CKF.time, data.ugv1CKF.EstPosition_gl(:,3), 'bs', 'displayname', 'ckf ugv1 z pos'); catch; end

  hold off
  title(['ugv z position, global measurement frame ' [meta.date meta.run]])
  legend('toggle')
%   legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end   
catch; end
%% figure(204); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try figure(204); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  grid on
  hold on
    try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.EstYaw_gl*pi/180, 'r.', 'displayname', 'est yaw wheel'); catch; end
    try plot(data.vicon.ugv1_.time, data.vicon.ugv1_.yaw.global, 'k.', 'displayname', 'vicon ugv1 yaw'); catch; end
    try plot(data.ugv1CKF.time, data.ugv1CKF.EstYaw_gl(:,1)*pi/180, 'bs', 'displayname', 'ckf ugv1 yaw'); catch; end

  hold off
  title(['ugv yaw position, global measurement frame ' [meta.date meta.run]])
%   legend([h1, h2, h3], 'Location', 'northwest')
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1 2];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end   
catch; end
%% figure(205); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(205); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstPosition_gl(:,1),'bs', 'displayname', 'ugv x CKF est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,1),'k.', 'displayname', 'ugv x gazebo global'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,1),'bx', 'displayname', 'uav x dkf init'); catch; end
    hold off; grid on
    title(['ugv2 x position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('x [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
%    
%     if meta.saveplots
%           meta.savefilename = ['uavSLAM_x_' meta.run];
%           try_save_plot(meta);
%     end
%% figure(206); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(206); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstPosition_gl(:,2),'bs', 'displayname', 'ugv2 y CKF est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,2),'k.', 'displayname', 'ugv2 y gazebo global'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,2),'bx', 'displayname', 'uav y dkf init'); catch; end
    hold off; grid on
    title(['ugv2 y position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('y [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
%    
%     if meta.saveplots
%           meta.savefilename = ['uavSLAM_x_' meta.run];
%           try_save_plot(meta);
%     end
%% figure(207); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(207); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstPosition_gl(:,3),'bs', 'displayname', 'ugv2 z CKF est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,3),'k.', 'displayname', 'ugv2 z gazebo global'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,3),'bx', 'displayname', 'uav z dkf init'); catch; end
    hold off; grid on
    title(['ugv2 z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
%    
%     if meta.saveplots
%           meta.savefilename = ['uavSLAM_x_' meta.run];
%           try_save_plot(meta);
%     end
%% figure(208); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(208); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstYaw_gl*(pi/180),'bs', 'displayname', 'ugv2 yaw CKF est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.yaw.global,'k.', 'displayname', 'ugv2 yaw gazebo global'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstYaw_gl*(pi/180),'bx', 'displayname', 'uav z dkf init'); catch; end
        
    hold off; grid on
    title(['ugv2 yaw position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('yaw [rad]')
    legend('toggle');legend('Location', 'SouthEast')
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       current_axes.XTick = [axes(1):5:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.5:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
%    
%     if meta.saveplots
%           meta.savefilename = ['uavSLAM_x_' meta.run];
%           try_save_plot(meta);
%     end


%% figure(209); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(209); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav x gazebo mocap'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,1),'r.', 'displayname', 'ugv2 x gazebo mocap'); catch; end
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,1),'bs', 'displayname', 'uav x slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,1),'gs', 'displayname', 'uav x oneCKF est'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.zk(:,1),'go', 'displayname', 'dkf zk x'); catch; end
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstPosition_gl(:,1),'bo', 'displayname', 'ugv x CKF est'); catch; end
    hold off; grid on
    title(['ugv2 x estimate and measurements' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('x [m]')
    legend('toggle');legend('Location', 'SouthEast')
%% figure(210); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(210); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav y gazebo mocap'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,2),'r.', 'displayname', 'ugv2 y gazebo mocap'); catch; end
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,2),'bs', 'displayname', 'uav y slam est'); catch; end
        try plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,2),'gs', 'displayname', 'uav y oneCKF est'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.zk(:,2),'go', 'displayname', 'dkf zk y'); catch; end
        try plot(data.ugv2CKF.time, data.ugv2CKF.EstPosition_gl(:,2),'bo', 'displayname', 'ugv y CKF est'); catch; end
    hold off; grid on
    title(['ugv2 y estimate and measurements' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('y [m]')
    legend('toggle');legend('Location', 'SouthEast')
%% figure(211); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(211); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time,   data.vicon.uav.P.global(:,3),'k.', 'displayname', 'uav z gazebo mocap'); catch; end
        try plot(data.uavSLAM.time,     data.uavSLAM.uav.est.p.global(:,3),'bs', 'displayname', 'uav z slam est'); catch; end
        try plot(data.oneCKF.time,      data.oneCKF.uav.EstPosition_gl(:,3),'gs', 'displayname', 'uav z oneCKF est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.P.global(:,3),'r.', 'displayname', 'ugv2 z gazebo mocap'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.zk(:,3),'go', 'displayname', 'dkf zk z'); catch; end
        try plot(data.ugv2CKF.time,     data.ugv2CKF.EstPosition_gl(:,3),'bo', 'displayname', 'ugv z CKF est'); catch; end
    hold off; grid on
    title(['ugv2 z estimate and measurements' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
%% figure(212); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(212); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time,   data.vicon.uav.yaw.global.degrees,'k.', 'displayname', 'uav yaw gazebo mocap'); catch; end
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.global*(180/pi),'bs', 'displayname', 'uav yaw slam est'); catch; end
        try plot(data.vicon.ugv2_.time, data.vicon.ugv2_.yaw.global*(180/pi),'r.', 'displayname', 'ugv2 yaw gazebo mocap'); catch; end
%         try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.Stereoyaw,'bx', 'displayname', 'Stereoyaw'); catch; end
        try plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.zk(:,4),'b.', 'displayname', 'dkf zk yaw'); catch; end
        try plot(data.oneCKF.time,      data.oneCKF.uav.Yaw,'gs', 'displayname', 'uav yaw oneCKF est'); catch; end
        try plot(data.ugv2CKF.time,     data.ugv2CKF.EstYaw_gl,'bo', 'displayname', 'ugv2 yaw CKF est'); catch; end
    hold off; grid on
    title(['ugv2 yaw estimate and measurements' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('yaw [deg]')
    legend('toggle');legend('Location', 'SouthEast')

end



function try_save_plot(meta)
        current_fig = gcf;
        disp("try save plot :")
        disp(["    " meta.saveplotroot '/figs/' meta.savefilename '.png'])
        try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
end

