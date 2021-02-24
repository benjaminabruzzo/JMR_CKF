
			ugv.data.writeDouble(uav.EstYaw_gl,						"  " + ugv.s_data_filename + ".oneCKF.uav.Yaw", oneCKF.counter);
			ugv.data.writeDouble(uav.EstYawBias,					"  " + ugv.s_data_filename + ".oneCKF.uav.YawBias", oneCKF.counter);

      data
      
      
%%figure(22); clf 

linewidth = 1.5;


ax1 = gca; ax1.XColor = 'r';ax1.YColor = 'r'; % current axes
ax1_pos = ax1.Position; % position of first axes
ax1.XLabel.String = 'ugv yaw distance traveled [rad]'; ax1.YLabel.String = 'ugv yaw error [rad]';

ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','top',...
    'YAxisLocation','right',...
    'Color','none');
ax2.XLabel.String = 'uav yaw distance traveled [rad]'; ax2.YLabel.String = 'uav yaw error [rad]';


try L1 = line(data.errors.ugv2.angular_dist, data.errors.ugv2.ckf.yaw_v_dist,...
    'displayname', 'ugv2 ckf yaw',...
    'LineWidth', linewidth, ...
    'Color',default_colors{1}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
try L2 = line(data.errors.ugv2.angular_dist, data.errors.ugv2.slam.yaw_v_dist,...
    'displayname', 'ugv2 slam yaw',...
    'LineWidth', linewidth, ...
    'Color',default_colors{2}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

try L3 = line(data.errors.uav.angular_dist, data.errors.uav.ckf.yaw_v_dist,...
    'Parent',ax2, ...
    'LineWidth', linewidth, ...
    'displayname', 'uav ckf yaw',...
    'LineStyle', '--',...
    'Color',default_colors{1}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
try L4 = line(data.errors.uav.angular_dist, data.errors.uav.slam.yaw_v_dist,...
    'Parent',ax2, ...
    'LineWidth', linewidth, ...
    'displayname', 'uav slam yaw',...
    'LineStyle', '--',...
    'Color',default_colors{2}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
grid on

legend(ax2, {...
  L1.DisplayName,...
  L2.DisplayName,...
  L3.DisplayName,...
  L3.DisplayName},'Location','northwest','NumColumns',2); legend('boxoff')


ax2.XTick = linspace(ax2.XLim(1),ax2.XLim(2),size(ax1.XTick,2));
ax2.YTick = linspace(ax2.YLim(1),ax2.YLim(2),size(ax1.YTick,2));



%%
% for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
%         disp(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx})
%         current_fig = gcf;
%         title(['Experiment ' meta.exp_code ', Map ' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1) ', error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [cm]')
%         hold on
%           switch meta.exp_code
%             case {'A', 'B'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.ckf.p_v_dist, ...
%                 'b', 'displayname', 'ckf', 'LineWidth', linewidth); 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1; 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.slam.p_v_dist, ...
%                 'r', 'displayname', 'slam', 'LineWidth', linewidth); 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2; 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%             case {'C', 'D', 'E', 'F'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.ckf.p_v_dist, ...
%                 'b', 'displayname', 'ckf', 'LineWidth', linewidth); 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1; 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} )))
%                   .ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.slam.p_v_dist, ...
%                 'r', 'displayname', 'slam', 'LineWidth', linewidth); 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2; 
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%           end
%           hold off
%           grid on
%           legend(meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
%                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names); 
%           legend('location', 'NorthWest')
% end

 %% find the reference ugv trajectories for each map type
 clc
 % find the reference ugv trajectories for each map type
  % For each experiment type
  for code_idx = 1:size(meta.exp_codes,2)
    meta.exp_code = meta.exp_codes{code_idx};
    for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDist = 1000;
    end
    % For each trial, switch between map types
    for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      try
        switch meta.exp_code
          case {'A', 'B'}
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).refDist = ...
              min([batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist(end)]);
          case {'C', 'D', 'E', 'F'}
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).refDist = ...
              min([batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist(end)]);
        end
      catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    end

    % now, spline errors to map trajectories
    for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec = ...
        0:0.01:batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDist;

      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.ckf = [];
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.slam = [];
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.yaw_ckf = [];
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.yaw_slam = [];
    end
    
    for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      refDistVec = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).refDistVec; % is not important
      try
        switch meta.exp_code
          case {'A', 'B'}
            ugvDistActal = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist;
            % slam position:
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.slam.p_v_dist;
            slam_p = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.slam = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.slam,slam_p];
            % slam yaw
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.slam.yaw_v_dist;
            yaw_slam = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_slam = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_slam, yaw_slam];

            % ckf position:
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.ckf.p_v_dist;
            ckf_p = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.ckf = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.ckf,ckf_p];
            % ckf yaw
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.ckf.yaw_v_dist;
            yaw_ckf = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_ckf = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_ckf, yaw_ckf];

          case {'C', 'D', 'E', 'F'}
            ugvDistActal = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist;
            % slam position:
            ugvError =     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.slam.p_v_dist;
            slam_p = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.slam = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.slam,slam_p];
            % slam yaw
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.slam.yaw_v_dist;
            yaw_slam = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_slam = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_slam, yaw_slam];

            % ckf position:
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.ckf.p_v_dist;
            ckf_p = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.ckf = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.ckf,ckf_p];
            % ckf yaw
            ugvError = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.ckf.yaw_v_dist;
            yaw_ckf = nD_interp1(ugvDistActal, ugvError, refDistVec);
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_ckf = ...
              [batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1)))).mean_errors.yaw_ckf, yaw_ckf];

        end
      catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    end
    
      for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec = ...
        0:0.01:batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDist;

      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_ckf = ...
        mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.ckf, 2);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_slam = ...
        mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.slam, 2);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_yaw_ckf = ...
        mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.yaw_ckf, 2);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_yaw_slam = ...
        mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.yaw_slam, 2);
    end
  
  end
  
  
%   plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec, ...
%     100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_ckf)
  

  
%% spline to minimum distance for each map
    
    for idx = 1:length(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr)
      qdisp(['trail idx: ' num2str(idx) ', trial: ' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}])
%       switch meta.exp_code
%         case {'A', 'B'}
%           timeofvector = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist;
%           % slam position:
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
%             batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist;
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p = ...
%             nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % slam yaw
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
%             batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.yaw_v_dist;
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % ckf position
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist; % error data
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % ckf yaw
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.yaw_v_dist; % error data
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%         case {'C', 'D', 'E', 'F'}
%           timeofvector = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist;
%           % slam position:
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist;
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % slam yaw
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.yaw_v_dist; 
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % ckf position
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist; % error data
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
%           % ckf yaw
%           vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.yaw_v_dist; % error data
%           batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
      end