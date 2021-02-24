function batchslam_1604(date_in, start, stop)
%% init
  meta.root = '/home/benjamin/ros/src/metahast/matlab/'; addpath(genpath([meta.root]));
  setpref('display','quiet', true); 
  warning('off','MATLAB:legend:PlotEmpty')
  meta.cutoff = 1; % cutof for a 'good' trial

%% define meta data
  switch nargin
    case 3
      meta.date = [date_in '/'];
    otherwise
%       meta.date = '20200511/'; % imperfect DKF estimate
%       meta.date = '20200513/'; % perfect DKF estimate
%       meta.date = '20200518/'; % perfect DKF estimate, ugv1 looking at ugv2
      meta.date = '20200521/'; % ugv1 looking at ugv2 in motion
%       meta.date = '20200524/'; % testing all configs A-F
      start = 100;
      stop = 109;
      meta.saveplots = true;
      meta.exp_codes = {'A', 'B', 'C', 'D', 'E', 'F'};
  end

  try 
    try 
      meta.dataroot = '/home/benjamin/ros/data/'; cd ([meta.dataroot meta.date meta.exp_code '/'])
      meta.datedata = dir(['/home/benjamin/ros/data/' meta.date]);
    catch
      meta.dataroot = '/media/benjamin/archive/'; cd ([meta.dataroot meta.date meta.exp_code '/'])
      meta.datedata = dir(['/media/benjamin/archive/' meta.date]);
%       meta.dataroot = '/media/benjamin/data/'; cd ([meta.dataroot meta.date meta.exp_code '/'])
%       meta.datedata = dir(['/media/benjamin/data/' meta.date]);
    end
  catch
    disp('Could not find data')
    return
  end
  
%   size(meta.exp_codes,2)
  
  meta.saveplotroot = [meta.dataroot meta.date 'figs/'];
  if ~exist(meta.saveplotroot, 'dir'); mkdir(meta.saveplotroot); end
  if ~exist([meta.saveplotroot 'e_vs_d'] , 'dir'); mkdir([meta.saveplotroot 'e_vs_d']); end

  %% actually load in data, either m or mat files
  batch_data.jstr = {};
  meta.exp_codes = {'D'};
  for run = start:stop
    meta.run = sprintf('%03d',run);
    
    try
      clear trial_data
      trial_data = loadJointSlamData(meta); 
      trial_data = calculateErrors(meta, trial_data);
      setpref('display','quiet', false); 
%       qdisp(sprintf('ugv1.net_displacement: %4.6f',trial_data.errors.ugv1.net_displacement));
%       qdisp(sprintf('ugv2.net_displacement: %4.6f',trial_data.errors.ugv2.net_displacement));
      setpref('display','quiet', true); 
      if or((trial_data.errors.ugv1.net_displacement>meta.cutoff), (trial_data.errors.ugv2.net_displacement>meta.cutoff))
        batch_data.jstr = [batch_data.jstr; meta.run];
      end
    catch; end
  end
  
  
  s_good_trials = sprintf('good trials (%i): ', size(batch_data.jstr,1));
  for idx = 1:size(batch_data.jstr,1)
    s_good_trials = sprintf('%s %s,', s_good_trials, batch_data.jstr{idx});
  end; clear idx; disp(s_good_trials)

  %% now that I have a list of good trials, i can double back and process just those datasets
  for idx = 1:length(batch_data.jstr)
    mat_file = [meta.dataroot meta.date batch_data.jstr{idx} '/errors_' batch_data.jstr{idx} '.mat'];
    qdisp(mat_file);  load(mat_file);
    batch_data.ugv1.enddist(idx,1) = errors.ugv1.dist(end);
    batch_data.ugv2.enddist(idx,1) = errors.ugv2.dist(end);
    batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))) = errors;
  end

    
  %% I still need to spline along the distance vector
  % here i want to calculate the error vs distance at fixed distances for both ugvs
  % dist_ref is the distances that I want to spline the errors to for plotting
  % set common distances for comparing perfomance
  
  batch_data.ugv2.refDist = 0:0.0001:min(batch_data.ugv2.enddist);
  
  for idx = 1:length(batch_data.jstr)
    referenceVector   = batch_data.ugv2.refDist; % is not important
    timeofvector      = batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist; % distance traveled
    % Slam data
      % slam position:
      vectorToBeSplined = batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.p_v_dist; % error data
      batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv2.refDist);
      % slam yaw
      vectorToBeSplined = batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.yaw_v_dist; % error data
      batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv2.refDist);
      
    % CKF data
      % ckf position
      vectorToBeSplined = batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.p_v_dist; % error data
      batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv2.refDist);
      % ckf yaw
      vectorToBeSplined = batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.yaw_v_dist; % error data
      batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv2.refDist);
      
    % add to batch for calculating mean
    batch_data.mean_errors.ugv2.slam(idx,:)= batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.p;
    batch_data.mean_errors.ugv2.ckf(idx,:) = batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.p;
    batch_data.yaw_errors.ugv2.slam(idx,:) = batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.yaw;
    batch_data.yaw_errors.ugv2.ckf(idx,:)  = batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.yaw;
  end
  
  batch_data.mean_errors.ugv2.mean_ckf  = mean(batch_data.mean_errors.ugv2.ckf);
  batch_data.mean_errors.ugv2.mean_slam = mean(batch_data.mean_errors.ugv2.slam);
  batch_data.yaw_errors.ugv2.mean_ckf   = mean(batch_data.yaw_errors.ugv2.ckf);
  batch_data.yaw_errors.ugv2.mean_slam  = mean(batch_data.yaw_errors.ugv2.slam);
  
  setpref('display','quiet', false); 
  if ~getpref('display', 'quiet')
    %% figure(5000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 01_all_errors_vs_all_distances
    linewidth = 3;
    figure(5000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title('error vs all distances traveled by ugv2'); xlabel('distance traveled [m]'); ylabel('error [m]')
      for idx = 1:length(batch_data.jstr)
        hold on  
          h1 = plot( ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.p_v_dist, ...
            'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.p_v_dist, ...
            'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '01_all_errors_vs_all_distances'); 
      printfig(meta, current_fig)
      
    %% figure(5001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 02_all_errors_vs_capped_distance
    linewidth = 3;
    figure(5001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
      for idx = 1:length(batch_data.jstr)
        hold on  
          h1 = plot(batch_data.ugv2.refDist, ...
                    batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.p, ...
                    'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot(batch_data.ugv2.refDist, ...
                    batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.p, ...
                    'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '02_all_errors_vs_capped_distance'); 
      printfig(meta, current_fig)

      
    %% figure(5002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 03_average_error_vs_capped_distance
    linewidth = 3;
    figure(5002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('averaged error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
      hold on  
        h1 = plot(batch_data.ugv2.refDist, batch_data.mean_errors.ugv2.mean_ckf, 'b', 'displayname', 'ckf mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
        h1 = plot(batch_data.ugv2.refDist, batch_data.mean_errors.ugv2.mean_slam, 'r', 'displayname', 'slam mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
      hold off
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '03_average_error_vs_capped_distance'); 
      printfig(meta, current_fig)

    %% figure(5100+idx); clf; current_fig = gcf; %fprintf('figure(%d)\n', current_fig.Number);   
    max_dist = 0;
    max_error = 0;
    for idx = 1:length(batch_data.jstr)  
      max_error = max([max_error; ...
        max(batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.p_v_dist);...
        max(batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.p_v_dist)]);
      max_dist = max([max_dist; max(batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist)]);
        
    end; clear idx

    for idx = 1:length(batch_data.jstr)  
    figure(5100+idx); clf; current_fig = gcf; %fprintf('figure(%d)\n', current_fig.Number);   
        title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
        hold on  
          h1 = plot(...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.p_v_dist, ...
                'b', 'displayname', sprintf('ckf %s', batch_data.jstr{idx} ));
          h2 = plot(...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.p_v_dist, ...
                'r', 'displayname', sprintf('slam %s', batch_data.jstr{idx} ));
        hold off
      legend('toggle'); legend('location', 'NorthWest')
      grid on
      

%       try %current axis
        current_limits = axis; current_axes = gca; 
        axes = [0 round(max_dist+0.2,1) 0 round(max_error+0.1,2)];
        current_axes.XTick = [axes(1):0.2:axes(2)];
        current_axes.XLim = [axes(1) current_axes.XTick(end)];
        current_axes.YTick = [axes(3):0.05:axes(4)];
        current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%       end
      
      current_fig.FileName = sprintf('%s%s/%s_%s', meta.saveplotroot, 'e_vs_d', 'pos_error_vs_dist', batch_data.jstr{idx}); printfig(meta, current_fig);
    end
    %% figure(6000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 02_all_yaw_errors_vs_all_distances
    figure(6000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('yaw error vs all distances traveled by ugv2'); xlabel('distance traveled [m]'); ylabel('error [rad]')
      for idx = 1:length(batch_data.jstr)
        hold on  
          h1 = plot( ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.ckf.yaw_v_dist, ...
            'b', 'displayname', 'ckf'); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.jstr{idx} ))).ugv2.slam.yaw_v_dist, ...
            'r', 'displayname', 'slam'); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '04_yaw_errors_vs_all_distances'); 
      printfig(meta, current_fig)
    %% figure(6001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 02_all_errors_vs_capped_distance
    figure(6001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [rad]')
      for idx = 1:length(batch_data.jstr)
        hold on  
          h1 = plot(batch_data.ugv2.refDist, ...
                    batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.yaw, ...
                    'b', 'displayname', 'ckf'); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot(batch_data.ugv2.refDist, ...
                    batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.yaw, ...
                    'r', 'displayname', 'slam'); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '05_yaw_errors_vs_capped_distances'); 
      printfig(meta, current_fig)
    %% figure(6002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    % 03_average_error_vs_capped_distance
    figure(6002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('averaged error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [rad]')
      hold on  
        h1 = plot(batch_data.ugv2.refDist, batch_data.yaw_errors.ugv2.mean_ckf, 'b', 'displayname', 'ckf mean', 'LineWidth', 2.0 ); 
          legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
        h1 = plot(batch_data.ugv2.refDist, batch_data.yaw_errors.ugv2.mean_slam, 'r', 'displayname', 'slam mean', 'LineWidth', 2.0 ); 
          legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
      hold off
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '06_yaw_average_errors_vs_capped_distances'); 
      printfig(meta, current_fig)
    %% figure(6100+idx); clf; current_fig = gcf; %fprintf('figure(%d)\n', current_fig.Number);   
%     for idx = 1:length(batch_data.jstr)  
%     figure(6100+idx); clf; current_fig = gcf; %fprintf('figure(%d)\n', current_fig.Number);   
%         title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [rad]')
%         hold on  
%           h1 = plot( batch_data.ugv2.refDist, batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).ckf.yaw, ...
%                 'b', 'displayname', sprintf('ckf %s', batch_data.jstr{idx} ));
%           h2 = plot( batch_data.ugv2.refDist, batch_data.ugv2.(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.jstr{idx} ))).slam.yaw, ...
%                 'r', 'displayname', sprintf('slam %s', batch_data.jstr{idx} ));
%         hold off
%       legend('toggle')
%       grid on
%     end
  end

  setpref('display','quiet', true); 
  
end

      function printfig(meta, handle)
        if (meta.saveplots && ~exist([handle.FileName '.png']))
            handle.PaperUnits = 'inches';
            handle.PaperPosition = [0 0 15 8];
            disp(['       ' handle.FileName '.eps'])
%             print(handle,  [handle.FileName '.eps'],'-depsc', '-r500'); 
            print(handle,  [handle.FileName '.png'],'-dpng', '-r500'); 
        end
      end

