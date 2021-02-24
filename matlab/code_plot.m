meta.date = '20200524/'; % testing all configs A-F
start = 100;
stop = 150;
meta.saveplots = true;
meta.exp_codes = {'A', 'B', 'C', 'D', 'E', 'F'};


for case_idx = 1:size(meta.exp_codes,2)
  %%
  case_idx = 6;
  switch meta.exp_codes{case_idx}
    case 'A'
      meta.fig_base_n = 1000;
    case 'B'
      meta.fig_base_n = 2000;
    case 'C'
      meta.fig_base_n = 3000;
    case 'D'
      meta.fig_base_n = 4000;
    case 'E'
      meta.fig_base_n = 5000;
    case 'F'
      meta.fig_base_n = 6000;
    otherwise
    	return
  end
  linewidth = 2;
  meta.exp_code = meta.exp_codes{case_idx};
  batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.mean_ckf  = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ckf);
  batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.mean_slam = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.slam);
  batch_data.(matlab.lang.makeValidName(meta.exp_code)).yaw_errors.mean_ckf   = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).yaw_errors.ckf);
  batch_data.(matlab.lang.makeValidName(meta.exp_code)).yaw_errors.mean_slam  = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).yaw_errors.slam);

  %%
  switch meta.exp_codes{case_idx}
    case {'A', 'B'}
  %% figure(X000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      figure(meta.fig_base_n); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            % batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error = max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error,max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist));
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
            % batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error = max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error, max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist));
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
    case {'C', 'D', 'E', 'F'}
  %% figure(X000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    figure(meta.fig_base_n); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
        hold on  
          h1 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
            'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
            'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
      current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances'); try printfig(meta, current_fig); end
  %% figure(X001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    figure(meta.fig_base_n+1); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
        hold on  
          h1 = plot( ...
              batch_data.ugv.refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
              batch_data.ugv.refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_capped_distances'); try printfig(meta, current_fig); end
  %% figure(X002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    figure(meta.fig_base_n+2); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('averaged error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
      hold on  
        h1 = plot(batch_data.ugv.refDist, batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.mean_ckf, ...
          'b', 'displayname', 'ckf mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
        h1 = plot(batch_data.ugv.refDist, batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.mean_slam, ...
          'r', 'displayname', 'slam mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
      hold off
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_average_errors_vs_capped_distances'); try printfig(meta, current_fig); end
  %% figure(X100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(meta.fig_base_n+100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        hold on  
          h1 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
            'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
            'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
        grid on
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
        current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
    otherwise
    	return
  end


end





if ~getpref('display', 'quiet')
    
    
    
  %% A figure(1000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'A';
      linewidth = 2;
      % batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error = 0;
      figure(1000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            % batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error = max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error,max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist));
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
            % batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error = max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_error, max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist));
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% A figure(1000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'A';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(1000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        hold on  
          h1 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
            'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
          legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
          h2 = plot( ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
            'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
          legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
        hold off
        grid on
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
        current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  %% B figure(2000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'B';
      linewidth = 3;
      figure(2000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% B figure(2000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'B';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(2000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
              'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
              'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
          grid on
          legend(legend_handles, legend_names); legend('location', 'NorthWest')
          axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
          current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  %% C figure(3000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'C';
      linewidth = 3;
      figure(3000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% C figure(3000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'C';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(3000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
          grid on
          legend(legend_handles, legend_names); legend('location', 'NorthWest')
          axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
          current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  %% D figure(4000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'D';
      linewidth = 3;
      figure(4000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% D figure(4000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'D';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(4000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
          grid on
          legend(legend_handles, legend_names); legend('location', 'NorthWest')
          axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
          current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  %% E figure(5000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'E';
      linewidth = 3;
      figure(5000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% E figure(5000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'E';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(5000+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
          grid on
          legend(legend_handles, legend_names); legend('location', 'NorthWest')
          axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
          current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  %% F figure(6000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      meta.exp_code = 'F';
      linewidth = 3;
      figure(6000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
        title(['Experiment ' meta.exp_code ' error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
        end
        legend(legend_handles, legend_names); legend('location', 'NorthWest')
        grid on
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');  printfig(meta, current_fig)
  %% F figure(6001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    linewidth = 2;
    figure(6001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
      title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [m]')
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
        hold on  
          h1 = plot( ...
              batch_data.ugv.refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p, ...
              'b', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(2,1) = h1; legend_names{2,1} = h1.DisplayName;
          h2 = plot( ...
              batch_data.ugv.refDist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p, ...
              'r', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
      end
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_capped_distances');  printfig(meta, current_fig)
  %% F figure(6100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number); 
    meta.exp_code = 'F';
    linewidth = 2;
    for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
      figure(6100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number); 
      title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [m]')
          hold on  
            h1 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
              'b', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
            h2 = plot( ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
              batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
              'r', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth); 
            legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          hold off
          grid on
          legend(legend_handles, legend_names); legend('location', 'NorthWest')
          axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
          current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');  printfig(meta, current_fig)
    end
  end