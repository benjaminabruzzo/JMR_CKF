function plotErrors(data, meta)  
%% figure(5000); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(5000); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title('ugv1/2 distance over time')
    hold on
      try plot(data.errors.time, data.errors.ugv1.dist,  'displayname', 'ugv1');catch; end
      try plot(data.errors.time, data.errors.ugv2.dist, 'displayname', 'ugv2');catch; end
    hold off
    legend('toggle')
    grid on
    xlabel('time [s]')
    ylabel('error [m]')  
  %% figure(5001); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(5001); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title(sprintf('ckf/slam rms error (%s %s)', meta.run, meta.exp_code))
    default_colors = getpref('display','default_colors');
    linewidth = 3;
    hold on
%       try plot(data.errors.time, 100*data.errors.uav.ckf.p_rms,      'Color', default_colors{3}, 'displayname', 'uav ckf',  'LineWidth', 1, 'Marker','.'); catch; end
%       try plot(data.errors.time, 100*data.errors.uav.slam.p_rms,     'Color', default_colors{1}, 'displayname', 'uav slam', 'LineWidth', 1, 'Marker','.'); catch; end
      try plot(data.errors.time, 100*data.errors.uav.ckf.p_rms,      'Color', default_colors{3}, 'displayname', 'uav ckf',  'LineStyle', 'none', 'Marker','.'); catch; end
      try plot(data.errors.time, 100*data.errors.uav.slam.p_rms,     'Color', default_colors{1}, 'displayname', 'uav slam', 'LineStyle', 'none', 'Marker','.'); catch; end
      try plot(data.errors.time, 100*data.errors.ugv1.ckf.p_rms_2D,  'r',                        'displayname', 'ugv1 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.time, 100*data.errors.ugv1.slam.p_rms_2D, 'b',                        'displayname', 'ugv1 slam', 'LineWidth', linewidth); catch; end
      try plot(data.errors.time, 100*data.errors.ugv2.ckf.p_rms_2D,  'Color', meta.colors{3},    'displayname', 'ugv2 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.time, 100*data.errors.ugv2.slam.p_rms_2D, 'Color', meta.colors{1},    'displayname', 'ugv2 slam', 'LineWidth', linewidth); catch; end
      
    hold off
%     axis([131 170 0 10])
    legend('toggle')
    grid on
    xlabel('time [s]')
    ylabel('error [cm]')
    current_fig.FileName = sprintf('%s%s', meta.saveplotroot, 'figs/01_ugvs_rms_ckf_vs_slam'); meta.saveplots = true;
%     printfig(meta, current_fig); meta.saveplots = false;
  %% figure(5002); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(5002); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title(sprintf('ckf/slam yaw error (%s %s)', meta.run, meta.exp_code))
    default_colors = getpref('display','default_colors');
    linewidth = 3;

    hold on
      try plot(data.errors.time, abs(data.errors.uav.ckf.yaw),      'Color', default_colors{3}, 'displayname', 'uav ckf',  'LineWidth', 1, 'Marker','.'); catch; end
      try plot(data.errors.time, abs(data.errors.uav.slam.yaw),     'Color', default_colors{1}, 'displayname', 'uav slam', 'LineWidth', 1, 'Marker','.'); catch; end
%       try plot(data.errors.time, data.errors.uav.ckf.yaw,      'Color', default_colors{3}, 'displayname', 'uav ckf',  'LineStyle', 'none', 'Marker','.'); catch; end
%       try plot(data.errors.time, data.errors.uav.slam.yaw,     'Color', default_colors{1}, 'displayname', 'uav slam', 'LineStyle', 'none', 'Marker','.'); catch; end

      try plot(data.errors.time, data.errors.ugv1.ckf.yaw,  'r',                        'displayname', 'ugv1 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.time, data.errors.ugv1.slam.yaw, 'b',                        'displayname', 'ugv1 slam', 'LineWidth', linewidth); catch; end
      try plot(data.errors.time, data.errors.ugv2.ckf.yaw,  'Color', meta.colors{3},    'displayname', 'ugv2 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.time, data.errors.ugv2.slam.yaw, 'Color', meta.colors{1},    'displayname', 'ugv2 slam', 'LineWidth', linewidth); catch; end
    hold off
    axis([100 180 -0.2 0.2])
    legend('toggle')
    grid on
    xlabel('time [s]')
    ylabel('error [rad]')
    current_fig.FileName = sprintf('%s%s', meta.saveplotroot, 'figs/01_ugvs_yaw_ckf_vs_slam'); meta.saveplots = true;
%     printfig(meta, current_fig); meta.saveplots = false;

  %% figure(5003); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(5003); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title('ugv ckf/slam error (RMS)')
    linewidth = 3;
    hold on
      try plot(data.errors.ugv2.dist, 100*data.errors.ugv2.ckf.p_v_dist,  'r', 'displayname', 'ugv2 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.ugv2.dist, 100*data.errors.ugv2.slam.p_v_dist, 'b', 'displayname', 'ugv2 slam', 'LineWidth', linewidth); catch; end
      try plot(data.errors.ugv1.dist, 100*data.errors.ugv1.ckf.p_v_dist,  'r', 'displayname', 'ugv1 ckf',  'LineWidth', linewidth); catch; end
      try plot(data.errors.ugv1.dist, 100*data.errors.ugv1.slam.p_v_dist, 'b', 'displayname', 'ugv1 slam', 'LineWidth', linewidth); catch; end
    hold off
    axis([0 4.5 0 25])
    legend('toggle')
    grid on
    xlabel('distance traveled [m]')
    ylabel('error [cm]')
  %% figure(5004); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(5004); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title('ugv ckf/slam yaw error (RMS)')
    hold on
      try plot(data.errors.ugv2.dist, data.errors.ugv2.ckf.yaw_v_dist,  'displayname', 'ugv2 ckf');catch; end
      try plot(data.errors.ugv2.dist, data.errors.ugv2.slam.yaw_v_dist, 'displayname', 'ugv2 slam');catch; end
      try plot(data.errors.ugv1.dist, data.errors.ugv1.ckf.yaw_v_dist,  'displayname', 'ugv1 ckf');catch; end
      try plot(data.errors.ugv1.dist, data.errors.ugv1.slam.yaw_v_dist, 'displayname', 'ugv1 slam');catch; end
    hold off
    legend('toggle')
    grid on
    xlabel('distance traveled [m]')
    ylabel('error [rad]')
  %% figure(7000); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
  figure(7000); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    legend_handles = []; legend_names = {}; legend_idx=1;
    linewidth = 1.5;
    ax1 = gca; ax1.XColor = 'r';ax1.YColor = 'r'; % current axes
    ax1_pos = ax1.Position; % position of first axes
    ax1.XLabel.String = 'time [s]'; ax1.YLabel.String = 'error [cm]';

    ax2 = axes('Position',ax1_pos,...
        'XAxisLocation','top',...
        'YAxisLocation','right',...
        'Color','none');
    ax2.XLabel.String = 'time [s]'; ax2.YLabel.String = 'actual x position [m]';
    
    title('uav ckf/slam error ')
    grid on
    hold on
    try L1 = line(data.errors.clip.time, ...
        100*data.errors.clip.uav.ckf.p.rms+...
        100*data.errors.clip.ugv1.ckf.p.rms+...
        100*data.errors.clip.ugv2.ckf.p.rms,...
        'Parent',ax1, ...
        'LineWidth', linewidth, ...
        'displayname', 'uav ckf rms',...
        'LineStyle', '-',...
        'Color',default_colors{1}); 
        legend_names{legend_idx,1} = L1.DisplayName; legend_idx = legend_idx +1;
    catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try L2 = line(data.errors.clip.time, ...
      100*data.errors.clip.uav.slam.p.rms+...
      100*data.errors.clip.ugv1.slam.p.rms+...
      100*data.errors.clip.ugv2.slam.p.rms,...
        'Parent',ax1, ...
        'LineWidth', linewidth, ...
        'displayname', 'uav slam rms',...
        'LineStyle', '-',...
        'Color',default_colors{2}); 
        legend_names{legend_idx,1} = L2.DisplayName; legend_idx = legend_idx +1;
    catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try L3 = line(data.errors.clip.time, data.errors.clip.uav.p.gl,...
        'Parent',ax2, ...
        'LineWidth', linewidth, ...
        'displayname', 'uav p (x)',...
        'LineStyle', '-',...
        'Color',default_colors{3}); 
        legend_names{legend_idx,1} = L3.DisplayName; legend_idx = legend_idx +1;
    catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    hold off
try  legend(ax2, legend_names,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%     try plot(data.errors.clip.time, data.errors.clip.uav.p.gl,  'displayname', 'uav p (x)');catch; end
%       try plot(data.errors.clip.time, data.errors.clip.uav.ckf.p.rms, 'displayname', 'uav ckf rms');catch; end
%       try plot(data.errors.clip.time, data.errors.clip.uav.slam.p.rms, 'displayname', 'uav slam rms');catch; end
    hold off
    legend('toggle')
    grid on
    xlabel('time [s]')
    ylabel('error [m]')
end


function printfig(meta, fig_handle)
  if (meta.saveplots)
      fig_handle.PaperUnits = 'inches';
      fig_handle.PaperPosition = [0 0 15 5];
      disp(['       ' fig_handle.FileName '.png'])
%       if (~exist([fig_handle.FileName '.eps'])); print(fig_handle,  [fig_handle.FileName '.eps'],'-depsc', '-r500'); end
      if (~exist([fig_handle.FileName '.png'])); print(fig_handle,  [fig_handle.FileName '.png'],'-dpng' , '-r500');  end
  end
end
