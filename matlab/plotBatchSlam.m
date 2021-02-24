function [data, meta] = plotBatchSlam(data, meta)
%% plot settings
  warning('off','images:imshow:magnificationMustBeFitForDockedFigure')
  linespec = {'d'; '<'; '>'; '^'; 'V'};
  % disp('Plotting...')

  % Matlab Default Colors
  meta.colors{1} = [0, 0.4470, 0.7410];
  meta.colors{2} = [0.8500, 0.3250, 0.0980];
  meta.colors{3} = [0.9290, 0.6940, 0.1250];
  meta.colors{4} = [0.4940, 0.1840, 0.5560];
  meta.colors{5} = [0.4660, 0.6740, 0.1880];
  meta.colors{6} = [0.3010, 0.7450, 0.9330];
  meta.colors{7} = [0.6350, 0.0780, 0.1840];
%% post process some things
  % make the fullstate slam into a plot-able matrix
  try
    data.fullSLAM.states = matrixFromCellData(data.fullSLAM.slam.x);
  end

%% 'top_down'
% f = figure('visible', 'off', 'FileName', 'top_down'); clf; current_fig = gcf; 
current_fig = figure('visible', 'off', 'FileName', 'top_down'); % fprintf('figure(%s)\n', current_fig.FileName); 
  current_fig.PaperUnits = 'inches';
  current_fig.PaperPosition = [0 0 15 8];
  current_fig.UserData = sprintf('%s%s_%s_%s.png',meta.saveplotroot, current_fig.FileName, meta.date(1:end-1), meta.run); 

  hold on; 
    ylabel('y [m]')
    xlabel('x [m]')
    title([meta.date meta.run])
    
    % ugv1
    try text(data.vicon.ugv1.position.gl(end,1), data.vicon.ugv1.position.gl(end,2), 'ugv1', 'FontSize',8); catch; end
    try plot(data.vicon.ugv1.position.gl(:,1), data.vicon.ugv1.position.gl(:,2), 'k.'); catch; end
    try plot(data.ugv1_mux.est.Position_gl(:,1), data.ugv1_mux.est.Position_gl(:,2), '.'); catch; end

    %ugv2
    try text(data.vicon.ugv2.position.gl(end,1), data.vicon.ugv2.position.gl(end,2), 'ugv2', 'FontSize',8); catch; end
    try plot(data.vicon.ugv2.position.gl(:,1), data.vicon.ugv2.position.gl(:,2), 'k.'); catch; end
    try plot(data.ugv2_mux.est.Position_gl(:,1), data.ugv2_mux.est.Position_gl(:,2), '.'); catch; end
    try plot(data.ugv2_mux.est.Position_gl(:,1), data.ugv2_mux.est.Position_gl(:,2), 'b.', 'displayname', ['ugv2 slam (+odom)']);catch; end

    % april tags
    for i = 0:12 % vicon
      apr_str = sprintf( 'april%02d', i );
      try 
        xt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,1);
        yt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,2);
        if (xt ~= 0 && yt ~= 0)
          plot( xt, yt, 'k*', 'displayname', apr_str);
        end
      catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,1), ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,2), ...
                '.', 'displayname', data.fullSLAM.april.names{i}); catch; end
    end

    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [-1.5 3.5 -2.0 1.5];
      current_axes.XTick = [axes(1):0.5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
  
    
  hold off
  grid on

disp(['      Saving ' current_fig.UserData])
print(current_fig, current_fig.UserData,'-dpng', '-r500')
close(current_fig)
  
end

