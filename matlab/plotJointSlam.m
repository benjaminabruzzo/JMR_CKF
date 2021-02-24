function [data, meta] = plotfullSLAM(data, meta)
%% Turn plotting back on
linespec = {'d'; '<'; '>'; '^'; 'V'};
disp('Plotting...')
set(0, 'DefaultFigureVisible', 'on');
warning('off','images:imshow:magnificationMustBeFitForDockedFigure')
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on');
clear figHandles
data.printplots = [];



% Matlab Default Colors
default_colors = getpref('display','default_colors');
meta.colors{1} = [0, 0.4470, 0.7410];
meta.colors{2} = [0.8500, 0.3250, 0.0980];
meta.colors{3} = [0.9290, 0.6940, 0.1250];
meta.colors{4} = [0.4940, 0.1840, 0.5560];
meta.colors{5} = [0.4660, 0.6740, 0.1880];
meta.colors{6} = [0.3010, 0.7450, 0.9330];
meta.colors{7} = [0.6350, 0.0780, 0.1840];
%% Set apriltags lists
  try vicon_bodies.fieldnames = fieldnames(data.vicon); catch; end
  vicon_bodies.aprilcount = 1;
  for i = 1:length(vicon_bodies.fieldnames)
    try 
      if strcmp(vicon_bodies.fieldnames{i}(1:5), 'april')
        vicon_bodies.aprilnames{vicon_bodies.aprilcount,1} = vicon_bodies.fieldnames{i}(1:7);
        vicon_bodies.aprilcount = vicon_bodies.aprilcount+1;
      end
    catch; end; end
  data.vicon_bodies = vicon_bodies;

  try data.fullSLAM.april.fieldnames = fieldnames(data.fullSLAM.april); catch; end
  data.fullSLAM.april.count = 1;
try
  for i = 1:length(data.fullSLAM.april.fieldnames)
    try 
      if strcmp(data.fullSLAM.april.fieldnames{i}(1:3), 'tag')
        try
          data.fullSLAM.april.names{data.fullSLAM.april.count,1} = data.fullSLAM.april.fieldnames{i}(1:6);
          data.fullSLAM.april.count = data.fullSLAM.april.count+1;
        catch; end
      end
    catch; end; end
end

  try data.uavSLAM.april.fieldnames = fieldnames(data.uavSLAM.april); catch; end
  try 
      data.uavSLAM.april.count = 1;
      for i = 1:length(data.uavSLAM.april.fieldnames)
        try 
          if strcmp(data.uavSLAM.april.fieldnames{i}(1:3), 'tag')
            try
              data.uavSLAM.april.names{data.uavSLAM.april.count,1} = data.uavSLAM.april.fieldnames{i}(1:6);
              data.uavSLAM.april.count = data.uavSLAM.april.count+1;
            catch; end
          end
        catch; end
      end
    catch; end
%% post process some things
  % make the fullstate slam into a plot-able matrix
try
	data.fullSLAM.states = matrixFromCellData(data.fullSLAM.slam.augmentedState);
end

% backfill ugv1 vicon positions using ugv2 locations?
  try 
    vic_idx = 1;
    while (sum(data.vicon.ugv2.position.gl(vic_idx,:))==0)
      vic_idx = vic_idx +1;
    end; clear vic_idx
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

  try data.uav_ckf.ckf.PosteriorEst_mat = (matrixFromCellData(data.uav_ckf.ckf.PosteriorEst))'; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try data.ugv1_ckf.ckf.PosteriorEst_mat = (matrixFromCellData(data.ugv1_ckf.ckf.PosteriorEst))'; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try data.ugv2_ckf.ckf.PosteriorEst_mat = (matrixFromCellData(data.ugv2_ckf.ckf.PosteriorEst))'; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end


  try data.vicon.ugv2.yaw.ugv_frame = data.vicon.ugv2.yaw.gl - data.vicon.ugv1.yaw.gl; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try data.vicon.ugv1.yaw.ugv_frame = data.vicon.ugv1.yaw.gl - data.vicon.ugv2.yaw.gl; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

%% Calc eigen values

try data.fullSLAM.eig(:,1) = eig(data.fullSLAM.augmentedCovariance{1,1});
  for idx = 2:length(data.fullSLAM.augmentedCovariance)
    full_eig = eig(data.fullSLAM.augmentedCovariance{idx,1});

    if (length(full_eig)~=size(data.fullSLAM.eig,1))
      % then a block or column of zeros is needed figure out where
      add_zeros = abs(size(data.fullSLAM.eig,1)-(length(full_eig)));
      if (length(full_eig)<size(data.fullSLAM.eig,1))
        % then add to column of new eigen values
          full_eig = [full_eig; zeros(add_zeros,1)];
      else
        % then there are more eigen values now than all previous times  add zeros to the bottom of previous matrix
					data.fullSLAM.eig = [data.fullSLAM.eig; zeros(add_zeros, idx-1)];
      end
    end
    % add eigen values to end of matrix
      data.fullSLAM.eig(:, idx) = full_eig;
	end
end; clear idx april_eig add_zeros


%% yy plot:: figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  figure(500); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  linewidth = 1.5;
  ax1 = gca; ax1.XColor = 'r';ax1.YColor = 'r'; % current axes
  ax1_pos = ax1.Position; % position of first axes
  ax1.XLabel.String = 'distance traveled [m]'; ax1.YLabel.String = 'error [cm]';

  ax2 = axes('Position',ax1_pos,...
      'XAxisLocation','top',...
      'YAxisLocation','right',...
      'Color','none');
  ax2.XLabel.String = 'angular distance traveled [rad]'; ax2.YLabel.String = 'angular error [rad]';


  try L1 = line(data.errors.ugv2.dist, 100*data.errors.ugv2.ckf.p_rms_2D,...
      'Parent',ax1, ...
      'LineWidth', linewidth, ...
      'displayname', 'ugv2 ckf p rms',...
      'LineStyle', '-',...
      'Color',default_colors{1}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try L2 = line(data.errors.ugv2.dist, 100*data.errors.ugv2.slam.p_rms_2D,...
      'Parent',ax1, ...
      'LineWidth', linewidth, ...
      'displayname', 'ugv2 slam p rms',...
      'LineStyle', '-',...
      'Color',default_colors{2}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

  try L3 = line(data.errors.ugv2.angular_dist, data.errors.ugv2.ckf.yaw_v_dist,...
      'Parent',ax2, ...
      'LineWidth', linewidth, ...
      'displayname', 'ugv2 ckf yaw',...
      'LineStyle', '--',...
      'Color',default_colors{1}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try L4 = line(data.errors.ugv2.angular_dist, data.errors.ugv2.slam.yaw_v_dist,...
      'Parent',ax2, ...
      'LineWidth', linewidth, ...
      'displayname', 'ugv2 slam yaw',...
      'LineStyle', '--',...
      'Color',default_colors{2}); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  grid on

  
try  legend(ax2, {...
    L1.DisplayName,...
    L2.DisplayName,...
    L3.DisplayName,...
    L4.DisplayName},'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end


%   ax2.XTick = linspace(ax2.XLim(1),ax2.XLim(2),size(ax1.XTick,2));
%   ax2.YTick = linspace(ax2.YLim(1),ax2.YLim(2),size(ax1.YTick,2));
%% figure(503); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(503); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
try 
  for idx =  1:size(data.ugv2_ckf.ckf.PosteriorCov,1)
    data.ugv2_ckf.ckf.PosteriorEig(:,idx) = eig(data.ugv2_ckf.ckf.PosteriorCov{idx});
  end
  title([meta.date meta.run '  ugv2 eig']); ylabel('eig'); xlabel('time [s]'); grid on
  hold on; 
    linewidth = 3; ii = 0; legend_cells = {}; 
    h = plot(data.ugv2_ckf.ckf.time,  data.ugv2_ckf.ckf.PosteriorEig, 'k.'); 
  hold off;
 catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%% figure(502); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(502); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  try 
    for idx =  1:size(data.ugv1_ckf.ckf.PosteriorCov,1)
      data.ugv1_ckf.ckf.PosteriorEig(:,idx) = eig(data.ugv1_ckf.ckf.PosteriorCov{idx});
    end
  end
    title([meta.date meta.run '  ugv1 eig']); ylabel('eig'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv1_ckf.ckf.time,  data.ugv1_ckf.ckf.PosteriorEig, 'k.'); end
    hold off;
%% figure(501); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(501); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  clear data.uav_ckf.ckf.PosteriorEig
  try 
    for idx =  1:size(data.uav_ckf.ckf.PosteriorCov,1)
      data.uav_ckf.ckf.PosteriorEig(:,idx) = eig(data.uav_ckf.ckf.PosteriorCov{idx});
    end
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  hold on;
    title([meta.date meta.run '  uav eig']); ylabel('eig'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.uav_ckf.ckf.time,  data.uav_ckf.ckf.PosteriorEig, 'k.'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
%% figure(496); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  figure(496); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
	hold on
	try 
    for idx = 1:size(data.fullSLAM.eig,1)
      try plot(data.fullSLAM.time, data.fullSLAM.eig(idx,:), '.', 'displayname', sprintf( 'eig %02d', idx ) ); catch; end
		end
	end
  hold off
  grid on
%   legend('toggle')
%   try %current axis
%     current_limits = axis; current_axes = gca; 
%     axes = [48 80 -0.005 0.010];
%     current_axes.XTick = [axes(1):2:axes(2)];
%     current_axes.XLim = [axes(1) current_axes.XTick(end)];
%     current_axes.YTick = [axes(3):0.005:axes(4)];
%     current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%   end
  title([ meta.date meta.run ', system eigenvalues'])
  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','eig/system_eigenvalues', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(140); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig  
figure(140); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig  
  hold on
  for idx = [0:7, 9:11]
    try plot(data.vicon.time, data.errors.april.offset.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p); catch; end
  end
  hold off
%% figure(139-128); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
for idx = 0:11
  figure(139-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(sprintf( 'april%02d x,y,z', idx ))
		tag_str = sprintf( 'tag_%02d', idx );
		apr_str = sprintf( 'april%02d', idx );

		hold on;
		
      try plot( data.vicon.time, ...
                data.vicon.(matlab.lang.makeValidName(apr_str)).yaw.gl, ...
                'k.', 'displayname', 'vicon yaw global'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasYaw_gl, ...
                'go', 'displayname', 'fullSLAM MeasYaw gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedPhi, ...
                'r^', 'displayname', 'fullSLAM PredictedPhi gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPhi_gl, ...
                'bs', 'displayname', 'fullSLAM EstPhi gl'); catch; end

    hold off; clear h1 ii tag_str
    grid on
    legend('toggle')
    
%         try %current axis
%             current_limits = axis; current_axes = gca; 
%             current_axes.XLim = [current_limits(1) current_limits(2)];
%             current_axes.YLim = [-1.5 1.5];
%         end
end
%% figure(120-109); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
for idx = 0:11
  figure(120-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(sprintf( 'april%02d x,y,z', idx ))
		tag_str = sprintf( 'tag_%02d', idx );
		apr_str = sprintf( 'april%02d', idx );

		hold on;
		
        try h1=plot( data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(:,1), ...
                  'k.', 'displayname', ['vicon x']); catch; end
        try h1=plot( data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
                  data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).MeasPosition_gl(:,1), ...
                  'r^', 'displayname', ['ckf x']); catch; end
        try h1=plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,1), ...
                  'bs', 'displayname', ['slam x']); catch; end


    hold off; clear h1 ii tag_str
    grid on
    legend('toggle')
    
%         try %current axis
%             current_limits = axis; current_axes = gca; 
%             current_axes.XLim = [current_limits(1) current_limits(2)];
%             current_axes.YLim = [-1.5 1.5];
%         end
end
%% figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on;
    title([meta.date meta.run '  uav/ugv1/ugv2 x pose']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 1; ii = 0; legend_cells = {}; 
      try h = plot(data.vicon.time,  data.errors.uav.slam.p(:,1), 'k.', 'displayname', 'uav x error');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time, data.errors.ugv1.slam.p(:,1), 'r.', 'displayname', 'ugv1 x error'); ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time, data.errors.ugv2.slam.p(:,1), 'b.', 'displayname', 'ugv2 x error'); ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%% figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);

gen_vec_norm = @(x) sqrt(sum(x.*x,2));

  for idx = 0:11
    hold on; ii = 0; legend_vector = []; 
    try 
      % set strings for makevalidname
      tag_str = sprintf( 'tag_%02d', idx );
      tag_label = sprintf( 'tag%02d', idx );
      % spline vicon data to slam rate
      data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).spline.slam.position.gl = nD_spliner(...
        data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl,...
        data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl);

      data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.err.gl = ...
        data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl - data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).spline.slam.position.gl;

      data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).spline.ckf.position.gl = nD_spliner(...
        data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl,...
        data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl);

      data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).spline.ckf.err.gl = ...
        data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl - data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).spline.ckf.position.gl;
  %         


    catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try plot(data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
        gen_vec_norm(data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).spline.ckf.err.gl), '.'); catch; end
    try plot(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
        gen_vec_norm(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.err.gl), 'x'); catch; end

    hold off; 

  end
  grid on
  xlabel('time [s]')
  ylabel('location, x y z [m]')
  title([meta.date meta.run ', position error of all april tags'])
%% figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
  try 
    [data.ugv1_mux.est.error, data.ugv1_mux.est.d_traveled] =  error_over_d_traveled(...
      data.vicon.time, data.vicon.ugv1.position.gl, ...
      data.ugv1_mux.est.time, data.ugv1_mux.est.Position_gl);
    [data.ugv1_ckf.est.error, data.ugv1_ckf.est.d_traveled] =  error_over_d_traveled(...
      data.vicon.time, data.vicon.ugv1.position.gl, ...
      data.ugv1_ckf.est.time, data.ugv1_ckf.est.Position_gl);
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try 
    [data.ugv2_mux.est.error, data.ugv2_mux.est.d_traveled] =  error_over_d_traveled(...
      data.vicon.time, data.vicon.ugv2.position.gl, ...
      data.ugv2_mux.est.time, data.ugv2_mux.est.Position_gl);
    [data.ugv2_ckf.est.error, data.ugv2_ckf.est.d_traveled] =  error_over_d_traveled(...
      data.vicon.time, data.vicon.ugv2.position.gl, ...
      data.ugv2_ckf.est.time, data.ugv2_ckf.est.Position_gl);
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

  
  hold on
    try plot(data.ugv1_mux.est.d_traveled, data.ugv1_mux.est.error, 's', 'displayname', 'ugv1 mux err'); catch; end
    try plot(data.ugv1_ckf.est.d_traveled, data.ugv1_ckf.est.error, '.', 'displayname', 'ugv1 ckf err'); catch; end
    try plot(data.ugv2_mux.est.d_traveled, data.ugv2_mux.est.error, 's', 'displayname', 'ugv2 mux err'); catch; end
    try plot(data.ugv2_ckf.est.d_traveled, data.ugv2_ckf.est.error, '.', 'displayname', 'ugv2 ckf err'); catch; end
  hold off
  grid on
  legend('toggle')
  title('error = f(dist traveled)')
  xlabel('distance traveled [m]')
  ylabel('rms error [m]')


  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','60_error_over_d', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  title([meta.date meta.run 'uav yaw '])
  hold on;
    try h1 = plot(data.vicon.time, data.vicon.uav.yaw.gl, 'Color', [0.75,0.75,0.75], 'LineWidth', 3, ...
          'displayname', 'uav yaw actual'); legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; ii = ii + 1; end
    try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'k.-',  'displayname', 'Desired yaw'); catch; end
    try plot(data.uavCon.time, data.uavCon.Current.Yaw *pi /180, 'b.-',  'displayname', 'Current yaw'); catch; end
    try plot(data.uavCon.time, data.uavCon.Error.Yaw, 'r.-',  'displayname', 'Error yaw'); catch; end

    
  hold off
  grid on
  xlabel('time [s]'); ylabel('yaw position [rad]')
  legend('toggle')
%% figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  title([meta.date meta.run 'uav z '])
  hold on;
    try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,3), 'k.-',  'displayname', 'Desired z'); catch; end
    try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,3), 'b.-',  'displayname', 'Current z'); catch; end
       
  hold off
  grid on
  xlabel('time [s]'); ylabel('z position [m]')
  legend('toggle')
%% figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  title([meta.date meta.run 'uav y '])
  hold on;
    try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'k.-',  'displayname', 'Desired y'); catch; end
    try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,2), 'b.-',  'displayname', 'Current y'); catch; end
       
  hold off
  grid on
  xlabel('time [s]'); ylabel('y position [m]')
  legend('toggle')
%% figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on;
    title([meta.date meta.run '  uav/ugv1/ugv2 x pose']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 1; ii = 0; legend_cells = {}; 
      try h = plot(data.vicon.time,               data.vicon.ugv1.position.gl(:,1),       'displayname', 'ugv1 x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_mux.est.time,      data.ugv1_mux.est.Position_gl(:,1), 'r.', 'displayname', 'ugv1 x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,                data.vicon.uav.position.gl(:,1),       'displayname', 'uav x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,1), 'b.', 'displayname', 'uav x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,               data.vicon.ugv2.position.gl(:,1),       'displayname', 'ugv2 x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_mux.est.time,      data.ugv2_mux.est.Position_gl(:,1), 'k.', 'displayname', 'ugv2 x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%% figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('led location, ugv2')
  hold on; linewidth = 3; ii = 0; legend_cells = {}; 

  try h = line(  data.vicon.time,   data.vicon.red.position.ugv2(:,2), 'displayname',   'ugv1 red y', 'Marker', '.', 'LineStyle', 'none', 'Color', 'red');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(  data.vicon.time,  data.vicon.blue.position.ugv2(:,2), 'displayname',  'ugv1 blue y', 'Marker', '.', 'LineStyle', 'none', 'Color', 'blue');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(  data.vicon.time, data.vicon.green.position.ugv2(:,2), 'displayname', 'ugv1 green y', 'Marker', '.', 'LineStyle', 'none', 'Color', 'green');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time,   data.ugv2Stereo.Red.P_ugv(:,2), 'displayname',   'ugv1 red y stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'red');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time,  data.ugv2Stereo.Blue.P_ugv(:,2), 'displayname',  'ugv1 blue y stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'blue');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time, data.ugv2Stereo.Green.P_ugv(:,2), 'displayname', 'ugv1 green y stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'green');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  
  hold off
%   try legend(legend_cells,'Location','southwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  clear h ii legend_cells linewidth      
%% figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('led location, ugv2')
  hold on; linewidth = 3; ii = 0; legend_cells = {}; 

  try h = line(  data.vicon.time,   data.vicon.red.position.ugv2(:,1), 'displayname',   'ugv1 red x', 'Marker', '.', 'LineStyle', 'none', 'Color', 'red');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(  data.vicon.time,  data.vicon.blue.position.ugv2(:,1), 'displayname',  'ugv1 blue x', 'Marker', '.', 'LineStyle', 'none', 'Color', 'blue');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(  data.vicon.time, data.vicon.green.position.ugv2(:,1), 'displayname', 'ugv1 green x', 'Marker', '.', 'LineStyle', 'none', 'Color', 'green');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time,   data.ugv2Stereo.Red.P_ugv(:,1), 'displayname',   'ugv1 red x stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'red');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time,  data.ugv2Stereo.Blue.P_ugv(:,1), 'displayname',  'ugv1 blue x stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'blue');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  try h = line(data.ugv2Stereo.time, data.ugv2Stereo.Green.P_ugv(:,1), 'displayname', 'ugv1 green x stereo', 'Marker', 'o', 'LineStyle', 'none', 'Color', 'green');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

  try plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.UAV_yaw_var(:,1)); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  
%   hold off
%   try legend(legend_cells,'Location','southwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%   clear h ii legend_cells linewidth      
%% figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('led location, gl')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.red.position.gl(:,1), 'r.', 'displayname', 'red x actual'); catch; end
		try plot(data.ugv1Stereo.Red.solo.Time, data.ugv1Stereo.Red.solo.P_ugv(:,1), 'ro', 'displayname', 'red x ugv1Stereo'); catch; end
		try plot(data.vicon.time, data.vicon.blue.position.gl(:,1), 'b.', 'displayname', 'blue x actual'); catch; end
		try plot(data.ugv1Stereo.Blue.solo.Time, data.ugv1Stereo.Blue.solo.P_ugv(:,1), 'bo', 'displayname', 'blue x ugv1Stereo'); catch; end
		try plot(data.vicon.time, data.vicon.green.position.gl(:,1), 'g.', 'displayname', 'green x actual'); catch; end
		try plot(data.ugv1Stereo.Green.solo.Time, data.ugv1Stereo.Green.solo.P_ugv(:,1), 'go', 'displayname', 'green x ugv1Stereo'); catch; end
	hold off
	grid on
% 	legend('toggle')
%% figure(48); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(48); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('odom debugging')
  hold on; 
  title([meta.date meta.run])

    ii = 0; legend_vector = [];
    try h1=plot( data.vicon.ugv2.position.gl(:,1), data.vicon.ugv2.position.gl(:,2), 'k.', ...
        'displayname', 'ugv2 vicon'); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
    try h1=plot(data.trial.ugv2.local_path.picket_end_gl(:,1), data.trial.ugv2.local_path.picket_end_gl(:,2), 'o-', ...
        'displayname', 'picket end'); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
    try h1=plot(data.trial.ugv2.local_path.picket_mid_gl(:,1), data.trial.ugv2.local_path.picket_mid_gl(:,2), 'o-',...
        'displayname', 'picket mid'); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end

%   for i = [0,1,4,5,8,9] % gazebo
  for i = [2,3,5,6,7,8,10,11] % vicon
		apr_str = sprintf( 'april%02d', i );
		try 
			xt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,1);
			yt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,2);
			plot( xt, yt, 'k*', 'displayname', apr_str);
			text(xt,yt,apr_str, 'FontSize',8);
		catch; end
	end

  
  text(data.vicon.ugv2.position.gl(end,1), data.vicon.ugv2.position.gl(end,2), 'ugv2', 'FontSize',8)
  text(data.vicon.ugv1.position.gl(end,1), data.vicon.ugv1.position.gl(end,2), 'ugv1', 'FontSize',8)
  
  try plot(data.vicon.ugv1.position.gl(:,1), data.vicon.ugv1.position.gl(:,2), 'k.'); catch; end

  
  try 
    for idx = 1:size(data.trial.ugv2.local_path.p_lo, 2)
      try 
        plot(data.trial.ugv2.local_path.p_gl{idx}(:,1), data.trial.ugv2.local_path.p_gl{idx}(:,2)); 
      catch; end
    end; clear idx
  end

  try 
    for idx = 1:size(data.trial.ugv1.local_path.p_lo, 2)
      try 
        plot(data.trial.ugv1.local_path.p_gl{idx}(:,1), data.trial.ugv1.local_path.p_gl{idx}(:,2)); 
      catch; end
    end; clear idx
  end
    
  
  try 
    for idx = 1:length(data.trial.ugv2.local_path.p_mid)
      kdx = floor(size(data.trial.ugv2.local_path.p_gl{idx}(:,1),1)/2);
      x = [data.trial.ugv2.local_path.picket_mid_gl(idx,1) data.trial.ugv2.local_path.p_gl{idx}(kdx,1)];
      y = [data.trial.ugv2.local_path.picket_mid_gl(idx,2) data.trial.ugv2.local_path.p_gl{idx}(kdx,2)];
      line(x,y,'Color','red','LineStyle','--')
    end
  end

  try 
    for idx = 1:length(data.trial.ugv1.local_path.p_mid)
      kdx = floor(size(data.trial.ugv1.local_path.p_gl{idx}(:,1),1)/2);
      x = [data.trial.ugv1.local_path.picket_mid_gl(idx,1) data.trial.ugv1.local_path.p_gl{idx}(kdx,1)];
      y = [data.trial.ugv1.local_path.picket_mid_gl(idx,2) data.trial.ugv1.local_path.p_gl{idx}(kdx,2)];
      line(x,y,'Color','red','LineStyle','--')
    end
  end

  
  hold off
  grid on
  
    if meta.columnlegend
      try 
        try columnlegend(3,legend_str); catch; end %clear legend_str
        clear legend_str legend_vector
      end
    else
      legend(legend_vector, legend_str)
    end


%     legend([h1, h2, h3], {'path', 'tf', 'listener'})
    
%    set(gca, 'XDir','reverse')
  xlabel('x position [m]')
  ylabel('y position [m]')
%% figure(47); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(47); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('odom debugging')
	
	hold on
%     for idx = 1:size(data.trial.ugv2.local_path.p_lo, 2)
%       try h1 = plot(data.trial.ugv2.local_path.p_lo{idx}(:,2), data.trial.ugv2.local_path.p_lo{idx}(:,1)); catch; end
%     end; clear idx
    
%     try h2 = plot(data.trial.ugv2.tf_endpt_odom.p(:,1), data.trial.ugv2.tf_endpt_odom.p(:,2), 'k.-'); catch; end
%     try h3 = plot(data.trial.ugv2.picket_end.p(:,1), data.trial.ugv2.picket_end.p(:,2), 'b.-'); catch; end
    
    
      try plot(data.trial.ugv2.local_path.picket_end_odom(:,2), data.trial.ugv2.local_path.picket_end_odom(:,1), 'ko-'); catch; end

	hold off
	grid on


%     legend([h1, h2, h3], {'path', 'tf', 'listener'})
    
   set(gca, 'XDir','reverse')
  xlabel('y position [m]')
  ylabel('x position [m]')
%% figure(46); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(46); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.uav.position.gl(:,3), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav z actual'); catch; end
		
        try plot(data.uavCon.cmd.time, data.uavCon.cmd.Desired.Position_g(:,3), 'k.-',  'displayname', 'Desired z'); catch; end
        try plot(data.uavCon.cmd.time, data.uavCon.cmd.Current.Position_g(:,3), 'b.-',  'displayname', 'Current z'); catch; end

        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,3), 'k^-',  'displayname', 'Desired z'); catch; end
        try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,3), 'b^-',  'displayname', 'Current z'); catch; end

        try plot(data.uavCon.msg.time, data.uavCon.msg.Current.Position_g(:,3), 'm-',  'displayname', 'Current msg z'); catch; end


	hold off
	grid on
	legend('toggle')
%% figure(45); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(45); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.uav.position.gl(:,2), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav y actual'); catch; end
		try plot(data.uavCon.cmd.time, data.uavCon.cmd.Desired.Position_g(:,2), 'k.-',  'displayname', 'Desired y'); catch; end
    try plot(data.uavCon.cmd.time, data.uavCon.cmd.Current.Position_g(:,2), 'b.-',  'displayname', 'Current y'); catch; end
    try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'k^-',  'displayname', 'Desired y'); catch; end
    try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,2), 'b^-',  'displayname', 'Current y'); catch; end
	hold off
	grid on
	legend('toggle')
%% figure(44); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(44); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
    try plot(data.vicon.time, data.vicon.uav.position.gl(:,1), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav x actual'); catch; end
    try plot(data.uavCon.cmd.time, data.uavCon.cmd.Desired.Position_g(:,1), 'k.-',  'displayname', 'Desired x'); catch; end
    try plot(data.uavCon.cmd.time, data.uavCon.cmd.Current.Position_g(:,1), 'b.-',  'displayname', 'Current x'); catch; end
    try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'k^-',  'displayname', 'Desired x'); catch; end
    try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,1), 'b^-',  'displayname', 'Current x'); catch; end
    try plot(data.uavCon.cmd.time, data.uavCon.cmd.linear(:,1), 'm-',  'displayname', 'linear cmd x'); catch; end
    try plot(data.uavCon.msg.time, data.uavCon.msg.Current.Position_g(:,1), 'm-',  'displayname', 'Current msg x'); catch; end
	hold off
	grid on
	legend('toggle')
% uavCon.cmd.time(1,:) =  82.48886752;
%   uavCon.cmd.linear(1,:)  = [ 0.00000000,  0.00000000,  0.00000000];
%   uavCon.cmd.angular(1,:) = [ 0.00000000,  0.00000000,  0.00000000];
%   uavCon.cmd.Desired.Position_g(1,:)   = [ 2.00000000,  0.00000000,  1.50000000];
%   uavCon.cmd.Current.Position_g(1,:)   = [ 0.00000000,  0.00000000,  0.50000000];
%   uavCon.cmd.Current.Yaw(1,:)  =  0.00000000;
%   uavCon.cmd.Desired.Yaw(1,:)  =  0.00000000;
%% figure(43); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(43); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location (yaw)')
	linewidth = 3;
	hold on
      try plot(data.vicon.time, data.vicon.uav.yaw.gl, 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav yaw actual'); catch; end

% 		try plot(data.vicon.time, data.vicon.uav.position.gl(:,3), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav z actual'); catch; end
		try plot(data.trial.uav.FlyTime, pi*data.trial.uav.Waypoint(:,4)/180, 'kd', 'displayname', 'waypoint z'); catch; end
		try plot(data.trial.uav.FlyTime, pi*data.trial.uav.DesiredState_gl(:,4)/180, 'k.', 'displayname', 'desired z'); catch; end
		try plot(data.trial.uav.FlyTime, pi*data.trial.uav.CurrentState_gl(:,4)/180, 'r.', 'displayname', 'current z'); catch; end
        try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.ugv.Yaw, '^',  'displayname', 'ugv1 yaw oneCKF'); catch; end
% 		try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.EstPosition_gl(:, 3), 'r^', 'displayname', ' UAV z ckf' ); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,1), 'b.', 'displayname', 'current x'); catch; end
% 		try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,3), 'bs', 'displayname', 'uav z (slam)'); catch; end
		
	hold off
	grid on
	legend('toggle')
%% figure(42); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(42); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.uav.position.gl(:,3), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav z actual'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,3), 'kd', 'displayname', 'waypoint z'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.DesiredState_gl(:,3), 'k.', 'displayname', 'desired z'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,3), 'r.', 'displayname', 'current z'); catch; end
		try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.EstPosition_gl(:, 3), 'r^', 'displayname', ' UAV z ckf' ); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,1), 'b.', 'displayname', 'current x'); catch; end
		try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,3), 'bs', 'displayname', 'uav z (slam)'); catch; end
		
	hold off
	grid on
	legend('toggle')
%% figure(41); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(41); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.uav.position.gl(:,2), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav y actual'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,2), 'kd', 'displayname', 'waypoint y'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.DesiredState_gl(:,2), 'k.', 'displayname', 'desired y'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,2), 'r.', 'displayname', 'current y'); catch; end
		try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.EstPosition_gl(:, 2), 'r^', 'displayname', ' UAV y ckf' ); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,1), 'b.', 'displayname', 'current x'); catch; end
		try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,2), 'bs', 'displayname', 'uav y (slam)'); catch; end
		
	hold off
	grid on
	legend('toggle')
%% figure(40); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(40); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
	title('uav desired location')
	linewidth = 3;
	hold on
		try plot(data.vicon.time, data.vicon.uav.position.gl(:,1), 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth, 'displayname', 'uav x actual'); catch; end
		try plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,1), 'kd', 'displayname', 'waypoint x'); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.DesiredState_gl(:,1), 'k.', 'displayname', 'desired x'); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,1), 'r.', 'displayname', 'current x'); catch; end
		try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.EstPosition_gl(:, 1), 'r^', 'displayname', ' UAV x ckf' ); catch; end
% 		try plot(data.trial.uav.FlyTime, data.trial.uav.CurrentState_gl(:,1), 'b.', 'displayname', 'current x'); catch; end
		try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,1), 'bs', 'displayname', 'uav x (slam)'); catch; end
		
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'k.-',  'displayname', 'Desired x'); catch; end
        try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,1), 'b.-',  'displayname', 'Current x'); catch; end

        try plot(data.uavCon.msg.time, data.uavCon.msg.Current.Position_g(:,1), 'ms-',  'displayname', 'Current x msg'); catch; end
        
	hold off
	grid on
	legend('toggle')
%% figure(39); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(39); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  ckf yaw correction']); ylabel('yaw [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.uav_ckf.ckf.time,   data.uav_ckf.ckf.correction_mat(:,4), 'k^', 'displayname',  'uav yaw ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time, data.ugv1_ckf.ckf.PosteriorEst_mat(:,8), 'b^', 'displayname', 'ugv1 yaw ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time, data.ugv2_ckf.ckf.PosteriorEst_mat(:,8), 'r^', 'displayname', 'ugv2 yaw ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth
  
    try %current axis
      current_limits = axis; current_axes = gca; 
      new_axes = [current_limits(1) current_limits(2) -2.5 2.5];
      dt = round((current_limits(2)-current_limits(1))/10);
      current_axes.XTick = [0:dt:new_axes(2)];
      current_axes.XLim = [0 current_axes.XTick(end)];
      current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end; clear dt
%% figure(38); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(38); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  ckf z correction']); ylabel('z [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.uav_ckf.ckf.time,   data.uav_ckf.ckf.correction_mat(:,3), 'k^', 'displayname',  'uav z ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time, data.ugv1_ckf.ckf.PosteriorEst_mat(:,7), 'b^', 'displayname', 'ugv1 z ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time, data.ugv2_ckf.ckf.PosteriorEst_mat(:,7), 'r^', 'displayname', 'ugv2 z ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth
  
    try %current axis
      current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -2.5 2.5];
      new_axes = [130 150 -2.5 2.5];
      dx = round((new_axes(2)-new_axes(1))/10);
      dy = round((new_axes(4)-new_axes(3))/10);
      current_axes.XTick = [new_axes(1):dx:new_axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [new_axes(3):dy:new_axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end; clear dx dy
%% figure(37); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(37); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  ckf y correction']); ylabel('y [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.uav_ckf.ckf.time,   data.uav_ckf.ckf.correction_mat(:,2), 'k^', 'displayname',  'uav y ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time, data.ugv1_ckf.ckf.PosteriorEst_mat(:,6), 'b^', 'displayname', 'ugv1 y ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time, data.ugv2_ckf.ckf.PosteriorEst_mat(:,6), 'r^', 'displayname', 'ugv2 y ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth
  
    try %current axis
      current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -2.5 2.5];
      new_axes = [130 150 -2.5 2.5];
      dx = round((new_axes(2)-new_axes(1))/10);
      dy = round((new_axes(4)-new_axes(3))/10);
      current_axes.XTick = [new_axes(1):dx:new_axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [new_axes(3):dy:new_axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end; clear dx dy
%% figure(36); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(36); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  ckf x correction']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.uav_ckf.ckf.time,   data.uav_ckf.ckf.correction_mat(:,1), 'k^', 'displayname',  'uav x ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time, data.ugv1_ckf.ckf.PosteriorEst_mat(:,5), 'b^', 'displayname', 'ugv1 x ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time, data.ugv2_ckf.ckf.PosteriorEst_mat(:,5), 'r^', 'displayname', 'ugv2 x ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth
  
    try %current axis
      current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -2.5 2.5];
      new_axes = [130 150 -2.5 2.5];
      dx = round((new_axes(2)-new_axes(1))/10);
      dy = round((new_axes(4)-new_axes(3))/10);
      current_axes.XTick = [new_axes(1):dx:new_axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [new_axes(3):dy:new_axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end; clear dx dy
%% figure(29); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);     
figure(29); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  for idx = 8
      hold on; ii = 0; legend_vector = []; 
        tag_str = sprintf( 'tag_%02d', idx );
        tag_label = sprintf( 'tag%02d', idx );
        tag_label_n = sprintf( '\ntag%02d', idx );
				try         
					text(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time(1,1),...
            data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasYaw_uav(1,1), [tag_label_n], 'FontSize',12)
				catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        
        try 
          h1=plot(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasYaw_uav, ...
                  'o', 'displayname', ['measured yaw x']);
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        try 
          h1=plot(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedPhi_uav, ...
                  's', 'Color', h1.Color, 'displayname', ['slam x']); 
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
      hold off; 
  end
  grid on
  xlabel('time [s]')
  ylabel('location, x y z [m]')
  title([meta.date meta.run ', all april tags'])

  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','landmark_estimated_states', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  
  clear h1 ii tag_str legend_str
%% figure(28); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);   
figure(28); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  for idx = 0:11
      hold on; ii = 0; legend_vector = []; 
        tag_str = sprintf( 'tag_%02d', idx );
        tag_label = sprintf( 'tag%02d', idx );
        tag_label_n = sprintf( '\ntag%02d', idx );
				try         
					text(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time(1,1),...
            data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasYaw_uav(1,1), [tag_label_n], 'FontSize',12)
				catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        
        try 
          h1=plot(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasYaw_uav, ...
                  '.', 'displayname', ['measured yaw x']);
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        try 
          h1=plot(data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  -data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedPhi_uav, ...
                  's', 'Color', h1.Color, 'displayname', ['slam x']); 
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
      hold off; 
  end
  grid on
  xlabel('time [s]')
  ylabel('location, phi [rad]')
  title([meta.date meta.run ', all april tags (uav frame)'])

  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','landmark_estimated_states', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  
  clear h1 ii tag_str legend_str
%% figure(27); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);   
figure(27); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  for idx = 0:11
      hold on; ii = 0; legend_vector = []; 
        tag_str = sprintf( 'tag_%02d', idx );
        tag_label = sprintf( 'tag%02d', idx );
        tag_label_n = sprintf( '\ntag%02d', idx );
				try         
					if (data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl(1,1) ~= 0)
					text(data.vicon.time(1,1), data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl(1,1), [tag_label_n], 'FontSize',12)
% 					text(data.vicon.time(1,1), data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl(1,2), [tag_label_n], 'FontSize',12)
					end
				catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        
        try 
          h1=plot( data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl, ...
                  '.', 'displayname', ['vicon yaw']);
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        try 
          h1=plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPhi_gl, ...
                  's', 'Color', h1.Color, 'displayname', ['slam yaw']); 
        catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        
      hold off; 
      
  end
  grid on
  xlabel('time [s]')
  ylabel('yaw [rad]')
  title([meta.date meta.run ', all april tags'])

  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','landmark_estimated_states', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  
  clear h1 ii tag_str legend_str
%% figure(26); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);   
figure(26); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  for idx = 0:11
      hold on; ii = 0; legend_vector = []; 
        tag_str = sprintf( 'tag_%02d', idx );
        tag_label = sprintf( 'tag%02d', idx );
        tag_label_n = sprintf( '\ntag%02d', idx );
				try         
					if (data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(1,1) ~= 0)
					text(data.vicon.time(1,1), data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(1,1), [tag_label_n], 'FontSize',12)
					text(data.vicon.time(1,1), data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(1,2), [tag_label_n], 'FontSize',12)
					end
				catch; end
        try h1=plot( data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(:,1), ...
                  'b.', 'displayname', ['vicon x']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        try h1=plot( data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(:,2), ...
                  'r.', 'displayname', ['vicon y']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        try h1=plot( data.vicon.time, data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl(:,3), ...
                  'k.', 'displayname', ['vicon z']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        
        try h1=plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,1), ...
                  'bs', 'displayname', ['slam x']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        try h1=plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,2), ...
                  'rs', 'displayname', ['slam y']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        try h1=plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,3), ...
                  'ks', 'displayname', ['slam z']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        
        try h1=plot(data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
                    data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl(:,1), ...
                    'b^', 'displayname', ['ckf x']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; 
                    catch Mexc_; disp([sprintf( 'tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        try h1=plot(data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
                    data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl(:,2), ...
                    'r^', 'displayname', ['ckf y']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
        try h1=plot(data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
                    data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl(:,3), ...
                    'k^', 'displayname', ['ckf z']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end
      hold off; 

      
%     ugv1_ckf.april.tag_02.time(654,:) =  77.20999999999999;
%       ugv1_ckf.april.tag_02.EstYaw_gl(655,:)   = 0.26738175695629;
%       ugv1_ckf.april.tag_02.MeasYaw_gl(654,:)   = 0.27066363372535;
%       ugv1_ckf.april.tag_02.MeasYaw_uav(654,:)  = 0.27325592089745;
%       ugv1_ckf.april.tag_02.EstPosition_gl(655,:)  = [2.29771611, 0.61312992, 0.00661035];
%       ugv1_ckf.april.tag_02.MeasPosition_gl(654,:)  = [2.28200470, 0.61926534, 0.01528181];
%       ugv1_ckf.april.tag_02.MeasPosition_uav(654,:) = [-0.30539304, 0.63216940, -1.19791506];

      
  end
  grid on
  xlabel('time [s]')
  ylabel('location, x y z [m]')
  title([meta.date meta.run ', all april tags'])
  
      if meta.columnlegend
      try 
        try columnlegend(3,legend_str); catch; end
        clear legend_str legend_vector
      end
    else
      legend(legend_vector, legend_str)
    end
%   legend(h1,h2,h3,h4,h5,h6,h7,h8,h9)
%   legend('toggle')
%   try %current axis
%     current_limits = axis; current_axes = gca; 
% %     axes = [current_limits(1) current_limits(2) -100 100];
%     axes = [current_limits(1) current_limits(2) -5 5];
%     current_axes.XTick = [0:5:axes(2)];
%     current_axes.XLim = [0 current_axes.XTick(end)];
%     current_axes.YTick = [axes(3):20:axes(4)];
%     current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%   end
  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','landmark_estimated_states', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  
  clear h1 ii tag_str legend_str
%% figure(25); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);   
figure(25); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
title(' ckf priori yaw and correction')
hold on
    try plot(data.vicon.time,           data.vicon.uav.yaw.gl,  'b.', 'displayname', 'uav phi actual'); catch; end
    try plot(data.vicon.time,           data.vicon.ugv1.yaw.gl, 'r.', 'displayname', 'ugv1 phi actual'); catch; end
    
%     try plot(data.vicon.time, data.vicon.uav.yaw.ugv1, 'Color', [0.75,0.75,0.75], 'LineWidth', 3, 'displayname', 'vicon UAV yaw ugv1 frame' ); catch; end    

%     try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv*pi/180, 'o', 'displayname', 'stereo UAV yaw ugv1 frame' ); catch; end
%     try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.yawCorrection*pi/180, 's', 'displayname', 'ckf UAV yaw correction' ); catch; end
%     try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.Yaw*pi/180, '.', 'displayname', 'ckf UAV yaw' ); catch; end
    try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.PiriorEstYaw_gl*pi/180, 'b^', 'displayname', 'ckf uav yaw' ); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.ugv.PiriorEstYaw_gl, 'r^', 'displayname', 'ckf ugv yaw' ); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    
    
    grid on
    
hold off
legend('toggle')

%   data.ugv1_ckf.oneCKF.uav.PiriorEstYaw_gl(593,:) = -1.45222884300436;
%   data.ugv1_ckf.oneCKF.ugv.PiriorEstYaw_gl(593,:) = -0.00000459011461;
%   data.ugv1_ckf.oneCKF.time(593,:) =  129.50200000000001;
%% figure(24); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number);   
figure(24); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on
    try plot(data.fullSLAM.april.tag_08.slam.time, data.fullSLAM.april.tag_08.slam.MeasYaw_uav,  'o', 'displayname', 'MeasYaw uav'); end
    try plot(data.fullSLAM.april.tag_08.slam.time, data.fullSLAM.april.tag_08.slam.MeasYaw_gl,   'x', 'displayname', 'MeasYaw gl'); end
    try plot(data.fullSLAM.april.tag_08.slam.time, data.fullSLAM.april.tag_08.slam.PredictedPhi, 's', 'displayname', 'PredictedPhi'); end
  hold off
  grid on
  legend('toggle')
%% figure(23); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(23); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
title('uav ckf yaw and correction')
hold on
    try plot(data.vicon.time, data.vicon.uav.yaw.ugv1, 'Color', [0.75,0.75,0.75], 'LineWidth', 3, 'displayname', 'vicon UAV yaw ugv1 frame' ); catch; end    

    try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv*pi/180, 'o', 'Color', default_colors{1}, 'displayname', 'stereo UAV yaw ugv1 frame' ); catch; end
    
%     try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.yawCorrection*pi/180, 's', 'displayname', 'ckf UAV yaw correction' ); catch; end
    try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.Yaw*pi/180, '.', 'displayname', 'ckf UAV yaw' ); catch; end
%     try plot(data.uavCKF.navdata.time, data.uavCKF.navdata.EstYaw_gl*pi/180, '.', 'displayname', 'ckf UAV yaw' ); catch; end

    grid on
    
    
hold off
legend('toggle')
%% figure(22); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(22); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
title('uav ckf yaw and correction')
hold on
    try plot(data.vicon.time, data.vicon.uav.yaw.ugv1, 'Color', [0.75,0.75,0.75], 'LineWidth', 3, 'displayname', 'vicon UAV yaw ugv1 frame' ); catch; end    

%     try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv*pi/180, 'o', 'displayname', 'stereo UAV yaw ugv1 frame' ); catch; end
%     try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.yawCorrection*pi/180, 's', 'displayname', 'ckf UAV yaw correction' ); catch; end
%     try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.uav.Yaw*pi/180, '.', 'displayname', 'ckf UAV yaw' ); catch; end
    try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.zk(:,4), '-', 'displayname', 'ckf zk yaw' ); catch; end
    try plot(data.ugv1_ckf.oneCKF.time, data.ugv1_ckf.oneCKF.yk(:,4), '-', 'displayname', 'ckf yk yaw' ); catch; end
    
    
%     
    
    
    grid on
    
hold off
legend('toggle')
%% figure(20); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(20); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on
    try plot(data.vicon.ugv2.position.gl(:,1), data.vicon.ugv2.position.gl(:,2), 'k.'); catch; end
%     plot(data.uavCon.Desired.Position_g(:,1), data.uavCon.Desired.Position_g(:,2), 'r.', 'displayname', 'uav desried position'); 
    try h1=plot( data.ugv2_mux.est.Position_gl(:,1), data.ugv2_mux.est.Position_gl(:,2), ...
            'b.', 'displayname', ['ugv2 slam (+odom)']); ii = ii + 1; legend_str{ii} = h1.DisplayName; legend_vector(ii) = h1; catch; end

%     for idx = 1 : size(data.trial.ugv2.global_path.p_lo,2)
%       try plot(data.trial.ugv2.global_path.p_lo{idx}(:,1), data.trial.ugv2.global_path.p_lo{idx}(:,2), 'o-', 'displayname', 'ugv2 global (gl) plan'); catch; end
%       try plot(data.trial.ugv2.global_path.p_gl{idx}(:,1), data.trial.ugv2.global_path.p_gl{idx}(:,2), '*-', 'displayname', 'ugv2 global (lo) plan'); catch; end
%       
%     end

	try for idx = 1 : size(data.trial.ugv2.local_path.p_lo,2)
%       try plot(data.trial.ugv2.local_path.p_lo{idx}(:,1), data.trial.ugv2.local_path.p_lo{idx}(:,2), 'o-', 'displayname', 'ugv2 local (lo) plan'); catch; end
      try plot(data.trial.ugv2.local_path.p_gl{idx}(:,1), data.trial.ugv2.local_path.p_gl{idx}(:,2), '-', 'displayname', 'ugv2 wheel (gl) plan'); catch; end
		end
	catch; end
    
    
    
  hold off
  grid on
%   legend('toggle')
  xlabel('x axis [m]')
  ylabel('y axis [m]')
%% figure(19); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(19); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on;
    title([meta.date meta.run '  ugv2 yaw pose']); ylabel('yaw [rad]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv2_mux.est.time,  data.ugv2_mux.est.Yaw,            'bs', 'displayname', 'ugv2 yaw (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.dkf.time,  data.ugv2_ckf.dkf.ugv.EstYaw_gl,  'm*', 'displayname', 'ugv2 yaw dkf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time,  data.ugv2_ckf.ckf.EstYaw_gl,      'r^', 'displayname', 'ugv2 yaw ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,         data.vicon.ugv2.yaw.gl,                 'displayname', 'ugv2 yaw actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth      

%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','04_ugv2_phi', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(18); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(18); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on;
    title([meta.date meta.run '  ugv2 z pose']); ylabel('z [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv2_mux.est.time,  data.ugv2_mux.est.Position_gl(:,3),  'bs', 'displayname', 'ugv2 z (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.dkf.time,  data.ugv2_ckf.dkf.ugv.EstP_gl(:,3),  'm*', 'displayname', 'ugv2 z dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time,      data.ugv2_ckf.ckf.EstP_gl(:,3),  'r^', 'displayname', 'ugv2 z ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv2.position.gl(:,3),        'displayname', 'ugv2 z actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(17); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(17); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on;
    title([meta.date meta.run '  ugv2 y pose']); ylabel('y [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv2_mux.est.time,  data.ugv2_mux.est.Position_gl(:,2),  'bs', 'displayname', 'ugv2 y (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.dkf.time,  data.ugv2_ckf.dkf.ugv.EstP_gl(:,2),  'm*', 'displayname', 'ugv2 y dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time,      data.ugv2_ckf.ckf.EstP_gl(:,2),  'r^', 'displayname', 'ugv2 y ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv2.position.gl(:,2),        'displayname', 'ugv2 y actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

    
    axis([0 180 0 5])
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(16); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(16); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on;
    title([meta.date meta.run '  ugv2 x pose']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv2_mux.est.time,  data.ugv2_mux.est.Position_gl(:,1),  'bs', 'displayname', 'ugv2 x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.dkf.time,  data.ugv2_ckf.dkf.ugv.EstP_gl(:,1),  'm*', 'displayname', 'ugv2 x dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv2_ckf.ckf.time,      data.ugv2_ckf.ckf.EstP_gl(:,1),  'r^', 'displayname', 'ugv2 x ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv2.position.gl(:,1),        'displayname', 'ugv2 x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%     axis([0 180 0 5])
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(15); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(15); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 

  hold on
    try plot(data.vicon.time, data.vicon.uav.yaw.ugv2, 'b-', 'displayname', 'vicon UAV yaw ugv2 frame' ); catch; end
    try plot(data.vicon.time, data.vicon.ugv1.yaw.ugv_frame, '-', 'Color', default_colors{4}, 'displayname', 'vicon ugv1 yaw in ugv2 frame' ); catch; end
    try plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.yaw_ugv.*(pi/180), 'r.', 'displayname', 'stereo UAV x ugv2 frame' ); catch; end
    try plot(data.ugv2Stereo.ugv_sensor.time, data.ugv2Stereo.ugv_sensor.yaw, '*', 'Color', default_colors{4}, 'displayname', 'stereo ugv1 yaw in ugv2 frame' ); catch; end
    
    hold off
  grid on
  legend('toggle')
%% figure(14); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(14); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on
    try plot(data.vicon.time, data.vicon.uav.yaw.ugv1,                        'b-',                            'displayname', 'vicon UAV yaw ugv1 frame' ); catch; end
    try plot(data.vicon.time, data.vicon.ugv2.yaw.ugv_frame,                  '-', 'Color', default_colors{4}, 'displayname', 'vicon ugv2 yaw in ugv1 frame' ); catch; end
    try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv.*(pi/180),     'r.',                            'displayname', 'stereo UAV yaw ugv1 frame' ); catch; end
    try plot(data.ugv1Stereo.ugv_sensor.time, data.ugv1Stereo.ugv_sensor.yaw, '*', 'Color', default_colors{4}, 'displayname', 'stereo ugv2 yaw in ugv1 frame' ); catch; end
    
  hold off
  grid on
  legend('toggle')
%% figure(13); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(13); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  try 
    title(['tag slam corections  ' meta.date meta.run])
  hold on
  try plot(data.fullSLAM.time, data.fullSLAM.numberOfTagsObserved/1000, 'LineWidth', 3, 'displayname', 'number of observed tags'); catch; end
  try plot(data.fullSLAM.april.time, data.fullSLAM.april.ArraySize/1000, 'LineWidth', 3, 'displayname', 'number of tags in FOV'); catch; end
  for idx=0:11
    try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
              data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,1), ...
              'x', 'displayname', 'fullSLAM correction gl'); catch; end
    try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
              data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,2), ...
              'o', 'displayname', 'fullSLAM correction gl'); catch; end
    try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
              data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,3), ...
              '.', 'displayname', 'fullSLAM correction gl'); catch; end
    try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
              data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionYaw, ...
              '^', 'displayname', 'fullSLAM correction gl'); catch; end
      
  end
  
  hold off
  grid on

  ylabel('correction [m]')
  xlabel('time [s]')
%   try %current axis
%     current_limits = axis; current_axes = gca; 
%     axes = [current_limits(1) current_limits(2) -0.025 0.025];
%     dt = round((axes(2)-axes(1))/10);
% %     dA = round((axes(4)-axes(3))/10);
%     current_axes.XTick = [0:dt:axes(2)];
%     current_axes.XLim = [0 current_axes.XTick(end)];
% %     current_axes.YTick = [axes(3):dA:axes(4)];
%     current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%   end; clear dt dA

  clear idx
  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','tag_slam_corrections', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  end
%% figure(12); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(12); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  try 
    hold on
    try plot(data.fullSLAM.april.time, data.fullSLAM.april.ArraySize/10, 'LineWidth', 3, 'displayname', '(# of tags in FOV)/10'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,1), 'o', 'displayname', 'uav correction x'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,2), '.', 'displayname', 'uav correction y'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,3), 'x', 'displayname', 'uav correction z'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,4), '^', 'displayname', 'uav correction yaw'); catch; end
  hold off
  grid on
  title([meta.date meta.run ', uav slam corections'])
  ylabel('correction [m]')
  xlabel('time [s]')
  legend('toggle')
%   try %current axis
%     current_limits = axis; current_axes = gca; 
%     axes = [current_limits(1) current_limits(2) -1.5 1.5];
%     current_axes.XTick = [0:5:axes(2)];
%     current_axes.XLim = [0 current_axes.XTick(end)];
%     current_axes.YTick = [axes(3):0.5:axes(4)];
%     current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%   end

  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','uav_slam_corrections', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  end
%% figure(11); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(11); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  try
  hold on
  try plot(data.fullSLAM.time, data.fullSLAM.numberOfTagsObserved/10, 'LineWidth', 3, 'displayname', 'number of observed tags'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,1), 'bo'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,2), 'ro'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,3), 'ko'); catch; end
    try plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.correction(:,4), 'go'); catch; end
    for idx=0:11
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,1), ...
                'bs', 'displayname', 'fullSLAM correction gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,2), ...
                'rs', 'displayname', 'fullSLAM correction gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,3), ...
                'ks', 'displayname', 'fullSLAM correction gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.correctionP_gl(:,4), ...
                'gs', 'displayname', 'fullSLAM correction gl'); catch; end
    end
  hold off
  grid on
  clear idx
  try %current axis
      current_limits = axis; current_axes = gca; 
      fig_axes = [current_limits(1) current_limits(2) -2 2];
%       current_axes.XTick = [0:10:axes(2)];
%       current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [fig_axes(3):0.5:fig_axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
  end  
  title([meta.date meta.run ', slam corections'])
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','uav_slam_corrections', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  end
%% figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on;
    title([meta.date meta.run '  ugv1 yaw pose']); ylabel('yaw [rad]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv1_mux.est.time,  data.ugv1_mux.est.Yaw,            'bs', 'displayname', 'ugv1 yaw (slam)');   ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.dkf.time,  data.ugv1_ckf.dkf.ugv.EstYaw_gl,  'm*', 'displayname', 'ugv1 yaw dkf');      ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time,  data.ugv1_ckf.ckf.EstYaw_gl,      'r^', 'displayname', 'ugv1 yaw ckf');      ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,         data.vicon.ugv1.yaw.gl,                 'displayname', 'ugv1 yaw actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.est.time,  data.ugv1_ckf.est.Yaw,            'r.', 'displayname', 'ugv1 yaw ckf');      ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','southwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth      

%     axis([0 50 -5 5])
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt

    
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','04_ugv2_phi', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title([meta.date meta.run '  ugv1 z pose']); ylabel('z [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv1_mux.est.time,  data.ugv1_mux.est.Position_gl(:,3),  'bs', 'displayname', 'ugv1 z (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.dkf.time,  data.ugv1_ckf.dkf.ugv.EstP_gl(:,3),  'm*', 'displayname', 'ugv1 z dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time,      data.ugv1_ckf.ckf.EstP_gl(:,3),  'r^', 'displayname', 'ugv1 z ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv1.position.gl(:,3),        'displayname', 'ugv1 z actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   
    
%         axis([0 50 -5 5])

%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt

%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title([meta.date meta.run '  ugv1 y pose']); ylabel('y [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv1_mux.est.time,  data.ugv1_mux.est.Position_gl(:,2),  'bs', 'displayname', 'ugv1 y (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       try h = plot(data.ugv1_mux.wheel.time,   data.ugv1_mux.wheel.P_odom(:,2),  'b.', 'displayname', 'ugv1 y (wheel)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.dkf.time,  data.ugv1_ckf.dkf.ugv.EstP_gl(:,2),  'm*', 'displayname', 'ugv1 y dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time,      data.ugv1_ckf.ckf.EstP_gl(:,2),  'r^', 'displayname', 'ugv1 y ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv1.position.gl(:,2),        'displayname', 'ugv1 y actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%   ugv1_mux.wheel.time(39197,1) =  147.96400000000000;
%   ugv1_mux.wheel.dt(39197,1)   =  0.01900000000001;
%   ugv1_mux.wheel.P_odom(39197,:)  = [ 1.56726456866286, -0.72502283445497,  0.00000000000000];
%   ugv1_mux.wheel.yaw_q(39197,1)   =  -0.21530467165534;
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt

    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on;
    title([meta.date meta.run '  ugv1 x pose']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.ugv1_mux.est.time,  data.ugv1_mux.est.Position_gl(:,1),  'bs', 'displayname', 'ugv1 x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.dkf.time,  data.ugv1_ckf.dkf.ugv.EstP_gl(:,1),  'm*', 'displayname', 'ugv1 x dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.ugv1_ckf.ckf.time,      data.ugv1_ckf.ckf.EstP_gl(:,1),  'r^', 'displayname', 'ugv1 x ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,           data.vicon.ugv1.position.gl(:,1),        'displayname', 'ugv1 x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth   

%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -1.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt

    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','03_ugv2_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(5); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
  figure(5); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  uav yaw pose']); ylabel('yaw [rad]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
%       try h = plot(data.uav_ckf.share_ckf.time, data.uav_ckf.share_ckf.req.obs_yaw, 'go', 'displayname', 'obs yaw ckf share');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.fullSLAM.uav.est.time,    data.fullSLAM.uav.est.yaw.global, 'bs', 'displayname', 'uav yaw (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       try h = plot(data.uav_ckf.ckf.time,              data.uav_ckf.ckf.EstYawBias, 'm^', 'displayname', 'uav bias ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.uav_ckf.ckf.time,               data.uav_ckf.ckf.EstYaw_gl, 'r^', 'displayname', 'uav yaw ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,                          data.vicon.uav.yaw.gl,       'displayname', 'uav yaw actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      
    hold off;
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth      

    axis([0 180 -5 5])
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -3.5 3.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','uav_position_yaw', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(4); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(4); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  uav z position']); ylabel('z [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_vector = []; 
      try h = plot(data.ugv1Stereo.time,            data.ugv1Stereo.uav.P_ugv(:,3), 'go', 'displayname', 'uav z stereo');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,3), 'bs', 'displayname', 'uav z (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.uav_ckf.ckf.time,            data.uav_ckf.ckf.EstP_gl(:,3), 'r^', 'displayname', 'uav z ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,                data.vicon.uav.position.gl(:,3),       'displayname', 'uav z actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth

    axis([0 180 -5 5])
    
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -0.5 4.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','uav_position_z', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(3); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(3); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  uav y position']); ylabel('y [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,2), 'bs', 'displayname', 'uav y (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.uav_ckf.ckf.time,            data.uav_ckf.ckf.EstP_gl(:,2), 'r^', 'displayname', 'uav y ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,                data.vicon.uav.position.gl(:,2),       'displayname', 'uav y actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth

%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -2.5 2.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
    
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','uav_position_y', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(2); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  figure(2); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
    title([meta.date meta.run '  uav x position']); ylabel('x [m]'); xlabel('time [s]'); grid on
    hold on; 
      linewidth = 3; ii = 0; legend_cells = {}; 
      try h = plot(data.fullSLAM.uav.est.time,  data.fullSLAM.uav.est.p.global(:,1), 'bs', 'displayname', 'uav x (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.uav_ckf.ckf.time,             data.uav_ckf.ckf.EstP_gl(:,1), 'r^', 'displayname', 'uav x ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      try h = plot(data.vicon.time,                 data.vicon.uav.position.gl(:,1),       'displayname', 'uav x actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
      
      
      
    hold off; 
    try legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    clear h ii legend_cells linewidth
  
    axis([0 180 0 5])
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       new_axes = [current_limits(1) current_limits(2) -0.5 4.5];
%       dt = round((current_limits(2)-current_limits(1))/10);
%       current_axes.XTick = [0:dt:new_axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [new_axes(3):0.5:new_axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end; clear dt
%   data.printplots = [data.printplots; current_fig.Number];
%   current_fig.FileName = sprintf('%s_%s','uav_position_x', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%% figure(1); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
figure(1); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
  hold on; 
  linewidth = 3; ii = 0; legend_cells = {}; 
  try title([meta.date meta.run ' ' meta.exp_code]); catch title([meta.date meta.run]); end

  
    % UGV positions
  
    % slam estimates
    try h = plot(data.ugv1_mux.est.Position_gl(:,1), data.ugv1_mux.est.Position_gl(:,2),  'b.', 'displayname', 'ugv1 slam');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try h = plot(data.ugv2_mux.est.Position_gl(:,1), data.ugv2_mux.est.Position_gl(:,2),  'b.', 'displayname', 'ugv2 slam');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    % CKF estimates 
    
    try h = plot(data.ugv1_ckf.ckf.EstP_gl(:,1), data.ugv1_ckf.ckf.EstP_gl(:,2),  'r^', 'displayname', 'ugv1 ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try h = plot(data.ugv2_ckf.ckf.EstP_gl(:,1), data.ugv2_ckf.ckf.EstP_gl(:,2),  'r.', 'displayname', 'ugv1 ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    % Actual positions
    try h = plot( data.vicon.ugv1.position.gl(:,1), data.vicon.ugv1.position.gl(:,2),     'k.', 'displayname', ['ugv1 vicon']);  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    try h = plot( data.vicon.ugv2.position.gl(:,1), data.vicon.ugv2.position.gl(:,2),     'k.', 'displayname', ['ugv2 vicon']);  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    
  % April locations 
    %   for i = [0,1,4,5,8,9] % gazebo
    tag_list = [];
    tag_scatter = [];
      for i = 0:12 % vicon
        apr_str = sprintf( 'april%02d', i );
        try 
          xt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,1);
          yt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,2);
          plot( xt, yt, 'k*', 'displayname', apr_str);
          text(xt,yt,apr_str, 'FontSize',8);
          tag_list = [tag_list; apr_str];
          tag_scatter = [tag_scatter; data.vicon.(matlab.lang.makeValidName(apr_str)).position.vic(1,1:2)];
        catch; end
        try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,1), ...
                  data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,2), ...
                  '.', 'displayname', data.fullSLAM.april.names{i}); catch; end
      end
  

    
%     try h=plot( data.ugv1_ckf.est.Position_gl(:,1), data.ugv1_ckf.est.Position_gl(:,2), 'r.', 'displayname', 'ugv1 ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%     try h=plot(data.ugv1_ckf.ckf.EstP_gl(:,1), data.ugv1_ckf.ckf.EstP_gl(:,2), 'r.', 'displayname', 'ugv1 ckf');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%     try h=plot( data.ugv2_mux.stereo.ugv_sensor.Position_gl(:,1), data.ugv2_mux.stereo.ugv_sensor.Position_gl(:,2), 'mo', 'displayname', ['ugv1/ugv2 sensor']);  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

    % ugv2

 
  
%       try plot(data.ugv1_mux.est.Position_gl(:,1), data.ugv1_mux.est.Position_gl(:,2), '.'); catch; end
  
  
  text(data.vicon.ugv2.position.gl(1,1), data.vicon.ugv2.position.gl(1,2), 'ugv2', 'FontSize',8)
  text(data.vicon.ugv1.position.gl(1,1), data.vicon.ugv1.position.gl(1,2), 'ugv1', 'FontSize',8)

 
  hold off
  grid on
  
  try lgd = legend(legend_cells,'Location','northwest','NumColumns',2); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%   lgd
%   clear h ii legend_cells linewidth

  ylabel('y [m]')
  xlabel('x [m]')
  data.printplots = [data.printplots; current_fig.Number];
  current_fig.FileName = sprintf('%s_%s','landmarks_top_down', meta.run); 
  data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
  clear h1 ii tag_str legend_str 
%   try %current axis
%     current_limits = axis; current_axes = gca; 
%     new_axes = [-1 5 -2.5 2.5];
%     current_axes.XTick = [new_axes(1):1.5:new_axes(2)];
%     current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%     current_axes.YTick = [new_axes(3):2.5:new_axes(4)];
%     current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%   end
%   

axis([-1 5 -2.5 2.5])
  
% % create smaller axes in top right, and plot on it
% axes('Position',[.7 .7 .2 .2])
% box on
% plot(x2,y2)


%   try plot(data.fullSLAM.mux_ugv2.ugvEst.P(:,1), data.fullSLAM.mux_ugv2.ugvEst.P(:,2), '.'); catch; end
% 	try plot(data.ugv1Stereo.Red.solo.P_ugv(:,1), data.ugv1Stereo.Red.solo.P_ugv(:,2), 'r*'); catch; end
% 	try plot(data.ugv1Stereo.Blue.solo.P_ugv(:,1), data.ugv1Stereo.Blue.solo.P_ugv(:,2), 'b*'); catch; end
% 	try plot(data.ugv1Stereo.Green.solo.P_ugv(:,1), data.ugv1Stereo.Green.solo.P_ugv(:,2), 'g*'); catch; end
%   try plot(data.fullSLAM.mux_ugv1.ugvEst.P(:,1), data.fullSLAM.mux_ugv1.ugvEst.P(:,2), '.'); catch; end
        
%   try
%     for i = 1:data.uavSLAM.april.count
%       try plot( data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.names{i})).slam.EstPosition_gl(:,1), ...
%                 data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.names{i})).slam.EstPosition_gl(:,2), ...
%                 'o', 'displayname', data.uavSLAM.april.names{i}); catch; end
%     end
%   catch
%   end
%% print plots
if meta.saveplots
  disp('Saving figures...'); tic
  for idx = 1:length(data.printplots)
    disp(['    Saving ' num2str(idx) ' of ' num2str(length(data.printplots)) ' figures...']);
    current_fig = data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', data.printplots(idx)))).handle;
    current_fig.UserData = sprintf('%sfigs/',meta.saveplotroot); 
    
    printfig(current_fig)
  end
  disp(['  figures saved, ' num2str(toc) ' seconds']);
end
end


function printfig(handle)
    handle.PaperUnits = 'inches';
    handle.PaperPosition = [0 0 15 8];
    disp(['       ' handle.UserData 'eps/' handle.FileName '.eps'])
    print(handle,  [handle.UserData 'eps/' handle.FileName '.eps'],'-depsc', '-r500'); 
    print(handle,  [handle.UserData 'png/' handle.FileName '.png'],'-dpng', '-r500'); 
end

function scaled_data = rescale_celldata(celldata)
  IMMIN = 0;
  IMMAX = 0;
  
  for idx = 1:length(celldata)
    imdata = celldata{idx};
    immin = min(min(imdata));
    immax = max(max(imdata));
  
    if ~isempty(immin)
      IMMIN = min(immin, IMMIN);
    end
    if ~isempty(immax)
      IMMAX = max(immax, IMMAX);
    end
    
  end; clear idx imdata immin immax
  IMSPAN = IMMAX - IMMIN;
%   disp(['IMMIN = ' num2str(IMMIN)])
%   disp(['IMMAX = ' num2str(IMMAX)])
%   disp(['IMSPAN = ' num2str(IMSPAN)])
  
  for idx = 1:length(celldata)
    try scaled_data{idx} = (celldata{idx}-IMMIN)./IMSPAN; catch; end
  end; clear idx IMMIN IMSPAN

end



function M = matrixFromCellData(C)
%% 
% C = data.fullSLAM.slam.x;

M = C{1,1};
  for idx = 2:length(C)
    data_vector = C{idx,1};

    if (length(data_vector)~=size(M,1))
      % then a block or column of zeros is needed
      % figure out where
      add_zeros = abs(size(M,1)-(length(data_vector)));
      if (length(data_vector)<size(M,1))
        % then add to column of new eigen values
          data_vector = [data_vector; zeros(add_zeros,1)];
      else
        % then there are more eigen values now than all previous times
          % add zeros to the bottom of previous matrix
            M = [M; zeros(add_zeros, idx-1)];
      end
    end
    % add eigen values to end of matrix
      M(:, idx) = data_vector;
    
  end
  
end

function plot_cruft()
%% figure(200-100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
for idx = 0:11
    figure(200-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(sprintf( 'Y Errors of april%02d', idx ))
    hold on; ii = 0; legend_vector = []; 
      tag_str = sprintf( 'tag_%02d', idx );
%       try plot( data.vicon.time, ...
%                 data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(:,1), ...
%                 'k.', 'displayname', data.vicon_bodies.aprilnames{1+idx}); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasPosition_gl(:,2) - data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
                'go', 'displayname', 'MeasPosition gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedMeasurement(:,2)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
                'r^', 'displayname', 'PredictedMeasurement gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,2)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
                'bs', 'displayname', 'EstPosition gl'); catch; end

      try 
        plot( data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
              data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).trace, ...
              'k-.', 'displayname', sprintf( 'trace tag%02d', idx ))
  %       PlotSpec{2}(i+1),
      catch
      end

            
    hold off; clear h1 ii tag_str
    grid on
    
        try %current axis
            current_limits = axis; current_axes = gca; 
%             current_axes.XLim = [current_limits(1) current_limits(2)];
            current_axes.XLim = [0 data.fullSLAM.time(end)];
            current_axes.YLim = [-1.5 1.5];
        end
        legend('toggle')
  
  figure(180-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(sprintf( 'X Errors of april%02d', idx ))
    hold on; ii = 0; legend_vector = []; 
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasPosition_gl(:,1) - data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,1), ...
                'go', 'displayname', 'MeasPosition gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedMeasurement(:,1)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,1), ...
                'r^', 'displayname', 'PredictedMeasurement gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,1)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,1), ...
                'bs', 'displayname', 'EstPosition gl'); catch; end
      try 
        plot( data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
              data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).trace, ...
              'k-.', 'displayname', sprintf( 'trace tag%02d', idx ))
  %       PlotSpec{2}(i+1),
      catch
      end

  
    hold off; clear h1 ii tag_str
    grid on
    
        try %current axis
            current_limits = axis; current_axes = gca; 
%             current_axes.XLim = [current_limits(1) current_limits(2)];
            current_axes.XLim = [0 data.fullSLAM.time(end)];
            current_axes.YLim = [-1.5 4.5];
        end
        legend('toggle')
  figure(160-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(sprintf( 'Y positions of april%02d', idx ))
    hold on; ii = 0; legend_vector = []; 
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,1), ...
                'kx', 'displayname', 'ActualPosition gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasPosition_gl(:,1), ...
                'go', 'displayname', 'MeasPosition gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedMeasurement(:,1), ...
                'r^', 'displayname', 'PredictedMeasurement gl'); catch; end
      try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
                data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,1), ...
                'bs', 'displayname', 'EstPosition gl'); catch; end
      try 
        plot( data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
              data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).trace, ...
              'k-.', 'displayname', sprintf( 'trace tag%02d', idx ))
  %       PlotSpec{2}(i+1),
      catch
      end
          hold off; clear h1 ii tag_str
    grid on
    
        try %current axis
            current_limits = axis; current_axes = gca; 
%             current_axes.XLim = [current_limits(1) current_limits(2)];
            current_axes.XLim = [0 data.fullSLAM.time(end)];
            current_axes.YLim = [-1.5 4.5];
        end
%         legend('toggle')
        

end
%% figure(100-80); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
for idx = 0:11
  figure(100-idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
		fig_title = sprintf( 'uav frame: april%02d x,y,z', idx );
		tag_str = sprintf( 'tag_%02d', idx );
		apr_str = sprintf( 'april%02d', idx );
		title(fig_title)

		hold on;
		  try plot( data.vicon.(matlab.lang.makeValidName(apr_str)).detection.time, ...
					data.vicon.(matlab.lang.makeValidName(apr_str)).position.uav(:,1), 'k.', 'displayname', 'vicon x'); catch; end
			try plot( data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
					data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,1), 'bo', 'displayname', 'measured x'); catch; end
		  
			try plot( data.vicon.(matlab.lang.makeValidName(apr_str)).detection.time, ...
					data.vicon.(matlab.lang.makeValidName(apr_str)).position.uav(:,2), 'k.', 'displayname', 'vicon y'); catch; end
			try plot( data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
					data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,2), 'ro', 'displayname', 'measured y'); catch; end
		  
			try plot( data.vicon.(matlab.lang.makeValidName(apr_str)).detection.time, ...
					data.vicon.(matlab.lang.makeValidName(apr_str)).position.uav(:,3), 'k.', 'displayname', 'vicon z'); catch; end
			try plot( data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
					data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,3), 'go', 'displayname', 'measured z'); catch; end
    hold off; 
    grid on
		legend('toggle')
    
% try %current axis
% 		current_limits = axis; current_axes = gca; 
% 		current_axes.XLim = [current_limits(1) current_limits(2)];
% 		current_axes.YLim = [-1.5 1.5];
% end
end
%% figure(60-79); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for idx = 0:11
%   figure(60+idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(sprintf( 'Y Errors of april%02d', idx ))
%     hold on; ii = 0; legend_vector = []; 
%       tag_str = sprintf( 'tag_%02d', idx );
% %       try plot( data.vicon.time, ...
% %                 data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(:,1), ...
% %                 'k.', 'displayname', data.vicon_bodies.aprilnames{1+idx}); catch; end
%       try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
%                 data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.MeasPosition_gl(:,2) - data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
%                 'go', 'displayname', 'MeasPosition gl'); catch; end
%       try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
%                 data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.PredictedMeasurement(:,2)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
%                 'r^', 'displayname', 'PredictedMeasurement gl'); catch; end
%       try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time, ...
%                 data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl(:,2)- data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{1+idx})).position.gl(1,2), ...
%                 'bs', 'displayname', 'EstPosition gl'); catch; end
% 
%       try 
%         plot( data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time, ...
%               data.fullSLAM.trace.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).trace, ...
%               'k-.', 'displayname', sprintf( 'trace tag%02d', idx ))
%   %       PlotSpec{2}(i+1),
%       catch
%       end
% 
%             
%     hold off; clear h1 ii tag_str
%     grid on
%     
%         try %current axis
%             current_limits = axis; current_axes = gca; 
% %             current_axes.XLim = [current_limits(1) current_limits(2)];
%             current_axes.XLim = [0 data.fullSLAM.time(end)];
%             current_axes.YLim = [-1.5 1.5];
%         end
%         legend('toggle')
% end
%% figure(60-79); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for idx = 0:11
%   figure(80+idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(sprintf( 'april%02d', idx ))
%     hold on; ii = 0; legend_vector = []; 
%       tag_str = sprintf( 'tag_%02d', idx );
%       try plot(data.fullSLAM.trace.time, data.fullSLAM.trace.(matlab.lang.makeValidName(tag_str)).trace, '^', 'displayname', tag_str); catch; end
%     hold off; clear h1 ii tag_str
%     grid on
% 
% end
%% figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% title([meta.date meta.run ', uav covariance matrix'])
% 
% row_width = 28;
%   row_1 = data.fullSLAM.slam.augmentedCovariance{1}(1:4,1:4);
%   for idx = 1:row_width
%     row_1 = [row_1 data.fullSLAM.slam.augmentedCovariance{idx}(1:4,1:4)];
%   end
%   uav_cov_image = row_1;
% 
%   try 
%     for covdex = 1:ceil(size(data.fullSLAM.slam.uav.aux_C,3)/row_width)
%       new_row = data.fullSLAM.slam.augmentedCovariance{row_width*covdex}(1:4,1:4);
%       for idx = 1:row_width
%         new_row = [new_row data.fullSLAM.slam.augmentedCovariance{row_width*covdex+idx}(1:4,1:4)];
%       end
%     uav_cov_image = [uav_cov_image; new_row];
%     end
%   catch
%   end
% 
% 
%   immin = min(min(uav_cov_image));
%   immax = max(max(uav_cov_image));
%   imspan = immax - immin;
%   
%   imdata = (uav_cov_image-immin)./imspan;
%   imshow(imdata)
%   
%   clear immin immax imdata imspan uav_cov_image  
%% figure(140-159); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for idx = 0:11
%   figure(140+idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(sprintf( 'april%02d', idx ))
%     hold on; ii = 0; legend_vector = []; 
% 
%       try
%         for i = 1:length(data.vicon_bodies.aprilnames)
%           try plot( data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{i})).position.gl(:,1), ...
%                     data.vicon.(matlab.lang.makeValidName(data.vicon_bodies.aprilnames{i})).position.gl(:,2), ...
%                     '*', 'Color', [0.5,0.5,0.5], 'displayname', data.vicon_bodies.aprilnames{i}); catch; end
%         end
%       catch; end
%     
%       april_str = sprintf( 'april%02d', idx );
%       tag_str = sprintf( 'tag_%02d', idx );
% 
%       try plot( data.vicon.(matlab.lang.makeValidName(april_str)).position.gl(:,1), ...
%                 data.vicon.(matlab.lang.makeValidName(april_str)).position.gl(:,2), ...
%                 'k*', 'displayname', april_str); catch; end
%             
%       try plot( data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.EstPosition_gl(:,1), ...
%                 data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.EstPosition_gl(:,2), ...
%                 'bs', 'displayname', 'fullSLAM'); catch; end
%       try plot( data.uavSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.EstPosition_gl(:,1), ...
%                 data.uavSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.EstPosition_gl(:,2), ...
%                   'r^', 'displayname', 'uavSLAM'); catch; end
%     hold off; clear h1 ii tag_str april_str
%     grid on
%     
%   try %current axis
%     current_limits = axis; current_axes = gca; 
%     current_axes.XLim = [-1.5 3];
%     current_axes.YLim = [-3 1.5];
%   end
% end
%% figure(160-179); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for idx = 0:11
%   figure(160+idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(sprintf( 'april%02d', idx ))
%     hold on; ii = 0; legend_vector = []; 
% 
%     
%       april_str = sprintf( 'april%02d', idx );
%       tag_str = sprintf( 'tag_%02d', idx );
% 
%       try plot(data.vicon.(matlab.lang.makeValidName(april_str)).detection.time, ...
%                data.vicon.(matlab.lang.makeValidName(april_str)).position.uav(:,1), 'k.', 'displayname', sprintf( 'v april%02d x', idx )); catch; end
%       try plot(data.vicon.(matlab.lang.makeValidName(april_str)).detection.time, ...
%                data.vicon.(matlab.lang.makeValidName(april_str)).position.uav(:,2), 'k.', 'displayname', sprintf( 'v april%02d y', idx )); catch; end
%       try plot(data.vicon.(matlab.lang.makeValidName(april_str)).detection.time, ...
%                data.vicon.(matlab.lang.makeValidName(april_str)).position.uav(:,3), 'k.', 'displayname', sprintf( 'v april%02d z', idx )); catch; end
%       try plot(data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
%                data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,1), 'bo', 'displayname', sprintf( 'o tag %02d x', idx ) ); catch; end
%       try plot(data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
%                data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,2), 'ro', 'displayname', sprintf( 'o tag %02d y', idx ) ); catch; end
%       try plot(data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time, ...
%                data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav(:,3), 'ko', 'displayname', sprintf( 'o tag %02d z', idx ) ); catch; end
% 
%       
%     hold off; clear h1 ii tag_str april_str
%     grid on
%     legend('toggle')
% end
%% figure(200-219); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for idx = 0:11
%   figure(200+idx); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(sprintf( 'april%02d', idx ))
%     hold on; ii = 0; legend_vector = []; 
%       april_str = sprintf( 'april%02d', idx );
%       tag_str = sprintf( 'tag_%02d', idx );
% 
%       try
%         length_obs = length(data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time);
%         err = data.vicon.(matlab.lang.makeValidName(april_str)).position.uav(end-(length_obs-1):end,:) - data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.MeasPosition_uav;
%         err_time = data.fullSLAM.april.(matlab.lang.makeValidName(tag_str)).slam.time;
%       catch; end
%       try plot(err_time, err(:,1), 'bo', 'displayname', sprintf( 'err tag %02d x', idx ) ); catch; end
%       try plot(err_time, err(:,2), 'ro', 'displayname', sprintf( 'err tag %02d y', idx ) ); catch; end
%       try plot(err_time, err(:,3), 'ko', 'displayname', sprintf( 'err tag %02d z', idx ) ); catch; end
%            
%            
%     hold off; clear h1 ii tag_str april_str err length_obs err_time
%     grid on
%     legend('toggle')
% end
%% figure(499); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(499); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% frame_start = 1;
% frame_length = 50;
% 
% tag_image = zeros(12,frame_length);
% 
% for i = 1:frame_length
%   frame = frame_start+i;
%   for idx = 1:length(data.fullSLAM.april.TagsInFOV{frame})
%     tag_image(data.fullSLAM.april.TagsInFOV{frame}(idx)+1,i) = 1;
%   end; clear idx
% end; clear i
% imshow(tag_image); clear tag_image
% clear frame_length frame_start tag_image
%% figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% title([meta.date meta.run ', final covariance matrix'])
%   immin = min(min(data.fullSLAM.slam.augmentedCovariance{end}));
%   immax = max(max(data.fullSLAM.slam.augmentedCovariance{end}));
%   imspan = immax - immin;
%   
%   imdata = (data.fullSLAM.slam.augmentedCovariance{end}-immin)./imspan;
%   imshow(imdata)
%   
%   clear immin immax imdata imspan
%% figure(501-5**); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for i = 1:frame_length
%   figure(500+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     
%   try
%     immin = min(min(data.fullSLAM.slam.augmentedCovariance{frame_start+i}));
%     immax = max(max(data.fullSLAM.slam.augmentedCovariance{frame_start+i}));
%     imspan = immax - immin;
% 
%     imdata = (data.fullSLAM.slam.augmentedCovariance{frame_start+i}-immin)./imspan;
% %     if isempty(imdata)
% %       current_fig = gcf;
% %       close(current_fig)
% %     else
%       imshow(imdata)
%       title([meta.date meta.run ', ' num2str(frame_start+i) ', covariance matrix'])
%       xlabel(['t = ' num2str(data.fullSLAM.time(frame_start+i))])
% %     end
%     
% %     try     saveas(gcf, [meta.saveplotroot '/figs/aug_cov_' sprintf('%06d',frame_start+i) '.png']); catch; end
%     
%   catch; end
% end
% clear immin immax imdata imspan
%% figure(601-6**); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for i = 1:frame_length
%   figure(600+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     
%   try
%     immin = min(min(data.fullSLAM.slam.Sk_inv{frame_start+i}));
%     immax = max(max(data.fullSLAM.slam.Sk_inv{frame_start+i}));
%     imspan = immax - immin;
% 
%     imdata = (data.fullSLAM.slam.Sk_inv{frame_start+i}-immin)./imspan;
%     if isempty(imdata)
%       current_fig = gcf;
%       close(current_fig)
%     else
%       imshow(imdata)
%       title([meta.date meta.run ', ' num2str(frame_start+i) ', sk inv matrix'])
%       xlabel(['t = ' num2str(data.fullSLAM.time(frame_start+i))])
%     end
%     
%   catch; end
% end
% clear immin immax imdata imspan
%% figure(701-7**); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% for i = 1:frame_length
%   figure(700+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   try
%     immin = min(min(data.fullSLAM.slam.Subset_P{frame_start+i}));
%     immax = max(max(data.fullSLAM.slam.Subset_P{frame_start+i}));
%     imspan = immax - immin;
% 
%     imdata = (data.fullSLAM.slam.Subset_P{frame_start+i}-immin)./imspan;
%     if isempty(imdata)
%       current_fig = gcf;
%       close(current_fig)
%     else
%       imshow(imdata)
%       title([meta.date meta.run ', ' num2str(frame_start+i) ', Subset P matrix'])
%       xlabel(['t = ' num2str(data.fullSLAM.time(frame_start+i))])
%     end
%   catch; end
% end
% clear immin immax imdata imspan
%% figure(801-8**); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

% for i = 1:frame_length
%   figure(800+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   try
%     imdata = data.fullSLAM.slam.PHt{frame_start+i};
%     immin = min(min(imdata));
%     immax = max(max(imdata));
%     imspan = immax - immin;
% 
%     imdata = (imdata{frame_start+i}-immin)./imspan;
%     if isempty(imdata)
%       current_fig = gcf;
%       close(current_fig)
%     else
%       imshow(imdata)
%       title([meta.date meta.run ', ' num2str(frame_start+i) ',  PHt matrix'])
%       xlabel(['t = ' num2str(data.fullSLAM.time(frame_start+i))])
%     end
%   catch; end
% end
% clear immin immax imdata imspan
%% figure(901-9**); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% scaled_data = rescale_celldata(data.fullSLAM.slam.Kk);
% 
% for i = 1:frame_length
%   figure(900+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   imdata = scaled_data{frame_start+i};
%     if isempty(imdata)
%       current_fig = gcf;
%       close(current_fig)
%     else
%       imshow(imdata)
%       title([meta.date meta.run ', ' num2str(frame_start+i) ',  Kk matrix'])
%       xlabel(['t = ' num2str(data.fullSLAM.time(frame_start+i))])
%     end
%   
% end
% clear scaled_data frame_start imdata
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ UGV 1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% % figure(1007); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(1007); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 1 slam " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv1.position.gl(:,1), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv1 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.fullSLAM.mux_ugv1.mux_time, data.fullSLAM.mux_ugv1.ugvEst.P(:,1), 'ro', ...
%                 'displayname', 'ugv1 x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
% 
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,1), 'Color', [0.25,0.25,0.25], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,1), 'g^', ...
%                 'displayname', 'ugv2 x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         
%         hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(1008); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(1008); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 1 slam " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv1.position.gl(:,2), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv1 y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.fullSLAM.mux_ugv1.mux_time, data.fullSLAM.mux_ugv1.ugvEst.P(:,2), 'ro', ...
%                 'displayname', 'ugv1 y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,2), 'Color', [0.25,0.25,0.25], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,2), 'g^', ...
%                 'displayname', 'ugv2 y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(1009); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(1009); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 1 slam " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv1.position.gl(:,3), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv1 z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.fullSLAM.mux_ugv1.mux_time, data.fullSLAM.mux_ugv1.ugvEst.P(:,3), 'ro', ...
%                 'displayname', 'ugv1 z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,3), 'Color', [0.25,0.25,0.25], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,3), 'g^', ...
%                 'displayname', 'ugv2 z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(1010); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(1010); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 1 slam " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.ugv1_muxRecorder.wheel.time, data.ugv1_muxRecorder.wheel.EstYaw_gl, 'b.', ...
%                 'displayname', 'ugv1 wheel yaw'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.vicon.time, data.vicon.ugv1.yaw.gl, 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv1 yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.fullSLAM.mux_ugv1.mux_time, data.fullSLAM.mux_ugv1.ugvEst.yaw, 'ro', ...
%                 'displayname', 'ugv1 yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
% 
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.yaw.gl, 'Color', [0.25,0.25,0.25], ...
%               'LineWidth', linewidth, 'displayname', 'ugv2 yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstYaw_gl_rad, 'g^', ...
%               'displayname', 'ugv2 yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
%       
%       
% %         try %current axis
% %             current_limits = axis; current_axes = gca; 
% %             axes = [current_limits(1) current_limits(2) -3.5 3.5];
% %             current_axes.XTick = [0:10:axes(2)];
% %             current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
% %             current_axes.YTick = [axes(3):0.5:axes(4)];
% %             current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
% %         end
% % figure(1011); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(1011); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 1 slam " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.ugv1_muxRecorder.wheel.time, data.ugv1_muxRecorder.wheel.yawrate, 'b.', ...
%                 'displayname', 'ugv1 wheel yawrate'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
% %         try h1 = plot(data.ugv1_muxRecorder.wheel.time, data.ugv1_muxRecorder.wheel.twist.angular_xyz(:,3), 'r.', ...
% %                 'displayname', 'ugv1 wheel twist angular z'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv1_muxRecorder.wheel.time, data.ugv1_muxRecorder.wheel.twist.angular_xyz(:,2), 'g.', ...
%                 'displayname', 'ugv1 wheel twist angular z'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv1_muxRecorder.wheel.time, data.ugv1_muxRecorder.wheel.twist.angular_xyz(:,1), 'k.', ...
%                 'displayname', 'ugv1 model noisefree angular z'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%             
%             
%             
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ UGV 2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% % figure(2001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["uav in ugv2 frame" meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.ugv2(:,1), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.P_ugv(:,1), 'go', ...
%                 'displayname', 'stereo x'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(2002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["uav in ugv2 frame" meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.ugv2(:,2), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.P_ugv(:,2), 'go', ...
%                 'displayname', 'stereo y'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(2003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["uav in ugv2 frame" meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.ugv2(:,3), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.P_ugv(:,3), 'go', ...
%                 'displayname', 'stereo z'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(2004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["uav in ugv2 frame" meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.uav.yaw.ugv2, 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.yaw_ugv*pi/180, 'go', ...
%                 'displayname', 'stereo yaw'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2Stereo.time, data.ugv2Stereo.uav.UAV_yaw_var, 'go', ...
%                 'displayname', 'stereo yaw var'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(2007); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2007); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 2 DKF " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,1), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2_muxRecorder.est.time, data.ugv2_muxRecorder.est.Position_gl(:,1), ...
%                 'b.', 'displayname', 'ugv2 x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,1), 'go', ...
%                 'displayname', 'ugv2 x dkf est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% % figure(2008); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2008); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 2 DKF " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,2), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2_muxRecorder.est.time, data.ugv2_muxRecorder.est.Position_gl(:,2), ...
%                 'b.', 'displayname', 'ugv2 y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,2), 'go', ...
%                 'displayname', 'ugv2 y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('y [m]')
%       grid on
% % figure(2009); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2009); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%       title(["ugv 2 DKF " meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.vicon.time, data.vicon.ugv2.position.gl(:,3), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv2 x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2_muxRecorder.est.time, data.ugv2_muxRecorder.est.Position_gl(:,3), ...
%                 'b.', 'displayname', 'ugv2 x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstPosition_gl(:,3), 'go', ...
%                 'displayname', 'ugv2 z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
%       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('z [m]')
%       grid on
% % figure(2010); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%   figure(2010); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     linewidth = 3;
%     title(["ugv 2 DKF " meta.date meta.run])
%     hold on; ii = 0; legend_vector = []; 
%       try h1 = plot(data.vicon.time, data.vicon.ugv2.yaw.gl, 'Color', [0.75,0.75,0.75], ...
%               'LineWidth', linewidth, 'displayname', 'ugv2 yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       try h1 = plot(data.ugv2CKF.dkf.time, data.ugv2CKF.dkf.EstYaw_gl_rad, 'go', ...
%               'displayname', 'ugv2 yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%     hold off; clear h1 ii
%     try columnlegend(3,legend_str); catch; end; clear legend_str
%     ylabel('yaw [rad]')
%     grid on
% %     try %current axis
% %         current_limits = axis; current_axes = gca; 
% %         axes = [current_limits(1) current_limits(2) -3.5 3.5];
% %         current_axes.XTick = [0:10:axes(2)];
% %         current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
% %         current_axes.YTick = [axes(3):0.5:axes(4)];
% %         current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
% %     end
% %% figure(10); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%   figure(10); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%     try 
%       linewidth = 3;
%       title([meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,4), 'd', 'Color', [1,1,1], ...
%                  'MarkerFaceColor', [0,0,0], 'displayname', 'uav yaw goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.vicon.time, data.vicon.uav.yaw.gl, 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%   
%         try h1 = plot(data.ugv1_muxRecorder.stereo.time, data.ugv1_muxRecorder.stereo.yaw_gl_est, 'go', 'MarkerFaceColor', 'g', ...
%                 'displayname', 'uav yaw measured'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         
%         try h1 = plot(data.oneCKF.time, data.oneCKF.uav.Yaw * pi /180, '^', 'Color', [0.5,0.5,0.5], ...
%             'displayname', 'oneCKF yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         
%         try h1 = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.yaw.global, 'bs', ...
%                 'displayname', 'fullSLAM yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         
% %         try h1 = plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.global, 'rs', ...
% %                 'displayname', 'uavSLAM yaw est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
% %         catch; end
%         
% 
%       hold off; clear h1 ii
% %       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('yaw [rad]')
%       grid on
% 
%     try %current axis
%       current_limits = axis; current_axes = gca; 
%       axes = [current_limits(1) current_limits(2) -0.25 0.25];
%       current_axes.XTick = [0:10:axes(2)];
%       current_axes.XLim = [0 current_axes.XTick(end)];
%       current_axes.YTick = [axes(3):0.05:axes(4)];
%       current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%     end
% %   data.printplots = [data.printplots; current_fig.Number];
% %   current_fig.FileName = sprintf('%s_%s','uav_yaw', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%     end
% %% figure(9); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%   figure(9); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%     try
% linewidth = 3;
%       title([meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,3), 'd', 'Color', [1,1,1], ...
%                  'MarkerFaceColor', [0,0,0], 'displayname', 'uav z goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.gl(:,3), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.P_ugv(:,3), 'go', ...
%                 'displayname', 'uav z measured'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,3), 's', 'Color', [0.5,0.5,0.5], ...
%                 'displayname', 'oneCKF z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         
%         try h1 = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,3), 'bs', ...
%                 'displayname', 'fullSLAM z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
% 
%         try h1 = plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,3), 'rs', ...
%                 'displayname', 'uavSLAM z est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%       hold off; clear h1 ii
% %       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% %   data.printplots = [data.printplots; current_fig.Number];
% %   current_fig.FileName = sprintf('%s_%s','uav_yaw', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%     end
% %% figure(8); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%   figure(8); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%     try 
%       linewidth = 3;
%       title([meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.trial.uav.FlyTime, data.trial.uav.Waypoint(:,2), 'd', 'Color', [1,1,1], ...
%                  'MarkerFaceColor', [0,0,0], 'displayname', 'uav y goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.gl(:,2), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.P_ugv(:,2), 'go', ...
%                 'displayname', 'uav y measured'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,2), 's', 'Color', [0.5,0.5,0.5], ...
%                 'displayname', 'oneCKF y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         
%         try h1 = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,2), 'bs', ...
%                 'displayname', 'fullSLAM y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
% 
% %         try h1 = plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,2), 'rs', ...
% %                 'displayname', 'uavSLAM y est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
% %         catch; end
% 
% 
%       hold off; clear h1 ii
% %       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% %   data.printplots = [data.printplots; current_fig.Number];
% %   current_fig.FileName = sprintf('%s_%s','uav_yaw', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%     end
% %% figure(7); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%   figure(7); clf; current_fig = gcf; fprintf('figure(%d) ..\n', current_fig.Number); 
%     try 
%       linewidth = 3;
%       title([meta.date meta.run])
%       hold on; ii = 0; legend_vector = []; 
%         try h1 = plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.P_ugv(:,1), 'go', ...
%                 'displayname', 'uav x measured'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
% %         try h1 = plot(data.oneCKF.time, data.oneCKF.uav.EstPosition_gl(:,1), 's', 'Color', [0.5,0.5,0.5], ...
% %                 'displayname', 'oneCKF x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%         try h1 = plot(data.fullSLAM.mux_ugv1.mux_time, data.fullSLAM.mux_ugv1.ugvEst.stereo_meas_P(:,1), 'go', 'MarkerFaceColor', 'g', ...
%                 'displayname', 'uav yaw measured'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%         catch; end
%         try h1 = plot(data.vicon.time, data.vicon.uav.position.gl(:,1), 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'uav x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
% 
%         try h1 = plot(data.fullSLAM.uav.est.time, data.fullSLAM.uav.est.p.global(:,1), 'bs', ...
%                 'displayname', 'fullSLAM x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%             
% 
% %         try h1 = plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,1), 'rs', ...
% %                 'displayname', 'uavSLAM x est'); ii = ii + 1; legend_str{ii} = h1.DisplayName;catch; end
%       hold off; clear h1 ii
% %       try columnlegend(3,legend_str); catch; end; clear legend_str
%       ylabel('x [m]')
%       grid on
% %   data.printplots = [data.printplots; current_fig.Number];
% %   current_fig.FileName = sprintf('%s_%s','uav_yaw', meta.run); 
%   data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; clear current_fig 
%     end
end

