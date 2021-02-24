%% today
% set(0, 'DefaultFigureVisible', 'off');
% set(0, 'DefaultFigureVisible', 'on');
  set(0, 'DefaultFigureWindowStyle','docked')
  setpref('display','quiet', false);
  meta.date = '20200903/';
  meta.run = '311'; 
  meta.exp_code = 'J';
  meta.saveplots = false;
  meta.columnlegend = false;
  meta.saveirosplots = false;

%% define meta data
  meta.data_paths = {...
    '/mnt/evo2/'; ...
    '/media/benjamin/archive/'; ...
    '/media/benjamin/data/'; ...
    '/home/benjamin/ros/data/';...
    '/mnt/evo2/bad/'; ...
    };
  data_found = false;
    
  for data_idx = 1:size(meta.data_paths,1)
    meta.dataroot = meta.data_paths{data_idx};
    disp(['trying to find ' meta.dataroot meta.date meta.exp_code '/' meta.run])
    if isfolder([meta.dataroot meta.date meta.exp_code '/' meta.run])
      cd ([meta.dataroot meta.date])
      disp(['found ' meta.dataroot meta.date])
      data_found = true;
      meta.datapath = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
      meta.saveplotroot = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
      break;
    end
  end
    
  if (~data_found)
    disp('Could not find data')
    return;
  end
  
%% data = loadData(meta);
  clc; close all
  data = loadJointSlamData(meta);
%%
  if (strcmp(meta.exp_code,"A")||strcmp(meta.exp_code,"B"))
    %%
    data = Code_errors(meta, data, {'ugv1'});
    trial_data = Code_errors(meta, data, {'ugv1'});
  else
    %%
    data = Code_errors(meta, data, {'ugv1', 'ugv2'});
    trial_data = Code_errors(meta, data, {'ugv1', 'ugv2'});
  end
  
%   max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p)
  
  %% Plotting
  [data, meta] = plotJointSlam(data, meta);
  %%
  try plotErrors(data, meta); end

% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title([meta.date meta.run '/' meta.exp_code '  ugv1 y pose']); ylabel('y [m]'); xlabel('time [s]'); grid on
%     hold on; 
%       linewidth = 3; ii = 0; legend_cells = {}; 
%       try h = plot(data.ugv1_mux.est.time,  data.ugv1_mux.est.Position_gl(:,2),  'bs', 'displayname', 'ugv1 y (slam)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
% %       try h = plot(data.ugv1_mux.wheel.time,   data.ugv1_mux.wheel.P_odom(:,2),  'b.', 'displayname', 'ugv1 y (wheel)');  ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       try h = plot(data.ugv1_ckf.dkf.time,  data.ugv1_ckf.dkf.ugv.EstP_gl(:,2),  'm*', 'displayname', 'ugv1 y dkf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       try h = plot(data.ugv1_ckf.ckf.time,      data.ugv1_ckf.ckf.EstP_gl(:,2),  'r^', 'displayname', 'ugv1 y ckf');     ii = ii+1; legend_cells{ii} = h.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       try h = plot(data.vicon.time,           data.vicon.ugv1.position.gl(:,2),        'displayname', 'ugv1 y actual', 'Color', [0.75,0.75,0.75], 'LineWidth', linewidth);   ii = ii+1; legend_cells{ii} = h.DisplayName;  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%     hold off;
%     try legend(legend_cells,'Location','northwest','NumColumns',1); legend('boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%     clear h ii legend_cells linewidth   

%%


