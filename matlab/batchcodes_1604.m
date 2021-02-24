% matlab -nosplash -nojvm -r batchcodes_1604
% rm /mnt/evo2/20200903/batch_data.mat

function [meta, batch_data] = batchcodes_1604(date_in, start, stop)
%% init
  default_colors = getpref('display','default_colors');
  meta.root = '/home/benjamin/ros/src/metahast/matlab'; addpath(genpath([meta.root]));
  setpref('display','quiet', true);
%   setpref('display','quiet', false);
  warning('off','MATLAB:legend:PlotEmpty')

  %% define meta data
  switch nargin
    case 3
      meta.date = [date_in '/'];
    otherwise
%       meta.exp_codes = {'A'};
%       meta.maps = {1; 2; 3; 4};

      meta.date = '20200903/'; %start = 1; stop = 700; % testing all configs A-F, specific maps
      meta.exp_codes = {'A', 'B', 'G', 'H', 'I', 'J'};
%       meta.exp_codes = {'A', 'B', 'G', 'H', 'I'};
% 			meta.exp_codes = {'G', 'H'};
%       meta.maps = {1; 2; 3; 4; 5; 6};

%       meta.list = [101:120]; meta.maps = {1};
%       meta.list = [201:220]; meta.maps = {2};
%       meta.list = [301:320]; meta.maps = {3};
%       meta.list = [401:420]; meta.maps = {4};
%       meta.list = [501:520]; meta.maps = {5};
%       meta.list = [601:620]; meta.maps = {6};
%       meta.list = [101:120, 201:220, 301:320, 401:420]; meta.maps = {1; 2; 3; 4};

%       maps 2 and 5 are essentially redundant with map 4
%       meta.list = [301:320, 401:420, 601:620]; meta.maps = {3; 4; 6};
%       meta.list = [101:105, 301:305, 401:405, 601:605]; meta.maps = {1; 3; 4; 6};
      meta.list = [101:120, 301:320, 401:420, 601:620]; meta.maps = {1; 3; 4; 6};


%       meta.list = [106, 111, 114, 201, 210, 215, 301, 303, 304, 402, 404, 405, 504, 513, 602, 604, 605];
%       meta.list = [110, 210, 310, 410, 510, 610];

      meta.cutoff = 2.1;
      meta.error_thresh = 200;

      meta.saveplots = false;
      meta.writeTables = true;
      
      meta.method = {'ckf',  'slam'};

  end
  %% Games with directories
  meta.data_paths = {...
    '/mnt/evo2/'; ...
    '/mnt/archive/'; ...
    '/media/benjamin/data/'; ...
    '/home/benjamin/ros/data/';...
    };
  data_found = false;

  for data_idx = 1:size(meta.data_paths,1)
    meta.dataroot = meta.data_paths{data_idx};
    disp(['trying to find ' meta.dataroot meta.date])
    if isfolder([meta.dataroot meta.date])
      cd ([meta.dataroot meta.date])
      disp(['found ' meta.dataroot meta.date])
      data_found = true;
      meta.datedata = dir([meta.dataroot meta.date]);
      meta.saveplotroot = [meta.dataroot meta.date 'figs/'];
      if ~exist(meta.saveplotroot, 'dir'); mkdir(meta.saveplotroot); end
      if ~exist([meta.saveplotroot 'e_vs_d'] , 'dir'); mkdir([meta.saveplotroot 'e_vs_d']); end
      break;
    end
  end

  if (~data_found)
    disp('Could not find data')
    return;
  end

  meta.batch_file = [meta.dataroot meta.date 'batch_data.mat'];

  for code_idx = 1:size(meta.exp_codes,2)
    meta.exp_code = meta.exp_codes{code_idx};
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr = {};
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad = {};
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir = [meta.dataroot meta.date meta.exp_code '/figs'];

    if ~exist(batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, 'dir')
        mkdir(batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir); end
  end

  %% load or process data
  if isfile([meta.dataroot meta.date 'batch_data.mat'])
    % actually load in data, either m or mat files
    fprintf('load(%s)\n', meta.batch_file); load(meta.batch_file);
    setpref('display','quiet', false);
  else
  %% process data
  setpref('display','quiet', false);
    for run = meta.list
      meta.run = sprintf('%03d',run);
      for code_idx = 1:size(meta.exp_codes,2)
        try
          setpref('display','quiet', true);
          meta.exp_code = meta.exp_codes{code_idx};
          meta.datapath = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
          clear trial_data
          trial_data = loadJointSlamData(meta);
%           setpref('display','quiet', false);
          switch meta.exp_code
            case {'A', 'B'}
              %%
              trial_data = Code_errors(meta, trial_data, {'ugv1'});
              if ((trial_data.errors.ugv1.net_displacement>meta.cutoff))% && (max(trial_data.errors.ugv1.slam.p_rms_2D)<0.2) )
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr; meta.run];
              else
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad; meta.run];
                meta.baddatapath = [meta.dataroot meta.date meta.exp_code '/bad/'];
%                 try mkdir(meta.baddatapath); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%                 status = movefile( meta.datapath, meta.baddatapath);
              end
            case {'G', 'H', 'I', 'J'}
              %%
              trial_data = Code_errors(meta, trial_data, {'ugv1', 'ugv2'});
              if ((trial_data.errors.ugv1.net_displacement>meta.cutoff))% && (max(trial_data.errors.ugv2.slam.p_rms_2D)<0.2) )
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr; meta.run];
              else
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad; meta.run];
%                 meta.baddatapath = [meta.dataroot meta.date meta.exp_code '/bad/' ];
%                 status = movefile( meta.datapath, meta.baddatapath);
              end
          end
          
          clear trial_data
        catch Mexc_
          disp(['    load data, ' meta.exp_code '.' meta.run ', ' Mexc_.identifier ' :: ' Mexc_.message]);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad; meta.run];
          meta.baddatapath = [meta.dataroot meta.date meta.exp_code '/bad/' meta.run '/'];
          %             status = movefile( meta.datapath, meta.baddatapath);
        end
      end
    end
    %%

    for code_idx = 1:size(meta.exp_codes,2)
      meta.exp_code = meta.exp_codes{code_idx};
      s_good_trials = sprintf('good trials %s (%i): ', meta.exp_code, size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1));
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
        s_good_trials = sprintf('%s %s', s_good_trials, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
      end; clear idx; % disp(s_good_trials)
    end

    disp("bad trials:")
    for code_idx = 1:size(meta.exp_codes,2)
      meta.exp_code = meta.exp_codes{code_idx};
%       s_bad_trials = sprintf('%s_TRIALS="', size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad,1), meta.exp_code);
      s_bad_trials = sprintf('%s_TRIALS="', meta.exp_code);
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad,1)
        s_bad_trials = sprintf('%s %s', s_bad_trials, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad{idx});
      end; clear idx; disp([s_bad_trials '"'])
    end

  if ~usejava('jvm')
    return
  end

  %% now that I have a list of good trials, i can double back and process just those datasets
  for code_idx = 1:size(meta.exp_codes,2)
    if ~usejava('jvm'); setpref('display','quiet', false); end
    meta.exp_code = meta.exp_codes{code_idx};
    for idx = 1:length(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr)
      try
        mat_file = [meta.dataroot meta.date meta.exp_code '/' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} '/errors_' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} '.mat'];
        disp(mat_file);
        load(mat_file);
      catch Mexc_; disp(['    mat_file: ' meta.exp_code '.' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ',' Mexc_.identifier ' :: ' Mexc_.message]); break; end
      % Save End distances for plotting purposes
%       if (strcmp(meta.exp_code,"A")||strcmp(meta.exp_code,"B"))

      switch meta.exp_code
        case {'A', 'B'}
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.enddist(idx,1) = errors.ugv1.dist(end);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))) = errors;
        otherwise
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.enddist(idx,1) = errors.ugv1.dist(end);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.enddist(idx,1) = errors.ugv2.dist(end);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))) = errors;
      end
      %% Save UAV errors to batch structure
      
      try % Save UAV errors to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.end_dist.p(idx,1)         = errors.uav.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.end_angular_dist.p(idx,1) = errors.uav.angular_dist(end);
        
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.end_error.p(idx,1)    = errors.uav.ckf.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.end_error.p(idx,1)   = errors.uav.slam.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.end_error.yaw(idx,1)  = errors.uav.ckf.yaw_abs(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.end_error.yaw(idx,1) = errors.uav.slam.yaw_abs(end);
        % Mean Trajectory Error MTE:
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.MTE_error.p(idx,1)    = errors.uav.ckf.p_mean;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.MTE_error.p(idx,1)   = errors.uav.slam.p_mean;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.MTE_error.yaw(idx,1)  = errors.uav.ckf.yaw_mean;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.MTE_error.yaw(idx,1) = errors.uav.slam.yaw_mean;

        % Save UAV percentages to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.end_percent.p(idx,1)    = 100 * errors.uav.ckf.p_rms(end) / errors.uav.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.end_percent.p(idx,1)   = 100 * errors.uav.slam.p_rms(end) / errors.uav.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.end_percent.yaw(idx,1)  = 100 * errors.uav.ckf.yaw_abs(end) / errors.uav.angular_dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.end_percent.yaw(idx,1) = 100 * errors.uav.slam.yaw_abs(end) / errors.uav.angular_dist(end);
      catch Mexc_; qdisp(['    UAV errors, ' meta.exp_code '.' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
      %% Save UGV1 errors to batch structure
      try % Save UGV1 errors to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.end_dist.p(idx,1)          = errors.ugv1.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.end_angular_dist.p(idx,1)  = errors.ugv1.angular_dist(end);
       
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.goal_rms(idx,1)        = errors.ugv1.end.goal_rms;
        
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.ckf.end_error.p(idx,1)     = errors.ugv1.ckf.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.slam.end_error.p(idx,1)    = errors.ugv1.slam.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.ckf.end_error.yaw(idx,1)   = errors.ugv1.ckf.yaw_abs(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.slam.end_error.yaw(idx,1)  = errors.ugv1.slam.yaw_abs(end);
        % Save UGV1 percentages to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.ckf.end_percent.p(idx,1)     = 100 * errors.ugv1.ckf.p_rms(end) / errors.ugv1.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.slam.end_percent.p(idx,1)    = 100 * errors.ugv1.slam.p_rms(end) / errors.ugv1.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.ckf.end_percent.yaw(idx,1)   = 100 * errors.ugv1.ckf.yaw_abs(end) / errors.ugv1.angular_dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.slam.end_percent.yaw(idx,1)  = 100 * errors.ugv1.slam.yaw_abs(end) / errors.ugv1.angular_dist(end);
      catch Mexc_; qdisp(['    UGV1 errors, ' meta.exp_code '.' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
      %% Save UGV2 errors to batch structure
      try % Save UGV2 errors to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.end_dist.p(idx,1)         = errors.ugv2.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.end_angular_dist.p(idx,1) = errors.ugv2.angular_dist(end);

        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.ckf.end_error.p(idx,1)    = errors.ugv2.ckf.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.slam.end_error.p(idx,1)   = errors.ugv2.slam.p_rms(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.ckf.end_error.yaw(idx,1)  = errors.ugv2.ckf.yaw_abs(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.slam.end_error.yaw(idx,1) = errors.ugv2.slam.yaw_abs(end);
        % Save UGV2 percentages to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.ckf.end_percent.p(idx,1)     = 100 * errors.ugv2.ckf.p_rms(end) / errors.ugv2.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.slam.end_percent.p(idx,1)    = 100 * errors.ugv2.slam.p_rms(end) / errors.ugv2.dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.ckf.end_percent.yaw(idx,1)   = 100 * errors.ugv2.ckf.yaw_abs(end) / errors.ugv2.angular_dist(end);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.slam.end_percent.yaw(idx,1)  = 100 * errors.ugv2.slam.yaw_abs(end) / errors.ugv2.angular_dist(end);
      catch Mexc_; qdisp(['    UGV2 errors, ' meta.exp_code '.' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
      %% Save april errors to batch structure
      try % Save april errors to batch structure
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).april.ckf.end_error.p(idx,1)    = errors.april.ckf.p_rms;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).april.slam.end_error.p(idx,1)   = errors.april.slam.p_rms;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).april.ckf.end_error.yaw(idx,1)  = errors.april.ckf.yaw;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).april.slam.end_error.yaw(idx,1) = errors.april.slam.yaw;
      catch Mexc_; qdisp(['    april errors, ' meta.exp_code '.' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
    end
  end

  %% I still need to spline along the distance vector
  % here i want to calculate the error vs distance at fixed distances for both ugvs
  % dist_ref is the distances that I want to spline the errors to for plotting
  % set common distances for comparing perfomance

  batch_data.ugv.refDistEnd = 1000;
  for code_idx = 1:size(meta.exp_codes,2)
    meta.exp_code = meta.exp_codes{code_idx};
    try
      switch meta.exp_code
        case {'A', 'B', 'G', 'H', 'I', 'J'}
          batch_data.ugv.refDistEnd = min([batch_data.ugv.refDistEnd, min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.enddist)]);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv.refDist = 0:0.01:min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv1.enddist);
        case {'C', 'D', 'E', 'F'}
          batch_data.ugv.refDistEnd = min([batch_data.ugv.refDistEnd, min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.enddist)]);
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv.refDist = 0:0.01:min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).ugv2.enddist);
      end
    catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
  end
  
  batch_data.ugv.refDist = 0:0.01:batch_data.ugv.refDistEnd;
  referenceVector   = batch_data.ugv.refDist; % is not important


  for code_idx = 1:size(meta.exp_codes,2)
    % only work on the good ones
    meta.exp_code = meta.exp_codes{code_idx};
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist = [];
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.ckf = [];
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.slam = [];
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_ckf = [];
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_slam = [];

    % spline to minimum distance for ALL trials
    for idx = 1:length(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr)
      qdisp(['trail idx: ' num2str(idx) ', trial: ' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}])
      switch meta.exp_code
        case {'A', 'B', 'G', 'H', 'I', 'J'}
          timeofvector = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist;
          % slam position:
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist;
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % slam yaw
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.yaw_v_dist;
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % ckf position
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist; % error data
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % ckf yaw
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.yaw_v_dist; % error data
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
        case {'C', 'D', 'E', 'F'}
          timeofvector = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist;
          % slam position:
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist;
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % slam yaw
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.yaw_v_dist;
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % ckf position
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist; % error data
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
          % ckf yaw
          vectorToBeSplined = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.yaw_v_dist; % error data
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.yaw = nD_interp1(timeofvector, vectorToBeSplined, batch_data.ugv.refDist);
      end

      if (max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p) > meta.error_thresh)
        qdisp(['bad trial:: ' sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ) ', max slam error:' num2str(max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p))])
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad; batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}];
        qdisp(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad)

        batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist; idx];
      else
        % add to batch for calculating mean
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.ckf  = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.ckf ; ...
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p'];
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.slam = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.slam; ...
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p'];

        batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_ckf  = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_ckf;  ...
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.yaw'];
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_slam = [batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_slam; ...
          batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.yaw'];
      end

    end

    % clear bad trials from good list
    s_bad_trials = sprintf('bad trials %s (%i): ', meta.exp_code, size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jbad,1));
      for bad_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist,1)
        bad_str = batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist(bad_idx)};
        s_bad_trials = sprintf('%s %s,', s_bad_trials, bad_str);
        meta.datapath = [meta.dataroot meta.date meta.exp_code '/' bad_str '/'];
        meta.baddatapath = [meta.dataroot meta.date meta.exp_code '/bad/' bad_str '/'];
%         status = movefile( meta.datapath, meta.baddatapath);
      end; clear bad_idx; %disp(s_bad_trials)
    batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr(batch_data.(matlab.lang.makeValidName(meta.exp_code)).badlist) = [];

    s_good_trials = sprintf('good trials %s (%i): ', meta.exp_code, size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1));
      for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
        s_good_trials = sprintf('%s %s,', s_good_trials, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
      end; clear idx; disp(s_good_trials)

      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_ckf  = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.ckf);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_slam = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.slam);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.RMSE_ckf  = mean(mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.ckf));
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.RMSE_slam = mean(mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.slam));

      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_yaw_ckf   = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_ckf);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_yaw_slam  = mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_slam);
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.RMSE_yaw_ckf   = mean(mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_ckf));
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.RMSE_yaw_slam  = mean(mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.yaw_slam));

  end

	%% Time domain error averages:
	% init variable containers
  for code_idx = 1:size(meta.exp_codes,2) % For each experiment type
    meta.exp_code = meta.exp_codes{code_idx}; % For each map

		% init exp.map.data
		for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.ckf.p.rms = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.slam.p.rms = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.ckf.yaw.abs = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.slam.yaw.abs = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.p.rms_2D = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.slam.p.rms_2D = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.yaw.abs = [];
			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.slam.yaw.abs = [];

      batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))) = [];
      batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))) = [];
      
			switch meta.exp_code
				case {'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'}
					batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.p.rms_2D = [];
					batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.slam.p.rms_2D = [];
					batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.yaw.abs = [];
					batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.slam.yaw.abs = [];
			end
		end
	end

	% now actually process data
	for code_idx = 1:size(meta.exp_codes,2) % For each experiment type
		meta.exp_code = meta.exp_codes{code_idx}; % For each map

    % process error vs time data
    for idx = 1:length(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr)
      clear errors;
      errors = batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} )));

      if (isempty(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.p.rms)) % then this is the first set of data
				batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).time_mat  = errors.time';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.p.gl  = errors.clip.uav.p.gl';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.p.rms  = errors.clip.uav.ckf.p.rms';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.p.rms = errors.clip.uav.slam.p.rms';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.yaw.abs  = errors.clip.uav.ckf.yaw.abs';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.yaw.abs = errors.clip.uav.slam.yaw.abs';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.p.rms_2D  = errors.clip.ugv1.ckf.p.rms_2D';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.p.rms_2D = errors.clip.ugv1.slam.p.rms_2D';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.yaw.abs  = errors.clip.ugv1.ckf.yaw.abs';
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.yaw.abs = errors.clip.ugv1.slam.yaw.abs';
        
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))) = errors.ugv1.end.goal_rms;
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))) = errors.april.offset.min;
        
        switch meta.exp_code
          case {'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'}
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.p.rms_2D  = errors.clip.ugv2.ckf.p.rms_2D';
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.p.rms_2D = errors.clip.ugv2.slam.p.rms_2D';
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.yaw.abs  = errors.clip.ugv2.ckf.yaw.abs';
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.yaw.abs = errors.clip.ugv2.slam.yaw.abs';
        end

      else % we're adding data
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).time_mat = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).time_mat,  errors.time');
				batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.p.gl = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.p.gl,  errors.clip.uav.p.gl');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.p.rms  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.p.rms,  errors.clip.uav.ckf.p.rms');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.p.rms = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.p.rms, errors.clip.uav.slam.p.rms');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.yaw.abs  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.ckf.yaw.abs,  errors.clip.uav.ckf.yaw.abs');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.yaw.abs = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).uav.slam.yaw.abs, errors.clip.uav.slam.yaw.abs');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.p.rms_2D  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.p.rms_2D,  errors.clip.ugv1.ckf.p.rms_2D');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.p.rms_2D = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.p.rms_2D, errors.clip.ugv1.slam.p.rms_2D');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.yaw.abs  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.yaw.abs,  errors.clip.ugv1.ckf.yaw.abs');
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.yaw.abs = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.slam.yaw.abs, errors.clip.ugv1.slam.yaw.abs');

        batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))) = ...
          matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))), errors.ugv1.end.goal_rms);
        batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))) = ...
          matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))), errors.april.offset.min);

        switch meta.exp_code
          case {'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'}
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.p.rms_2D  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.p.rms_2D,  errors.clip.ugv2.ckf.p.rms_2D');
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.p.rms_2D = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.p.rms_2D, errors.clip.ugv2.slam.p.rms_2D');
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.yaw.abs  = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.ckf.yaw.abs,  errors.clip.ugv2.ckf.yaw.abs');
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.yaw.abs = matrixify_error_vectors(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv2.slam.yaw.abs, errors.clip.ugv2.slam.yaw.abs');
        end
      end
    end
  end

	% calculate means and time
	% clip = rmfields(clip, {'timeOfRefVector', 'referenceVector'})
	for map_idx = 1:size(meta.maps,1)
		map_code = meta.maps{map_idx};
		for code_idx = 1:size(meta.exp_codes,2) % For each experiment type
			exp_code = meta.exp_codes{code_idx}; % For each map

			%calculate means:
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.p.gl  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.p.gl);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.ckf.p.rms    = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.ckf.p.rms);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.slam.p.rms   = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.slam.p.rms);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.ckf.yaw.abs  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.ckf.yaw.abs);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.slam.yaw.abs = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).uav.slam.yaw.abs);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.p.rms_2D  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.p.rms_2D);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.p.rms_2D = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.slam.p.rms_2D);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.yaw.abs   = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.yaw.abs);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.yaw.abs  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.slam.yaw.abs);
			switch exp_code
			case {'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'}
				batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.p.rms_2D  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.p.rms_2D);
				batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.p.rms_2D = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.slam.p.rms_2D);
				batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.yaw.abs   = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.yaw.abs);
				batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.yaw.abs  = mean(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.slam.yaw.abs);
			end
			% batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))) = rmfields(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))), {'time_mat'});

			% Adjust Time
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time = ...
						batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time_mat(1,end-size(batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.p.gl,2)+1:end);
			batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time = batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time - ...
																														batch_data.(matlab.lang.makeValidName(exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time(1);


		end


  end

%% Calculate end-of-trial metrics for UGV1 vs goal location
  for code_idx = 1:size(meta.exp_codes,2) % For each experiment type
    meta.exp_code = meta.exp_codes{code_idx}; % For each map
    for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      batch_data.end.goal.mean(code_idx, map_idx) = 100*mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.goal.max(code_idx, map_idx) = 100*max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.goal.min(code_idx, map_idx) = 100*min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.goal.std(code_idx, map_idx) = 100*std(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.ugv1.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));

      batch_data.end.april.mean(code_idx, map_idx) = 100*mean(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.april.max(code_idx, map_idx) = 100*max(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.april.min(code_idx, map_idx) = 100*min(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
      batch_data.end.april.std(code_idx, map_idx) = 100*std(batch_data.(matlab.lang.makeValidName(meta.exp_code)).end.april_min.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))));
    
    end
  end
  
  
  %% find the reference ugv trajectories for each map type
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
          case {'A', 'B', 'G', 'H', 'I', 'J'}
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
          case {'A', 'B', 'G', 'H', 'I', 'J'}
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

%%
  setpref('display','quiet', false);
% setpref('display','quiet', true);
  %% Save data
  fprintf('save(%s), meta, batch_data,-v7.3)\n', meta.batch_file);
  save([meta.dataroot meta.date 'batch_data.mat'], 'meta', 'batch_data','-v7.3')
  end

%% Find average of errors at end of all trials
% batch_data.G.errors_101.uav.dist(end)
% batch_data.G.errors_101.ugv1.dist(end)
% batch_data.G.errors_101.ugv2.dist(end)
% batch_data.G.errors_101.uav.slam.p_rms(end)
% batch_data.G.errors_101.ugv1.slam.p_rms(end)
% batch_data.G.errors_101.ugv2.slam.p_rms(end)


METHOD = {'CKF ', 'SLAM'};
method = {'ckf',  'slam'};

batch_data.end_errors.all_errors.dist = []; batch_data.end_errors.all_errors.ckf = []; batch_data.end_errors.all_errors.slam = [];

for code_idx = 1:size(meta.exp_codes,2)
  for method_idx = 1:size(METHOD,2)
				switch meta.exp_codes{code_idx}
					case {'A', 'B'}
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).dist = ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).uav.end_dist.p + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv1.end_dist.p;
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).(matlab.lang.makeValidName(method{method_idx})).error = ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_error.p  + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_error.p;
          case {'G', 'H', 'I', 'J'}
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).dist = ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).uav.end_dist.p + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv1.end_dist.p + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv2.end_dist.p;
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).(matlab.lang.makeValidName(method{method_idx})).error = ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_error.p  + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_error.p + ...
              batch_data.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ugv2.(matlab.lang.makeValidName(method{method_idx})).end_error.p;
        end % switch EXP_CODES{code_idx}
  end % for method_idx = 1:size(METHOD,2)
%   mean(batch_data.end_errors.H.error.ckf ./ batch_data.end_errors.H.dist)
  batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ckf.percent = ...
    100*mean(batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ckf.error ./ ...
         batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).dist);
  batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).slam.percent = ...
    100*mean(batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).slam.error ./ ...
         batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).dist);
  batch_data.end_errors.all_errors.dist = [batch_data.end_errors.all_errors.dist; batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).dist]; 
  batch_data.end_errors.all_errors.ckf  = [batch_data.end_errors.all_errors.ckf;  batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).ckf.error]; 
  batch_data.end_errors.all_errors.slam = [batch_data.end_errors.all_errors.slam; batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).slam.error]; 
  

end % for code_idx = 1:size(EXP_CODES,2)

batch_data.end_errors.all_errors.ckf_percent  = 100* mean(batch_data.end_errors.all_errors.ckf ./ batch_data.end_errors.all_errors.dist); 
batch_data.end_errors.all_errors.slam_percent = 100* mean(batch_data.end_errors.all_errors.slam ./ batch_data.end_errors.all_errors.dist); 


 %% write table of data
  if (meta.writeTables)
    batch_data.EXP_CODES = meta.exp_codes;
%     write_error_table_tex(batch_data);
%       write_EvD_table_tex(batch_data);
%       write_EvT_table_tex(batch_data, meta);
    write_EvMap_table_tex(batch_data, meta);
%      write_RMSE_table_tex(batch_data);
  end
  
  %% plotting
  close all
%   batch_data.RAL_axes = [0 8 0 50];
  batch_data.RAL_axes = [0 8 0 25];
  batch_data.EvT_axes = [0 150 0 60];
%   batch_data.EvT_axes = [0 batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time(end) 0 25];
  if ~getpref('display', 'quiet')
		%   set(0,'DefaultFigureVisible','on');
    set(0,'DefaultFigureVisible','off')
%% figure(1); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
	    linewidth = 1.5;
	    figure(1); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
	      title(['All Experiments , error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [cm]')
	      hold on

	      for case_idx = 1:size(meta.exp_codes,2)
	        meta.exp_code = meta.exp_codes{case_idx};
					% qdisp(sprintf('case_%s', meta.exp_codes{case_idx}))
	        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
	          switch meta.exp_codes{case_idx}
	            case {'A', 'B', 'G', 'H', 'I', 'J'}
	              h1 = plot( ...
	                    batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
	                'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
	              h2 = plot( ...
	                    batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
	                'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
	            case {'C', 'D', 'E', 'F'}
	              h1 = plot( ...
	                    batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
	                'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
	              h2 = plot( ...
	                    batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
	                'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
	          end
	        end
	      end
	      hold off
	      legend(legend_handles, legend_names); legend('location', 'NorthWest')
	      grid on
	      batch_data.all_max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
        current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
	      current_fig.FileName = sprintf('%s', '01_all_experiments_all_errors_vs_all_distances');
        current_fig.PaperPosition = [0 0 15 5];
				% printfig(meta, current_fig)
%% figure(2); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
    % plot everything
    linewidth = 1;
    figure(2); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
      title(['All Experiments , error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [angle]')
      hold on

      for case_idx = 1:size(meta.exp_codes,2)
        meta.exp_code = meta.exp_codes{case_idx};
				% qdisp(sprintf('case_%s', meta.exp_codes{case_idx}))
        for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
          switch meta.exp_codes{case_idx}
            case {'A', 'B', 'G', 'H', 'I', 'J'}
              h1 = plot( ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.yaw_abs, ...
                'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
              h2 = plot( ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.yaw_abs, ...
                'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
            case {'C', 'D', 'E', 'F'}
              h1 = plot( ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.yaw_abs, ...
                'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
              h2 = plot( ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
                batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.yaw_abs, ...
                'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
          end
        end
      end
      hold off
      legend(legend_handles, legend_names); legend('location', 'NorthWest')
      grid on
      batch_data.all_max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
      current_fig.PaperPosition = [0 0 15 5];
      current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
      current_fig.FileName = sprintf('%s', '01_all_experiments_all_errors_vs_all_distances');

%       printfig(meta, current_fig)
%% figure(XZX); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
  %     set(0,'DefaultFigureVisible','off')
  set(0, 'DefaultAxesFontSize', 36)
  set(0, 'DefaultTextFontSize', 36)

  meta.method = {'ckf',  'slam'};
  
    for case_idx = 1:size(meta.exp_codes,2)
      meta.exp_code = meta.exp_codes{case_idx};
      switch meta.exp_codes{case_idx}
        case 'A'
          meta.fig_base_n = 100;
          exp_tex = '_1C^H';
					strategy = '1CH';
        case 'B'
          meta.fig_base_n = 200;
          exp_tex = '_1C^L';
					strategy = '1CL';
        case {'C', 'G'}
          meta.fig_base_n = 300;
          exp_tex = '_2C^H_G';
					strategy = '2CHG';
        case {'D', 'H'}
          meta.fig_base_n = 400;
          exp_tex = '_2C^H_R';
					strategy = '2CHR';
        case {'E', 'I'}
          meta.fig_base_n = 500;
          exp_tex = '_2C^L_G';
					strategy = '2CLG';
        case {'F', 'J'}
          meta.fig_base_n = 600;
          exp_tex = '_2C^L_R';
					strategy = '2CLR';
        otherwise
          return
      end
%% figure(15X-65X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
        for map_idx = 1:size(meta.maps,1)
        map_code = meta.maps{map_idx};
        figure(meta.fig_base_n+50+map_code); current_fig = gcf; clf; fprintf('figure(%d)\n', current_fig.Number);
					linewidth = 4;
				
          ax1 = gca; % current axes
						ax1.XColor = 'k';ax1.YColor = 'k'; % axis color
	          ax1_pos = ax1.Position; % position of first axes
						ax1.XLim = [batch_data.EvT_axes(1) batch_data.EvT_axes(2)];
						ax1.YLim = [batch_data.EvT_axes(3) batch_data.EvT_axes(4)];
            ax1.YLabel.String = 'error [cm]';
% 						ax1.XLabel.String = 'time [s]'; 
%             title(sprintf( 'Mean all vehicles rms error vs time, %s, map_%i', exp_tex, map_code));

          switch meta.exp_codes{case_idx}
            case {'A', 'B'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.ckf.p.rms + ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.p.rms_2D, ...
											'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', 'mean ckf');
	                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
	                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.slam.p.rms + ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.p.rms_2D, ...
											'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', 'mean slam');
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

            case {'G', 'H', 'I', 'J'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.ckf.p.rms + ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.p.rms_2D+ ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.p.rms_2D, ...
                    'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', 'mean ckf');
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.slam.p.rms + ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.p.rms_2D+ ...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.p.rms_2D, ...
                      'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', 'mean slam');
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          end % switch meta.exp_codes{case_idx}
          hold off
          grid on
          % axis(batch_data.EvT_axes)

          try  legend(ax1, [L1 L2], {L1.DisplayName, L2.DisplayName},'Location','northwest','NumColumns',1); legend(ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

          if (meta.saveplots)
            current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
            current_fig.FileName = sprintf('map_%i_%s_rms_mean_%s', map_code, meta.exp_code, strategy);
            current_fig.PaperPosition = [0 0 15 5];
            try printfig(meta, current_fig); close(current_fig); catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          end
          
        end %for map_idx = 1:size(meta.maps,1)
%% figure(17X-67X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
      for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      figure(meta.fig_base_n+70+map_code); current_fig = gcf; clf; fprintf('figure(%d)\n', current_fig.Number);
        linewidth = 4;
        subplot(2, 1, 1);	

        for method_idx = 1:size(meta.method,2)
          subplot(2, 1, method_idx); grid on
          ax.(matlab.lang.makeValidName(method{method_idx})).ax1 = gca; % current axes
            ax.(matlab.lang.makeValidName(method{method_idx})).title = sprintf( 'Mean vehicles rms error vs time, %s, map_%i', exp_tex, map_code);
% 						ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XLabel.String = 'time [s]'; 
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YLabel.String = 'error [cm]';
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XColor = 'k';
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YColor = 'k'; % axis color
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XLim = [batch_data.EvT_axes(1) batch_data.EvT_axes(2)];
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YLim = [0 40];
            title(ax.(matlab.lang.makeValidName(method{method_idx})).title);
          
          switch meta.exp_codes{case_idx}
            case {'A', 'B'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', ['uav ' meta.method{method_idx}]);
                      meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
                      meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', ['ugv1 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, [L1 L2], {L1.DisplayName, L2.DisplayName},'Location','northwest','NumColumns',1); 
                legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

            case {'G', 'H', 'I', 'J'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
                    'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', ['uav ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', ['ugv1 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L3 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{1}, 'displayname', ['ugv2 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L3;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L3.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, [L1 L2 L3], {L1.DisplayName, L2.DisplayName, L3.DisplayName},'Location','northwest','NumColumns',1); 
                legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
          end % switch meta.exp_codes{case_idx}

        end % for method_idx = 1:size(meta.method,2)


        if (meta.saveplots)
          current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
          current_fig.FileName = sprintf('uxv_%s_map_%i_%s', meta.exp_code, map_code, strategy);
          current_fig.PaperPosition = [0 0 15 15];
          try printfig(meta, current_fig); close(current_fig);
          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        end


      end %for map_idx = 1:size(meta.maps,1)
%% figure(18X-69X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
      for map_idx = 1:size(meta.maps,1)
        map_code = meta.maps{map_idx};
        for method_idx = 1:size(meta.method,2)
          switch method{method_idx}
            case {'ckf'}
              figure(meta.fig_base_n+80+map_code); current_fig = gcf; clf; fprintf('figure(%d)\n', current_fig.Number);
            case {'slam'}
              figure(meta.fig_base_n+90+map_code); current_fig = gcf; clf; fprintf('figure(%d)\n', current_fig.Number);
          end % switch method{method_idx}

          ax.(matlab.lang.makeValidName(method{method_idx})).ax1 = gca; % current axes
            ax.(matlab.lang.makeValidName(method{method_idx})).title = sprintf( 'Mean vehicles rms error vs time, %s, map_%i', exp_tex, map_code);
% 						ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XLabel.String = 'time [s]'; 
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YLabel.String = 'error [cm]';
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XColor = 'k';
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YColor = 'k'; % axis color
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.XLim = [batch_data.EvT_axes(1) batch_data.EvT_axes(2)];
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YLim = [0 25];
            ax.(matlab.lang.makeValidName(method{method_idx})).ax1.YTick = [0:5:25];
            
%             title(ax.(matlab.lang.makeValidName(method{method_idx})).title);

          switch meta.exp_codes{case_idx}
            case {'A', 'B'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', ['uav ' meta.method{method_idx}]);
                      meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
                      meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', ['ugv1 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, [L1 L2], {L1.DisplayName, L2.DisplayName},'Location','northwest','NumColumns',1); 
                legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
            case {'G', 'H', 'I', 'J'}
              try L1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
                    'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', ['uav ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', ['ugv1 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try L3 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
                      100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D, ...
                      'Parent', ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'LineWidth', linewidth, 'Color',default_colors{1}, 'displayname', ['ugv2 ' meta.method{method_idx}]);
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L3;
                    meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L3.DisplayName;
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              try legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, [L1 L2 L3], {L1.DisplayName, L2.DisplayName, L3.DisplayName},'Location','northwest','NumColumns',1); 
                legend(ax.(matlab.lang.makeValidName(method{method_idx})).ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
          end % switch meta.exp_codes{case_idx}
        grid on
				try 
          current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
          current_fig.FileName = sprintf('map_%i_%s_%s_mean_%s', map_code, meta.exp_code, method{method_idx}, strategy);
          current_fig.PaperPosition = [0 0 15 5];
          printfig(meta, current_fig); close(current_fig); 
        catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

        end % for method_idx = 1:size(meta.method,2)

      end %for map_idx = 1:size(meta.maps,1)
    end

%% figure(XX); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
    for map_idx = 1:size(meta.maps,1)
      map_code = meta.maps{map_idx};
      figure(10+map_code); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
        xlabel('distance traveled [m]'); ylabel('error [cm]')
        title(['Map ' sprintf( '%i', map_code) ', error vs distance traveled']);
        for case_idx = 1:size(meta.exp_codes,2)
          meta.exp_code = meta.exp_codes{case_idx};
          % disp(['Experiment, ' meta.exp_code ', map, ' num2str(map_code)])
          hold on
            try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec, ...
                  100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_ckf,...
                  'r', 'displayname', 'mean ckf', 'LineWidth', 3);
                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
            catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
            try h2 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec, ...
                  100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_slam, ...
                  'b', 'displayname', 'mean slam', 'LineWidth', 3);
                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
            catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          hold off
          grid on
          axis([0 5 0 15])
        end
    end
%% figure(XXXX); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
    % by experiment type
    for case_idx = 1:size(meta.exp_codes,2)
      meta.exp_code = meta.exp_codes{case_idx};
      switch meta.exp_codes{case_idx}
        case 'A'
          meta.fig_base_n = 1000;
        case 'B'
          meta.fig_base_n = 2000;
        case {'C', 'G'}
          meta.fig_base_n = 3000;
        case {'D', 'H'}
          meta.fig_base_n = 4000;
        case {'E', 'I'}
          meta.fig_base_n = 5000;
        case {'F', 'J'}
          meta.fig_base_n = 6000;
        otherwise
          disp('meta.exp_codes is confused... quitting')
          return
      end

      switch meta.exp_codes{case_idx}
        case {'A', 'B', 'G', 'H', 'I', 'J'}
        %% figure(X000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
          figure(meta.fig_base_n); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
            title(['Experiment ' meta.exp_code ', error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [cm]')
            try
            for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
              hold on
                h1 = plot( ...
                  batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                  100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
                  'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                h2 = plot( ...
                  batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                  100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
                  'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
              hold off
            end
            catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
            legend(legend_handles, legend_names); legend('location', 'NorthWest')
            grid on
            batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
            current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
            current_fig.FileName = sprintf('%s%s', meta.exp_code, '_all_errors_vs_all_distances');
            current_fig.PaperPosition = [0 0 15 5];
%         try printfig(meta, current_fig); end
        %% figure(X001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
          figure(meta.fig_base_n+1); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
            title(['Experiment ' meta.exp_code ', error vs distance, matched distance traveled']);  xlabel('distance traveled [m]'); ylabel('error [cm]')
            try
            for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
              hold on
                h1 = plot( ...
                    batch_data.ugv.refDist, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p, ...
                    'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                h2 = plot( ...
                    batch_data.ugv.refDist, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p, ...
                    'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
                hold off
            end
            catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
            legend(legend_handles, legend_names); legend('location', 'NorthWest')
            grid on
            axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
            current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
            current_fig.FileName = sprintf('%s%s', meta.exp_code, '_all_errors_vs_capped_distances');
            current_fig.PaperPosition = [0 0 15 5];
%         try printfig(meta, current_fig); end
        %% figure(X002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
          figure(meta.fig_base_n+2); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
%             title(['Experiment ' meta.exp_code ', error vs distance']);
            xlabel('distance traveled [m]'); ylabel('error [cm]')
            hold on
            linewidth = 3;
            try
              h1 = plot(batch_data.ugv.refDist, 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_ckf, ...
                'r', 'displayname', 'ckf mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
              h1 = plot(batch_data.ugv.refDist, 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_slam, ...
                'b', 'displayname', 'slam mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};

              hold off
              legend(legend_handles, legend_names); legend('location', 'NorthWest')
              grid on
  %             axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
%               axis(batch_data.RAL_axes)
            catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
        current_fig.FileName = sprintf('%s%s', meta.exp_code, '_average_errors_vs_capped_distances');
        current_fig.UserData = sprintf('%s', [meta.dataroot meta.date 'figs']);
        current_fig.PaperPosition = [0 0 15 5];
%         try printfig(meta, current_fig); end
        %% figure(X100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
          for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
            figure(meta.fig_base_n+100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
              title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [cm]')
              hold on
              linewidth = 3;
                try
                h1 = plot( ...
                  batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.ckf.p_v_dist, ...
                    'r', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                    legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                  h2 = plot( ...
                    batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.dist, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv1.slam.p_v_dist, ...
                    'b', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                  legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
                hold off
                grid on
                legend(legend_handles, legend_names); legend('location', 'NorthWest')
                axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

              current_fig.UserData = sprintf('%s/all_evd', [meta.dataroot meta.date 'figs']);
              current_fig.FileName = sprintf('%s_%s', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
              current_fig.PaperPosition = [0 0 15 5];
%             try printfig(meta, current_fig);
              close(current_fig);
%           end
          end

        otherwise
          disp('plotting is confused... quitting')
          return
      end

        %% figure(X700+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
        switch meta.exp_codes{case_idx}
        case {'A', 'B'}
          for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
            figure(meta.fig_base_n+500+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
              title(sprintf( 'Experiment %s.%s all vehicles error vs time', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} )); xlabel('time [s]'); ylabel('error [cm]')
              hold on
              linewidth = 2;
                try
                  h1 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.ckf.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.ckf.p.rms, ...
                        'Color',default_colors{3}, 'displayname', sprintf( 'ugv1 ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                  h2 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.slam.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.slam.p.rms, ...
                        'Color',default_colors{1}, 'displayname', sprintf( 'ugv1 slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
                hold off
                grid on
                legend(legend_handles, legend_names); legend('location', 'NorthWest')
                axis(batch_data.EvT_axes)
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

              current_fig.UserData = sprintf('%s/all_evt', [meta.dataroot meta.date 'figs']);
              current_fig.FileName = sprintf('%s_%s', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
              current_fig.PaperPosition = [0 0 15 5];
%             try printfig(meta, current_fig);
            close(current_fig);
%           end
          end
        case {'G', 'H', 'I', 'J'}
          for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
            figure(meta.fig_base_n+500+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%               title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']);
              title(sprintf( 'Experiment %s.%s all vehicles error vs time', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} )); xlabel('time [s]'); ylabel('error [cm]')
              hold on
              linewidth = 2;
                try
                  h1 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.ckf.p.rms_2D, ...
                        'Color',default_colors{3}, 'displayname', 'ugv1 ckf', 'LineWidth', linewidth);
                        legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                  h2 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.slam.p.rms_2D, ...
                        'Color',default_colors{1}, 'displayname', 'ugv1 slam', 'LineWidth', linewidth);
                        legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
                  h3 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv2.ckf.p.rms_2D, ...
                        'Color',default_colors{2}, 'displayname', 'ugv2 ckf', 'LineWidth', linewidth);
                        legend_handles(3,1) = h3; legend_names{3,1} = h3.DisplayName;
                  h4 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv2.slam.p.rms_2D, ...
                        'Color',default_colors{4}, 'displayname', 'ugv2 slam', 'LineWidth', linewidth);
                        legend_handles(4,1) = h4; legend_names{4,1} = h4.DisplayName;
                hold off
                grid on
                legend(legend_handles, legend_names); legend('location', 'NorthWest')
                axis(batch_data.EvT_axes)
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
              current_fig.UserData = sprintf('%s/all_evt', [meta.dataroot meta.date 'figs']);
              current_fig.FileName = sprintf('%s_%s', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
              current_fig.PaperPosition = [0 0 15 5];
%             try printfig(meta, current_fig);
              close(current_fig);
%             end
          end
        otherwise
          disp('plotting is confused... quitting')
          return
      end

        %% figure(X700+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
        switch meta.exp_codes{case_idx}
        case {'A', 'B'}
          for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
            figure(meta.fig_base_n+700+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
              title(sprintf( 'Experiment %s.%s all vehicles sum(error) vs time', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} )); xlabel('time [s]'); ylabel('error [cm]')
              hold on
              linewidth = 2;
                try
                  h1 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.ckf.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.ckf.p.rms, ...
                        'Color',default_colors{3}, 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                  h2 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.slam.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.slam.p.rms, ...
                        'Color',default_colors{1}, 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
                hold off
                grid on
                legend(legend_handles, legend_names); legend('location', 'NorthWest')
                axis(batch_data.EvT_axes)
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

              current_fig.UserData = sprintf('%s/all_evt', [meta.dataroot meta.date 'figs']);
              current_fig.FileName = sprintf('%s_%s', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
              current_fig.PaperPosition = [0 0 15 5];
            % try printfig(meta, current_fig); close(current_fig); catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          end
        case {'G', 'H', 'I', 'J'}
          for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
            figure(meta.fig_base_n+700+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
              title(sprintf( 'Experiment %s.%s all vehicles sum(error) vs time', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} )); xlabel('time [s]'); ylabel('error [cm]')
              hold on
              linewidth = 2;
                try
                  h1 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.ckf.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.ckf.p.rms+ ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv2.ckf.p.rms, ...
                        'Color',default_colors{3}, 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
                  h2 = plot( ...
                        batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.time, ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.uav.slam.p.rms + ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv1.slam.p.rms+ ...
                    100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).clip.ugv2.slam.p.rms, ...
                        'Color',default_colors{1}, 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
                        legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;

                hold off
                grid on
                legend(legend_handles, legend_names); legend('location', 'NorthWest')
                axis(batch_data.EvT_axes)
              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

              current_fig.UserData = sprintf('%s/all_evt', [meta.dataroot meta.date 'figs']);
              current_fig.FileName = sprintf('%s_%s', meta.exp_code, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx});
              current_fig.PaperPosition = [0 0 15 5];
            % try printfig(meta, current_fig); close(current_fig); catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          end
          otherwise % switch meta.exp_codes{case_idx}
          disp('plotting is confused... quitting')
          return
        end % switch meta.exp_codes{case_idx}


    end

  end
  %% set(0,'DefaultFigureVisible','on');
  set(0,'DefaultFigureVisible','on');
  figHandles = findall(0, 'Type', 'figure');
  set(figHandles(:), 'visible', 'on');

end




          
        % figure(meta.fig_base_n+60+map_code); current_fig = gcf; clf; fprintf('figure(%d)\n', current_fig.Number);
				% 	linewidth = 2;

					% % title(sprintf( 'Mean all vehicles yaw error vs time, %s, map_%i', exp_tex, map_code));
          % ax1 = gca; % current axes
					% 	ax1.XColor = 'k';ax1.YColor = 'k'; % axis color
	        %   ax1_pos = ax1.Position; % position of first axes
					% 	% ax1.XLim = [batch_data.EvT_axes(1) batch_data.EvT_axes(2)];
					% 	% ax1.YLim = [batch_data.EvT_axes(3) batch_data.EvT_axes(4)];
					% 	ax1.XLabel.String = 'time [s]'; ax1.YLabel.String = 'error [cm]';
					%
					% 	try C0 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
					% 								batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.ckf.yaw.abs, ...
					% 								'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', 'mean ckf');
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
					% 								catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
					% 	try S0 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
					% 								batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.uav.slam.yaw.abs, ...
					% 								'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', 'mean slam');
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
					% 								catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
					% 	try C1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
					% 								batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.yaw.abs, ...
					% 								'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', 'mean ckf');
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
					% 								catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
					% 	try S1 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
					% 								batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.yaw.abs, ...
					% 								'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', 'mean slam');
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
					% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
					% 								catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          % switch meta.exp_codes{case_idx}
          %   case {'G', 'H', 'I', 'J'}
          %     try C2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
          %             			batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.p.rms_2D, ...
				  %                   'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{2}, 'displayname', 'mean ckf');
				  %                   meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = L1;
				  %                   meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = L1.DisplayName;
					% 		              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
          %     try S2 = line(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
					% 									batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.p.rms_2D, ...
					% 									'Parent',ax1, 'LineWidth', linewidth, 'Color',default_colors{3}, 'displayname', 'mean slam');
					% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = L2;
					% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = L2.DisplayName;
					% 		              catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
					% 		try  legend(ax1, [C0 C1 C2 S0 S1 S2], {C0.DisplayName, S0.DisplayName, C1.DisplayName, S1.DisplayName, C2.DisplayName, S2.DisplayName},'Location','northwest','NumColumns',2);
					% 			legend(ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
					% 		otherwise
					% 		try  legend(ax1, [C0 C1 S0 S1], {C0.DisplayName, S0.DisplayName, C1.DisplayName, S1.DisplayName},'Location','northwest','NumColumns',2);
					% 			legend(ax1, 'boxoff'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
          % end % switch meta.exp_codes{case_idx}
          % hold off
          % grid on
          % % axis(batch_data.EvT_axes)