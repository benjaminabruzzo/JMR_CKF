function data = Code_errors(meta, data, ugvn)
%% clear old data
  root = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
  mat_file = ['errors_' meta.run '.mat'];
  try delete([root mat_file]); catch Mexc_; qdisp(['    Code_errors, ' mat_file ',' Mexc_.identifier ' :: ' Mexc_.message]);  end


%% everything will be splined to the vicon clock
  errors.time = data.vicon.time;
  timeOfRefVector = data.vicon.time;

  clear clip
  % time to start tracking the error after slam is switched on
  clip.time.vicon = data.ugv1_mux.slam.time(1);
  clip.vector = data.vicon.time<clip.time.vicon;
  clip.time = data.vicon.time;
  clip.time(clip.vector) = [];

  clip.timeOfRefVector = clip.time;


%% uav
  errors.uav.dist = 0;            errors.uav.angular_dist = 0;
  errors.uav.delta = [0,0,0];     errors.uav.angular_delta = 0;
  errors.uav.delta_abs = 0;       errors.uav.angular_delta_abs = 0;

  errors.uav.vicon.gl = data.vicon.uav.position.gl;
  errors.uav.vicon.yaw_gl = data.vicon.uav.yaw.gl;

  for i = 2:length(errors.uav.vicon.gl)
    % position delta
    errors.uav.delta(i,:) = errors.uav.vicon.gl(i,:) - errors.uav.vicon.gl(i-1,:);
    errors.uav.delta_abs(i,1) = sqrt(errors.uav.delta(i,:)*errors.uav.delta(i,:)');
    if (errors.uav.delta_abs(i,1)>0.001) errors.uav.dist(i,1) = errors.uav.dist(i-1,1) + errors.uav.delta_abs(i,1);
    else errors.uav.dist(i,1) = errors.uav.dist(i-1,1); end
    %angular delta
    errors.uav.angular_delta(i,:) = errors.uav.vicon.yaw_gl(i,:) - errors.uav.vicon.yaw_gl(i-1,:);
    errors.uav.angular_delta_abs(i,1) = sqrt(errors.uav.angular_delta(i,:)*errors.uav.angular_delta(i,:)');
    if (errors.uav.angular_delta_abs(i,1)>0.0001) errors.uav.angular_dist(i,1) = errors.uav.angular_dist(i-1,1) + errors.uav.angular_delta_abs(i,1);
    else errors.uav.angular_dist(i,1) = errors.uav.angular_dist(i-1,1); end
  end

  try % uav slam
    referenceVector   = data.vicon.uav.position.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.p.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    slam_uav_p_gl     = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.slam.p = slam_uav_p_gl - data.vicon.uav.position.gl;
    errors.uav.slam.p_rms  = sqrt(sum(errors.uav.slam.p .*   errors.uav.slam.p,2));
    errors.uav.slam.p_mean = mean(errors.uav.slam.p_rms);
    errors.uav.slam.p_std  = std(errors.uav.slam.p_rms);

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.yaw.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    slam_uav_yaw_gl   = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.slam.yaw = slam_uav_yaw_gl - data.vicon.uav.yaw.gl;
    errors.uav.slam.yaw_abs  = abs(errors.uav.slam.yaw);
    errors.uav.slam.yaw_mean = mean(errors.uav.slam.yaw_abs);
    errors.uav.slam.yaw_std  = std(errors.uav.slam.yaw_abs);

  catch Mexc_; qdisp(['    uav slam ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % uav ckf
    referenceVector   = data.vicon.uav.position.gl;
    vectorToBeSplined = data.uav_ckf.ckf.EstP_gl;
    timeofvector      = data.uav_ckf.ckf.time;
    ckf_uav_p_gl      = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.ckf.p  = ckf_uav_p_gl - data.vicon.uav.position.gl;
    errors.uav.ckf.p_rms  = sqrt(sum(errors.uav.ckf.p .*   errors.uav.ckf.p,2));
    errors.uav.ckf.p_mean = mean(errors.uav.ckf.p_rms);
    errors.uav.ckf.p_std  = std(errors.uav.ckf.p_rms);

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.uav_ckf.ckf.EstYaw_gl;
    timeofvector      = data.uav_ckf.ckf.time;
    ckf_uav_yaw_gl    = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.ckf.yaw = ckf_uav_yaw_gl - data.vicon.uav.yaw.gl;
    errors.uav.ckf.yaw_abs  = abs(errors.uav.ckf.yaw);
    errors.uav.ckf.yaw_mean = mean(errors.uav.ckf.yaw_abs);
    errors.uav.ckf.yaw_std  = std(errors.uav.ckf.yaw_abs);
  catch Mexc_; qdisp(['    uav ckf ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try
  for i = 1:length(errors.uav.dist)
    if (errors.uav.dist(i) == 0)
      errors.uav.ckf.p_v_dist(i,1)  = 0;
      errors.uav.slam.p_v_dist(i,1) = 0;
      errors.uav.ckf.yaw_v_dist(i,1)  = 0;
      errors.uav.slam.yaw_v_dist(i,1) = 0;
    else
      errors.uav.ckf.p_v_dist(i,1)  = errors.uav.ckf.p_rms(i);
      errors.uav.slam.p_v_dist(i,1) = errors.uav.slam.p_rms(i);
      errors.uav.ckf.yaw_v_dist(i,1)  = errors.uav.ckf.yaw_abs(i);
      errors.uav.slam.yaw_v_dist(i,1) = errors.uav.slam.yaw_abs(i);
    end
  end
  catch Mexc_; qdisp([Mexc_.identifier ' :: ' Mexc_.message]); end


  try % uav slam clipped in time
    clip.referenceVector   = clip_matrix(clip.vector, data.vicon.uav.position.gl);
    clip.vectorToBeSplined = data.fullSLAM.uav.est.p.global;
    clip.timeofvector      = data.fullSLAM.uav.est.time;
    clip.uav.p.gl          = clip.referenceVector(:,1);
    clip.uav.slam.p.gl     = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
    clip.uav.slam.p.err    = clip.uav.slam.p.gl - clip.referenceVector;
    clip.uav.slam.p.rms    = sqrt(sum(clip.uav.slam.p.err .* clip.uav.slam.p.err,2));
    clip.uav.slam.p.mean   = mean(clip.uav.slam.p.rms);
    clip.uav.slam.p.std    =  std(clip.uav.slam.p.rms);
    clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});

    clip.referenceVector   = clip_matrix(clip.vector, data.vicon.uav.yaw.gl);
    clip.vectorToBeSplined = data.fullSLAM.uav.est.yaw.global;
    clip.timeofvector      = data.fullSLAM.uav.est.time;
    clip.uav.slam.yaw.gl   = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
    clip.uav.slam.yaw.err  = clip.uav.slam.yaw.gl - clip.referenceVector;
    clip.uav.slam.yaw.abs  =  abs(clip.uav.slam.yaw.err);
    clip.uav.slam.yaw.mean = mean(clip.uav.slam.yaw.abs);
    clip.uav.slam.yaw.std  =  std(clip.uav.slam.yaw.abs);
    clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});

  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end

  try % uav ckf
    clip.referenceVector   = clip_matrix(clip.vector, data.vicon.uav.position.gl);
    clip.vectorToBeSplined = data.uav_ckf.ckf.EstP_gl;
    clip.timeofvector      = data.uav_ckf.ckf.time;
    clip.uav.ckf.p.gl      = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
    clip.uav.ckf.p.err     = clip.uav.ckf.p.gl - clip.referenceVector;
    clip.uav.ckf.p.rms     = sqrt(sum(clip.uav.ckf.p.err .*   clip.uav.ckf.p.err,2));
    clip.uav.ckf.p.mean    = mean(clip.uav.ckf.p.rms);
    clip.uav.ckf.p.std     =  std(clip.uav.ckf.p.rms);
    clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});

    clip.referenceVector   = clip_matrix(clip.vector, data.vicon.uav.yaw.gl);
    clip.vectorToBeSplined = data.uav_ckf.ckf.EstYaw_gl;
    clip.timeofvector      = data.uav_ckf.ckf.time;
    clip.uav.ckf.yaw.gl    = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
    clip.uav.ckf.yaw.err   = clip.uav.ckf.yaw.gl - clip.referenceVector;
    clip.uav.ckf.yaw.abs   =  abs(clip.uav.ckf.yaw.err);
    clip.uav.ckf.yaw.mean  = mean(clip.uav.ckf.yaw.abs);
    clip.uav.ckf.yaw.std   =  std(clip.uav.ckf.yaw.abs);
    clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});
  catch Mexc_; qdisp(['    uav ckf ' Mexc_.identifier ' :: ' Mexc_.message]); end

%% ugvs
  for ugv_idx = 1:size(ugvn,2)

    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist = 0;            errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist = 0;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta = [0,0,0];     errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta = 0;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs = 0;       errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs = 0;

    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).position.gl;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;

    if (strcmp(ugvn{1,ugv_idx},"ugv2"))
      index = sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(:,1) == 0);
        for i = 1:index
          errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:) = ...
                  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(index+1,:);
          errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(i,1) = ...
                  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(index+1,1);
        end
    else
      % zero out small motions < 0.01 mm for ugv
      for i = 1:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl)
        a = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:);
        a(abs(a)<0.00001) = 0;
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:) = a;
        clear a
      end
    end

    for i = 2:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl)
      % position delta
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)     = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i-1,:);
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1) = sqrt(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:) * errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)');
      if ( errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1)>0.001)
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i,1)      = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i-1,1) + errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1);
      else errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i,1)      = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i-1,1); end
      %angular delta
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta(i,:)     = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(i,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(i-1,:);
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1) = sqrt(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta(i,:) * errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta(i,:)');
      if ( errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1)>0.0001)
           errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i,1)      = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i-1,1) + errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1);
      else errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i,1)      = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i-1,1); end
    end

    net_displacement = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(end,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(1,:);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).net_displacement = sqrt(net_displacement * net_displacement');
%     disp(sprintf('%s.net_displacement: %4.6f', ugvn{1,ugv_idx}, errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).net_displacement))
%     fprintf('%s.net_displacement: %4.6f\n', ugvn{1,ugv_idx}, errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).net_displacement)

    try %ugv mux est (slam + odom) clipped at slam turn on time
      clip.referenceVector   = clip_matrix(clip.vector, errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl);
      clip.vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Position_gl;
      clip.timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.gl     = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.err    =          clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.gl - clip.referenceVector;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.rms    = sqrt(sum(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.err .*  clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.err,2));
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.rms_2D = sqrt(sum(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.err(:,1:2) .*  clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.err(:,1:2),2));
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.mean   =     mean(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.rms_2D);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.std    =      std(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p.rms_2D);
      clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});

      clip.referenceVector   = clip_matrix(clip.vector, data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
      clip.vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Yaw;
      clip.timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.gl   = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.err  =      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.gl - clip.referenceVector;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.abs  =  abs(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.err);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.mean = mean(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.abs);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.std  =  std(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw.abs);
      clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});
    catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' slam (mux) ' Mexc_.identifier ' :: ' Mexc_.message]); end

   try %ugv mux est (slam + odom) clipped at slam turn on time
      clip.referenceVector   = clip_matrix(clip.vector, errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl);
      clip.vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Position_gl;
      clip.timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.gl     = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.err    =          clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.gl - clip.referenceVector;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.rms    = sqrt(sum(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.err .*  clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.err,2));
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.rms_2D = sqrt(sum(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.err(:,1:2) .*  clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.err(:,1:2),2));
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.mean   =     mean(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.rms_2D);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.std    =      std(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p.rms_2D);
      clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});

      clip.referenceVector   = clip_matrix(clip.vector, data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
      clip.vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Yaw;
      clip.timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.gl   = nD_interp1(clip.timeofvector, clip.vectorToBeSplined, clip.timeOfRefVector, clip.referenceVector);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.err  =      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.gl - clip.referenceVector;
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.abs  =  abs(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.err);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.mean = mean(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.abs);
      clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.std  =  std(clip.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw.abs);
      clip = rmfields(clip, {'referenceVector', 'vectorToBeSplined', 'timeofvector'});
    catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try %ugv ckf est (slam + odom)
      referenceVector   = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Position_gl;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
      errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p     =     errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p .*  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p,2));
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms_2D = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p(:,2) .*  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p(:,2),2));
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_mean   = mean(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms_2D);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_std    = std(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms_2D);


      referenceVector   = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Yaw;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
      errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw      =     errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs  = abs(errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_mean = mean(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_std  = std(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs);
    catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' slam (mux) ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try % ugv ckf est
      referenceVector   = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Position_gl;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
      errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p     =      errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p .* errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p,2));
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms_2D = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p(:,2) .* errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p(:,2),2));
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_mean   = mean(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms_2D);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_std    =  std(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms_2D);

      referenceVector   = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Yaw;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
      errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw     =     errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs = abs(errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_mean = mean(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_std  =  std(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs);
    catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end

    for i = 1:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist)
      if (errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i) == 0)
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_v_dist(i,1)  = 0;
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_v_dist(i,1) = 0;
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_v_dist(i,1) = 0;
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_v_dist(i,1) = 0;
      else
%         errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_v_dist(i,1)  = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms(i);
%         errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_v_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms(i);
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_v_dist(i,1)  = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms_2D(i);
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_v_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms_2D(i);
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_v_dist(i,1)  = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs(i);
        errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_v_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs(i);
      end
    end


  end

%% landmarks
  errors.april.ckf.X.p_rms  = []; errors.april.ckf.X.yaw  = [];
  errors.april.slam.X.p_rms = []; errors.april.slam.X.yaw = [];

	errors.april.ckf.X.sum_rms  = 0; errors.april.ckf.X.sum_yaw  = 0; errors.april.ckf.X.count_rms  = 0; errors.april.ckf.X.count_yaw  = 0;
	errors.april.slam.X.sum_rms = 0; errors.april.slam.X.sum_yaw = 0; errors.april.slam.X.count_rms = 0; errors.april.slam.X.count_yaw = 0;

  for idx = 0:11
    try
%       timeOfRefVector   = data.vicon.time;
      referenceVector   = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined = data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl;
      timeofvector      = data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time;
      aprilxx_ckf_p_gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors_aprilxx_ckf_p_gl = aprilxx_ckf_p_gl - referenceVector;
      errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p = errors_aprilxx_ckf_p_gl;
      errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms = sqrt(sum(errors_aprilxx_ckf_p_gl .* errors_aprilxx_ckf_p_gl,2));
			errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.p_rms   = sum(errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms);
			errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.p_rms = size(errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms,1);

      referenceVector         = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl;
      vectorToBeSplined       = data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstYaw_gl;
      timeofvector            = data.uav_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time;
      aprilxx_ckf_yaw         = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors_aprilxx_ckf_yaw  = aprilxx_ckf_yaw - referenceVector;
      errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw = errors_aprilxx_ckf_yaw;
			errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs = abs(errors_aprilxx_ckf_yaw);
			errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.yaw_abs   =  sum(errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs);
			errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.yaw_abs = size(errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs,1);

			errors.april.ckf.X.p_rms    = [errors.april.ckf.X.p_rms,     errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms(end)];
			errors.april.ckf.X.yaw      = [errors.april.ckf.X.yaw,       errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw(end)];
			errors.april.ckf.X.sum_rms   = errors.april.ckf.X.sum_rms   + errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.p_rms;
			errors.april.ckf.X.count_rms = errors.april.ckf.X.count_rms + errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.p_rms;
			errors.april.ckf.X.sum_yaw   = errors.april.ckf.X.sum_yaw   + errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.yaw_abs;
			errors.april.ckf.X.count_yaw = errors.april.ckf.X.count_yaw + errors.april.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.yaw_abs;
    catch Mexc_; qdisp([sprintf( '    tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try
      referenceVector           = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined         = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl;
      timeofvector              = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time;
      aprilxx_slam_p_gl         = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors_aprilxx_slam_p_gl  = aprilxx_slam_p_gl - referenceVector;
      errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p = errors_aprilxx_slam_p_gl;
      errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms = sqrt(sum(errors_aprilxx_slam_p_gl .* errors_aprilxx_slam_p_gl,2));
			errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.p_rms   =  sum(errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms);
			errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.p_rms = size(errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms,1);

      referenceVector         = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw.gl;
      vectorToBeSplined       = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPhi_gl;
      timeofvector            = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time;
      aprilxx_slam_yaw         = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors_aprilxx_slam_yaw  = aprilxx_slam_yaw - referenceVector;
      errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw = errors_aprilxx_slam_yaw;
			errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs = abs(errors_aprilxx_slam_yaw);
			errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.yaw_abs   =  sum(errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs);
			errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.yaw_abs = size(errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw_abs,1);

			errors.april.slam.X.p_rms    = [errors.april.slam.X.p_rms,      errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p_rms(end)];
			errors.april.slam.X.yaw      = [errors.april.slam.X.yaw,        errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).yaw(end)];
			errors.april.slam.X.sum_rms   = errors.april.slam.X.sum_rms   + errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.p_rms;
			errors.april.slam.X.count_rms = errors.april.slam.X.count_rms + errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.p_rms;
			errors.april.slam.X.sum_yaw   = errors.april.slam.X.sum_yaw   + errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).sum.yaw_abs;
			errors.april.slam.X.count_yaw = errors.april.slam.X.count_yaw + errors.april.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).count.yaw_abs;
    catch Mexc_; qdisp([sprintf( '   april%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

  end

  errors.april.ckf.p_rms  = mean(errors.april.ckf.X.p_rms);
  errors.april.slam.p_rms = mean(errors.april.slam.X.p_rms);

  errors.april.ckf.X.yaw(errors.april.ckf.X.yaw>pi)   = errors.april.ckf.X.yaw(errors.april.ckf.X.yaw>pi)-2*pi;
  errors.april.slam.X.yaw(errors.april.slam.X.yaw>pi) = errors.april.slam.X.yaw(errors.april.slam.X.yaw>pi)-2*pi;
  errors.april.ckf.yaw    = mean(abs(errors.april.ckf.X.yaw));

  errors.april.slam.yaw   = mean(abs(errors.april.slam.X.yaw));

	errors.april.ckf.X.mean.rms  = errors.april.ckf.X.sum_rms  / errors.april.ckf.X.count_rms;
	errors.april.ckf.X.mean.yaw  = errors.april.ckf.X.sum_yaw  / errors.april.ckf.X.count_yaw;
	errors.april.slam.X.mean.rms = errors.april.slam.X.sum_rms / errors.april.slam.X.count_rms;
	errors.april.slam.X.mean.yaw = errors.april.slam.X.sum_yaw / errors.april.slam.X.count_yaw;

%% UGV1 distance from goal
  offset = data.vicon.ugv1.position.gl(end,:) - data.vicon.april08.position.gl(end,:);
  errors.ugv1.end.p               = data.vicon.ugv1.position.gl(end,:);
  errors.ugv1.end.goal            = data.vicon.april08.position.gl(end,:);
  errors.ugv1.end.goal_offset     = offset;
  errors.ugv1.end.goal_rms        = sqrt(offset*offset');
  
  % non-goal obstacles:
  errors.april.offset.min = 1000;
  for idx = [1:7, 9:11]
    try 
      offset = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl - data.vicon.ugv1.position.gl;
      errors.april.offset.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p = sum(sqrt(offset .* offset),2);
      errors.april.offset.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).min = min(errors.april.offset.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p);
      errors.april.offset.min = min(errors.april.offset.min, errors.april.offset.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).min);
    catch Mexc_; qdisp([sprintf( '   april%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
  end
  
  
%% save
%   root = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
%   mat_file = ['errors_' meta.run '.mat'];
  clip = rmfields(clip, {'timeOfRefVector', 'vector'});
  clip.time = clip.time - clip.time(1);
  errors.clip = clip;
  save([root mat_file], 'errors')
  data.errors = errors;


  end
