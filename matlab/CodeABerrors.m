function data = CodeABerrors(meta, data)
%% everything will be splined to the vicon clock
  timeOfRefVector = data.vicon.time;
  errors.time = data.vicon.time;
  ugvn = {'ugv1', 'ugv2'};  % ugvn{1,2} ugvn{1,1} ugvn{1,ugv_idx}
  % (matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx})))

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
    errors.uav.angular_delta_abs(i,1) = sqrt(errors.uav.delta(i,:)*errors.uav.delta(i,:)');
    if (errors.uav.angular_delta_abs(i,1)>0.0001) errors.uav.angular_dist(i,1) = errors.uav.angular_dist(i-1,1) + errors.uav.angular_delta_abs(i,1);
    else errors.uav.angular_dist(i,1) = errors.uav.angular_dist(i-1,1); end
  end

  try % uav slam
    referenceVector   = data.vicon.uav.position.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.p.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    slam_uav_p_gl     = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.slam.p = slam_uav_p_gl - data.vicon.uav.position.gl;
    errors.uav.slam.p_rms = sqrt(sum(errors.uav.slam.p .*   errors.uav.slam.p,2));

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.yaw.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    slam_uav_yaw_gl   = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.slam.yaw = slam_uav_yaw_gl - data.vicon.uav.yaw.gl;
    errors.uav.slam.yaw_abs = abs(errors.uav.slam.yaw);
    
  catch Mexc_; qdisp(['    uav slam ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % uav ckf
    referenceVector   = data.vicon.uav.position.gl;
    vectorToBeSplined = data.ugv1_ckf.oneCKF.uav.EstPosition_gl;
    timeofvector      = data.ugv1_ckf.oneCKF.time;
    ckf_uav_p_gl      = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.ckf.p  = ckf_uav_p_gl - data.vicon.uav.position.gl;
    errors.uav.ckf.p_rms = sqrt(sum(errors.uav.ckf.p .*   errors.uav.ckf.p,2));

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.ugv1_ckf.oneCKF.uav.Yaw;
    timeofvector      = data.ugv1_ckf.oneCKF.time;
    ckf_uav_yaw_gl    = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.uav.ckf.yaw = ckf_uav_yaw_gl - data.vicon.uav.yaw.gl;
    errors.uav.ckf.yaw_abs = abs(errors.uav.ckf.yaw);
  catch Mexc_; qdisp(['    uav ckf ' Mexc_.identifier ' :: ' Mexc_.message]); end

%% ugv1
  ugv_idx = 1;

  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist = 0;            errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist = 0;
  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta = [0,0,0];     errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta = 0;
  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs = 0;       errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs = 0;

  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).position.gl;
  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
  
  % what is this really doing? 
  for i = 1:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl)
    a = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:); a(a<0.0001) = 0;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:) = a; clear a
  end

  for i = 2:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl)
    % position delta
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(i-1,:);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1) = sqrt(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)*errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)');
    if (errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1)>0.001) errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i-1,1) + errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta_abs(i,1);
    else errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i-1,1); end
    %angular delta
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta(i,:) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(i,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.yaw_gl(i-1,:);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1) = sqrt(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)*errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).delta(i,:)');
    if (errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1)>0.0001) errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i-1,1) + errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_delta_abs(i,1);
    else errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).angular_dist(i-1,1); end
  end

  net_displacement = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(end,:) - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl(1,:);
  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).net_displacement = sqrt(net_displacement * net_displacement');
  qdisp(sprintf('%s.net_displacement: %4.6f', ugvn{1,ugv_idx}, errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).net_displacement))

  try %ugv1 mux est (slam + odom)
    referenceVector   = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
    vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Position_gl;
    timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
    errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p     =     errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p .*  errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p,2));

    referenceVector   = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
    vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.Yaw;
    timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_mux', ugvn{1,ugv_idx}))).est.time;
    errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw     =     errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs = abs(errors.slam.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
  catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' slam (mux) ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % ugv1 ckf est
    referenceVector   = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
    vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Position_gl;
    timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
    errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p     =      errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).p.gl - errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).vicon.gl;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p .*   errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p,2));

    referenceVector   = data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
    vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.Yaw;
    timeofvector      = data.(matlab.lang.makeValidName(sprintf('%s_ckf', ugvn{1,ugv_idx}))).est.time;
    errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw     =     errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl;
    errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs = abs(errors.ckf.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).yaw.gl);
  catch Mexc_; qdisp(['    ' ugvn{1,ugv_idx} ' ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end

  for i = 1:length(errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist)
    if (errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).dist(i) == 0)
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_v_dist(i,1)  = 0;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_v_dist(i,1) = 0;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_v_dist(i,1) = 0;
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_v_dist(i,1) = 0;
    else
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_v_dist(i,1)  = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.p_rms(i);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_v_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.p_rms(i);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_v_dist(i,1)  = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).ckf.yaw_abs(i);
      errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_v_dist(i,1) = errors.(matlab.lang.makeValidName(ugvn{1,ugv_idx})).slam.yaw_abs(i);
    end
  end
  
%% landmarks
  for idx = 0:11
    try
      referenceVector   = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl;
      timeofvector      = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time;
      aprilxx_slam_p_gl = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).slam.p.gl = aprilxx_slam_p_gl - referenceVector;
                          
    catch Mexc_; qdisp([sprintf( '   april%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try
      referenceVector   = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined = data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl;
      timeofvector      = data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time;
      aprilxx_ckf_p_gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p.gl = aprilxx_ckf_p_gl - referenceVector;

    catch Mexc_; qdisp([sprintf( '    tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
  end

%% save
  root = [meta.dataroot meta.date meta.exp_code '/' meta.run '/'];
  mat_file = ['errors_' meta.run '.mat'];
  save([root mat_file], 'errors')
  data.errors = errors;

  end
