function data = CodeCDEFerrors(meta, data)
%% everything will be splined to the vicon clock
  timeOfRefVector = data.vicon.time;
  errors.time = data.vicon.time;

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
  errors.ugv1.dist = 0;            errors.ugv1.angular_dist = 0;
  errors.ugv1.delta = [0,0,0];     errors.ugv1.angular_delta = 0;
  errors.ugv1.delta_abs = 0;       errors.ugv1.angular_delta_abs = 0;

  errors.ugv1.vicon.gl = data.vicon.ugv1.position.gl;
  errors.ugv1.vicon.yaw_gl = data.vicon.ugv1.yaw.gl;
  
  % zero out small motions < 0.1 mm
  for i = 1:length(errors.ugv1.vicon.gl)
    a = errors.ugv1.vicon.gl(i,:); a(a<0.0001) = 0;
    errors.ugv1.vicon.gl(i,:) = a; clear a
  end

  for i = 2:length(errors.ugv1.vicon.gl)
    % position delta
    errors.ugv1.delta(i,:) = errors.ugv1.vicon.gl(i,:) - errors.ugv1.vicon.gl(i-1,:);
    errors.ugv1.delta_abs(i,1) = sqrt(errors.ugv1.delta(i,:)*errors.ugv1.delta(i,:)');
    if (errors.ugv1.delta_abs(i,1)>0.001) errors.ugv1.dist(i,1) = errors.ugv1.dist(i-1,1) + errors.ugv1.delta_abs(i,1);
    else errors.ugv1.dist(i,1) = errors.ugv1.dist(i-1,1); end
    %angular delta
    errors.ugv1.angular_delta(i,:) = errors.ugv1.vicon.yaw_gl(i,:) - errors.ugv1.vicon.yaw_gl(i-1,:);
    errors.ugv1.angular_delta_abs(i,1) = sqrt(errors.ugv1.delta(i,:)*errors.ugv1.delta(i,:)');
    if (errors.ugv1.angular_delta_abs(i,1)>0.0001) errors.ugv1.angular_dist(i,1) = errors.ugv1.angular_dist(i-1,1) + errors.ugv1.angular_delta_abs(i,1);
    else errors.ugv1.angular_dist(i,1) = errors.ugv1.angular_dist(i-1,1); end
  end

  net_displacement = errors.ugv1.vicon.gl(end,:) - errors.ugv1.vicon.gl(1,:);
  errors.ugv1.net_displacement = sqrt(net_displacement * net_displacement');
  qdisp(sprintf('ugv1.net_displacement: %4.6f', errors.ugv1.net_displacement))

  try %ugv1 mux est (slam + odom)
    referenceVector   = errors.ugv1.vicon.gl;
    vectorToBeSplined = data.ugv1_mux.est.Position_gl;
    timeofvector      = data.ugv1_mux.est.time;
    errors.slam.ugv1.p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv1.slam.p     =     errors.slam.ugv1.p.gl - errors.ugv1.vicon.gl;
    errors.ugv1.slam.p_rms = sqrt(sum(errors.ugv1.slam.p .*  errors.ugv1.slam.p,2));

    referenceVector   = data.vicon.ugv1.yaw.gl;
    vectorToBeSplined = data.ugv1_mux.est.Yaw;
    timeofvector      = data.ugv1_mux.est.time;
    errors.slam.ugv1.yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv1.slam.yaw     =     errors.slam.ugv1.yaw.gl - data.vicon.ugv1.yaw.gl;
    errors.ugv1.slam.yaw_abs = abs(errors.slam.ugv1.yaw.gl - data.vicon.ugv1.yaw.gl);
  catch Mexc_; qdisp(['    ugv1 slam (mux) ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % ugv1 ckf est
    referenceVector   = errors.ugv1.vicon.gl;
    vectorToBeSplined = data.ugv1_ckf.est.Position_gl;
    timeofvector      = data.ugv1_ckf.est.time;
    errors.ckf.ugv1.p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv1.ckf.p     =      errors.ckf.ugv1.p.gl - errors.ugv1.vicon.gl;
    errors.ugv1.ckf.p_rms = sqrt(sum(errors.ugv1.ckf.p .*   errors.ugv1.ckf.p,2));

    referenceVector   = data.vicon.ugv1.yaw.gl;
    vectorToBeSplined = data.ugv1_ckf.est.Yaw;
    timeofvector      = data.ugv1_ckf.est.time;
    errors.ckf.ugv1.yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv1.ckf.yaw     =     errors.ckf.ugv1.yaw.gl - data.vicon.ugv1.yaw.gl;
    errors.ugv1.ckf.yaw_abs = abs(errors.ckf.ugv1.yaw.gl - data.vicon.ugv1.yaw.gl);
  catch Mexc_; qdisp(['    ugv1 ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end

  for i = 1:length(errors.ugv1.dist)
    if (errors.ugv1.dist(i) == 0)
      errors.ugv1.ckf.p_v_dist(i,1)  = 0;
      errors.ugv1.slam.p_v_dist(i,1) = 0;
      errors.ugv1.ckf.yaw_v_dist(i,1) = 0;
      errors.ugv1.slam.yaw_v_dist(i,1) = 0;
    else
      errors.ugv1.ckf.p_v_dist(i,1)  = errors.ugv1.ckf.p_rms(i);
      errors.ugv1.slam.p_v_dist(i,1) = errors.ugv1.slam.p_rms(i);
      errors.ugv1.ckf.yaw_v_dist(i,1)  = errors.ugv1.ckf.yaw_abs(i);
      errors.ugv1.slam.yaw_v_dist(i,1) = errors.ugv1.slam.yaw_abs(i);
    end
  end
  
%% ugv2
  errors.ugv2.dist = 0;            errors.ugv2.angular_dist = 0;
  errors.ugv2.delta = [0,0,0];     errors.ugv2.angular_delta = 0;
  errors.ugv2.delta_abs = 0;       errors.ugv2.angular_delta_abs = 0;

  errors.ugv2.vicon.gl = data.vicon.ugv2.position.gl;
  errors.ugv2.vicon.yaw_gl = data.vicon.ugv2.yaw.gl;

  % fix init error of vicon logger
  index = sum(data.vicon.ugv2.position.gl(:,1) == 0);
  for i = 1:index
    data.vicon.ugv2.position.gl(i,:) = data.vicon.ugv2.position.gl(index+1,:);
  end
  
  for i = 2:length(data.vicon.ugv2.position.gl)
    errors.ugv2.delta(i,:) = data.vicon.ugv2.position.gl(i,:) - data.vicon.ugv2.position.gl(i-1,:);
    errors.ugv2.delta_abs(i,1) = sqrt(errors.ugv2.delta(i,:)*errors.ugv2.delta(i,:)');
    if (errors.ugv2.delta_abs(i,1)>0.001); errors.ugv2.dist(i,1) = errors.ugv2.dist(i-1,1) + errors.ugv2.delta_abs(i,1);
    else errors.ugv2.dist(i,1) = errors.ugv2.dist(i-1,1); end
  end

  net_displacement = data.vicon.ugv2.position.gl(end,:) - data.vicon.ugv2.position.gl(1,:);
  errors.ugv2.net_displacement = sqrt(net_displacement * net_displacement');
  qdisp(sprintf('ugv2.net_displacement: %4.6f', errors.ugv2.net_displacement))

  try %ugv2 mux est (slam + odom)
    referenceVector   = data.vicon.ugv2.position.gl;
    vectorToBeSplined = data.ugv2_mux.est.Position_gl;
    timeofvector      = data.ugv2_mux.est.time;
    errors.slam.ugv2.p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv2.slam.p     =     errors.slam.ugv2.p.gl - data.vicon.ugv2.position.gl;
    errors.ugv2.slam.p_rms = sqrt(sum(errors.ugv2.slam.p .*  errors.ugv2.slam.p,2));

    referenceVector   = data.vicon.ugv2.yaw.gl;
    vectorToBeSplined = data.ugv2_mux.est.Yaw;
    timeofvector      = data.ugv2_mux.est.time;
    errors.slam.ugv2.yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv2.slam.yaw     =     errors.slam.ugv2.yaw.gl - data.vicon.ugv2.yaw.gl;
    errors.ugv2.slam.yaw_abs = abs(errors.slam.ugv2.yaw.gl - data.vicon.ugv2.yaw.gl);
  catch Mexc_; qdisp(['    ugv2 mux ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % ugv2 ckf est
    referenceVector   = data.vicon.ugv2.position.gl;
    vectorToBeSplined = data.ugv2_ckf.est.Position_gl;
    timeofvector      = data.ugv2_ckf.est.time;
    errors.ckf.ugv2.p.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv2.ckf.p     =      errors.ckf.ugv2.p.gl - data.vicon.ugv2.position.gl;
    errors.ugv2.ckf.p_rms = sqrt(sum(errors.ugv2.ckf.p .*   errors.ugv2.ckf.p,2));

    referenceVector   = data.vicon.ugv2.yaw.gl;
    vectorToBeSplined = data.ugv2_ckf.est.Yaw;
    timeofvector      = data.ugv2_ckf.est.time;
    errors.ckf.ugv2.yaw.gl  = nD_interp1(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    errors.ugv2.ckf.yaw     =     errors.ckf.ugv2.yaw.gl - data.vicon.ugv2.yaw.gl;
    errors.ugv2.ckf.yaw_abs = abs(errors.ckf.ugv2.yaw.gl - data.vicon.ugv2.yaw.gl);
  catch Mexc_; qdisp(['    ugv2 ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end
  
  for i = 1:length(errors.ugv2.dist)
    if (errors.ugv2.dist(i) == 0)
      errors.ugv2.ckf.p_v_dist(i,1)  = 0;
      errors.ugv2.slam.p_v_dist(i,1) = 0;
      errors.ugv2.ckf.yaw_v_dist(i,1) = 0;
      errors.ugv2.slam.yaw_v_dist(i,1) = 0;
    else
      errors.ugv2.ckf.p_v_dist(i,1)  = errors.ugv2.ckf.p_rms(i);
      errors.ugv2.slam.p_v_dist(i,1) = errors.ugv2.slam.p_rms(i);
      errors.ugv2.ckf.yaw_v_dist(i,1)  = errors.ugv2.ckf.yaw_abs(i);
      errors.ugv2.slam.yaw_v_dist(i,1) = errors.ugv2.slam.yaw_abs(i);
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
