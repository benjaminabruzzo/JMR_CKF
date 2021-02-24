function data = calculateErrors(meta, data)
%% everything will be splined to the vicon clock
  timeOfRefVector = data.vicon.time;
  errors.time = data.vicon.time;

%% uav
  try % uav slam
    referenceVector   = data.vicon.uav.position.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.p.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    errors.slam.uav.p.gl = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.fullSLAM.uav.est.yaw.global;
    timeofvector      = data.fullSLAM.uav.est.time;
    errors.slam.uav.yaw.gl = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
  catch Mexc_; qdisp(['    uav slam ' Mexc_.identifier ' :: ' Mexc_.message]); end

  try % uav ckf
    referenceVector   = data.vicon.ugv1.position.gl;
    vectorToBeSplined = data.ugv1_ckf.oneCKF.uav.EstPosition_gl;
    timeofvector      = data.ugv1_ckf.oneCKF.time;
    errors.ckf.uav.p.gl = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);

    referenceVector   = data.vicon.uav.yaw.gl;
    vectorToBeSplined = data.ugv1_ckf.oneCKF.uav.Yaw;
    timeofvector      = data.ugv1_ckf.oneCKF.time;
    errors.ckf.uav.yaw.gl = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
  catch Mexc_; qdisp(['    uav ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end

% errors.ugvN = {'ugv1'; 'ugv2'};
  errors.ugvN = {'ugv1'; 'ugv2'}; 

  for idx = 1:length(errors.ugvN)
    errors.(matlab.lang.makeValidName(errors.ugvN{idx})).dist = 0;
    errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta = [0,0,0];
    errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta_abs = 0;

    for i = 2:length(data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl)
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta(i,:) = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(i,:) - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(i-1,:);
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta_abs(i,1) = sqrt(errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta(i,:)*errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta(i,:)');
      if (errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta_abs(i,1)>0.001)
        errors.(matlab.lang.makeValidName(errors.ugvN{idx})).dist(i,1) = errors.(matlab.lang.makeValidName(errors.ugvN{idx})).dist(i-1,1) + errors.(matlab.lang.makeValidName(errors.ugvN{idx})).delta_abs(i,1);
      else
        errors.(matlab.lang.makeValidName(errors.ugvN{idx})).dist(i,1) = errors.(matlab.lang.makeValidName(errors.ugvN{idx})).dist(i-1,1);
      end
    end
    
    if(strcmp(errors.ugvN{idx},'ugv2'))
      index = sum(data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(:,1) == 0);
      for i = 1:index
        data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(i,:) = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(index+1,:);
      end
    end
    
    net_displacement = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(end,:) - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl(1,:);
    errors.(matlab.lang.makeValidName(errors.ugvN{idx})).net_displacement = sqrt(net_displacement * net_displacement');
    qdisp(sprintf('%s.net_displacement: %4.6f', errors.ugvN{idx}, errors.(matlab.lang.makeValidName(errors.ugvN{idx})).net_displacement))
    
    
    try % mux est (slam + odom)
      referenceVector   = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf( '%s_mux', errors.ugvN{idx} ))).est.Position_gl;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf( '%s_mux', errors.ugvN{idx} ))).est.time;    
      errors.slam.(matlab.lang.makeValidName(errors.ugvN{idx})).p.gl  = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.p     =     errors.slam.(matlab.lang.makeValidName(errors.ugvN{idx})).p.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl;
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.p .*  errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.p,2));

      referenceVector   = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf( '%s_mux', errors.ugvN{idx} ))).est.Yaw;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf( '%s_mux', errors.ugvN{idx} ))).est.time;
      errors.slam.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl  = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.yaw     =     errors.slam.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl;
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).slam.yaw_abs = abs(errors.slam.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl);
    catch Mexc_; qdisp(['    ' errors.ugvN{idx} ' slam (mux) ' Mexc_.identifier ' :: ' Mexc_.message]); end  

    try % ckf est
      referenceVector   = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl;
      
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf( '%s_ckf', errors.ugvN{idx} ))).est.Position_gl;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf( '%s_ckf', errors.ugvN{idx} ))).est.time;
      errors.ckf.(matlab.lang.makeValidName(errors.ugvN{idx})).p.gl  = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.p     =      errors.ckf.(matlab.lang.makeValidName(errors.ugvN{idx})).p.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).position.gl;
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.p_rms = sqrt(sum(errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.p .*   errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.p,2));

      referenceVector   = data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl;
      vectorToBeSplined = data.(matlab.lang.makeValidName(sprintf( '%s_ckf', errors.ugvN{idx} ))).est.Yaw;
      timeofvector      = data.(matlab.lang.makeValidName(sprintf( '%s_ckf', errors.ugvN{idx} ))).est.time;  
      errors.ckf.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl  = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.yaw     =     errors.ckf.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl;
      errors.(matlab.lang.makeValidName(errors.ugvN{idx})).ckf.yaw_abs = abs(errors.ckf.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl - data.vicon.(matlab.lang.makeValidName(errors.ugvN{idx})).yaw.gl);
    catch Mexc_; qdisp(['    ' errors.ugvN{idx} ' ckf est ' Mexc_.identifier ' :: ' Mexc_.message]); end  
  end

  
%% landmarks
  for idx = 0:11
    try
      referenceVector   = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.EstPosition_gl;
      timeofvector      = data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).slam.time;
      errors.slam.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p.gl = ...
                          nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    catch Mexc_; qdisp([sprintf( '   april%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end

    try
      referenceVector   = data.vicon.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).position.gl;
      vectorToBeSplined = data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).EstPosition_gl;
      timeofvector      = data.ugv1_ckf.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', idx ))).time;
      errors.ckf.(matlab.lang.makeValidName(sprintf( 'april%02d', idx ))).p.gl = ...
                          nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector);
    catch Mexc_; qdisp([sprintf( '    tag_%02d', idx ) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
  end

%% save
  root = [meta.dataroot meta.date meta.run '/'];
  mat_file = ['errors_' meta.run '.mat'];
  save([root mat_file], 'errors')
  data.errors = errors;
    
  figure(2); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
    title('ugv1/2 distance over time')
    hold on
      try plot(data.errors.time, data.errors.ugv1.dist, 'displayname', 'ugv1');catch; end
      try plot(data.errors.time, data.errors.ugv2.dist, 'displayname', 'ugv2');catch; end
    hold off
    legend('toggle')
    grid on
    xlabel('time [s]')
    ylabel('error [m]')  

  
  end
