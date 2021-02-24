function [error, d_traveled] = error_over_d_traveled(time_actual, position_actual, time_est, position_est)
% function [error, d_traveled] =  error_over_d_traveled(data.vicon.time, data.vicon.ugv1.position.gl, ...
% data.ugv1_mux.est.time, data.ugv1_mux.est.Position_gl)
%% calculate error of a vector against the distance traveled 
% position_actual :: actual position over time    :: [x,y,z] x n rows
% time_actual     :: timestamp of actual position :: [t] x n rows  
% position_est    :: estimated position vs time   :: [x,y,z] x m rows
% time_est        :: timestamp of est position    :: [t] x m rows  

%% zeroth (remove later) init data
%   time_actual = data.vicon.time;
%   position_actual = data.vicon.ugv1.position.gl;
%   time_est = data.ugv1_mux.est.time;
%   position_est = data.ugv1_mux.est.Position_gl;

%% first remove all data prior to actual time init  
  position_est(time_est<time_actual(1),:)= [];
  time_est(time_est<time_actual(1)) = [];

% then remove all data after to actual time ends
  position_est(time_est>time_actual(end),:)= [];
  time_est(time_est>time_actual(end)) = [];

%% then spline actual data to match esitmated time stamps
% function [spline, vector_newtime, diff]  = spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector)

  [~, p_at_est_time(:,1), diff(:,1)]  = spliner(time_actual, position_actual(:,1), time_est, position_est(:,1));
  [~, p_at_est_time(:,2), diff(:,2)]  = spliner(time_actual, position_actual(:,2), time_est, position_est(:,2));
  [~, p_at_est_time(:,3), diff(:,3)]  = spliner(time_actual, position_actual(:,3), time_est, position_est(:,3));

  
  d_traveled = 0;
  
  for index =  2:size(p_at_est_time,1)
    d_pos = p_at_est_time(index,1:2) - p_at_est_time(index-1,1:2);
    d_traveled(index,1) = d_traveled(index-1) + sqrt(d_pos*d_pos');
  end
  
  for index =  1:size(diff,1)
    err = diff(index,:);
    error(index,1) = sqrt(err*err');
  end
  
  
end