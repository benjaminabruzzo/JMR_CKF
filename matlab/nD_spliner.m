function vector_newtime = nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector)
    %% spline data and compare to ref
    % timeofvector : time vector corresponding to vector that will be splined
    % vectorToBeSplined : data vector to be splined
    % timeOfRefVector : time vector corresponding to times which will be interpolated
    % referenceVector : data vector corresponding to reference time to compute diff

    if(timeOfRefVector(end) > timeofvector(end))
      % if the vicon data extends past the experiment time, add an extra data point to prevent edge relics
      timeofvector(end+1) = timeOfRefVector(end);
      vectorToBeSplined(end+1,:) = vectorToBeSplined(end,:);
    end

    if(timeOfRefVector(1) < timeofvector(1))
      % if the vicon data extends past the experiment time, add an extra data point to prevent edge relics
      timeofvector = [timeOfRefVector(1);timeofvector];
      vectorToBeSplined = [vectorToBeSplined(1,:);vectorToBeSplined];
    end

    
    for index = 1:size(vectorToBeSplined,2)
      [~, vector_newtime(:,index), ~]  = spliner(timeofvector, vectorToBeSplined(:,index), timeOfRefVector, referenceVector);
%       vector_newtime(:,index) = interp1(timeofvector,vectorToBeSplined(:,index),referenceVector(:,index));
    end


end

