function vector_newtime = nD_interp1(timeofvector,   vectorToBeSplined,        timeOfRefVector,       referenceVector)
%% spline data and compare to ref
% sampleTime : time vector corresponding to times which will be interpolated
% vectorToBeSplined : data vector to be interpolated
% timeOfRefVector  : time vector of the new/interplated data points
% referenceVector: data vector corresponding to reference time to compute diff
%                    nD_spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector)

%     f = @(x) isequal(length(x), length(unique(x)));
    
    switch nargin
        % for robustness, make sure all data is row-vectors...?
        case 4
          if size(timeofvector,1)  < size(timeofvector,2);  timeofvector  = timeofvector';  end
          if size(vectorToBeSplined,1)  < size(vectorToBeSplined,2);  vectorToBeSplined  = vectorToBeSplined';  end
          if size(timeOfRefVector,1)   < size(timeOfRefVector,2);   timeOfRefVector   = timeOfRefVector';   end
          if size(referenceVector,1) < size(referenceVector,2); referenceVector = referenceVector'; end
        case 3
          if size(timeofvector,1)  < size(timeofvector,2);  timeofvector  = timeofvector';  end
          if size(vectorToBeSplined,1)  < size(vectorToBeSplined,2);  vectorToBeSplined  = vectorToBeSplined';  end
          if size(timeOfRefVector,1)   < size(timeOfRefVector,2);   timeOfRefVector   = timeOfRefVector';   end
        otherwise
          qdisp(sprintf('nD_interp1 needs either 3 or 4 inputs, receieved: %d', nargin));
          return
    end
    
    %%
    if(timeOfRefVector(end) > timeofvector(end))
      % if the query data extends past the sample time, add an extra data point to the sampled data to prevent edge relics
      timeofvector = [timeofvector; timeOfRefVector(end)];
      vectorToBeSplined = [vectorToBeSplined; vectorToBeSplined(end,:)];
    end

    if(timeOfRefVector(1) < timeofvector(1))
      % if the query data starts earlier than the sample time, add an extra data point to the sampled data to prevent edge relics
      timeofvector = [timeOfRefVector(1)   ; timeofvector];
      vectorToBeSplined = [vectorToBeSplined(1,:); vectorToBeSplined];
    end
    
    %% remove duplicates from sample data:
    a = timeofvector; % size(a)
    b = vectorToBeSplined; % size(b)
    [va,ia] = unique(a,'stable');
     %size(va)
     %size(ia)
    Same = ones(size(a));
    Same(ia) = 0;
    timeofvector(Same==1) = []; % size(timeofvector)
    vectorToBeSplined(Same==1,:) = []; % size(vectorToBeSplined)
    
    for index = 1:size(vectorToBeSplined,2)
%       vq = interp1(x,v,xq)
%     disp(f(timeofvector))
%     disp(f(vectorToBeSplined(:,index)))
%     disp(f(timeOfRefVector))
%       
      vector_newtime(:,index) = interp1(timeofvector,vectorToBeSplined(:,index),timeOfRefVector, 'linear');
    end


end

