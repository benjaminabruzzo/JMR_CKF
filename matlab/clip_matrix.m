function m = clip_matrix(bool_vec, data_matrix)

  try 
    for idx = 1:size(data_matrix,2)
      temp = data_matrix(:,idx); % temproary vector
      temp(bool_vec) = []; %clear the contents based on the boolean vector
      m(:,idx) = temp; % append to output matrix
    end
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
    
    
end

