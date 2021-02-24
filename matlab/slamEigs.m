%% Calc eigen values

try data.fullSLAM.eig(:,1) = eig(data.fullSLAM.augmentedCovariance{1,1});
  for idx = 2:length(data.fullSLAM.augmentedCovariance)
    full_eig = eig(data.fullSLAM.augmentedCovariance{idx,1});

    if (length(full_eig)~=size(data.fullSLAM.eig,1))
      % then a block or column of zeros is needed figure out where
      add_zeros = abs(size(data.fullSLAM.eig,1)-(length(full_eig)));
      if (length(full_eig)<size(data.fullSLAM.eig,1))
        % then add to column of new eigen values
          full_eig = [full_eig; zeros(add_zeros,1)];
      else
        % then there are more eigen values now than all previous times  add zeros to the bottom of previous matrix
					data.fullSLAM.eig = [data.fullSLAM.eig; zeros(add_zeros, idx-1)];
      end
    end
    % add eigen values to end of matrix
      data.fullSLAM.eig(:, idx) = full_eig;
	end
end; clear idx april_eig add_zeros
