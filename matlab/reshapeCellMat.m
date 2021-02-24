function R = reshapeCellMat(M)

  for idx = 1:length(M)
    Midx = M{idx,1};
    R(idx,:) = reshape(Midx, [1,size(Midx,1)*size(Midx,2)]);
  end

end

%%
% clc
% 
% m = data.jointSLAM.trace.uav.covmat{100,1}
% l = reshape(m, [1,size(m,1)*size(m,2)])