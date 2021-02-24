function data = loadJointSlamData(meta)
%% Load experiment mfiles
% clc
  current_dir = pwd;
  cd(meta.datapath)
  disp(['loading data, ' meta.datapath])

  data_files = what;
  for i = 1:length(data_files.m)
		tic
    qdisp(data_files.m{i})
    file_str = data_files.m{i}(1:end-6);
    try
      str_call = ['[data.' file_str '] = loadrundotm(file_str, meta);'];
%       disp(['   ' str_call]);
      eval(str_call);
    catch
      % disp(['    ** Issue loading ' file_str]);
    end
    try close(wb); end
		if ~usejava('jvm')
        % disp(str_call);
				disp([ '  processing time, '  num2str(toc) ' seconds.'])
    end
  end


  % vprint('mfiles Loaded')
%   data = syncTimers(data, meta);
cd(current_dir)
end
