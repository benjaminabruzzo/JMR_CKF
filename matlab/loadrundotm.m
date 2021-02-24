function [out] = loadrundotm(in, meta)
  here = pwd;
  root = [meta.datapath];
  mat_file = [in '_' meta.run '.mat'];
  m_file = [in '_' meta.run '.m'];
  cd(root);
%   disp(root)
  if exist(mat_file)
      % pdisp(['    Loading ' mat_file ' ...'])
      load(mat_file);
  elseif exist(m_file, 'file')

      % pdisp(['    Loading ' root m_file ' ...'])
      try
          run(['./prealloc/pre_' in '_' meta.run])
      catch
          % pdisp(['     ' in ': no preallocation found'])
      end
      eval([in '_' meta.run])
      if exist(in, 'var')
          % pdisp(['     Saving ' root mat_file ' ...'])
          save([root mat_file], in)
      end
  end
  out = eval(in);
  cd(here);
end
