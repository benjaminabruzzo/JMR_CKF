function write_EvD_table_tex(batch_data)
%%

RAL_path = '/home/benjamin/ros/src/metahast/matlab/RALtables';
tex_filename = '671_EvD_table.tex';

disp([RAL_path '/' tex_filename]);
error_file = fopen([RAL_path '/' tex_filename], 'w');

% EXP_CODES = {'A', 'B', 'C', 'D', 'E', 'F'};
EXP_CODES = batch_data.EXP_CODES;

METHOD = {'CKF ', 'SLAM'};
method = {'ckf',  'slam'};

STRATEGIES={...
  '$\prescript{}{1}{\bm{C}}_{H}^{}$';... %A
  '$\prescript{}{1}{\bm{C}}_{L}^{}$';... %B
  '$\prescript{}{2}{\bm{C}}_{H}^{G}$';... %CG
  '$\prescript{}{2}{\bm{C}}_{H}^{R}$';... %DH
  '$\prescript{}{2}{\bm{C}}_{L}^{G}$';... %EI
  '$\prescript{}{2}{\bm{C}}_{L}^{R}$'}; %FJ

for method_idx = 1:size(METHOD,2)
  for code_idx = 1:size(EXP_CODES,2)
    uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_percent.p);
    uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_percent.yaw);
    ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_percent.p);
    ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_percent.yaw);
    try
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv2.(matlab.lang.makeValidName(method{method_idx})).end_percent.p);
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv2.(matlab.lang.makeValidName(method{method_idx})).end_percent.yaw);
    catch
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = 0;
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = 0;
    end
  end
end

caption = 'Position and yaw errors of the of the vehicle pose estimates for \textsub{UGV}{1} and the UAV averaged across all trials of each configuration as a percentage of distance traveled.';
label = 'tab:EvD';

fprintf(error_file, '\\begin{table}\n');
fprintf(error_file, '\t%% \\tiny\n');
fprintf(error_file, '\t%% \\fontsize{9pt}{9.25pt}\\selectfont\n');
fprintf(error_file, '\t%% \\renewcommand\\arraystretch{2}\n');
fprintf(error_file, '\t\\centering\n');
fprintf(error_file, '\t\\caption{%s}\n', caption);
fprintf(error_file, '\t\\label{%s}\n', label);
fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1.25}\n');
fprintf(error_file, '\t\\begin{tabular}{c c | c c | c c }\n');
fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\t& &\\multicolumn{2}{c|}{UAV} \n');
fprintf(error_file, '\t\t& \\multicolumn{2}{c}{\\textsub{UGV}{1}}  \\\\ \n');
fprintf(error_file, '\t\t\\multicolumn{2}{c|}{Configuration} & RMS [\\%%] & $\\psi$ [\\%%] & RMS [\\%%] & $\\psi$ [\\%%]  \\\\ \n');
for code_idx = 1:size(EXP_CODES,2)
  fprintf(error_file, '\t\t\\midrule\n');
  % fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', EXP_CODES{code_idx});
	fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', STRATEGIES{code_idx});
  for method_idx = 1:size(METHOD,2)
    fprintf(error_file, '\t\t\t\t& %s & %6.3f & %6.2f', METHOD{method_idx}, ...
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi);
      switch EXP_CODES{code_idx}
        case {'A', 'B', 'G', 'H', 'I', 'J'}
          fprintf(error_file, '& %6.3f & %6.2f ', ...
            ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
            ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi);
        case {'C', 'D', 'E', 'F'}
          fprintf(error_file, '& %6.3f & %6.2f ', ...
            ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
            ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi);
      end
    fprintf(error_file, '\\\\\n');
  end
end
% fprintf(error_file, '\t\t\\midrule\n');
% for code_idx = 1:size(EXP_CODES,2)
%   fprintf(error_file, '\t\t\t%s ', EXP_CODES{code_idx});
%   fprintf(error_file, '& %6.2f & %6.2f', uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).RMS, uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).phi);
%     switch EXP_CODES{code_idx}
%       case {'A', 'B'}
%         fprintf(error_file, '& %6.2f & %6.2f ', ...
%           ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).RMS, ...
%           ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).phi);
%       case {'C', 'D', 'E', 'F'}
%         fprintf(error_file, '& %6.2f & %6.2f ', ...
%           ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).RMS, ...
%           ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).phi);
%     end
%   fprintf(error_file, '\\\\\n');
% end

fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\\end{tabular}\n');
fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1}\n');
fprintf(error_file, '\t%%\\renewcommand{\\arraystretch}{2}\n');
fprintf(error_file, '\t%%vspace{-8pt}%%use this in case the image is in the margin\n');
fprintf(error_file, '\\end{table}\n');

end


% 	\begin{tabular}{c | c c | c c }
% 	\toprule
% 	\toprule
% 	& \multicolumn{2}{c|}{UAV}
% 	& \multicolumn{2}{c}{\textsub{UGV}{1}}  \\
% 	Experiment & RMS [m] & $\psi$ [deg] & RMS [m] & $\psi$ [deg] \\
% 	\midrule
% 	A & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	B & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	C & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	D & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	E & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	F & 0.00 & 0.00 & 0.00 & 0.00 \\
% 	\toprule
% 	\toprule
% 	\end{tabular}
% 	\renewcommand{\arraystretch}{1}
% \end{table}
