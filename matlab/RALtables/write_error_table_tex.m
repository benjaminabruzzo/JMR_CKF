function write_error_table_tex(batch_data)
%%

RAL_path = '/home/benjamin/ros/src/metahast/matlab/RALtables';
tex_filename = '670_error_table.tex';

disp([RAL_path '/' tex_filename]);
error_file = fopen([RAL_path '/' tex_filename], 'w');

% EXP_CODES = {'A', 'B', 'C', 'D', 'E', 'F'};
EXP_CODES = batch_data.EXP_CODES;

METHOD = {'CKF ', 'SLAM'};
method = {'ckf',  'slam'};

STRATEGIES={'$\prescript{}{1}{\bm{C}}_{H}^{}$';... %A
'$\prescript{}{1}{\bm{C}}_{L}^{}$';... %B
'$\prescript{}{2}{\bm{C}}_{H}^{G}$';... %CG
'$\prescript{}{2}{\bm{C}}_{H}^{R}$';... %DH
'$\prescript{}{2}{\bm{C}}_{L}^{G}$';... %EI
'$\prescript{}{2}{\bm{C}}_{L}^{R}$'}; %FJ



for method_idx = 1:size(METHOD,2)
  for code_idx = 1:size(EXP_CODES,2)
    uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_error.p); 
    uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).end_error.yaw); 
    ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_error.p); 
    ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
      mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv1.(matlab.lang.makeValidName(method{method_idx})).end_error.yaw); 
    try
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv2.(matlab.lang.makeValidName(method{method_idx})).end_error.p); 
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).ugv2.(matlab.lang.makeValidName(method{method_idx})).end_error.yaw); 
    catch
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = 0; 
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = 0; 
    end
    try
      april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).april.(matlab.lang.makeValidName(method{method_idx})).end_error.p); 
      april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = ...
        mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).april.(matlab.lang.makeValidName(method{method_idx})).end_error.yaw); 
    catch
      april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS = 0; 
      april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi = 0; 
    end
  end
end



caption = 'Mean position and yaw errors of each vehicle and landmark pose estimates averaged across all trials of each map for each strategy.';
label = 'tab:avg_errors';

fprintf(error_file, '\\begin{table*}\n'); 
fprintf(error_file, '\t%% \\tiny\n'); 
fprintf(error_file, '\t%% \\fontsize{9pt}{9.25pt}\\selectfont\n'); 
fprintf(error_file, '\t%% \\renewcommand\\arraystretch{2}\n'); 
fprintf(error_file, '\t\\centering\n');
fprintf(error_file, '\t\\caption{%s}\n', caption);
fprintf(error_file, '\t\\label{%s}\n', label);
fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1.25}\n');
fprintf(error_file, '\t\\begin{tabular}{c c | c c | c c | c c | c c}\n');
fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\t\\toprule\n');
% fprintf(error_file, '\t\t& &\\multicolumn{2}{c|}{UAV} \n');
% fprintf(error_file, '\t\t& \\multicolumn{2}{c|}{\\textsub{UGV}{1}} \n');
% fprintf(error_file, '\t\t& \\multicolumn{2}{c|}{\\textsub{UGV}{2}} \n');
% fprintf(error_file, '\t\t& \\multicolumn{2}{c}{Landmarks} \\\\ \n');
fprintf(error_file, '\t\t& &\\multicolumn{2}{c|}{UAV}' );
fprintf(error_file, '& \\multicolumn{2}{c|}{\\textsub{UGV}{1}} ');
fprintf(error_file, '& \\multicolumn{2}{c|}{\\textsub{UGV}{2}} ');
fprintf(error_file, '& \\multicolumn{2}{c}{Landmarks} \\\\ \n');
fprintf(error_file, '\t\t\\multicolumn{2}{c|}{Strategy} & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] \\\\ \n');
for code_idx = 1:size(EXP_CODES,2)
  fprintf(error_file, '\t\t\\midrule\n');
  fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', STRATEGIES{code_idx});
  for method_idx = 1:size(METHOD,2)
    fprintf(error_file, '\t\t\t\t& %s & %6.2f & %6.2f', METHOD{method_idx}, ...
      100*uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
      switch EXP_CODES{code_idx} 
        case {'A', 'B'}
          fprintf(error_file, '& %6.2f & %6.2f & - & - ', ...
            100*ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
            ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
        case {'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'}
          fprintf(error_file, '& %6.2f & %6.2f ', ...
            100*ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
            ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
          fprintf(error_file, '& %6.2f & %6.2f ', ...
            100*ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
            ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
      end
    fprintf(error_file, '& %6.2f & %6.2f', ...
      100*april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
      april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
    fprintf(error_file, '\\\\\n');
  end
  
end

fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\t\\toprule\n');
fprintf(error_file, '\t\\end{tabular}\n');
fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1}\n');
fprintf(error_file, '\t%%\\renewcommand{\\arraystretch}{2}\n');
fprintf(error_file, '\t%%vspace{-8pt}%%use this in case the image is in the margin\n');
fprintf(error_file, '\\end{table*}\n'); 

end


