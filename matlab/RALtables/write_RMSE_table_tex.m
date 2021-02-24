function write_RMSE_table_tex(batch_data)
%%

RAL_path = '/home/benjamin/ros/src/metahast/matlab/RALtables';
tex_filename = '672_RMSE_table.tex';

disp([RAL_path '/' tex_filename]);
error_file = fopen([RAL_path '/' tex_filename], 'w');

% EXP_CODES = {'A', 'B', 'C', 'D', 'E', 'F'};
EXP_CODES = batch_data.EXP_CODES;

method = {'ckf',  'slam'};
METHOD = {'CKF',  'SLAM'};

for method_idx = 1:size(method,2) %CKF vs SLAM
  for code_idx = 1:size(EXP_CODES,2) % A-F
    uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi = ...
      batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).uav.(matlab.lang.makeValidName(method{method_idx})).MTE_error.yaw;
%       batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.ckf.MTE_error.yaw(idx,1)  = errors.uav.ckf.yaw_mean;
%       batch_data.(matlab.lang.makeValidName(meta.exp_code)).uav.slam.MTE_error.yaw(idx,1) = errors.uav.slam.yaw_mean;
      
    for trial_idx = 1:length(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr)
      try % UAV
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).uav.(matlab.lang.makeValidName(method{method_idx})).p_mean;
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).uav.(matlab.lang.makeValidName(method{method_idx})).p_std;
      
      
      
%       uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi(trial_idx,1) = ...
%         batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
%         batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).uav.(matlab.lang.makeValidName(method{method_idx})).yaw_mean;
      uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).uav.(matlab.lang.makeValidName(method{method_idx})).yaw_std;
      
      
      catch Mexc_; qdisp(['    write RMSE table ugv1, ' EXP_CODES{code_idx} '.' batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
      try % UGV1
      ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv1.(matlab.lang.makeValidName(method{method_idx})).p_mean;
      ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv1.(matlab.lang.makeValidName(method{method_idx})).p_std;
      ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv1.(matlab.lang.makeValidName(method{method_idx})).yaw_mean;
      ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv1.(matlab.lang.makeValidName(method{method_idx})).yaw_std;      
      catch Mexc_; qdisp(['    write RMSE table ugv1, ' EXP_CODES{code_idx} '.' batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
      try      % UGV2
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv2.(matlab.lang.makeValidName(method{method_idx})).p_mean;
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv2.(matlab.lang.makeValidName(method{method_idx})).p_std;
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv2.(matlab.lang.makeValidName(method{method_idx})).yaw_mean;
      ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi_std(trial_idx,1) = ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(sprintf( 'errors_%s', ...
        batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx}))).ugv2.(matlab.lang.makeValidName(method{method_idx})).yaw_std;
    catch Mexc_; qdisp(['    write RMSE table ugv2, ' EXP_CODES{code_idx} '.' batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).jstr{trial_idx} ',' Mexc_.identifier ' :: ' Mexc_.message]);  end
  
    end
  end
end

caption = 'Average position and yaw errors of the of the vehicle pose estimates for \textsub{UGV}{1} and the UAV averaged across all trials of each configuration.';
label = 'tab:RMSE';

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
fprintf(error_file, '\t\t& &\\multicolumn{2}{c|}{UAV} ');
fprintf(error_file, '& \\multicolumn{2}{c}{\\textsub{UGV}{1}}  \\\\ \n');
fprintf(error_file, '\t\t\\multicolumn{2}{c|}{Configuration} & RMS [cm] & $\\psi$ [^o] & RMS [cm] & $\\psi$ [^o]  \\\\ \n');
for code_idx = 1:size(EXP_CODES,2)
  fprintf(error_file, '\t\t\\midrule\n');
  fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', EXP_CODES{code_idx});
  for method_idx = 1:size(method,2)
    fprintf(error_file, '\t\t\t\t& %s & %6.3f & %6.2f', METHOD{method_idx}, ...
      100*mean(uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS), ...
      (180/pi)*mean(uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi));
      switch EXP_CODES{code_idx} 
        case {'A', 'B', 'G', 'H', 'I', 'J'}
          fprintf(error_file, '& %6.3f & %6.2f ', ...
          100*mean(ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS), ...
          (180/pi)*mean(ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi));
        case {'C', 'D', 'E', 'F'}
          fprintf(error_file, '& %6.3f & %6.2f ', ...
          100*mean(ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).RMS), ...
          (180/pi)*mean(ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi));
      end
    fprintf(error_file, '\\\\\n');
  end
end

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


