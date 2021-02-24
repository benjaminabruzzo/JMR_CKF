function write_EvMap_table_tex(batch_data, meta)
%%
	RAL_path = '/home/benjamin/ros/src/metahast/matlab/RALtables';
	tex_filename = '678_EvMap_table.tex';

	disp([RAL_path '/' tex_filename]);
	error_file = fopen([RAL_path '/' tex_filename], 'w');

	% EXP_CODES = {'A', 'B', 'C', 'D', 'E', 'F'};
	EXP_CODES = batch_data.EXP_CODES;

	METHOD = {'CKF ', 'SLAM'};
	method = {'ckf',  'slam'};

	STRATEGIES={...
	  '$\prescript{}{1}{\bm{C}}_{}^{H}$';... %A
	  '$\prescript{}{1}{\bm{C}}_{}^{L}$';... %B
	  '$\prescript{}{2}{\bm{C}}_{G}^{H}$';... %CG
	  '$\prescript{}{2}{\bm{C}}_{R}^{H}$';... %DH
	  '$\prescript{}{2}{\bm{C}}_{G}^{L}$';... %EI
	  '$\prescript{}{2}{\bm{C}}_{R}^{L}$'}; %FJ

  
  cap{1} = 'Mean position errors of all vehicle estimates averaged across all trials of each strategy for each map. ';
  cap{2} = 'RMSE is calculated by comparing the entire trajectory of all vehicles to the ground truth. ';
  cap{3} = 'Mean RMSE is calculated by averaging all of the errors of the entire trajectories of all vehicles to the ground truth for all maps. ';
  cap{4} = 'The mean percentage is the average of the total errors of all vehicles at the end of each trajectory for all iterations of all maps. ';
  
  caption = []; for cap_idx = 1:size(cap,2)
    caption = [caption cap{cap_idx}];
  end
% 	caption = 'Mean position errors of all vehicle estimates averaged across all trials of each strategy for each map. ';
	label = 'tab:avg_errors';

 
  % Smaller table
	fprintf(error_file, '\\begin{table}\n');
	fprintf(error_file, '\t%% \\tiny\n');
  fprintf(error_file, '\t%% \\setlength{\\tabcolsep}{5pt} %% Default value: 6pt\n');
	fprintf(error_file, '\t%% \\fontsize{9pt}{9.25pt}\\selectfont\n');
	fprintf(error_file, '\t%% \\renewcommand\\arraystretch{2}\n');
	fprintf(error_file, '\t\\centering\n');
	fprintf(error_file, '\t\\caption{%s}\n', caption);
	fprintf(error_file, '\t\\label{%s}\n', label);
	fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1.25}\n');
	fprintf(error_file, '\t\\begin{tabular}{c c | c  c  c  c | c c}\n');
	fprintf(error_file, '\t\t\\toprule\n');
	fprintf(error_file, '\t\t\\toprule\n');
	fprintf(error_file, '\t\t & Map \\#  & 1 & 2 & 3 & 4 & \\multicolumn{2}{c}{Mean}\\\\ \n');
  fprintf(error_file, '\t\t\\multicolumn{2}{c|}{Strategy} &\\multicolumn{4}{c|}{RMSE [cm]} & [cm] & [\\%%] \\\\ \n');

  	for code_idx = 1:size(EXP_CODES,2)
	  fprintf(error_file, '\t\t\\midrule\n');
	  fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', STRATEGIES{code_idx});
	  for method_idx = 1:size(METHOD,2)
      all_uav = []; all_ugv1 = []; all_ugv2 = [];
      fprintf(error_file, '\t\t\t\t& %s ', METHOD{method_idx});
      
      for map_idx = 1:size(meta.maps,1)
        map_str = sprintf('map_%i', meta.maps{map_idx});
        all_uav  = [all_uav  100*batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms];
        all_ugv1 = [all_ugv1 100*batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D];
				switch EXP_CODES{code_idx}
					case {'A', 'B'}
		        fprintf(error_file, '& %6.1f ', ...
		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms) + ...
		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D));
					case {'G', 'H', 'I', 'J'}
            all_ugv2 = [all_ugv2 100*batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D];
		        fprintf(error_file, '& %6.1f ', ...
		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms) + ...
		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D) + ...
							100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D));
				end % switch EXP_CODES{code_idx}
      end % for map_idx = 1:size(meta.maps,1)
      switch EXP_CODES{code_idx}
        case {'A', 'B'}
          fprintf(error_file, '& %6.1f & %6.2f \\\\\n', ...
            mean(all_uav) + mean(all_ugv1), ...
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).(matlab.lang.makeValidName(method{method_idx})).percent);
        case {'G', 'H', 'I', 'J'}
          fprintf(error_file, '& %6.1f & %6.2f \\\\\n', ...
            mean(all_uav) + mean(all_ugv1) + mean(all_ugv2), ...
            batch_data.end_errors.(matlab.lang.makeValidName(meta.exp_codes{code_idx})).(matlab.lang.makeValidName(method{method_idx})).percent);
      end % switch EXP_CODES{code_idx}
    end % for method_idx = 1:size(METHOD,2)
	end % for code_idx = 1:size(EXP_CODES,2)
  
	fprintf(error_file, '\t\t\\toprule\n');
	fprintf(error_file, '\t\t\\toprule\n');
	fprintf(error_file, '\t\\end{tabular}\n');
	fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1}\n');
	fprintf(error_file, '\t%%\\renewcommand{\\arraystretch}{2}\n');
	fprintf(error_file, '\t%%vspace{-8pt}%%use this in case the image is in the margin\n');
	fprintf(error_file, '\\end{table}\n');
  

  fprintf(error_file, '\t%% mean(batch_data.end_errors.all_errors.dist) = %6.4f\n', mean(batch_data.end_errors.all_errors.dist));
  fprintf(error_file, '\t%% mean(batch_data.end_errors.all_errors.ckf)  = %6.4f\n', mean(batch_data.end_errors.all_errors.ckf));
  fprintf(error_file, '\t%% mean(batch_data.end_errors.all_errors.slam) = %6.4f\n', mean(batch_data.end_errors.all_errors.slam));
  fprintf(error_file, '\t%% mean(batch_data.end_errors.all_errors.ckf_percent)  = %6.2f\n', mean(batch_data.end_errors.all_errors.ckf_percent));
  fprintf(error_file, '\t%% mean(batch_data.end_errors.all_errors.slam_percent) = %6.2f\n', mean(batch_data.end_errors.all_errors.slam_percent));
  
  
  
  
%   %  table with yaw data
%  	fprintf(error_file, '\\begin{table*}\n');
% 	fprintf(error_file, '\t%% \\tiny\n');
% 	fprintf(error_file, '\t%% \\fontsize{9pt}{9.25pt}\\selectfont\n');
% 	fprintf(error_file, '\t%% \\renewcommand\\arraystretch{2}\n');
% 	fprintf(error_file, '\t\\centering\n');
% 	fprintf(error_file, '\t\\caption{%s}\n', caption);
% 	fprintf(error_file, '\t\\label{%s}\n', label);
% 	fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1.25}\n');
% 	fprintf(error_file, '\t\\begin{tabular}{c c | c c | c c | c c | c c}\n');
% 	fprintf(error_file, '\t\t\\toprule\n');
% 	fprintf(error_file, '\t\t\\toprule\n');
% 	fprintf(error_file, '\t\t& &\\multicolumn{2}{c|}{map 1}' );
% 	fprintf(error_file, '& \\multicolumn{2}{c|}{map 2} ');
% 	fprintf(error_file, '& \\multicolumn{2}{c|}{map 3} ');
% 	fprintf(error_file, '& \\multicolumn{2}{c}{map 4} \\\\ \n');
% 	fprintf(error_file, '\t\t\\multicolumn{2}{c|}{Strategy} & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] & RMS [cm] & $\\psi$ [deg] \\\\ \n');
% 	for code_idx = 1:size(EXP_CODES,2)
% 	  fprintf(error_file, '\t\t\\midrule\n');
% 	  fprintf(error_file, '\t\t\t\\multirow{2}{*}{%s}\n', STRATEGIES{code_idx});
% 	  for method_idx = 1:size(METHOD,2)
%       fprintf(error_file, '\t\t\t\t& %s ', METHOD{method_idx});
%       for map_idx = 1:size(meta.maps,1)
%         map_str = sprintf('map_%i', meta.maps{map_idx});
% 				switch EXP_CODES{code_idx}
% 					case {'A', 'B'}
% 		        fprintf(error_file, '& %6.2f & %6.2f', ...
% 		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms) + ...
% 		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D), ...
% 		          			mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).yaw.abs)*180/pi +...
% 		                mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).yaw.abs)*180/pi);
% 					case {'G', 'H', 'I', 'J'}
% 		        fprintf(error_file, '& %6.2f & %6.2f', ...
% 		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).p.rms) + ...
% 		          100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D) + ...
% 							100 * mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).p.rms_2D), ...
% 		          			mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.uav.(matlab.lang.makeValidName(method{method_idx})).yaw.abs)*180/pi  +...
% 		                mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv1.(matlab.lang.makeValidName(method{method_idx})).yaw.abs)*180/pi + ...
% 										mean(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).mean.ugv2.(matlab.lang.makeValidName(method{method_idx})).yaw.abs)*180/pi);
% 				end % switch EXP_CODES{code_idx}
%       end % for map_idx = 1:size(meta.maps,1)
% 	    fprintf(error_file, '\\\\\n');
%     end % for method_idx = 1:size(METHOD,2)
% 	end % for code_idx = 1:size(EXP_CODES,2)
% 
% 	fprintf(error_file, '\t\t\\toprule\n');
% 	fprintf(error_file, '\t\t\\toprule\n');
% 	fprintf(error_file, '\t\\end{tabular}\n');
% 	fprintf(error_file, '\t\\renewcommand{\\arraystretch}{1}\n');
% 	fprintf(error_file, '\t%%\\renewcommand{\\arraystretch}{2}\n');
% 	fprintf(error_file, '\t%%vspace{-8pt}%%use this in case the image is in the margin\n');
% 	fprintf(error_file, '\\end{table*}\n');
%   fprintf(error_file, '\\\\ \\\\\n');
  
end

% Note. The cumulative tracking RMSE adds up from the tracking RMSE of
% both agents and is calculated using the online pose estimates for each
% frame after it was processed by tracking. All values are averaged over
% three runs for each experiment. Bold values indicate the best performing
% system for a specific experiment.
% RMSE: root mean squared error.  
