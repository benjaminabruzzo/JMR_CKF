function write_error_table_tex(batch_data, meta)
%%

RAL_path = '/home/benjamin/ros/src/metahast/matlab/RALtables';
tex_filename = '679_EvT_table.tex';

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

for code_idx = 1:size(EXP_CODES,2)
	switch EXP_CODES{code_idx}
		case {'A', 'B'}
			UXV={'uav', 'ugv1'};
			% UXV={'uav', 'ugv1', 'april'};
		case {'G', 'H', 'I', 'J'}
			UXV={'uav', 'ugv1', 'ugv2'};
			% UXV={'uav', 'ugv1', 'ugv2', 'april'};
	end
	for method_idx = 1:size(METHOD,2)
		for uxv_idx = 1:size(UXV,2)
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count  = 0;
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum = 0;
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.count  = 0;
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.sumsum = 0;
			for map_idx = 1:size(meta.maps,1)
				map_str = sprintf('map_%i', meta.maps{map_idx});
				switch UXV{uxv_idx}(2)
					case {'a'} %uav
						%position
						evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count  = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count + ...
									size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms,1) * ...
									size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms,2);
						evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum + ...
							 sum(sum(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms));
					case {'g'} %ugvN
						evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count  = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count + ...
									 size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms_2D,1) * ...
									 size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms_2D,2);
						evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum + ...
								sum(sum(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms_2D));
					end % switch UXV{uxv_idx}(2)
					% yaw
					evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.count  = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.count + ...
								size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).yaw.abs,1) * ...
								size(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).yaw.abs,2);
					evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.sumsum = evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.sumsum + ...
						 sum(sum(batch_data.(matlab.lang.makeValidName(EXP_CODES{code_idx})).evt.(matlab.lang.makeValidName(map_str)).(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(method{method_idx})).yaw.abs));
			end % for map_idx = 1:size(meta.maps,1)
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms = ...
				evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.sumsum/ ...
				evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.count;
			evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.rms = ...
				evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.sumsum/ ...
				evt.(matlab.lang.makeValidName(UXV{uxv_idx})).(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.count;
		end % for uxv_idx = 1:size(UXV,2)
	end % for method_idx = 1:size(METHOD,2)
end % for code_idx = 1:size(EXP_CODES,2)

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
      100*evt.uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
      		evt.uav.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.rms*180/pi);
      switch EXP_CODES{code_idx}
        case {'A', 'B'}
          fprintf(error_file, '& %6.2f & %6.2f & - & - ', ...
            100*evt.ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
			      		evt.ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.rms*180/pi);
        case {'G', 'H', 'I', 'J'}
          fprintf(error_file, '& %6.2f & %6.2f ', ...
            100*evt.ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
			      		evt.ugv1.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.rms*180/pi);
          fprintf(error_file, '& %6.2f & %6.2f ', ...
            100*evt.ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).p.rms, ...
			      		evt.ugv2.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(method{method_idx})).phi.rms*180/pi);
      end
    fprintf(error_file, '& - & -');
    % fprintf(error_file, '& %6.2f & %6.2f', ...
      % 100*april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).RMS, ...
      % april.(matlab.lang.makeValidName(EXP_CODES{code_idx})).(matlab.lang.makeValidName(METHOD{method_idx})).phi*180/pi);
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
