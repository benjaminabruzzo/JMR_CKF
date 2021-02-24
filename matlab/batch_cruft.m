

				% figure(10X-60X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%         figure(meta.fig_base_n+map_code); current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
% 	        xlabel('distance traveled [m]'); ylabel('error [cm]')
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles = [];
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names = {};
% 	        hold on
% 	          try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec, ...
% 	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_ckf,...
% 	                'r', 'displayname', 'mean ckf', 'LineWidth', 5);
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 	          try h2 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).refDistVec, ...
% 	                100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean_errors.mean_slam, ...
% 	                'b', 'displayname', 'mean slam', 'LineWidth', 5);
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 	        hold off
% 	        grid on
				% figure(11X-61X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%         figure(meta.fig_base_n+10+map_code); current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
% 	        xlabel('time [s]'); ylabel('error [cm]')
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles = [];
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names = {};
% 	        hold on
% 	          try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.p.rms_2D, ...
% 	                'Color',default_colors{3}, 'displayname', ' ugv1 mean ckf', 'LineWidth', 3);
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 						try h2 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.p.rms_2D, ...
% 		                'Color',default_colors{1}, 'displayname', ' ugv1 mean slam', 'LineWidth', 3);
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%
% 						switch meta.exp_codes{case_idx}
% 	            case {'G', 'H', 'I', 'J'}
%                 try
% 		              h3 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 											100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.p.rms_2D, ...
% 	                		'Color',default_colors{2}, 'displayname', ' ugv2 mean ckf', 'LineWidth', 3);
% 	                		meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(3,1) = h3;
% 	                		meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{3,1} = h3.DisplayName;
% 		              h4 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 											100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.p.rms_2D, ...
% 	                		'Color',default_colors{4}, 'displayname', ' ugv2 mean ckf', 'LineWidth', 3);
% 	                		meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(4,1) = h4;
% 	                		meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{4,1} = h4.DisplayName;
%                 catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 						end
% 	        hold off
% 	        grid on
% 					axis(batch_data.EvT_axes)
% 					legend( meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names); legend('location', 'NorthWest')

				% figure(12X-62X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%         figure(meta.fig_base_n+20+map_code); current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
% 	        xlabel('time [s]'); ylabel('error [cm]')
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles = [];
% 	        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names = {};
% 	        hold on
% 					for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.p.rms_2D,1)
% 	          try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.ckf.p.rms_2D(trial_idx,:), ...
% 	                'Color',default_colors{3}, 'LineWidth', 0.5);
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 						try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv1.slam.p.rms_2D(trial_idx,:), ...
% 	                'Color',default_colors{1}, 'LineWidth', 0.5);
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 					end
% 					try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 									100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.ckf.p.rms_2D, ...
% 								'Color',default_colors{3}, 'displayname', ' ugv1 mean ckf', 'LineWidth', 3);
% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
% 					catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 					try h2 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 									100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv1.slam.p.rms_2D, ...
% 	                'Color',default_colors{1}, 'displayname', ' ugv1 mean slam', 'LineWidth', 3);
% 								meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
%                 meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%           catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%
% 	        hold off
% 	        grid on
% 					axis(batch_data.EvT_axes)
% 					legend( meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names); legend('location', 'NorthWest')

				% ugv 2 ckf and slam plots
% 				switch meta.exp_codes{case_idx}
% 					case {'G', 'H', 'I', 'J'}
% 					% figure(13X-63X); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
% 	        figure(meta.fig_base_n+30+map_code); current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number);
% 		        xlabel('time [s]'); ylabel('error [cm]')
% 		        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles = [];
% 		        meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names = {};
% 		        hold on
% 						for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.p.rms_2D,1)
% 		          try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 											100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.ckf.p.rms_2D(trial_idx,:), ...
% 		                'Color',default_colors{3}, 'LineWidth', 0.5);
% 		          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 							try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 											100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).ugv2.slam.p.rms_2D(trial_idx,:), ...
% 		                'Color',default_colors{1}, 'LineWidth', 0.5);
% 		          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 						end
% 						try h1 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.ckf.p.rms_2D, ...
% 									'Color',default_colors{3}, 'displayname', ' ugv2 mean ckf', 'LineWidth', 3);
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
% 						catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
% 						try h2 = plot(batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).time,...
% 										100 * batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%i', map_code))).mean.ugv2.slam.p.rms_2D, ...
% 		                'Color',default_colors{1}, 'displayname', ' ugv2 mean slam', 'LineWidth', 3);
% 									meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
% 	                meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
% 	          catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%
% 		        hold off
% 		        grid on
% 						axis(batch_data.EvT_axes)
% 						legend( meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
% 										meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names); legend('location', 'NorthWest')
% 				end% ugv 2 ckf and slam plots :: switch meta.exp_codes{case_idx}



%       for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
% %         disp(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx})
%         figure(meta.fig_base_n+eval(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1))); current_fig = gcf;
%         title(['Experiment ' meta.exp_code ', Map ' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1) ', error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [cm]')
%         hold on
%         linewidth = 2;
%           switch meta.exp_code
%             case {'A', 'B', 'G', 'H', 'I', 'J'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.ckf.p_v_dist, ...
%                 'r', 'displayname', 'ckf', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.slam.p_v_dist, ...
%                 'b', 'displayname', 'slam', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%             case {'C', 'D', 'E', 'F'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.ckf.p_v_dist, ...
%                 'r', 'displayname', 'ckf', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.slam.p_v_dist, ...
%                 'b', 'displayname', 'slam', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%           end
%           hold off
%           grid on
%           axis(batch_data.RAL_axes)
%           legend(meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
%                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names);
%           legend('location', 'NorthWest')
%       end



%       for trial_idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
% %         disp(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx})
%         figure(meta.fig_base_n+10+eval(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1))); current_fig = gcf;
%         title(['Experiment ' meta.exp_code ', Map ' batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx}(1) ', error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [cm]')
%         hold on
%           switch meta.exp_code
%             case {'A', 'B', 'G', 'H', 'I', 'J'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.ckf.p_v_dist, ...
%                 'r', 'displayname', 'ckf', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv1.slam.p_v_dist, ...
%                 'b', 'displayname', 'slam', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%             case {'C', 'D', 'E', 'F'}
%               h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.ckf.p_v_dist, ...
%                 'r', 'displayname', 'ckf', 'LineWidth', linewidth);
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(1,1) = h1;
%               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{1,1} = h1.DisplayName;
% %               h2 = plot( ...
% %                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.dist, ...
% %                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{trial_idx} ))).ugv2.slam.p_v_dist, ...
% %                 'b', 'displayname', 'slam', 'LineWidth', linewidth);
% %               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles(2,1) = h2;
% %               meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names{2,1} = h2.DisplayName;
%           end
%           hold off
%           grid on
%           axis(batch_data.RAL_axes)
%           legend(meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_handles, ...
%                  meta.figures.(matlab.lang.makeValidName(sprintf( 'fig_%i', current_fig.Number))).legend_names);
%           legend('location', 'NorthWest')
%       end
%


%         case {'C', 'D', 'E', 'F'}
%         %% figure(X000); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%           figure(meta.fig_base_n); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
%             title(['Experiment ' meta.exp_code ', error vs all distances traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [cm]')
%             try
%               for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
%                 hold on
%                   h1 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
%                     100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
%                     'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
%                   h2 = plot( ...
%                     batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
%                     100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
%                     'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
%                 hold off
%               end
%
%               legend(legend_handles, legend_names); legend('location', 'NorthWest')
%               grid on
%               batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
%             catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%             current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_all_distances');
% %         try printfig(meta, current_fig); end
%         %% figure(X001); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%           figure(meta.fig_base_n+1); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
%             title('error vs distance for ugv2, matched distance traveled');  xlabel('distance traveled [m]'); ylabel('error [cm]')
%             try
%               for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
%                 hold on
%                   h1 = plot( ...
%                       batch_data.ugv.refDist, ...
%                       100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ckf.p, ...
%                       'r', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
%                   h2 = plot( ...
%                       batch_data.ugv.refDist, ...
%                       100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'batch_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).slam.p, ...
%                       'b', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
%                   hold off
%               end
%
%               legend(legend_handles, legend_names); legend('location', 'NorthWest')
%               grid on
%               axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
%             catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%             current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_all_errors_vs_capped_distances');
% %         try printfig(meta, current_fig); end
%         %% figure(X002); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%           figure(meta.fig_base_n+2); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
% %             title(['Configuration ' meta.exp_code ', averaged error vs distance']);
%             xlabel('distance traveled [m]'); ylabel('error [cm]')
%             try
%             hold on
%               h1 = plot(batch_data.ugv.refDist, 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_ckf, ...
%                 'r', 'displayname', 'ckf mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
%               h1 = plot(batch_data.ugv.refDist, 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).mean_errors.ugvA.mean_slam, ...
%                 'b', 'displayname', 'slam mean', 'LineWidth', linewidth ); legend_handles = [legend_handles, h1]; legend_names = {legend_names{:}, h1.DisplayName};
%             catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%             hold off
%             try legend(legend_handles, legend_names); legend('location', 'NorthWest'); end
%             grid on
% %             axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
%             axis(batch_data.RAL_axes)
%             current_fig.FileName = sprintf('%s%s%s', meta.saveplotroot, meta.exp_code, '_average_errors_vs_capped_distances');
% %         try printfig(meta, current_fig); end
%         %% figure(X100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
%         for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
%           figure(meta.fig_base_n+100+idx); clf; current_fig = gcf; legend_handles = []; legend_names = {}; % fprintf('figure(%d)\n', current_fig.Number);
%             title(['Experiment ' meta.exp_code ' error vs distance traveled by ugv1 ']); xlabel('distance traveled [m]'); ylabel('error [cm]')
%             hold on
%             try
%               h1 = plot( ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.ckf.p_v_dist, ...
%                 'r', 'displayname', sprintf( 'ckf %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
%                 legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName;
%               h2 = plot( ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.dist, ...
%                 100*batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).ugv2.slam.p_v_dist, ...
%                 'b', 'displayname', sprintf( 'slam %s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ), 'LineWidth', linewidth);
%                 legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName;
%             catch Mexc_; qdisp(['    ' sprintf('figure(%d) ', current_fig.Number) ' ' Mexc_.identifier ' :: ' Mexc_.message]); end
%             hold off
%             grid on
%             legend(legend_handles, legend_names); legend('location', 'NorthWest')
%             axis(batch_data.(matlab.lang.makeValidName(meta.exp_code)).max_axes)
%             current_fig.FileName = sprintf('%s/%s_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).figdir, batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}, '_errors_vs_distance');
% %             try printfig(meta, current_fig); end
%         end


%% figure(3); clf; current_fig = gcf; legend_handles = []; legend_names = {}; %fprintf('figure(%d)\n', current_fig.Number);
% plot everything
%     linewidth = 1;
%     figure(3); clf; current_fig = gcf; legend_handles = []; legend_names = {}; fprintf('figure(%d)\n', current_fig.Number);
%       title(['UAV:: All Experiments, error vs all distances traveled']); xlabel('distance traveled [m]'); ylabel('error [angle]')
%       hold on
%
%       for case_idx = 1:size(meta.exp_codes,2)
%         meta.exp_code = meta.exp_codes{case_idx};
% %         qdisp(sprintf('case_%s', meta.exp_codes{case_idx}))
%         for idx = 1:size(batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr,1)
%           try h1 = plot( ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).uav.angular_dist, ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).uav.ckf.yaw_abs, ...
%                 'r.', 'displayname', 'ckf', 'LineWidth', linewidth); legend_handles(1,1) = h1; legend_names{1,1} = h1.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%           try h2 = plot( ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).uav.angular_dist, ...
%                 batch_data.(matlab.lang.makeValidName(meta.exp_code)).(matlab.lang.makeValidName(sprintf( 'errors_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx} ))).uav.slam.yaw_abs, ...
%                 'b.', 'displayname', 'slam', 'LineWidth', linewidth); legend_handles(2,1) = h2; legend_names{2,1} = h2.DisplayName; catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%         end
%       end
%       hold off
%       try legend(legend_handles, legend_names); legend('location', 'NorthWest'); catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
%       grid on
%       batch_data.all_max_axes = [current_fig.CurrentAxes.XLim current_fig.CurrentAxes.YLim ];
%       current_fig.FileName = sprintf('%s%s', meta.saveplotroot, '01_all_experiments_all_errors_vs_all_distances');
%       printfig(meta, current_fig)
