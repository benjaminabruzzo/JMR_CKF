function process_iteration(date, trial, exp_code)
% process_iteration('20200610', '025', 'E')
% matlab -nodisplay -nosplash -nojvm -r "process_iteration('20200610', '025', 'F');exit;"
%   setpref('display','quiet', true); 
  setpref('display','quiet', false); 

  meta.date = [date '/'];
  meta.run = trial;
  meta.exp_code = exp_code;
  meta.dataroot = '/home/benjamin/ros/data/';
  meta.datapath = ['/home/benjamin/ros/data/' meta.date meta.exp_code '/' meta.run '/'];

  try data = loadJointSlamData(meta); end
  
  try
    %%
    if (strcmp(meta.exp_code,"A")||strcmp(meta.exp_code,"B"))
%       data = Code_errors(meta, data, {'ugv1'});
      trial_data = Code_errors(meta, data, {'ugv1'});
    else
%       data = Code_errors(meta, data, {'ugv1', 'ugv2'});
      trial_data = Code_errors(meta, data, {'ugv1', 'ugv2'});
    end
  end
  
  
  if usejava('jvm')
    set(0,'DefaultFigureVisible','off')
    meta.saveplots = true;
    meta.saveirosplots = false;
    meta.saveplotroot = ['/home/benjamin/ros/data/' meta.date 'figs/bulk'];
    data.printplots = [];

    %% figure(1); clf; current_fig = gcf; fprintf('figure(%d)\n', current_fig.Number); 
      figure(1); clf; current_fig = gcf;
        hold on; 
        title([meta.date meta.run ' ' meta.exp_code])

          ii = 0; 
        try h1=plot( data.ugv2_mux.est.Position_gl(:,1), data.ugv2_mux.est.Position_gl(:,2), ...
                  'b.', 'displayname', ['ugv2 slam (+odom)']); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        try h1=plot( data.ugv2_ckf.est.Position_gl(:,1), data.ugv2_ckf.est.Position_gl(:,2), ...
                  'r.', 'displayname', ['ugv2 ckf']); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        try h1=plot( data.ugv1_mux.stereo.ugv_sensor.Position_gl(:,1), data.ugv1_mux.stereo.ugv_sensor.Position_gl(:,2), ...
                  'mo', 'displayname', ['ugv1/ugv2 sensor']); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        try h1=plot( data.vicon.ugv2.position.gl(:,1), data.vicon.ugv2.position.gl(:,2), ...
                  'k.', 'displayname', ['ugv2 vicon']); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end

      %   for i = [0,1,4,5,8,9] % gazebo
      tag_list = [];
      tag_scatter = [];
        for i = 0:12 % vicon
          apr_str = sprintf( 'april%02d', i );
          try 
            xt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,1);
            yt = data.vicon.(matlab.lang.makeValidName(apr_str)).position.gl(1,2);
            plot( xt, yt, 'k*', 'displayname', apr_str);
            text(xt,yt,apr_str, 'FontSize',8);
            tag_list = [tag_list; apr_str];
            tag_scatter = [tag_scatter; data.vicon.(matlab.lang.makeValidName(apr_str)).position.vic(1,1:2)];
          catch; end
          try plot( data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,1), ...
                    data.fullSLAM.april.(matlab.lang.makeValidName(sprintf( 'tag_%02d', i ))).slam.EstPosition_gl(:,2), ...
                    '.', 'displayname', data.fullSLAM.april.names{i}); catch; end
        end


        try text(data.vicon.ugv2.position.gl(end,1), data.vicon.ugv2.position.gl(end,2), 'ugv2', 'FontSize',8); end
        try text(data.vicon.ugv1.position.gl(end,1), data.vicon.ugv1.position.gl(end,2), 'ugv1', 'FontSize',8); end

        try plot(data.vicon.ugv1.position.gl(:,1), data.vicon.ugv1.position.gl(:,2), 'k.'); catch; end
        try plot(data.ugv1_mux.est.Position_gl(:,1), data.ugv1_mux.est.Position_gl(:,2), '.'); catch; end


        hold off
        grid on
%         try columnlegend(3,legend_str); catch; end; clear legend_str

        ylabel('y [m]')
        xlabel('x [m]')
        data.printplots = [data.printplots; current_fig.Number];
        current_fig.FileName = sprintf('%s_%s_%s','landmarks_top_down', meta.run, meta.exp_code); 
        data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', current_fig.Number))).handle = current_fig; 
        clear current_fig h1 ii tag_str legend_str 

    %% print plots
    % meta.saveplotroot = ['/home/benjamin/ros/data/' meta.date 'bulk'];
    if meta.saveplots
      qdisp('Saving figures...'); tic
      for idx = 1:length(data.printplots)
        qdisp(['    Saving ' num2str(idx) ' of ' num2str(length(data.printplots)) ' figures...']);
        current_fig = data.fig_handles.(matlab.lang.makeValidName(sprintf('fig_%d', data.printplots(idx)))).handle;
        current_fig.UserData = sprintf('%s/',meta.saveplotroot); 

        printfig(current_fig)
      end
      qdisp(['  figures saved, ' num2str(toc) ' seconds']);
    end
  
  end
end


function printfig(handle)
    handle.PaperUnits = 'inches';
    handle.PaperPosition = [0 0 15 8];
%     disp(['       ' handle.UserData 'eps/' handle.FileName '.eps'])
%     print(handle,  [handle.UserData 'eps/' handle.FileName '.eps'],'-depsc', '-r500'); 
    print(handle,  [handle.UserData handle.FileName '.png'],'-dpng', '-r500'); 
end
