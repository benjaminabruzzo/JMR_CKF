function printfig(meta, fig_handle)
%%
    warning('off', 'MATLAB:MKDIR:DirectoryExists');
    fig_handle.PaperUnits = 'inches';
    ext = {'eps', 'png'}; print_param = {'-depsc', '-dpng'};
    for ext_idx = 1:size(ext,2)
      try mkdir(sprintf('%s/%s/', fig_handle.UserData, ext{ext_idx})); end
      disp(['       ' sprintf('%s/%s/%s.%s', fig_handle.UserData, ext{ext_idx}, fig_handle.FileName, ext{ext_idx})])
      if (~exist(sprintf('%s/%s/%s.%s', fig_handle.UserData, ext{ext_idx}, fig_handle.FileName, ext{ext_idx})))
        print(fig_handle, sprintf('%s/%s/%s.%s', fig_handle.UserData, ext{ext_idx}, fig_handle.FileName, ext{ext_idx}), print_param{ext_idx}, '-r500'); 
      end
    end
    warning('on', 'MATLAB:MKDIR:DirectoryExists');

end


%     ext = {'eps'}; print_param = {'-depsc'};
%     ext = {'png'}; print_param = {'-dpng'};


%     if (~exist([fig_handle.FileName '.eps'])); print(fig_handle,  [fig_handle.FileName '.eps'],'-depsc', '-r500'); end
%     if (~exist([fig_handle.FileName '.png'])); print(fig_handle,  [fig_handle.FileName '.png'],'-dpng' , '-r500');  end
