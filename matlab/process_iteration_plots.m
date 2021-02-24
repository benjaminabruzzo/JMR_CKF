function process_iteration_plots(DATE, TRIALS, EXP_CODES)

  switch nargin
    case 3
      % disp('process_iteration_plots called with 3 arguments')
      % disp('DATE: '); disp(DATE)
      % disp('TRIALS: '); disp(TRIALS)
      % disp('EXP_CODES: '); disp(EXP_CODES)
    case 0
      DATE = '20200610';
      TRIALS = {'024', '025'};
      EXP_CODES = {'A', 'B', 'C', 'D', 'E', 'F'};
    otherwise
      return
  end


  for idx_trial = 1:length(TRIALS)
    for idx_code = 1:length(EXP_CODES)
      close all
      set(0, 'DefaultFigureVisible', 'on');
      warning('off','images:imshow:magnificationMustBeFitForDockedFigure')
      figHandles = findall(0, 'Type', 'figure');
      set(figHandles(:), 'visible', 'on');
        process_iteration(DATE, TRIALS{idx_trial}, EXP_CODES{idx_code})
    end
  end

end


%{
batch file call
matlab -nodisplay -nosplash -r "process_iteration('20200610', '025', 'F');exit;"

DATE="20200610";
TRIALS="{'024', '025'}";
EXP_CODES="{'A', 'B', 'C', 'D', 'E', 'F'}";

process_iteration_plots($DATE, $TRIALS, $EXP_CODES)

%}
