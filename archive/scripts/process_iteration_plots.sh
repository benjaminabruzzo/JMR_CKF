#!/bin/bash
DATE="20200610";
EXP_CODES="{'A', 'B', 'C', 'D', 'E', 'F'}";

START=10
STOP=25
TRIALS="{"
for i in $(eval echo {$START..$STOP}); do
	if (( $i == $START )); then
		TRIALS="${TRIALS}'$(printf "%03d" $i)'"
	else
		TRIALS="${TRIALS},'$(printf "%03d" $i)'"
	fi
done
TRIALS="${TRIALS}}"

matlab -nodisplay -nosplash -r "process_iteration_plots('$DATE', $TRIALS, $EXP_CODES);exit;"
# matlab -nodisplay -nosplash -nojvm -r "process_iteration_plots('$DATE', $TRIALS, $EXP_CODES);exit;"
