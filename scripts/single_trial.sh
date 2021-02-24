#!/bin/bash
DATE=$1
TRIAL="$(printf "%03d" $2)"
CODE=$3
# SHOW_RVIS=$4

# bash gazebo_iterant_$CODE.sh $DATE $TRIAL;
bash gazebo_iterant_$CODE.sh $DATE $TRIAL $SHOW_RVIS;
sleep 30;
killall screen

echo "process_iteration('$DATE', '$TRIAL', '$CODE')" | mail -s "process_iteration('$DATE', '$TRIAL', '$CODE')" gutter.puddles@gmail.com
matlab -nodisplay -nosplash -nojvm -r "process_iteration('$DATE', '$TRIAL', '$CODE');exit;"
