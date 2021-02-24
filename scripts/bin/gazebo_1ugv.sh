#!/bin/bash
# file: run-youbot-ros-startup.sh

DATE=$1
TRIAL=$2

printf "launching roscore     ... "
screen -dmS ros_core ros_core.sh
sleep 2
printf " launched.\n"

printf "launching gazebo_init ... "
# echo "screen -dmS gazebo_init gazebo_init.sh $DATE $TRIAL"
# echo "screen -dmS gazebo_init"
screen -dmS gazebo_init
screen -S gazebo_init -p 0 -X stuff "source ~/.bashrc^M"
screen -S gazebo_init -p 0 -X stuff "roslaunch hast_gazebo init.launch date:=$DATE trial:=$TRIAL^M"
sleep 5
printf " launched.\n"

printf "launching ugv1_base   ... "
# echo "screen -dmS ugv1_base"
screen -dmS ugv1_base
screen -S ugv1_base -p 0 -X stuff "source ~/.bashrc^M"
screen -S ugv1_base -p 0 -X stuff "roslaunch hast_gazebo ugv1_base.launch^M"
sleep 2
printf " launched.\n"

printf "launching ugv2_base   ... "
screen -dmS ugv2_base
screen -S ugv2_base -p 0 -X stuff "source ~/.bashrc^M"
screen -S ugv2_base -p 0 -X stuff "roslaunch hast_gazebo ugv2_base.launch^M"
sleep 2
printf " launched.\n"

sleep 5
printf "launching trial       ... "
screen -dmS trial
screen -S trial -p 0 -X stuff "source ~/.bashrc^M"
screen -S trial -p 0 -X stuff "roslaunch hast_gazebo trial.launch oneUGV:=true^M"
sleep 2
printf " launched.\n"

sleep 5
printf "launching failswitch  ... "
screen -dmS failswitch
screen -S trial -p 0 -X stuff "source ~/.bashrc^M"
screen -S trial -p 0 -X stuff "roslaunch hast wait_for_trial_finish.launch^M"
sleep 2
printf " launched.\n\n"


screen -ls
printf "\nlaunching countdown node: \n"
rosparam dump ~/ros/data/$1/$2/param_dump.yaml
roslaunch hast countdown.launch
