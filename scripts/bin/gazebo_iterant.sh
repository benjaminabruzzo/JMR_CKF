#!/bin/bash
export HOST=$HOSTNAME
source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros

# create selective kill
function KILL_SCREENS
{ # KILL_SCREENS "${SCREEN_LIST[@]}"W
	f_SCREEN_LIST=("$@")
	for TAG in "${f_SCREEN_LIST[@]}";
	do
		unset KILL_TAG; KILL_TAG="$(screen -list | grep $TAG | cut -d'(' -f1 ) ";
		if [ -z "$KILL_TAG" ]
		then
			echo "    killing: $KILL_TAG"
			screen -X -S $KILL_TAG kill
		fi
	done
}

function CHECK_SCREENS()
{ # KILL_SCREENS "${SCREEN_LIST[@]}"
	f_SCREEN_LIST=("$@")
	f_RESULT=true
	f_dummy=true

	for f_TAG in "${f_SCREEN_LIST[@]}";
	do
		unset f_SCREEN_TAG; f_SCREEN_TAG="$(screen -list | grep $f_TAG | cut -d'(' -f1 ) ";
		if screen -list | grep $f_TAG > /dev/null
		then
			# echo "    $f_SCREEN_TAG found"
			# do nothing
			f_dummy=true
		else
			# echo "    no $f_TAG screen found"
			f_RESULT=false
		fi
	done

	echo $f_RESULT
}

unset SCREEN_LIST

DATE=$1
TRIAL=$2
EXP_CODE=$3
SHOW_RVIS=$4


printf "\n${HOST} gazebo_iterant.sh $DATE, $TRIAL, $EXP_CODE\n"
mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/config"; cp /home/benjamin/ros/src/metahast/robot_descriptions/tags/launch/tags_rand.launch /home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/config/tags_rand.launch
mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/prealloc"
LOGPATH="/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/logs";
rm -rf $LOGPATH
mkdir -p $LOGPATH
mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/figs/eps"
mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/figs/png/stereo/"

printf "  screen -r #####.name_name for viewing screen, [ctrl]+[a]+[d] to exit\n"

unset SCREEN_NAME; SCREEN_NAME="gazebo_init"
	printf "  launching ${SCREEN_NAME}  ... "; SCREEN_LIST=("${SCREEN_NAME}")
	screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
	screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
	screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
	screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo init.launch date:=$DATE exp_code:=$EXP_CODE trial:=$TRIAL ; exit^M"
	sleep 5
	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
	printf " as $SCREEN_TAG\n"

unset SCREEN_NAME; SCREEN_NAME="uav_rviz"
	printf "  launching ${SCREEN_NAME}     ... "; SCREEN_LIST+=("${SCREEN_NAME}")
	screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
	screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
	screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
	screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo init_uav_rviz.launch show_rviz:=$SHOW_RVIS map:=$5 ; exit^M"
	sleep 5
	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
	if [ $SHOW_RVIS == "true" ]; then
		printf " as $SCREEN_TAG\n"
	else
		printf " as $SCREEN_TAG without rvis.\n"
	fi

unset SCREEN_NAME; SCREEN_NAME="ugv1_base"
	printf "  launching ${SCREEN_NAME}    ... "; SCREEN_LIST+=("${SCREEN_NAME}")
	screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
	screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
	screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
	screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo ugv1_base.launch exp_code:=$EXP_CODE ; exit^M"
	sleep 3
	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
	printf " as $SCREEN_TAG\n"

unset SCREEN_NAME; SCREEN_NAME="ugv2_base"
case $EXP_CODE in
	A|B)
		printf "   skipping ${SCREEN_NAME} \n"
		sleep 1
		;;
	*)
		printf "  launching ${SCREEN_NAME}    ... "; SCREEN_LIST+=("${SCREEN_NAME}")
		screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
		screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
		screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
		screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
		screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
		screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo ugv2_base.launch exp_code:=$EXP_CODE ; exit^M"
		unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
		printf " as $SCREEN_TAG\n"
		sleep 3
		;;
esac

unset SCREEN_NAME; SCREEN_NAME="trial"
printf "  launching ${SCREEN_NAME} $EXP_CODE      ... "; SCREEN_LIST+=("${SCREEN_NAME}")
sleep 5
screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
	case $EXP_CODE in
		A|B) screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=true ; exit^M" ;;
		C)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv1_lookatgoal:=true  ugv1_watchugv2:=false ugv2_picket:=false ugv2_w_hover:=true ; exit^M" ;;
		D)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv1_lookatgoal:=false ugv1_watchugv2:=true  ugv2_picket:=false ugv2_w_hover:=true ; exit^M" ;;
		E)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv1_lookatgoal:=true  ugv1_watchugv2:=false ugv2_picket:=true  ugv2_w_hover:=false ; exit^M" ;;
		F)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv1_lookatgoal:=false ugv1_watchugv2:=true  ugv2_picket:=true  ugv2_w_hover:=false ; exit^M" ;;
		G)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv2_lookatgoal:=true  ugv2_watchugv1:=false ugv1_picket:=false ugv1_w_hover:=true ; exit^M" ;;
		H)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv2_lookatgoal:=false ugv2_watchugv1:=true  ugv1_picket:=false ugv1_w_hover:=true ; exit^M" ;;
		I)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv2_lookatgoal:=true  ugv2_watchugv1:=false ugv1_picket:=true  ugv1_w_hover:=false ; exit^M" ;;
		J)   screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv2_lookatgoal:=false ugv2_watchugv1:=true  ugv1_picket:=true  ugv1_w_hover:=false ; exit^M" ;;
		*) echo "No matching EXP_CODE found" ;;
	esac
sleep 5
unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
printf " as $SCREEN_TAG\n"

rosparam dump ~/ros/data/$1/$EXP_CODE/$2/config/param_dump.yaml;

unset SCREEN_NAME; SCREEN_NAME="failswitch"
	printf "  launching ${SCREEN_NAME}   ... "; SCREEN_LIST+=("${SCREEN_NAME}")
	sleep 5
	screen -dmS ${SCREEN_NAME}                        						#Start detached screen session
	screen -S ${SCREEN_NAME} -X logfile $LOGPATH/${SCREEN_NAME}$TRIAL$EXP_CODE.txt #Use logfile command to set logging file
	screen -S ${SCREEN_NAME} -X log        						            #Use log command to start logging
	screen -S ${SCREEN_NAME} -p 0 -X stuff "export HOST=$HOSTNAME^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
	screen -S ${SCREEN_NAME} -p 0 -X stuff "roslaunch hast wait_for_trial_finish.launch ; exit^M"
	sleep 3
	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
	printf " as $SCREEN_TAG\n"

counter=0
result=true
while $result
# while screen -list | grep Detached > /dev/null
do
	if [ $counter -gt 400 ]
	then
		# KILL_SCREENS "${SCREEN_LIST[@]}"
		killall screen
		break
	fi
	result=$(CHECK_SCREENS ${SCREEN_LIST[@]})   # or result=`myfunc`
	counter=$(( $counter + 1 ))
	echo -ne "  waiting for finish ... $counter ... \r"
	sleep 1
done

printf "\n\n  done. \n\n"

# KILL_SCREENS "${SCREEN_LIST[@]}"
killall screen
