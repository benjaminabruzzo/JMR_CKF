#!/bin/bash

update_BAD_TRIALS() {
	for CODE in "${EXP_CODES[@]}";
	{
		# echo "CODE: $CODE"

		unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
		IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"

		CODE_BAD="${CODE}_BAD";
		CODE_BAD_LEN="#${CODE_BAD}[@]"
		eval ARRAY_END=${CODE_BAD_LEN}
		echo "ARRAY_END $ARRAY_END"


		TEMP_BAD="${CODE_BAD}[@]";
		echo "$CODE_BAD TRIALS: ${!CODE_BAD[@]}"
		echo "CODE_BAD END: ${#TEMP_BAD}"
		# echo "$CODE_BAD TRIALS: ${TRIALS[@]}"
		# echo "TEMP_BAD: ${!TEMP_BAD}"


		for TRIAL_IDX in ${TRIALS[@]};
		{
			TRIAL=$(expr $IDX \* 100 + $TRIAL_IDX); # echo $i
			# echo "$CODE_BAD TRIAL: ${TRIAL}"
			unset ARRAY_END; ARRAY_END=${#CODE_BAD}
			# echo "ARRAY_END: ${ARRAY_END}"
			eval ${CODE}_BAD[$(expr ${ARRAY_END} + 1)]="${TRIAL}"
		}
		unset TRIAL_IDX
		# echo "TEMP_BAD: ${!TEMP_BAD}"
	}

}


clear

# cd ~/ros/src/metahast && bash ./scripts/test_scripts.sh
unset DATE EXP_CODES
unset A_TRIALS B_TRIALS G_TRIALS
unset H_TRIALS I_TRIALS J_TRIALS
unset MAP_CODES MAP_LIST MAP_IDX MAP_CODE EXCLUDE_FILE
MAP_LIST=()
MAP_CODES=(1 2 3 4 5 6)

EXCLUDE_FILE="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt"
rm ${EXCLUDE_FILE}; touch ${EXCLUDE_FILE}


# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #

	DATE="20200821";					# date folder

	EXP_CODES=(A B G H I J);	# Trial strategies

	unset BAD_TRIALS; BAD_TRIALS=()
	unset A_BAD; A_BAD=(); BAD_TRIALS+=("A");
	unset B_BAD; B_BAD=(); BAD_TRIALS+=("B");
	unset G_BAD; G_BAD=(); BAD_TRIALS+=("G");
	unset H_BAD; H_BAD=(); BAD_TRIALS+=("H");
	unset I_BAD; I_BAD=(); BAD_TRIALS+=("I");
	unset J_BAD; J_BAD=(); BAD_TRIALS+=("J");

echo "H_BAD_end: ${#H_BAD}"

	# # --------------------------------------
	# IDX=1;
	# A_TRIALS="1   3   5       9          13       16       19   ";
	# B_TRIALS="    3     6 7     10                         19 20";
	# G_TRIALS="  2 3   5               12             17 18    20";
	# H_TRIALS="1     4   6     9    11                17    19 20";
	# I_TRIALS="      4 5       9 10             15 16            ";
	# J_TRIALS="1   3 4 5   7 8 9 10 11 12    14    16 17       20";
	# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# update_BAD_TRIALS;
	# # --------------------------------------
	# IDX=2;
	# A_TRIALS="  2     5 6 7   9             14          18      ";
	# B_TRIALS="1 2     5 6 7   9    11 12 13    15          19   ";
	# G_TRIALS="      4 5 6          11 12       15 16 17 18      ";
	# H_TRIALS="  2           8      11    13 14       17 18 19 20";
	# I_TRIALS="  2               10                      18    20";
	# J_TRIALS="  2         7     10 11       14 15 16            ";
	# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# update_BAD_TRIALS;
	# # --------------------------------------
	# IDX=3; MAP_LIST+=("${IDX}")
	# A_TRIALS="1       5                  13 14 15               "
	# B_TRIALS="    3             10                              "
	# G_TRIALS="      4           10 11                           "
	# H_TRIALS="                                    16            "
	# I_TRIALS="  2   4       8                                   "
	# J_TRIALS="                9    11                      19   "
	# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# update_BAD_TRIALS;
	# # --------------------------------------
	IDX=4; MAP_LIST+=("${IDX}")
	A_TRIALS="  2       6     9      12 13       16 17 18      "
	B_TRIALS="            7             13       16    18      "
	G_TRIALS="1 2         7 8   10   12 13 14          18 19 20"
	H_TRIALS="1   3     6   8   10         14    16    18 19   "
	I_TRIALS="  2 3                     13    15               "
	J_TRIALS="1             8                    16    18      "
	#X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	update_BAD_TRIALS;
	# # --------------------------------------
	# IDX=5; MAP_LIST+=("${IDX}")
	# A_TRIALS="1 2                                       18      ";
	# B_TRIALS="  2     5   7 8 9    11 12          16 17    19 20";
	# G_TRIALS="    3   5     8               14    16            ";
	# H_TRIALS="      4     7           12    14          18    20";
	# I_TRIALS="                                    16            ";
	# J_TRIALS="                  10                              ";
	# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# update_BAD_TRIALS;
	# --------------------------------------
	# IDX=6;  MAP_LIST+=("${IDX}")
	# A_TRIALS="      4 5     8      11                         20";
	# B_TRIALS="        5 6   8      11    13    15    17       20";
	# G_TRIALS="  2 3                         14                  ";
	# H_TRIALS="  2     5 6       10    12          16            ";
	# I_TRIALS="                     11    13             18 19   ";
	# J_TRIALS="1 2 3   5     8 9             14 15 16       19   ";
	# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# update_BAD_TRIALS;
	#
	# echo "BAD_TRIALS: ${BAD_TRIALS[@]}"
	# #
	#
	# for CODE in "${BAD_TRIALS[@]}";
	# {
	# 	echo "CODE: $CODE"
	# 	CODE_BAD="${CODE}_BAD";
	# 	TEMP_BAD="$CODE_BAD[@]";
	# 	echo $CODE_BAD;
	# 	# for BAD_TRIAL in "${!TEMP_BAD}";
	# 	# {
	# 	# 	# echo $BAD_TRIAL;
	# 	# 	echo "$CODE : $BAD_TRIAL"
	# 	# 	mkdir -p /mnt/evo2/$DATE/bad/$CODE/
	# 	# 	# 		# mv /mnt/evo2/$DATE/$CODE/$TRIAL /mnt/evo2/$DATE/bad/$CODE/
	# 	#
	# 	# }
	# }


# echo "A_BAD: ${A_BAD[@]}"
# echo "B_BAD: ${B_BAD[@]}"
# echo " "
#
#
# CODE="A"; CODE_BAD=${CODE}_BAD; echo "$CODE_BAD"
# TEST_BAD=${!CODE_BAD[@]}; echo "TEST_BAD: ${TEST_BAD[@]}"
#
# #
# for CODE in "${BAD_TRIALS[@]}";
# {
# 	unset CODE_BAD; CODE_BAD=${CODE}_BAD; echo "$CODE_BAD"
# 	IFS=' ' read -r -a TRIALS <<< "${!CODE_BAD}"
# 	echo ${TRIALS[@]}
# 	# echo "CODE: $CODE, CODE_BAD(${CODE_BAD}): ${!CODE_BAD[@]}"
# 	# IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"
# }


#
# # remove bad files
# echo " ";
# for TRIAL in "${BAD_TRIALS[@]}";
# {
# 		echo "$CODE : $TRIAL"
# 		# rm -rf /mnt/evo2/$DATE/bad/$CODE/
# 		# mkdir -p /mnt/evo2/$DATE/bad/$CODE/
# 		# mv /mnt/evo2/$DATE/$CODE/$TRIAL /mnt/evo2/$DATE/bad/$CODE/
# }

#
#
# # remove bad files
# echo " ";
# for CODE in "${EXP_CODES[@]}";
# {
# 	# CODE_start_time="$(date -u +%s.%N)"
# 	unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
# 	IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"
#
# 	for i in ${TRIALS[@]};
# 	{
# 		echo "$CODE : $i"
# 		rm -rf /mnt/evo2/$DATE/bad/$CODE/
# 		mkdir -p /mnt/evo2/$DATE/bad/$CODE/
# 		mv /mnt/evo2/$DATE/$CODE/$i /mnt/evo2/$DATE/bad/$CODE/
# 		# mv secrets /home/secrets
# 	}
#
# }
#
# # Archive remaining 'good' trials
# echo "All map codes: ${MAP_CODES[@]}"
# echo "Map codes for rsync: ${MAP_LIST[@]}"
#
# 	for MAP_CODE in "${MAP_LIST[@]}";
# 	{
# 		# echo "$MAP_CODE"
# 		MAP_CODES=( ${MAP_CODES[@]/$MAP_CODE} ) #Quotes when working with strings
# 		# echo ${MAP_CODES[@]}
# 	}
#
# echo "Map codes excluded from rsync: ${MAP_CODES[@]}"
# 	for MAP_CODE in "${MAP_CODES[@]}";
# 	{
# 		echo "${MAP_CODE}*" >> ${EXCLUDE_FILE}
# 	}
#
# 	for CODE in "${EXP_CODES[@]}";
# 	{
# 		rsync -av --exclude-from "${EXCLUDE_FILE}"  /mnt/evo2/20200821/$CODE /mnt/archive/20200821/
# 	}




# 	#
# 	# Run this command to exclude all directories that end with number 3:
# 	#


# rsync -av --exclude '3*' sourcedir/ destinationdir/
# rsync -av --exclude={'*.txt','dir3','dir4'} sourcedir/ destinationdir/

# rsync -av --dry-run --exclude '6*' --exclude '5*'  /mnt/evo2/20200821/A /mnt/archive/20200821/A

# unset MAP_CODES MAP_LIST MAP_IDX MAP_CODE EXCLUDE_FILE
#
# EXCLUDE_FILE="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt"
# rm ${EXCLUDE_FILE}; touch ${EXCLUDE_FILE}
#
#
# EXP_CODES=(A B G H I J);	# Trial strategies
# MAP_CODES=(1 2 3 4 5 6)
#
# # Archive remaining 'good' trials
# echo "All map codes: ${MAP_CODES[@]}"
# echo "Map codes for rsync: ${MAP_LIST[@]}"
#
# 	for MAP_CODE in "${MAP_LIST[@]}";
# 	{
# 		# echo "$MAP_CODE"
# 		MAP_CODES=( ${MAP_CODES[@]/$MAP_CODE} ) #Quotes when working with strings
# 		# echo ${MAP_CODES[@]}
# 	}
#
# echo "Map codes excluded from rsync: ${MAP_CODES[@]}"
# 	for MAP_CODE in "${MAP_CODES[@]}";
# 	{
# 		echo "${MAP_CODE}*" >> ${EXCLUDE_FILE}
# 	}
#
# 	for CODE in "${EXP_CODES[@]}";
# 	{
# 		rsync -av --dry-run --exclude-from "${EXCLUDE_FILE}"  /mnt/evo2/20200821/$CODE /mnt/archive/20200821/$CODE
# 	}



# EXCLUDE_FILE="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt"
# rm ${EXCLUDE_FILE}; touch ${EXCLUDE_FILE}
# echo "6*" | tee -a ${EXCLUDE_FILE}
#






# for MAP_CODE in "${MAP_CODES[@]}";
# {
# 	if [[ " ${EXP_CODES[@]} " =~ " ${CODE} " ]]; then
# 			# whatever you want to do when EXP_CODES contains CODE
# 			unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
# 			echo "    ${CODE}_TRIALS: ${!CODE_TRIALS}" | tee -a $TRIAL_CONFIG_LOG
# 	fi
# }
#
#


# rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/$CODE/$TRIAL/ benjamin@atlas.local:/mnt/evo2/$DATE/$CODE/$TRIAL/



# echo " "
# export HOST=$HOSTNAME
# source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros
#
# # ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
# source ~/ros/src/metahast/scripts/trial_configs/${HOST}_config.sh #  do this just to grab the date
# rm -rf ~/ros/data/$DATE && mkdir -p ~/ros/data/$DATE
#
# # LOGPATH="/home/benjamin/ros/data/$DATE/${HOST}_screen.log";
# unset SCREEN_NAME; SCREEN_NAME="${HOST}_sims"; printf "  launching $SCREEN_NAME    ... "
# 	screen -dmS $SCREEN_NAME                        						#Start detached screen session
# 	screen -S $SCREEN_NAME -X logfile /home/benjamin/ros/data/$DATE/${HOST}_screen.log #Use logfile command to set logging file
# 	screen -S $SCREEN_NAME -X log        						            #Use log command to start logging
# 	screen -S $SCREEN_NAME -p 0 -X stuff "export HOST=$HOSTNAME^M"
# 	screen -S $SCREEN_NAME -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
# 	screen -S $SCREEN_NAME -p 0 -X stuff "source ~/ros/src/metahast/scripts/trial_configs/${HOST}_config.sh^M"
# 	screen -S $SCREEN_NAME -p 0 -X stuff "bash ~/ros/src/metahast/scripts/specific_trials.sh; exit^M"
# 	sleep 3
# 	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep $SCREEN_NAME | cut -d'(' -f1 ) ";
# 	printf " as $SCREEN_TAG\n"
#
#
#
# #
# # create selective kill
# function KILL_SCREENS
# { # KILL_SCREENS "${SCREEN_LIST[@]}"
# 	local f_SCREEN_LIST=("$@")
# 	for f_TAG in "${f_SCREEN_LIST[@]}";
# 	do
# 		unset f_SCREEN_TAG; f_SCREEN_TAG="$(screen -list | grep $f_TAG | cut -d'(' -f1 ) ";
# 		echo "    killing: $f_SCREEN_TAG"
# 		screen -X -S $f_SCREEN_TAG kill
# 	done
#
# }
#
# # function myfunc()
# # {
# #     local  myresult='some value'
# #     echo "$myresult"
# # }
# #
# # result=$(myfunc)   # or result=`myfunc`
# # echo $result
#
# function CHECK_SCREENS()
# { # KILL_SCREENS "${SCREEN_LIST[@]}"
# 	f_SCREEN_LIST=("$@")
# 	f_RESULT=true
# 	f_dummy=true
#
# 	for f_TAG in "${f_SCREEN_LIST[@]}";
# 	do
# 		unset f_SCREEN_TAG; f_SCREEN_TAG="$(screen -list | grep $f_TAG | cut -d'(' -f1 ) ";
# 		if screen -list | grep $f_TAG > /dev/null
# 		then
# 			# echo "    $f_SCREEN_TAG found"
# 			# do nothing
# 			f_dummy=true
# 		else
# 			# echo "    no $f_TAG screen found"
# 			f_RESULT=false
# 		fi
# 	done
#
# 	echo $f_RESULT
# }
#
#
#
#
# SCREEN_LIST=("gazebo_init" "uav_rviz" "ugv1_base" "ugv2_base" "trial" "failswitch")
#
#
#
# # RESULT=true
# # for TAG in "${SCREEN_LIST[@]}";
# # do
# # 	unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep $TAG | cut -d'(' -f1 ) ";
# # 	if screen -list | grep $TAG > /dev/null
# # 	then
# # 		echo "    $SCREEN_TAG found"
# # 	else
# # 		echo "    no $TAG screen found"
# # 		RESULT=false
# # 	fi
# # done
# # echo RESULT
#
#
#
#
# result=$(CHECK_SCREENS SCREEN_LIST)   # or result=`myfunc`
# echo $result
# # KILL_SCREENS "${SCREEN_LIST[@]}"
#
#
#
# # for CODE in "${EXP_CODES[@]}";
# # {
# # 	if [[ " ${EXP_CODES[@]} " =~ " ${CODE} " ]]; then
# # 			# whatever you want to do when EXP_CODES contains CODE
# # 			unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
# # 			echo "    ${CODE}_TRIALS: ${!CODE_TRIALS}" | tee -a $TRIAL_CONFIG_LOG
# # 	fi
# # }
#
# #!/bin/bash
# # local:   cd ~/ros/src/metahast/scripts/remotes && . atlas_trials.sh atlas && cd ~/ros/src/metahast
# # remote:  cd ~/ros/src/metahast/scripts && . remote_trials.sh atlas && cd ~/ros/src/metahast
# # clear screen
# # clear
# #
# # # clear logs
# # rm -rf /home/benjamin/.ros/log
# #
# # function screen_wait
# # {
# # 	while screen -list | grep $1 > /dev/null
# # 	do
# # 		sleep 1
# # 	done
# # }
# #
# # echo " "
# # export HOST=$HOSTNAME
# # rm ~/ros/${HOST}_trial_config.txt;
# # source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros | tee ~/ros/${HOST}_trial_config.txt
# #
# # unset DATE MAP SHOW_RVIS
# # unset TRIALS TRIAL START LENGTH STOP
# # unset CODE EXP_CODES
# #
# # source ~/ros/src/metahast/scripts/remotes/${HOST}.conf
# #
# # echo "  DATE: $DATE" | tee -a ~/ros/${HOST}_trial_config.txt
# # echo "  MAP: $MAP" | tee -a ~/ros/${HOST}_trial_config.txt
# # echo "  SHOW_RVIS: $SHOW_RVIS" | tee -a ~/ros/${HOST}_trial_config.txt
# # echo "  SEND_TEXT: $SEND_TEXT" | tee -a ~/ros/${HOST}_trial_config.txt
# # echo "  SEND_EMAILS: $SEND_EMAILS" | tee -a ~/ros/${HOST}_trial_config.txt
#
#
#
# # counter=0
# # while :
# # do
# # 	counter=$(( $counter + 1 ))
# # 	# echo $counter
# # 	echo -ne "  waiting for finish ... $counter ...\r"
# # 	# read -p "Enter two numnbers ( - 1 to quit ) : " a b
# # 	if [ $counter -gt 5 ]
# # 	then
# # 		break
# # 	fi
# # 	sleep 1
# # done
#
#
# # # clear screen
# # clear
# # # clear logs
# # rm -rf /home/benjamin/.ros/log
# #
# # export HOST=$HOSTNAME
# # rm ~/ros/${HOST}_trial_config.txt;
# # source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros | tee ~/ros/${HOST}_trial_config.txt
# #
# # function screen_wait
# # {
# # 	while screen -list | grep $1 > /dev/null
# # 	do
# # 		sleep 1
# # 	done
# # }
# #
# # 	if $SEND_EMAILS ; then
# # 		EMAIL_SUBJECT="${HOST} test completed."
# # 		mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < ~/ros/${HOST}_trial_config.txt
# # 	fi
#
# 	# EXP_CODES=(G)
# 	# EXP_CODES=(C D)
#
#
#
# # nohup bash ~/ros/src/metahast/scripts/test_script2.sh &> ~/ros/src/metahast/scripts/ping.out &
#
# # MAP=test_map.launch; DATE=20200701; CODE=A; TRIAL=1; EMAIL_BODY=$'test email:\n  date:"$DATE"\n  trial:$i\n  code:$CODE\n  map:$MAP'; echo "$EMAIL_BODY"; echo "$EMAIL_BODY" | mail -s "test email" gutter.puddles@gmail.com
#
#
# # NL=$'\n'; EMAIL_BODY="test email:${NL}  date:$DATE${NL}  trial:$i${NL}  code:$CODE${NL}  map:$MAP"; echo "$EMAIL_BODY"; echo "$EMAIL_BODY" | mail -s "test email" gutter.puddles@gmail.com
#
#
#
# # USERNAME=benjamin
# # HOSTS="host1 host2 host3"
# # SCRIPT="pwd; ls"
# # for HOSTNAME in ${HOSTS} ; do
# #     ssh -l ${USERNAME} ${HOSTNAME} "${SCRIPT}"
# # done
#
# # ssh -l benjamin titan.local "~/ros/src/metahast/scripts/test_scripts.sh titan '1 2 3 4 5'"
# # ssh benjamin@titan.local 'bash -s' < /path/to/test.sh
#
# # ssh benjamin@titan.local "bash -s" < ~/ros/src/metahast/scripts/test_scripts.sh "titan" "1 2 3 4 5"
#
# # REMOTE_HOST=$1
# # INPUT_LIST=$2
#
# # touch ~/$REMOTE_HOST.txt
#
#
# # CODE="F"
# # echo "code: $CODE" ~/$REMOTE_HOST.txt
#
# # unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
# # echo "CODE_TRIALS: $CODE_TRIALS" ~/$REMOTE_HOST.txt
#
# # echo "input string (trials): $INPUT_LIST" ~/$REMOTE_HOST.txt
# # F_TRIALS=$1
# # echo "F_TRIALS: ${F_TRIALS}" ~/$REMOTE_HOST.txt
# # IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"
# # printf "CODE_TRIALS: " ~/$REMOTE_HOST.txt
#
# # 	for i in ${TRIALS[@]};
# # 	{
# # 		printf " $i \n" ~/$REMOTE_HOST.txt
# # 	}; printf "\n" ~/$REMOTE_HOST.txt
#
#
# # scp ~/$REMOTE_HOST.txt benjamin@atlas.local:~/
# # rm $REMOTE_HOST.txt
#
# # exit
#
#
# # case $CODE in
# # A|B) echo " 24th January international Day of Education." ;;
# # C) echo "8th March International women’s day." ;;
# # D) echo "7th April The World Health Day" ;;
# # F) echo "The 15 May International Day of Families" ;;
# # G) echo "20th June World Refugee Day" ;;
# # H) echo "11th July World Population Day";;
# # *) echo "No matching information found";;
# # esac
#
#
#
#
# # unset DATE MAP SHOW_RVIS
# # unset TRIALS TRIAL START LENGTH STOP
# # unset CODE EXP_CODES
#
#
# # DATE="20200713"
# # mkdir -p "/home/$USER/ros/data/$DATE/figs/bulk"
#
# # 	IDX=5; # map number
# # 	TRIALS=(1 2 3 4 5 6 7 8 9 10); echo "Trials: ${TRIALS[@]}"
# # 	START=${TRIALS[0]};	echo "START: $(expr $IDX \* 100 + $START)";
# # 	LENGTH=$(expr ${#TRIALS[@]} - 1);
# # 	STOP=${TRIALS[@]:$LENGTH:1};	echo "STOP: $(expr $IDX \* 100 + $STOP)";
# # 	MAP=tags_0$IDX.launch;
# # 	SHOW_RVIS="false";
#
# # 	A_TRIALS="1 3 5"; # echo "A_TRIALS: ${A_TRIALS}"
# # 	B_TRIALS="3 5 7"; # echo "B_TRIALS: ${B_TRIALS}"
# # 	C_TRIALS="5 7 9"; # echo "C_TRIALS: ${C_TRIALS}"
# # 	D_TRIALS="2 4 6"; # echo "D_TRIALS: ${D_TRIALS}"
# # 	E_TRIALS="4 6 8"; # echo "E_TRIALS: ${E_TRIALS}"
# # 	F_TRIALS="6 8 10"; # echo "F_TRIALS: ${F_TRIALS}"
#
# # 	src_map=/media/benjamin/archive/maps/$MAP;
# # 	cp $src_map /home/benjamin/ros/src/metahast/robot_descriptions/tags/launch/tags_rand.launch # Copy map to use for trials
# # 	EXP_CODES=(A B C D E F)
#
#
#
# # 	for CODE in "${EXP_CODES[@]}";
# # 	{
# # 		# CODE=${EXP_CODES[0]};
# # 		echo "CODE: ${CODE}"
# # 		unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
# # 		echo "$CODE_TRIALS: ${!CODE_TRIALS}"
# # 		IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"
#
# # 		for TRIAL in ${TRIALS[@]};
# # 		{
# # 			# echo $TRIAL
# # 				i=$(expr $IDX \* 100 + $TRIAL); # echo $i
# # 				# echo "single_trial.sh date:$DATE trial:$i code:$CODE map:$MAP" | mail -s "single_trial.sh $DATE $i $CODE" gutter.puddles@gmail.com
# # 				NL=$'\n'; EMAIL_BODY="  map: $MAP${NL}  date: $DATE${NL}  trial: $i${NL}  code: $CODE${NL}"; echo "$EMAIL_BODY" | mail -s "single_trial.sh $i$CODE" gutter.puddles@gmail.com
# # 				# cd ~/ros/src/metahast/scripts && 	. single_trial.sh $DATE $i $CODE
# # 				# cd ~/ros/src/metahast/scripts && 	. single_trial.sh $DATE $i $CODE $SHOW_RVIS
# # 		}
# # 	}
#
#
#
#
#
# 	# H="abc"
# 	# PARAM="H"
# 	# echo ${!PARAM} #gives abc
#
#
# 	# IFS=' ' read -r -a TRIALS <<< "$A_TRIALS"
# 	#
# 	# for TRIAL in "${TRIALS[@]}"
# 	# {
# 	# 	echo "$TRIAL"
# 	# }
# 	#
# 	# #
# 	# # echo "A_TRIALS: ${A_TRIALS[@]}"
# 	# # B_TRIALS=(2 4 6 8 10); echo "B_TRIALS: ${B_TRIALS[@]}"
# 	# #
# 	# echo "EXP_CODES: ${EXP_CODES[@]}"
# 	# unset CODE
# 	# for CODE in "${EXP_CODES[@]}";
# 	# {
# 	# 	unset TRIALS
# 	# 	# TRIALS="${CODE}_TRIALS"; echo "TRIALS: $TRIALS"
# 	# 	TRIALS=${${CODE}_TRIALS[@]}
# 	# 	echo "TRIALS: $TRIALS ....  ${TRIALS[@]}"
# 	# }
#
#
# # DATE=20200701;
# # CODE=A;
#
# # IDX=3;
# # MAP=tags_0$IDX.launch;
# # TRIALS=(`seq 11 20 | tr " " "\n"`); echo "Trials: ${TRIALS[@]}"
# # # TRIALS=(1 2 4 8 16 32 64); echo "Trials: ${TRIALS[*]}"
# # START=${TRIALS[0]}; echo "START: $START";
# # # START=${TRIALS[@]:0:1}; echo "Start: $START";
# # LENGTH=$(expr ${#TRIALS[@]} - 1);
# # STOP=${TRIALS[@]:$LENGTH:1}; echo "Stop: $STOP";
# # # for TRIAL in ${TRIALS[*]};
# # # {
# # # 	i=$(expr $IDX \* 100 + $TRIAL); echo $i
# # # 	NL=$'\n'; EMAIL_BODY="test email:${NL}  date:$DATE${NL}  trial:$i${NL}  code:$CODE${NL}  map:$MAP"; echo "$EMAIL_BODY"
# # # }
# #
#
#
#
#
#
# # NL=$'\n'; EMAIL_BODY="test email:${NL}  date:$DATE${NL}  trial:$i${NL}  code:$CODE${NL}  map:$MAP"; echo "$EMAIL_BODY"
#
# #
# #
# # # process completed data for plots
# # 	CELL_TRIALS="{"
# # 	for TRIAL in ${TRIALS[@]};
# # 	{
# # 		# echo "TRIAL: $TRIAL"
# # 		# echo "START: ${TRIALS[0]}"
# # 		i=$(expr $IDX \* 100 + $TRIAL); echo $i
# # 		if (( $TRIAL == ${TRIALS[0]} )); then
# # 			CELL_TRIALS="${CELL_TRIALS}'$(printf "%03d" $i)'"
# # 		else
# # 			CELL_TRIALS="${CELL_TRIALS},'$(printf "%03d" $i)'"
# # 		fi
# # 	}
# # 	CELL_TRIALS="${CELL_TRIALS}}"
# # 	echo "CELL_TRIALS: $CELL_TRIALS"
# # #
# # #
# # # echo {11..20}
# # #
# # # seq 7 15
# # #
# # # TRIALS=(`seq 11 20 | tr " " "\n"`)
