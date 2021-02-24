#!/bin/bash
# cd ~/ros/src/metahast && bash ./scripts/specific_trials.sh
# clear screen
clear

# clear logs
rm -rf /home/benjamin/.ros/log

NL=$'\n';

function screen_wait
{
	while screen -list | grep $1 > /dev/null
	do
		sleep 1
	done
}

unset CODE TRIALS TRIAL START LENGTH STOP


echo " "
export HOST=$HOSTNAME

# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
source ~/ros/src/metahast/scripts/trial_configs/${HOST}_config.sh
	TRIAL_CONFIG_LOG="/home/benjamin/ros/data/${DATE}/${HOST}_trial_config.log"
	touch $TRIAL_CONFIG_LOG
	source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros | tee $TRIAL_CONFIG_LOG
	date | tee $TRIAL_CONFIG_LOG

echo "  DATE: ${DATE}" | tee -a $TRIAL_CONFIG_LOG
echo "  MAP:  ${MAP}" | tee -a $TRIAL_CONFIG_LOG
echo "  MNT:  $MNT" | tee -a $TRIAL_CONFIG_LOG
echo "  SHOW_RVIS:   ${SHOW_RVIS}" | tee -a $TRIAL_CONFIG_LOG
echo "  SEND_TEXT:   ${SEND_TEXT}" | tee -a $TRIAL_CONFIG_LOG
echo "  SEND_EMAILS: ${SEND_EMAILS}" | tee -a $TRIAL_CONFIG_LOG
# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #

	echo "  TRIAL_CODES: ${EXP_CODES[@]}" | tee -a $TRIAL_CONFIG_LOG
	for CODE in "${EXP_CODES[@]}";
	{
		if [[ " ${EXP_CODES[@]} " =~ " ${CODE} " ]]; then
		    # whatever you want to do when EXP_CODES contains CODE
				unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
				echo "    ${CODE}_TRIALS: ${!CODE_TRIALS}" | tee -a $TRIAL_CONFIG_LOG
		fi
	}

	if ${SEND_EMAILS} ; then
		EMAIL_SUBJECT="${HOST} simulations started"
		mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG
	fi

# Run trials
echo " ";
for CODE in "${EXP_CODES[@]}";
{
	# CODE_START_TIME="$(date -u +%s.%N)"
	unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
	IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"

	for i in ${TRIALS[@]};
	{
		# run trial
		TRIAL=$(expr $IDX \* 100 + $i); # echo $i

		unset START_TIME END_TIME ELAPSED
		START_TIME="$(date -u +%s.%N)"
			bash ~/ros/src/metahast/scripts/bin/gazebo_iterant.sh ${DATE} ${TRIAL} ${CODE} ${SHOW_RVIS} ${MAP};
		END_TIME="$(date -u +%s.%N)"
		ELAPSED="$(bc <<<"$END_TIME-$START_TIME")"

		# EMAIL_SUBJECT="$HOST finished trial: ${TRIAL}${CODE}"
		# EMAIL_BODY="$HOST${NL}  map: ${MAP}${NL}  date: ${DATE}${NL}  trial: ${TRIAL}${NL}  code: ${CODE}${NL}  duration:$ELAPSED seconds";
		# echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com
		EMAIL_SUBJECT="SIM: $HOST, ${MAP}, ${TRIAL}, ${CODE}, $ELAPSED seconds";
		mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG

		# postprocess with matlab
		if $PROCESS_MATLAB ; then
			unset START_TIME END_TIME ELAPSED
			START_TIME="$(date -u +%s.%N)"
				printf "${HOST} :: matlab process_iteration('${DATE}', '${TRIAL}', '${CODE}')  ... "
				SCREEN_NAME="matlab_screen"
				MATLAB_LOG="/home/benjamin/ros/data/${DATE}/${CODE}/${TRIAL}/logs/${SCREEN_NAME}_${TRIAL}${CODE}.log";
				screen -dmS ${SCREEN_NAME}
				screen -S ${SCREEN_NAME} -X logfile $MATLAB_LOG #Use logfile command to set logging file
				screen -S ${SCREEN_NAME} -X log #Use log command to start logging
				screen -S ${SCREEN_NAME} -p 0 -X stuff "matlab -nodisplay -nosplash -nojvm -r \"process_iteration('${DATE}', '${TRIAL}', '${CODE}');exit\"; exit^M" # needs extra ; exit to close screen when complete
				sleep 5
				unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep ${SCREEN_NAME} | cut -d'(' -f1 ) ";
				printf " on screen $SCREEN_TAG\n"
				counter=0
				while ps -e | grep MATLAB > /dev/null
				do
					if [ $counter -gt 500 ]
					then
						killall screen
						break
					fi
					counter=$(( $counter + 1 ))
					echo -ne "  waiting for finish ... $counter ... \r"
					sleep 1
				done
				printf "\n\n  done. \n\n"
				killall screen
				END_TIME="$(date -u +%s.%N)"
				ELAPSED="$(bc <<<"$END_TIME-$START_TIME")"

			cat $MATLAB_LOG | grep "net_displacement"
		fi # postprocess with matlab

		# EMAIL_SUBJECT="$HOST processed trial: ${TRIAL}${CODE}"
		# EMAIL_BODY="$HOST${NL}  process_iteration('${DATE}', '${TRIAL}', '${CODE}')${NL}  duration:$ELAPSED seconds";
		# echo "$HOST${NL}  process_iteration('${DATE}', '${TRIAL}', '${CODE}')${NL}  duration:$ELAPSED seconds" | tee ~/ros/data/${DATE}/${CODE}/rsync.log

		EMAIL_SUBJECT="PROC: $HOST, ${MAP}, ${TRIAL}, ${CODE}, $ELAPSED seconds";
		mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG

		echo "${HOST} :: Synchronizing to benjamin@atlas.local:/mnt/${MNT}/${DATE}/${CODE}/${TRIAL}/"
		rsync -av --exclude='*.png' -e ssh ~/ros/data/${DATE}/${CODE}/${TRIAL}/ benjamin@atlas.local:/mnt/${MNT}/${DATE}/${CODE}/${TRIAL}/ | tee ~/ros/data/${DATE}/${CODE}/rsync.log

		EMAIL_SUBJECT="RSYNC: $HOST, ${MAP}, ${TRIAL}, ${CODE}";
		# mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG
		mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < ~/ros/data/${DATE}/${CODE}/rsync.log
	}

}

EMAIL_SUBJECT="${HOST} simulations completed"
mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG


echo "  done." | tee -a $TRIAL_CONFIG_LOG
