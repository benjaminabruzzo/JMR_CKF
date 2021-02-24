# cd ~/ros/src/metahast && bash ./scripts/move_bad.sh
# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
unset DATE EXP_CODES

unset A_TRIALS B_TRIALS G_TRIALS
unset H_TRIALS I_TRIALS J_TRIALS

	IDX=1; 										# map number
	DATE="20200814";					# date folder

	EXP_CODES=(A B G H I J);	# Trial strategies

	LIST="16 17 18 19 20";				# Trial numbers
	A_TRIALS=$LIST;
	B_TRIALS=$LIST;
	G_TRIALS=$LIST;
	H_TRIALS=$LIST;
	I_TRIALS=$LIST;
	J_TRIALS=$LIST;

	echo " ";
	for CODE in "${EXP_CODES[@]}";
	{
		# CODE_start_time="$(date -u +%s.%N)"
		unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
		IFS=' ' read -r -a TRIALS <<< "${!CODE_TRIALS}"

		for i in ${TRIALS[@]};
		{
			# run trial
			TRIAL=$(expr $IDX \* 100 + $i); # echo $i

			# postprocess with matlab
			start_time="$(date -u +%s.%N)"
			LOGPATH="/home/benjamin/ros/data/$DATE/$CODE/$TRIAL/logs";
			unset SCREEN_NAME; SCREEN_NAME="matlab_screen"
				printf "  launching $SCREEN_NAME  ... "
				screen -dmS $SCREEN_NAME																										#Start detached screen session
				screen -S $SCREEN_NAME -X logfile $LOGPATH/$SCREEN_NAME$TRIAL$EXP_CODE.txt	#Use logfile command to set logging file
				screen -S $SCREEN_NAME -X log																								#Use log command to start logging
				screen -S $SCREEN_NAME -p 0 -X stuff "export HOST=$HOSTNAME^M"
				screen -S $SCREEN_NAME -p 0 -X stuff "source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros^M"
				screen -S $SCREEN_NAME -p 0 -X stuff "matlab -nosplash -nojvm -r \"process_iteration('$DATE', '$TRIAL', '$CODE');exit\"; exit^M" # needs extra ; exit to close screen when complete
				sleep 5
				unset SCREEN_TAG; SCREEN_TAG="$(screen -list | grep $SCREEN_NAME | cut -d'(' -f1 ) ";
				printf " as $SCREEN_TAG\n"
				screen_wait $SCREEN_NAME; printf " done. \n"
			end_time="$(date -u +%s.%N)"
			elapsed="$(bc <<<"$end_time-$start_time")"

			EMAIL_SUBJECT="$HOST post-processed trial: $TRIAL$CODE"
			EMAIL_BODY="$HOST${NL}  post-processed .mat files map: $MAP${NL}  date: $DATE${NL}  trial: $TRIAL${NL}  code: $CODE${NL}  duration:$elapsed seconds";
			echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com
		}
	}

	echo "Synchronizing to benjamin@atlas.local:/mnt/evo2/$DATE/$CODE/"
	rsync -aI --exclude='*.png' -e ssh ~/ros/data/$DATE/$CODE/ benjamin@atlas.local:/mnt/evo2/$DATE/$CODE/
}


# echo "rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@atlas.local:/mnt/evo2/$DATE/"
rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@atlas.local:/mnt/evo2/$DATE/

if $SEND_EMAILS ; then
	EMAIL_SUBJECT="$HOST simualtions complete."
	mail -s "$EMAIL_SUBJECT" gutter.puddles+${HOST}@gmail.com < $TRIAL_CONFIG_LOG
fi

if $SEND_TEXT ; then
	EMAIL_SUBJECT="$HOST simualtions complete."
	mail -s "$EMAIL_SUBJECT" 9735346583@txt.att.net < $TRIAL_CONFIG_LOG
fi

echo "DONE."
