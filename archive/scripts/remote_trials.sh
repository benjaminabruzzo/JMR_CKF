#!/bin/bash
# remote call
# cd ~/ros/src/metahast/ && bash ./scripts/remote_trials.sh argo
# cd ~/ros/src/metahast/ && bash ./scripts/remote_trials.sh atlas
# cd ~/ros/src/metahast/ && bash ./scripts/remote_trials.sh hermes
# cd ~/ros/src/metahast/ && bash ./scripts/remote_trials.sh pompei
# cd ~/ros/src/metahast/ && bash ./scripts/remote_trials.sh titan
unset REMOTE; REMOTE=$1
ssh -X benjamin@$REMOTE.local 'bash -s' <<"ENDSSH"
echo " "
export HOST=$HOSTNAME
	source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros
	cd ~/ros/src/metahast && . clear_stash.sh
# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
	echo "loading ${HOST}_config.sh"
	source ~/ros/src/metahast/scripts/trial_configs/${HOST}_config.sh #  do this just to grab the date
	rm -rf ~/ros/data/$DATE && mkdir -p ~/ros/data/$DATE
	echo "  DATE: $DATE"
	echo "  MAP: $MAP"
	echo "  SHOW_RVIS: $SHOW_RVIS"
	echo "  SEND_TEXT: $SEND_TEXT"
	echo "  SEND_EMAILS: $SEND_EMAILS"
	echo "  TRIAL_CODES: ${EXP_CODES[@]}"
	for CODE in "${EXP_CODES[@]}";
	{
		if [[ " ${EXP_CODES[@]} " =~ " ${CODE} " ]]; then
		    # whatever you want to do when EXP_CODES contains CODE
				unset CODE_TRIALS; CODE_TRIALS=${CODE}_TRIALS
				echo "    ${CODE}_TRIALS: ${!CODE_TRIALS}" | tee -a $TRIAL_CONFIG_LOG
		fi
	}

	# sleep 1
	# ~~~~~~~~~~~~~~~~~~  run specific trials ~~~~~~~~~~~~~~~~~~~~~~~~ #

	echo starting trials

	nohup bash ~/ros/src/metahast/scripts/specific_trials.sh &> ~/ros/data/${DATE}/${HOST}_nohup.log &
	echo $! > ~/ros/data/${DATE}/${HOST}_trials_pid.log
	# sleep 1
ENDSSH
