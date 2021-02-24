# cd ~/ros/src/metahast && bash ./scripts/local_trials.sh

echo " "
export HOST=$HOSTNAME
source ~/ros/src/metahast/hast/config/bashros/${HOST}.bashros
# DISPLAY=${HOST}:0


# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
source ~/ros/src/metahast/scripts/trial_configs/${HOST}_config.sh #  do this just to grab the date
rm -rf ~/ros/data/$DATE && mkdir -p ~/ros/data/$DATE

# ~~~~~~~~~~~~~~~~~~  run specific trials ~~~~~~~~~~~~~~~~~~~~~~~~ #
	bash ~/ros/src/metahast/scripts/specific_trials.sh | tee ~/ros/data/${DATE}/${HOST}_tee.log
	scp ~/ros/data/${DATE}/${HOST}_tee.log benjamin@atlas.local:/mnt/evo2/$DATE/
	# rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@atlas.local:/mnt/evo2/$DATE/

	TRIAL_CONFIG_LOG="/home/benjamin/ros/data/${DATE}/${HOST}_trial_config.log"
	if $SEND_EMAILS ; then
		EMAIL_SUBJECT="$HOST simualtions complete."
		mail -s "$EMAIL_SUBJECT" gutter.puddles+complete@gmail.com < $TRIAL_CONFIG_LOG
	fi

	if $SEND_TEXT ; then
		EMAIL_SUBJECT="$HOST simualtions complete."
		mail -s "$EMAIL_SUBJECT" 9735346583@txt.att.net < $TRIAL_CONFIG_LOG
	fi

	# sudo apt-get install xvfb
	# Xvfb :1 -screen 0 1600x1200x16  &
	# export DISPLAY=:1.0
	# roslaunch etc etc.launch



# Cannot establish any listening sockets - Make sure an X server isn't already running(EE)
# stop or sudo lightdm stop
# Enter runlevel 3 by typing sudo init 3
