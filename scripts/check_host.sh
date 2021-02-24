# cd ~/ros/src/metahast && bash ./scripts/move_bad.sh
# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
unset DATE EXP_CODES

unset A_TRIALS B_TRIALS G_TRIALS
unset H_TRIALS I_TRIALS J_TRIALS

	DATE="20200814";					# date folder

	EXP_CODES=(A B G H I J);	# Trial strategies
	HOSTS=("atlas" "argo" "hermes" "pompei" "titan")

	LIST1=$(seq 101 1 120);
	LIST2=$(seq 201 1 220);
	LIST3=$(seq 301 1 320);
	LIST4=$(seq 401 1 420);
	LIST5=$(seq 501 1 520);
	LIST6=$(seq 601 1 620);

	rm /mnt/evo2/$DATE/hosts_list.txt && touch /mnt/evo2/$DATE/hosts_list.txt

	echo " ";
	for TRIAL in $LIST1 $LIST2 $LIST3 $LIST4 $LIST5 $LIST6;
	{
		for CODE in "${EXP_CODES[@]}";
		{
			for aHost in ${HOSTS[@]};
			{
				if cat /mnt/evo2/$DATE/$CODE/$TRIAL/logs/gazebo_init${TRIAL}${CODE}.txt | grep ${aHost} > /dev/null
				then
					echo "  ${TRIAL}/${CODE}/${aHost}" | tee -a /mnt/evo2/$DATE/hosts_list.txt
				fi
			}
		}
	}
