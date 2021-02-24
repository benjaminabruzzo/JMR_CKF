# DATE=20200524
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 100 110
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 111 120
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 121 130
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 131 135
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 136 140
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 141 145
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 146 150

# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 150 151
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 152 153
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 154 156

# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 151 160
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 161 170
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 171 180
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 181 190
# cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh $DATE 191 200


DATE=20200610
for TRIAL in {16..25}
do
	cd ~/ros/src/metahast/scripts && . iterate_trials.sh $DATE $TRIAL
done
