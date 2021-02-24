EXP_CODES=( A B C D E F )

DATE=$1
START=$2
if [ "$#" -eq 2 ]; then
	STOP=$2
fi
if [ "$#" -eq 3 ]; then
	STOP=$3
fi

echo "Starting iterate_trials $START $STOP " | mail -s "Atlas::iterate_trials($START/$STOP)" gutter.puddles@gmail.com
mkdir -p "/home/$USER/ros/data/$DATE/figs/bulk"

killall screen; matlab -nodisplay -nosplash -nojvm -r "make_rand_map;exit;"

i=$START
printf "Start = $i, Stop = $STOP\n"
while [ $i -le $STOP ]; do
	printf "\nbash gazebo_iterant.sh $i of $STOP\n"
	TRIAL="$(printf "%03d" $i)"

	for CODE in "${EXP_CODES[@]}"; do
		echo "bash gazebo_iterant_$CODE.sh $DATE $TRIAL" | mail -s "bash gazebo_iterant_$CODE.sh $DATE $TRIAL" gutter.puddles@gmail.com
		bash gazebo_iterant_$CODE.sh $DATE $TRIAL; sleep 30
	done

	# if (( $i % 5 == 0 )); then
	# 	echo "gazebo_iterant ($i of $STOP) complete" | mail -s "Atlas::iterate($i of $STOP)" gutter.puddles@gmail.com
	# fi

	i=$((i+=1))
	sleep 5
	killall screen
done

############# uncomment to batch process data for matlab
i=$START
while [ $i -le $STOP ]
do
	TRIAL="$(printf "%03d" $i)"

	for CODE in "${EXP_CODES[@]}"; do
		echo "start process_iteration('$DATE', '$TRIAL', '$CODE')" | mail -s "start process_iteration('$DATE', '$TRIAL', '$CODE')" gutter.puddles@gmail.com
		matlab -nodisplay -nosplash -nojvm -r "process_iteration('$DATE', '$TRIAL', '$CODE');exit;"
	done

	# if (( $i % 5 == 0 )); then
	# 	echo "process_iteration ($i/$STOP) complete" | mail -s "Atlas::process($i/$STOP)" gutter.puddles@gmail.com
	# fi
	i=$((i+=1))
done
