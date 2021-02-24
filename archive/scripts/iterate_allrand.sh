DATE=$1
mkdir -p "/home/$USER/ros/data/$DATE/figs"

i=$2
STOP=$3

printf "Start = $2, Stop = $3\n"
while [ $i -le $STOP ]
do
	TRIAL="$(printf "%03d" $i)"
	mkdir -p "/home/$USER/ros/data/$DATE/$TRIAL/prealloc"
	mkdir -p "/home/$USER/ros/data/$DATE/$TRIAL/figs/eps"
	mkdir -p "/home/$USER/ros/data/$DATE/$TRIAL/figs/png"

	killall screen
	matlab -nodisplay -nosplash -nojvm -r "make_rand_map;exit;"
	printf "\n\n\n\n\nbash gazebo_iterant.sh $i of $3\n\n"
	bash gazebo_iterant.sh $DATE $TRIAL
	# bash gazebo_1ugv.sh $DATE $TRIAL

	if (( $i % 10 == 0 ))
	then
		echo "gazebo_iterant ($i/$STOP) complete" | mail -s "Atlas::iterate($i/$STOP)" gutter.puddles@gmail.com
	fi

	i=$((i+=1))
	sleep 5
	killall screen
done

############## uncomment to batch process data for matlab
i=$2
while [ $i -le $3 ]
do
	TRIAL="$(printf "%03d" $i)"
	matlab -nodisplay -nosplash -nojvm -r "process_iteration('$DATE', '$TRIAL');exit;"
	if (( $i % 10 == 0 ))
	then
		echo "process_iteration ($i/$STOP) complete" | mail -s "Atlas::process($i/$STOP)" gutter.puddles@gmail.com
	fi
	i=$((i+=1))
done

# matlab -nodisplay -nosplash -nodesktop -r "batchslam_1604('$DATE', $i, $STOP);exit;"
# echo "batchslam_1604('$DATE', $i, $STOP) complete" | mail -s "Atlas::batch ($STOP)" gutter.puddles@gmail.com
