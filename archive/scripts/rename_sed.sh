echo "Welcome To The Geek Stuff" | sed 's/\(\b[A-Z]\)/\(\1\)/g'
SCRIPTHOME=$PWD
echo "SCRIPTHOME: $SCRIPTHOME"

DATA_HOME=/media/benjamin/data/20200626

EXP_CODES=(A B C D F)

START=111; DST_START=$(expr $START + 100);
STOP=120; DST_STOP=$(expr $STOP + 100);


for CODE in "${EXP_CODES[@]}"; do
	cd $DATA_HOME/$CODE
	# pwd

	for TRIAL in $(eval echo {$START..$STOP}); do
		DST_TRIAL=$(expr $TRIAL + 100);

		# echo $DATA_HOME/$CODE/$DST_TRIAL;
		mkdir -p $DATA_HOME/$CODE/$DST_TRIAL/figs/png
		mkdir -p $DATA_HOME/$CODE/$DST_TRIAL/figs/eps
		mkdir -p $DATA_HOME/$CODE/$DST_TRIAL/config
		mkdir -p $DATA_HOME/$CODE/$DST_TRIAL/prealloc

		# echo $TRIAL $DST_TRIAL
		# x=( $(find -maxdepth 5 -type d) );
		X=( $(find ./$TRIAL -name "*.m") );
		Y=( $(find ./$TRIAL -name "*.mat") );

		for ITEM in "${X[@]}";
		{
			# echo "$ITEM"
			# echo "$ITEM" | sed "s:$TRIAL:$DST_TRIAL:g"
			# cp "$ITEM" "echo $ITEM | sed 's:$TRIAL:$DST_TRIAL:g'"
			NEW_ITEM=$(echo $ITEM | sed "s:$TRIAL:$DST_TRIAL:g")
			echo "cp $ITEM $NEW_ITEM"
			cp $ITEM $NEW_ITEM
		}
		for ITEM in "${Y[@]}";
		{
			# echo "$ITEM"
			# echo "$ITEM" | sed "s:$TRIAL:$DST_TRIAL:g"
			# cp "$ITEM" "echo $ITEM | sed 's:$TRIAL:$DST_TRIAL:g'"
			NEW_ITEM=$(echo $ITEM | sed "s:$TRIAL:$DST_TRIAL:g")
			echo "cp $ITEM $NEW_ITEM"
			cp $ITEM $NEW_ITEM
		}

	done

done

# echo daydayday | sed 's/day/night/g'

# for i in *; do
#   mv "$i" "`echo $i | sed "s/\(.*\) - \(.*\) - \(.*\) - \(.*\).ogg/\1 - \4 - \3 - \2.ogg/"`";
# done
#
# IDX=3;

#
#
# START=111;
# STOP=120;
#
# for i in $(eval echo {$START..$STOP}); do
# 	# echo "single_trial.sh $DATE $i $CODE" | mail -s "single_trial.sh $DATE $i $CODE" gutter.puddles@gmail.com
# 	# cd ~/ros/src/metahast/scripts && 	. single_trial.sh $DATE $i $CODE
# 	# cd ~/ros/src/metahast/scripts && 	. single_trial.sh $DATE $i $CODE $SHOW_RVIS
#
# done
#
#
#
#
# pwd

cd $SCRIPTHOME
pwd






# /media/benjamin/data/20200626/114/uavCKF_114.m
