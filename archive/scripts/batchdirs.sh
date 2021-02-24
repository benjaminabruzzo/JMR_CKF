# DATE="20151001"
# call : $ . makedirs.sh 20160930 1 5

DATE=$1
i=$2
STOP=$3

echo "i = $2"
echo "Stop = $3"
FOLDER="/home/benjamin/ros/data/$DATE"

while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"
	echo $FOLDER/N/$RUN
	mkdir -p "$FOLDER/A/$RUN/config"; mkdir -p "$FOLDER/A/$RUN/prealloc"
	mkdir -p "$FOLDER/B/$RUN/config"; mkdir -p "$FOLDER/B/$RUN/prealloc"
	mkdir -p "$FOLDER/C/$RUN/config"; mkdir -p "$FOLDER/C/$RUN/prealloc"
	mkdir -p "$FOLDER/D/$RUN/config"; mkdir -p "$FOLDER/D/$RUN/prealloc"
	mkdir -p "$FOLDER/E/$RUN/config"; mkdir -p "$FOLDER/E/$RUN/prealloc"
	mkdir -p "$FOLDER/F/$RUN/config"; mkdir -p "$FOLDER/F/$RUN/prealloc"

	i=$((i+=1))
done

	# GOOD="$NEWDATE/$RUN/good"
	# GLEFT="$NEWDATE/$RUN/good/LEFT"
	# GRIGHT="$NEWDATE/$RUN/good/RIGHT"
	# GGREEN="$NEWDATE/$RUN/good/GREEN"
	# GBLUE="$NEWDATE/$RUN/good/BLUE"
	# GRED="$NEWDATE/$RUN/good/RED"

	# BAD="$NEWDATE/$RUN/bad"
	# BRIGHT="$NEWDATE/$RUN/bad/RIGHT"
	# BLEFT="$NEWDATE/$RUN/bad/LEFT"
	# BGREEN="$NEWDATE/$RUN/bad/GREEN"
	# BBLUE="$NEWDATE/$RUN/bad/BLUE"
	# BRED="$NEWDATE/$RUN/bad/RED"
	#
	#
	#
	# mkdir $GOOD
	# mkdir $GLEFT
	# mkdir $GRIGHT
	# mkdir $GGREEN
	# mkdir $GBLUE
	# mkdir $GRED
	# mkdir $G_ORIG
	#
	# mkdir $BAD
	# mkdir $BLEFT
	# mkdir $BRIGHT
	# mkdir $BGREEN
	# mkdir $BBLUE
	# mkdir $BRED
	# mkdir $B_ORIG

# echo $NEWRUNDIR
# mkdir $NEWRUNDIR

# mkdir $GOOD
# mkdir $GLEFT
# mkdir $GRIGHT
# mkdir $GGREEN
# mkdir $GBLUE
# mkdir $GRED
# mkdir $G_ORIG

# mkdir $BAD
# mkdir $BLEFT
# mkdir $BRIGHT
# mkdir $BGREEN
# mkdir $BBLUE
# mkdir $BRED
# mkdir $B_ORIG

	# echo $GOOD
	# echo $GLEFT
	# echo $GRIGHT
	# echo $GGREEN
	# echo $GBLUE
	# echo $GRED
	# echo $G_ORIG

	# echo $BAD
	# echo $BLEFT
	# echo $BRIGHT
	# echo $BGREEN
	# echo $BBLUE
	# echo $BRED
	# echo $B_ORIG
