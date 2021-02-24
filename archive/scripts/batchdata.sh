DATE=$1
START=$2
STOP=$3


matlab -nodisplay -nosplash -nodesktop -r "batchslam_1604('$DATE', $START, $STOP);exit;"


# cd ~/ros/src/metahast/scripts && 	. batchdata.sh 20200428 102 105
