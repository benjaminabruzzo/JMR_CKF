#!/bin/bashTime
# . copy_cal_data.sh argo --include

DST_HOST=$1
INCLUDE=$2

if [ $INCLUDE = "--include" ]; then
  rsync -avI -e ssh ~/ros/data/calibrations/ benjamin@$1.local:~/ros/data/calibrations/
else
  rsync -avI --exclude='*.png' -e ssh ~/ros/data/calibrations/ benjamin@$1.local:~/ros/data/calibrations/
fi
