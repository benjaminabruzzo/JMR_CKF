#!/bin/bashTime
# . ~/ros/src/metahast/scripts/pull_to_ares.sh 20200122 -x
# . ~/ros/src/metahast/scripts/pull_to_ares.sh 20200122 --include

DATE=$1
INCLUDE=$2

if [ $INCLUDE = "--include" ]; then
  rsync -avI -e ssh ~/ros/data/$DATE/ benjamin@ares.local:~/ros/data/$DATE/
else
  rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@ares.local:~/ros/data/$DATE/
fi


# rsync -avI --exclude='*.png' -e ssh benjamin@mk3.local:~/ros/data/20200122/ ~/ros/data/20200122/
# rsync -avI --exclude='*.png' -e ssh ~/ros/data/20200124/ benjamin@ares.local:~/ros/data/20200124/
