#!/bin/bashTime
# . pull_from.sh ares 20200226

SRC=$1
DATE=$2


rsync -avI --exclude='*.mat' -e ssh benjamin@$SRC.local:~/ros/data/$DATE/ ~/ros/data/$DATE/
