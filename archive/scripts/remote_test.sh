#!/bin/bash
# remote call
# cd ~/ros/src/metahast/scripts && . remote_test.sh atlas
# cd ~/ros/src/metahast/scripts && . remote_test.sh hermes
# cd ~/ros/src/metahast/scripts && . remote_test.sh pompei
# cd ~/ros/src/metahast/scripts && . remote_test.sh titan

# ssh -t username@host 'top'

unset REMOTE; REMOTE=$1
scp /home/benjamin/ros/src/metahast/scripts/test_scripts.sh benjamin@${REMOTE}.local:~/ros/src/metahast/scripts/test_scripts.sh
ssh benjamin@$REMOTE.local REMOTE=$REMOTE 'bash -s' <<"ENDSSH"
nohup source ~/ros/src/metahast/scripts/test_scripts.sh ${REMOTE} &> ~/ros/nohup_${REMOTE}_trials.out &
echo $! > ~/ros/${REMOTE}_trials_pid.txt
ENDSSH
