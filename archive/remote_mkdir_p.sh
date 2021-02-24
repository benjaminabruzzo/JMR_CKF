#!/bin/bash

# cd ~/ros/src/metahast/ && bash ./scripts/remote_mkdir_p.sh atlas /mnt/evo2/${DATE}/${CODE}/figs
# cd ~/ros/src/metahast/ && bash ./scripts/remote_mkdir_p.sh atlas /mnt/evo2/20200801/L/figs

# ssh USER@HOST 'COMMAND1; COMMAND2; COMMAND3'
# ssh USER@HOST << EOF
# COMMAND1
# COMMAND2
# COMMAND3
# EOF
# ssh USER@HOST 'bash -s' < SCRIPT
# ssh root@192.168.1.1 'bash -s' < script.sh
# ssh root@192.168.1.1 'bash -s' < script.sh

unset REMOTE; REMOTE=$1
unset PATH; PATH=$2

mkdirp="mkdir -p /mnt/evo2/${DATE}/${CODE}/figs"

ssh -X benjamin@$1.local PATH=\$HOME/bin:\$PATH\; $2
