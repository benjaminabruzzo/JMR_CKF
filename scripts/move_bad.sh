# cd ~/ros/src/metahast && bash ./scripts/move_bad.sh

cd ~/ros/src/metahast && python ./scripts/move_bad.py
source ~/ros/src/metahast/scripts/rsync_good.sh

# ~~~~~~~~~~~ make sure the correct blocks of trials are commented out
								source ~/ros/src/metahast/scripts/rsync_bad.sh
								cd ~/ros/src/metahast && bash ./scripts/rmrf.sh
# ~~~~~~~~~~~ make sure the correct blocks of trials are commented out
