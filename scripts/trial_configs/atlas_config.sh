# cd ~/ros/src/metahast && bash ./scripts/local_trials.sh
# tail -f ~/ros/data/20200903/${HOST}_tee.log
# rsync -av --exclude='*.png' -e ssh ~/mnt/evo2/20200901/J /mnt/archive/20200901
# rsync -av --exclude='*.png' -e ssh ~/ros/data/20200903/ benjamin@atlas.local:/mnt/evo2/20200903/

unset IDX DATE EXP_CODES LIST

unset A_TRIALS B_TRIALS
unset C_TRIALS D_TRIALS E_TRIALS F_TRIALS
unset G_TRIALS H_TRIALS I_TRIALS J_TRIALS

unset MAP SHOW_RVIS SEND_TEXT SEND_EMAILS

# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
	MNT="evo2"
	# MNT="archive"

	# DATE="20200901";	# using DKF to initialize ugv2
	# DATE="20200902";	# both ugv1 & ugv2 start with perfect initialization
	DATE="20200903";	# DKF with ugv2 looking at UGV1 for init

	# EXP_CODES=(A B G H I J);	# Trial strategies
	EXP_CODES=(G);					# Trial strategies
	# EXP_CODES=(A B G);					# Trial strategies

	# IDX=5;
	# # LIST="1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
	# # LIST="1 2 3 4 5";
	# # LIST="6 7 8 9 10";
	# # LIST="11 12 13 14 15";
	# # LIST="16 17 18 19 20";
	LIST=" 6";
	A_TRIALS=$LIST;
	B_TRIALS=$LIST;
	G_TRIALS=$LIST;
	H_TRIALS=$LIST;
	I_TRIALS=$LIST;
	J_TRIALS=$LIST;

	IDX=4;
	# A_TRIALS="              8   10       13          17 18      ";
	# B_TRIALS="  2         7     10    12                18 19   ";
	# G_TRIALS="                9       12       15 16 17    19 20";
	# H_TRIALS="            7                                   20";
	# I_TRIALS="                                                  ";
	# J_TRIALS="    3   5                     14    16    18      ";


	MAP=tags_0$IDX;
	SHOW_RVIS="true";
	SEND_TEXT="false";
	SEND_EMAILS="false";
	PROCESS_MATLAB="false";
