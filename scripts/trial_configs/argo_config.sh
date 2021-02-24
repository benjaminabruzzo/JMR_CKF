# cd ~/ros/src/metahast && bash ./scripts/local_trials.sh

unset IDX DATE EXP_CODES LIST

unset A_TRIALS B_TRIALS
unset C_TRIALS D_TRIALS E_TRIALS F_TRIALS
unset G_TRIALS H_TRIALS I_TRIALS J_TRIALS

unset MAP SHOW_RVIS SEND_TEXT SEND_EMAILS

# ~~~~~~~~~~~~~~~~~~  Trial specific params ~~~~~~~~~~~~~~~~~~~~~~~~ #
	IDX=2; 										# map number
	DATE="20200821";					# date folder

	EXP_CODES=(A B G H I J);	# Trial strategies
	# EXP_CODES=(I );					# Trial strategies

# Trial numbers
	LIST="1 2 3 4 5";
	# LIST="6 7 8 9 10";
	# LIST="11 12 13 14 15";
	# LIST="16 17 18 19 20";

	# A_TRIALS=" 17 18 19"
	# B_TRIALS=" 16 18 19 20"
	# G_TRIALS=" 16 17 18 20"
	# H_TRIALS=" 16 17 18 19 20"
	# I_TRIALS=" 16 17 18 19"
	# J_TRIALS=" 16 17 18 19 20"

	A_TRIALS=$LIST;
	B_TRIALS=$LIST;
	G_TRIALS=$LIST;
	H_TRIALS=$LIST;
	I_TRIALS=$LIST;
	J_TRIALS=$LIST;

	MAP=tags_0$IDX;
	SHOW_RVIS="false";
	SEND_TEXT="false";
	SEND_EMAILS="true";
