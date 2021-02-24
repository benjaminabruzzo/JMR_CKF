import re

IDX=1;
A_TRIALS="1   3           9          13                     ";
B_TRIALS="    3     6 7     10                         19 20";
G_TRIALS="  ";
H_TRIALS="            7        11                      19 20";
I_TRIALS="      4 5         10             15               ";
J_TRIALS="            7     10 11       14 15    17 18 19 20";


CODES = ["A", "B", "G", "H", "I", "J"]
# map_idx = 1
# for code in CODES:
# 	new_key = "{}_src".format(code)
# 	X = eval("{}_TRIALS".format(code))
# 	X = re.sub(' +', ' ', X.strip());
# 	if bool(X):
# 		X = X.split(' ')
# 		print(X)
# 		for trial in X:
# 			TRIAL = ("{} + {}").format(100*map_idx, trial)
# 			print(TRIAL)
# 			print(eval(TRIAL))
# 			BAD_DICT["{}_TRIALS".format(code)].append(eval(TRIAL))

	# bad_src[new_key] = eval(new_data)

DATE = 20200821
RSYNC_GOOD = '/home/benjamin/ros/src/metahast/scripts/rsync_test.sh'; rsync_good = open(RSYNC_GOOD, 'w')

for key in CODES:
	# rsync_good.write(('rsync -aI --exclude-from="{0}" /mnt/evo2/{2}/{1} /mnt/archive/{2}\n').format(EXCLUDE_FILENAME, key[0], DATE))
	rsync_good.write(('EMAIL_SUBJECT="$HOST finished rsync: {0}/{1}"\n').format(DATE, key[0]))
	rsync_good.write(('EMAIL_BODY="$HOST finished rsync: {0}/{1}"\n').format(DATE, key[0]))
	rsync_good.write('echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com\n\n')





#
# infile = open("input.txt", "r")
#
# lines = infile.readlines()
#
# create_inline_script = 0
# script_str = ""
# start_delim = "//START_PY"
# end_delim = "//END_PY"
#
# for line in lines:
#     if create_inline_script == 1:
#         if (re.search(end_delim,line)):
#             exec (script_str)
#             create_inline_script = 0
#             script_str = ""
#         else:
#             script_str += line
#     elif (re.search(start_delim,line)):
#         create_inline_script = 1
#     else:
#         print (line)
#
# infile.close()
