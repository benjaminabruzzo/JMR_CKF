#!/usr/bin/env python
# cd ~/ros/src/metahast && bash ./scripts/move_bad.sh
# cd ~/ros/src/metahast && bash ./scripts/move_bad.sh && rm /mnt/evo2/20200903/batch_data.mat

import shutil # for moving directories
import distutils

import os # for os.makedirs('my_folder/another/folder', exists_ok=True)
import re # for removing extra spaces

def makedir_p(path):
	if not os.path.exists(path):
		os.makedirs(path)

def remove_values_from_list(the_list, val):
   return [value for value in the_list if value != val]

def update_BAD_TRIALS(BAD_DICT, map_idx, A, B, G, H, I, J):
	CODES = ["A", "B", "G", "H", "I", "J"]

	for code in CODES:
		new_key = "{}_src".format(code)
		X = eval("{}_TRIALS".format(code))
		X = re.sub(' +', ' ', X.strip());
		if bool(X):
			X = X.split(' ')
			for trial in X:
				TRIAL = ("{} + {}").format(100*map_idx, trial)
				BAD_DICT["{}_TRIALS".format(code)].append(eval(TRIAL))

	return BAD_DICT

BAD_TRIALS = {}
TRIAL_NAMES = ["A_TRIALS", "B_TRIALS", "G_TRIALS", "H_TRIALS", "I_TRIALS", "J_TRIALS"]
print(TRIAL_NAMES)
for NAME in TRIAL_NAMES:
	BAD_TRIALS[NAME] = []

EXEMPT_MAPS = [1,2,3,4,5,6]

DATE="20200903"

# # #X_TRIALS="1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
# # # # # # # # # # # # # # ## --------------------------------------
# IDX=1;
# A_TRIALS="1                                15               ";
# B_TRIALS="    3           9    11 12             17         ";
# G_TRIALS="  2   4         9                15             20";
# H_TRIALS="  2 3   5     8                                   ";
# I_TRIALS="  2 3     6                13 14                  ";
# J_TRIALS="        5 6 7 8   10    12       15          19   ";
# # # #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
# BAD_TRIALS = update_BAD_TRIALS(BAD_TRIALS, IDX, A_TRIALS, B_TRIALS, G_TRIALS, H_TRIALS, I_TRIALS, J_TRIALS)
# EXEMPT_MAPS = remove_values_from_list(EXEMPT_MAPS, IDX)
# # # # # # # # # # ## --------------------------------------
# IDX=3;
# A_TRIALS="                                 15               ";
# B_TRIALS="  2                                               ";
# G_TRIALS="                              14                  ";
# H_TRIALS="          6          11                           ";
# I_TRIALS="                        12                        ";
# J_TRIALS="                        12                        ";
# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
# EXEMPT_MAPS = remove_values_from_list(EXEMPT_MAPS, IDX)
# BAD_TRIALS = update_BAD_TRIALS(BAD_TRIALS, IDX, A_TRIALS, B_TRIALS, G_TRIALS, H_TRIALS, I_TRIALS, J_TRIALS)
# # # # # # # # # # # # ## --------------------------------------
# IDX=4;
# A_TRIALS="              8   10       13          17 18      ";
# B_TRIALS="  2         7     10    12                18 19   ";
# G_TRIALS="                9       12       15 16 17    19 20";
# H_TRIALS="            7                                   20";
# I_TRIALS="                                                  ";
# J_TRIALS="    3   5                     14    16    18      ";
# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
# EXEMPT_MAPS = remove_values_from_list(EXEMPT_MAPS, IDX)
# BAD_TRIALS = update_BAD_TRIALS(BAD_TRIALS, IDX, A_TRIALS, B_TRIALS, G_TRIALS, H_TRIALS, I_TRIALS, J_TRIALS)
# # # # # # # # # # # # # ## --------------------------------------
# IDX=6;
# A_TRIALS="1 2       6       10                         19   ";
# B_TRIALS="          6       10 11    13                     ";
# G_TRIALS="          6   8      11 12                        ";
# H_TRIALS="  2       6                      15          19 20";
# I_TRIALS="          6                         16 17 18      ";
# J_TRIALS="  2 3 4     7                             18      ";
# #X_TRIALS"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20";
# EXEMPT_MAPS = remove_values_from_list(EXEMPT_MAPS, IDX)
# BAD_TRIALS = update_BAD_TRIALS(BAD_TRIALS, IDX, A_TRIALS, B_TRIALS, G_TRIALS, H_TRIALS, I_TRIALS, J_TRIALS)

print("Maps to explude from rsync: {}".format(EXEMPT_MAPS))
EXCLUDE_FILENAME = '/home/benjamin/ros/src/metahast/scripts/exclude_list.txt'; exclude_logger = open(EXCLUDE_FILENAME, 'w')
for MAP in EXEMPT_MAPS:
	# print("- {}*").format(MAP)
	exclude_logger.write(("- {}*\n").format(MAP))

RM_FILE = '/home/benjamin/ros/src/metahast/scripts/rmrf.sh'; rmrf_logger = open(RM_FILE, 'w')
RSYNC_BAD  = '/home/benjamin/ros/src/metahast/scripts/rsync_bad.sh';  rsync_bad = open(RSYNC_BAD, 'w')
RSYNC_GOOD = '/home/benjamin/ros/src/metahast/scripts/rsync_good.sh'; rsync_good = open(RSYNC_GOOD, 'w')

for key in BAD_TRIALS:
	print("BAD_TRIALS[{}] = {}").format(key, BAD_TRIALS[key])

	for trial in BAD_TRIALS[key]:
		original = "/mnt/evo2/{0}/{1}/{2}".format(DATE, key[0], trial)
		target = "/mnt/evo2/bad/{0}/{1}".format(DATE, key[0])

		makedir_p(target)
		print("rsync -a --delete-after {} {}".format(original, target))
		rsync_bad.write(('echo "rsync -a --delete-after {} {}  ..."\n').format(original, target))
		rsync_bad.write(('rsync -a --delete-after {} {}\n').format(original, target))
		rmrf_logger.write(('rm -rf {}\n').format(original))

	print(('rsync -a --exclude-from="{0}" /mnt/evo2/{2}/{1} /mnt/archive/{2}\n').format(EXCLUDE_FILENAME, key[0], DATE))
	rsync_good.write(('echo "rsync /mnt/evo2/{1}/{0} /mnt/archive/{1}  ..."\n').format(key[0], DATE))
	rsync_good.write(('rsync -a --exclude-from="{0}" /mnt/evo2/{2}/{1} /mnt/archive/{2}\n').format(EXCLUDE_FILENAME, key[0], DATE))
	rsync_good.write(('EMAIL_SUBJECT="$HOST finished rsync: {0}/{1}"\n').format(DATE, key[0]))
	rsync_good.write(('EMAIL_BODY="$HOST finished rsync: {0}/{1}"\n').format(DATE, key[0]))
	rsync_good.write('echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com\n\n')
