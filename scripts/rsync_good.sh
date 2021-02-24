echo "rsync /mnt/evo2/20200903/G /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/G /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/G"
EMAIL_BODY="$HOST finished rsync: 20200903/G"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

echo "rsync /mnt/evo2/20200903/I /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/I /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/I"
EMAIL_BODY="$HOST finished rsync: 20200903/I"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

echo "rsync /mnt/evo2/20200903/J /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/J /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/J"
EMAIL_BODY="$HOST finished rsync: 20200903/J"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

echo "rsync /mnt/evo2/20200903/A /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/A /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/A"
EMAIL_BODY="$HOST finished rsync: 20200903/A"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

echo "rsync /mnt/evo2/20200903/H /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/H /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/H"
EMAIL_BODY="$HOST finished rsync: 20200903/H"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

echo "rsync /mnt/evo2/20200903/B /mnt/archive/20200903  ..."
rsync -a --exclude-from="/home/benjamin/ros/src/metahast/scripts/exclude_list.txt" /mnt/evo2/20200903/B /mnt/archive/20200903
EMAIL_SUBJECT="$HOST finished rsync: 20200903/B"
EMAIL_BODY="$HOST finished rsync: 20200903/B"
echo "$EMAIL_BODY" | mail -s "$EMAIL_SUBJECT" gutter.puddles+rsync@gmail.com

