# update Autocorrect script directly
rm ~/ros/src/metahast/wifi_autoconnect/script/log
DRONEIP=${1:-"192.168.1.1"}

{( sleep 1; echo "
	cp /home/default/ARAutoConnect/log /data/video/log
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "Moved AutoConnect Log file"

sleep 1
curl "ftp://$DRONEIP/log" > log
sleep 1

{( sleep 1; echo "
	rm /data/video/log 
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "Moved AutoConnect Log file"
