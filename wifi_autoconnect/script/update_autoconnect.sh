# update Autocorrect script directly
DRONEIP=${1:-"192.168.1.1"}
curl -T ../config/ARAutoConnect.sh "ftp://$DRONEIP"

sleep 1

{( sleep 1; echo "
	mv /data/video/ARAutoConnect.sh /home/default/ARAutoConnect && rm /data/video/ARAutoConnect.sh
	chmod +x /home/default/ARAutoConnect/ARAutoConnect.sh
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "ARAutoConnect.sh Updated"