#!/bin/sh

DRONEIP=${1:-"192.168.1.1"}
# DRONEIP=${1:-"192.168.64.25"}
curl -T ../config/ip "ftp://$DRONEIP"
curl -T ../config/essid "ftp://$DRONEIP"
curl -T ../config/netmask "ftp://$DRONEIP"
curl -T ../config/password "ftp://$DRONEIP"
curl -T ../config/wpa_supplicant.conf "ftp://$DRONEIP"

curl -T ../config/ARAutoConnect.sh "ftp://$DRONEIP"
curl -T ../config/enableonboot.sh "ftp://$DRONEIP"

sleep 1

{( sleep 1; echo "
	chmod +x /data/video/enableonboot.sh 
	. /data/video/enableonboot.sh 
	rm /data/video/enableonboot.sh 
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "ARAutoConnect.sh installed."


# mv /data/video/enableonboot.sh /home/default/ARAutoConnect && rm /data/video/enableonboot.sh