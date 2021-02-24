# update ARAutoConnect settings
DRONEIP=${1:-"192.168.1.1"}
curl -T ../config/ip "ftp://$DRONEIP"
curl -T ../config/essid "ftp://$DRONEIP"
curl -T ../config/netmask "ftp://$DRONEIP"
curl -T ../config/password "ftp://$DRONEIP"
curl -T ../config/wpa_supplicant.conf "ftp://$DRONEIP"

sleep 1
{( sleep 1; echo "
	mv /data/video/ip /home/default/ARAutoConnect && rm /data/video/ip
	mv /data/video/essid /home/default/ARAutoConnect && rm /data/video/essid
	mv /data/video/netmask /home/default/ARAutoConnect && rm /data/video/netmask
	mv /data/video/password /home/default/ARAutoConnect && rm /data/video/password
	mv /data/video/wpa_supplicant.conf /etc && rm /data/video/wpa_supplicant.conf
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "ARAutoConnect settings updated."