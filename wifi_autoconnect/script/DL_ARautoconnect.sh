# update Autocorrect script directly
rm ~/ros/src/metahast/wifi_autoconnect/script/ARAutoConnect.sh
rm ~/ros/src/metahast/wifi_autoconnect/script/ip
rm ~/ros/src/metahast/wifi_autoconnect/script/netmask
rm ~/ros/src/metahast/wifi_autoconnect/script/sandbox.sh
rm ~/ros/src/metahast/wifi_autoconnect/script/wifi_setup.sh.bak
rm ~/ros/src/metahast/wifi_autoconnect/script/essid
rm ~/ros/src/metahast/wifi_autoconnect/script/log
rm ~/ros/src/metahast/wifi_autoconnect/script/randhex
rm ~/ros/src/metahast/wifi_autoconnect/script/uninstall.sh

DRONEIP=${1:-"192.168.1.1"}

{( sleep 1; echo "
	cp /home/default/ARAutoConnect/ARAutoConnect.sh /data/video/ARAutoConnect.sh
	cp /home/default/ARAutoConnect/ip /data/video/ip
	cp /home/default/ARAutoConnect/netmask /data/video/netmask
	cp /home/default/ARAutoConnect/sandbox.sh /data/video/sandbox.sh
	cp /home/default/ARAutoConnect/wifi_setup.sh.bak /data/video/wifi_setup.sh.bak
	cp /home/default/ARAutoConnect/essid /data/video/essid
	cp /home/default/ARAutoConnect/log /data/video/log
	cp /home/default/ARAutoConnect/randhex /data/video/randhex
	cp /home/default/ARAutoConnect/uninstall.sh /data/video/uninstall.sh
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "Moved AutoConnect.sh and sandbox.sh to script folder"

sleep 1
curl "ftp://$DRONEIP/ARAutoConnect.sh" > ARAutoConnect.sh
curl "ftp://$DRONEIP/ip" > ip
curl "ftp://$DRONEIP/netmask" > netmask
curl "ftp://$DRONEIP/sandbox.sh" > sandbox.sh
curl "ftp://$DRONEIP/wifi_setup.sh.bak" > wifi_setup.sh.bak
curl "ftp://$DRONEIP/essid" > essid
curl "ftp://$DRONEIP/log" > log
curl "ftp://$DRONEIP/randhex" > randhex
curl "ftp://$DRONEIP/uninstall.sh" > uninstall.sh
sleep 1


{( sleep 1; echo "
	rm /data/video/ARAutoConnect.sh
	rm /data/video/ip
	rm /data/video/netmask
	rm /data/video/sandbox.sh
	rm /data/video/wifi_setup.sh.bak
	rm /data/video/essid
	rm /data/video/log
	rm /data/video/randhex
	rm /data/video/uninstall.sh
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "cleaned up remote folder"