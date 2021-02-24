#!/bin/bash
DRONEIP=${1:-"192.168.1.1"}

echo "Uploading binaries..."
curl -T ../bin/wpa_cli "ftp://$DRONEIP"
curl -T ../bin/wpa_passphrase "ftp://$DRONEIP"
curl -T ../bin/wpa_supplicant "ftp://$DRONEIP"
curl -T ../config/wpa_supplicant.conf "ftp://$DRONEIP"
sleep 1

{( sleep 1; echo "
	mv /data/video/wpa_supplicant.conf /etc
    mv /data/video/wpa_* /bin
	chmod +x /bin/wpa_*
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "wpa_supplicant installed."