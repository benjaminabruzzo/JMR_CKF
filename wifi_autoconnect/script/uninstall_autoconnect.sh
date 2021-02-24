#!/usr/bin/env bash
set -ue

DRONEIP=${1:-"192.168.1.1"}

echo "Removing binaries..."
{( sleep 1; echo "
	INSTALL_PATH=/home/default/ARAutoConnect
	if [ -s /bin/wifi_setup.sh.bak ] ; then
		mv /bin/wifi_setup.sh.bak /bin/wifi_setup.sh
		# removing ARAutoConnect
	    rm -r $INSTALL_PATH
	fi
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "autoconnect uninstalled."




