	# adding a line to /bin/wifi_setup.sh to launch the script
	# creating the target directory
	INSTALL_PATH=/home/default/ARAutoConnect
	mkdir -p $INSTALL_PATH
	SCRIPT_NAME=ARAutoConnect.sh

	sleep 1

	mv /data/video/ip /home/default/ARAutoConnect && rm /data/video/ip
	mv /data/video/essid /home/default/ARAutoConnect && rm /data/video/essid
	mv /data/video/netmask /home/default/ARAutoConnect && rm /data/video/netmask
	mv /data/video/password /home/default/ARAutoConnect && rm /data/video/password
	mv /data/video/wpa_supplicant.conf /home/default/ARAutoConnect && rm /data/video/wpa_supplicant.conf

	mv /data/video/ARAutoConnect.sh /home/default/ARAutoConnect && rm /data/video/ARAutoConnect.sh
	chmod +x /home/default/ARAutoConnect/ARAutoConnect.sh


	if [ -s $INSTALL_PATH/$SCRIPT_NAME ] ; then
	    # checking if  wifi_setup.sh has already been modified
	    if cat /bin/wifi_setup.sh | grep -q $SCRIPT_NAME ; then
	          echo "Wifi_setup.sh has already been modified. "
	    else
	          cp /bin/wifi_setup.sh /bin/wifi_setup.sh.bak
	          cp /bin/wifi_setup.sh $INSTALL_PATH/wifi_setup.sh.bak
	          echo "$INSTALL_PATH/$SCRIPT_NAME &" >> /bin/wifi_setup.sh
	    fi
	fi

	# syncing filesytem to be sure that our modifications will survive a reboot
	sync