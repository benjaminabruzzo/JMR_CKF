#!/bin/bash
# linksys e2500
# IP=192.168.0.25
# ESSID=ARDRONE250024ghz
# NETMASK=255.255.0.0
# PASSWORD=abruzzo2018
# asus wifi6
IP=192.168.10.125
ESSID=hast2020
NETMASK=255.255.0.0
PASSWORD=stevens2020


cd ../config 
touch ip && rm ip && echo "$IP" >> ip
touch essid && rm essid && echo "$ESSID" >> essid
touch netmask && rm netmask && echo "$NETMASK" >> netmask
touch password && rm password && echo "$PASSWORD" >> password
touch wpa_supplicant.conf && rm wpa_supplicant.conf
	echo "network={" >> wpa_supplicant.conf
	echo '    ssid="'$ESSID'"' >> wpa_supplicant.conf
	echo '    psk="'$PASSWORD'"' >> wpa_supplicant.conf
	echo "}" >> wpa_supplicant.conf

cd ../install