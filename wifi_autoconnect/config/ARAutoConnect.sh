#!/bin/sh

# This file is part of ARAutoConnect.
#
# ARAutoConnect is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ARAutoConnect is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ARAutoConnect.  If not, see <http://www.gnu.org/licenses/>.


INSTALL_PATH=/home/default/ARAutoConnect
ESSID_PATH=$INSTALL_PATH/essid
IP_PATH=$INSTALL_PATH/ip
NETMASK_PATH=$INSTALL_PATH/netmask
PASSWORD_PATH=$INSTALL_PATH/password

ESSID=rezo
IP=192.168.1.1
NETMASK=255.255.0.0

BASE_ADDRESS=192.168.1.
LAST_NUMBER="2 3 4 5"
CONNECTED=0
COUNT=0

#retrieving ad-hoc ssid to restore the network
DRONESSID=`grep ssid_single_player /data/config.ini | awk -F "=" '{print $2}'`

# Removing leading and trailing spaces
DRONESSID=`echo $DRONESSID`

if [ -n "$DRONESSID" ]
then
        echo "found drone ssid =$DRONESSID"
else
        #default SSID.
        DRONESSID=ardrone_wifi
fi

#retrieving managed mode configuration
if [ -s $ESSID_PATH ] ; then
        ESSID=`cat $ESSID_PATH`
fi

if [ -s $IP_PATH ] ; then
        IP=`cat $IP_PATH`
fi

if [ -s $NETMASK_PATH ] ; then
        NETMASK=`cat $NETMASK_PATH`
fi

# PASSWORD="stevens2018"
if [ -s $PASSWORD_PATH ] ; then
        PASSWORD=`cat $PASSWORD_PATH`
fi

HEXLOG=$INSTALL_PATH/randhex
tr -dc 'A-F0-9' < /dev/urandom | dd bs=1 count=8>$HEXLOG
# HEXSUFFIX=$(cat $HEXLOG)
# LOG=$INSTALL_PATH/log_$HEXSUFFIX
LOG=$INSTALL_PATH/log
DHCPC=""

echo "  --- " >> $LOG
echo "Network configuration" > $LOG
echo "Drone SSID                : $DRONESSID" >> $LOG
echo "Access Point ESSID        : $ESSID" >> $LOG
echo "Access Point PSK          : $PASSWORD" >> $LOG
echo "Infrastructure IP address : $IP" >> $LOG
# echo "Infrastructure Netmask    : $NETMASK" >>$LOG
echo "Random log ID             : $HEXSUFFIX" >> $LOG
echo "  --- " >> $LOG


MASTERn=0
MASTERLIMIT=6 # 6*5 == 30 second wait before connecting to router
ADHOCn=1


while [ 1 ]
do
    
    CONFIG=`iwconfig ath0`
    sleep 5

    if echo $CONFIG | grep -q "Master" ;
    then
        echo "In Master mode: n = $MASTERn" >> $LOG
        # MASTERn=$(($MASTERn+1))
        MASTERn=`expr "$MASTERn" + 1`
        CONNECTED=0
        # are we connected to one of the standard ad-hoc IPs 
        for i in $LAST_NUMBER
        do
            if ping -W 1 -c 1 -q $BASE_ADDRESS$i ; then
                CONNECTED=1
                break
            fi
        done

        if [ "$MASTERn" -ge "$MASTERLIMIT" ] && [ "$CONNECTED" -eq 0 ] ; then
            echo "  **$MASTERn >= $MASTERLIMIT" >> $LOG
            echo "  connecting to $ESSID ..." >> $LOG   
            echo "{ifconfig ath0 $IP; iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf $DHCPC; }" >> $LOG

            sleep 1
            MASTERn=0
            ifconfig ath0 $IP
            iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf $DHCPC

            continue
        fi

        # if we are, there is nothing else to do, waiting 10s before checking again 
        if [ "$CONNECTED" -eq 1 ] ; then
            echo "  connected to $BASE_ADDRESS$i" >> $LOG
            MASTERn=0
            sleep 5
            continue
        fi

        if [ "$CONNECTED" -eq 0 ] ; then
            echo "  not connected ..." >> $LOG
            sleep 5
            continue
        fi
    fi


    if echo $CONFIG | grep -q "Managed" ;
    then
        echo "$CONFIG" >> $LOG
        sleep 5
    fi


done
