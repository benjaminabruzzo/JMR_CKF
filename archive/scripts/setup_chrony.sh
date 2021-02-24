sudo apt-get update && sudo apt-get install -y ntpdate chrony
sudo subl /etc/chrony/chrony.conf

# for time server machine, add these lines:
	# make it serve time even if it is not synced (as it can't reach out)
	local stratum 8
	# allow the IP of your peer to connect
	# allow <IP of your time-clinet>
	# allow ALL clients
	allow

# for time client machines, add these lines:
	server <server_ip> minpoll 0 maxpoll 5 maxdelay .05


# check with
	ntpdate -q <IP-of-Server>
	
