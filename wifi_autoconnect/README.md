---
```
       __     __  __  __      __
   /\ |__)   |  \|__)/  \|\ ||_ 
  /--\| \ .  |__/| \ \__/| \||__
  
              WPA/WPA2 support
        Modified from : https://github.com/daraosn/ardrone-wpa2
        Modified from : https://sites.google.com/site/androflight/arautoconnect
```
---

Install
==========
---

1. Connect your computer to your drone.
2. Configure settings for your router ('./install setup_config.sh')
3. Install wpa supplicant
  `cd ~/ros/src/metahast/wifi_autoconnect/install && . setup_config.sh && . install_wpa.sh `
4. Install autoconnect scripts
  `cd ~/ros/src/metahast/wifi_autoconnect/install && . install_autoconnect.sh `


Manually connect UAV to router
==========
---
Change drone connection from directly to a network with: ```script/connect "<essid>" -p "<password>" -a <address> -d <droneip>```

Run ```script/connect -h``` to get help.

Changing settings
=======
---
5. Change settings in `setup_config.sh`
	`cd ~/ros/src/metahast/wifi_autoconnect/install/ && . setup_config.sh && . upload_config.sh `

License
=======
---

ardrone-wpa2 @daraosn, MIT (see LICENSE)

wpa_supplicant (BSD licensed) Copyright (c) 2003-2013, Jouni Malinen <j@w1.fi> and contributors.

arautoconnect (GNU licensed)


