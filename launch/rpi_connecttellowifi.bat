@REM For connecting drone to RPi WiFi. Remember to connect PC to same network as RPi before running

start ssh -t telloswarm@192.168.0.111 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-3B256C' password 'telloswarm' --ask; bash"