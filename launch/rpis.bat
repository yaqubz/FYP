@REM TBC how to connect with passwords and start calling commands directly (see Junyao's code)


@REM OPTION 1: Remember to connect to SCORPION_4 before running
start ssh telloswarm@192.168.0.111 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-3B4094' --ask"
@REM start ssh telloswarm@192.168.0.112 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-3B43AA'"


@REM OPTION 2: Remember to edit drones.json first when using RPIGUI
start python 0Diagnostics/connectrpiGUI.py