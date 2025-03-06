# Unknown Search Area (aka. USA) - README

**Last Updated:** 24 Feb 2025

## Overview

This package runs a drone to perform a set of known waypoints, before turning on monocular depth-mapping and reading ToF sensor data to search an unknown area for ArUco markers. Deployed along with `swarmserver` on multiple drones will ensure that no two drones target the same marker.

## Quick Start Guide

To start the system from the main directory, simply run the batch file `unknownarea_server_run.bat` located in the `.\launch` directory. This will automatically run the processes in sequence:

1. **Run `python swarmserver/swarmserverclient.py`**: Initiates server and launches Server Monitor GUI. Closing the GUI also closes the server. 
3. **Run `python -m UnknownArea_v2.main shared_params.params`**: Connects to drone (or laptop, if in LAPTOP_ONLY mode). Modify configuration files under `shared_params`. Can run multiple nodes via RPi.
>>> This module also works in **NO_FLY** and **LAPTOP_ONLY** modes for easy/remote debugging respectively.

_Note: Step 2 can be run on different computers from the server from Step 1, as long as devices are on the same network._

## Troubleshooting

- **'cv2.aruco' has no attribute 'detectMarkers'**: The default `opencv-python` module that is installed with `djitellopy` does not include ArUco. Instead, run:
```powershell
pip uninstall opencv-python opencv-contrib-python
pip install opencv-contrib-python
```

## To-Dos
- ✔ running two USA drones on the same computer 
- ✔ running multiple clients on different computers, connected to the same network
- ✔ clear detected = True if not updated for 10 seconds 
- Fix approach overshoot

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
