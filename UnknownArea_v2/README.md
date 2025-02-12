# Unknown Search Area (aka. USA) - README

**Last Updated:** 11 Feb 2025

## Overview

This package, along with `markerserver`, allows multiple drones to autonomously search an unknown area and land on specific ArUco markers, while ensuring that no two drones target the same marker,

## Quick Start Guide

To start the system from the main directory, simply run the batch file `unknownarea_server_run.bat` located in the `.\launch` directory. This will automatically run the processes in sequence:

1. **Run `python -m markerserver.server`**: Initiates server. (NOTE: Server must be manually restarted between each run)
2. **Run `python markerserver/servergui.py`** Launches Server Monitor GUI. 
3. **Run `python -m UnknownArea_v2.main`**: Connects to drone (or laptop, if in LAPTOP_ONLY mode). Can run multiple nodes via RPi.
>>> This module also works in **NO_FLY** and **LAPTOP_ONLY** modes for easy/remote debugging respectively.

_Note: Steps 2 and 3 can be run on different computers from the server, as long as on the same network._

## Troubleshooting

- **'cv2.aruco' has no attribute 'detectMarkers'**: The default `opencv-python` module that is installed with `djitellopy` does not include ArUco. Instead, run:
```powershell
pip uninstall opencv-python opencv-contrib-python
pip install opencv-contrib-python
```

## To-Dos
To Test 10 Feb
- running two USA drones on the same computer ✔
- running multiple clients on different computers, connected to the same network
- clear detected = True if not updated for 10 seconds

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
