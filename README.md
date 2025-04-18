#FYP - README

**Last Updated:** 9 Jan 2025

## Overview

SIMPLIFIED GITHUB FOR UPLOADING IN FYP REPORT. NOT RELATED/IMPORTANT REFERS TO CODE THAT IS NOT DIRECTLY LINKED TO OBSTACLE AVOIDANCE. 

## Quick Start Guide

To start the system from the main directory, simply run the batch file `main_run_sequence.bat` located in the `.\launch` directory. This will automatically run the processes in sequence:

1. **Run PPGUI**: Opens the GUI to create and save the waypoint JSON file.
2. **Run `.exe`**: Prompts you to connect to the UWB console by selecting the correct COM port. If connected successfully, it will run in the background.
3. **Run UWB_Viz**: Displays the map and the positions of the UWB tags.
4. **Run PPFLY**: Executes the saved waypoint JSON for flight control.

**Note**: This package should work out-of-the-box for Windows users, as long as the required `.exe` file is available and Windows-based dependencies are met.

---

## Platform-Specific Instructions

### Windows Users:
- See **Annex A** for setup instructions
- The process will run as expected by following the quick start guide above.
- Ensure you have a Windows PC to run the `.exe` file, as it is not compatible with MacOS.

### Mac Users:
- If you are using a Mac, you will need an additional Windows PC connected to the UWB console to run the `.exe` file.
- Once both the Windows and Mac PCs are connected to the same network, the Mac can run the rest of the package as usual and access the `.exe`'s published information via UDP.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
