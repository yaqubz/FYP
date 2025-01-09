# UWB Tag Positioning System - README

**Last Updated:** 9 Jan 2025

## Overview

This package provides a solution for waypoint generation, UWB tag positioning, and flight control. The process is fully automated through a single batch file, ensuring easy startup. The system is compatible with both Windows and Mac operating systems, but certain steps are specific to each OS. Please follow the instructions based on your setup.

## Quick Start Guide

To start the system from the main directory, simply run the batch file `main_run_sequence.bat` located in the `.\launch` directory. This will automatically run the processes in sequence:

1. **Start the system**: Initialize the processes.
2. **PPGUI**: Opens the GUI to create and save the waypoint JSON file.
3. **Run `.exe`**: Prompts you to connect to the UWB console by selecting the correct COM port.
4. **Start UWB_Viz**: Displays the map and the positions of the UWB tags.
5. **Run PPFLY**: Executes the saved waypoint JSON for flight control.

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

---

## Annex A: UWB Console Setup on Windows

Follow these steps to set up the UWB console on a Windows PC:

1. **Install Serial Drivers**:
   - Download the serial drivers for Nooploop from [Nooploop Download Page](https://www.nooploop.com/download/), under "Development Support".
   - Download the `CP210X_Windows` drivers for Windows.
   
2. **Unzip the Folder**:
   - Extract the downloaded ZIP file to a known location on your Windows PC.

3. **Install the Drivers**:
   - Open **Device Manager** on your Windows PC.
   - Locate the connected UWB device under **Ports (COM & LPT)**.
   - Right-click the device and select **Update Driver**.
   - Choose **Browse my computer for driver software** and navigate to the location where you extracted the driver files.
   - Select the correct driver and complete the installation.

4. **Find the COM Port Number**:
   - In **Device Manager**, locate the connected UWB device under **Ports (COM & LPT)**.
   - Take note of the COM Port number assigned to the device (e.g., COM11).
   - This COM port number will be required when prompted by the `.exe` during the connection process.

---

## Troubleshooting

- **Error: COM Port not found**: Ensure that the UWB device is properly connected and that the correct COM port is selected in the `.exe` prompt.
- **Connection issues**: Verify that both the Mac and Windows PCs are connected to the same network and that the `.exe` is running on the Windows PC.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
