# SwarmServer User Guide
**Updated: Gabriel, 6 Mar**

## Overview
`swarmserverclient.py` is an all-in-one module containing both `MarkerServer` and `MarkerClient` classes.  
- A single `MarkerServer` must be running per network.  
- The server **must be started before** drone initialization to handle takeoff requests.  

🔹 **See `example.py` for a simple implementation using a real drone.**
🔹 **See `example_nodrone.py` for a more thorough implementation without a drone.**
🔹 **Ensure `swarmserverclient.py` is running before starting both scripts!**

## Installation & Usage

1. **Import `swarmserverclient.py`** and initialize `MarkerClient` in your code.
2. **Start the server**: Run `swarmserverclient.py` before takeoff.
3. **Trigger takeoff**:
   - Use `client_takeoff_simul(waiting_list)` to initiate takeoff once all drones in the list are ready.
   - If manual triggering is needed, use `waiting_list = [99]`.
   - You can also add a status message (usually for battery level) to display while drone is waiting to takeoff. E.g.`client_takeoff_simul(waiting_list, 'drone.get_battery()')`

## Important Notes
⚠️ **Landing function limitation**  
- The **"Land all drones"** function requires additional implementation in your code.  
- It sends a `land_signal`, which must be processed by your main loop or a separate thread to trigger `drone.end()`.  

