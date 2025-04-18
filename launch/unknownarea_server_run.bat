@REM Actual batch script to run for competition
@REM Launches server, then launches 3 instances of Unknown Area drones with their respective params files.

start python swarmserver/swarmserverclient.py

timeout /t 1 /nobreak

start python -m UnknownArea_v2.main shared_params.params11ap
start python -m UnknownArea_v2.main shared_params.params12ap
start python -m UnknownArea_v2.main shared_params.params13ap

@REM @REM FOR TESTING DIRECT CONNECTION (no longer works with AP drones)
@REM start python -m UnknownArea_v2.main shared_params.params0

@REM @REM FOR TESTING CLIENTS REMOTELY - set LAPTOP_ONLY = True
@REM start python -m UnknownArea_v2.main shared_params.params
@REM start python -m UnknownArea_v2.main shared_params.params17
