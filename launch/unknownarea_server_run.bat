start python -m markerserver.server

@REM Wait 3 seconds before starting the flight routine
timeout /t 3 /nobreak

start python markerserver/servergui.py

python -m UnknownArea_v2.main