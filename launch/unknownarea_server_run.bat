start python -m markerserver.server

@REM start python markerserver/markerserverclient.py THIS WORKS TOO

@REM Wait 3 seconds before starting the flight routine
timeout /t 3 /nobreak

start python markerserver/servergui.py

@REM start python -m UnknownArea_v2.main_copy
@REM start python -m UnknownArea_v2.main

start python -m UnknownArea_v2.main UnknownArea_v2.params
@REM start python -m UnknownArea_v2.main UnknownArea_v2.params11
@REM start python -m UnknownArea_v2.main UnknownArea_v2.params12