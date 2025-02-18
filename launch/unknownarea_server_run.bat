start python -m markerserver.swarmserver           @REM This prints out more things!! Better.
@REM start python markerserver/swarmserverclient.py

@REM start python -m markerserver.server                @REM OLD version without takeoff_simul
@REM start python markerserver/markerserverclient.py    @REM OLD version without takeoff_simul

timeout /t 3 /nobreak

start python markerserver/servergui.py

@REM start python -m UnknownArea_v2.main UnknownArea_v2.params
start python -m UnknownArea_v2.main shared_params.params11
start python -m UnknownArea_v2.main shared_params.params12
@REM start python -m UnknownArea_v2.main shared_params.params13