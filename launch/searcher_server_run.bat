start python -m markerserver.swarmserver           @REM This prints out more things!! Better.
@REM start python markerserver/swarmserverclient.py

@REM start python -m markerserver.server                @REM OLD version without takeoff_simul
@REM start python markerserver/markerserverclient.py    @REM OLD version without takeoff_simul

timeout /t 3 /nobreak

start python markerserver/servergui.py

start python -m Search.Searcher_halfbaked19feb shared_params.params
@REM start python -m Search.Searcher_halfbaked19feb shared_params.params11
@REM start python -m Search.Searcher_halfbaked19feb shared_params.params12