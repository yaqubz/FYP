start python swarmserver/swarmserverclient.py

timeout /t 1 /nobreak

@REM start python -m UnknownArea_v2.main shared_params.params0

@REM start python -m UnknownArea_v2.main shared_params.params11ap
start python -m UnknownArea_v2.main shared_params.params12ap
@REM start python -m UnknownArea_v2.main shared_params.params13ap


@REM @REM FOR TESTING CLIENTS REMOTELY - set LAPTOP_ONLY = True
@REM start python -m UnknownArea_v2.main shared_params.params
@REM start python -m UnknownArea_v2.main shared_params.params3