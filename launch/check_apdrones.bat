@echo off
set IP_LIST=192.168.0.131 192.168.0.132 192.168.0.133

for %%i in (%IP_LIST%) do (
    ping -n 1 %%i | find "Reply from %%i" >nul
    if errorlevel 1 (
        echo %%i is DOWN
    ) else (
        echo %%i is UP
    )
)
pause