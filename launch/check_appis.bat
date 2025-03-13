@echo off
@REM set IP_LIST=192.168.0.111 192.168.0.112 192.168.0.113
set IP_LIST=192.168.0.103 192.168.0.117 192.168.0.118 192.168.0.111 192.168.0.112 192.168.0.113

for %%i in (%IP_LIST%) do (
    ping -n 1 %%i | find "Reply from %%i" >nul
    if errorlevel 1 (
        echo %%i is DOWN
    ) else (
        echo %%i is UP
    )
)
pause