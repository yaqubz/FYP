@echo off
@REM set IP_LIST=192.168.0.131 192.168.0.132 192.168.0.133
@REM set IP_LIST=192.168.0.123 192.168.0.137 192.168.0.138
set IP_LIST=192.168.0.131 192.168.0.132 192.168.0.133 192.168.0.123 192.168.0.137 192.168.0.138
@REM set IP_LIST=192.168.0.122 192.168.0.128 192.168.0.129 192.168.0.130

for %%i in (%IP_LIST%) do (
    ping -n 1 %%i | find "Reply from %%i" >nul
    if errorlevel 1 (
        echo %%i is DOWN
    ) else (
        echo %%i is UP
    )
)
pause