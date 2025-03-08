@REM This script should be run from the main workspace, NOT inside the launch directory
@REM All build directory artifacts can be deleted! But keep the empty \build directory itself.

cd "nlink_unpack-master\build"

cmake ..
cmake --build . --config Release

@REM .\Release\nlink_unpack_COMx_udp.exe
.\Release\nlink_unpack_COMx_udp2.exe      @REM ---- CHANGE .exe FILENAME HERE!!

@REM To return to main workspace
cd ../..