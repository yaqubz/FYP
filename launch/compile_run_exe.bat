@REM This script should be run from the main workspace, NOT inside the launch directory
@REM All build directory artifacts can be deleted! But keep the empty \build directory itself.

cd "nlink_unpack-master\build"

cmake ..
cmake --build . --config Release

.\Release\nlink_unpack_COMx_udp.exe

@REM To return to main workspace
cd ../..