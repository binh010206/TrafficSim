@echo off
cd /d "%~dp0"
if not exist build\TrafficSim.exe call build.bat
if errorlevel 1 exit /b 1
build\TrafficSim.exe --model-test
if errorlevel 1 (pause & exit /b 1)
build\TrafficSim.exe --self-test
if errorlevel 1 (pause & exit /b 1)
build\TrafficSim.exe --smoke
pause
