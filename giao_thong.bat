@echo off
setlocal
cd /d "%~dp0"
if not exist build\TrafficSim.exe call build.bat
if errorlevel 1 exit /b 1
build\TrafficSim.exe --traffic-demo
