@echo off
setlocal
cd /d "%~dp0"
if not exist build mkdir build
g++ -std=c++17 -O2 -Wall -Wextra -Isrc -I../glad/include -I../glfw-3.5.1.bin.WIN64/glfw-3.5.1.bin.WIN64/include src/main.cpp src/Simulation.cpp src/Geometry.cpp src/BikeModel.cpp src/Renderer.cpp src/Font8x8.cpp -L../glfw-3.5.1.bin.WIN64/glfw-3.5.1.bin.WIN64/lib-mingw-w64 -lglfw3 -lopengl32 -lgdi32 -static -static-libgcc -static-libstdc++ -o build/TrafficSim.exe
if errorlevel 1 exit /b 1

