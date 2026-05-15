@echo off
setlocal

set "SKETCH_DIR=%~dp0."
set "CLI=C:\Users\sidth\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "FQBN=rp2040:rp2040:seeed_xiao_rp2350"

if not exist "%CLI%" set "CLI=arduino-cli"

"%CLI%" compile --fqbn "%FQBN%" "%SKETCH_DIR%"
