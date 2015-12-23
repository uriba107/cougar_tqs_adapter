@echo off
set hexpath="../firmware"
set basename="cougar_tqs"
set version=003
set cougar="old"
rem set cougar="new"
set vid="LUFA_VID"
rem set vid="TM_VID"

set /p comport="Enter COM port number of device: "
avrdude.exe -patmega32u4 -cavr109 -b57600 -D -U flash:w:%hexpath%/%basename%_%cougar%-%vid%_%version%.hex:i -PCOM%comport%
pause
