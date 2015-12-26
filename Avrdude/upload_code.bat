@echo off
set hexpath="../firmware"
set basename="cougar_tqs"
set version=003
set cougar=""
rem set cougar="_alternate"
set vid=""
rem set vid="_LUFA_VID"


set /p comport="Enter COM port number of device: "
avrdude.exe -patmega32u4 -cavr109 -b57600 -D -U flash:w:%hexpath%/%basename%%cougar%-%version%%vid%.hex:i -PCOM%comport%
pause
