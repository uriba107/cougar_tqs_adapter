@echo off
set hexpath="../firmware"
set basename="cougar_tqs"
set version=002
set vid="LUFA_VID"
REM set vid="TM_VID"

set /p comport="Enter COM port number of device: "
avrdude.exe -patmega32u4 -cavr109 -b57600 -D -U flash:w:%hexpath%/%basename%-%vid%_%version%.hex:i -PCOM%comport%
pause
