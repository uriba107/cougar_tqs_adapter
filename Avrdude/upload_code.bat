@echo off
set /p comport="Enter COM port number of device: "
avrdude.exe -patmega32u4 -cavr109 -b57600 -D -U flash:w:../firmware/cougar_tqs-LUFA_VID.hex:i -PCOM%comport%
REM avrdude.exe -patmega32u4 -cavr109 -b57600 -D -U flash:w:../firmware/cougar_tqs_TM_VID.hex:i -PCOM%comport%
pause
