## TM cougar TQS USB adapter
This is a DIY USB adapter to make your TM Cougar throttle work as a standalone USB device.

**This software is based on the LUFA framework and distributed under MIT license as it stated in the [LUFA license](http://www.fourwalledcubicle.com/files/LUFA/Doc/120730/html/_page__license_info.html), Original license is provided here un modified in the file "license.txt".**

**Hardware is also distributed under MIT license, as specified under the "LICENSE_HW.md" file in this repo.**

**It is highly recommended that you flash or update the device with TQS connected. Without the throttle connected, there are a lot of floating connections which may result in weird behavior.**


http://pits.108vfs.org

#### Other useful stuff
##### Hardware
My version is based on Atmel ATMEGA 32u4 chip.
I've been using a replica of Sparkfun's Arduino Pro-Micro, with other versions YMMV.
Schematics can be found in the "*hardware*" folder.
for more details on the build process:
http://pits.108vfs.org/uriba/standalone-cougar-tqs-part-i/

In addition, you can buy a complete product based on the base design from ViperCore a well know store in the Viper pit builder community.
http://www.vipercore.nl/index.php/webshop/view/productdetails/virtuemart_product_id/57/virtuemart_category_id/14

##### Software
Source code provided can compiled and opened using Atmel Studio 7.
for more details check out the relevent blog posts:
http://pits.108vfs.org/uriba/standalone-cougar-tqs-part-ii/
http://pits.108vfs.org/uriba/standalone-cougar-tqs-part-iii/


##### Installation
Connect the Arduino Pro-micro to the computer, Drivers may be needed.
press the reset button twice in rapid succession to force device into "bootloader" mode.
Your computer will now Identify a new device, please note this device's COM port.
from the AVRdude folder, run the "*upload_code.bat*" file. In the window opened, type the device's com port number (eg. if device is on COM12, type "12").
double press the reset button of the arduino to force it into bootloaded mode again, and hit Enter. The code will now be uploaded to device. once completed, the device should register as a new joystick.

##### User Interface
The TQS code allows the user some calibration options via the use of "config mode".

To enter "Config Mode" press and hold Uncage <u>and</u> Speedbreaks open for 2 seconds.

Once enabled Config Mode will remain enabled for 20 seconds or until you select an options.

Available options:

| Button | Action |
|:------:|:------:|
| UHF    | Exit Config Mode immidiatly (and save) |
| VHF    | center Microstick (and exit config mode) |
| IFF IN | Save Current position of roteries as center (Detent calibration)|
| IFF OUT | drop all current Min/Max values for microstick and throttle. <br> Then, while in config mode, update Min/Max Values (20 seconds or UHF is pushed) |

an addition special mode is available to boot TQS controller to bootloader to allow device firmware to be updated.
to enter boot loader press and hold the following buttons for 5 seconds: Uncage, Cursor Enable, Speedbreaks Open.
##### Debugging
In the "*TQS_debugging*" folder, you will find an Arduino sketch to help you debug the hardware.
it will sample the device in exactly the same way the actual code does, but outputs the data out via the Serial console.
compile using "Arduino Micro" in the Arduino IDE.

###### Fun Facts
resolution of the device:

| Axis | Resolution |
|:-----|:----------:|
|Throttle|12bits|
|Microstick|8 bits|
|ANT|10bits|
|RNG|10bits|

The code in action:<br>
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/_ej5wT5lJjc/0.jpg)](https://www.youtube.com/watch?v=_ej5wT5lJjc)
