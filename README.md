# SunsparkESP32
A new ESP32 hardware platform for low power applications.

This PCB utilizes the a small solar cell (e.g. 1.5V 0.65W) in order to harvest energy and store it in a supercapacitor. The open circuit voltage should not exceed 3.3V.

## Features
1. ESP32 S2 Mini module with a single core MCU. Most projects utilize one core anyway or don't benefit much form a second one. So no current is wasted by idling unnecessary.
2. Native USB support, converter for RS232 can be connected externally. Many USB to RS232 transceivers have a high quiescent consumption. They are mostly needed for debugging, in order to show data to a console. Flashing the chip also works via the USB bootloader (and is also faster).
3. Integrated low side switch to control external peripherals and turn them off when not needed
4. Recharging the supercap can also be done via VBUS from USB
5. Optional RF95/96/97/98 LoRa module can be added
6. TBD

## Current consumption estimation
First testing with V1 board and posting data to a node red UDP server every 10 minutes via WiFi. Static IP config.
Values for estimating discharging of capacitor:
0.5s on-time, with average 30mA
~10min sleep, with 90µA current
->115µA average operating current. TODO: Measure charging

