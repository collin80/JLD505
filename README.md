JLD505
=====

Battery Monitoring System and CHAdeMO Car-Side

This project runs on the JLD505 AVR based board and compiles with the Arduino
1.6.x IDE.

The following Arduino libraries, which are not distributed with the IDE, are required:
- mcp_can - Canbus library
- INA226 - Library for current / voltage sensing chip
- EEPROMAnything - Provides a user friendly way to store structures to EEPROM
- AltSoftSerial - Efficiently provides an extra serial port
- DS2480B - Interfaces with DS2480B serial to 1-wire chip
- DallasTemperature - Provides support for 1-wire temperature sensors
- FrequencyTimer2 - Provides support for steady timer interrupts on the AVR processor

All of these libraries are found in the repos for Collin80 on GitHub

All libraries belong in %USERPROFILE%\Documents\Arduino\libraries (Windows) or ~/Arduino/libraries (Linux/Mac).
You will need to remove -master or any other postfixes. Your library folders should be named as above.

The canbus is supposed to be terminated on both ends of the bus. The JLD505 board has termination resistors which
can be used by soldering the jumpers near the center of the board.






This software is MIT licensed:

Copyright (c) 2014 Collin Kidder, Paulo Alemeida, Jack Rickard

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

