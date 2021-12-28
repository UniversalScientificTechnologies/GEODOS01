#!/bin/bash
./avrdude -C./avrdude.conf -v -patmega1284p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:./AIRDOSC_1024_LS.ino.hex:i 


