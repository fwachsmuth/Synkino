#!/bin/bash

# Fuses: e:FD h:D6 l:FF
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
	-C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf  \
	-v -patmega328p \
	-cstk500v2 \
	-Pusb \
	-e \
	-Ulock:w:0x3F:m \
	-Uefuse:w:0xFD:m -Uhfuse:w:0xD6:m -Ulfuse:w:0xFF:m
#avrdude -Cavrdude.conf -v -patmega328p -cstk500v2 -Pusb -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xD6:m -Ulfuse:w:0xFF:m


# Bootloader Optimboot:
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude  
	-C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
	-v -patmega328p \
	-cstk500v2 \
	-Pusb \
	-Uflash:w:/Users/peaceman/Library/Arduino15/packages/Optiboot/hardware/avr/0.6.2/bootloaders/optiboot/optiboot_atmega328.hex:i \
	-Ulock:w:0x0F:m
#avrdude -Cavrdude.conf -v -patmega328p -cstk500v2 -Pusb -Uflash:w:optiboot_atmega328.hex:i -Ulock:w:0x0F:m

# Burn Frontend Code
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
	-C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
	-v \
	-patmega328p \
	-cstk500v2 \
	-Pusb \
	-Uflash:w:Synkino_Frontend_1.0.hex:i 
