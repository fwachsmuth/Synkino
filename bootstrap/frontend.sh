#!/bin/bash

# Copied from "Burn Bootloader"

# writing Fuses (1st attempt)
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
-C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
-v -patmega328p \
-cstk500v2 \
-Pusb -e \
-Ulock:w:0x2F:m \
-Uefuse:w:0xFD:m \
-Uhfuse:w:0xD6:m \
-Ulfuse:w:0xF7:m 

# writing Optiboot
# /Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
# -C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
# -v -patmega328p \
# -cstk500v2 \
# -Pusb \
# -Uflash:w:/Users/peaceman/Library/Arduino15/packages/Optiboot/hardware/avr/0.6.2/bootloaders/optiboot/optiboot_atmega328.hex:i \
# -Ulock:w:0x0F:m

# Burn Frontend Code & bootloader
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
	-C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
	-v \
	-patmega328p \
	-cstk500v2 \
	-Pusb \
	-Uflash:w:/Users/peaceman/code/Arduino/Synkino/bootstrap/Synkino_Frontend_1.1.hex:i

# writing Fuses (2nd attempt)
# /Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
# -C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
# -v -patmega328p \
# -cstk500v2 \
# -Pusb -e \
# -Ulock:w:0x2F:m \
# -Uefuse:w:0xFD:m \
# -Uhfuse:w:0xD6:m \
# -Ulfuse:w:0xF7:m
#
# # writing Optiboot again
# /Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude \
# -C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/avrdude.conf \
# -v -patmega328p \
# -cstk500v2 \
# -Pusb \
# -Uflash:w:/Users/peaceman/Library/Arduino15/packages/Optiboot/hardware/avr/0.6.2/bootloaders/optiboot/optiboot_atmega328.hex:i \
# -Ulock:w:0x0F:m
