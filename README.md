Bootstrap loader for MDR32F9Q2I, К1986ВЕ92QI, 1986ВЕ9x microcontrollers.
It uses factory boot loader burned in MCU boot sector to load next stage
loader 1986_BOOT_UART.hex into RAM, which, in turn, can load custom
firmware, burn it into EEPROM and run.

Usage:
-s 115200
    set upload baud rate
-d /dev/ttyUSB0
    set serial device
-b 1986_BOOT_UART.hex
    set stage 2 loader hex image
-f
    set user custom firmware hex image

Example:
./mdrbootstrap -s 115200 -d /dev/ttyUSB0 -b 1986_BOOT_UART.hex -f 1986BE9x_Demo.hex
