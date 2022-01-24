#!/bin/sh

# copy me to the linux board ;)

echo "Checking ext. rt mcu stm32h7 firmware version..."
openocd -f /usr/arduino/extra/openocd_script.cfg -c "flash verify_image /usr/arduino/extra/STM32H747AII6_CM7.bin 0x8000000" -c "reset" -c "exit"
res=$?

if [ $res != 0 ]; then
    echo "Ext. rt mcu stm32h7 is not programmed/updated, starting programming procedure..."
    openocd -f /usr/arduino/extra/openocd_script.cfg -c "program /usr/arduino/extra/STM32H747AII6_CM7.bin verify reset exit 0x8000000"
else
    echo "Ext. rt mcu stm32h7 is programmed/updated, exiting..."
fi
exit 0
